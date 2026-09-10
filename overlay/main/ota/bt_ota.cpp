
/*

 * ============================================================

 * BDK V2.5 - DEH-P650 WRD - BLUETOOTH CLASSIC SPP OTA

 * ============================================================

 *

 * Formato .dehota (64 bytes de cabecalho):

 *

 *   offset  size  descricao

 *   0       8     "DEHOTA1\0"

 *   8       4     format_version = 1 (little endian)

 *   12      4     firmware_size (little endian)

 *   16      32    SHA-256 do firmware .bin

 *   48      16    modelo ASCII "DEH-P650-WRD" + zero padding

 *   64      N     firmware .bin bruto

 *

 * Integridade:

 *   - modelo

 *   - tamanho

 *   - SHA-256

 *   - esp_ota_end() valida a imagem ESP

 *

 * IMPORTANTE:

 *   Esta V1 protege contra arquivo corrompido/errado, mas NAO implementa

 *   assinatura criptografica de fabricante. O modo OTA exige entrada fisica

 *   pelo botao BOOT e usa SPP autenticado.

 * ============================================================

 */



#include "bt_ota.h"



#include <stddef.h>

#include <stdint.h>

#include <string.h>

#include <stdio.h>



#include "freertos/FreeRTOS.h"

#include "freertos/task.h"

#include "freertos/queue.h"



#include "esp_err.h"

#include "esp_log.h"

#include "esp_system.h"

#include "esp_ota_ops.h"

#include "esp_partition.h"



#include "nvs.h"



#include "driver/gpio.h"



#include "esp_bt.h"

#include "esp_bt_main.h"

#include "esp_bt_device.h"

#include "esp_gap_bt_api.h"

#include "esp_spp_api.h"



#include "mbedtls/sha256.h"



static const char *TAG = "BDK_BT_OTA";



// ------------------------------------------------------------

// OTA mode flag

// ------------------------------------------------------------

static constexpr const char *NVS_NS = "dehota";

static constexpr const char *NVS_KEY_MODE = "mode";



// ------------------------------------------------------------

// .dehota format

// ------------------------------------------------------------

static constexpr size_t HEADER_SIZE = 64;

static constexpr uint8_t MAGIC[8] = {

    'D','E','H','O','T','A','1','\0'

};

static constexpr uint32_t FORMAT_VERSION = 1;

static constexpr char MODEL_NAME[] = "DEH-P650-WRD";



// ------------------------------------------------------------

// SPP RX queue

// ------------------------------------------------------------

struct RxChunk {

    uint16_t len;

    uint8_t data[ESP_SPP_MAX_MTU];

};



static QueueHandle_t g_rxQueue = nullptr;

static TaskHandle_t g_workerTask = nullptr;



static volatile uint32_t g_sppHandle = 0;

static volatile bool g_sppConnected = false;

static volatile bool g_sppCongested = false;

static volatile bool g_sppWritePending = false;

static volatile bool g_rxOverflow = false;



// ------------------------------------------------------------

// Transfer state

// ------------------------------------------------------------

enum class TransferState : uint8_t {

    WAIT_HEADER = 0,

    WRITING,

    COMPLETE,

    ERROR

};



static TransferState g_state = TransferState::WAIT_HEADER;

static uint8_t g_header[HEADER_SIZE];

static size_t g_headerUsed = 0;



static uint32_t g_expectedSize = 0;

static uint32_t g_receivedSize = 0;

static uint8_t g_expectedSha[32];



static const esp_partition_t *g_updatePartition = nullptr;

static esp_ota_handle_t g_otaHandle = 0;

static bool g_otaHandleValid = false;



static mbedtls_sha256_context g_shaCtx;

static bool g_shaActive = false;



static uint32_t g_nextProgress = 10;



// ------------------------------------------------------------

// Helpers

// ------------------------------------------------------------

static uint32_t readLe32(const uint8_t *p)

{

    return ((uint32_t)p[0]) |

           ((uint32_t)p[1] << 8) |

           ((uint32_t)p[2] << 16) |

           ((uint32_t)p[3] << 24);

}



static void shaToHex(const uint8_t sha[32], char out[65])

{

    static const char hex[] = "0123456789abcdef";

    for (int i = 0; i < 32; ++i) {

        out[i * 2]     = hex[(sha[i] >> 4) & 0x0F];

        out[i * 2 + 1] = hex[sha[i] & 0x0F];

    }

    out[64] = '\0';

}



static void sppSendText(const char *text)

{

    if (!text || !g_sppConnected || g_sppHandle == 0) return;

    if (g_sppCongested || g_sppWritePending) return;



    const size_t n = strlen(text);

    if (n == 0 || n > 512) return;



    // esp_spp_write() nao copia necessariamente o buffer para sempre.

    // Usamos apenas literais/strings estaticas ou buffer da task que permanece

    // valido ate o retorno. Mensagens sao pequenas e pouco frequentes.

    esp_err_t err = esp_spp_write(

        g_sppHandle,

        (int)n,

        (uint8_t *)text);



    if (err == ESP_OK) {

        g_sppWritePending = true;

    }

}



static void abortTransfer(const char *reason)

{

    if (g_otaHandleValid) {

        esp_ota_abort(g_otaHandle);

        g_otaHandleValid = false;

    }



    if (g_shaActive) {

        mbedtls_sha256_free(&g_shaCtx);

        g_shaActive = false;

    }



    g_state = TransferState::ERROR;



    ESP_LOGE(TAG, "OTA ABORTADO: %s", reason ? reason : "erro desconhecido");

    sppSendText("\r\nOTA ERROR - transferencia cancelada\r\n");

}



static void resetTransfer()

{

    if (g_otaHandleValid) {

        esp_ota_abort(g_otaHandle);

    }

    g_otaHandleValid = false;



    if (g_shaActive) {

        mbedtls_sha256_free(&g_shaCtx);

    }

    g_shaActive = false;



    memset(g_header, 0, sizeof(g_header));

    g_headerUsed = 0;

    g_expectedSize = 0;

    g_receivedSize = 0;

    memset(g_expectedSha, 0, sizeof(g_expectedSha));

    g_updatePartition = nullptr;

    g_nextProgress = 10;

    g_rxOverflow = false;

    g_state = TransferState::WAIT_HEADER;

}



static bool validateAndBeginHeader()

{

    if (memcmp(g_header, MAGIC, sizeof(MAGIC)) != 0) {

        abortTransfer("magic DEHOTA invalida");

        return false;

    }



    const uint32_t version = readLe32(&g_header[8]);

    if (version != FORMAT_VERSION) {

        abortTransfer("versao do pacote DEHOTA nao suportada");

        return false;

    }



    g_expectedSize = readLe32(&g_header[12]);

    if (g_expectedSize < 4096) {

        abortTransfer("firmware pequeno demais");

        return false;

    }



    memcpy(g_expectedSha, &g_header[16], 32);



    char model[17] = {};

    memcpy(model, &g_header[48], 16);



    if (strncmp(model, MODEL_NAME, strlen(MODEL_NAME)) != 0) {

        ESP_LOGE(TAG, "Modelo recebido: '%s'", model);

        abortTransfer("pacote nao pertence ao DEH-P650 WRD");

        return false;

    }



    g_updatePartition = esp_ota_get_next_update_partition(nullptr);

    if (!g_updatePartition) {

        abortTransfer("nenhuma particao OTA disponivel");

        return false;

    }



    ESP_LOGI(TAG,

             "Destino OTA: label=%s addr=0x%08lx size=%lu",

             g_updatePartition->label,

             (unsigned long)g_updatePartition->address,

             (unsigned long)g_updatePartition->size);



    if (g_expectedSize > g_updatePartition->size) {

        ESP_LOGE(TAG,

                 "Firmware=%lu / particao=%lu",

                 (unsigned long)g_expectedSize,

                 (unsigned long)g_updatePartition->size);

        abortTransfer("firmware maior que a particao OTA");

        return false;

    }



    esp_err_t err = esp_ota_begin(

        g_updatePartition,

        g_expectedSize,

        &g_otaHandle);



    if (err != ESP_OK) {

        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));

        abortTransfer("esp_ota_begin falhou");

        return false;

    }



    g_otaHandleValid = true;



    mbedtls_sha256_init(&g_shaCtx);

    if (mbedtls_sha256_starts(&g_shaCtx, 0) != 0) {

        abortTransfer("SHA256 init falhou");

        return false;

    }

    g_shaActive = true;



    char shaHex[65];

    shaToHex(g_expectedSha, shaHex);



    ESP_LOGI(TAG, "Cabecalho DEHOTA OK");

    ESP_LOGI(TAG, "Modelo    : %s", model);

    ESP_LOGI(TAG, "Tamanho   : %lu bytes", (unsigned long)g_expectedSize);

    ESP_LOGI(TAG, "SHA256 exp: %s", shaHex);

    ESP_LOGI(TAG, "Recebendo firmware...");



    sppSendText("\r\nDEHOTA HEADER OK - recebendo firmware\r\n");

    g_state = TransferState::WRITING;

    return true;

}



static bool finishTransfer()

{

    if (!g_otaHandleValid || !g_shaActive) {

        abortTransfer("estado interno invalido no final");

        return false;

    }



    uint8_t receivedSha[32];

    if (mbedtls_sha256_finish(&g_shaCtx, receivedSha) != 0) {

        abortTransfer("SHA256 final falhou");

        return false;

    }



    mbedtls_sha256_free(&g_shaCtx);

    g_shaActive = false;



    char gotHex[65];

    char expHex[65];

    shaToHex(receivedSha, gotHex);

    shaToHex(g_expectedSha, expHex);



    ESP_LOGI(TAG, "SHA256 got: %s", gotHex);

    ESP_LOGI(TAG, "SHA256 exp: %s", expHex);



    if (memcmp(receivedSha, g_expectedSha, 32) != 0) {

        abortTransfer("SHA256 nao confere");

        return false;

    }



    esp_err_t err = esp_ota_end(g_otaHandle);

    g_otaHandleValid = false;



    if (err != ESP_OK) {

        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));

        abortTransfer("imagem ESP invalida");

        return false;

    }



    err = esp_ota_set_boot_partition(g_updatePartition);

    if (err != ESP_OK) {

        ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));

        abortTransfer("nao foi possivel selecionar novo boot");

        return false;

    }



    g_state = TransferState::COMPLETE;



    ESP_LOGI(TAG, "");

    ESP_LOGI(TAG, "==============================================");

    ESP_LOGI(TAG, " OTA BLUETOOTH CONCLUIDO COM SUCESSO");

    ESP_LOGI(TAG, " %lu bytes gravados", (unsigned long)g_receivedSize);

    ESP_LOGI(TAG, " Reiniciando no firmware novo...");

    ESP_LOGI(TAG, "==============================================");



    sppSendText(

        "\r\n"

        "OTA 100% OK\r\n"

        "SHA256 OK\r\n"

        "IMAGEM ESP OK\r\n"

        "REINICIANDO...\r\n");



    vTaskDelay(pdMS_TO_TICKS(1800));

    esp_restart();

    return true;

}



static bool consumeFirmwareBytes(const uint8_t *data, size_t len)

{

    if (g_state != TransferState::WRITING) return false;

    if (!data || len == 0) return true;



    const uint32_t remaining = g_expectedSize - g_receivedSize;

    if (len > remaining) {

        abortTransfer("arquivo contem bytes extras apos o firmware");

        return false;

    }



    esp_err_t err = esp_ota_write(g_otaHandle, data, len);

    if (err != ESP_OK) {

        ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));

        abortTransfer("falha gravando flash");

        return false;

    }



    if (mbedtls_sha256_update(&g_shaCtx, data, len) != 0) {

        abortTransfer("SHA256 update falhou");

        return false;

    }



    g_receivedSize += (uint32_t)len;



    if (g_expectedSize > 0) {

        const uint32_t pct =

            (uint32_t)(((uint64_t)g_receivedSize * 100ULL) / g_expectedSize);



        if (pct >= g_nextProgress && g_nextProgress <= 90) {

            char msg[96];

            snprintf(msg, sizeof(msg),

                     "\r\nOTA %lu%% - %lu/%lu bytes\r\n",

                     (unsigned long)pct,

                     (unsigned long)g_receivedSize,

                     (unsigned long)g_expectedSize);



            ESP_LOGI(TAG,

                     "Progresso: %lu%% (%lu/%lu)",

                     (unsigned long)pct,

                     (unsigned long)g_receivedSize,

                     (unsigned long)g_expectedSize);



            sppSendText(msg);



            while (g_nextProgress <= pct) {

                g_nextProgress += 10;

            }

        }

    }



    if (g_receivedSize == g_expectedSize) {

        return finishTransfer();

    }



    return true;

}



static void consumeStream(const uint8_t *data, size_t len)

{

    if (!data || len == 0) return;

    if (g_state == TransferState::ERROR ||

        g_state == TransferState::COMPLETE) {

        return;

    }



    size_t pos = 0;



    if (g_state == TransferState::WAIT_HEADER) {

        const size_t need = HEADER_SIZE - g_headerUsed;

        const size_t take = (len < need) ? len : need;



        memcpy(&g_header[g_headerUsed], data, take);

        g_headerUsed += take;

        pos += take;



        if (g_headerUsed == HEADER_SIZE) {

            if (!validateAndBeginHeader()) return;

        }

    }



    if (pos < len && g_state == TransferState::WRITING) {

        consumeFirmwareBytes(&data[pos], len - pos);

    }

}



static void otaWorker(void *)

{

    RxChunk chunk;



    ESP_LOGI(TAG, "OTA worker pronto");



    while (true) {

        if (g_rxOverflow) {

            g_rxOverflow = false;

            abortTransfer("fila RX cheia; envie o arquivo novamente");

        }



        if (xQueueReceive(g_rxQueue, &chunk, pdMS_TO_TICKS(250)) == pdTRUE) {

            if (g_state == TransferState::ERROR) {

                // Nova conexao/arquivo pode reiniciar o parser quando o usuario

                // reconectar. Enquanto isso descartamos bytes.

                continue;

            }



            consumeStream(chunk.data, chunk.len);

        }

    }

}



// ------------------------------------------------------------

// GAP / SPP callbacks

// ------------------------------------------------------------

static void gapCallback(

    esp_bt_gap_cb_event_t event,

    esp_bt_gap_cb_param_t *param)

{

    if (!param) return;



    switch (event) {

        case ESP_BT_GAP_PIN_REQ_EVT: {

            esp_bt_pin_code_t pin_code = {'6','5','0','0'};

            ESP_LOGI(TAG, "PIN solicitado -> 6500");

            esp_bt_gap_pin_reply(

                param->pin_req.bda,

                true,

                4,

                pin_code);

            break;

        }



#if defined(CONFIG_BT_SSP_ENABLED) && CONFIG_BT_SSP_ENABLED

        case ESP_BT_GAP_CFM_REQ_EVT:

            ESP_LOGI(TAG,

                     "SSP confirm value: %lu",

                     (unsigned long)param->cfm_req.num_val);

            esp_bt_gap_ssp_confirm_reply(

                param->cfm_req.bda,

                true);

            break;

#endif



        case ESP_BT_GAP_AUTH_CMPL_EVT:

            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {

                ESP_LOGI(TAG, "Bluetooth autenticado: %s",

                         param->auth_cmpl.device_name);

            } else {

                ESP_LOGW(TAG, "Falha autenticacao BT: %d",

                         (int)param->auth_cmpl.stat);

            }

            break;



        default:

            break;

    }

}



static void sppCallback(

    esp_spp_cb_event_t event,

    esp_spp_cb_param_t *param)

{

    if (!param) return;



    switch (event) {

        case ESP_SPP_INIT_EVT:

            ESP_LOGI(TAG, "SPP init OK; iniciando servidor...");

            esp_spp_start_srv(

                ESP_SPP_SEC_AUTHENTICATE,

                ESP_SPP_ROLE_SLAVE,

                0,

                "DEH_OTA");

            break;



        case ESP_SPP_START_EVT:

            ESP_LOGI(TAG, "SPP server pronto / SCN=%d",

                     (int)param->start.scn);

            esp_bt_gap_set_scan_mode(

                ESP_BT_CONNECTABLE,

                ESP_BT_GENERAL_DISCOVERABLE);

            break;



        case ESP_SPP_SRV_OPEN_EVT:

            g_sppHandle = param->srv_open.handle;

            g_sppConnected = true;

            g_sppCongested = false;

            g_sppWritePending = false;

            resetTransfer();



            ESP_LOGI(TAG,

                     "Celular conectado ao OTA SPP / handle=%lu",

                     (unsigned long)g_sppHandle);



            sppSendText(

                "\r\n"

                "DEH-P650 WRD OTA V1\r\n"

                "Envie um arquivo .dehota completo.\r\n");

            break;



        case ESP_SPP_CLOSE_EVT:

            ESP_LOGW(TAG, "SPP desconectado");



            if (g_state == TransferState::WRITING) {

                abortTransfer("Bluetooth desconectou durante OTA");

            }



            g_sppConnected = false;

            g_sppHandle = 0;

            g_sppCongested = false;

            g_sppWritePending = false;

            break;



        case ESP_SPP_DATA_IND_EVT: {

            if (!g_rxQueue ||

                !param->data_ind.data ||

                param->data_ind.len == 0) {

                break;

            }



            if (param->data_ind.len > ESP_SPP_MAX_MTU) {

                g_rxOverflow = true;

                break;

            }



            RxChunk chunk;

            chunk.len = param->data_ind.len;

            memcpy(

                chunk.data,

                param->data_ind.data,

                param->data_ind.len);



            if (xQueueSend(g_rxQueue, &chunk, 0) != pdTRUE) {

                g_rxOverflow = true;

            }

            break;

        }



        case ESP_SPP_WRITE_EVT:

            g_sppWritePending = false;

            g_sppCongested = param->write.cong;

            break;



        case ESP_SPP_CONG_EVT:

            g_sppCongested = param->cong.cong;

            break;



        default:

            break;

    }

}



// ------------------------------------------------------------

// NVS mode flag

// ------------------------------------------------------------

bool btOtaRequestOnNextBoot()

{

    nvs_handle_t h;

    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);

    if (err != ESP_OK) {

        ESP_LOGE(TAG, "nvs_open mode: %s", esp_err_to_name(err));

        return false;

    }



    err = nvs_set_u8(h, NVS_KEY_MODE, 1);

    if (err == ESP_OK) {

        err = nvs_commit(h);

    }

    nvs_close(h);



    if (err != ESP_OK) {

        ESP_LOGE(TAG, "nvs_set/commit mode: %s", esp_err_to_name(err));

        return false;

    }



    ESP_LOGI(TAG, "Flag OTA gravada no NVS");

    return true;

}



bool btOtaShouldEnterAndConsume()

{

    nvs_handle_t h;

    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);



    if (err == ESP_ERR_NVS_NOT_FOUND) {

        return false;

    }

    if (err != ESP_OK) {

        ESP_LOGW(TAG, "NVS OTA indisponivel: %s", esp_err_to_name(err));

        return false;

    }



    uint8_t mode = 0;

    err = nvs_get_u8(h, NVS_KEY_MODE, &mode);



    if (err == ESP_OK && mode == 1) {

        nvs_erase_key(h, NVS_KEY_MODE);

        nvs_commit(h);

        nvs_close(h);



        ESP_LOGW(TAG, "Flag OTA consumida -> entrando em SPP OTA");

        return true;

    }



    nvs_close(h);

    return false;

}



// ------------------------------------------------------------

// Bluetooth-only OTA server

// ------------------------------------------------------------

void btOtaRunServerBlocking(const char *device_name)

{

    if (!device_name || !device_name[0]) {

        device_name = "DEH-P650 WRD OTA";

    }



    ESP_LOGW(TAG, "");

    ESP_LOGW(TAG, "==================================================");

    ESP_LOGW(TAG, " DEH-P650 WRD - MODO BLUETOOTH OTA");

    ESP_LOGW(TAG, " A2DP / LDAC / DSP / I2S / IP-BUS NAO INICIADOS");

    ESP_LOGW(TAG, " Nome BT: %s", device_name);

    ESP_LOGW(TAG, " SPP PIN legado: 6500");

    ESP_LOGW(TAG, "==================================================");



    g_rxQueue = xQueueCreate(24, sizeof(RxChunk));

    if (!g_rxQueue) {

        ESP_LOGE(TAG, "Falha criando fila RX");

        while (true) vTaskDelay(pdMS_TO_TICKS(1000));

    }



    if (xTaskCreatePinnedToCore(

            otaWorker,

            "bt_ota_worker",

            6144,

            nullptr,

            5,

            &g_workerTask,

            0) != pdPASS) {

        ESP_LOGE(TAG, "Falha criando OTA worker");

        while (true) vTaskDelay(pdMS_TO_TICKS(1000));

    }



    esp_bt_controller_config_t cfg =

        BT_CONTROLLER_INIT_CONFIG_DEFAULT();




    // BDK V2.5 FIX3
    // O projeto normal usa BTDM (Classic + BLE), entao o macro
    // BT_CONTROLLER_INIT_CONFIG_DEFAULT() herda mode=BTDM do sdkconfig.
    // No modo OTA queremos SOMENTE Bluetooth Classic SPP.
    // esp_bt_controller_enable() exige que o argumento corresponda ao
    // cfg.mode usado em esp_bt_controller_init().
    cfg.mode = ESP_BT_MODE_CLASSIC_BT;

    ESP_LOGI(TAG,
             "BT controller cfg.mode = CLASSIC_BT (%d)",
             (int)cfg.mode);

    esp_err_t err = esp_bt_controller_init(&cfg);

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {

        ESP_LOGE(TAG, "BT controller init: %s", esp_err_to_name(err));

        while (true) vTaskDelay(pdMS_TO_TICKS(1000));

    }
    err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG,
                 "BT controller enable CLASSIC_BT: %s",
                 esp_err_to_name(err));
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "BT controller CLASSIC_BT: ENABLED");




    esp_bluedroid_config_t bdCfg =

        BT_BLUEDROID_INIT_CONFIG_DEFAULT();

    bdCfg.ssp_en = true;



    err = esp_bluedroid_init_with_cfg(&bdCfg);

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {

        ESP_LOGE(TAG, "Bluedroid init: %s", esp_err_to_name(err));

        while (true) vTaskDelay(pdMS_TO_TICKS(1000));

    }



    err = esp_bluedroid_enable();

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {

        ESP_LOGE(TAG, "Bluedroid enable: %s", esp_err_to_name(err));

        while (true) vTaskDelay(pdMS_TO_TICKS(1000));

    }



    esp_bt_gap_register_callback(gapCallback);

    esp_spp_register_callback(sppCallback);



#if defined(CONFIG_BT_SSP_ENABLED) && CONFIG_BT_SSP_ENABLED

    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;

    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;

    esp_bt_gap_set_security_param(

        param_type,

        &iocap,

        sizeof(uint8_t));

#endif



    // Tambem deixa PIN legado 6500 disponivel para clientes que solicitarem.

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;

    esp_bt_pin_code_t pin_code = {'6','5','0','0'};

    esp_bt_gap_set_pin(pin_type, 4, pin_code);



    esp_bt_gap_set_device_name(device_name);



    esp_spp_cfg_t sppCfg = {};

    sppCfg.mode = ESP_SPP_MODE_CB;

    sppCfg.enable_l2cap_ertm = true;

    sppCfg.tx_buffer_size = 0;



    err = esp_spp_enhanced_init(&sppCfg);

    if (err != ESP_OK) {

        ESP_LOGE(TAG, "SPP init: %s", esp_err_to_name(err));

        while (true) vTaskDelay(pdMS_TO_TICKS(1000));

    }



    ESP_LOGI(TAG, "Aguardando celular conectar ao SPP...");



    while (true) {

        vTaskDelay(pdMS_TO_TICKS(1000));

    }

}



// ------------------------------------------------------------

// Rollback confirmation

// ------------------------------------------------------------

static void validationTask(void *arg)

{

    const uint32_t delayMs = (uint32_t)(uintptr_t)arg;

    vTaskDelay(pdMS_TO_TICKS(delayMs));



    const esp_partition_t *running =

        esp_ota_get_running_partition();



    if (!running) {

        ESP_LOGW(TAG, "Nao foi possivel obter particao atual");

        vTaskDelete(nullptr);

        return;

    }



    esp_ota_img_states_t state;

    esp_err_t err =

        esp_ota_get_state_partition(running, &state);



    if (err == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY) {

        ESP_LOGW(TAG,

                 "App OTA PENDING_VERIFY -> confirmando como VALID");



        err = esp_ota_mark_app_valid_cancel_rollback();



        if (err == ESP_OK) {

            ESP_LOGI(TAG, "App OTA confirmado / rollback cancelado");

        } else {

            ESP_LOGE(TAG,

                     "Falha confirmando OTA: %s",

                     esp_err_to_name(err));

        }

    } else if (err == ESP_OK) {

        ESP_LOGI(TAG,

                 "Estado OTA atual=%d; nenhuma confirmacao necessaria",

                 (int)state);

    } else {

        ESP_LOGI(TAG,

                 "Estado OTA nao aplicavel: %s",

                 esp_err_to_name(err));

    }



    vTaskDelete(nullptr);

}



void btOtaScheduleRunningImageValidation(uint32_t delay_ms)

{

    if (delay_ms < 1000) delay_ms = 1000;



    xTaskCreatePinnedToCore(

        validationTask,

        "ota_validate",

        3072,

        (void *)(uintptr_t)delay_ms,

        2,

        nullptr,

        0);

}



// ------------------------------------------------------------

// BOOT button watcher

// ------------------------------------------------------------

struct ButtonCfg {

    int gpio;

    uint32_t holdMs;

};



static void buttonTask(void *arg)

{

    ButtonCfg cfg = *(ButtonCfg *)arg;

    delete (ButtonCfg *)arg;



    const gpio_num_t pin = (gpio_num_t)cfg.gpio;



    gpio_set_direction(pin, GPIO_MODE_INPUT);

    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);



    uint32_t lowSince = 0;

    bool triggered = false;



    ESP_LOGI(TAG,

             "BOOT OTA watcher: GPIO%d / segure %lu ms",

             cfg.gpio,

             (unsigned long)cfg.holdMs);



    while (true) {

        const int level = gpio_get_level(pin);

        const uint32_t now = (uint32_t)(esp_log_timestamp());



        if (!triggered) {

            if (level == 0) {

                if (lowSince == 0) {

                    lowSince = now;

                } else if ((uint32_t)(now - lowSince) >= cfg.holdMs) {

                    triggered = true;



                    ESP_LOGW(TAG, "");

                    ESP_LOGW(TAG, "BOOT pressionado -> preparando OTA");

                    ESP_LOGW(TAG, "SOLTE O BOTAO para reiniciar em OTA");



                    if (!btOtaRequestOnNextBoot()) {

                        ESP_LOGE(TAG, "Nao foi possivel gravar flag OTA");

                        triggered = false;

                        lowSince = 0;

                    }

                }

            } else {

                lowSince = 0;

            }

        } else {

            // Nunca reiniciar com GPIO0 ainda LOW: isso poderia entrar

            // no ROM download bootloader em vez do firmware OTA.

            if (level != 0) {

                ESP_LOGW(TAG, "BOOT solto -> reiniciando em OTA...");

                vTaskDelay(pdMS_TO_TICKS(300));

                esp_restart();

            }

        }



        vTaskDelay(pdMS_TO_TICKS(50));

    }

}



bool btOtaStartBootButtonWatcher(int gpio_num, uint32_t hold_ms)

{

    if (hold_ms < 1000) hold_ms = 1000;



    ButtonCfg *cfg = new ButtonCfg;

    if (!cfg) return false;



    cfg->gpio = gpio_num;

    cfg->holdMs = hold_ms;



    BaseType_t ok = xTaskCreatePinnedToCore(

        buttonTask,

        "ota_boot_btn",

        3072,

        cfg,

        2,

        nullptr,

        0);



    if (ok != pdPASS) {

        delete cfg;

        return false;

    }



    return true;

}
