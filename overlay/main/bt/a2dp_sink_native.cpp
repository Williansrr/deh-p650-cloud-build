/* Native ESP-IDF A2DP Sink Implementation */



#include "a2dp_sink_native.h"

#include "esp_timer.h"

#include "nvs_flash.h"

#include "nvs.h"



// LDAC vendor codec constants (inlined to avoid internal stack header dependency)

#define A2DP_LDAC_VENDOR_ID           0x0000012D

#define A2DP_LDAC_CODEC_ID            0x00AA

#define A2DP_LDAC_SAMPLING_FREQ_MASK  0x3F

#define A2DP_LDAC_SAMPLING_FREQ_44100 0x20

#define A2DP_LDAC_SAMPLING_FREQ_48000 0x10

#define A2DP_LDAC_SAMPLING_FREQ_88200 0x08

#define A2DP_LDAC_SAMPLING_FREQ_96000   0x04

#define A2DP_LDAC_SAMPLING_FREQ_176400  0x02

#define A2DP_LDAC_SAMPLING_FREQ_192000  0x01

#define A2DP_LDAC_CHANNEL_MODE_MASK   0x07

#define A2DP_LDAC_CHANNEL_MODE_MONO   0x04



// Opus over A2DP vendor codec constants. This stack decodes Opus to 48 kHz

// signed 16-bit PCM, so report 48k/16-bit to the render pipeline.

#define A2DP_OPUS_VENDOR_ID          0x000005F1

#define A2DP_OPUS_CODEC_ID           0x1005



#define BT_AV_TAG    "BT_AV"

#define BT_RC_TG_TAG "BT_RC_TG"

#define BT_RC_CT_TAG "BT_RC_CT"



extern "C" void bdkNativeAvrcConnection(bool connected);

extern "C" void bdkNativeAvrcMetadata(uint8_t attr, const uint8_t *data, size_t len);



static void bdkRequestMetadata()

{

    const uint8_t mask =

        ESP_AVRC_MD_ATTR_TITLE |

        ESP_AVRC_MD_ATTR_ARTIST |

        ESP_AVRC_MD_ATTR_ALBUM |

        ESP_AVRC_MD_ATTR_TRACK_NUM |

        ESP_AVRC_MD_ATTR_NUM_TRACKS;



    esp_err_t err = esp_avrc_ct_send_metadata_cmd(1, mask);

    ESP_LOGI(BT_RC_CT_TAG, "metadata request: %s", esp_err_to_name(err));

}



NativeA2DPSink* NativeA2DPSink::instance = nullptr;



// BDK V2.3.3 - automatic reconnection to the last successfully

// connected A2DP source saved in NVS.

static TaskHandle_t s_bdk_auto_reconnect_task = nullptr;

// ============================================================
// BDK XM V1.0.7.9 - PORT RECONNECT ESP32-A2DP 1.8.11
// ============================================================
enum Bdk1811ReconnectState : uint8_t {
    BDK1811_NO_RECONNECT = 0,
    BDK1811_AUTO_RECONNECT,
    BDK1811_IS_RECONNECTING
};

static volatile Bdk1811ReconnectState s_bdk1811ReconnectState =
    BDK1811_NO_RECONNECT;

static volatile uint32_t s_bdk1811ConnectionRetryCount = 0;
static volatile uint32_t s_bdk1811ReconnectTimeoutMs = 0;

static constexpr uint32_t BDK1811_DEFAULT_RECONNECT_TIMEOUT_MS = 10000U;
static constexpr uint32_t BDK1811_RECONNECT_START_DELAY_MS = 1000U;


static bool bdkBdaIsZero(const uint8_t *bda)
{
    if (!bda) return true;
    for (int i = 0; i < ESP_BD_ADDR_LEN; ++i) {
        if (bda[i] != 0) return false;
    }
    return true;
}







struct bt_app_msg_t {

    uint16_t sig;

    uint16_t event;

    void *param;

    void (*cb)(uint16_t event, void *param);

};



NativeA2DPSink::NativeA2DPSink()
{
    instance = this;
    _lock_init(&s_volume_lock);

    // Supervisor de reconnect parte de estado conhecido.
    connection_state = ESP_A2D_CONNECTION_STATE_DISCONNECTED;
    memset(peer_bd_addr, 0, ESP_BD_ADDR_LEN);
    memset(last_connection, 0, ESP_BD_ADDR_LEN);
}

NativeA2DPSink::~NativeA2DPSink() { if (app_task_handle) { end(true); } }



void NativeA2DPSink::gap_cb_trampoline(esp_bt_gap_cb_event_t e, esp_bt_gap_cb_param_t *p) { if (instance) instance->gap_cb(e,p); }

void NativeA2DPSink::a2d_cb_trampoline(esp_a2d_cb_event_t e, esp_a2d_cb_param_t *p) { if (instance) instance->a2d_cb(e,p); }

void NativeA2DPSink::rc_ct_cb_trampoline(esp_avrc_ct_cb_event_t e, esp_avrc_ct_cb_param_t *p) { if (instance) instance->rc_ct_cb(e,p); }

void NativeA2DPSink::rc_tg_cb_trampoline(esp_avrc_tg_cb_event_t e, esp_avrc_tg_cb_param_t *p) { if (instance) instance->rc_tg_cb(e,p); }

void NativeA2DPSink::data_cb_trampoline(const uint8_t *data, uint32_t len) { if (instance) instance->data_cb(data,len); }

void NativeA2DPSink::app_task_handler_trampoline(void *arg) { if (instance) instance->app_task_handler(arg); }



void NativeA2DPSink::init_bluetooth() {

    // Bluetooth controller and bluedroid are initialized centrally in main.cpp

    // to avoid double-init when both BLE and Classic BT (A2DP) are used.

    // ESP32-A2DP 1.8.11 default when explicit PIN mode is not active:
    // SSP IO capability = NONE / fixed PIN with length 0.
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_err_t sec_err =
        esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code = {};
    esp_err_t pin_err =
        esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(
        TAG,
        "V1.0.7.9 SEC 1.8.11: IOCAP_NONE=%s PIN_LEN0=%s",
        esp_err_to_name(sec_err),
        esp_err_to_name(pin_err));

}



void NativeA2DPSink::start(const char *name) {

    if (is_start_disabled) { ESP_LOGE(TAG, "re-start not supported after end(true)"); return; }

    if (name) bt_name = name;

    // ESP32-A2DP 1.8.11:
    // start() ativa o gerenciamento de reconnect a partir do estado
    // configurado por set_auto_reconnect(true), carrega o peer ANTES
    // do stack-up e aplica 1000 ms de reconnect_delay.
    is_autoreconnect_allowed = (reconnect_status == AutoReconnect);

    if (is_autoreconnect_allowed) {
        get_last_connection();

        if (!bdkBdaIsZero(last_connection)) {
            memcpy(peer_bd_addr, last_connection, ESP_BD_ADDR_LEN);
            s_bdk1811ReconnectState = BDK1811_AUTO_RECONNECT;

            ESP_LOGI(
                BT_AV_TAG,
                "BDK XM V1.0.7.12 ACL_DIAG: peer pre-carregado; reconnect sera feito somente pelo supervisor unico");
        } else {
            ESP_LOGW(
                BT_AV_TAG,
                "BDK XM V1.0.7.9 1.8.11 START: sem peer salvo; auto interno aguardara conexao manual");
            is_autoreconnect_allowed = false;
            s_bdk1811ReconnectState = BDK1811_NO_RECONNECT;
        }
    }

    init_bluetooth();

    app_task_queue = xQueueCreate(20, sizeof(bt_app_msg_t));

    if (!app_task_queue) { ESP_LOGE(TAG, "queue create failed"); return; }

    if (xTaskCreatePinnedToCore(app_task_handler_trampoline, "btAppTask", 3072, this,

            configMAX_PRIORITIES - 3, &app_task_handle, task_core) != pdPASS) {

        ESP_LOGE(TAG, "task create failed"); vQueueDelete(app_task_queue); app_task_queue=nullptr; return;

    }

    app_work_dispatch([](uint16_t e, void *p){ if(instance) instance->av_hdl_stack_evt(e,p); }, 0, nullptr, 0);

}



void NativeA2DPSink::start(const char *name, bool auto_reconnect) {

    set_auto_reconnect(auto_reconnect);

    start(name);

}



void NativeA2DPSink::end(bool release_memory) {

    is_autoreconnect_allowed = false;



    if (s_bdk_auto_reconnect_task) {

        vTaskDelete(s_bdk_auto_reconnect_task);

        s_bdk_auto_reconnect_task = nullptr;

    }



    if (app_task_handle) { vTaskDelete(app_task_handle); app_task_handle = nullptr; }

    if (app_task_queue) { vQueueDelete(app_task_queue); app_task_queue = nullptr; }

    esp_a2d_sink_deinit();

    esp_avrc_ct_deinit();

    esp_avrc_tg_deinit();

    esp_bluedroid_disable();

    esp_bluedroid_deinit();

    esp_bt_controller_disable();

    esp_bt_controller_deinit();

    if (release_memory) is_start_disabled = true;

}



bool NativeA2DPSink::app_work_dispatch(void (*p_cback)(uint16_t,void*), uint16_t event, void *p_params, int param_len) {

    if (!app_task_queue) return false;

    bt_app_msg_t msg = {}; msg.sig = APP_SIG_WORK_DISPATCH; msg.event = event; msg.cb = p_cback;

    if (param_len > 0) { msg.param = malloc(param_len); if (!msg.param) return false; memcpy(msg.param, p_params, param_len); }

    if (xQueueSend(app_task_queue, &msg, portMAX_DELAY) != pdPASS) { free(msg.param); return false; }

    return true;

}



void NativeA2DPSink::app_task_handler(void *arg) {

    bt_app_msg_t msg;

    while (true) {

        if (xQueueReceive(app_task_queue, &msg, portMAX_DELAY) == pdPASS) {

            if (msg.sig == APP_SIG_WORK_DISPATCH && msg.cb) { msg.cb(msg.event, msg.param); free(msg.param); }

        }

    }

}



void NativeA2DPSink::av_hdl_stack_evt(uint16_t event, void *p_param) {

    (void)event;

    (void)p_param;



    ESP_LOGI(TAG, "========================================");

    ESP_LOGI(TAG, "BDK XM V1.0.7.9 AUTO CONNECT + OTA - LDAC + PCM5102 + AVRCP + XM");

    ESP_LOGI(TAG, "========================================");



    esp_err_t name_err = esp_bt_dev_set_device_name(bt_name.c_str());

    ESP_LOGI(TAG, "set_device_name: %s", esp_err_to_name(name_err));



    esp_err_t gap_err = esp_bt_gap_register_callback(gap_cb_trampoline);

    ESP_LOGI(TAG, "GAP callback: %s", esp_err_to_name(gap_err));



    // AVRCP must be initialized before A2DP.

    esp_err_t ct_cb_err = esp_avrc_ct_register_callback(rc_ct_cb_trampoline);

    ESP_LOGI(TAG, "AVRCP CT callback: %s", esp_err_to_name(ct_cb_err));



    esp_err_t ct_init_err = esp_avrc_ct_init();

    ESP_LOGI(TAG, "AVRCP CT init: %s", esp_err_to_name(ct_init_err));



    esp_err_t tg_cb_err = esp_avrc_tg_register_callback(rc_tg_cb_trampoline);

    ESP_LOGI(TAG, "AVRCP TG callback: %s", esp_err_to_name(tg_cb_err));



    esp_err_t tg_init_err = esp_avrc_tg_init();

    ESP_LOGI(TAG, "AVRCP TG init: %s", esp_err_to_name(tg_init_err));



    esp_avrc_rn_evt_cap_mask_t evt_set = {};

    esp_avrc_rn_evt_bit_mask_operation(

        ESP_AVRC_BIT_MASK_OP_SET,

        &evt_set,

        ESP_AVRC_RN_VOLUME_CHANGE);

    esp_err_t cap_err = esp_avrc_tg_set_rn_evt_cap(&evt_set);

    ESP_LOGI(TAG, "AVRCP TG RN cap: %s", esp_err_to_name(cap_err));



    esp_err_t a2d_cb_err = esp_a2d_register_callback(a2d_cb_trampoline);

    ESP_LOGI(TAG, "A2DP callback: %s", esp_err_to_name(a2d_cb_err));



    esp_err_t a2d_init_err = esp_a2d_sink_init();

    ESP_LOGI(TAG, "A2DP sink init: %s", esp_err_to_name(a2d_init_err));



    esp_err_t data_cb_err = esp_a2d_sink_register_data_callback(data_cb_trampoline);

    ESP_LOGI(TAG, "A2DP data callback: %s", esp_err_to_name(data_cb_err));



    // ========================================================
    // BDK XM V1.0.7.12 - PAGE FOCUS RECONNECT
    // Nao conecta no STACK_UP. A pilha termina de subir, entra em
    // scan connectable e SOMENTE o supervisor unico pode iniciar
    // esp_a2d_sink_connect(). Isso evita duas maquinas disputando
    // CONNECTING/DISCONNECTED durante a inicializacao.
    // ========================================================
    if (is_autoreconnect_allowed && !bdkBdaIsZero(last_connection)) {
        memcpy(peer_bd_addr, last_connection, ESP_BD_ADDR_LEN);
        s_bdk1811ReconnectState = BDK1811_AUTO_RECONNECT;
        ESP_LOGI(
            BT_AV_TAG,
            "BDK XM V1.0.7.12 ACL_DIAG: STACK_UP sem connect; peer pronto para supervisor unico");
    }



    esp_err_t scan_err = esp_bt_gap_set_scan_mode(

        ESP_BT_CONNECTABLE,

        ESP_BT_GENERAL_DISCOVERABLE

    );

    ESP_LOGI(TAG, "set_scan_mode: %s", esp_err_to_name(scan_err));



    if (scan_err != ESP_OK) {

        ESP_LOGE(TAG, "set_scan_mode failed: %s", esp_err_to_name(scan_err));

    }



    ESP_LOGI(TAG, "A2DP + AVRCP prontos");

    // V1.0.7.12: consulta o PAGE timeout apenas para diagnostico.
    // Nao altera o valor: as quedas observadas acontecem em ~20-30 ms,
    // muito antes do timeout de PAGE.
    esp_err_t page_to_q = esp_bt_gap_get_page_timeout();
    ESP_LOGI(BT_AV_TAG,
             "BDK XM V1.0.7.12 ACL_DIAG: get_page_timeout request=%s",
             esp_err_to_name(page_to_q));



    if (reconnect_status == AutoReconnect && s_bdk_auto_reconnect_task == nullptr) {

        BaseType_t task_ok = xTaskCreatePinnedToCore(

            [](void *arg) {

                NativeA2DPSink *self =
                    static_cast<NativeA2DPSink *>(arg);

                // V1.0.7.12 ACL_DIAG RECONNECT STATE:
                // a task abaixo e a UNICA dona de esp_a2d_sink_connect().
                // Primeiro deixa A2DP/AVRCP/GAP estabilizarem por 2,5 s.
                // Depois: 5 s nas primeiras 12 tentativas; 15 s apos isso.
                vTaskDelay(pdMS_TO_TICKS(2500));

                ESP_LOGI(
                    BT_AV_TAG,
                    "BDK XM V1.0.7.12 ACL_DIAG: supervisor unico iniciado apos 2500 ms");

                uint32_t opAttempts = 0;

                while (true) {

                    const esp_a2d_connection_state_t state =
                        self->get_connection_state();

                    if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
                        opAttempts = 0;
                        vTaskDelay(pdMS_TO_TICKS(15000));
                        continue;
                    }

                    if (state != ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
                        vTaskDelay(pdMS_TO_TICKS(500));
                        continue;
                    }

                    if (!self->has_last_connection()) {
                        ESP_LOGW(
                            BT_AV_TAG,
                            "BDK XM V1.0.7.12 ACL_DIAG: "
                            "sem last_bda; aguardando conexao iniciada pelo telefone");
                        vTaskDelay(pdMS_TO_TICKS(5000));
                        continue;
                    }

                    self->get_last_connection();

                    if (bdkBdaIsZero(self->last_connection)) {
                        vTaskDelay(pdMS_TO_TICKS(5000));
                        continue;
                    }

                    memcpy(
                        self->peer_bd_addr,
                        self->last_connection,
                        ESP_BD_ADDR_LEN);

                    // Supervisor unico assume a tentativa. Nao existe
                    // reconnect no STACK_UP nem retry dentro do callback.
                    self->is_autoreconnect_allowed = true;
                    s_bdk1811ReconnectState = BDK1811_IS_RECONNECTING;
                    s_bdk1811ReconnectTimeoutMs =
                        (uint32_t)(esp_timer_get_time() / 1000ULL) +
                        BDK1811_DEFAULT_RECONNECT_TIMEOUT_MS;

                    // V1.0.7.12 ACL DIAG:
                    // Durante uma tentativa iniciada pelo ESP32, permanece
                    // CONNECTABLE, mas sai do inquiry scan. Assim o radio
                    // fica focado em PAGE/ACL/AVDTP durante o reconnect.
                    esp_err_t page_scan_err = esp_bt_gap_set_scan_mode(
                        ESP_BT_CONNECTABLE,
                        ESP_BT_NON_DISCOVERABLE);

                    ESP_LOGI(
                        BT_AV_TAG,
                        "BDK XM V1.0.7.12 ACL_DIAG: PRE-PAGE scan=%s "
                        "(CONNECTABLE/NON_DISCOVERABLE)",
                        esp_err_to_name(page_scan_err));

                    // Aguarda o controller aplicar o novo scan mode.
                    vTaskDelay(pdMS_TO_TICKS(150));

                    opAttempts++;

                    ESP_LOGI(
                        BT_AV_TAG,
                        "BDK XM V1.0.7.12 ACL_DIAG: RECONNECT #%u -> "
                        "%02x:%02x:%02x:%02x:%02x:%02x timeout=%u",
                        (unsigned)opAttempts,
                        self->peer_bd_addr[0], self->peer_bd_addr[1],
                        self->peer_bd_addr[2], self->peer_bd_addr[3],
                        self->peer_bd_addr[4], self->peer_bd_addr[5],
                        (unsigned)s_bdk1811ReconnectTimeoutMs);

                    esp_err_t err =
                        esp_a2d_sink_connect(self->peer_bd_addr);

                    ESP_LOGI(
                        BT_AV_TAG,
                        "BDK XM V1.0.7.12 ACL_DIAG: connect = %s",
                        esp_err_to_name(err));

                    const uint32_t nextDelayMs =
                        (opAttempts < 12U) ? 5000U : 15000U;

                    vTaskDelay(pdMS_TO_TICKS(nextDelayMs));
                }
            },

            "a2dp_single_reconnect",

            4096,

            this,

            4,

            &s_bdk_auto_reconnect_task,

            0);

        if (task_ok == pdPASS) {
            ESP_LOGI(
                TAG,
                "Auto reconnect: ON / V1.0.7.12 ACL_DIAG RECONNECT STATE");
        } else {
            s_bdk_auto_reconnect_task = nullptr;
            ESP_LOGE(
                TAG,
                "Auto reconnect supervisor: CREATE FAILED");
        }
    }
}

void NativeA2DPSink::gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {

    uint8_t *bda = nullptr;

    switch (event) {

    case ESP_BT_GAP_AUTH_CMPL_EVT:

        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "auth success: %s", param->auth_cmpl.device_name);

            if (!bdkBdaIsZero(param->auth_cmpl.bda)) {
                set_last_connection(param->auth_cmpl.bda);
                memcpy(peer_bd_addr, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
                ESP_LOGI(BT_AV_TAG, "BDK XM V1.0.7.9 AUTO: peer salvo no AUTH");
            }

        } else {
            ESP_LOGE(BT_AV_TAG, "auth failed, status:%d", param->auth_cmpl.stat);
        }

        break;

    case ESP_BT_GAP_CFM_REQ_EVT:

        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);

        break;

    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        ESP_LOGI(BT_AV_TAG,
                 "ACL-DIAG CONN_CMPL stat=0x%02X handle=0x%04X bda=%02x:%02x:%02x:%02x:%02x:%02x",
                 (unsigned)param->acl_conn_cmpl_stat.stat,
                 (unsigned)param->acl_conn_cmpl_stat.handle,
                 param->acl_conn_cmpl_stat.bda[0], param->acl_conn_cmpl_stat.bda[1],
                 param->acl_conn_cmpl_stat.bda[2], param->acl_conn_cmpl_stat.bda[3],
                 param->acl_conn_cmpl_stat.bda[4], param->acl_conn_cmpl_stat.bda[5]);
        break;

    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        ESP_LOGW(BT_AV_TAG,
                 "ACL-DIAG DISCONN_CMPL reason=0x%02X handle=0x%04X bda=%02x:%02x:%02x:%02x:%02x:%02x",
                 (unsigned)param->acl_disconn_cmpl_stat.reason,
                 (unsigned)param->acl_disconn_cmpl_stat.handle,
                 param->acl_disconn_cmpl_stat.bda[0], param->acl_disconn_cmpl_stat.bda[1],
                 param->acl_disconn_cmpl_stat.bda[2], param->acl_disconn_cmpl_stat.bda[3],
                 param->acl_disconn_cmpl_stat.bda[4], param->acl_disconn_cmpl_stat.bda[5]);
        break;

    case ESP_BT_GAP_ENC_CHG_EVT:
        ESP_LOGI(BT_AV_TAG, "ACL-DIAG ENC_CHG event recebido");
        break;

    case ESP_BT_GAP_SET_PAGE_TO_EVT:
        ESP_LOGI(BT_AV_TAG,
                 "ACL-DIAG SET_PAGE_TO stat=0x%02X",
                 (unsigned)param->set_page_timeout.stat);
        break;

    case ESP_BT_GAP_GET_PAGE_TO_EVT:
        ESP_LOGI(BT_AV_TAG,
                 "ACL-DIAG PAGE_TIMEOUT stat=0x%02X value=0x%04X (%u ms aprox)",
                 (unsigned)param->get_page_timeout.stat,
                 (unsigned)param->get_page_timeout.page_to,
                 (unsigned)((param->get_page_timeout.page_to * 625U) / 1000U));
        break;

    case ESP_BT_GAP_MODE_CHG_EVT:

        ESP_LOGI(BT_AV_TAG, "ACL-DIAG MODE_CHG mode=%d", param->mode_chg.mode);

        break;

    default: break;

    }

}



void NativeA2DPSink::a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {

    app_work_dispatch([](uint16_t e, void *p){ if(instance) instance->av_hdl_a2d_evt(e,p); }, event, param, sizeof(esp_a2d_cb_param_t));

}



static const char* bdk_a2dp_event_name(uint16_t event) {

    switch (event) {

        case ESP_A2D_CONNECTION_STATE_EVT:      return "CONNECTION_STATE";

        case ESP_A2D_AUDIO_STATE_EVT:           return "AUDIO_STATE";

        case ESP_A2D_AUDIO_CFG_EVT:             return "AUDIO_CFG";

        case ESP_A2D_MEDIA_CTRL_ACK_EVT:        return "MEDIA_CTRL_ACK";

        case ESP_A2D_PROF_STATE_EVT:            return "PROF_STATE";

        case ESP_A2D_SEP_REG_STATE_EVT:         return "SEP_REG_STATE";

        case ESP_A2D_SNK_PSC_CFG_EVT:           return "SNK_PSC_CFG";

        case ESP_A2D_SNK_SET_DELAY_VALUE_EVT:   return "SNK_SET_DELAY";

        case ESP_A2D_SNK_GET_DELAY_VALUE_EVT:   return "SNK_GET_DELAY";

        default:                                return "OTHER";

    }

}



void NativeA2DPSink::av_hdl_a2d_evt(uint16_t event, void *p_param) {

    esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t*)p_param;



    ESP_LOGI(

        BT_AV_TAG,

        "BDK2.3.3 EVT id=%u name=%s",

        (unsigned)event,

        bdk_a2dp_event_name(event));



    switch (event) {

    case ESP_A2D_CONNECTION_STATE_EVT: {

        uint8_t *bda = a2d->conn_stat.remote_bda;
        connection_state = a2d->conn_stat.state;

        ESP_LOGI(
            BT_AV_TAG,
            "conn state=%d hdl=%u mtu=%u disc_rsn=%d [%02x:%02x:%02x:%02x:%02x:%02x]",
            (int)connection_state,
            (unsigned)a2d->conn_stat.conn_hdl,
            (unsigned)a2d->conn_stat.audio_mtu,
            (int)a2d->conn_stat.disc_rsn,
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

        // ========================================================
        // PORT ESP32-A2DP 1.8.11 / handle_connection_state()
        // ========================================================
        if (connection_state == ESP_A2D_CONNECTION_STATE_DISCONNECTING) {

            ESP_LOGI(BT_AV_TAG, "1.8.11: DISCONNECTING");

            if (a2d->conn_stat.disc_rsn == ESP_A2D_DISC_RSN_NORMAL) {
                // A biblioteca 1.8.11 desarma o auto interno em NORMAL.
                // O reforço Operational V2.16 continua podendo rearmar
                // explicitamente depois.
                is_autoreconnect_allowed = false;
            }

        } else if (connection_state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {

            const char *reason =
                (a2d->conn_stat.disc_rsn == ESP_A2D_DISC_RSN_NORMAL)
                    ? "NORMAL"
                    : "ABNORMAL";

            ESP_LOGW(
                BT_AV_TAG,
                "BDK XM V1.0.7.12 ACL_DIAG DISCONNECTED reason=%d -> %s",
                (int)a2d->conn_stat.disc_rsn,
                reason);

            if (a2d->conn_stat.disc_rsn == ESP_A2D_DISC_RSN_NORMAL) {
                is_autoreconnect_allowed = false;
            }

            // V1.0.7.10: callback NUNCA chama esp_a2d_sink_connect().
            // Ele apenas libera o estado para o supervisor unico tentar
            // novamente na cadencia Operational.
            if (reconnect_status == AutoReconnect) {
                is_autoreconnect_allowed = true;
                s_bdk1811ReconnectState = BDK1811_AUTO_RECONNECT;
            }

            // Fora da tentativa outbound, volta a ficar visivel para
            // permitir que o telefone tambem possa iniciar a conexao.
            esp_err_t restore_scan_err = esp_bt_gap_set_scan_mode(
                ESP_BT_CONNECTABLE,
                ESP_BT_GENERAL_DISCOVERABLE);
            ESP_LOGI(
                BT_AV_TAG,
                "BDK XM V1.0.7.12 ACL_DIAG: DISCONNECTED restore scan=%s "
                "(CONNECTABLE/GENERAL_DISCOVERABLE)",
                esp_err_to_name(restore_scan_err));

            ESP_LOGI(
                BT_AV_TAG,
                "BDK XM V1.0.7.12 ACL_DIAG: DISCONNECTED -> aguardando supervisor; sem retry no callback");

            if (!bdkBdaIsZero(bda)) {
                memcpy(peer_bd_addr, bda, ESP_BD_ADDR_LEN);
            }

        } else if (connection_state == ESP_A2D_CONNECTION_STATE_CONNECTING) {

            s_bdk1811ConnectionRetryCount++;

            ESP_LOGI(
                BT_AV_TAG,
                "BDK XM V1.0.7.12 ACL_DIAG: CONNECTING count=%u",
                (unsigned)s_bdk1811ConnectionRetryCount);

        } else if (connection_state == ESP_A2D_CONNECTION_STATE_CONNECTED) {

            // 1.8.11: fim do IsReconnecting -> AutoReconnect
            if (s_bdk1811ReconnectState == BDK1811_IS_RECONNECTING) {
                s_bdk1811ReconnectState = BDK1811_AUTO_RECONNECT;
            }

            is_autoreconnect_allowed = true;
            s_bdk1811ConnectionRetryCount = 0;
            s_bdk1811ReconnectTimeoutMs = 0;

            esp_bt_gap_set_scan_mode(
                ESP_BT_CONNECTABLE,
                ESP_BT_NON_DISCOVERABLE);

            if (!bdkBdaIsZero(bda)) {
                memcpy(peer_bd_addr, bda, ESP_BD_ADDR_LEN);
                memcpy(last_connection, bda, ESP_BD_ADDR_LEN);

                // 1.8.11 persiste a conexão quando o modo de
                // auto-reconnect está ativo.
                if (reconnect_status == AutoReconnect) {
                    set_last_connection(peer_bd_addr);
                }

                ESP_LOGI(
                    BT_AV_TAG,
                    "BDK XM V1.0.7.12 ACL_DIAG: CONNECTED / peer persistido / supervisor pausado");
            }
        }

        if (connection_state_cb) {
            connection_state_cb(connection_state, this);
        }

        break;
    }

    case ESP_A2D_AUDIO_STATE_EVT:

        ESP_LOGI(BT_AV_TAG, "audio state %d", a2d->audio_stat.state);

        if (audio_state_cb) audio_state_cb(a2d->audio_stat.state, this);

        break;

    case ESP_A2D_AUDIO_CFG_EVT: {

        esp_a2d_mcc_t *p_mcc = &a2d->audio_cfg.mcc;

        audio_type = p_mcc->type;

        uint32_t sr = 44100; uint8_t bits = 16; uint8_t ch = 2;

        if (audio_type == ESP_A2D_MCT_SBC) {

            if (p_mcc->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_32K) sr = 32000;

            else if (p_mcc->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_44K) sr = 44100;

            else if (p_mcc->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_48K) sr = 48000;

            if (p_mcc->cie.sbc_info.ch_mode & ESP_A2D_SBC_CIE_CH_MODE_MONO) ch = 1;

        } else if (audio_type == ESP_A2D_MCT_M24) {

            const esp_a2d_cie_m24_t &aac = p_mcc->cie.m24_info;

            const uint8_t *raw = reinterpret_cast<const uint8_t*>(&aac);

            uint16_t sampleBits = ((uint16_t)aac.samp_freq1 << 4) | (aac.samp_freq2 & 0x0F);

            if (sampleBits & 0x800) sr = 8000;

            else if (sampleBits & 0x400) sr = 11025;

            else if (sampleBits & 0x200) sr = 12000;

            else if (sampleBits & 0x100) sr = 16000;

            else if (sampleBits & 0x080) sr = 22050;

            else if (sampleBits & 0x040) sr = 24000;

            else if (sampleBits & 0x020) sr = 32000;

            else if (sampleBits & 0x010) sr = 44100;

            else if (sampleBits & 0x008) sr = 48000;

            else if (sampleBits & 0x004) sr = 64000;

            else if (sampleBits & 0x002) sr = 88200;

            else if (sampleBits & 0x001) sr = 96000;

            else sr = 44100;

            ch = (aac.ch == 1) ? 1 : 2;

            bits = 16;

            ESP_LOGI(BT_AV_TAG, "Detected AAC codec sr=%u bits=0x%03X ch_field=0x%X raw=%02X %02X %02X %02X %02X %02X",

                     sr, sampleBits, aac.ch,

                     raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);

        } else if (audio_type == ESP_A2D_MCT_NON_A2DP) {

            // Vendor codec (aptX, LDAC, etc.): parse raw CIE bytes

            // LDAC layout in 8-byte cie.ldac: vendorId[0-3], codecId[4-5], sampleRate[6], channelMode[7]

            uint8_t *raw = p_mcc->cie.ldac;



            ESP_LOGI(

                BT_AV_TAG,

                "VENDOR CIE raw=%02X %02X %02X %02X %02X %02X %02X %02X",

                raw[0], raw[1], raw[2], raw[3],

                raw[4], raw[5], raw[6], raw[7]);



            uint32_t vendorId = raw[0] | (raw[1] << 8) | (raw[2] << 16) | (raw[3] << 24);

            uint16_t codecId  = raw[4] | (raw[5] << 8);

            if (vendorId == A2DP_LDAC_VENDOR_ID && codecId == A2DP_LDAC_CODEC_ID) {

                uint8_t sr_bits = raw[6] & A2DP_LDAC_SAMPLING_FREQ_MASK;

                if (sr_bits & A2DP_LDAC_SAMPLING_FREQ_44100) sr = 44100;

                else if (sr_bits & A2DP_LDAC_SAMPLING_FREQ_48000) sr = 48000;

                else if (sr_bits & A2DP_LDAC_SAMPLING_FREQ_88200) sr = 88200;

                else if (sr_bits & A2DP_LDAC_SAMPLING_FREQ_96000) sr = 96000;

                else if (sr_bits & A2DP_LDAC_SAMPLING_FREQ_176400) sr = 176400;

                else if (sr_bits & A2DP_LDAC_SAMPLING_FREQ_192000) sr = 192000;

                uint8_t ch_bits = raw[7] & A2DP_LDAC_CHANNEL_MODE_MASK;

                if (ch_bits == A2DP_LDAC_CHANNEL_MODE_MONO) ch = 1;

                // LDACBT decoder outputs LDACBT_SMPL_FMT_S24 (24-bit)

                bits = 24;

                ESP_LOGI(BT_AV_TAG, "Detected LDAC codec sr=%u ch=%u", sr, ch);

            } else if (vendorId == 0x0000004F && codecId == 0x0001) {

                // aptX Classic: aptx_decode32 outputs 24-bit in 32-bit LE containers

                uint8_t sr_ch = raw[6];

                if (sr_ch & 0x20) sr = 44100;

                else if (sr_ch & 0x10) sr = 48000;

                if ((sr_ch & 0x0F) == 0x01) ch = 1;

                bits = 32;

                ESP_LOGI(BT_AV_TAG, "Detected aptX codec sr=%u ch=%u", sr, ch);

            } else if (vendorId == 0x000000D7 && codecId == 0x0024) {

                // aptX-HD: aptx_decode32 outputs 24-bit in 32-bit LE containers

                uint8_t sr_ch = raw[6];

                if (sr_ch & 0x20) sr = 44100;

                else if (sr_ch & 0x10) sr = 48000;

                if ((sr_ch & 0x0F) == 0x01) ch = 1;

                bits = 32;

                ESP_LOGI(BT_AV_TAG, "Detected aptX-HD codec sr=%u ch=%u", sr, ch);

            } else if (vendorId == 0x0000000A && codecId == 0x0002) {

                // aptX-LL

                bits = 32;

                sr = 48000;

                ESP_LOGI(BT_AV_TAG, "Detected aptX-LL codec sr=%u ch=%u", sr, ch);

            } else if (vendorId == A2DP_OPUS_VENDOR_ID && codecId == A2DP_OPUS_CODEC_ID) {

                // A2DP Opus vendor codec. The ESP-IDF Opus decoder path outputs

                // interleaved opus_int16 PCM at 48 kHz. In the CIE, raw[6] is

                // channel count and raw[7] is coupled stream count.

                sr = 48000;

                bits = 16;

                ch = raw[6];

                if (ch == 0 || ch > 2) ch = 2;

                ESP_LOGI(BT_AV_TAG, "Detected Opus codec sr=%u bits=%u ch=%u coupled=%u",

                         sr, bits, ch, raw[7]);

            } else {

                sr = 48000; bits = 32;

                ESP_LOGI(BT_AV_TAG, "Unknown vendor codec vendorId=0x%08X codecId=0x%04X", vendorId, codecId);

            }

        }

        m_sample_rate = sr;

        ESP_LOGI(BT_AV_TAG, "codec cfg sr=%u bits=%u ch=%u type=%d", sr, bits, ch, audio_type);

        if (codec_config_cb) codec_config_cb(sr, bits, ch);

        break;

    }



    case ESP_A2D_SNK_PSC_CFG_EVT: {

        esp_a2d_psc_t psc_mask =

            a2d->a2d_psc_cfg_stat.psc_mask;



        ESP_LOGW(

            BT_AV_TAG,

            "BDK2.3.3 PSC mask=0x%04X delay_report=%s",

            (unsigned)psc_mask,

            (psc_mask & ESP_A2D_PSC_DELAY_RPT) ? "SIM" : "NAO");



        break;

    }



    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {

        // BDK V2.5.2 AUTO CONNECT + OTA: apenas instrumentacao. Mantemos exatamente o

        // comportamento anterior para nao mudar duas variaveis no mesmo teste.

        uint16_t current_delay =

            a2d->a2d_get_delay_value_stat.delay_value;



        uint16_t requested_delay =

            (uint16_t)(current_delay + 1500);



        ESP_LOGW(

            BT_AV_TAG,

            "BDK2.3.3 DELAY get=%u (0.1ms) -> set=%u (0.1ms)",

            (unsigned)current_delay,

            (unsigned)requested_delay);



        esp_err_t delay_err =

            esp_a2d_sink_set_delay_value(requested_delay);



        ESP_LOGI(

            BT_AV_TAG,

            "BDK2.3.3 DELAY set call: %s",

            esp_err_to_name(delay_err));



        break;

    }

    default: break;

    }

}



void NativeA2DPSink::data_cb(const uint8_t *data, uint32_t len) {

    if (raw_stream_reader) raw_stream_reader(data, len);

    if (stream_reader) stream_reader(data, len);

    if (data_received) data_received();

}



void NativeA2DPSink::rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {

    app_work_dispatch([](uint16_t e, void *p){ if(instance) instance->av_hdl_avrc_ct_evt(e,p); }, event, param, sizeof(esp_avrc_ct_cb_param_t));

}



void NativeA2DPSink::av_hdl_avrc_ct_evt(uint16_t event, void *p_param) {

    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t*)p_param;



    switch (event) {

    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {

        bool connected = rc->conn_stat.connected;

        avrc_connection_state = connected;



        ESP_LOGI(BT_RC_CT_TAG, "AVRCP CT connected=%d", connected ? 1 : 0);

        bdkNativeAvrcConnection(connected);



        if (connected) {

            esp_avrc_ct_send_get_rn_capabilities_cmd(0);

        } else {

            s_avrc_peer_rn_cap.bits = 0;

        }

        break;

    }



    case ESP_AVRC_CT_METADATA_RSP_EVT:

        ESP_LOGI(BT_RC_CT_TAG,

                 "metadata attr=0x%02X len=%u",

                 (unsigned)rc->meta_rsp.attr_id,

                 (unsigned)rc->meta_rsp.attr_length);

        bdkNativeAvrcMetadata(

            (uint8_t)rc->meta_rsp.attr_id,

            rc->meta_rsp.attr_text,

            rc->meta_rsp.attr_length);

        break;



    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {

        uint8_t id = rc->change_ntf.event_id;

        av_notify_evt_handler(id, &rc->change_ntf.event_parameter);



        if (id == ESP_AVRC_RN_TRACK_CHANGE) {

            // Android metadata may arrive a little after the notification.

            // First request immediately; BDK XM V1.0.7.9 debounces/applies metadata.

            bdkRequestMetadata();

        }

        break;

    }



    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:

        s_avrc_peer_rn_cap.bits = rc->get_rn_caps_rsp.evt_set.bits;

        av_new_track();

        av_playback_changed();

        av_play_pos_changed();

        bdkRequestMetadata();

        break;



    default:

        break;

    }

}





void NativeA2DPSink::rc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param) {

    app_work_dispatch([](uint16_t e, void *p){ if(instance) instance->av_hdl_avrc_tg_evt(e,p); }, event, param, sizeof(esp_avrc_tg_cb_param_t));

}



void NativeA2DPSink::av_hdl_avrc_tg_evt(uint16_t event, void *p_param) {

    esp_avrc_tg_cb_param_t *rc = (esp_avrc_tg_cb_param_t*)p_param;

    switch (event) {

    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:

        volume_set_by_controller(rc->set_abs_vol.volume);

        break;

    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:

        if (rc->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {

            s_volume_notify = true;

            esp_avrc_rn_param_t rn_param; rn_param.volume = s_volume;

            esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn_param);

        }

        break;

    default: break;

    }

}



void NativeA2DPSink::execute_avrc_command(int cmd) {

    esp_avrc_ct_send_passthrough_cmd(0, (uint8_t)cmd, ESP_AVRC_PT_CMD_STATE_PRESSED);

    vTaskDelay(pdMS_TO_TICKS(100));

    esp_avrc_ct_send_passthrough_cmd(0, (uint8_t)cmd, ESP_AVRC_PT_CMD_STATE_RELEASED);

}



void NativeA2DPSink::play()        { execute_avrc_command(ESP_AVRC_PT_CMD_PLAY); }

void NativeA2DPSink::pause()       { execute_avrc_command(ESP_AVRC_PT_CMD_PAUSE); }

void NativeA2DPSink::stop()        { execute_avrc_command(ESP_AVRC_PT_CMD_STOP); }

void NativeA2DPSink::next()        { execute_avrc_command(ESP_AVRC_PT_CMD_FORWARD); }

void NativeA2DPSink::previous()    { execute_avrc_command(ESP_AVRC_PT_CMD_BACKWARD); }

void NativeA2DPSink::fast_forward(){ execute_avrc_command(ESP_AVRC_PT_CMD_FAST_FORWARD); }

void NativeA2DPSink::rewind()      { execute_avrc_command(ESP_AVRC_PT_CMD_REWIND); }



void NativeA2DPSink::volume_up()   { volume_set_by_local_host((s_volume + 5) & 0x7f); }

void NativeA2DPSink::volume_down() { volume_set_by_local_host(s_volume > 5 ? s_volume - 5 : 0); }

void NativeA2DPSink::set_volume(uint8_t v) { volume_set_by_local_host(v & 0x7f); }

int  NativeA2DPSink::get_volume()  { _lock_acquire(&s_volume_lock); int v=s_volume; _lock_release(&s_volume_lock); return v; }



void NativeA2DPSink::volume_set_by_controller(uint8_t volume) {

    _lock_acquire(&s_volume_lock); s_volume = volume; _lock_release(&s_volume_lock);

    if (volumechange_cb) volumechange_cb(volume);

    if (s_volume_notify) {

        esp_avrc_rn_param_t rn_param; rn_param.volume = s_volume;

        esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_CHANGED, &rn_param);

        s_volume_notify = false;

    }

    if (volumechange_completed_cb) volumechange_completed_cb(volume);

}



void NativeA2DPSink::volume_set_by_local_host(uint8_t volume) {

    _lock_acquire(&s_volume_lock); s_volume = volume; _lock_release(&s_volume_lock);

    if (volumechange_cb) volumechange_cb(volume);

    if (s_volume_notify) {

        esp_avrc_rn_param_t rn_param; rn_param.volume = s_volume;

        esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_CHANGED, &rn_param);

        s_volume_notify = false;

    }

}



void NativeA2DPSink::av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter) {

    switch (event_id) {

    case ESP_AVRC_RN_TRACK_CHANGE: av_new_track(); break;

    case ESP_AVRC_RN_PLAY_STATUS_CHANGE: av_playback_changed(); break;

    case ESP_AVRC_RN_PLAY_POS_CHANGED: av_play_pos_changed(); break;

    case ESP_AVRC_RN_VOLUME_CHANGE:

        volume_set_by_controller(event_parameter->volume);

        break;

    default: break;

    }

}



void NativeA2DPSink::av_new_track() {

    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap, ESP_AVRC_RN_TRACK_CHANGE))

        esp_avrc_ct_send_register_notification_cmd(2, ESP_AVRC_RN_TRACK_CHANGE, 0);

}

void NativeA2DPSink::av_playback_changed() {

    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap, ESP_AVRC_RN_PLAY_STATUS_CHANGE))

        esp_avrc_ct_send_register_notification_cmd(3, ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);

}

void NativeA2DPSink::av_play_pos_changed() {

    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap, ESP_AVRC_RN_PLAY_POS_CHANGED))

        esp_avrc_ct_send_register_notification_cmd(4, ESP_AVRC_RN_PLAY_POS_CHANGED, 10);

}



void NativeA2DPSink::set_discoverability(esp_bt_discovery_mode_t mode) {

    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, (mode==ESP_BT_NON_DISCOVERABLE)?ESP_BT_NON_DISCOVERABLE:mode);

}



void NativeA2DPSink::set_scan_mode_connectable(bool connectable) {

    esp_bt_gap_set_scan_mode(connectable?ESP_BT_CONNECTABLE:ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

}



esp_a2d_connection_state_t NativeA2DPSink::get_connection_state() { return connection_state; }

void NativeA2DPSink::disconnect() { esp_a2d_sink_disconnect(peer_bd_addr); }

void NativeA2DPSink::set_auto_reconnect(bool reconnect, int count) {

    reconnect_status = reconnect ? AutoReconnect : NoReconnect;
    try_reconnect_max_count = count;
    is_autoreconnect_allowed = reconnect;

    s_bdk1811ReconnectState =
        reconnect ? BDK1811_AUTO_RECONNECT : BDK1811_NO_RECONNECT;

    if (!reconnect) {
        s_bdk1811ConnectionRetryCount = 0;
        s_bdk1811ReconnectTimeoutMs = 0;
    }

    ESP_LOGI(
        TAG,
        "V1.0.7.12 ACL_DIAG set_auto_reconnect=%d count=%d state=%d",
        reconnect ? 1 : 0,
        count,
        (int)s_bdk1811ReconnectState);
}

void NativeA2DPSink::set_task_core(int core) { task_core = core; }

void NativeA2DPSink::set_output_active(bool active) { is_i2s_active = active; }

esp_bd_addr_t* NativeA2DPSink::get_current_peer_address() { return &peer_bd_addr; }



bool NativeA2DPSink::has_last_connection() {

    nvs_handle_t handle;

    if (nvs_open("a2dp", NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    uint8_t bda[ESP_BD_ADDR_LEN] = {};
    size_t len = ESP_BD_ADDR_LEN;

    esp_err_t err =
        nvs_get_blob(handle, "last_bda", bda, &len);

    nvs_close(handle);

    return (
        err == ESP_OK &&
        len == ESP_BD_ADDR_LEN &&
        !bdkBdaIsZero(bda));
}

void NativeA2DPSink::get_last_connection() {

    nvs_handle_t handle;

    esp_err_t openErr =
        nvs_open("a2dp", NVS_READONLY, &handle);

    if (openErr != ESP_OK) {
        ESP_LOGW(
            BT_AV_TAG,
            "BDK XM V1.0.7.9 AUTO: NVS read open = %s",
            esp_err_to_name(openErr));
        return;
    }

    uint8_t bda[ESP_BD_ADDR_LEN] = {};
    size_t len = ESP_BD_ADDR_LEN;

    esp_err_t err =
        nvs_get_blob(handle, "last_bda", bda, &len);

    nvs_close(handle);

    if (
        err == ESP_OK &&
        len == ESP_BD_ADDR_LEN &&
        !bdkBdaIsZero(bda))
    {
        memcpy(last_connection, bda, ESP_BD_ADDR_LEN);
        memcpy(peer_bd_addr, bda, ESP_BD_ADDR_LEN);

        ESP_LOGI(
            BT_AV_TAG,
            "BDK XM V1.0.7.9 AUTO: NVS peer "
            "%02x:%02x:%02x:%02x:%02x:%02x",
            bda[0], bda[1], bda[2],
            bda[3], bda[4], bda[5]);
    }
    else
    {
        ESP_LOGW(
            BT_AV_TAG,
            "BDK XM V1.0.7.9 AUTO: last_bda ausente/invalido (%s len=%u)",
            esp_err_to_name(err),
            (unsigned)len);
    }
}

void NativeA2DPSink::set_last_connection(esp_bd_addr_t bda) {

    if (bdkBdaIsZero(bda)) {
        ESP_LOGW(
            BT_AV_TAG,
            "BDK XM V1.0.7.9 AUTO: recusado salvar BDA zerado");
        return;
    }

    nvs_handle_t handle;

    esp_err_t openErr =
        nvs_open("a2dp", NVS_READWRITE, &handle);

    if (openErr != ESP_OK) {
        ESP_LOGE(
            BT_AV_TAG,
            "BDK XM V1.0.7.9 AUTO: NVS write open = %s",
            esp_err_to_name(openErr));
        return;
    }

    esp_err_t setErr =
        nvs_set_blob(handle, "last_bda", bda, ESP_BD_ADDR_LEN);

    esp_err_t commitErr =
        (setErr == ESP_OK) ? nvs_commit(handle) : setErr;

    nvs_close(handle);

    if (setErr == ESP_OK && commitErr == ESP_OK) {
        memcpy(last_connection, bda, ESP_BD_ADDR_LEN);
        memcpy(peer_bd_addr, bda, ESP_BD_ADDR_LEN);

        ESP_LOGI(
            BT_AV_TAG,
            "BDK XM V1.0.7.9 AUTO: last_bda salvo "
            "%02x:%02x:%02x:%02x:%02x:%02x",
            bda[0], bda[1], bda[2],
            bda[3], bda[4], bda[5]);
    }
    else
    {
        ESP_LOGE(
            BT_AV_TAG,
            "BDK XM V1.0.7.9 AUTO: falha NVS set=%s commit=%s",
            esp_err_to_name(setErr),
            esp_err_to_name(commitErr));
    }
}



// Callback setters

void NativeA2DPSink::set_stream_reader(a2dp_stream_reader_cb cb, bool is_i2s) { stream_reader = cb; is_i2s_active = is_i2s; }

void NativeA2DPSink::set_raw_stream_reader(a2dp_stream_reader_cb cb) { raw_stream_reader = cb; }

void NativeA2DPSink::set_on_data_received(a2dp_data_received_cb cb) { data_received = cb; }

void NativeA2DPSink::set_codec_config_callback(a2dp_codec_config_cb cb) { codec_config_cb = cb; }

void NativeA2DPSink::set_on_connection_state_changed(a2dp_connection_state_cb cb) { connection_state_cb = cb; }

void NativeA2DPSink::set_on_audio_state_changed(a2dp_audio_state_cb cb) { audio_state_cb = cb; }

void NativeA2DPSink::set_avrc_rn_volumechange(a2dp_volumechange_cb cb) { volumechange_cb = cb; }

void NativeA2DPSink::set_avrc_rn_volumechange_completed(a2dp_volumechange_cb cb) { volumechange_completed_cb = cb; }

esp_a2d_mct_t NativeA2DPSink::get_audio_type() { return audio_type; }

