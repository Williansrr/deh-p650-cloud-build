/*
 * ============================================================
 * BDK V2.3.3 HQ HARMONIC DETAIL + AUTO CONNECT
 * LDAC ate 96 kHz / PCM 24-bit -> DSP -> PCM5102 I2S
 * ============================================================
 *
 * Base: BDK V2.1 validada em hardware.
 *
 * Experimental:
 *   - LDAC anuncia 44.1/48/88.2/96 kHz e prefere 96 kHz.
 *   - Contador de payload LDAC antes do decoder para estimar bitrate.
 *   - DSP psychoacoustic bass enhancement / virtual bass.
 *   - Grave real moderado + harmonicos 2o/3o para simular subgrave.
 *   - Headroom digital de -4 dB e soft limiter.
 *   - IP-BUS XM V3.26 nativo: LINK 0x11E / APP 0x1E.
 *
 * PCM5102:
 *   GPIO25 -> BCK/BCLK
 *   GPIO32 -> LCK/LRCK/WS
 *   GPIO33 -> DIN/DATA
 *   GND    -> GND
 *   5V     -> VIN/VCC
 *   SCK    -> GND
 *   XSMT   -> A3V3
 * ============================================================
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "ota/bt_ota.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"

#include "driver/i2s_std.h"
#include "driver/i2s_common.h"

#include "bt/a2dp_sink_native.h"
#include "ipbus/ipbus_xm_v326.h"
#include "dsp/bdk_bass_dsp.h"

static const char* TAG = "BDK_XM_V10";
static const char* FW_VERSION  = "BDK XM V1.0.8.5 DIAG / SWAP AUDIO XM1-XM2";
static const char* DEVICE_NAME = "DEH-P650 WRD";

// ------------------------------------------------------------
// I2S / PCM5102
// ------------------------------------------------------------
static constexpr gpio_num_t PIN_BCK  = GPIO_NUM_25;
static constexpr gpio_num_t PIN_LRCK = GPIO_NUM_32;
static constexpr gpio_num_t PIN_DATA = GPIO_NUM_33;
static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

static i2s_chan_handle_t g_i2sTx = nullptr;
static volatile bool g_i2sReady = false;
static volatile bool g_i2sReconfiguring = false;
static volatile uint32_t g_i2sSampleRate = 48000;

// ------------------------------------------------------------
// Ring buffer: 2048 frames stereo x 8 bytes = 16 KB
// ------------------------------------------------------------
static constexpr uint32_t RING_FRAMES = 4096;
static constexpr uint32_t PREBUFFER_FRAMES = 2048;
static constexpr uint32_t RENDER_FRAMES = 256;
static constexpr uint32_t HIGH_WATERMARK_FRAMES = 3072;

static int32_t* g_ring = nullptr;
static volatile uint32_t g_ringWrite = 0;
static volatile uint32_t g_ringRead  = 0;
static int32_t g_render[RENDER_FRAMES * 2];


// ------------------------------------------------------------
// BDK V1.0.8.3 - HIFI STAGE CORSA / TIME ALIGNMENT XM1+XM2
// Estimativa inicial Corsa B 2 portas, posicao do motorista:
// lado esquerdo ~53 cm mais proximo -> atraso de 1,55 ms no L.
// O atraso e recalculado conforme 44.1/48/88.2/96 kHz.
// ------------------------------------------------------------
static constexpr float STAGE_LEFT_DELAY_MS = 1.55f;
static constexpr uint32_t STAGE_DELAY_MAX_FRAMES = 192;
static int32_t g_stageDelayL[STAGE_DELAY_MAX_FRAMES] = {};
static uint32_t g_stageDelayWrite = 0;
static uint32_t g_stageDelayFrames = 0;
static bool g_stageDelayActive = false;

static void resetStageDelay()
{
    memset(g_stageDelayL, 0, sizeof(g_stageDelayL));
    g_stageDelayWrite = 0;
    g_stageDelayFrames = 0;
    g_stageDelayActive = false;
}

static uint32_t stageDelayFramesForRate(uint32_t rate)
{
    if (!rate) rate = 48000;
    uint32_t frames = (uint32_t)((((float)rate * STAGE_LEFT_DELAY_MS) / 1000.0f) + 0.5f);
    if (frames < 1) frames = 1;
    if (frames >= STAGE_DELAY_MAX_FRAMES) frames = STAGE_DELAY_MAX_FRAMES - 1;
    return frames;
}

static void applyStageDelay(int32_t *stereo, uint32_t frames, bool want)
{
    if (!want) {
        if (g_stageDelayActive) resetStageDelay();
        return;
    }

    const uint32_t wantedFrames = stageDelayFramesForRate(g_i2sSampleRate);
    if (!g_stageDelayActive || g_stageDelayFrames != wantedFrames) {
        memset(g_stageDelayL, 0, sizeof(g_stageDelayL));
        g_stageDelayWrite = 0;
        g_stageDelayFrames = wantedFrames;
        g_stageDelayActive = true;
        ESP_LOGW(TAG, "HIFI STAGE: delay L=%.2f ms / %u frames @ %u Hz",
                 (double)STAGE_LEFT_DELAY_MS,
                 (unsigned)g_stageDelayFrames,
                 (unsigned)g_i2sSampleRate);
    }

    for (uint32_t i = 0; i < frames; ++i) {
        const uint32_t read =
            (g_stageDelayWrite + STAGE_DELAY_MAX_FRAMES - g_stageDelayFrames) %
            STAGE_DELAY_MAX_FRAMES;
        const int32_t inL = stereo[i * 2];
        const int32_t delayedL = g_stageDelayL[read];
        g_stageDelayL[g_stageDelayWrite] = inL;
        g_stageDelayWrite = (g_stageDelayWrite + 1) % STAGE_DELAY_MAX_FRAMES;
        stereo[i * 2] = delayedL;
    }
}

// ------------------------------------------------------------
// A2DP / estado
// ------------------------------------------------------------
static NativeA2DPSink g_a2dp;
static BDKBassDSP g_dsp;


// ============================================================
// BDK XM V1.0.7.4 - XM BAND -> AUDIO PROFILE
// ============================================================
// DIAG: XM1 usa temporariamente o perfil limpo do XM2
// DIAG: XM2 usa temporariamente o BBR6 do XM1
// XM3 = Hi-Fi maximum transparency: DSP bypass
//
// The IP-BUS task only requests a profile. The audio render task
// applies the change between PCM blocks to avoid mutating filter
// state concurrently with processBlock().
// ============================================================

static volatile uint8_t g_requestedXmBand = 1;
static volatile bool g_profileChangePending = true;

static BDKAudioProfile profileFromXmBand(uint8_t band)
{
    // DIAGNOSTICO CRUZADO:
    // XM1 do radio -> perfil de audio originalmente usado pelo XM2 (limpo)
    // XM2 do radio -> perfil BBR6 originalmente usado pelo XM1 (suspeito)
    // XM3 do radio -> HIFI PURE BYPASS, sem alteracao
    switch (band) {
        case 1: return BDK_AUDIO_PROFILE_XTREME4;
        case 2: return BDK_AUDIO_PROFILE_BOOMBOX3;
        case 3: return BDK_AUDIO_PROFILE_HIFI;
        default:
            return BDK_AUDIO_PROFILE_XTREME4;
    }
}

extern "C" void bdkXmBandChanged(uint8_t band)
{
    if (band < 1 || band > 3) band = 1;
    g_requestedXmBand = band;
    g_profileChangePending = true;
}

static void applyRequestedXmProfile()
{
    if (!g_profileChangePending) return;

    const uint8_t band = g_requestedXmBand;
    g_profileChangePending = false;

    const BDKAudioProfile profile = profileFromXmBand(band);
    g_dsp.setProfile(profile);

    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG, "==============================================");
    ESP_LOGW(TAG, " XM%u -> PERFIL DE AUDIO", (unsigned)band);
    ESP_LOGW(TAG, " %s", g_dsp.profileName());
    if (profile == BDK_AUDIO_PROFILE_HIFI) {
        ESP_LOGW(TAG, " DSP: BYPASS / PCM LDAC preservado sem voicing");
    } else {
        ESP_LOGW(TAG, " DSP: V2.4 JBL Profiles BBR6");
    }
    ESP_LOGW(TAG, "==============================================");
}

// Instrumentacao adicionada pelo patch LDAC HQ.
// weak permite que o main continue linkavel mesmo se a instrumentacao
// for removida durante um teste de stack.
extern "C" uint32_t bdk_ldac_encoded_bytes_total(void) __attribute__((weak));
extern "C" uint32_t bdk_ldac_encoded_packets_total(void) __attribute__((weak));

static uint32_t ldacEncodedBytesTotal()
{
    return bdk_ldac_encoded_bytes_total ? bdk_ldac_encoded_bytes_total() : 0;
}

static uint32_t ldacEncodedPacketsTotal()
{
    return bdk_ldac_encoded_packets_total ? bdk_ldac_encoded_packets_total() : 0;
}


// ============================================================
// BDK V2.2 - AVRCP <-> IP-BUS BRIDGE
// ============================================================

static volatile bool g_avrcConnected = false;

extern "C" void bdkNativeAvrcConnection(bool connected)
{
    g_avrcConnected = connected;
    ipbusXmAvrcConnection(connected);
}

extern "C" void bdkNativeAvrcMetadata(
    uint8_t attr,
    const uint8_t *data,
    size_t len)
{
    ipbusXmAvrcMetadata(attr, data, len);
}

extern "C" bool bdkAvrcConnected(void)
{
    return g_avrcConnected;
}

extern "C" void bdkAvrcNext(void)
{
    if (g_avrcConnected) g_a2dp.next();
}

extern "C" void bdkAvrcPrevious(void)
{
    if (g_avrcConnected) g_a2dp.previous();
}

extern "C" void bdkAvrcPlay(void)
{
    if (g_avrcConnected) g_a2dp.play();
}
static volatile uint32_t g_sampleRate = 0;
static volatile uint8_t  g_bits = 0;
static volatile uint8_t  g_channels = 0;
static volatile bool g_connected = false;
static volatile bool g_audioStarted = false;
static volatile esp_a2d_connection_state_t g_connectionState = ESP_A2D_CONNECTION_STATE_DISCONNECTED;

// V1.0.8.0 - supervisor unico de reconnect, baseado no standalone V1.3 validado.
// O reconnect interno da NativeA2DPSink permanece DESLIGADO para evitar tentativas duplicadas.
static esp_bd_addr_t g_lastBda = {0};
static volatile bool g_haveLastBda = false;
static TaskHandle_t g_reconnectTask = nullptr;

// Telemetria
static volatile uint32_t g_btBytes = 0;
static volatile uint32_t g_btChunks = 0;
static volatile uint32_t g_i2sBytes = 0;
static volatile uint32_t g_i2sFrames = 0;
static volatile uint32_t g_drops = 0;
static volatile uint32_t g_underruns = 0;
static volatile uint32_t g_writeErrors = 0;

static void logHeap(const char* stage)
{
    size_t freeInternal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t minimum = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    ESP_LOGI(TAG, "HEAP %-18s free=%u KB largest=%u KB min=%u KB",
             stage,
             (unsigned)(freeInternal / 1024),
             (unsigned)(largest / 1024),
             (unsigned)(minimum / 1024));
}

static inline uint32_t counterDelta(uint32_t current, uint32_t previous)
{
    // DSP stats reset intentionally when XM band/profile changes.
    return (current >= previous) ? (current - previous) : current;
}

// ------------------------------------------------------------
// Ring
// ------------------------------------------------------------
static inline uint32_t ringNext(uint32_t i)
{
    ++i;
    return (i >= RING_FRAMES) ? 0 : i;
}

static uint32_t ringFill()
{
    uint32_t w = g_ringWrite;
    uint32_t r = g_ringRead;
    return (w >= r) ? (w - r) : (RING_FRAMES - r + w);
}

static uint32_t ringFree()
{
    return (RING_FRAMES - 1) - ringFill();
}

static void clearRing()
{
    g_ringRead = g_ringWrite;
}

static bool pushFrame(int32_t l, int32_t r)
{
    if (!g_ring) return false;

    uint32_t w = g_ringWrite;
    uint32_t n = ringNext(w);
    if (n == g_ringRead) return false;

    uint32_t p = w * 2;
    g_ring[p] = l;
    g_ring[p + 1] = r;
    g_ringWrite = n;
    return true;
}

static bool popFrame(int32_t& l, int32_t& r)
{
    if (!g_ring) return false;

    uint32_t rd = g_ringRead;
    if (rd == g_ringWrite) return false;

    uint32_t p = rd * 2;
    l = g_ring[p];
    r = g_ring[p + 1];
    g_ringRead = ringNext(rd);
    return true;
}

// ------------------------------------------------------------
// PCM packed -> Q1.31
// Mesma conversao usada pelo pipeline original do projeto.
// ------------------------------------------------------------
static inline int32_t loadQ31(const uint8_t* p, uint32_t bytesPerSample)
{
    if (bytesPerSample == 2) {
        int16_t v = (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
        return ((int32_t)v) << 16;
    }

    if (bytesPerSample == 3) {
        int32_t v = ((int32_t)p[0]) |
                    ((int32_t)p[1] << 8) |
                    ((int32_t)p[2] << 16);
        v = (v << 8) >> 8;   // sign extend 24 -> 32
        return v << 8;        // 24 bits validos no topo do slot 32-bit
    }

    uint32_t v = ((uint32_t)p[0]) |
                 ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) |
                 ((uint32_t)p[3] << 24);
    return (int32_t)v;
}

// ------------------------------------------------------------
// I2S
// ------------------------------------------------------------
static i2s_std_clk_config_t makeClock(uint32_t rate, bool apll)
{
    i2s_std_clk_config_t c = I2S_STD_CLK_DEFAULT_CONFIG(rate);
    if (apll) c.clk_src = I2S_CLK_SRC_APLL;
    c.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    return c;
}

static bool initI2S(uint32_t rate)
{
    if (g_i2sReady) return true;

    ESP_LOGI(TAG, "PCM5102 I2S: BCK=25 LRCK=32 DATA=33, %u Hz, 32-bit stereo", (unsigned)rate);

    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
    chan.auto_clear = true;
    chan.dma_desc_num = 6;
    chan.dma_frame_num = 128;

    esp_err_t err = i2s_new_channel(&chan, &g_i2sTx, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel: %s", esp_err_to_name(err));
        g_i2sTx = nullptr;
        return false;
    }

    i2s_std_clk_config_t clk = makeClock(rate, true);
    i2s_std_config_t cfg = {
        .clk_cfg = clk,
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_BCK,
            .ws   = PIN_LRCK,
            .dout = PIN_DATA,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    bool usingApll = true;
    err = i2s_channel_init_std_mode(g_i2sTx, &cfg);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "APLL init falhou: %s; tentando clock padrao", esp_err_to_name(err));
        cfg.clk_cfg = makeClock(rate, false);
        usingApll = false;
        err = i2s_channel_init_std_mode(g_i2sTx, &cfg);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode: %s", esp_err_to_name(err));
        i2s_del_channel(g_i2sTx);
        g_i2sTx = nullptr;
        return false;
    }

    err = i2s_channel_enable(g_i2sTx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable: %s", esp_err_to_name(err));
        i2s_del_channel(g_i2sTx);
        g_i2sTx = nullptr;
        return false;
    }

    g_i2sSampleRate = rate;
    g_i2sReady = true;

    ESP_LOGI(TAG, "I2S ATIVO: %u Hz / 32-bit stereo / %s",
             (unsigned)rate, usingApll ? "APLL" : "clock padrao");

    int32_t silence[64 * 2] = {0};
    size_t written = 0;
    i2s_channel_write(g_i2sTx, silence, sizeof(silence), &written, pdMS_TO_TICKS(20));

    return true;
}

static bool setI2SRate(uint32_t rate)
{
    if (!g_i2sReady || !g_i2sTx || rate == 0) return false;
    if (g_i2sSampleRate == rate) return true;

    g_i2sReconfiguring = true;
    clearRing();

    ESP_LOGI(TAG, "I2S clock: %u -> %u Hz", (unsigned)g_i2sSampleRate, (unsigned)rate);

    esp_err_t dis = i2s_channel_disable(g_i2sTx);
    if (dis != ESP_OK) ESP_LOGW(TAG, "I2S disable: %s", esp_err_to_name(dis));

    i2s_std_clk_config_t clk = makeClock(rate, true);
    esp_err_t err = i2s_channel_reconfig_std_clock(g_i2sTx, &clk);
    bool usingApll = true;

    if (err != ESP_OK) {
        clk = makeClock(rate, false);
        usingApll = false;
        err = i2s_channel_reconfig_std_clock(g_i2sTx, &clk);
    }

    esp_err_t en = i2s_channel_enable(g_i2sTx);

    if (err == ESP_OK && en == ESP_OK) {
        g_i2sSampleRate = rate;
        ESP_LOGI(TAG, "I2S clock OK: %u Hz / %s", (unsigned)rate,
                 usingApll ? "APLL" : "clock padrao");
    } else {
        ESP_LOGE(TAG, "I2S reconfig: clock=%s enable=%s",
                 esp_err_to_name(err), esp_err_to_name(en));
    }

    g_i2sReconfiguring = false;
    return (err == ESP_OK && en == ESP_OK);
}

// ------------------------------------------------------------
// Codec
// ------------------------------------------------------------
static const char* codecName(esp_a2d_mct_t type)
{
    switch (type) {
        case ESP_A2D_MCT_SBC:      return "SBC";
        case ESP_A2D_MCT_M12:      return "MPEG-1/2";
        case ESP_A2D_MCT_M24:      return "AAC";
        case ESP_A2D_MCT_ATRAC:    return "ATRAC";
        case ESP_A2D_MCT_NON_A2DP: return "VENDOR / LDAC";
        default:                    return "UNKNOWN";
    }
}

static void onCodecConfig(uint32_t rate, uint8_t bits, uint8_t channels)
{
    g_sampleRate = rate;
    g_bits = bits;
    g_channels = channels;

    esp_a2d_mct_t type = g_a2dp.get_audio_type();

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, " CODEC CONFIGURATION RECEIVED");
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "Codec      : %s", codecName(type));
    ESP_LOGI(TAG, "Codec type : 0x%02X", (unsigned)type);
    ESP_LOGI(TAG, "Sample rate: %u Hz", (unsigned)rate);
    ESP_LOGI(TAG, "Bits       : %u", (unsigned)bits);
    ESP_LOGI(TAG, "Channels   : %u", (unsigned)channels);
    ESP_LOGI(TAG, "==================================================");

    if (g_i2sReady) setI2SRate(rate);
    g_dsp.setSampleRate(rate);
    ESP_LOGI(TAG, "DSP sample rate: %u Hz", (unsigned)g_dsp.sampleRate());
    logHeap("codec_config");
}

// ------------------------------------------------------------
// PCM callback - produtor Core 1
// ------------------------------------------------------------
static void onStreamData(const uint8_t* data, uint32_t len)
{
    if (!data || len == 0 || !g_ring) return;

    g_btBytes += len;
    g_btChunks++;

    uint8_t bits = g_bits;
    uint8_t channels = g_channels;
    if (bits == 0 || channels == 0) return;

    uint32_t bps = (bits <= 16) ? 2u : ((bits <= 24) ? 3u : 4u);
    uint32_t ch = (channels > 2) ? 2u : channels;
    uint32_t bpf = bps * ch;
    if (bpf == 0) return;

    // A2DP backpressure: deixa o render drenar as rajadas do LDAC.
    if (g_audioStarted) {
        uint32_t spins = 0;
        while (ringFill() >= HIGH_WATERMARK_FRAMES && spins < 16) {
            vTaskDelay(pdMS_TO_TICKS(1));
            spins++;
        }
    }

    const uint8_t* p = data;
    uint32_t remaining = len;

    while (remaining >= bpf) {
        int32_t left = loadQ31(p, bps);
        int32_t right = (ch == 1) ? left : loadQ31(p + bps, bps);

        if (!pushFrame(left, right)) g_drops++;

        p += bpf;
        remaining -= bpf;
    }
}

// ------------------------------------------------------------
// V1.0.8.0 - NVS + supervisor unico de auto reconnect
// ------------------------------------------------------------
static bool bdaIsZero(const uint8_t* bda)
{
    if (!bda) return true;
    for (int i = 0; i < ESP_BD_ADDR_LEN; ++i) {
        if (bda[i] != 0) return false;
    }
    return true;
}

static void logBda(const char* prefix, const uint8_t* bda)
{
    if (!bda) return;
    ESP_LOGI(TAG, "%s %02X:%02X:%02X:%02X:%02X:%02X", prefix,
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

static bool loadBdaFromNamespace(const char* ns, esp_bd_addr_t out)
{
    nvs_handle_t h;
    if (nvs_open(ns, NVS_READONLY, &h) != ESP_OK) return false;
    size_t len = ESP_BD_ADDR_LEN;
    esp_err_t err = nvs_get_blob(h, "last_bda", out, &len);
    nvs_close(h);
    return err == ESP_OK && len == ESP_BD_ADDR_LEN && !bdaIsZero(out);
}

static bool loadLastReconnectBda()
{
    esp_bd_addr_t tmp = {0};

    // Namespace proprio da V1.0.8.0 / standalone V1.3.
    if (loadBdaFromNamespace("a2dp_auto", tmp)) {
        memcpy(g_lastBda, tmp, ESP_BD_ADDR_LEN);
        g_haveLastBda = true;
        logBda("NVS reconnect peer:", g_lastBda);
        return true;
    }

    // Importa automaticamente o bond/peer historico salvo pela NativeA2DPSink.
    if (loadBdaFromNamespace("a2dp", tmp)) {
        memcpy(g_lastBda, tmp, ESP_BD_ADDR_LEN);
        g_haveLastBda = true;
        logBda("Importando peer historico a2dp:", g_lastBda);
        return true;
    }

    ESP_LOGW(TAG, "NVS reconnect peer: vazio; primeiro pareamento sera manual");
    return false;
}

static void saveLastReconnectBda(const uint8_t* bda)
{
    if (!bda || bdaIsZero(bda)) return;

    memcpy(g_lastBda, bda, ESP_BD_ADDR_LEN);
    g_haveLastBda = true;

    nvs_handle_t h;
    esp_err_t err = nvs_open("a2dp_auto", NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_blob(h, "last_bda", g_lastBda, ESP_BD_ADDR_LEN);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }

    if (err == ESP_OK) logBda("MAC salvo para auto reconnect:", g_lastBda);
    else ESP_LOGW(TAG, "Falha salvando last_bda: %s", esp_err_to_name(err));
}

static void autoReconnectTask(void*)
{
    // Exatamente a progressao que funcionou no standalone V1.3.
    static const uint32_t delaysMs[] = {1500, 3000, 6000, 12000, 20000, 30000};
    size_t attempt = 0;

    while (true) {
        if (!g_haveLastBda) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        const esp_a2d_connection_state_t state = g_connectionState;
        if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            attempt = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (state == ESP_A2D_CONNECTION_STATE_CONNECTING ||
            state == ESP_A2D_CONNECTION_STATE_DISCONNECTING) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        const size_t idx = (attempt < (sizeof(delaysMs) / sizeof(delaysMs[0])))
                         ? attempt
                         : (sizeof(delaysMs) / sizeof(delaysMs[0]) - 1);
        const uint32_t waitMs = delaysMs[idx];

        ESP_LOGI(TAG, "AUTO RECONNECT #%u em %u ms", (unsigned)(attempt + 1), (unsigned)waitMs);
        vTaskDelay(pdMS_TO_TICKS(waitMs));

        if (g_connectionState != ESP_A2D_CONNECTION_STATE_DISCONNECTED || !g_haveLastBda) {
            continue;
        }

        logBda("Chamando esp_a2d_sink_connect ->", g_lastBda);
        esp_err_t err = esp_a2d_sink_connect(g_lastBda);
        ESP_LOGI(TAG, "esp_a2d_sink_connect: %s", esp_err_to_name(err));

        if (attempt < 1000) ++attempt;

        // A chamada e assincrona; nao iniciar outra tentativa imediatamente.
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}

// ------------------------------------------------------------
// Connection / audio state
// ------------------------------------------------------------
static void onConnectionState(esp_a2d_connection_state_t state, void* user)
{
    (void)user;
    g_connectionState = state;

    if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        g_connected = true;
        ESP_LOGI(TAG, ">>> A2DP Connection: CONNECTED");

        esp_bd_addr_t* peer = g_a2dp.get_current_peer_address();
        if (peer && !bdaIsZero(*peer)) saveLastReconnectBda(*peer);

        logHeap("connected");
        return;
    }

    if (state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
        ESP_LOGI(TAG, ">>> A2DP Connection: CONNECTING");
        return;
    }

    if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTING) {
        ESP_LOGW(TAG, ">>> A2DP Connection: DISCONNECTING");
        return;
    }

    if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        g_connected = false;
        g_audioStarted = false;
        clearRing();
        ESP_LOGW(TAG, ">>> A2DP Connection: DISCONNECTED");
        logHeap("disconnected");
        return;
    }

    ESP_LOGW(TAG, ">>> A2DP Connection: estado=%d", (int)state);
}

static void onAudioState(esp_a2d_audio_state_t state, void* user)
{
    (void)user;

    if (state == ESP_A2D_AUDIO_STATE_STARTED) {
        g_audioStarted = true;

        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "**********************************************");
        ESP_LOGI(TAG, " A2DP AUDIO STATE: STARTED");
        ESP_LOGI(TAG, " PCM IN : %u Hz / %u bit / %u ch",
                 (unsigned)g_sampleRate, (unsigned)g_bits, (unsigned)g_channels);
        ESP_LOGI(TAG, " I2S OUT: %u Hz / 32 bit / 2 ch", (unsigned)g_i2sSampleRate);
        ESP_LOGI(TAG, "**********************************************");
        logHeap("audio_started");
        return;
    }

    if (state == ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND) {
        g_audioStarted = false;
        clearRing();
        ESP_LOGI(TAG, ">>> A2DP Audio State: SUSPEND / STOPPED");
        return;
    }

    g_audioStarted = false;
    clearRing();
    ESP_LOGI(TAG, ">>> A2DP Audio State: %d", (int)state);
}

// ------------------------------------------------------------
// Render Core 0
// ------------------------------------------------------------
static void audioRenderTask(void* arg)
{
    (void)arg;
    bool prebuffer = false;

    ESP_LOGI(TAG, "pcm5102_render iniciado no Core %d", xPortGetCoreID());

    while (true) {
        if (!g_audioStarted || !g_i2sReady || !g_i2sTx || g_i2sReconfiguring) {
            prebuffer = false;
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        uint32_t fill = ringFill();

        if (!prebuffer) {
            if (fill < PREBUFFER_FRAMES) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }

            prebuffer = true;
            ESP_LOGI(TAG, "I2S prebuffer OK: %u frames (~%u ms)",
                     (unsigned)fill,
                     (unsigned)((fill * 1000UL) /
                                (g_i2sSampleRate ? g_i2sSampleRate : 48000)));
        }

        if (fill < RENDER_FRAMES) {
            g_underruns++;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        uint32_t frames = 0;
        for (uint32_t i = 0; i < RENDER_FRAMES; ++i) {
            int32_t l = 0, r = 0;
            if (!popFrame(l, r)) break;
            g_render[i * 2] = l;
            g_render[i * 2 + 1] = r;
            frames++;
        }

        if (frames == 0) continue;

        // Apply XM band/profile only between render blocks.
        applyRequestedXmProfile();

        // XM1: HIFI BBR6 EXTENDED | XM2: HIFI STAGE CORSA | XM3: true bypass.
        g_dsp.processBlock(g_render, frames);

        // XM1/XM2: centraliza o palco atrasando apenas o canal esquerdo.
        applyStageDelay(g_render, frames, (g_requestedXmBand == 1 || g_requestedXmBand == 2));

        size_t bytes = frames * 2u * sizeof(int32_t);
        size_t written = 0;
        esp_err_t err = i2s_channel_write(g_i2sTx, g_render, bytes, &written,
                                          pdMS_TO_TICKS(100));

        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            g_writeErrors++;
            ESP_LOGE(TAG, "I2S write: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (written < bytes) g_writeErrors++;

        g_i2sBytes += (uint32_t)written;
        g_i2sFrames += (uint32_t)(written / (2u * sizeof(int32_t)));
    }
}

// ------------------------------------------------------------
// Telemetria
// ------------------------------------------------------------
static const char* nearestLdacQuality(float kbps, uint32_t rate)
{
    if (kbps < 1.0f) return "SEM DADOS";

    const bool family441 = (rate == 44100 || rate == 88200);
    const float qLow  = family441 ? 303.0f : 330.0f;
    const float qMid  = family441 ? 606.0f : 660.0f;
    const float qHigh = family441 ? 909.0f : 990.0f;

    float dLow  = fabsf(kbps - qLow);
    float dMid  = fabsf(kbps - qMid);
    float dHigh = fabsf(kbps - qHigh);

    if (dHigh <= dMid && dHigh <= dLow) return "HQ / QUALITY";
    if (dMid <= dLow) return "SQ / MEDIUM";
    return "MQ / CONNECTION";
}

static void telemetryTask(void* arg)
{
    (void)arg;

    uint32_t pBtB = 0, pBtC = 0, pI2SB = 0, pI2SF = 0;
    uint32_t pDrop = 0, pUnder = 0, pErr = 0;
    uint32_t pLdacB = ldacEncodedBytesTotal();
    uint32_t pLdacP = ldacEncodedPacketsTotal();
    BDKBassDSPStats pDsp = g_dsp.stats();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        uint32_t btB = g_btBytes;
        uint32_t btC = g_btChunks;
        uint32_t i2sB = g_i2sBytes;
        uint32_t i2sF = g_i2sFrames;
        uint32_t drop = g_drops;
        uint32_t under = g_underruns;
        uint32_t wrErr = g_writeErrors;
        uint32_t ldacB = ldacEncodedBytesTotal();
        uint32_t ldacP = ldacEncodedPacketsTotal();
        BDKBassDSPStats dsp = g_dsp.stats();

        const uint32_t dLdacBytes = ldacB - pLdacB;
        const uint32_t dLdacPackets = ldacP - pLdacP;
        const float ldacKbps = ((float)dLdacBytes * 8.0f) / 5.0f / 1000.0f;

        ESP_LOGI(TAG,
                 "STATUS conn=%d audio=%d codec=%uHz/%ubit/%uch "
                 "BT +%uB/%uC I2S +%uB/%uF ring=%u free=%u "
                 "drop+%u underrun+%u wrerr+%u",
                 g_connected ? 1 : 0,
                 g_audioStarted ? 1 : 0,
                 (unsigned)g_sampleRate,
                 (unsigned)g_bits,
                 (unsigned)g_channels,
                 (unsigned)(btB - pBtB),
                 (unsigned)(btC - pBtC),
                 (unsigned)(i2sB - pI2SB),
                 (unsigned)(i2sF - pI2SF),
                 (unsigned)ringFill(),
                 (unsigned)ringFree(),
                 (unsigned)(drop - pDrop),
                 (unsigned)(under - pUnder),
                 (unsigned)(wrErr - pErr));

        esp_a2d_mct_t type = g_a2dp.get_audio_type();
        if (type == ESP_A2D_MCT_NON_A2DP && g_audioStarted && dLdacBytes > 0) {
            ESP_LOGI(TAG,
                     "LDAC HQ PAYLOAD: ~%.0f kbps / +%u bytes / +%u packets / "
                     "estimativa=%s",
                     (double)ldacKbps,
                     (unsigned)dLdacBytes,
                     (unsigned)dLdacPackets,
                     nearestLdacQuality(ldacKbps, g_sampleRate));
        }

        ESP_LOGI(TAG,
                 "AUDIO PROFILE=%s state=%s sr=%u BASS=%s DETAIL=%s "
                 "harmonic+%u realSub+%u maskCut+%u midTr+%u presTr+%u airTr+%u limiter+%u "
                 "peakIn=%.3f peakOut=%.3f peakHarm=%.3f peakDetail=%.3f",
                 g_dsp.profileName(),
                 g_dsp.isEnabled() ? "DSP" : "BYPASS",
                 (unsigned)g_dsp.sampleRate(),
                 g_dsp.harmonicBassEnabled() ? "ON" : "OFF",
                 g_dsp.detailEnabled() ? "ON" : "OFF",
                 (unsigned)counterDelta(dsp.virtualBassSamples, pDsp.virtualBassSamples),
                 (unsigned)counterDelta(dsp.realBassSamples, pDsp.realBassSamples),
                 (unsigned)counterDelta(dsp.maskReductionSamples, pDsp.maskReductionSamples),
                 (unsigned)counterDelta(dsp.midTransientSamples, pDsp.midTransientSamples),
                 (unsigned)counterDelta(dsp.presenceTransientSamples, pDsp.presenceTransientSamples),
                 (unsigned)counterDelta(dsp.airTransientSamples, pDsp.airTransientSamples),
                 (unsigned)counterDelta(dsp.limiterSamples, pDsp.limiterSamples),
                 (double)dsp.peakInput,
                 (double)dsp.peakOutput,
                 (double)dsp.peakVirtual,
                 (double)dsp.peakDetail);

        pBtB = btB; pBtC = btC; pI2SB = i2sB; pI2SF = i2sF;
        pDrop = drop; pUnder = under; pErr = wrErr;
        pLdacB = ldacB; pLdacP = ldacP;
        pDsp = dsp;
    }
}

// ------------------------------------------------------------
// NVS / Bluetooth
// ------------------------------------------------------------
static bool initNvs()
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS precisa ser recriada");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

static bool initBluetooth()
{
    ESP_LOGI(TAG, "Inicializando BT controller em BR/EDR ONLY...");

    esp_bt_controller_config_t btCfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    btCfg.mode = ESP_BT_MODE_CLASSIC_BT;
    esp_err_t err = esp_bt_controller_init(&btCfg);

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "BT controller init: %s", esp_err_to_name(err));
        return false;
    }

    logHeap("bt_controller_init");

    err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "BT controller enable: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "BT controller BR_EDR_ONLY: OK");
    logHeap("bt_controller_on");

    esp_bluedroid_config_t bdCfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    bdCfg.ssp_en = true;

    err = esp_bluedroid_init_with_cfg(&bdCfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Bluedroid init: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Bluedroid enable: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Bluedroid: OK");
    logHeap("bluedroid");
    return true;
}

// ------------------------------------------------------------
// app_main
// ------------------------------------------------------------
extern "C" void app_main(void)
{
    // V1.0.6: armar RX IP-BUS antes de qualquer inicializacao pesada.
    const bool earlyIpbusArmed = ipbusXmEarlyArm();

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, " %s", FW_VERSION);
    ESP_LOGI(TAG, " LDAC HQ -> XM1 BBR6 EXT / XM2 HIFI STAGE / XM3 HIFI -> PCM5102");
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "BT MODE          : BR/EDR ONLY (V1.3 validado)");
    ESP_LOGI(TAG, "BLE APP/GATT/ADV : OFF");
    ESP_LOGI(TAG, "AVRCP CT/TG      : ON");
    ESP_LOGI(TAG, "DSP              : XM1 BBR6 EXT / XM2 HIFI STAGE / XM3 BYPASS");
    ESP_LOGI(TAG, "IP-BUS           : XM / LINK 0x11E APP 0x1E");
    ESP_LOGI(TAG, "LDAC              : 44.1/48/88.2/96 kHz / prefere 96k");
    ESP_LOGI(TAG, "PCM5102           : ON");
    ESP_LOGI(TAG, "I2S BCK/LRCK/DATA : GPIO25 / GPIO32 / GPIO33");
    ESP_LOGI(TAG, "==================================================");

    ESP_LOGI(TAG, "Early IP-BUS RX     : %s", earlyIpbusArmed ? "ARMADO" : "FALHOU");

    if (!initNvs()) {
        ipbusXmEarlyDisarm();
        return;
    }
    logHeap("after_nvs");
    loadLastReconnectBda();

    // BDK V2.5 OTA: exclusive updater mode before IP-BUS/A2DP/I2S/DSP.
    if (btOtaShouldEnterAndConsume()) {
        ESP_LOGW(TAG, "Modo OTA solicitado -> liberando IP-BUS e iniciando Bluetooth SPP OTA");
        ipbusXmEarlyDisarm();
        btOtaRunServerBlocking("DEH-P650 WRD OTA");
        return;
    }

    // ========================================================
    // IP-BUS - ARMAR ANTES DO BLUETOOTH
    // ========================================================
    ESP_LOGI(TAG, "Iniciando IP-BUS XM V3.26 nativo antes do Bluetooth...");
    if (!ipbusXmStart()) {
        ESP_LOGE(TAG, "Falha ao iniciar IP-BUS XM V3.26");
        ipbusXmEarlyDisarm();
        return;
    }
    ESP_LOGI(TAG, "IP-BUS RX ARMADO / pronto");
    ESP_LOGI(TAG, "Aguardando handshake XM antes de iniciar Bluetooth/audio...");

    // Mesma filosofia da base Multi-CD que funcionava com ambos ligados juntos:
    // IP-BUS primeiro; Bluetooth somente depois do handshake.
    const bool xmLinkedBeforeAudio = ipbusXmWaitForInitialLink(12000UL);

    if (xmLinkedBeforeAudio) {
        ESP_LOGI(TAG, "XM LINK OK -> liberando Bluetooth/audio.");
    } else {
        ESP_LOGW(TAG, "XM ainda nao linkou em 12 s; Bluetooth/audio serao iniciados e IP-BUS continua tentando.");
    }

    const size_t ringBytes = RING_FRAMES * 2u * sizeof(int32_t);
    g_ring = (int32_t*)heap_caps_malloc(ringBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!g_ring) {
        ESP_LOGE(TAG, "Falha ring buffer: %u bytes", (unsigned)ringBytes);
        return;
    }
    memset(g_ring, 0, ringBytes);
    ESP_LOGI(TAG, "Ring buffer: %u frames / %u bytes", (unsigned)RING_FRAMES, (unsigned)ringBytes);
    logHeap("ring_alloc");

    g_dsp.init(48000);
    g_dsp.setHarmonicBassEnabled(true);
    g_dsp.setDetailEnabled(true);
    g_dsp.setProfile(BDK_AUDIO_PROFILE_BOOMBOX3);
    g_requestedXmBand = 1;
    g_profileChangePending = false;
    ESP_LOGI(TAG,
             "Perfis XM: XM1=DSP-1 | XM2=DSP2 | "
             "XM3=HIFI PURE BYPASS");
    logHeap("dsp_ready");

    if (!initI2S(48000)) {
        ESP_LOGE(TAG, "Falha fatal no I2S");
        return;
    }
    logHeap("i2s_ready");

    BaseType_t rt = xTaskCreatePinnedToCore(audioRenderTask, "pcm5102_render", 4096,
                                            nullptr, 5, nullptr, 0);
    if (rt != pdPASS) {
        ESP_LOGE(TAG, "Falha ao criar pcm5102_render");
        return;
    }

    if (!initBluetooth()) {
        ESP_LOGE(TAG, "Falha fatal no Bluetooth");
        return;
    }

    g_a2dp.set_output_active(false);
    g_a2dp.set_stream_reader(onStreamData, false);
    g_a2dp.set_codec_config_callback(onCodecConfig);
    g_a2dp.set_on_connection_state_changed(onConnectionState);
    g_a2dp.set_on_audio_state_changed(onAudioState);
    g_a2dp.set_auto_reconnect(false);
    g_a2dp.set_task_core(1);

    ESP_LOGI(TAG, "Iniciando NativeA2DPSink...");
    g_a2dp.start(DEVICE_NAME);
    vTaskDelay(pdMS_TO_TICKS(700));
    g_a2dp.set_discoverability(ESP_BT_GENERAL_DISCOVERABLE);

    if (xTaskCreatePinnedToCore(autoReconnectTask, "a2dp_reconnect", 3072,
                                nullptr, 2, &g_reconnectTask, 1) != pdPASS) {
        ESP_LOGE(TAG, "Falha criando supervisor de auto reconnect");
        return;
    }

    xTaskCreatePinnedToCore(telemetryTask, "v23_stats", 2816,
                            nullptr, 1, nullptr, 0);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, " BDK XM V1.0.8.0 AUTO RECONNECT VALIDADO PRONTA");
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "Nome Bluetooth: %s", DEVICE_NAME);
    ESP_LOGI(TAG, "Auto reconnect   : ATIVO / supervisor unico V1.3");
    ESP_LOGI(TAG, "Conexao BT       : automatica para o ultimo Galaxy pareado");
    ESP_LOGI(TAG, "Galaxy: habilite LDAC.");
    ESP_LOGI(TAG, "Para maior qualidade, selecione no Android:");
    ESP_LOGI(TAG, "  LDAC + Optimized for audio quality.");
    ESP_LOGI(TAG, "A telemetria mostrara o payload LDAC aproximado em kbps.");
    ESP_LOGI(TAG, "Se 96 kHz ficar instavel, teste 48 kHz mantendo Quality/HQ.");
    ESP_LOGI(TAG, "XM1: HIFI BBR6 EXTENDED / grave macio.");
    ESP_LOGI(TAG, "XM2: HIFI STAGE CORSA / delay L 1.55 ms.");
    ESP_LOGI(TAG, "XM3: HIFI PURE / DSP BYPASS.");
    ESP_LOGI(TAG, "BBR6 protegido em 50-52 Hz; subgrave abaixo disso alimenta harmonicos.");
    ESP_LOGI(TAG, "BAND no Pioneer troca XM1/XM2/XM3 e o perfil em runtime.");
    ESP_LOGI(TAG, "IP-BUS prioritario: task prio 8 / audio render prio 5.");
    ESP_LOGI(TAG, "Canal XM: CH001..CH255, sem CH000 e sem sync AVRCP TRACK_NUM.");
    ESP_LOGI(TAG, "96k jitter buffer: ring=4096 prebuffer=2048 render=256 highWM=3072.");
    ESP_LOGI(TAG, "OTA Bluetooth    : BOOT 3 s -> DEH-P650 WRD OTA");
    ESP_LOGI(TAG, "AUTO CONNECT     : VALIDADO / BR_EDR_ONLY + LDAC/SBC");
    ESP_LOGI(TAG, "==================================================");

    // Validate a newly installed OTA image only after normal startup.
    btOtaScheduleRunningImageValidation(10000);

    // GPIO0 / BOOT: hold ~3 s and release to reboot into SPP OTA mode.
    if (!btOtaStartBootButtonWatcher(0, 3000)) {
        ESP_LOGW(TAG, "Nao foi possivel iniciar watcher do BOOT OTA");
    } else {
        ESP_LOGI(TAG, "OTA Bluetooth: segure BOOT 3 s para entrar no updater");
    }

    logHeap("system_ready");
}
