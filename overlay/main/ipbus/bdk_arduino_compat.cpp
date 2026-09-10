
#include "bdk_arduino_compat.h"

#include <stdlib.h>
#include <string.h>

static const char *TAG = "IPBUS_COMPAT";

BdkSerialCompat Serial;

static portMUX_TYPE g_noInterruptMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_gpioIsrServiceInstalled = false;
static void (*g_gpioHandlers[GPIO_NUM_MAX])(void) = {};

static void IRAM_ATTR compatGpioThunk(void *arg);


static rmt_obj_t g_rmtTx = {};
static rmt_obj_t g_rmtRx = {};

uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

void delay(uint32_t ms)
{
    if (ms == 0) {
        taskYIELD();
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void delayMicroseconds(uint32_t us)
{
    esp_rom_delay_us(us);
}

void pinMode(uint8_t pin, uint8_t mode)
{
    if (pin >= GPIO_NUM_MAX) return;
    esp_rom_gpio_pad_select_gpio(pin);
    if (mode == OUTPUT) gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
    else gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
}

int digitalRead(uint8_t pin)
{
    return gpio_get_level((gpio_num_t)pin);
}

void digitalWrite(uint8_t pin, uint8_t level)
{
    gpio_set_level((gpio_num_t)pin, level ? 1 : 0);
}

int digitalPinToInterrupt(uint8_t pin)
{
    return (int)pin;
}

static void IRAM_ATTR compatGpioThunk(void *arg)
{
    int pin = (int)(intptr_t)arg;
    if (pin >= 0 && pin < GPIO_NUM_MAX) {
        void (*fn)(void) = g_gpioHandlers[pin];
        if (fn) fn();
    }
}

void attachInterrupt(int pin, void (*fn)(void), int mode)
{
    if (pin < 0 || pin >= GPIO_NUM_MAX || fn == nullptr) return;

    if (!g_gpioIsrServiceInstalled) {
        esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
            g_gpioIsrServiceInstalled = true;
        } else {
            ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(err));
            return;
        }
    }

    gpio_isr_handler_remove((gpio_num_t)pin);
    g_gpioHandlers[pin] = fn;

    gpio_int_type_t intr = GPIO_INTR_ANYEDGE;
    if (mode != CHANGE) intr = GPIO_INTR_ANYEDGE;
    gpio_set_intr_type((gpio_num_t)pin, intr);

    esp_err_t err = gpio_isr_handler_add((gpio_num_t)pin, compatGpioThunk, (void *)(intptr_t)pin);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add(%d): %s", pin, esp_err_to_name(err));
        g_gpioHandlers[pin] = nullptr;
    }
}

void detachInterrupt(int pin)
{
    if (pin < 0 || pin >= GPIO_NUM_MAX) return;
    gpio_isr_handler_remove((gpio_num_t)pin);
    g_gpioHandlers[pin] = nullptr;
}

void noInterrupts(void)
{
    portENTER_CRITICAL(&g_noInterruptMux);
}

void interrupts(void)
{
    portEXIT_CRITICAL(&g_noInterruptMux);
}

void pinMatrixOutAttach(uint8_t pin, uint32_t signal, bool invert, bool oenInvert)
{
    esp_rom_gpio_pad_select_gpio(pin);
    esp_rom_gpio_connect_out_signal(pin, signal, invert, oenInvert);
}

void pinMatrixOutDetach(uint8_t pin, bool invert, bool oenInvert)
{
    (void)invert;
    (void)oenInvert;
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
}

static bool configureRmt(rmt_obj_t *obj, uint8_t pin, bool tx, int memSize)
{
    if (!obj) return false;

    memset(obj, 0, sizeof(*obj));
    obj->tx = tx;
    obj->channel = tx ? RMT_CHANNEL_0 : RMT_CHANNEL_4;
    obj->tick_ns = 1000.0f;
    obj->idle_threshold = 800;
    obj->filter_enabled = true;
    obj->filter_threshold = 2;

    // RMT_DEFAULT_CONFIG_TX/RX expandem para aggregate initializers
    // entre chaves. Em C++ eles nao podem ser usados diretamente nos
    // operandos de ?: (foi o erro real encontrado pelo GCC 14).
    rmt_config_t cfg = {};
    if (tx) {
        cfg = RMT_DEFAULT_CONFIG_TX((gpio_num_t)pin, obj->channel);
    } else {
        cfg = RMT_DEFAULT_CONFIG_RX((gpio_num_t)pin, obj->channel);
    }

    cfg.clk_div = 80; // APB 80 MHz -> 1 us tick
    cfg.mem_block_num = (memSize >= 256) ? 4 : 1;

    if (!tx) {
        cfg.rx_config.filter_en = true;
        cfg.rx_config.filter_ticks_thresh = obj->filter_threshold;
        cfg.rx_config.idle_threshold = obj->idle_threshold;
    }

    esp_err_t err = rmt_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_config ch=%d: %s", (int)obj->channel, esp_err_to_name(err));
        return false;
    }

    size_t rb_size = tx ? 0 : 4096;
    err = rmt_driver_install(obj->channel, rb_size, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "rmt_driver_install ch=%d: %s", (int)obj->channel, esp_err_to_name(err));
        return false;
    }

    if (!tx) {
        rmt_get_ringbuf_handle(obj->channel, &obj->ringbuf);
        if (!obj->ringbuf) {
            ESP_LOGE(TAG, "RMT RX ringbuffer indisponivel");
            return false;
        }
    }

    obj->configured = true;
    return true;
}

rmt_obj_t *rmtInit(uint8_t pin, int mode, int memSize)
{
    rmt_obj_t *obj = (mode == RMT_TX_MODE) ? &g_rmtTx : &g_rmtRx;
    if (obj->configured) return obj;
    if (!configureRmt(obj, pin, mode == RMT_TX_MODE, memSize)) return nullptr;
    return obj;
}

float rmtSetTick(rmt_obj_t *obj, float tick_ns)
{
    if (!obj || !obj->configured) return 0.0f;
    // This compatibility layer intentionally fixes the validated IP-BUS tick to 1 us.
    // The Operational source requests exactly 1000 ns.
    if (tick_ns < 900.0f || tick_ns > 1100.0f) return 0.0f;
    obj->tick_ns = 1000.0f;
    return obj->tick_ns;
}

bool rmtSetRxThreshold(rmt_obj_t *obj, uint32_t threshold_ticks)
{
    if (!obj || obj->tx || !obj->configured) return false;
    obj->idle_threshold = threshold_ticks;
    return rmt_set_rx_idle_thresh(obj->channel, (uint16_t)threshold_ticks) == ESP_OK;
}

bool rmtSetFilter(
    rmt_obj_t* obj,
    bool enable,
    uint8_t threshold)
{
    if (obj == nullptr)
    {
        return false;
    }

    /*
     * Operational V2.15 calcula threshold na escala do tick
     * configurado no canal: 1 us com clk_div = 80.
     *
     * rmt_set_rx_filter() da API legacy recebe clocks APB
     * de 80 MHz. Portanto 1 us = 80 clocks APB.
     *
     * A base validada normalmente solicita 2 ticks (~2 us):
     * 2 x 80 = 160 clocks APB.
     */
    uint16_t sourceTicks =
        (uint16_t)threshold * 80U;

    if (sourceTicks > 255U)
    {
        sourceTicks = 255U;
    }

    esp_err_t err =
        rmt_set_rx_filter(
            obj->channel,
            enable,
            (uint8_t)sourceTicks);

    return err == ESP_OK;
}

static void flushRxRing(rmt_obj_t *obj)
{
    if (!obj || !obj->ringbuf) return;
    while (true) {
        size_t size = 0;
        void *item = xRingbufferReceive(obj->ringbuf, &size, 0);
        if (!item) break;
        vRingbufferReturnItem(obj->ringbuf, item);
    }
}

bool rmtReadAsync(rmt_obj_t *obj, rmt_data_t *dest, size_t items,
                  EventGroupHandle_t events, bool wait, uint32_t timeout)
{
    (void)events;
    (void)wait;
    (void)timeout;
    if (!obj || obj->tx || !obj->configured || !dest || items == 0) return false;

    rmt_rx_stop(obj->channel);
    flushRxRing(obj);

    memset(dest, 0, items * sizeof(rmt_data_t));
    obj->rx_dest = dest;
    obj->rx_max_items = items;
    obj->rx_done = false;

    esp_err_t err = rmt_rx_start(obj->channel, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_rx_start: %s", esp_err_to_name(err));
        return false;
    }
    obj->rx_started = true;
    return true;
}

bool rmtReceiveCompleted(rmt_obj_t *obj)
{
    if (!obj || obj->tx || !obj->configured) return false;
    if (obj->rx_done) return true;
    if (!obj->rx_started || !obj->ringbuf || !obj->rx_dest) return false;

    size_t size = 0;
    void *item = xRingbufferReceive(obj->ringbuf, &size, 0);
    if (!item) return false;

    size_t count = size / sizeof(rmt_data_t);
    if (count > obj->rx_max_items) count = obj->rx_max_items;
    memcpy(obj->rx_dest, item, count * sizeof(rmt_data_t));
    if (count < obj->rx_max_items) {
        memset(obj->rx_dest + count, 0, (obj->rx_max_items - count) * sizeof(rmt_data_t));
    }
    vRingbufferReturnItem(obj->ringbuf, item);

    rmt_rx_stop(obj->channel);
    obj->rx_started = false;
    obj->rx_done = true;
    return true;
}

bool rmtWriteBlocking(rmt_obj_t *obj, rmt_data_t *data, size_t items)
{
    if (!obj || !obj->tx || !obj->configured || !data || items == 0) return false;
    esp_err_t err = rmt_write_items(obj->channel, data, (int)items, true);
    return err == ESP_OK;
}

void rmtEnd(rmt_obj_t *obj)
{
    if (!obj || !obj->configured) return;
    if (!obj->tx && obj->rx_started) {
        rmt_rx_stop(obj->channel);
        obj->rx_started = false;
    }
    obj->rx_done = false;
}
