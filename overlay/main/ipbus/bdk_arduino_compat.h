
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <type_traits>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "esp_rom_sys.h"
#include "esp_rom_gpio.h"

#include "driver/gpio.h"
#include "driver/rmt.h"

#include "soc/gpio_struct.h"
#include "soc/gpio_sig_map.h"

#include "esp_avrc_api.h"
#include "esp_task_wdt.h"
#include "esp_idf_version.h"

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
#ifndef DRAM_ATTR
#define DRAM_ATTR
#endif

#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW 0
#endif
#ifndef INPUT
#define INPUT 0x01
#endif
#ifndef OUTPUT
#define OUTPUT 0x03
#endif
#ifndef CHANGE
#define CHANGE 0x02
#endif
#ifndef HEX
#define HEX 16
#endif

// Arduino ESP32 HAL compatibility constants used by the validated source.
#define RMT_TX_MODE 1
#define RMT_RX_MODE 0
#define RMT_MEM_256 256

// Arduino ESP32 HAL exposes RMT_FLAGS_ALL for its event group.
 // Our native ESP-IDF compatibility layer does not use those event bits,
 // but the original Operational V2.15 clears them before arming RX.
 // FreeRTOS reserves the upper control bits, so clear the 24 user bits.
#ifndef RMT_FLAGS_ALL
#define RMT_FLAGS_ALL ((EventBits_t)0x00FFFFFFU)
#endif

// The legacy ESP-IDF rmt item has the same duration0/level0/duration1/level1
// layout expected by the Arduino ESP32 HAL source.
typedef rmt_item32_t rmt_data_t;

struct rmt_obj_t {
    rmt_channel_t channel;
    bool tx;
    bool configured;
    bool rx_started;
    bool rx_done;
    float tick_ns;
    uint32_t idle_threshold;
    bool filter_enabled;
    uint8_t filter_threshold;
    rmt_data_t *rx_dest;
    size_t rx_max_items;
    RingbufHandle_t ringbuf;
};


class BdkSerialCompat {
public:
    void begin(uint32_t baud) { (void)baud; }

    size_t print(const char *s) {
        if (!s) s = "(null)";
        int n = printf("%s", s);
        fflush(stdout);
        return n > 0 ? (size_t)n : 0;
    }
    size_t print(char c) {
        int n = printf("%c", c);
        fflush(stdout);
        return n > 0 ? (size_t)n : 0;
    }
    size_t print(unsigned char v) { return print((unsigned long)v); }
    size_t print(signed char v) { return print((long)v); }
    size_t print(unsigned short v) { return print((unsigned long)v); }
    size_t print(short v) { return print((long)v); }
    size_t print(unsigned int v) { return print((unsigned long)v); }
    size_t print(int v) { return print((long)v); }
    size_t print(unsigned long v) {
        int n = printf("%lu", v); fflush(stdout); return n > 0 ? (size_t)n : 0;
    }
    size_t print(long v) {
        int n = printf("%ld", v); fflush(stdout); return n > 0 ? (size_t)n : 0;
    }
    size_t print(unsigned long long v) {
        int n = printf("%llu", v); fflush(stdout); return n > 0 ? (size_t)n : 0;
    }
    size_t print(long long v) {
        int n = printf("%lld", v); fflush(stdout); return n > 0 ? (size_t)n : 0;
    }
    size_t print(float v) {
        int n = printf("%.2f", (double)v); fflush(stdout); return n > 0 ? (size_t)n : 0;
    }
    size_t print(double v) {
        int n = printf("%.2f", v); fflush(stdout); return n > 0 ? (size_t)n : 0;
    }

    template<typename T>
    size_t print(T v, int format) {
        if (format == HEX) {
            unsigned long long u = (unsigned long long)v;
            int n = printf("%llX", u); fflush(stdout); return n > 0 ? (size_t)n : 0;
        }
        return print(v);
    }

    size_t println() {
        int n = printf("\r\n"); fflush(stdout); return n > 0 ? (size_t)n : 0;
    }
    template<typename T>
    size_t println(T v) {
        size_t n = print(v);
        n += println();
        return n;
    }
};

extern BdkSerialCompat Serial;

uint32_t millis(void);
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

void pinMode(uint8_t pin, uint8_t mode);
int digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, uint8_t level);
int digitalPinToInterrupt(uint8_t pin);
void attachInterrupt(int pin, void (*fn)(void), int mode);
void detachInterrupt(int pin);
void noInterrupts(void);
void interrupts(void);

void pinMatrixOutAttach(uint8_t pin, uint32_t signal, bool invert, bool oenInvert);
void pinMatrixOutDetach(uint8_t pin, bool invert, bool oenInvert);

rmt_obj_t *rmtInit(uint8_t pin, int mode, int memSize);
float rmtSetTick(rmt_obj_t *obj, float tick_ns);
bool rmtSetRxThreshold(rmt_obj_t *obj, uint32_t threshold_ticks);
bool rmtSetFilter(rmt_obj_t *obj, bool enable, uint8_t threshold_ticks);
bool rmtReadAsync(rmt_obj_t *obj, rmt_data_t *dest, size_t items,
                  EventGroupHandle_t events, bool wait, uint32_t timeout);
bool rmtReceiveCompleted(rmt_obj_t *obj);
bool rmtWriteBlocking(rmt_obj_t *obj, rmt_data_t *data, size_t items);
void rmtEnd(rmt_obj_t *obj);
