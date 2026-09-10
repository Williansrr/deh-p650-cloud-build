
#pragma once

#include <stdint.h>

// BDK V2.5 - Bluetooth Classic SPP OTA bootstrap.
//
// Fluxo:
// 1) firmware normal roda A2DP/LDAC normalmente;
// 2) segure BOOT (GPIO0) por ~3 s;
// 3) firmware grava flag em NVS e espera soltar BOOT;
// 4) reinicia no modo "DEH-P650 WRD OTA";
// 5) recebe arquivo .dehota via Bluetooth SPP;
// 6) valida tamanho + SHA-256 + imagem ESP;
// 7) grava a proxima particao OTA e reinicia;
// 8) firmware novo confirma boot; se falhar, rollback do bootloader.
//
// O modo OTA eh exclusivo: nao inicializa A2DP, I2S, DSP nem IP-BUS.

bool btOtaShouldEnterAndConsume();
bool btOtaRequestOnNextBoot();

void btOtaRunServerBlocking(const char *device_name = "DEH-P650 WRD OTA");

// Chamar quando o firmware normal terminou o startup com sucesso.
// Se o app estiver ESP_OTA_IMG_PENDING_VERIFY, confirma apos um pequeno atraso.
void btOtaScheduleRunningImageValidation(uint32_t delay_ms = 10000);

// Segurar o botao em nivel LOW solicita o proximo boot em modo OTA.
// Para ESP32 DevKit, usar GPIO0 / BOOT.
bool btOtaStartBootButtonWatcher(int gpio_num = 0, uint32_t hold_ms = 3000);
