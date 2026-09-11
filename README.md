# DEH-P650 — Cloud Build V1.0.8.5 DIAG XM1↔XM2

Este repositório compila na nuvem o firmware de diagnóstico do Pioneer DEH-P650.

## Resultado do teste
- XM1 do rádio usa temporariamente o perfil de áudio originalmente usado pelo XM2.
- XM2 do rádio usa temporariamente o BBR6 originalmente usado pelo XM1.
- XM3 permanece HIFI PURE / bypass.

O objetivo é descobrir se o chiado acompanha o BBR6/DSP.

## Como usar
1. Crie um repositório novo no GitHub.
2. Envie TODO o conteúdo desta pasta, inclusive `.github`.
3. Faça o commit.
4. A compilação inicia automaticamente.
5. Abra a aba Actions.
6. Abra `Build DEH-P650 V1.0.8.5 DIAG`.
7. Quando ficar verde, abra a execução.
8. Na área Artifacts, baixe `DEH-P650-V1.0.8.5-DIAG-XM1-XM2`.
9. Extraia o ZIP do artefato.
10. Use `BDK_XM_V1_0_8_5_DIAG_SWAP_XM1_XM2.dehota` pelo OTA normal.

## O que o cloud build usa
O workflow baixa:
https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED

Essa base contém a árvore ESP-IDF 5.5.2 modificada com suporte LDAC.
Depois o workflow aplica o overlay DEH-P650 e compila.

## Importante
Para este diagnóstico use SOMENTE o arquivo `.dehota`.
Os arquivos `bootloader_reference.bin` e `partition-table_reference.bin` são guardados
apenas como referência de build. Não precisam ser gravados.

Se a Action ficar vermelha, abra a etapa que falhou, copie o erro e envie ao ChatGPT.
	/

## Internal Hardware Connection — Pioneer DEH-P650

The project communicates directly with the Pioneer DEH-P650 internal IP-BUS interface through IC101, a HA12187FP.

### ESP32 ↔ HA12187FP

| Signal | ESP32 | Connection |
|---|---:|---|
| R / RX | GPIO16 | 10 kΩ → HA12187FP pin 2 |
| RX divider | GPIO16 | 20 kΩ → HA12187FP pin 4 (GND) |
| S1 monitor | GPIO17 | 10 kΩ → HA12187FP pin 1 |
| S1 divider | GPIO17 | 20 kΩ → HA12187FP pin 4 (GND) |
| STB | GPIO21 | 10 kΩ → HA12187FP pin 8 |
| STB divider | GPIO21 | 20 kΩ → HA12187FP pin 4 (GND) |
| TX | GPIO18 | → MM74HCT244N pin 2 (1A1) |
| TX enable | GPIO19 | → MM74HCT244N pin 1 (/1OE) |
| /OE pull-up | — | 10 kΩ between 3.3 V and GPIO19 / pin 1 |
| IP-BUS TX output | MM74HCT244N pin 18 (1Y1) | 1 kΩ → HA12187FP pin 3 (S2) |

### MM74HCT244N

- Pin 20 → 5 V
- Pin 10 → GND
- 100 nF decoupling capacitor between pins 20 and 10
- ESP32 and radio grounds must be common

### PCM5102 DAC

| PCM5102 | ESP32 |
|---|---:|
| BCK | GPIO25 |
| LRCK / WS | GPIO32 |
| DIN / DATA | GPIO33 |
| VIN | 5 V |
| GND | Common GND |
| SCK | GND |
| XSMT | 3.3 V |

The PCM5102 analog audio output is routed to the DEH-P650 audio path associated with the IP-BUS source.

### Notes

This wiring represents the internal hardware configuration used during development and testing of the DEH-P650 IP-BUS emulator.

The MM74HCT244N is used as the output buffer between the ESP32 and the HA12187FP IP-BUS interface.

The ESP32 power supply connection inside the DEH-P650 is still under evaluation and is therefore not documented here as a final connection.

Internal modifications to the head unit should only be performed with the unit disconnected from power.


## XM IP-BUS Emulation

The ESP32 firmware emulates a Pioneer XM tuner through the DEH-P650 IP-BUS interface.

The current implementation is based on the behavior of Pioneer XM tuner families such as the GEX-P900XM / GEX-P910XM.

No physical XM tuner module is required. The ESP32 presents itself to the head unit as an XM-compatible IP-BUS source and handles the communication required for source detection, control commands and display information.

The XM emulation is integrated with the Bluetooth audio path used by this project, allowing the DEH-P650 to select and control the emulated XM source while audio is provided by the ESP32 and PCM5102 DAC.

The current diagnostic firmware uses three XM profiles:

- XM1: test audio/DSP profile
- XM2: alternate test audio/DSP profile
- XM3: HIFI PURE / bypass profile

The XM implementation is located mainly in:

`overlay/main/ipbus/ipbus_xm_v326.cpp`

and is built together with the Bluetooth, DSP and OTA modules.
