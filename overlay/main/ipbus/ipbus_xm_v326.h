
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Native ESP-IDF entry points for the DEH-P650 XM V3.26 protocol core.
// Arma somente o RX de boot o mais cedo possivel, antes de NVS/BT.
bool ipbusXmEarlyArm(void);
void ipbusXmEarlyDisarm(void);

bool ipbusXmStart(void);

// V1.0.7.1: boot robusto inspirado na Operational V2.13 Multi-CD.
bool ipbusXmWaitForInitialLink(uint32_t timeoutMs);
bool ipbusXmIsLinked(void);

bool ipbusXmReady(void);
uint8_t ipbusXmCurrentBand(void);

// Bluetooth AVRCP -> Pioneer XM metadata bridge.
void ipbusXmAvrcConnection(bool connected);
void ipbusXmAvrcMetadata(uint8_t attr, const uint8_t *data, size_t len);

// Implemented by main.cpp. Called whenever BAND changes XM1/XM2/XM3.
void bdkXmBandChanged(uint8_t band);

// A2DP/AVRCP controls implemented by main.cpp.
bool bdkAvrcConnected(void);
void bdkAvrcNext(void);
void bdkAvrcPrevious(void);
void bdkAvrcPlay(void);

#ifdef __cplusplus
}
#endif
