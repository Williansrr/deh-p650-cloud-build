from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("uso: configure_sdkconfig.py <sdkconfig>")

p = Path(sys.argv[1])
if not p.exists():
    raise SystemExit(f"[ERRO] sdkconfig nao encontrado: {p}")

lines = p.read_text(encoding="utf-8", errors="strict").splitlines()

def remove_key(key):
    global lines
    a = key + "="
    b = f"# {key} is not set"
    lines = [x for x in lines if not (x.startswith(a) or x == b)]

def set_y(key):
    remove_key(key); lines.append(f"{key}=y")

def set_n(key):
    remove_key(key); lines.append(f"# {key} is not set")

def set_v(key, value):
    remove_key(key); lines.append(f"{key}={value}")

set_y("CONFIG_BT_ENABLED")
set_y("CONFIG_BT_CLASSIC_ENABLED")
set_n("CONFIG_BT_BLE_ENABLED")
set_y("CONFIG_BT_A2DP_ENABLE")
set_y("CONFIG_BT_A2DP_LDAC_DECODER")
set_n("CONFIG_BT_A2DP_APTX_DECODER")
set_n("CONFIG_BT_A2DP_AAC_DECODER")
set_n("CONFIG_BT_A2DP_OPUS_DECODER")
set_n("CONFIG_BT_A2DP_LC3PLUS_DECODER")
set_n("CONFIG_BTDM_CTRL_MODE_BTDM")
set_y("CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY")
set_n("CONFIG_BTDM_CTRL_MODE_BLE_ONLY")
set_v("CONFIG_BTDM_CTRL_BR_EDR_MAX_ACL_CONN", "2")
set_v("CONFIG_BT_ACL_CONNECTIONS", "2")
set_v("CONFIG_BTDM_CTRL_BR_EDR_MAX_SYNC_CONN", "0")
set_v("CONFIG_BT_BTC_TASK_STACK_SIZE", "3584")
set_v("CONFIG_BT_BTU_TASK_STACK_SIZE", "3584")
set_y("CONFIG_BT_SPP_ENABLED")
set_y("CONFIG_BT_SSP_ENABLED")

set_y("CONFIG_PARTITION_TABLE_CUSTOM")
set_v("CONFIG_PARTITION_TABLE_CUSTOM_FILENAME", '"partitions.csv"')
set_v("CONFIG_PARTITION_TABLE_FILENAME", '"partitions.csv"')
set_y("CONFIG_ESPTOOLPY_FLASHSIZE_4MB")
set_y("CONFIG_ESPTOOLPY_FLASHFREQ_80M")
set_v("CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ", "240")
set_v("CONFIG_FREERTOS_HZ", "1000")
set_v("CONFIG_ESP_INT_WDT_TIMEOUT_MS", "800")
set_v("CONFIG_ESP_TASK_WDT_TIMEOUT_S", "10")

# Explicitly undo the experimental boot-speed options used later.
set_n("CONFIG_BOOTLOADER_SKIP_VALIDATE_ON_POWER_ON")
set_n("CONFIG_BOOTLOADER_SKIP_VALIDATE_IN_DEEP_SLEEP")

p.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
print("[OK] sdkconfig: BR/EDR ONLY + LDAC/SBC + SPP OTA + boot padrao")
