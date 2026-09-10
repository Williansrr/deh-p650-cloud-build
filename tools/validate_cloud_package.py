from pathlib import Path
import sys

r = Path(__file__).resolve().parents[1]
m = (r/"overlay/main/main.cpp").read_text(encoding="utf-8")
ip = (r/"overlay/main/ipbus/ipbus_xm_v326.cpp").read_text(encoding="utf-8")
dsp = (r/"overlay/main/dsp/bdk_bass_dsp.cpp").read_text(encoding="utf-8")
cm = (r/"overlay/main/CMakeLists.txt").read_text(encoding="utf-8")
wf = (r/".github/workflows/build.yml").read_text(encoding="utf-8")

checks = [
    ("DIAG identificado", "V1.0.8.5 DIAG / SWAP AUDIO XM1-XM2" in m),
    ("XM1 -> XM2 profile", "case 1: return BDK_AUDIO_PROFILE_XTREME4;" in m),
    ("XM2 -> BBR6", "case 2: return BDK_AUDIO_PROFILE_BOOMBOX3;" in m),
    ("XM3 bypass", "case 3: return BDK_AUDIO_PROFILE_HIFI;" in m),
    ("IPBUS LINK 0x11E", "#define DEVICE_ADDR    0x11E" in ip),
    ("APP 0x1E", "#define DEVICE_ID8     0x1E" in ip),
    ("late boot 150", "#define XM_FIRST_FORCED_BOOT_MS       150UL" in ip),
    ("late boot 600", "#define XM_FORCED_BOOT_RETRY_MS       600UL" in ip),
    ("late boot 8", "#define XM_MAX_FORCED_BOOT_ATTEMPTS     8" in ip),
    ("recovery 15s", "#define XM_RECOVERY_SILENCE_MS  15000UL" in ip),
    ("BBR6 0.08 preservado", "m_harmonicMixBase = 0.08f;" in dsp),
    ("CMake minimal", "ipbus/ipbus_xm_v326.cpp" in cm and "bt/a2dp_sink_native.cpp" in cm),
    ("workflow upstream", "WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED" in wf),
    ("workflow IDF install", "./install.sh esp32" in wf),
    ("workflow build", "idf.py build" in wf),
    ("workflow dehota", "GERAR_DEHOTA.py" in wf),
    ("artifact v4", "actions/upload-artifact@v4" in wf),
]
bad = False
for n, ok in checks:
    print(("[OK] " if ok else "[ERRO] ") + n)
    bad |= not ok
if bad:
    raise SystemExit(1)
print(f"[OK] Cloud package: {len(checks)} checks")
