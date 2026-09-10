from pathlib import Path
import sys

root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent
m = (root/"main.cpp").read_text(encoding="utf-8")
ip = (root/"ipbus_xm_v326.cpp").read_text(encoding="utf-8")
dsp = (root/"bdk_bass_dsp.cpp").read_text(encoding="utf-8")

checks = [
    ("identificacao DIAG", "BDK XM V1.0.8.5 DIAG / SWAP AUDIO XM1-XM2" in m),
    ("XM1 -> perfil XM2 limpo", "case 1: return BDK_AUDIO_PROFILE_XTREME4;" in m),
    ("XM2 -> BBR6 suspeito", "case 2: return BDK_AUDIO_PROFILE_BOOMBOX3;" in m),
    ("XM3 continua bypass", "case 3: return BDK_AUDIO_PROFILE_HIFI;" in m),
    ("stage continua igual em XM1/XM2", "(g_requestedXmBand == 1 || g_requestedXmBand == 2)" in m),
    ("IP-BUS 0x11E", "#define DEVICE_ADDR    0x11E" in ip),
    ("IP-BUS APP 0x1E", "#define DEVICE_ID8     0x1E" in ip),
    ("late boot 150ms", "#define XM_FIRST_FORCED_BOOT_MS       150UL" in ip),
    ("late boot 600ms", "#define XM_FORCED_BOOT_RETRY_MS       600UL" in ip),
    ("late boot 8x", "#define XM_MAX_FORCED_BOOT_ATTEMPTS     8" in ip),
    ("recovery 15s", "#define XM_RECOVERY_SILENCE_MS  15000UL" in ip),
    ("sem bridge no app", "BOOTLOADER IPBUS BRIDGE" not in m.upper()),
    ("BBR6 preservado", "m_harmonicMixBase = 0.08f;" in dsp),
    ("perfil XM2 preservado", "m_harmonicMixBase = 0.025f;" in dsp),
]
bad = False
for name, ok in checks:
    print(("[OK] " if ok else "[ERRO] ") + name)
    bad |= not ok
if bad:
    raise SystemExit(1)
print(f"[OK] Diagnostico cruzado validado: {len(checks)} checks")
