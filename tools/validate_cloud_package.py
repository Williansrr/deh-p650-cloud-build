from pathlib import Path

r = Path(__file__).resolve().parents[1]
m = (r / "overlay/main/main.cpp").read_text(encoding="utf-8")
ip = (r / "overlay/main/ipbus/ipbus_xm_v326.cpp").read_text(encoding="utf-8")
dsp = (r / "overlay/main/dsp/bdk_bass_dsp.cpp").read_text(encoding="utf-8")
pt = (r / "overlay/partitions.csv").read_text(encoding="utf-8")
wf = (r / ".github/workflows/build.yml").read_text(encoding="utf-8")

checks = [
    ("stage V1.0.8.3", "BDK V1.0.8.3 - HIFI STAGE CORSA / TIME ALIGNMENT XM1+XM2" in m),
    ("XM1 BBR6", "return BDK_AUDIO_PROFILE_BOOMBOX3;" in m and "case 2: return BDK_AUDIO_PROFILE_XTREME4;" in m),
    ("XM2 HIFI STAGE", "case 2: return BDK_AUDIO_PROFILE_XTREME4;" in m),
    ("XM3 bypass", "case 3: return BDK_AUDIO_PROFILE_HIFI;" in m),
    ("delay L 1.55 ms", "STAGE_LEFT_DELAY_MS = 1.55f" in m),
    ("sem pagina de versao DISP", "XM_FW_DISPLAY_VERSION" not in ip),
    ("sem NVS de ultima banda", "xm_state" not in ip and "last_band" not in ip),
    ("4 paginas DISP", "#define XM_METADATA_PAGE_COUNT 4" in ip),
    ("recovery 7 s", "#define XM_RECOVERY_SILENCE_MS   7000UL" in ip),
    ("late boot 150 ms", "#define XM_FIRST_FORCED_BOOT_MS       150UL" in ip),
    ("late boot 600 ms", "#define XM_FORCED_BOOT_RETRY_MS       600UL" in ip),
    ("late boot 8 tentativas", "#define XM_MAX_FORCED_BOOT_ATTEMPTS     8" in ip),
    ("LINK 0x11E", "#define DEVICE_ADDR    0x11E" in ip),
    ("APP 0x1E", "#define DEVICE_ID8     0x1E" in ip),
    ("BBR6 XM1", "m_harmonicMixBase = 0.08f;" in dsp),
    ("voicing XM2", "m_harmonicMixBase = 0.025f;" in dsp),
    ("ota_0 0x20000", "ota_0,    app,  ota_0,   0x20000,  0x180000" in pt),
    ("flash 4MB", "CONFIG_ESPTOOLPY_FLASHSIZE_4MB" in (r / "tools/configure_sdkconfig.py").read_text(encoding="utf-8")),
    ("install via bash", "bash ./install.sh esp32" in wf),
    ("factory image", "MAKE_FACTORY_IMAGE.py" in wf),
    ("artifact factory", "DEH-P650-V1.0.8.3-FULL-FACTORY" in wf),
]

bad = False
for name, ok in checks:
    print(("[OK] " if ok else "[ERRO] ") + name)
    bad |= not ok
if bad:
    raise SystemExit(1)
print(f"[OK] V1.0.8.3 FULL FACTORY: {len(checks)} checks")
