from pathlib import Path
import hashlib
import sys

FLASH_SIZE = 4 * 1024 * 1024
BOOT_OFF = 0x1000
PART_OFF = 0x8000
APP_OFF = 0x20000
OTA0_SIZE = 0x180000

if len(sys.argv) != 5:
    raise SystemExit("uso: MAKE_FACTORY_IMAGE.py <bootloader.bin> <partition-table.bin> <app.bin> <saida.bin>")

boot_p, part_p, app_p, out_p = map(Path, sys.argv[1:])
for p in (boot_p, part_p, app_p):
    if not p.is_file():
        raise SystemExit(f"[ERRO] arquivo nao encontrado: {p}")

boot = boot_p.read_bytes()
part = part_p.read_bytes()
app = app_p.read_bytes()

if not boot or boot[0] != 0xE9:
    raise SystemExit("[ERRO] bootloader nao parece imagem ESP32 valida")
if not app or app[0] != 0xE9:
    raise SystemExit("[ERRO] app nao parece imagem ESP32 valida")
if len(app) > OTA0_SIZE:
    raise SystemExit(f"[ERRO] app excede ota_0: {len(app)} > {OTA0_SIZE}")
if BOOT_OFF + len(boot) > PART_OFF:
    raise SystemExit("[ERRO] bootloader invade tabela de particoes")
if PART_OFF + len(part) > 0x9000:
    raise SystemExit("[ERRO] tabela de particoes invade NVS")
if APP_OFF + len(app) > 0x1A0000:
    raise SystemExit("[ERRO] app invade ota_1")

# Imagem de flash completa: todo o restante fica 0xFF.
# Depois de erase_flash, NVS, PHY, otadata, ota_1 e SPIFFS ficam limpos.
# Com otadata apagada e sem particao factory, o bootloader ESP-IDF escolhe ota_0.
image = bytearray(b"\xFF") * FLASH_SIZE
image[BOOT_OFF:BOOT_OFF+len(boot)] = boot
image[PART_OFF:PART_OFF+len(part)] = part
image[APP_OFF:APP_OFF+len(app)] = app

out_p.parent.mkdir(parents=True, exist_ok=True)
out_p.write_bytes(image)

print(f"[OK] FULL FACTORY: {out_p}")
print(f"[OK] tamanho     : {len(image)} bytes (4 MiB)")
print(f"[OK] bootloader  : 0x{BOOT_OFF:06X} / {len(boot)} bytes")
print(f"[OK] partitions  : 0x{PART_OFF:06X} / {len(part)} bytes")
print(f"[OK] app ota_0   : 0x{APP_OFF:06X} / {len(app)} bytes")
print(f"[OK] SHA256      : {hashlib.sha256(image).hexdigest()}")
