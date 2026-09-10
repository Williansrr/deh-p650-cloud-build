from pathlib import Path
import hashlib, struct, sys
if len(sys.argv) != 3:
    raise SystemExit('uso: GERAR_DEHOTA.py firmware.bin saida.dehota')
src=Path(sys.argv[1]); dst=Path(sys.argv[2])
data=src.read_bytes()
if not data or data[0] != 0xE9:
    raise SystemExit('firmware ESP32 invalido')
magic=b'DEHOTA1\0'
version=struct.pack('<I',1)
size=struct.pack('<I',len(data))
sha=hashlib.sha256(data).digest()
target=b'DEH-P650-WRD'.ljust(16,b'\0')
header=magic+version+size+sha+target
assert len(header)==64
dst.write_bytes(header+data)
print(f'[OK] {dst} | {len(data)} bytes | SHA256={hashlib.sha256(data).hexdigest()}')
