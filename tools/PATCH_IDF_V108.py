from pathlib import Path
import sys
idf = Path(sys.argv[1])
p = idf / 'components/bt/host/bluedroid/btc/profile/std/a2dp/btc_a2dp_sink.c'
if not p.exists():
    raise SystemExit(f'[ERRO] arquivo nao encontrado: {p}')
s = p.read_text(encoding='utf-8', errors='strict')
old = 'APPL_TRACE_WARNING("media task unhandled evt: 0x%x\\n", sig);'
new = 'APPL_TRACE_WARNING("media task unhandled evt");'
if old in s:
    bak = p.with_suffix(p.suffix + '.pre_v108.bak')
    if not bak.exists(): bak.write_text(s, encoding='utf-8', newline='\n')
    s = s.replace(old, new)
    p.write_text(s, encoding='utf-8', newline='\n')
    print('[OK] patch btc_a2dp_sink.c aplicado.')
elif new in s:
    print('[OK] patch btc_a2dp_sink.c ja estava aplicado.')
else:
    print('[INFO] linha do patch nao encontrada; nenhuma alteracao feita.')
