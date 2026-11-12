from pathlib import Path
import json
from gnss_uart6_decoder import Decoder

fixtures=Path(__file__).resolve().parent/'Fixtures'
t=json.loads((fixtures/'typed_c_golden.json').read_text(encoding='utf-8-sig'))
r=json.loads((fixtures/'raw_c_golden.json').read_text(encoding='utf-8-sig'))
def run(name):
    raw=bytes.fromhex((fixtures/name).read_text(encoding='ascii'))
    events=Decoder().feed(raw,now_ms=0)
    assert not any(e['event']=='error' for e in events),events
    return events
typed=run('typed_c_golden.hex')
assert sum(e['event']=='gnss_bundle' for e in typed)==1
raw=run('raw_c_golden.hex')
frames=[e for e in raw if e['event']=='gnss_raw_frame']
assert len(frames)==1 and frames[0]['bytes_hex'].upper()==r['original_wire_hex'].upper()
assert frames[0]['wire_crc16_verified'] and not frames[0]['source_checksum_confirmed']
assert not frames[0]['usable_as_navigation_update'] and not frames[0]['authoritative_navigation']
assert any(e['event']=='gnss_raw_stats' for e in raw)
print('PASS: actual C typed bundle and independent BAD GPS original wire align byte-for-byte')
