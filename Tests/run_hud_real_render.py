"""Compile the real LVGL software renderer and production HUD callback on Windows."""
from pathlib import Path
import os,subprocess,sys,json,time,zipfile,hashlib
root=Path(__file__).resolve().parents[1]
output=root/'build/20261111_hud_frame/real_render'
output.mkdir(parents=True,exist_ok=True)
baseline='--baseline' in sys.argv
golden=output/'e65_golden';golden.mkdir(exist_ok=True)
variant=output/('baseline' if baseline else 'current');variant.mkdir(exist_ok=True)
compiler=Path('D:/mingw64/bin/gcc.exe')
env=os.environ.copy();env['PATH']=str(compiler.parent)+';'+env['PATH']
sources=[root/'Display/Src/display_lvgl_numeric_font.c',root/'Display/Src/display_hud_present.c']+sorted((root/'Third_Party/LVGL/src').rglob('*.c'))
mask_helper=root/'Display/Src/display_hud_masks.c'
if mask_helper.exists():sources.append(mask_helper)
args=['-std=c99','-O2','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Wno-unused-parameter','-DLV_CONF_PATH='+str(root/'Tests/stubs/hud_lv_conf.h').replace('\\','/')]
for name in ('Tests/stubs','Core/Inc','Debug/Inc','Bsp/Inc','Sensor/Inc','Framework/Inc','Business/Inc','Display/Inc','Storage/Inc','Third_Party/LVGL'):args.append('-I'+(root/name).as_posix())
if baseline:
    with zipfile.ZipFile(root/'Release/Archive/2026-11-11_hud-exact-diff/production_source.zip') as z:(variant/'e65_display_lvgl.c').write_bytes(z.read('Display/Src/display_lvgl.c'))
    unit_extra=['-DHUD_REAL_RENDER_SOURCE="'+(variant/'e65_display_lvgl.c').as_posix()+'"']
else:unit_extra=[]
unitargs=args+unit_extra+['-fwhole-program','-Werror','-c',(root/'Tests/Unit/test_hud_real_render.c').as_posix(),'-o',(variant/'test_hud_real_render.o').as_posix()]
args += [p.as_posix() for p in sources]+[(variant/'test_hud_real_render.o').as_posix(),'-Wl,--gc-sections','-lm','-o',(variant/'test_hud_real_render.exe').as_posix()]
rsp=variant/'compile.rsp';rsp.write_text('\n'.join('"'+a+'"' for a in args),encoding='utf-8')
started=time.monotonic()
with (variant/'build.log').open('w',encoding='utf-8') as log:
    result=subprocess.run([str(compiler)]+unitargs,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
    if result.returncode==0:result=subprocess.run([str(compiler),'@'+str(rsp)],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
if result.returncode:
    print((variant/'build.log').read_text());sys.exit(result.returncode)
with (variant/'run.log').open('w',encoding='utf-8') as log:
    result=subprocess.run([str(variant/'test_hud_real_render.exe'),str(golden),'write' if baseline else 'compare'],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180)
print((variant/'run.log').read_text(),flush=True)
evidence={'passed':result.returncode==0,'exit_code':result.returncode,'source_count':len(sources)+1,'elapsed_s':round(time.monotonic()-started,3),'golden_mode':'write' if baseline else 'compare','production_display_sha256':hashlib.sha256((variant/'e65_display_lvgl.c' if baseline else root/'Display/Src/display_lvgl.c').read_bytes()).hexdigest(),'backend_sha256':hashlib.sha256((root/'Display/Src/display_hud_present.c').read_bytes()).hexdigest(),'test_source_sha256':hashlib.sha256((root/'Tests/Unit/test_hud_real_render.c').read_bytes()).hexdigest()}
if mask_helper.exists():evidence['mask_helper_sha256']=hashlib.sha256(mask_helper.read_bytes()).hexdigest()
(variant/'result.json').write_text(json.dumps(evidence,indent=2)+'\n')
if baseline and result.returncode==0:
    (golden/'manifest.json').write_text(json.dumps({'source_archive':'Release/Archive/2026-11-11_hud-exact-diff/production_source.zip','display_sha256':evidence['production_display_sha256'],'format':'258x258 little-endian RGB565, full rectangle, background 0x12202D','frames':[{'path':p.name,'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(golden.glob('*.r565'))]},indent=2)+'\n')
sys.exit(result.returncode)
