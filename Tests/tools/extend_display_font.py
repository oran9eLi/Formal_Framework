"""为现有 LVGL 16px 中文字库补充缺字；保留已存在字模和索引。

用法：python Tests/tools/extend_display_font.py 只读
依赖 Pillow 和 Windows 微软雅黑；输出 4bpp、16×16 字模。
"""
from pathlib import Path
import re
import sys
from PIL import Image, ImageDraw, ImageFont

path = Path(__file__).resolve().parents[2] / 'Display/Src/display_lvgl_font_zh.c'
source = path.read_text(encoding='utf-8')
bitmap_start = source.index('static const uint8_t s_zh_bitmap[] = {')
bitmap_end = source.index('\n};', bitmap_start)
table_start = source.index('static const Display_LvglFontGlyphDsc_t s_zh_glyphs[] = {')
table_end = source.index('\n};', table_start)
offset = len(re.findall(r'0x[0-9A-Fa-f]{2}\b', source[bitmap_start:bitmap_end]))
entries = re.findall(r'^    \{0x([0-9A-F]+)U.*$', source[table_start:table_end], re.M)
known = {int(x, 16) for x in entries}
font = ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', 16)
new_bitmap, new_entries = [], []
for char in sorted(set(''.join(sys.argv[1:]))):
    cp = ord(char)
    if cp in known:
        continue
    canvas = Image.new('L', (16, 16))
    # 微软雅黑 16px 常用汉字下缘为 y=19；平移 3px 保持共同基线。
    ImageDraw.Draw(canvas).text((0, -3), char, font=font, fill=255)
    pixels = list(canvas.tobytes())
    packed = [(pixels[i] >> 4) << 4 | (pixels[i + 1] >> 4) for i in range(0, 256, 2)]
    assert any(packed), f'空字模: {char}'
    new_bitmap.extend('    ' + ', '.join(f'0x{x:02X}' for x in packed[i:i+12]) + ',' for i in range(0, 128, 12))
    new_entries.append((cp, f'    {{0x{cp:04X}U, {offset}U, 16U, 16U, 16U, 0, 0}}, /* {char} */'))
    offset += 128
if new_entries:
    table = source[table_start:table_end].splitlines()
    rows = table[1:] + [row for _, row in new_entries]
    rows.sort(key=lambda row: int(re.search(r'0x([0-9A-F]+)', row)[1], 16))
    source = source[:table_start] + table[0] + '\n' + '\n'.join(rows) + source[table_end:]
    source = source[:bitmap_end] + '\n' + '\n'.join(new_bitmap) + source[bitmap_end:]
    path.write_text(source, encoding='utf-8')
print(f'added {len(new_entries)} glyphs; bitmap {offset} bytes')
