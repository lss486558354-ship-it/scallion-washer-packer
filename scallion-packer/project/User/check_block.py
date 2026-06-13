# -*- coding: utf-8 -*-
f = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\final_edit.py"
with open(f, 'rb') as fh:
    raw = fh.read()

# Extract just the new_block string (lines 85 to 474)
lines = raw.split(b'\n')
start = 85 - 1  # 0-indexed
end = 474 - 1   # 0-indexed (exclusive)
block = b'\n'.join(lines[start:end])
print(f"Block size: {len(block)} bytes")

# Count occurrences of triple quotes in the block
tq_count = block.count(b"'''")
print(f"Triple quotes in block: {tq_count}")

if tq_count > 0:
    # Find positions
    pos = 0
    while True:
        idx = block.find(b"'''", pos)
        if idx < 0:
            break
        line_num = block[:idx].count(b'\n') + 1
        line_start = block.rfind(b'\n', 0, idx) + 1
        line_end = block.find(b'\n', idx)
        line_content = block[line_start:line_end]
        print(f"  Found ''' at byte {idx}, line {line_num + start}: {repr(line_content[:60])}")
        pos = idx + 3

# Try to create the string
try:
    s = block.decode('utf-8')
    print(f"Block decodes OK as UTF-8")
except Exception as e:
    print(f"Block decode error: {e}")
