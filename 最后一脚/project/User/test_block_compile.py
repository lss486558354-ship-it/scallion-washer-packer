# -*- coding: utf-8 -*-
# Extract and compile the new_block string
f = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\final_edit.py"
with open(f, 'rb') as fh:
    raw = fh.read()
lines = raw.split(b'\n')

# Try to compile just the new_block definition (lines 85-474)
try:
    block_raw = b'\n'.join(lines[84:474])
    compile(block_raw, '<block>', 'exec')
    print("new_block definition compiles OK")
except SyntaxError as e:
    print(f"new_block error at line {e.lineno}: {e.msg}")
    actual_line = 85 + e.lineno - 1
    print(f"  -> Line {actual_line}: {repr(lines[actual_line-1][:80])}")

# Now try lines 1-474
try:
    block_raw2 = b'\n'.join(lines[:474])
    compile(block_raw2, '<block2>', 'exec')
    print("Lines 1-474 compile OK")
except SyntaxError as e:
    print(f"Lines 1-474 error at line {e.lineno}: {e.msg}")
    if e.lineno:
        print(f"  -> Line {e.lineno}: {repr(lines[e.lineno-1][:80])}")
