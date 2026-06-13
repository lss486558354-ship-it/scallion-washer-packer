# -*- coding: utf-8 -*-
# Test: extract just the new_block definition and see if it parses
f = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\final_edit.py"
with open(f, 'rb') as fh:
    raw = fh.read()
lines = raw.split(b'\n')

# Build the script from lines 1-473, then just a simple "pass" line
test_lines = lines[:473] + [b"pass  # test"] + lines[474:]
test_raw = b'\n'.join(test_lines)

try:
    compile(test_raw, '<test>', 'exec')
    print("Lines 1-473 + 'pass' compile OK")
except SyntaxError as e:
    print(f"Error at line {e.lineno}: {e.msg}")
    for i in range(max(0, e.lineno-3), min(len(lines), e.lineno+2)):
        print(f"  L{i+1}: {repr(lines[i])}")

# Try a different approach: extract just the first 474 lines
try:
    compile(b'\n'.join(lines[:474]), '<test>', 'exec')
    print("First 474 lines compile OK")
except SyntaxError as e:
    print(f"First 474 lines error at line {e.lineno}: {e.msg}")
