# -*- coding: utf-8 -*-
# Binary search for the problematic line
f = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\final_edit.py"
with open(f, 'rb') as fh:
    raw = fh.read()
lines = raw.split(b'\n')

# Binary search: test ranges
for test_end in [100, 200, 300, 400, 450, 475, 476, 477]:
    try:
        compile(b'\n'.join(lines[:test_end]), '<test>', 'exec')
        print(f"Lines 1-{test_end}: OK")
    except SyntaxError as e:
        print(f"Lines 1-{test_end}: FAIL at line {e.lineno}: {e.msg}")
        print(f"  Text: {repr(e.text)}")
        break
