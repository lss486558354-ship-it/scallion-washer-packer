# -*- coding: utf-8 -*-
f = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\final_edit.py"
with open(f, 'rb') as fh:
    raw = fh.read()
lines = raw.split(b'\n')

# Test each line 474-480 individually
for test_end in range(474, 482):
    try:
        compile(b'\n'.join(lines[:test_end]), '<test>', 'exec')
        print(f"Lines 1-{test_end}: OK")
    except SyntaxError as e:
        print(f"Lines 1-{test_end}: FAIL line {e.lineno}: {e.msg}")
        break
