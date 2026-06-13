# -*- coding: utf-8 -*-
f = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\final_edit.py"
with open(f, 'rb') as fh:
    raw = fh.read()

# Try to compile the script to find the actual error
try:
    compile(raw, f, 'exec')
    print("Script compiles OK!")
except SyntaxError as e:
    print(f"SyntaxError at line {e.lineno}: {e.msg}")
    print(f"Text: {repr(e.text)}")
    print(f"Offset: {e.offset}")
    # Show the line with context
    lines = raw.split(b'\n')
    for i in range(max(0, e.lineno-5), min(len(lines), e.lineno+2)):
        print(f"  L{i+1}: {repr(lines[i])}")
