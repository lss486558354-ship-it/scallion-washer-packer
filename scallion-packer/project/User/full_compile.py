# -*- coding: utf-8 -*-
f = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\final_edit.py"
with open(f, 'rb') as fh:
    raw = fh.read()

try:
    compile(raw, '<full>', 'exec')
    print("Full script compiles OK!")
except SyntaxError as e:
    print(f"SyntaxError at line {e.lineno}: {e.msg}")
    print(f"Offset: {e.offset}")
    lines = raw.split(b'\n')
    for i in range(max(0, e.lineno-5), min(len(lines), e.lineno+3)):
        line = lines[i]
        stripped = line.lstrip()
        indent = len(line) - len(stripped)
        try:
            content = line.decode('utf-8', errors='replace')
        except:
            content = repr(line)
        print(f"  L{i+1:3d} (indent={indent}): {repr(content[:100])}")
