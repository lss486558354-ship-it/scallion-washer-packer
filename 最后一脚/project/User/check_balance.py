# -*- coding: utf-8 -*-
f = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c"
with open(f, 'rb') as fh:
    raw = fh.read()
lines = raw.split(b'\n')
in_comment = False
depth = 0
for i, line in enumerate(lines):
    j = 0
    while j < len(line):
        c1 = line[j:j+2]
        if not in_comment and c1 == b'/*': in_comment = True; j += 2; continue
        if in_comment and c1 == b'*/': in_comment = False; j += 2; continue
        if not in_comment:
            if line[j:j+1] == b'{': depth += 1
            elif line[j:j+1] == b'}': depth -= 1
        j += 1
print(f"Total: {len(lines)} lines, Final depth: {depth}")
if depth != 0:
    print(f"ERROR: Unbalanced!")
else:
    print("OK!")
