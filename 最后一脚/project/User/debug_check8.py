# -*- coding: utf-8 -*-
# Full analysis: find where the depth=1 at end comes from
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c'
with open(f, 'rb') as fh:
    raw = fh.read()

lines = raw.split(b'\n')
in_comment = False
depth = 0

print("=== Lines where depth is 0 (top-level) ===")
for i, line in enumerate(lines):
    j = 0
    while j < len(line):
        c1 = line[j:j+2]
        if not in_comment and c1 == b'/*':
            in_comment = True
            j += 2
            continue
        if in_comment and c1 == b'*/':
            in_comment = False
            j += 2
            continue
        if not in_comment:
            if line[j:j+1] == b'{':
                depth += 1
            elif line[j:j+1] == b'}':
                depth -= 1
        j += 1

    if depth == 0:
        content = line.strip().decode('utf-8', errors='replace')
        if content:
            print(f'  L{i+1}: {content[:80]}')

print(f'\nFinal depth: {depth}')
