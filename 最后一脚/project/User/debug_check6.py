# -*- coding: utf-8 -*-
# Find unmatched braces: track depth, print transitions
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c'
with open(f, 'rb') as fh:
    raw = fh.read()

lines = raw.split(b'\n')
in_comment = False
depth = 0

for i, line in enumerate(lines):
    j = 0
    changes = []
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
                changes.append(('+', j, depth))
            elif line[j:j+1] == b'}':
                depth -= 1
                changes.append(('-', j, depth))
        j += 1

    # Show lines where depth changes
    if changes:
        content = line.strip()[:60].decode('utf-8', errors='replace')
        for action, col, d in changes:
            print(f'L{i+1:4d} [{action}] depth={d}: {content}')

print(f'\nFinal depth: {depth}')
