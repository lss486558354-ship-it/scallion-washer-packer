# -*- coding: utf-8 -*-
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c'
with open(f, 'rb') as fh:
    raw = fh.read()
lines = raw.split(b'\n')
in_comment = False
depth = 0
max_depth = 0
transitions = []
for i, line in enumerate(lines):
    old = depth
    j = 0
    while j < len(line):
        c1 = line[j:j+2]
        if not in_comment and c1 == b'/*':
            in_comment = True; j += 2; continue
        if in_comment and c1 == b'*/':
            in_comment = False; j += 2; continue
        if not in_comment:
            if line[j:j+1] == b'{': depth += 1
            elif line[j:j+1] == b'}': depth -= 1
        j += 1
    if depth > max_depth: max_depth = depth
    if old != depth or depth < 0:
        transitions.append((i+1, old, depth, line.strip()[:50]))

print(f'Final: {depth}, Max: {max_depth}')
for t in transitions[-30:]:
    ln, old, d, content = t
    try:
        print(f'L{ln:4d}: {old} -> {d}: {content.decode("utf-8", errors="replace")}')
    except:
        print(f'L{ln:4d}: {old} -> {d}: {content}')
