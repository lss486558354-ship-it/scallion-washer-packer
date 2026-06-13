# -*- coding: utf-8 -*-
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c'
with open(f, 'rb') as fh:
    raw = fh.read()

# Manual brace counting: track nesting, ignore braces inside multi-line comments
lines = raw.split(b'\n')

in_comment = False
depth = 0
depth_history = []
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
                if depth < 0:
                    print(f'L{i+1}: NEGATIVE DEPTH ({depth})! Line: {line.strip()[:60]}')
        j += 1

    depth_history.append((i+1, depth, line.strip()[:50]))

print(f'Final depth: {depth} (should be 0)')
print(f'Max depth: {max(d for _,d,_ in depth_history)}')

# Find the specific area where we lose a brace
print('\nDepth changes in lines 390-395:')
for ln, d, content in depth_history[389:396]:
    print(f'  L{ln} depth={d}: {content[:60]}')

print('\nDepth changes in lines 930-975:')
for ln, d, content in depth_history[929:975]:
    print(f'  L{ln} depth={d}: {content[:60]}')
