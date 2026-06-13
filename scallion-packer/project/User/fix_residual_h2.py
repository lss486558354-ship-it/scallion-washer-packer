# -*- coding: utf-8 -*-
# Remove lines 973-1133 (old partition H code) from main.c
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c'
with open(f, 'r', encoding='utf-8') as fh:
    lines = fh.readlines()

print(f"Before: {len(lines)} lines")

# Lines 973-1133 = indices 972-1132 (old partition H code)
# Keep 0-972, skip 973-1133, keep 1134+
result = lines[:972] + lines[1133:]

# Verify brace balance after deletion
in_comment = False
depth = 0
for i, line in enumerate(result):
    j = 0
    while j < len(line):
        c1 = line[j:j+2]
        if not in_comment and c1 == '/*':
            in_comment = True
            j += 2
            continue
        if in_comment and c1 == '*/':
            in_comment = False
            j += 2
            continue
        if not in_comment:
            if line[j] == '{':
                depth += 1
            elif line[j] == '}':
                depth -= 1
        j += 1

print(f"After: {len(result)} lines, brace depth={depth}")

# Show transition
for i in range(970, 978):
    print(f"L{i+1}: {repr(result[i][:70])}")

with open(f, 'w', encoding='utf-8') as fh:
    fh.writelines(result)
print("Written")
