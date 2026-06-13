# -*- coding: utf-8 -*-
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c'
with open(f, 'rb') as fh:
    raw = fh.read()

lines = raw.split(b'\n')
# Find all function definitions
print("=== Functions (lines starting with 'static' or return type) ===")
for i, line in enumerate(lines):
    stripped = line.strip()
    if stripped.startswith(b'static ') or stripped.startswith(b'void ') or stripped.startswith(b'uint16_t '):
        if b'(' in line:
            print(f'  L{i+1}: {line.strip()[:80]}')

print()
# Find all standalone '}' lines (braces that close a block)
print("=== Standalone '}' lines ===")
for i, line in enumerate(lines):
    stripped = line.strip()
    if stripped == b'}' or stripped == b'};':
        print(f'  L{i+1}: {repr(stripped)}')
