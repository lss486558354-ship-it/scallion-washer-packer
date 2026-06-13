# -*- coding: utf-8 -*-
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c'
with open(f, 'rb') as fh:
    raw = fh.read()

lines = raw.split(b'\n')
print(f'Total newlines: {len(lines)}')
print(f'Lines 973-982:')
for i in range(972, 982):
    print(f'  idx {i+1}: {repr(lines[i])}')

# Count opening and closing braces in the whole file
open_count = raw.count(b'{')
close_count = raw.count(b'}')
print(f'Open braces: {open_count}, Close braces: {close_count}')

# Check function declaration line
decl = b'static void delay_us(uint32_t us)'
idx = raw.find(decl)
if idx >= 0:
    # Count newlines before this position
    before = raw[:idx]
    lines_before = before.count(b'\n')
    print(f'delay_us found at byte {idx}, line {lines_before + 1}')
