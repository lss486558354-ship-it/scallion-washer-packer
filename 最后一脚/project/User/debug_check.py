# -*- coding: utf-8 -*-
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c'
with open(f, 'rb') as fh:
    raw = fh.read()

lines = raw.split(b'\n')
print(f'Total newlines: {len(lines)}')
print(f'Line 978 (idx 977): {repr(lines[977])}')
print(f'Line 979 (idx 978): {repr(lines[978])}')
print(f'Line 980 (idx 979): {repr(lines[979])}')

# Check if 'static void delay_us' is in the raw content
target = b'static void delay_us(uint32_t us)'
print(f'delay_us decl found: {target in raw}')

# Find all positions of '/*' and '*/' in the file
count_star_slash = raw.count(b'*/')
print(f'Total occurrences of "*/": {count_star_slash}')

# Find first 5 and last 5 occurrences of '*/'
positions = [i for i in range(len(raw)) if raw[i:i+2] == b'*/']
print(f'First 5 "*/" positions: {positions[:5]}')
print(f'Last 5 "*/" positions: {positions[-5:]}')
