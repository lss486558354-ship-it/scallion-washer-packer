# -*- coding: utf-8 -*-
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c'
with open(f, 'r', encoding='utf-8') as fh:
    lines = fh.readlines()

print(f"Before: {len(lines)} lines")
# Show what we'd get
for i in range(970, 976):
    print(f"  L{i+1}: {repr(lines[i][:70])}")

print("After deletion of 973-1133:")
# Pretend deletion and show
result = lines[:972] + lines[1133:]
for i in range(970, 980):
    if i < len(result):
        print(f"  L{i+1}: {repr(result[i][:70])}")
