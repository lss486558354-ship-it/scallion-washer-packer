#!/usr/bin/env python3
"""
Find unbalanced comments in main.c
"""

src_path = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c"
with open(src_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

open_count = 0
for i, line in enumerate(lines):
    opens = line.count('/*')
    closes = line.count('*/')
    net = opens - closes

    if net != 0:
        print(f"Line {i+1}: opens={opens} closes={closes} -> net={net}")
        print(f"  {line[:80].rstrip()}")

    open_count += net
    if open_count < 0:
        print(f"  WARNING: Negative balance at line {i+1}!")
        open_count = 0  # Reset to continue finding other issues

print(f"\nFinal balance: {open_count}")
