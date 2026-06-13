#!/usr/bin/env python3
"""
Fix the main.c file - add missing /* for partition G comment.
"""

src_path = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c"
with open(src_path, 'r', encoding='utf-8') as f:
    content = f.read()

lines = content.split('\n')
print(f"Current file: {len(lines)} lines")

# Fix line 926 which starts with "     * " instead of "/* "
for i, line in enumerate(lines):
    if '     * 【分区 G】调试' in line:
        lines[i] = '    /*' + line[4:]  # Replace "     *" with "    /*"
        print(f"Fixed missing /* at line {i+1}")
        break

# Reconstruct
content = '\n'.join(lines)

# Verification
open_braces = content.count('{')
close_braces = content.count('}')
open_comments = content.count('/*')
close_comments = content.count('*/')

print(f"\n=== VERIFICATION ===")
print(f"Braces: {{ = {open_braces}, }} = {close_braces}, BALANCED" if open_braces == close_braces else f"Braces: UNBALANCED! {open_braces}/{close_braces}")
print(f"Comments: /* = {open_comments}, */ = {close_comments}, BALANCED" if open_comments == close_comments else f"Comments: UNBALANCED! {open_comments}/{close_comments}")
ends_with_brace = content.rstrip().endswith('}')
print(f"Ends with closing brace: {'YES' if ends_with_brace else 'NO!'}")

# Write output
with open(src_path, 'w', encoding='utf-8') as f:
    f.write(content)

print(f"\nOutput written to: {src_path}")
