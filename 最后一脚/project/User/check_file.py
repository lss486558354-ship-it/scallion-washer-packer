# -*- coding: utf-8 -*-
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c'
with open(f, 'r', encoding='utf-8') as fh:
    content = fh.read()

# Count total braces in the raw file
raw = open(f, 'rb').read()
open_braces = raw.count(b'{')
close_braces = raw.count(b'}')
open_comments = raw.count(b'/*')
close_comments = raw.count(b'*/')

print(f'File size: {len(content)} chars')
print(f'Raw braces: {{ = {open_braces}, }} = {close_braces}, balance = {open_braces - close_braces}')
print(f'Raw comments: /* = {open_comments}, */ = {close_comments}, balance = {open_comments - close_comments}')

# Check first multi-line comment
idx = content.find('/*')
if idx >= 0:
    nl_before = content[:idx].count('\n')
    print(f'First /* at index {idx}, line {nl_before+1}')
idx2 = content.find('*/')
if idx2 >= 0:
    nl_before = content[:idx2].count('\n')
    print(f'First */ at index {idx2}, line {nl_before+1}')

# Check end of file
print(f'Last 30 chars: {repr(content[-30:])}')
