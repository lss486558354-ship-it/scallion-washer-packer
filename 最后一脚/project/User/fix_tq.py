# -*- coding: utf-8 -*-
# Fix the indentation of the closing triple-quote in final_edit.py
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\final_edit.py'
raw = open(f, 'rb').read()
lines = raw.split(b'\n')
# Line 474 (index 473) should be b"'''" not b"    '''"
if lines[473] == b"    '''":
    lines[473] = b"'''"
    print(f"Fixed line 474: {repr(lines[473])}")
else:
    print(f"Line 474 is: {repr(lines[473])}")
with open(f, 'wb') as fh:
    fh.write(b'\n'.join(lines))
print("Written")
