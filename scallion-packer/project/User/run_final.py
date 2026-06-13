# -*- coding: utf-8 -*-
# FIXED PATH VERSION
import sys
sys.stdout.reconfigure(encoding='utf-8')

backup_path = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00 - 副本 (2)\project\User\main.c"
main_path = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c"

with open(backup_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

print(f"Backup: {len(lines)} lines")
raw = open(backup_path, 'rb').read()
print(f"Backup braces: {{={raw.count(b'{')} }}={raw.count(b'}')}")

# (same content as final_edit.py but with corrected main_path)
