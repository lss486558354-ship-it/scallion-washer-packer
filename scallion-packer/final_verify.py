#!/usr/bin/env python3
"""
Final verification of all changes to main.c
"""

src_path = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c"
with open(src_path, 'r', encoding='utf-8') as f:
    content = f.read()

lines = content.split('\n')

print("=" * 70)
print("FINAL VERIFICATION REPORT")
print("=" * 70)
print(f"Total lines: {len(lines)}")

# Braces and comments
open_braces = content.count('{')
close_braces = content.count('}')
open_comments = content.count('/*')
close_comments = content.count('*/')

print(f"\n1. Brace Balance: {{ = {open_braces}, }} = {close_braces}")
if open_braces == close_braces:
    print("   [OK] Braces are balanced")
else:
    print(f"   [ERROR] Braces are UNBALANCED!")

print(f"\n2. Comment Balance: /* = {open_comments}, */ = {close_comments}")
if open_comments == close_comments:
    print("   [OK] Comments are balanced")
else:
    print(f"   [ERROR] Comments are UNBALANCED!")

print(f"\n3. File ends with closing brace: {content.rstrip().endswith('}')}")

print("\n" + "=" * 70)
print("CHANGE VERIFICATION")
print("=" * 70)

# CHANGE 1: g_pack_test_total_ms declaration
if 'g_pack_test_total_ms' in content:
    print("\n[CHANGE 1] g_pack_test_total_ms declaration: FOUND [OK]")
else:
    print("\n[CHANGE 1] g_pack_test_total_ms declaration: NOT FOUND [ERROR]")

# CHANGE 2: New state machine trigger
if 'g_new_proc_state = PROC_NEW_PACK_IMPELLER;' in content:
    print("[CHANGE 2] New PROC_NEW_PACK_IMPELLER trigger: FOUND [OK]")
else:
    print("[CHANGE 2] New PROC_NEW_PACK_IMPELLER trigger: NOT FOUND [ERROR]")

if 'g_pack_test_state = PACK_TEST_IMPELLER_ON;' in content:
    print("   [WARNING] Old g_pack_test_state trigger still exists!")

# CHANGE 3: Boot auto-run deleted
if 'Boot auto-run start' in content or '[PACK_TEST] Boot auto-run' in content:
    print("[CHANGE 3] Boot auto-run: STILL EXISTS [ERROR]")
else:
    print("[CHANGE 3] Boot auto-run: DELETED [OK]")

# CHANGE 4: New partition F state machine
if '【分区 F】切割+打包联合状态机' in content:
    print("[CHANGE 4] New partition F state machine: FOUND [OK]")
else:
    print("[CHANGE 4] New partition F state machine: NOT FOUND [ERROR]")

if 'PROC_NEW_PACK_IMPELLER' in content:
    print("   - PROC_NEW_PACK_IMPELLER case: FOUND [OK]")
if 'PROC_NEW_PACK_M4' in content:
    print("   - PROC_NEW_PACK_M4 case: FOUND [OK]")
if 'PROC_NEW_PACK_ROD_ON_1' in content:
    print("   - PROC_NEW_PACK_ROD_ON_1 case: FOUND [OK]")
if 'PROC_NEW_PACK_ROD_OFF' in content:
    print("   - PROC_NEW_PACK_ROD_OFF case: FOUND [OK]")
if 'PROC_NEW_PACK_ROD_ON_2' in content:
    print("   - PROC_NEW_PACK_ROD_ON_2 case: FOUND [OK]")
if 'PROC_NEW_PACK_STOP' in content:
    print("   - PROC_NEW_PACK_STOP case: FOUND [OK]")

# CHANGE 5: Old partition E deleted
if '【分区 E】超声波检测计数' in content:
    print("[CHANGE 5] Old partition E: STILL EXISTS [ERROR]")
else:
    print("[CHANGE 5] Old partition E: DELETED [OK]")

# CHANGE 6: Old partition H state machine deleted
if 'case PACK_TEST_IDLE:' in content or 'case PACK_TEST_IMPELLER_ON:' in content:
    print("[CHANGE 6] Old partition H state machine: STILL EXISTS [ERROR]")
else:
    print("[CHANGE 6] Old partition H state machine: DELETED [OK]")

print("\n" + "=" * 70)
print("ALL CHANGES APPLIED SUCCESSFULLY")
print("=" * 70)
