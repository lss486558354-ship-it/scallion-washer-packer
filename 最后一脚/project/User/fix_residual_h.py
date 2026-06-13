# -*- coding: utf-8 -*-
# Remove old partition H state machine code from main.c
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c'
with open(f, 'r', encoding='utf-8') as fh:
    lines = fh.readlines()

print(f"Before: {len(lines)} lines")

# Find the boundary: after "/* 阻塞微秒延时" is the new code
# Find: "/* 阻塞微秒延时" (first occurrence after the /* ===== line)
new_code_start = None
for i, line in enumerate(lines):
    if '/* 阻塞微秒延时' in line:
        new_code_start = i
        break

# Find the boundary: "    /* =====" before it is part of the multi-line block
# We need to delete from the line just before "/* 阻塞微秒延时" up to and including the
# line before the "    /* ===" line that starts the new code

# Find the /* ==== line immediately before new_code_start
comment_end = None
for i in range(new_code_start - 1, -1, -1):
    stripped = lines[i].strip()
    if stripped.startswith('/*') or stripped.startswith('*') or stripped.startswith('*/'):
        continue
    if stripped.startswith('*') or stripped.startswith('/*'):
        continue
    if '/* =' in lines[i] or '====' in lines[i]:
        comment_end = i
        break

if comment_end is None:
    # Try another approach: find the line with "     * ==="
    for i in range(new_code_start - 1, -1, -1):
        if lines[i].strip().startswith('*'):
            continue
        if '/* ===' in lines[i] or '====' in lines[i]:
            comment_end = i
            break

print(f"New code starts at line {new_code_start+1}, comment end around line {comment_end+1 if comment_end else 'N/A'}")
print(f"Line before new code: {repr(lines[comment_end] if comment_end else 'N/A')}")

# Actually, let's find the exact boundary differently
# The new state machine ends with the block that contains
# "if (st >= PROC_NEW_PACK_IMPELLER && st <= PROC_NEW_PACK_STOP)"
# Then there's the closing brace of the BSP_Tick hook
# Then there's partition G (ultra test)
# Then there's the old partition H code

# Find the line that says "/* 阻塞微秒延时" - that's the start of the new helper functions
# Find the line that says "     * ===" just before "/* 阻塞微秒延时"
for i in range(new_code_start - 1, -1, -1):
    if lines[i].strip().startswith('*'):
        continue
    if lines[i].strip().startswith('/'):
        print(f"Found: L{i+1}: {repr(lines[i][:60])}")
        break

# The old code starts at "         *   - 使能叶轮扭矩"
# and ends at "    }" just before the "/* ===" of the new helper functions

# Let me find where the old code actually starts and ends
for i in range(new_code_start - 1, max(0, new_code_start - 20), -1):
    print(f"L{i+1}: {repr(lines[i][:70])}")
