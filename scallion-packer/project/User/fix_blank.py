# -*- coding: utf-8 -*-
# Remove blank lines with odd indentation from final_edit.py
f = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\final_edit.py"
with open(f, 'rb') as fh:
    raw = fh.read()
lines = raw.split(b'\n')

# Remove blank lines (0 or 1 spaces only) that are between the triple-quoted string
# and the code that follows
# Keep lines 1-473, skip line 474 if it's empty or has wrong indent, then add lines
# Actually, the problem is line 475: blank with 1-space indent

# Find the blank line with 1-space indent
for i in range(473, 478):
    line = lines[i]
    stripped = line.rstrip(b'\r\n')
    if len(stripped) == 0 or (len(stripped) == 1 and stripped in b' \t'):
        print(f"Blank/WS-only line {i+1}: {repr(line)}")

# Check: what if we remove all lines that are ONLY whitespace?
cleaned = []
for i, line in enumerate(lines):
    stripped = line.rstrip(b'\r\n')
    # Keep lines that have content, or blank lines with 0 indent
    if len(stripped) > 0:
        cleaned.append(line)
    else:
        # Blank line - check if it has indentation
        if len(line) > 0 and line[0:1] in [b' ', b'\t']:
            # Has leading whitespace but no content
            # Skip it (remove the blank indented line)
            print(f"Removing blank indented line {i+1}: {repr(line)}")
        else:
            cleaned.append(line)

print(f"Original: {len(lines)} lines, Cleaned: {len(cleaned)} lines")

# Write and compile test
test_raw = b'\n'.join(cleaned)
try:
    compile(test_raw, '<test>', 'exec')
    print("Cleaned script compiles OK!")
except SyntaxError as e:
    print(f"Cleaned script error at line {e.lineno}: {e.msg}")
    lines2 = test_raw.split(b'\n')
    for j in range(max(0, e.lineno-5), min(len(lines2), e.lineno+3)):
        print(f"  L{j+1}: {repr(lines2[j][:80])}")

# Write if OK
with open(f, 'wb') as fh:
    fh.write(test_raw)
print("Written")
