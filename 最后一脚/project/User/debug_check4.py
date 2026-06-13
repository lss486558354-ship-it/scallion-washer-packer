# -*- coding: utf-8 -*-
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c'
with open(f, 'rb') as fh:
    raw = fh.read()

lines = raw.split(b'\n')

depth = 0
max_depth = 0
for i, line in enumerate(lines):
    # Count braces in this line (ignore chars in strings/comments roughly)
    # Simple approach: count all { and } not inside string literals
    s = line.decode('utf-8', errors='replace')
    # Remove line comments
    if '//' in s:
        s = s[:s.index('//')]
    # Count braces
    open_d = s.count('{')
    close_d = s.count('}')
    depth += open_d - close_d
    if depth > max_depth:
        max_depth = depth
    if depth < 0:
        print(f'  L{i+1} (depth={depth}): NEGATIVE! Line: {repr(line.strip()[:60])}')
    if depth == 0 and i > 0 and i < 1000:
        pass  # OK
    # Show lines where depth is 0 but we have unclosed blocks
    # Specifically track when we hit 0
    if i < 1000:  # Only early part for analysis
        if depth == 0:
            pass  # All good

print(f'Max nesting depth: {max_depth}')
print(f'Final depth: {depth} (should be 0)')

# Now find the exact problem area: where depth becomes negative or where it stays > 0 at the end
# Reset and find the specific area
depth = 0
problem_area = []
for i, line in enumerate(lines):
    s = line.decode('utf-8', errors='replace')
    if '//' in s:
        s = s[:s.index('//')]
    open_d = s.count('{')
    close_d = s.count('}')
    old_depth = depth
    depth += open_d - close_d

    # Track if we're in a suspicious area
    if i >= 390 and i <= 975:
        if old_depth > 0 and depth == 0:
            # This is where we unexpectedly close to 0
            problem_area.append(('CLOSE_TO_ZERO', i+1, old_depth, repr(line.strip()[:60])))
        if depth < 0:
            problem_area.append(('NEGATIVE', i+1, depth, repr(line.strip()[:60])))

for item in problem_area:
    print(item)

# Show brace depth around lines 940-970
print('\nBrace depth around key area (lines 930-975):')
depth = 0
for i, line in enumerate(lines):
    if i < 930:
        s = line.decode('utf-8', errors='replace')
        if '//' in s:
            s = s[:s.index('//')]
        depth += s.count('{') - s.count('}')
        continue
    if i > 974:
        break
    s = line.decode('utf-8', errors='replace')
    orig = s
    if '//' in s:
        s = s[:s.index('//')]
    old_depth = depth
    depth += s.count('{') - s.count('}')
    print(f'  L{i+1} depth={depth} (delta={depth-old_depth:+d}): {repr(orig.strip()[:60])}')
