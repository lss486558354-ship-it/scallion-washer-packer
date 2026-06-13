# -*- coding: utf-8 -*-
# Full brace tracking with better nesting display
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c'
with open(f, 'rb') as fh:
    raw = fh.read()

lines = raw.split(b'\n')
in_comment = False
depth = 0
func_stack = []

# Find all function definitions
funcs = {}
for i, line in enumerate(lines):
    s = line.decode('utf-8', errors='replace').strip()
    # Function definitions (not declarations)
    if ('(' in s and ')' in s and
        not s.startswith('//') and not s.startswith('/*') and
        not 'typedef' in s and not 'struct' in s and
        not '#define' in s and not 'extern' in s and
        s.endswith('{') or
        (not ';' in s and not '//' in s and
         '(' in s and ')' in s and
         s.endswith('{'))):
        # Heuristic: line contains 'void' or 'uint' or 'static' followed by '('
        for kw in ['void ', 'uint16_t ', 'uint32_t ', 'int16_t ', 'int ', 'static void ', 'static uint', 'static int']:
            if kw in s and '(' in s:
                name = s.split('(')[0].strip().split()[-1]
                funcs[i+1] = name
                break

# Now track braces
depth = 0
for i, line in enumerate(lines):
    j = 0
    while j < len(line):
        c1 = line[j:j+2]
        if not in_comment and c1 == b'/*':
            in_comment = True
            j += 2
            continue
        if in_comment and c1 == b'*/':
            in_comment = False
            j += 2
            continue
        if not in_comment:
            if line[j:j+1] == b'{':
                depth += 1
            elif line[j:j+1] == b'}':
                depth -= 1
        j += 1

    # Show lines 398 (start of BSP_Tick_AppHook1ms) to 410 (after init code)
    if 398 <= i+1 <= 410:
        ln = i+1
        name = funcs.get(ln, '')
        print(f'L{ln:4d} depth={depth} {"<-- " + name if name else ""}')
        if ln == 398:
            print('--- start of BSP_Tick_AppHook1ms ---')
        if ln == 410:
            print('---')

    # Show end of file
    if i+1 >= 1430:
        ln = i+1
        name = funcs.get(ln, '')
        print(f'L{ln:4d} depth={depth} {"<-- " + name if name else ""}')

print(f'\nFinal depth: {depth}')
