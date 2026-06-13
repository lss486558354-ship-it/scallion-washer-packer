f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\final_edit.py'
raw = open(f, 'rb').read()
lines = raw.split(b'\n')
print(f"Total: {len(lines)} lines")
for i in range(470, 480):
    if i < len(lines):
        line = lines[i]
        print(f"L{i+1:3d} (indent={len(line) - len(line.lstrip())}): {repr(line)}")
