f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\final_edit.py'
raw = open(f, 'rb').read()
lines = raw.split(b'\n')
print(f"Total lines: {len(lines)}")
for i in range(469, 485):
    if i < len(lines):
        print(f"L{i+1}: {repr(lines[i])}")
