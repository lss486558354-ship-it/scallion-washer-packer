# Read and analyze the script
f = r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\final_edit.py'
with open(f, 'rb') as fh:
    raw = fh.read()
lines = raw.split(b'\n')

# Find lines 472-477 with indentation info
for i in range(471, 478):
    if i < len(lines):
        line = lines[i]
        stripped = line.lstrip()
        indent = len(line) - len(stripped)
        try:
            content = line.decode('utf-8', errors='replace')
        except:
            content = repr(line)
        print(f"L{i+1:3d} indent={indent}: {repr(content[:80])}")
