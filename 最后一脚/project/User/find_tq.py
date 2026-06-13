# Find all triple-quote occurrences
raw = open(r'C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\final_edit.py', 'rb').read()
pos = 0
count = 0
while True:
    idx = raw.find(b"'", pos)
    if idx < 0 or idx + 2 >= len(raw):
        print(f'No more single quotes after pos {pos}')
        break
    next3 = raw[idx:idx+3]
    if next3 == b"'''":
        line_num = raw[:idx].count(b'\n') + 1
        print(f"Found triple-quote at byte {idx}, line {line_num}")
        pos = idx + 3
    else:
        pos = idx + 1
    count += 1
    if count > 200:
        break
