# -*- coding: utf-8 -*-
# Compare brace structure between backup and current version
backup_path = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00 - 副本 (2)\project\User\main.c"
current_path = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c"

def count_braces(path):
    with open(path, 'rb') as f:
        raw = f.read()
    ob = raw.count(b'{')
    cb = raw.count(b'}')
    oc = raw.count(b'/*')
    cc = raw.count(b'*/')
    return ob, cb, oc, cc, raw.count(b'\n') + 1

b_ob, b_cb, b_oc, b_cc, b_lines = count_braces(backup_path)
c_ob, c_cb, c_oc, c_cc, c_lines = count_braces(current_path)

print(f"Backup:  {{={b_ob} }}={b_cb} balance={b_ob-b_cb}, comments={{{b_oc} */={b_cc} bal={b_oc-b_cc}, lines={b_lines}")
print(f"Current: {{={c_ob} }}={c_cb} balance={c_ob-c_cb}, comments={{{c_oc} */={c_cc} bal={c_oc-c_cc}, lines={c_lines}")
print(f"Diff:    {{+{c_ob-b_ob} }}+{c_cb-b_cb} brace_bal={c_ob-c_cb-b_ob+b_cb}, comments +{c_oc-b_oc} -diff{c_cc-b_cc}")
