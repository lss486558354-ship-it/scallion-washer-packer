#!/usr/bin/env python3
"""
STS3020 舵机调试助手 v3
- 速度控制（轮模式）+ 位置控制（关节模式）+ 寄存器读写
- 日志同时记录TX和RX，自动保存到临时文件
"""

import os, sys
import serial
import serial.tools.list_ports
import struct
import time
import threading
import csv
import tempfile
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional, List

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext, filedialog
except ImportError:
    print("请安装 tkinter")
    exit(1)


# ============================================================================
# 协议定义
# ============================================================================

SCS_HEADER  = 0xFF
SCS_INST_WRITE    = 0x03
SCS_INST_READ    = 0x02
SCS_INST_PING    = 0x01

# STS 寄存器
ADDR_TORQUE       = 0x28
ADDR_ACC_SPEED    = 0x29  # ACC(1B) + 速度(2B) = 3字节
ADDR_GOAL_POS_L   = 0x2A  # 目标位置 2字节
ADDR_GOAL_SPEED_L = 0x2E  # 目标速度 2字节
ADDR_MODE         = 0x33  # 运行模式(ROM)
ADDR_CUR_POS_L    = 0x38  # 当前位置L
ADDR_LOCK         = 0x55  # EPROM锁定

MODE_WHEEL  = 1   # 轮模式：连续旋转
MODE_JOINT  = 0   # 关节模式：位置控制

SERVO_RESOLUTION = 4096


def calc_checksum(data: bytes) -> int:
    return (~sum(data)) & 0xFF


def build_packet(servo_id: int, instruction: int, params: bytes = b"") -> bytes:
    length = 2 + len(params)
    header = bytes([SCS_HEADER, SCS_HEADER, servo_id, length, instruction])
    pkt = header + params
    pkt += bytes([calc_checksum(pkt[2:])])
    return pkt


def parse_response(data: bytes) -> dict:
    r = {"valid": False, "raw_hex": data.hex().upper(), "error": None,
         "id": None, "length": None, "error_code": None, "params": b"", "checksum": None}

    if len(data) < 6:
        r["error"] = f"太短({len(data)}B)"
        return r
    if data[0] != 0xFF or data[1] != 0xFF:
        r["error"] = "帧头错误"
        return r

    r["id"]         = data[2]
    r["length"]     = data[3]
    r["error_code"] = data[4]
    r["params"]     = data[5:-1] if len(data) > 6 else b""
    r["checksum"]   = data[-1]

    calc_cs = calc_checksum(data[2:-1])
    r["checksum_ok"] = (calc_cs == r["checksum"])

    err_map = {1:"指令错误", 2:"未定义指令", 3:"ID超范围", 4:"参数错误",
               5:"寄存器锁定", 6:"EEPROM繁忙", 21:"位置超限", 23:"温度过高", 26:"电压异常"}
    if r["error_code"] != 0:
        r["error"] = err_map.get(r["error_code"], f"错误{r['error_code']}")

    r["valid"] = r["checksum_ok"] and r["error_code"] == 0
    return r


def describe_hex(data: bytes) -> str:
    """把Hex转成可读描述"""
    if len(data) < 5:
        return data.hex().upper()
    inst_names = {0x01:"PING", 0x02:"READ", 0x03:"WRITE", 0x04:"REG_WRITE", 0x05:"ACTION", 0x0C:"READ"}
    inst = data[4]
    parts = [f"FF FF", f"ID={data[2]}", f"Len={data[3]}",
             f"Inst=0x{inst:02X}({inst_names.get(inst,'?')})"]
    if inst == 0x03 and len(data) >= 7:
        parts.append(f"Addr=0x{data[5]:02X} Data={data[6:-1].hex().upper()}")
    if inst == 0x02 and len(data) >= 7:
        parts.append(f"Read 0x{data[5]:02X} len={data[6]}")
    return " | ".join(parts)


# ============================================================================
# 日志记录器（TX和RX都记录）
# ============================================================================

class SessionLogger:
    def __init__(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.txt_path = os.path.join(tempfile.gettempdir(), f"servo_log_{ts}.txt")
        self.csv_path = os.path.join(tempfile.gettempdir(), f"servo_log_{ts}.csv")
        self.entries: List[dict] = []
        self._write_txt("=== STS3020 调试会话 ===")
        self._write_txt(f"开始: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    def log_tx(self, raw_hex: str, note: str = ""):
        entry = {
            "ts": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "dir": "TX", "hex": raw_hex,
            "id": "-", "error": "", "params": "", "valid": "", "note": note,
        }
        self.entries.append(entry)
        self._write_txt(f"[TX] {raw_hex} {note}")

    def log_rx(self, raw_hex: str, parsed: dict, note: str = ""):
        err = parsed.get("error", "")
        valid = parsed.get("valid", False)
        params = parsed.get("params", b"")
        params_hex = params.hex().upper() if params else ""
        entry = {
            "ts": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "dir": "RX", "hex": raw_hex,
            "id": str(parsed.get("id", "-")),
            "error": err, "params": params_hex,
            "valid": valid, "note": note,
        }
        self.entries.append(entry)
        status = "OK" if valid else f"ERR:{err}"
        self._write_txt(f"[RX] {raw_hex} [{status}] {params_hex} {note}")

    def log_servo_speed(self, sid: int, raw_speed: int, reg_hex: str):
        entry = {
            "ts": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "dir": "TX", "hex": "",
            "id": str(sid), "error": "", "params": f"raw={raw_speed}", "valid": "", "note": f"速度 reg={reg_hex}",
        }
        self.entries.append(entry)
        self._write_txt(f"[SERVO_SPEED] ID={sid} raw={raw_speed} reg={reg_hex}")



    def log_scan_ping(self, baud: int, sid: int, hex_str: str):
        entry = {
            "ts": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "dir": "TX", "hex": hex_str,
            "id": str(sid), "error": "", "params": f"baud={baud}", "valid": "", "note": f"PING_ID",
        }
        self.entries.append(entry)
        self._write_txt(f"[SCAN] TX PING ID={sid} baud={baud} hex={hex_str}")

    def log_scan_found(self, baud: int, sid: int):
        entry = {
            "ts": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "dir": "RX", "hex": "FOUND",
            "id": str(sid), "error": "", "params": f"baud={baud}", "valid": "FOUND", "note": "舵机已找到",
        }
        self.entries.append(entry)
        self._write_txt(f"[SCAN] ** FOUND ** servo ID={sid} baud={baud}")

    def log_scan_summary(self, found: list):
        self._write_txt(f"[SCAN] === 扫描完成，找到 {len(found)} 个舵机 ===")
        for b, i in found:
            self._write_txt(f"[SCAN]   ★ ID={i} @ baud={b}")

    def _write_txt(self, line: str):
        with open(self.txt_path, "a", encoding="utf-8") as f:
            f.write(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {line}\n")

    def export(self, path: str):
        if path.endswith(".csv"):
            with open(path, "w", newline="", encoding="utf-8-sig") as f:
                w = csv.DictWriter(f, fieldnames=["ts","dir","hex","id","error","params","valid","note"])
                w.writeheader()
                w.writerows(self.entries)
        else:
            with open(self.txt_path, "r", encoding="utf-8") as src:
                content = src.read()
            with open(path, "w", encoding="utf-8") as dst:
                dst.write(content)


# ============================================================================
# 主应用
# ============================================================================

class ServoDebugTool:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("STS3020 舵机调试助手 v3")
        self.root.geometry("1100x780")
        self.root.minsize(900, 650)

        self.ser: Optional[serial.Serial] = None
        self.rx_thread: Optional[threading.Thread] = None
        self.running = False
        self.logger = SessionLogger()
        self.tx_count = 0
        self.rx_count = 0
        self.count_labels = []

        self._build_ui()
        self._refresh_ports()
        self._populate_commands()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(300, self._status_poll)

    # ------------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------------

    def _build_ui(self):
        nb = ttk.Notebook(self.root)
        nb.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        tab_speed = ttk.Frame(nb); nb.add(tab_speed, text="速度控制（轮模式）")
        tab_pos   = ttk.Frame(nb); nb.add(tab_pos,   text="位置控制（关节模式）")
        tab_reg   = ttk.Frame(nb); nb.add(tab_reg,   text="寄存器读写")
        tab_log   = ttk.Frame(nb); nb.add(tab_log,   text="通信日志")

        self._build_speed_tab(tab_speed)
        self._build_position_tab(tab_pos)
        self._build_register_tab(tab_reg)
        self._build_log_tab(tab_log)
        self._build_statusbar()

    # ---- 速度Tab ----
    def _build_speed_tab(self, parent):
        left = ttk.Frame(parent); left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        right = ttk.Frame(parent); right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 串口
        conn = ttk.LabelFrame(left, text="串口连接", padding=8); conn.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(conn, text="串口:").grid(row=0, column=0, sticky=tk.W)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=18, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5)
        ttk.Button(conn, text="刷新", command=self._refresh_ports).grid(row=0, column=2)
        ttk.Label(conn, text="波特率:").grid(row=1, column=0, sticky=tk.W, pady=(5,0))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(conn, textvariable=self.baud_var, width=18, state="readonly",
                     values=["9600","19200","38400","57600","115200","500000"]).grid(row=1,column=1,padx=5,pady=(5,0))
        self.connect_btn = ttk.Button(conn, text="连接", command=self._toggle_connection)
        self.connect_btn.grid(row=2, column=0, columnspan=3, pady=(8,0), sticky=tk.EW)
        self.status_label = ttk.Label(conn, text="未连接", foreground="red")
        self.status_label.grid(row=3, column=0, columnspan=3, pady=(4,0))

        # 速度参数
        ctrl = ttk.LabelFrame(left, text="速度参数", padding=8); ctrl.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(ctrl, text="舵机ID:").grid(row=0, column=0, sticky=tk.W)
        self.servo_id_var = tk.IntVar(value=1)
        ttk.Spinbox(ctrl, from_=0, to=254, textvariable=self.servo_id_var, width=8).grid(row=0,column=1,padx=5)
        ttk.Label(ctrl, text="加速度:").grid(row=1, column=0, sticky=tk.W, pady=(5,0))
        self.acc_var = tk.IntVar(value=0)
        ttk.Spinbox(ctrl, from_=0, to=254, textvariable=self.acc_var, width=8).grid(row=1,column=1,padx=5,pady=(5,0))

        # 速度滑块区
        slider_fr = ttk.LabelFrame(left, text="速度滑块", padding=8); slider_fr.pack(fill=tk.X, pady=(0, 8))
        self.speed_dir_label = ttk.Label(slider_fr, text="停止", font=("Consolas", 14, "bold"),
                                          foreground="#888")
        self.speed_dir_label.pack()
        self.speed_pct_label = ttk.Label(slider_fr, text="0% (0)", font=("Consolas", 11))
        self.speed_pct_label.pack()

        self.speed_var = tk.IntVar(value=0)
        speed_slider = ttk.Scale(slider_fr, from_=-1000, to=1000, orient=tk.HORIZONTAL,
                                  variable=self.speed_var, command=self._on_speed_slider_change)
        speed_slider.pack(fill=tk.X, pady=(5,0))

        val_fr = ttk.Frame(slider_fr); val_fr.pack(fill=tk.X, pady=(3,0))
        ttk.Label(val_fr, text="-1000").pack(side=tk.LEFT)
        ttk.Label(val_fr, text="0").pack(side=tk.LEFT, expand=True)
        ttk.Label(val_fr, text="+1000").pack(side=tk.RIGHT)

        speed_val_fr = ttk.Frame(slider_fr); speed_val_fr.pack(fill=tk.X, pady=(4,0))
        ttk.Label(speed_val_fr, text="速度值:").pack(side=tk.LEFT)
        self.speed_spin = ttk.Spinbox(speed_val_fr, from_=-1000, to=1000,
                                       textvariable=self.speed_var, width=8)
        self.speed_spin.pack(side=tk.LEFT, padx=5)
        ttk.Button(speed_val_fr, text="发送", command=self._set_speed).pack(side=tk.LEFT)

        ttk.Button(slider_fr, text="急停(设speed=0)", command=self._emergency_stop,
                   style="Accent.TButton").pack(fill=tk.X, pady=(6,0))

        b0 = ttk.Frame(ctrl); b0.grid(row=2, column=0, columnspan=2, pady=(8,0), sticky=tk.EW)
        ttk.Button(b0, text="扭矩ON",  command=self._torque_on).pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Button(b0, text="扭矩OFF", command=self._torque_off).pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        b1 = ttk.Frame(ctrl); b1.grid(row=3, column=0, columnspan=2, pady=(4,0), sticky=tk.EW)
        ttk.Button(b1, text="PING",     command=self._ping).pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Button(b1, text="读速度",   command=self._read_speed).pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Button(ctrl, text="★ 发送速度", command=self._set_speed,
                   style="Accent.TButton").grid(row=4, column=0, columnspan=2, pady=(8,0), sticky=tk.EW)

        # 轮模式初始化
        wheel = ttk.LabelFrame(left, text="轮模式初始化", padding=8); wheel.pack(fill=tk.X, pady=(0,8))
        w_top = ttk.Frame(wheel); w_top.pack(fill=tk.X)
        ttk.Label(w_top, text="舵机ID:").pack(side=tk.LEFT)
        self.wheel_id_var = tk.IntVar(value=1)
        ttk.Spinbox(w_top, from_=1, to=254, textvariable=self.wheel_id_var, width=8).pack(side=tk.LEFT, padx=5)
        ttk.Button(wheel, text="执行进入轮模式（6步）", command=self._enter_wheel_mode).pack(fill=tk.X)

        # 快捷命令
        quick = ttk.LabelFrame(left, text="快捷命令", padding=8); quick.pack(fill=tk.BOTH, expand=True)
        self.cmd_listbox = tk.Listbox(quick, font=("Consolas", 9), height=10)
        self.cmd_listbox.pack(fill=tk.BOTH, expand=True, side=tk.TOP)
        self.cmd_listbox.bind("<Double-Button-1>", lambda _: self._send_selected_command())
        ttk.Scrollbar(quick, command=self.cmd_listbox.yview).pack(side=tk.RIGHT, fill=tk.Y)
        self.cmd_listbox.config(yscrollcommand=lambda f, t: None)
        bf = ttk.Frame(quick); bf.pack(fill=tk.X, pady=(4,0))
        ttk.Button(bf, text="发送", command=self._send_selected_command).pack(side=tk.LEFT, padx=2)
        ttk.Button(bf, text="清空", command=self._clear_log).pack(side=tk.LEFT, padx=2)
        ttk.Button(bf, text="导出会话", command=self._export_log).pack(side=tk.RIGHT, padx=2)

        self._build_log_area(right, "speed")

    # ---- 位置Tab ----
    def _build_position_tab(self, parent):
        left = ttk.Frame(parent); left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        right = ttk.Frame(parent); right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        ctrl = ttk.LabelFrame(left, text="位置参数（关节模式）", padding=8); ctrl.pack(fill=tk.X, pady=(0,8))
        ttk.Label(ctrl, text="舵机ID:").grid(row=0, column=0, sticky=tk.W)
        self.pos_id_var = tk.IntVar(value=1)
        ttk.Spinbox(ctrl, from_=0, to=254, textvariable=self.pos_id_var, width=8).grid(row=0,column=1,padx=5)

        ttk.Label(ctrl, text="位置(0~4095):").grid(row=1, column=0, sticky=tk.W, pady=(5,0))
        self.pos_var = tk.IntVar(value=2048)
        ttk.Spinbox(ctrl, from_=0, to=4095, textvariable=self.pos_var, width=8).grid(row=1,column=1,padx=5,pady=(5,0))

        self.deg_label = ttk.Label(ctrl, text="0.00°", font=("Consolas", 11, "bold"))
        self.deg_label.grid(row=2, column=1, padx=5, pady=(2,0), sticky=tk.W)

        slider_fr = ttk.Frame(ctrl); slider_fr.grid(row=3, column=0, columnspan=2, pady=(8,0), sticky=tk.EW)
        ttk.Label(slider_fr, text="0").pack(side=tk.LEFT)
        self.pos_slider = ttk.Scale(slider_fr, from_=0, to=4095, orient=tk.HORIZONTAL,
                                     variable=self.pos_var, command=self._on_slider_change)
        self.pos_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Label(slider_fr, text="4095").pack(side=tk.RIGHT)

        ttk.Label(ctrl, text="运行速度:").grid(row=4, column=0, sticky=tk.W, pady=(5,0))
        self.pos_speed_var = tk.IntVar(value=300)
        ttk.Spinbox(ctrl, from_=0, to=1023, textvariable=self.pos_speed_var, width=8).grid(row=4,column=1,padx=5,pady=(5,0))

        pb = ttk.Frame(ctrl); pb.grid(row=5, column=0, columnspan=2, pady=(8,0), sticky=tk.EW)
        ttk.Button(pb, text="扭矩ON",  command=self._pos_torque_on).pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Button(pb, text="扭矩OFF", command=self._pos_torque_off).pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Button(pb, text="读位置",  command=self._read_pos).pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Button(ctrl, text="★ 发送目标位置", command=self._set_position,
                   style="Accent.TButton").grid(row=6, column=0, columnspan=2, pady=(8,0), sticky=tk.EW)

        # 角度预设
        preset = ttk.LabelFrame(left, text="角度预设", padding=8); preset.pack(fill=tk.X, pady=(0,8))
        pr = ttk.Frame(preset); pr.pack()
        for i,(label,val) in enumerate([("0°",2048),("+90°",3072),("-90°",1024),("+180°",4095),("-180°",0)]):
            r,c = divmod(i,3)
            ttk.Button(pr, text=label, width=6,
                       command=lambda v=val:(self.pos_var.set(v), self._on_slider_change(None))).grid(row=r,column=c,padx=2,pady=2)

        # 关节模式初始化
        joint = ttk.LabelFrame(left, text="关节模式初始化", padding=8); joint.pack(fill=tk.X, pady=(0,8))
        jt = ttk.Frame(joint); jt.pack(fill=tk.X)
        ttk.Label(jt, text="舵机ID:").pack(side=tk.LEFT)
        self.joint_id_var = tk.IntVar(value=1)
        ttk.Spinbox(jt, from_=1, to=254, textvariable=self.joint_id_var, width=8).pack(side=tk.LEFT, padx=5)
        ttk.Button(joint, text="执行进入关节模式", command=self._enter_joint_mode).pack(fill=tk.X)

        self._build_log_area(right, "pos")

    # ---- 寄存器Tab ----
    def _build_register_tab(self, parent):
        fr = ttk.LabelFrame(parent, text="寄存器读写", padding=10); fr.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        ttk.Label(fr, text="舵机ID:").grid(row=0, column=0, sticky=tk.W)
        self.reg_id_var = tk.IntVar(value=1)
        ttk.Spinbox(fr, from_=0, to=254, textvariable=self.reg_id_var, width=8).grid(row=0,column=1,padx=5,sticky=tk.W)

        cols = ("地址","名称","值(Hex)","值(Dec)")
        tree = ttk.Treeview(fr, columns=cols, show="headings", height=9)
        for c in cols:
            tree.heading(c, text=c)
            tree.column(c, width=120)
        tree.grid(row=1, column=0, columnspan=5, pady=(8,0), sticky=tk.EW)
        self.reg_tree = tree
        self._populate_reg_table()
        ttk.Scrollbar(fr, orient=tk.VERTICAL, command=tree.yview).grid(row=1,column=5,pady=(8,0),sticky=tk.NS)

        br = ttk.Frame(fr); br.grid(row=2, column=0, columnspan=5, pady=(8,0), sticky=tk.EW)
        ttk.Button(br, text="读所有", command=self._read_all_regs).pack(side=tk.LEFT, padx=2)
        ttk.Button(br, text="扫描", command=self._scan_all).pack(side=tk.LEFT, padx=2)
        ttk.Button(br, text="停止扫描", command=self._scan_stop).pack(side=tk.LEFT, padx=2)
        ttk.Button(br, text="诊断", command=self._run_diagnostic).pack(side=tk.LEFT, padx=2)

        cust = ttk.LabelFrame(fr, text="自定义读/写", padding=8); cust.grid(row=3, column=0, columnspan=5, pady=(10,0), sticky=tk.EW)
        ttk.Label(cust, text="地址(hex):").grid(row=0, column=0, sticky=tk.W)
        self.reg_addr_var = tk.StringVar(value="2E")
        ttk.Entry(cust, textvariable=self.reg_addr_var, width=8).grid(row=0,column=1,padx=5)
        ttk.Label(cust, text="长度/值(hex):").grid(row=0, column=2, sticky=tk.W, padx=(10,0))
        self.reg_val_var = tk.StringVar(value="")
        ttk.Entry(cust, textvariable=self.reg_val_var, width=15).grid(row=0,column=3,padx=5)
        rw = ttk.Frame(cust); rw.grid(row=1, column=0, columnspan=4, pady=(6,0), sticky=tk.EW)
        ttk.Button(rw, text="读取", command=self._reg_read).pack(side=tk.LEFT, padx=2)
        ttk.Button(rw, text="写入", command=self._reg_write).pack(side=tk.LEFT, padx=2)

        self.reg_log = scrolledtext.ScrolledText(fr, font=("Consolas", 8), height=10)
        self.reg_log.grid(row=4, column=0, columnspan=5, pady=(8,0), sticky=tk.EW)

    # ---- 日志Tab ----
    def _build_log_tab(self, parent):
        self._build_log_area(parent, "log_tab")

    def _build_log_area(self, parent, key: str):
        fr = ttk.LabelFrame(parent, text="通信日志", padding=5); fr.pack(fill=tk.BOTH, expand=True)
        tb = ttk.Frame(fr); tb.pack(fill=tk.X, pady=(0,3))
        self.hex_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(tb, text="Hex", variable=self.hex_var).pack(side=tk.LEFT)
        self.desc_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(tb, text="解析", variable=self.desc_var).pack(side=tk.LEFT, padx=5)
        self.ts_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(tb, text="时间戳", variable=self.ts_var).pack(side=tk.LEFT, padx=5)
        ttk.Label(tb, text="  TX:0  RX:0", foreground="#888").pack(side=tk.LEFT)
        self.count_labels.append(tb.winfo_children()[-1])
        ttk.Button(tb, text="清空", command=self._clear_log).pack(side=tk.RIGHT, padx=2)
        ttk.Button(tb, text="导出CSV", command=self._export_log).pack(side=tk.RIGHT, padx=2)

        txt = scrolledtext.ScrolledText(fr, font=("Consolas", 9), wrap=tk.WORD)
        txt.pack(fill=tk.BOTH, expand=True)
        txt.tag_config("TX",    foreground="#0055CC", font=("Consolas", 9, "bold"))
        txt.tag_config("RX",    foreground="#006600")
        txt.tag_config("ERROR", foreground="#CC0000")
        txt.tag_config("INFO",  foreground="#666666")

        if key == "speed":
            self.output_text = txt
        elif key == "pos":
            self.pos_output = txt
        elif key == "log_tab":
            self.log_tab_text = txt

    def _build_statusbar(self):
        sb = ttk.Frame(self.root, relief=tk.SUNKEN); sb.pack(fill=tk.X, side=tk.BOTTOM)
        self.status_var = tk.StringVar(value="就绪")
        ttk.Label(sb, textvariable=self.status_var).pack(side=tk.LEFT, padx=5)
        self.log_count_var = tk.StringVar(value="TX:0 | RX:0")
        ttk.Label(sb, textvariable=self.log_count_var).pack(side=tk.RIGHT, padx=5)

    def _populate_reg_table(self):
        self.reg_tree.delete(*self.reg_tree.get_children())
        regs = [
            (0x28,"扭矩开关"), (0x29,"ACC+速度(3B)"), (0x2A,"目标位置L"),
            (0x2C,"目标位置H"), (0x2E,"目标速度L"), (0x30,"目标速度H"),
            (0x33,"运行模式"), (0x38,"当前位置L"), (0x3B,"当前位置H"),
            (0x55,"EPROM锁定"),
        ]
        for addr, name in regs:
            self.reg_tree.insert("", tk.END, values=(f"0x{addr:02X}", name, "-", "-"))

    def _populate_commands(self):
        cmds = [
            ("PING",              self._ping),
            ("扭矩ON",            self._torque_on),
            ("扭矩OFF",           self._torque_off),
            ("读扭矩(0x28)",      self._read_torque),
            ("读速度(0x2E)",      self._read_speed),
            ("读ACC(0x29)",      self._read_acc),
            ("--- 轮模式步骤 ---", None),
            ("[1]关扭矩",         lambda: self._write_reg(0x28, "00")),
            ("[2]解锁(0x55=0)",   lambda: self._write_reg(0x55, "00")),
            ("[3]角限位清零",     lambda: self._write_reg(0x09, "00000000")),
            ("[4]轮模式(0x33=1)", lambda: self._write_reg(0x33, "01")),
            ("[5]锁定(0x55=1)",   lambda: self._write_reg(0x55, "01")),
            ("[6]开扭矩",         self._torque_on),
            ("--- 测试 ---",      None),
            ("诊 断（自动）",     self._run_diagnostic),
        ]
        self.commands = cmds
        for name, _ in cmds:
            self.cmd_listbox.insert(tk.END, name)

    # ------------------------------------------------------------------------
    # 串口
    # ------------------------------------------------------------------------

    def _refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        plist = [f"{p.device} - {p.description}" for p in ports]
        self.port_combo["values"] = plist if plist else ["无可用串口"]
        if plist:
            self.port_combo.current(0)

    def _toggle_connection(self):
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        try:
            port = self.port_var.get().split(" - ")[0]
            baud = int(self.baud_var.get())
            self.ser = serial.Serial(port, baud, timeout=0.5)
            self.running = True
            self.rx_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.rx_thread.start()
            self.connect_btn.config(text="断开")
            self.status_label.config(text=f"已连接 {port}@{baud}", foreground="green")
            self._log_all("INFO", f"串口打开: {port} @ {baud}")
            self.logger.log_info(f"串口打开: {port} @ {baud}")
        except Exception as e:
            messagebox.showerror("连接失败", str(e))

    def _disconnect(self):
        self.running = False
        if self.ser:
            self.ser.close()
        self.connect_btn.config(text="连接")
        self.status_label.config(text="未连接", foreground="red")
        self._log_all("INFO", "串口关闭")

    def _read_loop(self):
        buf = bytearray()
        while self.running and self.ser:
            try:
                d = self.ser.read(256)
                if d:
                    hex_raw = d.hex().upper()
                    buf.extend(d)
                    self._log_all("INFO", f"    [RAW RX {len(d)}B] {hex_raw}")
                    self._process_buf(buf)
            except Exception:
                break

    def _process_buf(self, buf: bytearray):
        while len(buf) >= 6:
            if buf[0] != 0xFF: buf.pop(0); continue
            if buf[1] != 0xFF: buf.pop(1); continue
            length = buf[3]
            pkg_len = 4 + length + 1
            if len(buf) < pkg_len:
                break
            pkg = bytes(buf[:pkg_len])
            del buf[:pkg_len]
            parsed = parse_response(pkg)
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3] if self.ts_var.get() else ""
            self.root.after(0, self._on_rx, pkg, parsed, ts)

    def _send(self, data: bytes) -> bool:
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("未连接", "请先连接串口")
            return False
        try:
            self.ser.write(data)
            self.tx_count += 1
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3] if self.ts_var.get() else ""
            hex_str = data.hex().upper()
            self.logger.log_tx(hex_str)
            self.root.after(0, self._display_tx, data, ts)
            return True
        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("发送失败", str(e)))
            return False

    def _on_rx(self, raw: bytes, parsed: dict, ts: str):
        self.rx_count += 1
        hex_str = raw.hex().upper()
        self.logger.log_rx(hex_str, parsed)
        self._update_counts()

        tag = "RX"
        if parsed.get("error"):
            tag = "ERROR"
        elif not parsed.get("valid"):
            tag = "ERROR"

        prefix = f"[{ts}] " if ts else ""
        if self.desc_var.get():
            line = f"{prefix}RX: {describe_hex(raw)}"
        else:
            line = f"{prefix}RX: {hex_str}"

        self._log_all(tag, line)
        if parsed.get("error"):
            self._log_all("ERROR", f"    错误: {parsed['error']}")
        if parsed.get("valid") and parsed.get("params"):
            self._log_all("RX", f"    数据: {parsed['params'].hex().upper()}")

    def _display_tx(self, data: bytes, ts: str):
        prefix = f"[{ts}] " if ts else ""
        line = f"{prefix}TX: {data.hex().upper()}"
        self._log_all("TX", line)
        self._update_counts()

    def _update_counts(self):
        self.log_count_var.set(f"TX:{self.tx_count} | RX:{self.rx_count}")

    def _log_all(self, tag: str, line: str):
        for txt in [self.output_text, getattr(self, "pos_output", None),
                    getattr(self, "log_tab_text", None)]:
            if txt:
                txt.insert(tk.END, line + "\n", tag)
                txt.see(tk.END)

    def _clear_log(self):
        self._log_all("INFO", "=== 日志已清空 ===")

    def _status_poll(self):
        self.status_var.set(f"TX={self.tx_count} RX={self.rx_count} 记录={len(self.logger.entries)}")
        self.root.after(300, self._status_poll)

    # ------------------------------------------------------------------------
    # 速度控制
    # ------------------------------------------------------------------------

    def _torque_on(self):
        sid = self.servo_id_var.get()
        self._send(build_packet(sid, SCS_INST_WRITE, bytes([ADDR_TORQUE, 1])))

    def _torque_off(self):
        sid = self.servo_id_var.get()
        self._send(build_packet(sid, SCS_INST_WRITE, bytes([ADDR_TORQUE, 0])))

    def _ping(self):
        self._send(build_packet(self.servo_id_var.get(), SCS_INST_PING))

    def _read_torque(self):
        self._send(build_packet(self.servo_id_var.get(), SCS_INST_READ, bytes([ADDR_TORQUE, 1])))

    def _read_speed(self):
        self._send(build_packet(self.servo_id_var.get(), SCS_INST_READ, bytes([ADDR_ACC_SPEED, 3])))

    def _read_acc(self):
        self._send(build_packet(self.servo_id_var.get(), SCS_INST_READ, bytes([ADDR_ACC_SPEED, 3])))

    def _on_speed_slider_change(self, val_str: str):
        speed = int(float(val_str))
        pct = round(abs(speed) / 1000 * 100)
        if speed == 0:
            self.speed_dir_label.config(text="停止", foreground="#888")
            self.speed_pct_label.config(text="0% (0)")
        elif speed > 0:
            self.speed_dir_label.config(text="正转 →", foreground="#006600")
            self.speed_pct_label.config(text=f"{pct}% ({speed})")
        else:
            self.speed_dir_label.config(text="← 反转", foreground="#CC0000")
            self.speed_pct_label.config(text=f"{pct}% ({speed})")

    def _speed_to_reg(self, speed: int) -> bytes:
        """速度编码：正数=原值，负数=1024+|v|（STS无符号格式，BigEndian）"""
        if speed >= 0:
            v = min(speed, 1000)
        else:
            v = 1024 + min(abs(speed), 1000)
        return struct.pack(">H", v & 0x3FF)

    def _emergency_stop(self):
        """急停：关扭矩 + 速度归零，写0x2E（BigEndian）"""
        sid = self.servo_id_var.get()
        self.speed_var.set(0)
        self.speed_dir_label.config(text="急停", foreground="#FF0000")
        self.speed_pct_label.config(text="0 (已发送)")
        spd_b = self._speed_to_reg(0)
        self.logger.log_servo_speed(sid, 0, spd_b.hex().upper())
        self._log_all("INFO", f"急停 ID{sid} speed=0 -> reg={spd_b.hex().upper()}")
        self._send(build_packet(sid, SCS_INST_WRITE, bytes([ADDR_TORQUE, 0])))
        self._send(build_packet(sid, SCS_INST_WRITE, bytes([ADDR_GOAL_SPEED_L]) + spd_b))

    def _set_speed(self):
        sid = self.servo_id_var.get()
        speed = self.speed_var.get()
        acc   = self.acc_var.get()
        spd_b = self._speed_to_reg(speed)
        self.logger.log_servo_speed(sid, speed, spd_b.hex().upper())
        self._log_all("INFO", f"设速度 ID{sid}: raw={speed} -> reg={spd_b.hex().upper()} acc={acc}")
        self._send(build_packet(sid, SCS_INST_WRITE, bytes([ADDR_GOAL_SPEED_L]) + spd_b))
        time.sleep(0.02)
        data_29 = bytes([acc & 0xFF]) + spd_b
        self._send(build_packet(sid, SCS_INST_WRITE, bytes([ADDR_ACC_SPEED]) + data_29))

    def _enter_wheel_mode(self):
        sid = self.wheel_id_var.get()
        self._log_all("INFO", f"=== 进入轮模式 ID{sid} ===")
        self.logger.log_info(f"轮模式初始化 ID={sid}")
        steps = [
            ("[1]关扭矩",    build_packet(sid, SCS_INST_WRITE, bytes([ADDR_TORQUE, 0]))),
            ("[2]解锁",      build_packet(sid, SCS_INST_WRITE, bytes([ADDR_LOCK, 0]))),
            ("[3]角限位0",   build_packet(sid, SCS_INST_WRITE, bytes([0x09, 0, 0, 0, 0]))),
            ("[4]轮模式",    build_packet(sid, SCS_INST_WRITE, bytes([ADDR_MODE, MODE_WHEEL]))),
            ("[5]锁定",      build_packet(sid, SCS_INST_WRITE, bytes([ADDR_LOCK, 1]))),
            ("[6]开扭矩",    build_packet(sid, SCS_INST_WRITE, bytes([ADDR_TORQUE, 1]))),
        ]
        for name, pkt in steps:
            self._log_all("INFO", f"  {name}...")
            self._send(pkt)
            time.sleep(0.05)
        time.sleep(0.1)
        speed = self.speed_var.get()
        spd_b = self._speed_to_reg(speed)
        self.logger.log_servo_speed(sid, speed, spd_b.hex().upper())
        self._log_all("INFO", f"设速度={speed} -> reg={spd_b.hex().upper()}")
        self._send(build_packet(sid, SCS_INST_WRITE, bytes([ADDR_GOAL_SPEED_L]) + spd_b))
        time.sleep(0.02)
        data_29 = bytes([self.acc_var.get() & 0xFF]) + spd_b
        self._send(build_packet(sid, SCS_INST_WRITE, bytes([ADDR_ACC_SPEED]) + data_29))
        self._log_all("INFO", f"=== 轮模式完成 ===")

    def _send_selected_command(self):
        idx = self.cmd_listbox.curselection()
        if not idx:
            return
        name, cmd = self.commands[idx[0]]
        if cmd:
            cmd()

    # ------------------------------------------------------------------------
    # 位置控制
    # ------------------------------------------------------------------------

    def _pos_torque_on(self):
        self._send(build_packet(self.pos_id_var.get(), SCS_INST_WRITE, bytes([ADDR_TORQUE, 1])))

    def _pos_torque_off(self):
        self._send(build_packet(self.pos_id_var.get(), SCS_INST_WRITE, bytes([ADDR_TORQUE, 0])))

    def _read_pos(self):
        self._send(build_packet(self.pos_id_var.get(), SCS_INST_READ, bytes([ADDR_CUR_POS_L, 2])))

    def _set_position(self):
        sid = self.pos_id_var.get()
        pos = self.pos_var.get()
        spd = self.pos_speed_var.get()
        self._log_pos(f"设位置 ID{sid}: pos={pos}, speed={spd}")
        data = struct.pack(">HH", pos, spd)
        self._send(build_packet(sid, SCS_INST_WRITE, bytes([ADDR_GOAL_POS_L]) + data))

    def _enter_joint_mode(self):
        sid = self.joint_id_var.get()
        self._log_pos(f"=== 关节模式 ID{sid} ===")
        steps = [
            ("关扭矩",  build_packet(sid, SCS_INST_WRITE, bytes([ADDR_TORQUE, 0]))),
            ("解锁",    build_packet(sid, SCS_INST_WRITE, bytes([ADDR_LOCK, 0]))),
            ("关节模式",build_packet(sid, SCS_INST_WRITE, bytes([ADDR_MODE, MODE_JOINT]))),
            ("锁定",    build_packet(sid, SCS_INST_WRITE, bytes([ADDR_LOCK, 1]))),
            ("开扭矩",  build_packet(sid, SCS_INST_WRITE, bytes([ADDR_TORQUE, 1]))),
        ]
        for name, pkt in steps:
            self._log_pos(f"  {name}...")
            self._send(pkt)
            time.sleep(0.05)
        self._log_pos("=== 完成 ===")

    def _on_slider_change(self, _):
        pos = self.pos_var.get()
        deg = round((pos - 2048) / 4095 * 360, 2)
        self.deg_label.config(text=f"{deg}°")

    def _log_pos(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3] if self.ts_var.get() else ""
        prefix = f"[{ts}] " if ts else ""
        if hasattr(self, "pos_output"):
            self.pos_output.insert(tk.END, prefix + msg + "\n", "INFO")
            self.pos_output.see(tk.END)

    # ------------------------------------------------------------------------
    # 寄存器读写
    # ------------------------------------------------------------------------

    def _read_all_regs(self):
        sid = self.reg_id_var.get()
        regs = [
            (0x28,"扭矩",1), (0x29,"ACC+速度",3), (0x2A,"目标位置",2),
            (0x2E,"目标速度",2), (0x33,"模式",1), (0x38,"当前位置",2),
            (0x55,"锁定",1),
        ]
        self.reg_log.insert(tk.END, f"\n=== 读寄存器 ID{sid} ===\n")
        for addr, name, length in regs:
            self._send(build_packet(sid, SCS_INST_READ, bytes([addr, length])))
            time.sleep(0.03)
            self.reg_log.insert(tk.END, f"  读 0x{addr:02X} ({name})\n")
        self.reg_log.see(tk.END)

    def _reg_read(self):
        sid = self.reg_id_var.get()
        try:
            addr = int(self.reg_addr_var.get(), 16)
            self._send(build_packet(sid, SCS_INST_READ, bytes([addr, 1])))
            self.reg_log.insert(tk.END, f"读 0x{addr:02X}\n")
            self.reg_log.see(tk.END)
        except ValueError:
            messagebox.showerror("错误", "地址格式无效")

    def _reg_write(self):
        sid = self.reg_id_var.get()
        try:
            addr = int(self.reg_addr_var.get(), 16)
            hex_val = self.reg_val_var.get().strip()
            if not hex_val:
                messagebox.showwarning("空值", "请输入要写入的值")
                return
            data = bytes.fromhex(hex_val)
            self._send(build_packet(sid, SCS_INST_WRITE, bytes([addr]) + data))
            self.reg_log.insert(tk.END, f"写 0x{addr:02X} = {hex_val.upper()}\n")
            self.reg_log.see(tk.END)
        except ValueError:
            messagebox.showerror("错误", "值格式无效")

    def _write_reg(self, addr: int, hex_val: str):
        sid = self.reg_id_var.get()
        data = bytes.fromhex(hex_val)
        self._send(build_packet(sid, SCS_INST_WRITE, bytes([addr]) + data))

    # ------------------------------------------------------------------------
    # 诊断功能
    # ------------------------------------------------------------------------

    def _run_diagnostic(self):
        sid = self.reg_id_var.get()
        self.reg_log.insert(tk.END, f"\n{'='*50}\n")
        self.reg_log.insert(tk.END, f"=== 诊断 ID={sid} ===\n")
        self.reg_log.see(tk.END)

        # 1. PING
        self.reg_log.insert(tk.END, f"\n[1] PING ID={sid}\n")
        self._send(build_packet(sid, SCS_INST_PING))
        time.sleep(0.1)
        self.reg_log.insert(tk.END, f"    等待响应...\n"); self.reg_log.see(tk.END)

        # 2. 读锁定状态
        self.reg_log.insert(tk.END, f"\n[2] 读锁定状态(0x55)\n")
        self._send(build_packet(sid, SCS_INST_READ, bytes([ADDR_LOCK, 1])))
        time.sleep(0.1)

        # 3. 读扭矩
        self.reg_log.insert(tk.END, f"\n[3] 读扭矩(0x28)\n")
        self._send(build_packet(sid, SCS_INST_READ, bytes([ADDR_TORQUE, 1])))
        time.sleep(0.1)

        # 4. 读速度
        self.reg_log.insert(tk.END, f"\n[4] 读速度(0x29)\n")
        self._send(build_packet(sid, SCS_INST_READ, bytes([ADDR_ACC_SPEED, 3])))
        time.sleep(0.1)

        # 5. 读位置
        self.reg_log.insert(tk.END, f"\n[5] 读位置(0x38)\n")
        self._send(build_packet(sid, SCS_INST_READ, bytes([ADDR_CUR_POS_L, 2])))
        time.sleep(0.1)

        # 6. 读模式
        self.reg_log.insert(tk.END, f"\n[6] 读模式(0x33)\n")
        self._send(build_packet(sid, SCS_INST_READ, bytes([ADDR_MODE, 1])))
        time.sleep(0.1)

        # 7. 广播PING
        self.reg_log.insert(tk.END, f"\n[7] 广播PING(ID=254)\n")
        self._send(build_packet(254, SCS_INST_PING))
        time.sleep(0.3)

        # 8. 尝试开扭矩
        self.reg_log.insert(tk.END, f"\n[8] 开扭矩(0x28=1)\n")
        self._send(build_packet(sid, SCS_INST_WRITE, bytes([ADDR_TORQUE, 1])))
        time.sleep(0.1)

        # 9. 设速度（先写0x2E，再写0x29）
        spd = self.speed_var.get()
        spd_b = self._speed_to_reg(spd)
        self.reg_log.insert(tk.END, f"\n[9] 设速度={spd} -> reg={spd_b.hex().upper()}\n")
        self._send(build_packet(sid, SCS_INST_WRITE, bytes([ADDR_GOAL_SPEED_L]) + spd_b))
        time.sleep(0.02)
        data_29 = bytes([self.acc_var.get() & 0xFF]) + spd_b
        self._send(build_packet(sid, SCS_INST_WRITE, bytes([ADDR_ACC_SPEED]) + data_29))
        time.sleep(0.1)

        self.reg_log.insert(tk.END, f"\n=== 诊断完成，请观察上方RX响应 ===\n", "INFO")
        self.reg_log.see(tk.END)

    # ------------------------------------------------------------------------
    # 多波特率多ID扫描（后台线程，不会卡死UI）
    # ------------------------------------------------------------------------

    def _scan_all(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("未连接", "请先连接串口")
            return

        if getattr(self, "_scanning", False):
            messagebox.showwarning("正在扫描", "扫描进行中，请稍候")
            return

        self._scanning = True
        self._scan_cancel = threading.Event()
        self._scan_found = []
        self._scan_total = 0
        self._scan_cur_baud = 0
        self._scan_cur_id = 0

        self.reg_log.insert(tk.END, "\n" + "=" * 50 + "\n")
        self.reg_log.insert(tk.END, "=== 轮询ID扫描（后台运行）===\n")
        self.reg_log.insert(tk.END, f"端口: {self.ser.port}\n")
        self.reg_log.insert(tk.END, "波特率: [115200, 57600, 38400, 19200, 9600, 500000]\n")
        self.reg_log.insert(tk.END, "ID范围: 0 ~ 254（共255个）\n")
        self.reg_log.insert(tk.END, f"点击【停止扫描】可取消\n")
        self.reg_log.see(tk.END)

        self._scan_thread = threading.Thread(target=self._scan_worker, daemon=True)
        self._scan_thread.start()

        self._scan_status_label = ttk.Label(self.reg_log.master, text="  [扫描中...]  ", foreground="orange")
        self._scan_status_label.pack(side=tk.RIGHT, padx=5)

    def _scan_worker(self):
        bauds = [115200, 57600, 38400, 19200, 9600, 500000]
        ids = list(range(0, 255))
        total_pings = len(bauds) * len(ids)
        self._scan_total = total_pings

        old_baud = self.ser.baudrate
        old_timeout = self.ser.timeout
        self.ser.timeout = 0.05
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        try:
            for baud in bauds:
                if self._scan_cancel.is_set():
                    break
                try:
                    self.ser.baudrate = baud
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                except Exception:
                    continue

                self._scan_cur_baud = baud
                self.root.after(0, lambda b=baud: self._scan_append(f"\n[波特率 {b}]"))

                for sid in ids:
                    if self._scan_cancel.is_set():
                        break

                    self._scan_cur_id = sid
                    rx_before = self.rx_count
                    pkt = build_packet(sid, SCS_INST_PING)

                    try:
                        self.ser.write(pkt)
                        self.tx_count += 1
                        hex_str = pkt.hex().upper()
                        self.logger.log_scan_ping(baud, sid, hex_str)
                    except Exception:
                        pass

                    self.root.after(0, lambda s=sid, b=baud: self._scan_append(f"  ID={s:3d} PING -> "))

                    for _ in range(4):
                        if self._scan_cancel.is_set():
                            break
                        time.sleep(0.05)
                        if self.rx_count > rx_before:
                            self._scan_found.append((baud, sid))
                            self.logger.log_scan_found(baud, sid)
                            self.root.after(0, lambda s=sid, b=baud: self._scan_append(f"** 响应! ** (ID={s} @ {b})\n", tag="INFO"))
                            break
                    else:
                        self.root.after(0, lambda s=sid: self._scan_append("无响应\n"))

        finally:
            self.ser.baudrate = old_baud
            self.ser.timeout = old_timeout
            self._scanning = False
            self.root.after(0, self._scan_done, old_baud)

    def _scan_append(self, text: str, tag: str = ""):
        self.reg_log.insert(tk.END, text + ("\n" if not text.endswith("\n") else ""))
        if tag:
            self.reg_log.tag_add(tag, "end-2l", "end")
        self.reg_log.see(tk.END)

    def _scan_done(self, old_baud: int):
        if hasattr(self, "_scan_status_label") and self._scan_status_label.winfo_exists():
            self._scan_status_label.destroy()

        found = self._scan_found
        self.reg_log.insert(tk.END, f"\n{'=' * 50}\n")
        if found:
            self.reg_log.insert(tk.END, f"=== 找到 {len(found)} 个舵机 ===\n", "INFO")
            for b, i in found:
                self.reg_log.insert(tk.END, f"  ★ 波特率={b}, ID={i}\n", "INFO")
            self.logger.log_scan_summary(found)
        else:
            self.reg_log.insert(tk.END, "=== 未找到任何舵机 ===\n", "ERROR")
        self.reg_log.see(tk.END)

        msg = f"扫描了 {self._scan_total} 次PING\n找到 {len(found)} 个舵机"
        if found:
            msg += f"\n\n推荐设置:\n波特率={found[0][0]}\n舵机ID={found[0][1]}"
        messagebox.showinfo("扫描完成", msg)

    def _scan_stop(self):
        if getattr(self, "_scanning", False):
            self._scan_cancel.set()
            self.reg_log.insert(tk.END, "\n[扫描已停止]\n", "ERROR")
            self.reg_log.see(tk.END)


    # ------------------------------------------------------------------------
    # 导出
    # ------------------------------------------------------------------------

    def _export_log(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV文件", "*.csv"), ("文本文件", "*.txt"), ("所有文件", "*.*")]
        )
        if not path:
            return
        try:
            self.logger.export(path)
            messagebox.showinfo("导出成功", f"已保存:\n{path}")
            self._log_all("INFO", f"导出成功: {path}")
        except Exception as e:
            messagebox.showerror("导出失败", str(e))

    def _on_close(self):
        if self.logger.entries:
            ans = messagebox.askyesnocancel("关闭", f"共{len(self.logger.entries)}条记录，导出日志?\n是=CSV 否=TXT 取消=不导出")
            if ans:
                self._export_log()
            elif ans is False:
                path = filedialog.asksaveasfilename(
                    defaultextension=".txt",
                    filetypes=[("文本文件", "*.txt"), ("所有文件", "*.*")]
                )
                if path:
                    self.logger.export(path)
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ServoDebugTool(root)
    root.mainloop()
