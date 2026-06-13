#!/usr/bin/env python3
"""
测试脚本：验证ID=8速度=-134、ID=3速度=6的协议编码
手动执行以下步骤后导出会话日志到 tools/11.csv
"""
import struct, csv, os, serial, time, threading
from datetime import datetime

# ---- 协议常量 ----
SCS_FRAME_HDR   = 0xFFFF
SCS_INST_PING   = 0x01
SCS_INST_READ   = 0x02
SCS_INST_WRITE  = 0x03
SCS_INST_REG_WRITE = 0x04
SCS_INST_ACTION = 0x05
SCS_INST_SYNC_WRITE = 0x83

ADDR_TORQUE       = 0x28
ADDR_GOAL_SPEED_L = 0x2E
ADDR_ACC_SPEED    = 0x29
ADDR_MODE         = 0x33
ADDR_LOCK         = 0x37
MODE_WHEEL        = 1

# ---- 速度编码（BigEndian，无符号） ----
def speed_to_reg(speed: int) -> bytes:
    """正数=原值，负数=1024+|v|"""
    if speed >= 0:
        v = min(speed, 1000)
    else:
        v = 1024 + min(abs(speed), 1000)
    return struct.pack(">H", v & 0x3FF)

# ---- 数据包构建 ----
def build_packet(servo_id: int, instruction: int, params: bytes = b"") -> bytes:
    data = bytes([servo_id]) + bytes([instruction]) + params
    crc = sum(data) & 0xFF
    return bytes([0xFF, 0xFF]) + data + bytes([crc])

def build_hex(pkt: bytes) -> str:
    return "".join(f"{b:02X}" for b in pkt)

def calc_crc(data: bytes) -> int:
    return sum(data) & 0xFF

# ---- CSV记录 ----
CSV_PATH = os.path.join(os.path.dirname(__file__), "tools", "11.csv")

def log_tx(ts: str, pkt: bytes, note: str = ""):
    row = {
        "ts": ts, "dir": "TX", "hex": build_hex(pkt),
        "id": str(pkt[2] if len(pkt) > 2 else "-"),
        "error": "", "params": "", "valid": "", "note": note
    }
    write_csv(row)

def log_rx(ts: str, raw: str, note: str = ""):
    row = {"ts": ts, "dir": "RX", "hex": raw,
           "id": "-", "error": "", "params": "", "valid": "", "note": note}
    write_csv(row)

def write_csv(row: dict):
    exists = os.path.exists(CSV_PATH)
    with open(CSV_PATH, "a", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=["ts","dir","hex","id","error","params","valid","note"])
        if not exists:
            w.writeheader()
        w.writerow(row)

def ts_now():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

# ---- 轮模式初始化步骤 ----
def enter_wheel_mode(ser, sid: int):
    steps = [
        ("关扭矩",    build_packet(sid, SCS_INST_WRITE, bytes([ADDR_TORQUE, 0]))),
        ("解锁",      build_packet(sid, SCS_INST_WRITE, bytes([ADDR_LOCK, 0]))),
        ("角限位0",   build_packet(sid, SCS_INST_WRITE, bytes([0x09, 0, 0, 0, 0]))),
        ("轮模式",    build_packet(sid, SCS_INST_WRITE, bytes([ADDR_MODE, MODE_WHEEL]))),
        ("锁定",      build_packet(sid, SCS_INST_WRITE, bytes([ADDR_LOCK, 1]))),
        ("开扭矩",    build_packet(sid, SCS_INST_WRITE, bytes([ADDR_TORQUE, 1]))),
    ]
    for name, pkt in steps:
        log_tx(ts_now(), pkt, f"轮模式:{name} ID={sid}")
        ser.write(pkt)
        time.sleep(0.06)
    time.sleep(0.1)

def set_speed(ser, sid: int, speed: int):
    spd_b = speed_to_reg(speed)
    reg_val = struct.unpack(">H", spd_b)[0]
    print(f"  ID={sid} speed={speed} -> reg={spd_b.hex().upper()} (dec={reg_val})")
    log_tx(ts_now(), b"", f"速度 raw={speed} reg={spd_b.hex().upper()}")

    # 写0x2E
    pkt_2e = build_packet(sid, SCS_INST_WRITE, bytes([ADDR_GOAL_SPEED_L]) + spd_b)
    log_tx(ts_now(), pkt_2e, f"写速度到0x2E ID={sid}")
    ser.write(pkt_2e)
    time.sleep(0.03)

    # 写0x29 ACC+速度
    data_29 = bytes([0]) + spd_b
    pkt_29 = build_packet(sid, SCS_INST_WRITE, bytes([ADDR_ACC_SPEED]) + data_29)
    log_tx(ts_now(), pkt_29, f"写速度到0x29 ID={sid}")
    ser.write(pkt_29)

def main():
    # 速度参数
    TEST_ID8_SPEED = -134
    TEST_ID3_SPEED = 6

    print("=== 速度寄存器编码验证 ===")
    print(f"ID=8  speed={TEST_ID8_SPEED} -> reg={speed_to_reg(TEST_ID8_SPEED).hex().upper()}")
    print(f"ID=3  speed={TEST_ID3_SPEED} -> reg={speed_to_reg(TEST_ID3_SPEED).hex().upper()}")
    print()

    # 手动确认：请先在GUI中连接串口，扫描找到舵机ID=3和ID=8
    # 然后运行本脚本前，确保GUI已连接
    port = input("请输入串口名 (如 COM3): ").strip()
    if not port:
        print("未输入串口，生成协议预览")
        print()
        print("--- ID=8 轮模式初始化 ---")
        enter_wheel_mode(None, 8)
        print()
        print(f"--- ID=8 设置速度={TEST_ID8_SPEED} ---")
        spd8 = speed_to_reg(TEST_ID8_SPEED)
        reg8 = struct.unpack(">H", spd8)[0]
        print(f"  寄存器值: 0x{spd8.hex().upper()} = {reg8} (1024+|speed| = {1024+abs(TEST_ID8_SPEED)})")
        return

    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        print(f"打开串口失败: {e}")
        return

    print(f"串口已打开: {port}")
    log_rx(ts_now(), "", "=== 开始测试 ===")

    print("=== ID=8 进入轮模式 ===")
    enter_wheel_mode(ser, 8)
    time.sleep(0.1)

    print(f"=== ID=8 设置速度={TEST_ID8_SPEED} ===")
    set_speed(ser, 8, TEST_ID8_SPEED)
    time.sleep(0.1)

    print("=== ID=3 进入轮模式 ===")
    enter_wheel_mode(ser, 3)
    time.sleep(0.1)

    print(f"=== ID=3 设置速度={TEST_ID3_SPEED} ===")
    set_speed(ser, 3, TEST_ID3_SPEED)

    log_rx(ts_now(), "", "=== 测试完成 ===")
    print()
    print(f"协议记录已写入: {CSV_PATH}")
    ser.close()

if __name__ == "__main__":
    main()
