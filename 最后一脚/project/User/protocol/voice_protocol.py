#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
大葱切割-清洗-打包一体机 上位机通信测试脚本
适用于语音模块 / PC调试 / Jetson
============================================================
用法:
  python voice_protocol.py                      # 交互模式
  python voice_protocol.py --send DCM1_START    # 单条命令模式
  python voice_protocol.py --auto                # 自动演示模式

帧格式: [AA] [命令] [长度] [数据...] [校验]
校验:  命令 ^ 长度 ^ data[0] ^ ... ^ data[N-1]
============================================================
"""

import serial
import struct
import time
import argparse
from typing import Optional

# =============================================================================
# 硬件映射定义
# =============================================================================
# 步进电机ID
STEP_ID = {0x01: "步进1(切割)", 0x02: "步进2(传送带)", 0x03: "步进3(打包)"}
# 直流电机ID
DCM_ID  = {0x11: "DCM1(传送带)", 0x12: "DCM2(刀片1)", 0x13: "DCM3(刀片2)", 0x1F: "全部直流电机"}
# 方向
DIR_NAME = {0x00: "正转", 0x01: "反转"}

# ACK状态
ACK_STATUS = {0x00: "OK", 0x01: "PARAM_ERR", 0x02: "UNSUPPORT", 0x03: "BUSY", 0x04: "IDLE"}

# 命令名称（用于ACK显示）
CMD_NAME_MAP = {
    0x20: "CMD_DCM_START",
    0x21: "CMD_DCM_STOP",
    0x22: "CMD_STEPPER_START",
    0x23: "CMD_STEPPER_STOP",
    0x24: "CMD_SET_SPEED_DIR",
    0x25: "CMD_STEPPER_DIR",
    0x26: "CMD_DCM_DIR",
    0x27: "CMD_OFFSET_ADJUST",
    0x28: "CMD_OFFSET_RESET",
    0x2F: "CMD_ALL_STOP",
    0x30: "CMD_PROCESS_START",
    0x31: "CMD_PROCESS_STOP",
    0x32: "CMD_PROCESS_PAUSE",
    0x33: "CMD_PROCESS_RESUME",
    0x34: "CMD_SET_CLEAN_MODE",
    0x35: "CMD_SET_PACK_MODE",
    0x36: "CMD_SET_PARAM",
    0x37: "CMD_GET_STATUS",
}


# =============================================================================
# 帧封装
# =============================================================================
class ProtocolFrame:
    FRAME_HEAD = 0xAA

    @staticmethod
    def checksum(cmd: int, length: int, data: bytes) -> int:
        cs = cmd ^ length
        for b in data:
            cs ^= b
        return cs

    @staticmethod
    def build(cmd: int, data: bytes = b"") -> bytes:
        length = len(data)
        cs = ProtocolFrame.checksum(cmd, length, data)
        return bytes([ProtocolFrame.FRAME_HEAD, cmd, length]) + data + bytes([cs])

    @staticmethod
    def parse(raw: bytes) -> dict:
        """解析收到的帧，返回字典或None"""
        if len(raw) < 4 or raw[0] != ProtocolFrame.FRAME_HEAD:
            return None
        cmd, length = raw[1], raw[2]
        if len(raw) < 4 + length:
            return None
        data = raw[3:3 + length]
        cs = raw[3 + length]
        expected_cs = ProtocolFrame.checksum(cmd, length, data)
        return {
            "cmd": cmd, "len": length, "data": data, "checksum": cs,
            "valid": (cs == expected_cs)
        }

    @staticmethod
    def parse_ack(raw: bytes) -> Optional[dict]:
        """解析ACK帧，返回状态信息或None"""
        frame = ProtocolFrame.parse(raw)
        if not frame or not frame["valid"]:
            return None
        cmd = frame["cmd"]
        if cmd < 0x80:  # ACK命令 = 原命令 + 0x80
            return None
        data = frame["data"]
        if len(data) < 2:
            return None
        original_cmd = cmd - 0x80
        status = data[0]
        device_id = data[1]
        status_map = {
            0x00: "OK(执行成功)",
            0x01: "PARAM_ERR(参数错误)",
            0x02: "UNSUPPORT(不支持)",
            0x03: "BUSY(设备忙)",
            0x04: "IDLE(设备已停止)"
        }
        return {
            "original_cmd": original_cmd,
            "status": status,
            "status_name": status_map.get(status, f"未知({status:02X})"),
            "device_id": device_id
        }


# =============================================================================
# 指令构建函数
# =============================================================================
class VoiceProtocol:
    # --- 直流电机命令 ---
    @staticmethod
    def dcm_start(dev_id: int, duty: int, direction: int = 0x00) -> bytes:
        """直流电机启动（绝对值设置，自动清偏移）"""
        return ProtocolFrame.build(0x20, bytes([dev_id, duty, direction]))

    @staticmethod
    def dcm_stop(dev_id: int) -> bytes:
        """直流电机停止"""
        return ProtocolFrame.build(0x21, bytes([dev_id]))

    @staticmethod
    def dcm_dir(dev_id: int, direction: int) -> bytes:
        """直流电机设置方向（下次Run生效）"""
        return ProtocolFrame.build(0x26, bytes([dev_id, direction]))

    # --- 步进电机命令 ---
    @staticmethod
    def stepper_start(dev_id: int, speed_hz: int, direction: int = 0x00) -> bytes:
        """步进电机启动（绝对值设置，自动清偏移）"""
        speed_b = struct.pack("<H", speed_hz)  # 小端序: 高8位在前
        return ProtocolFrame.build(0x22, bytes([dev_id]) + speed_b)

    @staticmethod
    def stepper_stop(dev_id: int) -> bytes:
        """步进电机停止"""
        return ProtocolFrame.build(0x23, bytes([dev_id]))

    @staticmethod
    def stepper_dir(dev_id: int, direction: int) -> bytes:
        """步进电机设置方向"""
        return ProtocolFrame.build(0x25, bytes([dev_id, direction]))

    # --- 通用速度+方向命令 ---
    @staticmethod
    def set_speed_dir(dev_id: int, dev_type: int, direction: int,
                      value: int) -> bytes:
        """
        通用设置转速+方向（绝对值，自动清偏移）
        dev_type: 0x01=步进电机, 0x02=直流电机
        value:    步进=速度Hz(1~20000), 直流=占空比%(0~100)
        """
        value_b = struct.pack("<H", value)
        return ProtocolFrame.build(0x24, bytes([dev_id, dev_type, direction]) + value_b)

    # --- 偏移量命令 ---
    @staticmethod
    def offset_adjust(dev_id: int, dev_type: int, signed_offset: int) -> bytes:
        """
        调整偏移量（加/减），不清零
        dev_type: 0x01=步进电机, 0x02=直流电机
        signed_offset: 步进=-20000~+20000, 直流=-100~+100
        """
        # int16 有符号，struct.pack('<h') 自动处理符号
        off_b = struct.pack("<h", signed_offset)
        return ProtocolFrame.build(0x27, bytes([dev_id, dev_type]) + off_b)

    @staticmethod
    def offset_reset(dev_id: int) -> bytes:
        """
        清零偏移量
        0x01~0x03=步进电机, 0x11~0x13=直流电机, 0x1F=全部
        """
        return ProtocolFrame.build(0x28, bytes([dev_id]))

    # --- 急停 ---
    @staticmethod
    def all_stop() -> bytes:
        """全部急停（步进+直流）"""
        return ProtocolFrame.build(0x2F)

    # --- 工艺流程命令 ---
    @staticmethod
    def process_start() -> bytes:
        """开始工艺流程"""
        return ProtocolFrame.build(0x30)

    @staticmethod
    def process_stop() -> bytes:
        """停止工艺流程"""
        return ProtocolFrame.build(0x31)

    @staticmethod
    def process_pause() -> bytes:
        """暂停工艺流程"""
        return ProtocolFrame.build(0x32)

    @staticmethod
    def process_resume() -> bytes:
        """恢复工艺流程"""
        return ProtocolFrame.build(0x33)

    @staticmethod
    def set_clean_mode(mode: int) -> bytes:
        """
        设置清洗模式
        mode: 0x01=仅喷气  0x02=仅水泵  0x03=喷气+水泵
        """
        return ProtocolFrame.build(0x34, bytes([mode]))

    @staticmethod
    def set_pack_mode(mode: int) -> bytes:
        """
        设置打包模式
        mode: 0x01=全部  0x02=中间  0x03=头尾
        """
        return ProtocolFrame.build(0x35, bytes([mode]))

    @staticmethod
    def set_param(param_id: int, value: int) -> bytes:
        """
        设置工艺参数
        param_id: 参数ID (见process_control.h定义)
        value: 参数值 (0~65535)
        """
        value_b = struct.pack(">H", value)  # 大端序：高8位在前
        return ProtocolFrame.build(0x36, bytes([param_id]) + value_b)

    @staticmethod
    def get_status() -> bytes:
        """查询工艺状态"""
        return ProtocolFrame.build(0x37)


# =============================================================================
# 串口通信封装
# =============================================================================
class MotorController:
    def __init__(self, port: str = "COM3", baudrate: int = 115200, timeout: float = 1.0):
        self.ser: Optional[serial.Serial] = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

    def open(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(0.1)
            print(f"[OK] 串口 {self.port} 打开成功，波特率 {self.baudrate}")
            return True
        except serial.SerialException as e:
            print(f"[ERROR] 串口打开失败: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[OK] 串口已关闭")

    def send(self, frame: bytes, show_hex: bool = True) -> bool:
        if not self.ser or not self.ser.is_open:
            print("[ERROR] 串口未打开")
            return False
        try:
            self.ser.write(frame)
            if show_hex:
                print(f"[TX] {' '.join(f'{b:02X}' for b in frame)}")
            return True
        except serial.SerialException as e:
            print(f"[ERROR] 发送失败: {e}")
            return False

    def recv(self, max_bytes: int = 64) -> Optional[bytes]:
        if self.ser and self.ser.in_waiting > 0:
            raw = self.ser.read(min(self.ser.in_waiting, max_bytes))
            print(f"[RX] {' '.join(f'{b:02X}' for b in raw)}")
            ack = ProtocolFrame.parse_ack(raw)
            if ack:
                cmd_name = CMD_NAME_MAP.get(ack["original_cmd"], f"0x{ack['original_cmd']:02X}")
                print(f"     ACK ← {cmd_name}  状态={ack['status_name']}  设备ID=0x{ack['device_id']:02X}")
                return raw
            # 解析 GET_STATUS 响应帧（8字节数据）
            frame = ProtocolFrame.parse(raw)
            if frame and frame["valid"] and frame["cmd"] == 0xB7:  # 0x37 + 0x80
                data = frame["data"]
                if len(data) >= 8:
                    phase_names = ["IDLE","进料","测高","调高","切割准备",
                                   "切割","清洗","传送","称重","打包","完成"]
                    clean_names = ["IDLE","ENTER","AIR_ON","AIR_OFF","WATER_ON","WATER_OFF","DONE"]
                    pack_names  = ["IDLE","ENTER","检测头","卷头","卷头等","卷尾",
                                   "卷尾等","切断","切断等","第一次","第一次等","第二次",
                                   "第二次等","DONE"]
                    p  = data[0];  if p  >= len(phase_names): p  = 0
                    c  = data[1];  if c  >= len(clean_names): c  = 0
                    pk = data[2];  if pk >= len(pack_names):  pk = 0
                    print(f"     STATUS 主流程={phase_names[p]}  清洗={clean_names[c]}"
                          f"  打包={pack_names[pk]}  清洗模式=0x{data[3]:02X}"
                          f"  打包模式=0x{data[4]:02X}")
            return raw
        return None


# =============================================================================
# 预设快捷指令（语音场景封装）
# =============================================================================
class VoiceCommands:
    """供语音模块直接调用的语义化接口"""

    def __init__(self, ctrl: MotorController):
        self.ctrl = ctrl

    def _send(self, frame: bytes, desc: str = ""):
        ok = self.ctrl.send(frame)
        if desc:
            print(f"  → {desc}")
        return ok

    # ---- 直流电机 ----
    def dcm1_forward(self, duty: int):
        """传送带正转"""
        return self._send(VoiceProtocol.dcm_start(0x11, duty, 0x00),
                          f"DCM1(传送带)正转 {duty}%")

    def dcm1_reverse(self, duty: int):
        """传送带反转"""
        return self._send(VoiceProtocol.dcm_start(0x11, duty, 0x01),
                          f"DCM1(传送带)反转 {duty}%")

    def dcm2_forward(self, duty: int):
        """刀片1正转"""
        return self._send(VoiceProtocol.dcm_start(0x12, duty, 0x00),
                          f"DCM2(刀片1)正转 {duty}%")

    def dcm2_reverse(self, duty: int):
        """刀片1反转"""
        return self._send(VoiceProtocol.dcm_start(0x12, duty, 0x01),
                          f"DCM2(刀片1)反转 {duty}%")

    def dcm3_forward(self, duty: int):
        """刀片2正转"""
        return self._send(VoiceProtocol.dcm_start(0x13, duty, 0x00),
                          f"DCM3(刀片2)正转 {duty}%")

    def dcm3_reverse(self, duty: int):
        """刀片2反转"""
        return self._send(VoiceProtocol.dcm_start(0x13, duty, 0x01),
                          f"DCM3(刀片2)反转 {duty}%")

    def dcm_all_forward(self, duty: int):
        """全部直流电机正转"""
        return self._send(VoiceProtocol.dcm_start(0x1F, duty, 0x00),
                          f"全部直流电机正转 {duty}%")

    def dcm1_stop(self):
        return self._send(VoiceProtocol.dcm_stop(0x11), "DCM1停止")
    def dcm2_stop(self):
        return self._send(VoiceProtocol.dcm_stop(0x12), "DCM2停止")
    def dcm3_stop(self):
        return self._send(VoiceProtocol.dcm_stop(0x13), "DCM3停止")
    def dcm_all_stop(self):
        return self._send(VoiceProtocol.dcm_stop(0x1F), "全部直流停止")

    # ---- 步进电机 ----
    def stepper1_forward(self, speed_hz: int):
        """步进1(切割)正转"""
        return self._send(VoiceProtocol.stepper_start(0x01, speed_hz, 0x00),
                          f"步进1(切割)正转 {speed_hz}Hz")
    def stepper1_reverse(self, speed_hz: int):
        return self._send(VoiceProtocol.stepper_start(0x01, speed_hz, 0x01),
                          f"步进1(切割)反转 {speed_hz}Hz")
    def stepper2_forward(self, speed_hz: int):
        return self._send(VoiceProtocol.stepper_start(0x02, speed_hz, 0x00),
                          f"步进2(传送带)正转 {speed_hz}Hz")
    def stepper2_reverse(self, speed_hz: int):
        return self._send(VoiceProtocol.stepper_start(0x02, speed_hz, 0x01),
                          f"步进2(传送带)反转 {speed_hz}Hz")
    def stepper3_forward(self, speed_hz: int):
        return self._send(VoiceProtocol.stepper_start(0x03, speed_hz, 0x00),
                          f"步进3(打包)正转 {speed_hz}Hz")
    def stepper3_reverse(self, speed_hz: int):
        return self._send(VoiceProtocol.stepper_start(0x03, speed_hz, 0x01),
                          f"步进3(打包)反转 {speed_hz}Hz")
    def stepper1_stop(self):
        return self._send(VoiceProtocol.stepper_stop(0x01), "步进1停止")
    def stepper2_stop(self):
        return self._send(VoiceProtocol.stepper_stop(0x02), "步进2停止")
    def stepper3_stop(self):
        return self._send(VoiceProtocol.stepper_stop(0x03), "步进3停止")

    # ---- 速度+方向 ----
    def set_stepper_speed_dir(self, dev_id: int, speed_hz: int, forward: bool):
        """通用步进速度+方向设置"""
        direction = 0x00 if forward else 0x01
        name = STEP_ID.get(dev_id, f"未知({dev_id:02X})")
        dir_name = "正转" if forward else "反转"
        return self._send(
            VoiceProtocol.set_speed_dir(dev_id, 0x01, direction, speed_hz),
            f"{name} {dir_name} {speed_hz}Hz"
        )

    def set_dcm_speed_dir(self, dev_id: int, duty: int, forward: bool):
        """通用直流速度+方向设置"""
        direction = 0x00 if forward else 0x01
        name = DCM_ID.get(dev_id, f"未知({dev_id:02X})")
        dir_name = "正转" if forward else "反转"
        return self._send(
            VoiceProtocol.set_speed_dir(dev_id, 0x02, direction, duty),
            f"{name} {dir_name} {duty}%"
        )

    # ---- 偏移量（语音"加速/减速"） ----
    def dcm1_speed_up(self, delta: int = 5):
        """DCM1占空比 +delta"""
        return self._send(VoiceProtocol.offset_adjust(0x11, 0x02, delta),
                          f"DCM1占空比 +{delta}")
    def dcm1_slow_down(self, delta: int = 5):
        return self._send(VoiceProtocol.offset_adjust(0x11, 0x02, -delta),
                          f"DCM1占空比 -{delta}")
    def dcm2_speed_up(self, delta: int = 5):
        return self._send(VoiceProtocol.offset_adjust(0x12, 0x02, delta),
                          f"DCM2占空比 +{delta}")
    def dcm2_slow_down(self, delta: int = 5):
        return self._send(VoiceProtocol.offset_adjust(0x12, 0x02, -delta),
                          f"DCM2占空比 -{delta}")
    def dcm3_speed_up(self, delta: int = 5):
        return self._send(VoiceProtocol.offset_adjust(0x13, 0x02, delta),
                          f"DCM3占空比 +{delta}")
    def dcm3_slow_down(self, delta: int = 5):
        return self._send(VoiceProtocol.offset_adjust(0x13, 0x02, -delta),
                          f"DCM3占空比 -{delta}")

    def stepper1_speed_up(self, delta: int = 100):
        """步进1速度 +delta Hz"""
        return self._send(VoiceProtocol.offset_adjust(0x01, 0x01, delta),
                          f"步进1速度 +{delta}Hz")
    def stepper1_slow_down(self, delta: int = 100):
        return self._send(VoiceProtocol.offset_adjust(0x01, 0x01, -delta),
                          f"步进1速度 -{delta}Hz")
    def stepper2_speed_up(self, delta: int = 100):
        return self._send(VoiceProtocol.offset_adjust(0x02, 0x01, delta),
                          f"步进2速度 +{delta}Hz")
    def stepper2_slow_down(self, delta: int = 100):
        return self._send(VoiceProtocol.offset_adjust(0x02, 0x01, -delta),
                          f"步进2速度 -{delta}Hz")

    def dcm1_reset_offset(self):
        return self._send(VoiceProtocol.offset_reset(0x11), "DCM1偏移量清零")
    def dcm2_reset_offset(self):
        return self._send(VoiceProtocol.offset_reset(0x12), "DCM2偏移量清零")
    def dcm3_reset_offset(self):
        return self._send(VoiceProtocol.offset_reset(0x13), "DCM3偏移量清零")
    def all_offset_reset(self):
        return self._send(VoiceProtocol.offset_reset(0x1F), "全部偏移量清零")

    # ---- 急停 ----
    def all_stop(self):
        return self._send(VoiceProtocol.all_stop(), "===== 全部急停 =====")

    # ---- 工艺流程 ----
    def process_start(self):
        return self._send(VoiceProtocol.process_start(), "工艺流程启动")
    def process_stop(self):
        return self._send(VoiceProtocol.process_stop(), "工艺流程停止")
    def process_pause(self):
        return self._send(VoiceProtocol.process_pause(), "工艺流程暂停")
    def process_resume(self):
        return self._send(VoiceProtocol.process_resume(), "工艺流程恢复")

    # ---- 清洗模式 ----
    def clean_mode_air(self):
        """清洗模式：仅喷气"""
        return self._send(VoiceProtocol.set_clean_mode(0x01), "清洗模式→仅喷气")
    def clean_mode_water(self):
        """清洗模式：仅水泵"""
        return self._send(VoiceProtocol.set_clean_mode(0x02), "清洗模式→仅水泵")
    def clean_mode_both(self):
        """清洗模式：喷气+水泵"""
        return self._send(VoiceProtocol.set_clean_mode(0x03), "清洗模式→喷气+水泵")

    # ---- 打包模式 ----
    def pack_mode_full(self):
        """打包模式：全部"""
        return self._send(VoiceProtocol.set_pack_mode(0x01), "打包模式→全部")
    def pack_mode_middle(self):
        """打包模式：中间"""
        return self._send(VoiceProtocol.set_pack_mode(0x02), "打包模式→中间")
    def pack_mode_head_tail(self):
        """打包模式：头尾"""
        return self._send(VoiceProtocol.set_pack_mode(0x03), "打包模式→头尾")

    # ---- 参数查询 ----
    def get_process_status(self):
        """查询工艺状态"""
        return self._send(VoiceProtocol.get_status(), "查询工艺状态")


# =============================================================================
# 命令行交互
# =============================================================================
COMMANDS = {
    # --- 直流电机 ---
    "DCM1_START":      lambda v: v.dcm1_forward(80),
    "DCM1_START_50":   lambda v: v.dcm1_forward(50),
    "DCM2_START":      lambda v: v.dcm2_forward(60),
    "DCM3_START":      lambda v: v.dcm3_forward(60),
    "DCM_ALL_START":   lambda v: v.dcm_all_forward(70),
    "DCM1_STOP":       lambda v: v.dcm1_stop(),
    "DCM2_STOP":       lambda v: v.dcm2_stop(),
    "DCM3_STOP":       lambda v: v.dcm3_stop(),
    "DCM_ALL_STOP":    lambda v: v.dcm_all_stop(),
    # --- 步进电机 ---
    "S1_START":        lambda v: v.stepper1_forward(2000),
    "S1_START_1000":   lambda v: v.stepper1_forward(1000),
    "S2_START":        lambda v: v.stepper2_forward(2000),
    "S3_START":        lambda v: v.stepper3_forward(2000),
    "S1_STOP":         lambda v: v.stepper1_stop(),
    "S2_STOP":         lambda v: v.stepper2_stop(),
    "S3_STOP":         lambda v: v.stepper3_stop(),
    # --- 偏移量加减 ---
    "DCM1_UP":         lambda v: v.dcm1_speed_up(5),
    "DCM1_DOWN":       lambda v: v.dcm1_slow_down(5),
    "DCM2_UP":         lambda v: v.dcm2_speed_up(5),
    "DCM2_DOWN":       lambda v: v.dcm2_slow_down(5),
    "S1_UP":           lambda v: v.stepper1_speed_up(100),
    "S1_DOWN":         lambda v: v.stepper1_slow_down(100),
    "S2_UP":           lambda v: v.stepper2_speed_up(100),
    # --- 偏移量清零 ---
    "DCM1_RESET":      lambda v: v.dcm1_reset_offset(),
    "DCM_ALL_RESET":   lambda v: v.all_offset_reset(),
    # --- 急停 ---
    "ALL_STOP":        lambda v: v.all_stop(),
    # --- 工艺流程 ---
    "PROCESS_START":   lambda v: v.process_start(),
    "PROCESS_STOP":    lambda v: v.process_stop(),
    "PROCESS_PAUSE":   lambda v: v.process_pause(),
    "PROCESS_RESUME":  lambda v: v.process_resume(),
    "CLEAN_AIR":      lambda v: v.clean_mode_air(),
    "CLEAN_WATER":     lambda v: v.clean_mode_water(),
    "CLEAN_BOTH":      lambda v: v.clean_mode_both(),
    "PACK_FULL":       lambda v: v.pack_mode_full(),
    "PACK_MIDDLE":     lambda v: v.pack_mode_middle(),
    "PACK_HEAD_TAIL":  lambda v: v.pack_mode_head_tail(),
    "GET_STATUS":      lambda v: v.get_process_status(),
    # --- 组合场景 ---
    "CUTTING":         lambda v: (
        v.stepper1_forward(2000) or
        v.dcm2_forward(60) or
        v.dcm3_forward(60)
    ),
    "CONVEYOR":        lambda v: v.dcm1_forward(80),
}


def auto_demo(voice: VoiceCommands):
    """自动演示：切割场景"""
    print("\n========== 自动演示：切割场景 ==========")
    print("1. 启动传送带 DCM1 80%...")
    voice.dcm1_forward(80)
    time.sleep(1)
    print("2. 启动切割步进1 2000Hz...")
    voice.stepper1_forward(2000)
    time.sleep(1)
    print("3. 刀片1加速 +10%...")
    voice.dcm2_speed_up(10)
    time.sleep(1)
    print("4. 刀片2加速 +5%...")
    voice.dcm3_speed_up(5)
    time.sleep(1)
    print("5. 刀片1设置为60% (绝对值，自动清零偏移)...")
    voice.dcm2_forward(60)
    time.sleep(1)
    print("6. 全部急停...")
    voice.all_stop()
    time.sleep(1)
    print("\n========== 演示完成 ==========")


def interactive_mode(ctrl: MotorController):
    """交互模式"""
    voice = VoiceCommands(ctrl)
    print("\n========== 交互模式 ==========")
    print("可用命令:", ", ".join(COMMANDS.keys()))
    print("输入 'quit' 退出\n")

    while True:
        try:
            user = input("> ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\n退出")
            break
        if not user:
            continue
        if user.lower() in ("quit", "exit", "q"):
            print("退出")
            break
        if user == "demo":
            auto_demo(voice)
            continue
        if user not in COMMANDS:
            print(f"未知命令: {user}")
            print("可用:", ", ".join(COMMANDS.keys()))
            continue
        COMMANDS[user](voice)
        time.sleep(0.1)
        ctrl.recv()


# =============================================================================
# 主入口
# =============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="大葱切割-清洗-打包一体机 上位机测试")
    parser.add_argument("--port",  default="COM3", help="串口名, 默认COM3")
    parser.add_argument("--baud",  type=int, default=115200, help="波特率, 默认115200")
    parser.add_argument("--send",  help="单条命令(如 DCM1_START)")
    parser.add_argument("--auto",  action="store_true", help="自动演示模式")
    args = parser.parse_args()

    ctrl = MotorController(args.port, args.baud)
    if not ctrl.open():
        exit(1)

    voice = VoiceCommands(ctrl)

    if args.send:
        if args.send in COMMANDS:
            print(f"发送: {args.send}")
            COMMANDS[args.send](voice)
            time.sleep(0.2)
            ctrl.recv()
        else:
            print(f"未知命令: {args.send}")
    elif args.auto:
        auto_demo(voice)
    else:
        interactive_mode(ctrl)

    ctrl.close()
