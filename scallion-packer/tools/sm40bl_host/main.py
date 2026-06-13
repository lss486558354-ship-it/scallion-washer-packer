#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
飞特 SM40BL-C001 等 RS485 总线舵机 — 控制上位机（仅发送）

依据公开资料整理（请以飞特最新 PDF 为准）：
  - 产品页：SM-40BL-C001，RS485 半双工异步串行、总线数据包通讯
  - 位置分辨率：0～4095 对应 360°（官网规格表）
  - 通讯速率：约 38400 bps～1 Mbps（以舵机内配置为准）
  - 协议帧与飞特《舵机协议手册-磁编码版本》一致：FF FF ID LEN INST 参数… CHK

硬件：SM40BL 为 RS485 电平，PC 侧请使用 USB-RS485 转换器（或飞特转接板），
 接到 A/B 差分线；GND 共地参考转换器说明。不可把普通 USB-TTL 直接当 RS485 用。

本程序只向串口写数据，不读回包。
"""

from __future__ import annotations

import sys
from typing import Optional

import serial
from serial.tools import list_ports

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

# —— 飞特总线协议（与 SCS/SMS 磁编码常用写法一致，具体寄存器以该型号内存表为准）——
SCS_INST_WRITE = 0x03
ADDR_GOAL_BLOCK = 0x2A
ADDR_GOAL_SPEED = 0x2E
POS_MAX = 4095


def scs_checksum(servo_id: int, length: int, inst: int, addr_and_data: bytes) -> int:
    s = servo_id + length + inst
    for b in addr_and_data:
        s += b
    return (~s) & 0xFF


def build_write_packet(servo_id: int, start_addr: int, data: bytes) -> bytes:
    if not (0 <= servo_id <= 253):
        raise ValueError("ID 应在 0～253")
    inst = SCS_INST_WRITE
    addr_and_data = bytes([start_addr]) + data
    length = 1 + len(addr_and_data) + 1
    chk = scs_checksum(servo_id, length, inst, addr_and_data)
    return bytes([0xFF, 0xFF, servo_id, length, inst]) + addr_and_data + bytes([chk])


def u16_le(v: int) -> bytes:
    v = int(v) & 0xFFFF
    return bytes([v & 0xFF, (v >> 8) & 0xFF])


def i16_le(v: int) -> bytes:
    v = int(v)
    if v < 0:
        v += 65536
    v &= 0xFFFF
    return bytes([v & 0xFF, (v >> 8) & 0xFF])


def angle_to_pos(angle_0_360: float) -> int:
    p = int(round(angle_0_360 / 360.0 * POS_MAX))
    return max(0, min(POS_MAX, p))


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("飞特 SM40BL / RS485 总线舵机（仅发送）")
        self._ser: Optional[serial.Serial] = None
        self._sent = 0

        root_w = QWidget()
        self.setCentralWidget(root_w)
        root = QVBoxLayout(root_w)

        lbl = QLabel(
            "<b>SM40BL</b> 为 <b>RS485</b> 半双工总线舵机；请用 <b>USB-RS485</b> 接电脑 COM 口。"
            "协议为飞特标准总线包（与磁编码系列常用寄存器一致：<b>0x2A</b> 目标位置块、"
            "<b>0x2E</b> 目标速度等）。若与实物不符，请以官网「内存表 / 规格书」为准。"
        )
        lbl.setWordWrap(True)
        lbl.setTextFormat(Qt.TextFormat.RichText)
        root.addWidget(lbl)

        g0 = QGroupBox("串口（RS485 适配器）")
        g0l = QGridLayout(g0)
        self.port = QComboBox()
        self.port.setMinimumWidth(180)
        b_ref = QPushButton("刷新端口")
        b_ref.clicked.connect(self._refresh_ports)
        self.baud = QComboBox()
        for b in (1_000_000, 500_000, 250_000, 128_000, 115200, 76_800, 57_600, 38_400):
            self.baud.addItem(str(b), b)
        i = self.baud.findText("115200")
        if i >= 0:
            self.baud.setCurrentIndex(i)
        self.btn_conn = QPushButton("打开串口")
        self.btn_conn.clicked.connect(self._toggle_serial)
        g0l.addWidget(QLabel("端口"), 0, 0)
        g0l.addWidget(self.port, 0, 1)
        g0l.addWidget(b_ref, 0, 2)
        g0l.addWidget(QLabel("波特率"), 1, 0)
        g0l.addWidget(self.baud, 1, 1)
        g0l.addWidget(self.btn_conn, 1, 2)
        root.addWidget(g0)

        g1 = QGroupBox("目标舵机")
        f1 = QFormLayout(g1)
        self.spin_id = QSpinBox()
        self.spin_id.setRange(0, 253)
        self.spin_id.setValue(1)
        f1.addRow("ID", self.spin_id)
        root.addWidget(g1)

        g2 = QGroupBox("位置控制（0x2A：位置 + 时间 + 速度，小端，与飞特例程一致）")
        g2l = QGridLayout(g2)
        self.spin_pos = QSpinBox()
        self.spin_pos.setRange(0, POS_MAX)
        self.spin_pos.setValue(2048)
        self.spin_ang = QDoubleSpinBox()
        self.spin_ang.setRange(0.0, 360.0)
        self.spin_ang.setDecimals(2)
        self.spin_ang.setSuffix(" °")
        self.spin_ang.setValue(180.0)
        self.chk_sync_ang = QCheckBox("用角度同步到「目标位置」")
        self.chk_sync_ang.setChecked(True)
        self.spin_ang.valueChanged.connect(self._on_angle_changed)
        self.spin_pos.valueChanged.connect(self._on_pos_changed)
        self.spin_time = QSpinBox()
        self.spin_time.setRange(0, 65535)
        self.spin_time.setValue(0)
        self.spin_spd = QSpinBox()
        self.spin_spd.setRange(0, 65535)
        self.spin_spd.setValue(1000)
        b_send_goal = QPushButton("发送位置块")
        b_send_goal.clicked.connect(self._send_goal)
        g2l.addWidget(QLabel("目标位置 0～4095"), 0, 0)
        g2l.addWidget(self.spin_pos, 0, 1)
        g2l.addWidget(QLabel("或角度0～360°"), 1, 0)
        g2l.addWidget(self.spin_ang, 1, 1)
        g2l.addWidget(self.chk_sync_ang, 2, 0, 1, 2)
        g2l.addWidget(QLabel("目标时间"), 3, 0)
        g2l.addWidget(self.spin_time, 3, 1)
        g2l.addWidget(QLabel("目标速度"), 4, 0)
        g2l.addWidget(self.spin_spd, 4, 1)
        g2l.addWidget(b_send_goal, 5, 0, 1, 2)
        root.addWidget(g2)

        g3 = QGroupBox("轮模式 / 速度（寄存器 0x2E，有符号 16 位小端；需舵机已切轮模式）")
        h3 = QHBoxLayout(g3)
        self.spin_wheel = QSpinBox()
        self.spin_wheel.setRange(-32768, 32767)
        self.spin_wheel.setValue(0)
        bw = QPushButton("发送速度")
        bw.clicked.connect(self._send_wheel)
        bz = QPushButton("停（0）")
        bz.clicked.connect(self._send_wheel_zero)
        h3.addWidget(QLabel("速度"))
        h3.addWidget(self.spin_wheel)
        h3.addWidget(bw)
        h3.addWidget(bz)
        h3.addStretch()
        root.addWidget(g3)

        g4 = QGroupBox("原始帧（空格分隔十六进制，整帧含校验）")
        v4 = QVBoxLayout(g4)
        self.ed_hex = QLineEdit()
        self.ed_hex.setPlaceholderText("例：FF FF 01 09 03 2A 00 08 00 00 E8 03 D5")
        bh = QPushButton("发送")
        bh.clicked.connect(self._send_hex)
        v4.addWidget(self.ed_hex)
        v4.addWidget(bh)
        root.addWidget(g4)

        self.st = QLabel("就绪")
        self.st.setWordWrap(True)
        root.addWidget(self.st)

        self._refresh_ports()
        self._on_angle_changed(self.spin_ang.value())

    def _on_angle_changed(self, v: float) -> None:
        if not self.chk_sync_ang.isChecked():
            return
        self.spin_pos.blockSignals(True)
        self.spin_pos.setValue(angle_to_pos(v))
        self.spin_pos.blockSignals(False)

    def _on_pos_changed(self, _v: int) -> None:
        if not self.chk_sync_ang.isChecked():
            return
        p = self.spin_pos.value()
        ang = p *360.0 / POS_MAX
        self.spin_ang.blockSignals(True)
        self.spin_ang.setValue(round(ang, 2))
        self.spin_ang.blockSignals(False)

    def _refresh_ports(self) -> None:
        self.port.clear()
        for p in list_ports.comports():
            self.port.addItem(f"{p.device} — {p.description}", p.device)

    def _toggle_serial(self) -> None:
        if self._ser is not None and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            self.btn_conn.setText("打开串口")
            self._st("已关闭串口")
            return
        if self.port.count() == 0:
            QMessageBox.warning(self, "提示", "无可用串口，请插入 USB-RS485 后刷新。")
            return
        dev = self.port.currentData()
        br = self.baud.currentData()
        try:
            self._ser = serial.Serial(port=dev, baudrate=br, timeout=0, write_timeout=1)
        except serial.SerialException as e:
            QMessageBox.critical(self, "失败", str(e))
            self._ser = None
            return
        self.btn_conn.setText("关闭串口")
        self._st(f"已打开 {dev} @ {br}（仅发送）")

    def _need_ser(self) -> bool:
        if self._ser is None or not self._ser.is_open:
            QMessageBox.warning(self, "提示", "请先打开串口。")
            return False
        return True

    def _write(self, data: bytes, note: str = "") -> bool:
        if self._ser is None or not self._ser.is_open:
            self._st("串口未打开")
            return False
        try:
            n = self._ser.write(data)
            self._ser.flush()
        except (serial.SerialException, OSError) as e:
            self._st(f"发送失败：{e!s} — 请检查 RS485 与 COM 口")
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            self.btn_conn.setText("打开串口")
            return False
        self._sent += n
        hx = data.hex(" ").upper()
        msg = f"已发 {n} 字节，累计 {self._sent}：{hx}"
        self._st(f"{note + ' | ' if note else ''}{msg}")
        return True

    def _st(self, t: str) -> None:
        self.st.setText(t)

    def _send_goal(self) -> None:
        if not self._need_ser():
            return
        sid = self.spin_id.value()
        body = u16_le(self.spin_pos.value()) + u16_le(self.spin_time.value()) + u16_le(self.spin_spd.value())
        try:
            pkt = build_write_packet(sid, ADDR_GOAL_BLOCK, body)
        except ValueError as e:
            QMessageBox.warning(self, "参数错误", str(e))
            return
        self._write(pkt, note="0x2A 位置块")

    def _send_wheel(self) -> None:
        if not self._need_ser():
            return
        sid = self.spin_id.value()
        try:
            pkt = build_write_packet(sid, ADDR_GOAL_SPEED, i16_le(self.spin_wheel.value()))
        except ValueError as e:
            QMessageBox.warning(self, "参数错误", str(e))
            return
        self._write(pkt, note="0x2E 速度")

    def _send_wheel_zero(self) -> None:
        self.spin_wheel.setValue(0)
        self._send_wheel()

    def _send_hex(self) -> None:
        if not self._need_ser():
            return
        t = self.ed_hex.text().strip()
        if not t:
            return
        try:
            data = bytes(int(x, 16) for x in t.replace(",", " ").split())
        except ValueError:
            QMessageBox.warning(self, "格式错误", "请用空格分隔十六进制字节")
            return
        self._write(data, note="原始")


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MainWindow()
    w.resize(520, 640)
    w.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
