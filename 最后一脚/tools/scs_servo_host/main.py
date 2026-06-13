#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
飞特 SCS 串口舵机 — 仅发送上位机（PySide6 + pyserial）
不读取串口回包，适合半双工或只关心下发的场景。

硬件：飞特 SCS 常见为 3 芯——电源正、GND、单根信号线（半双工总线）。
USB-TTL 侧仍是 TX/RX 两线；接到舵机前须按飞特手册用二极管与电阻把 TX、RX 汇合到一根信号线
再接舵机信号脚，禁止把 USB-TTL 的 TX 与 RX 直接短接。
GND 与舵机 GND 必须共地；舵机电源按额定电压电流供电。
电平与波特率须与舵机一致。
"""

from __future__ import annotations

import sys
from typing import Optional

import serial
from serial.tools import list_ports

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
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


SCS_INST_WRITE = 0x03
SCS_ADDR_GOAL_BLOCK = 0x2A
SCS_ADDR_GOAL_SPEED = 0x2E


def scs_checksum(servo_id: int, length: int, inst: int, addr_and_data: bytes) -> int:
    s = servo_id + length + inst
    for b in addr_and_data:
        s += b
    return (~s) & 0xFF


def build_write_packet(servo_id: int, start_addr: int, data: bytes) -> bytes:
    """SCS「写数据」指令帧：FF FF ID LEN 03 地址 数据... CHK"""
    if not (0 <= servo_id <= 253):
        raise ValueError("舵机 ID 应在 0～253")
    if not (0 <= start_addr <= 255):
        raise ValueError("起始地址非法")
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
        v = v + 65536
    v &= 0xFFFF
    return bytes([v & 0xFF, (v >> 8) & 0xFF])


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("SCS 舵机控制（仅发送）")
        self._ser: Optional[serial.Serial] = None
        self._sent_bytes = 0

        self._traverse_active = False
        self._traverse_cur_id = 0
        self._traverse_timer = QTimer(self)
        self._traverse_timer.setSingleShot(True)
        self._traverse_timer.timeout.connect(self._on_traverse_tick)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # —— 串口 ——
        grp_serial = QGroupBox("串口")
        lay_s = QGridLayout(grp_serial)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(160)
        btn_refresh = QPushButton("刷新端口")
        btn_refresh.clicked.connect(self.refresh_ports)
        self.baud_combo = QComboBox()
        for b in (1_000_000, 500_000, 250_000, 115200, 57600, 38400, 19200, 9600):
            self.baud_combo.addItem(str(b), b)
        idx = self.baud_combo.findText("115200")
        if idx >= 0:
            self.baud_combo.setCurrentIndex(idx)
        self.btn_connect = QPushButton("打开串口")
        self.btn_connect.clicked.connect(self.toggle_serial)
        lay_s.addWidget(QLabel("端口"), 0, 0)
        lay_s.addWidget(self.port_combo, 0, 1)
        lay_s.addWidget(btn_refresh, 0, 2)
        lay_s.addWidget(QLabel("波特率"), 1, 0)
        lay_s.addWidget(self.baud_combo, 1, 1)
        lay_s.addWidget(self.btn_connect, 1, 2)
        root.addWidget(grp_serial)

        lbl_hw = QLabel(
            "<b>3 芯单线（半双工）+ USB-TTL</b>：舵机为 <b>VCC / GND / 信号</b> 三根；"
            "信号线是单根半双工总线，<b>不能</b>用「USB-TTL TX 单接舵机线」当普通三线串口用。"
            "须按飞特文档用 <b>二极管 + 电阻</b> 把 USB-TTL 的 TX、RX <b>汇合到一根线</b>再接舵机信号脚；"
            "<b>禁止</b>将 TX 与 RX 直接拧在一起。"
            "<b>GND 必须共地</b>；舵机电源按额定接（不要指望 USB-TTL 口能带满舵机电流）。"
            "本软件仅发不收时，电路仍建议按手册接全，便于以后读状态。波特率与舵机一致（常用 115200）。"
        )
        lbl_hw.setWordWrap(True)
        lbl_hw.setTextFormat(Qt.TextFormat.RichText)
        root.addWidget(lbl_hw)

        # —— 公共 ——
        grp_id = QGroupBox("目标舵机")
        lay_id = QFormLayout(grp_id)
        self.spin_id = QSpinBox()
        self.spin_id.setRange(0, 253)
        self.spin_id.setValue(1)
        lay_id.addRow("ID（0～253）", self.spin_id)
        root.addWidget(grp_id)

        # —— 轮模式：写 0x2E 速度 ——
        grp_wheel = QGroupBox("轮模式 · 目标速度（寄存器 0x2E，小端有符号 16 位）")
        lay_w = QHBoxLayout(grp_wheel)
        self.spin_wheel = QSpinBox()
        self.spin_wheel.setRange(-32768, 32767)
        self.spin_wheel.setValue(500)
        btn_wheel = QPushButton("发送")
        btn_wheel.clicked.connect(self.send_wheel_speed)
        lay_w.addWidget(QLabel("速度"))
        lay_w.addWidget(self.spin_wheel)
        lay_w.addWidget(btn_wheel)
        lay_w.addStretch()
        root.addWidget(grp_wheel)

        # —— 遍历 ID ——
        grp_tr = QGroupBox("遍历 ID（依次对每个 ID 下发，每 ID 持续一段时间）")
        lay_tr = QGridLayout(grp_tr)
        self.chk_traverse_enable = QCheckBox("启用遍历功能（与下方「开始遍历」配合；关闭时仍为单次发送）")
        self.chk_traverse_enable.setChecked(False)
        self.chk_traverse_enable.toggled.connect(self._on_traverse_switch_toggled)
        lay_tr.addWidget(self.chk_traverse_enable, 0, 0, 1, 4)
        self.spin_tr_min = QSpinBox()
        self.spin_tr_min.setRange(0, 253)
        self.spin_tr_min.setValue(1)
        self.spin_tr_max = QSpinBox()
        self.spin_tr_max.setRange(0, 253)
        self.spin_tr_max.setValue(4)
        self.spin_tr_dwell_ms = QSpinBox()
        self.spin_tr_dwell_ms.setRange(50, 600_000)
        self.spin_tr_dwell_ms.setValue(1000)
        self.spin_tr_dwell_ms.setSuffix(" ms")
        self.combo_tr_kind = QComboBox()
        self.combo_tr_kind.addItem("轮模式 · 目标速度（0x2E）", "wheel")
        self.combo_tr_kind.addItem("位置模式 · 0x2A 六字节", "goal")
        lay_tr.addWidget(QLabel("ID 从"), 1, 0)
        lay_tr.addWidget(self.spin_tr_min, 1, 1)
        lay_tr.addWidget(QLabel("到"), 1, 2)
        lay_tr.addWidget(self.spin_tr_max, 1, 3)
        lay_tr.addWidget(QLabel("每 ID 持续"), 2, 0)
        lay_tr.addWidget(self.spin_tr_dwell_ms, 2, 1)
        lay_tr.addWidget(QLabel("发送内容"), 2, 2)
        lay_tr.addWidget(self.combo_tr_kind, 2, 3)
        self.btn_tr_start = QPushButton("开始遍历")
        self.btn_tr_start.clicked.connect(self.traverse_start)
        self.btn_tr_stop = QPushButton("停止遍历")
        self.btn_tr_stop.clicked.connect(self.traverse_stop)
        self.btn_tr_stop.setEnabled(False)
        lay_tr.addWidget(self.btn_tr_start, 3, 0, 1, 2)
        lay_tr.addWidget(self.btn_tr_stop, 3, 2, 1, 2)
        root.addWidget(grp_tr)
        self._update_traverse_controls_enabled()

        # —— 位置块 0x2A ——
        grp_goal = QGroupBox("位置模式 · 0x2A 起 6 字节（位置/时间/速度，小端）")
        lay_g = QGridLayout(grp_goal)
        self.spin_pos = QSpinBox()
        self.spin_pos.setRange(0, 65535)
        self.spin_pos.setValue(2048)
        self.spin_time = QSpinBox()
        self.spin_time.setRange(0, 65535)
        self.spin_time.setValue(0)
        self.spin_gspd = QSpinBox()
        self.spin_gspd.setRange(0, 65535)
        self.spin_gspd.setValue(1000)
        btn_goal = QPushButton("发送")
        btn_goal.clicked.connect(self.send_goal_block)
        lay_g.addWidget(QLabel("目标位置"), 0, 0)
        lay_g.addWidget(self.spin_pos, 0, 1)
        lay_g.addWidget(QLabel("目标时间"), 1, 0)
        lay_g.addWidget(self.spin_time, 1, 1)
        lay_g.addWidget(QLabel("目标速度"), 2, 0)
        lay_g.addWidget(self.spin_gspd, 2, 1)
        lay_g.addWidget(btn_goal, 3, 0, 1, 2)
        root.addWidget(grp_goal)

        # —— 原始十六进制 ——
        grp_hex = QGroupBox("原始发送（空格分隔十六进制，整帧原样写出，含 FF FF 与校验）")
        lay_h = QVBoxLayout(grp_hex)
        self.edit_hex = QLineEdit()
        self.edit_hex.setPlaceholderText("示例：FF FF 01 09 03 2A 00 08 00 00 E8 03 D5")
        btn_hex = QPushButton("按字节发送（原样）")
        btn_hex.clicked.connect(self.send_raw_hex)
        lay_h.addWidget(self.edit_hex)
        lay_h.addWidget(btn_hex)
        root.addWidget(grp_hex)

        # —— 快捷 ——
        row_quick = QHBoxLayout()
        b_stop = QPushButton("停转（轮速 0）")
        b_stop.clicked.connect(self.send_stop_wheel)
        row_quick.addWidget(b_stop)
        row_quick.addStretch()
        root.addLayout(row_quick)

        self.status = QLabel("未连接")
        self.status.setWordWrap(True)
        root.addWidget(self.status)

        self.refresh_ports()

    def refresh_ports(self) -> None:
        self.port_combo.clear()
        for p in list_ports.comports():
            self.port_combo.addItem(f"{p.device} — {p.description}", p.device)

    def toggle_serial(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self.traverse_stop()
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
            self.btn_connect.setText("打开串口")
            self._set_status("已关闭串口")
            return
        if self.port_combo.count() == 0:
            QMessageBox.warning(self, "提示", "未找到串口，请检查 USB 转串口后点「刷新端口」。")
            return
        dev = self.port_combo.currentData()
        baud = self.baud_combo.currentData()
        try:
            self._ser = serial.Serial(
                port=dev,
                baudrate=baud,
                timeout=0,
                write_timeout=1,
            )
        except serial.SerialException as e:
            QMessageBox.critical(self, "打开失败", str(e))
            self._ser = None
            return
        self.btn_connect.setText("关闭串口")
        self._set_status(f"已打开 {dev} @ {baud}，仅发送模式（不读回包）")

    def _on_traverse_switch_toggled(self, checked: bool) -> None:
        if not checked and self._traverse_active:
            self.traverse_stop()
        self._update_traverse_controls_enabled()

    def _update_traverse_controls_enabled(self) -> None:
        en = self.chk_traverse_enable.isChecked() and not self._traverse_active
        self.spin_tr_min.setEnabled(en)
        self.spin_tr_max.setEnabled(en)
        self.spin_tr_dwell_ms.setEnabled(en)
        self.combo_tr_kind.setEnabled(en)
        self.btn_tr_start.setEnabled(self.chk_traverse_enable.isChecked() and not self._traverse_active)

    def _traverse_reset_state_only(self) -> None:
        """串口已无效时只复位遍历状态，不向串口写数据。"""
        self._traverse_timer.stop()
        self._traverse_active = False
        self.btn_tr_stop.setEnabled(False)
        self.btn_tr_start.setEnabled(self.chk_traverse_enable.isChecked())
        self._update_traverse_controls_enabled()

    def _handle_serial_write_failed(self, exc: BaseException) -> None:
        """WriteFile 失败等：关闭串口、复位遍历，避免未捕获异常。"""
        self._set_status(
            f"串口发送失败（设备可能已拔掉或占用）：{exc!s} — 请重插 USB 后重新「打开串口」。"
        )
        self._traverse_reset_state_only()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self.btn_connect.setText("打开串口")

    def traverse_stop(self) -> None:
        self._traverse_timer.stop()
        send_failed = False
        if self._traverse_active and self._ser is not None and self._ser.is_open:
            kind = self.combo_tr_kind.currentData()
            if kind == "wheel":
                try:
                    pkt = build_write_packet(self._traverse_cur_id, SCS_ADDR_GOAL_SPEED, i16_le(0))
                    if not self._write_only(pkt, note="停止遍历（轮速 0）"):
                        send_failed = True
                except ValueError:
                    pass
        self._traverse_active = False
        self.btn_tr_stop.setEnabled(False)
        self.btn_tr_start.setEnabled(self.chk_traverse_enable.isChecked())
        self._update_traverse_controls_enabled()
        if not send_failed:
            self._set_status(f"已停止遍历（累计已发 {self._sent_bytes} 字节）")

    def traverse_start(self) -> None:
        if not self.chk_traverse_enable.isChecked():
            QMessageBox.information(self, "提示", "请先勾选「启用遍历功能」。")
            return
        if not self._require_serial():
            return
        id_min = self.spin_tr_min.value()
        id_max = self.spin_tr_max.value()
        if id_min > id_max:
            QMessageBox.warning(self, "提示", "ID「从」不能大于「到」。")
            return
        self._traverse_timer.stop()
        self._traverse_active = True
        self._traverse_cur_id = id_min
        self.btn_tr_stop.setEnabled(True)
        self.btn_tr_start.setEnabled(False)
        self._update_traverse_controls_enabled()
        if not self._traverse_send_current():
            return
        self._traverse_timer.start(self.spin_tr_dwell_ms.value())

    def _traverse_send_current(self) -> bool:
        assert self._ser is not None
        sid = self._traverse_cur_id
        kind = self.combo_tr_kind.currentData()
        if kind == "wheel":
            spd = self.spin_wheel.value()
            pkt = build_write_packet(sid, SCS_ADDR_GOAL_SPEED, i16_le(spd))
        else:
            body = (
                u16_le(self.spin_pos.value())
                + u16_le(self.spin_time.value())
                + u16_le(self.spin_gspd.value())
            )
            pkt = build_write_packet(sid, SCS_ADDR_GOAL_BLOCK, body)
        return self._write_only(pkt, note=f"遍历 ID={sid}")

    def _on_traverse_tick(self) -> None:
        if not self._traverse_active or self._ser is None or not self._ser.is_open:
            self._traverse_reset_state_only()
            return
        id_max = self.spin_tr_max.value()
        kind = self.combo_tr_kind.currentData()
        if kind == "wheel":
            try:
                pkt = build_write_packet(self._traverse_cur_id, SCS_ADDR_GOAL_SPEED, i16_le(0))
                if not self._write_only(pkt, note=f"遍历切换：停 ID={self._traverse_cur_id}"):
                    return
            except ValueError:
                pass
        self._traverse_cur_id += 1
        if self._traverse_cur_id > id_max:
            self._traverse_active = False
            self.btn_tr_stop.setEnabled(False)
            self.btn_tr_start.setEnabled(self.chk_traverse_enable.isChecked())
            self._update_traverse_controls_enabled()
            self._set_status(f"遍历完成（累计已发字节 {self._sent_bytes}）")
            return
        if not self._traverse_send_current():
            return
        self._traverse_timer.start(self.spin_tr_dwell_ms.value())

    def _require_serial(self) -> bool:
        if self._ser is None or not self._ser.is_open:
            QMessageBox.warning(self, "提示", "请先打开串口。")
            return False
        return True

    def _write_only(self, data: bytes, note: str = "") -> bool:
        """写入串口；失败时捕获 SerialException/OSError，避免界面崩溃。返回是否成功。"""
        if self._ser is None or not self._ser.is_open:
            self._set_status("串口未打开，无法发送")
            return False
        try:
            n = self._ser.write(data)
            self._ser.flush()
        except (serial.SerialException, OSError) as e:
            self._handle_serial_write_failed(e)
            return False
        self._sent_bytes += n
        hex_preview = data.hex(" ").upper()
        msg = f"已发送 {n} 字节（累计 {self._sent_bytes}）：{hex_preview}"
        if note:
            msg = f"{note} | {msg}"
        self._set_status(msg)
        return True

    def _set_status(self, text: str) -> None:
        self.status.setText(text)

    def send_wheel_speed(self) -> None:
        if not self._require_serial():
            return
        sid = self.spin_id.value()
        spd = self.spin_wheel.value()
        pkt = build_write_packet(sid, SCS_ADDR_GOAL_SPEED, i16_le(spd))
        self._write_only(pkt)

    def send_goal_block(self) -> None:
        if not self._require_serial():
            return
        sid = self.spin_id.value()
        body = u16_le(self.spin_pos.value()) + u16_le(self.spin_time.value()) + u16_le(
            self.spin_gspd.value()
        )
        pkt = build_write_packet(sid, SCS_ADDR_GOAL_BLOCK, body)
        self._write_only(pkt)

    def send_stop_wheel(self) -> None:
        self.spin_wheel.setValue(0)
        if not self._require_serial():
            return
        sid = self.spin_id.value()
        pkt = build_write_packet(sid, SCS_ADDR_GOAL_SPEED, i16_le(0))
        self._write_only(pkt)

    def send_raw_hex(self) -> None:
        if not self._require_serial():
            return
        text = self.edit_hex.text().strip()
        if not text:
            return
        try:
            parts = text.replace(",", " ").split()
            data = bytes(int(x, 16) for x in parts)
        except ValueError:
            QMessageBox.warning(self, "格式错误", "请输入空格分隔的十六进制字节，例如：FF FF 01 02 01 FB")
            return
        self._write_only(data)


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MainWindow()
    w.resize(520, 560)
    w.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
