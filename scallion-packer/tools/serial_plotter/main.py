#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 UART1 — HX711 重量曲线（仅重量，无 MPU）。
串口 pyserial，默认 115200。

固件 bsp_hx711.c 格式（已去皮）：
  [HX711] Weight=... kg (... g)  raw=... d=...  SCS:...

导出 CSV 含 raw、d（与串口一致），独占串口：「一键校准」发 K+克数（固件写 RAM 比例）；标定助手可生成头文件宏；两点对比查零漂。

安装：pip install -r requirements.txt
运行：python main.py
"""

from __future__ import annotations

import csv
import re
import sys
import time
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np
import pyqtgraph as pg
import serial
from serial.tools import list_ports
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

LINE_RE = re.compile(
    r"\[HX711\]\s+Weight=([+-]?\d*\.?\d+)\s+kg\s+\(([+-]?\d*\.?\d+)\s+g\)\s+"
    r"raw=(-?\d+)\s+d=(-?\d+)"
)


def _median_int(vals: list[int]) -> float:
    if not vals:
        return float("nan")
    s = sorted(vals)
    n = len(s)
    m = n // 2
    if n % 2:
        return float(s[m])
    return (s[m - 1] + s[m]) / 2.0


class SerialPlotterWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("STM32 UART1 — HX711 重量")
        self.resize(920, 700)

        self._ser: Optional[serial.Serial] = None
        self._rx_buf = bytearray()
        self._t0: Optional[float] = None

        self._max_points = 600
        self._t: Deque[float] = deque(maxlen=self._max_points)
        self._w_kg: Deque[float] = deque(maxlen=self._max_points)
        self._raw: Deque[int] = deque(maxlen=self._max_points)
        self._d_raw: Deque[int] = deque(maxlen=self._max_points)

        self._lines_ok = 0
        self._lines_skip = 0

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        grp = QGroupBox("串口（板子 UART1 / USB 转串口）")
        form = QFormLayout(grp)
        row_port = QHBoxLayout()
        self._cb_port = QComboBox()
        self._btn_refresh = QPushButton("刷新端口")
        self._btn_refresh.clicked.connect(self._refresh_ports)
        row_port.addWidget(self._cb_port, stretch=1)
        row_port.addWidget(self._btn_refresh)
        port_w = QWidget()
        port_w.setLayout(row_port)
        form.addRow("端口", port_w)

        self._cb_baud = QComboBox()
        for b in (9600, 19200, 38400, 57600, 115200):
            self._cb_baud.addItem(str(b), b)
        _i115 = self._cb_baud.findData(115200)
        self._cb_baud.setCurrentIndex(_i115 if _i115 >= 0 else 0)
        form.addRow("波特率", self._cb_baud)

        row_btn = QHBoxLayout()
        self._btn_connect = QPushButton("连接")
        self._btn_connect.clicked.connect(self._toggle_connect)
        self._lbl_status = QLabel("未连接")
        self._lbl_status.setStyleSheet("color: gray;")
        row_btn.addWidget(self._btn_connect)
        row_btn.addWidget(self._lbl_status, stretch=1)
        btn_w = QWidget()
        btn_w.setLayout(row_btn)
        form.addRow("", btn_w)

        spin_row = QHBoxLayout()
        self._spin_max = QSpinBox()
        self._spin_max.setRange(100, 10000)
        self._spin_max.setValue(self._max_points)
        self._spin_max.setSuffix(" 点")
        self._spin_max.valueChanged.connect(self._on_max_points_changed)
        spin_row.addWidget(QLabel("曲线保留点数"))
        spin_row.addWidget(self._spin_max)
        spin_row.addStretch()
        spin_w = QWidget()
        spin_w.setLayout(spin_row)
        form.addRow("", spin_w)

        row_cmd = QHBoxLayout()
        row_cmd.addWidget(QLabel("固件命令（与曲线同口）"))
        self._btn_cmd_t = QPushButton("T 去皮")
        self._btn_cmd_t.setToolTip("空载稳定后发送，等同串口助手敲 T")
        self._btn_cmd_t.clicked.connect(lambda: self._send_hx711_byte(b"T"))
        row_cmd.addWidget(self._btn_cmd_t)
        self._btn_cmd_r = QPushButton("R 清皮")
        self._btn_cmd_r.setToolTip("取消去皮状态")
        self._btn_cmd_r.clicked.connect(lambda: self._send_hx711_byte(b"R"))
        row_cmd.addWidget(self._btn_cmd_r)
        self._btn_cmd_c = QPushButton("C 配置")
        self._btn_cmd_c.setToolTip("打印 REF、标定 RATIO 等（固件 uart输出）")
        self._btn_cmd_c.clicked.connect(lambda: self._send_hx711_byte(b"C"))
        row_cmd.addWidget(self._btn_cmd_c)
        row_cmd.addSpacing(12)
        row_cmd.addWidget(QLabel("标准砝码(g)"))
        self._spin_std_cal_g = QDoubleSpinBox()
        self._spin_std_cal_g.setRange(0.1, 500000.0)
        self._spin_std_cal_g.setDecimals(2)
        self._spin_std_cal_g.setValue(682.0)
        self._spin_std_cal_g.setToolTip("一键校准时台上的标准物质量（克）")
        row_cmd.addWidget(self._spin_std_cal_g)
        self._btn_one_click_cal = QPushButton("一键校准")
        self._btn_one_click_cal.setToolTip(
            "前提：已去皮，台上仅有该砝码且示值已稳定。\n"
            "发送 K<克数>\\r\\n，固件主循环采样 d 并写入 RAM 内 RATIO（掉电恢复编译值；R 清皮并恢复编译比例）。"
        )
        self._btn_one_click_cal.clicked.connect(self._send_one_click_cal)
        row_cmd.addWidget(self._btn_one_click_cal)
        row_cmd.addStretch()
        cmd_w = QWidget()
        cmd_w.setLayout(row_cmd)
        form.addRow("", cmd_w)

        root.addWidget(grp)

        pg.setConfigOptions(antialias=True)
        self._plot_weight = pg.PlotWidget(title="重量 Weight (kg)")
        self._plot_weight.showGrid(x=True, y=True, alpha=0.3)
        self._plot_weight.setLabel("bottom", "时间", units="s")
        self._curve_w = self._plot_weight.plot(pen=pg.mkPen("#00bcd4", width=2), name="kg")

        row_follow = QHBoxLayout()
        self._chk_follow_x = QCheckBox("新数据超出右侧时自动跟随 X")
        self._chk_follow_x.setChecked(True)
        row_follow.addWidget(self._chk_follow_x)
        row_follow.addStretch()
        follow_w = QWidget()
        follow_w.setLayout(row_follow)
        root.addWidget(follow_w)

        root.addWidget(self._plot_weight, stretch=1)

        grp_cal = QGroupBox("标定助手（独占串口：框选稳定段 → 填时间或用「X轴范围」）")
        cal_root = QVBoxLayout(grp_cal)
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("标准质量(克)"))
        self._spin_cal_mass = QDoubleSpinBox()
        self._spin_cal_mass.setRange(0.1, 500000.0)
        self._spin_cal_mass.setDecimals(2)
        self._spin_cal_mass.setValue(410.0)
        row1.addWidget(self._spin_cal_mass)
        row1.addWidget(QLabel("t0(s)"))
        self._spin_cal_t0 = QDoubleSpinBox()
        self._spin_cal_t0.setRange(0.0, 1.0e9)
        self._spin_cal_t0.setDecimals(3)
        self._spin_cal_t0.setValue(0.0)
        row1.addWidget(self._spin_cal_t0)
        row1.addWidget(QLabel("t1(s)"))
        self._spin_cal_t1 = QDoubleSpinBox()
        self._spin_cal_t1.setRange(0.0, 1.0e9)
        self._spin_cal_t1.setDecimals(3)
        self._spin_cal_t1.setValue(10.0)
        row1.addWidget(self._spin_cal_t1)
        self._btn_cal_xrange = QPushButton("从 X 轴可见范围填入")
        self._btn_cal_xrange.setToolTip("在重量图上拖选/缩放后点此，自动填入 t0、t1")
        self._btn_cal_xrange.clicked.connect(self._calib_fill_from_xrange)
        row1.addWidget(self._btn_cal_xrange)
        self._btn_cal_gen = QPushButton("生成单段宏")
        self._btn_cal_gen.clicked.connect(self._calib_generate_single)
        row1.addWidget(self._btn_cal_gen)
        cal_root.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("两点：m1(g)"))
        self._spin_cal_m1 = QDoubleSpinBox()
        self._spin_cal_m1.setRange(0.1, 500000.0)
        self._spin_cal_m1.setDecimals(2)
        self._spin_cal_m1.setValue(410.0)
        row2.addWidget(self._spin_cal_m1)
        row2.addWidget(QLabel("t1a"))
        self._spin_cal_t1a = QDoubleSpinBox()
        self._spin_cal_t1a.setRange(0.0, 1.0e9)
        self._spin_cal_t1a.setDecimals(3)
        row2.addWidget(self._spin_cal_t1a)
        row2.addWidget(QLabel("t1b"))
        self._spin_cal_t1b = QDoubleSpinBox()
        self._spin_cal_t1b.setRange(0.0, 1.0e9)
        self._spin_cal_t1b.setDecimals(3)
        row2.addWidget(self._spin_cal_t1b)
        row2.addWidget(QLabel("m2(g)"))
        self._spin_cal_m2 = QDoubleSpinBox()
        self._spin_cal_m2.setRange(0.1, 500000.0)
        self._spin_cal_m2.setDecimals(2)
        self._spin_cal_m2.setValue(682.0)
        row2.addWidget(self._spin_cal_m2)
        row2.addWidget(QLabel("t2a"))
        self._spin_cal_t2a = QDoubleSpinBox()
        self._spin_cal_t2a.setRange(0.0, 1.0e9)
        self._spin_cal_t2a.setDecimals(3)
        row2.addWidget(self._spin_cal_t2a)
        row2.addWidget(QLabel("t2b"))
        self._spin_cal_t2b = QDoubleSpinBox()
        self._spin_cal_t2b.setRange(0.0, 1.0e9)
        self._spin_cal_t2b.setDecimals(3)
        row2.addWidget(self._spin_cal_t2b)
        self._btn_cal_2 = QPushButton("两点检查")
        self._btn_cal_2.setToolTip("检查 m2>m1 时 d 是否更大；给出建议宏（优先较重段）")
        self._btn_cal_2.clicked.connect(self._calib_two_point)
        row2.addWidget(self._btn_cal_2)
        cal_root.addLayout(row2)

        row3 = QHBoxLayout()
        self._txt_cal = QTextEdit()
        self._txt_cal.setReadOnly(True)
        self._txt_cal.setFont(QFont("Consolas", 9))
        self._txt_cal.setMaximumHeight(140)
        self._txt_cal.setPlaceholderText("单段/两点结果与可复制宏将显示在此…")
        row3.addWidget(self._txt_cal, stretch=1)
        self._btn_cal_copy = QPushButton("复制全文")
        self._btn_cal_copy.clicked.connect(self._calib_copy)
        row3.addWidget(self._btn_cal_copy)
        cal_root.addLayout(row3)
        root.addWidget(grp_cal)

        row_stats = QHBoxLayout()
        self._btn_export = QPushButton("导出 CSV…")
        self._btn_export.setToolTip(
            "导出当前缓冲：t_s, w_kg, w_g, raw, d（与固件串口一致，便于独占串口时标定）"
        )
        self._btn_export.clicked.connect(self._export_csv)
        self._lbl_stats = QLabel("已解析: 0 行 | 跳过: 0 行")
        self._lbl_stats.setStyleSheet("color: #666;")
        row_stats.addWidget(self._btn_export)
        row_stats.addWidget(self._lbl_stats, stretch=1)
        stats_w = QWidget()
        stats_w.setLayout(row_stats)
        root.addWidget(stats_w)

        self._refresh_ports()

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(5)
        self._poll_timer.timeout.connect(self._poll_serial)

        self._redraw_timer = QTimer(self)
        self._redraw_timer.setInterval(33)
        self._redraw_timer.timeout.connect(self._redraw_plots)
        self._redraw_timer.start()

        self._dirty = False

    def _on_max_points_changed(self, v: int) -> None:
        self._max_points = int(v)
        self._t = deque(self._t, maxlen=self._max_points)
        self._w_kg = deque(self._w_kg, maxlen=self._max_points)
        self._raw = deque(self._raw, maxlen=self._max_points)
        self._d_raw = deque(self._d_raw, maxlen=self._max_points)

    def _refresh_ports(self) -> None:
        cur = self._cb_port.currentData()
        self._cb_port.clear()
        for p in list_ports.comports():
            label = f"{p.device} — {p.description or 'serial'}"
            self._cb_port.addItem(label, p.device)
        if cur is not None:
            idx = self._cb_port.findData(cur)
            if idx >= 0:
                self._cb_port.setCurrentIndex(idx)
        if self._cb_port.count() == 0:
            self._cb_port.addItem("(无串口设备)", "")

    def _disconnect_serial(self) -> None:
        self._poll_timer.stop()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self._btn_connect.setText("连接")
        self._lbl_status.setText("已断开")
        self._lbl_status.setStyleSheet("color: gray;")
        self._t0 = None

    def _toggle_connect(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self._disconnect_serial()
            return

        port = self._cb_port.currentData()
        if not port:
            QMessageBox.warning(self, "提示", "请选择有效串口")
            return

        baud = int(self._cb_baud.currentData())
        try:
            self._ser = serial.Serial(
                port=str(port),
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
            )
        except serial.SerialException as e:
            QMessageBox.critical(self, "错误", f"无法打开 {port}：{e}")
            self._ser = None
            return

        self._rx_buf.clear()
        self._t.clear()
        self._w_kg.clear()
        self._raw.clear()
        self._d_raw.clear()
        self._lines_ok = 0
        self._lines_skip = 0
        self._t0 = time.perf_counter()
        self._btn_connect.setText("断开")
        self._lbl_status.setText(f"已连接 {port} @ {baud}")
        self._lbl_status.setStyleSheet("color: green;")
        self._poll_timer.start()

    def _send_hx711_byte(self, data: bytes) -> None:
        if self._ser is None or not self._ser.is_open:
            QMessageBox.warning(self, "提示", "请先连接串口再发送命令。")
            return
        if len(data) != 1:
            return
        try:
            self._ser.write(data)
            self._ser.flush()
        except serial.SerialException as e:
            QMessageBox.critical(self, "发送失败", str(e))

    def _send_hx711_bytes(self, data: bytes) -> None:
        if self._ser is None or not self._ser.is_open:
            QMessageBox.warning(self, "提示", "请先连接串口再发送命令。")
            return
        if not data:
            return
        try:
            self._ser.write(data)
            self._ser.flush()
        except serial.SerialException as e:
            QMessageBox.critical(self, "发送失败", str(e))

    def _send_one_click_cal(self) -> None:
        m = float(self._spin_std_cal_g.value())
        if m <= 0:
            return
        # 与固件 sscanf("%f") 一致，避免多余小数0
        line = f"K{m:g}\r\n".encode("ascii", errors="strict")
        self._send_hx711_bytes(line)

    def _poll_serial(self) -> None:
        if self._ser is None or not self._ser.is_open:
            return
        try:
            n = self._ser.in_waiting
            if n > 0:
                self._rx_buf.extend(self._ser.read(n))
        except serial.SerialException as e:
            self._lbl_status.setText(f"串口错误: {e}")
            self._disconnect_serial()
            return

        while True:
            nl = self._rx_buf.find(b"\n")
            if nl < 0:
                break
            line_b = bytes(self._rx_buf[:nl])
            del self._rx_buf[: nl + 1]
            line = line_b.decode("utf-8", errors="replace").strip()
            self._parse_line(line)

    def _export_csv(self) -> None:
        n = len(self._t)
        if n == 0:
            QMessageBox.information(
                self,
                "导出",
                "当前无缓冲数据。请先连接、去皮，并确认固件输出含 Weight=与 raw= d= 的行。",
            )
            return
        default_name = time.strftime("hx711_weight_%Y%m%d_%H%M%S.csv")
        path, _ = QFileDialog.getSaveFileName(
            self,
            "导出重量曲线",
            default_name,
            "CSV (*.csv);;所有文件 (*)",
        )
        if not path:
            return
        try:
            rows: list[Tuple[float, float, int, int]] = sorted(
                zip(
                    list(self._t),
                    list(self._w_kg),
                    list(self._raw),
                    list(self._d_raw),
                ),
                key=lambda r: r[0],
            )
            with open(path, "w", newline="", encoding="utf-8-sig") as fp:
                fp.write(
                    "# HX711标定(bsp_hx711.h): d=去皮后diff; "
                    "KNOWN_MASS_G=砝码克数, KNOWN_MASS_RAW_DIFF=仅放该砝码时本表d稳定值(含符号)\n"
                )
                cw = csv.writer(fp)
                cw.writerow(["t_s", "w_kg", "w_g", "raw", "d"])
                for t_i, kg_i, raw_i, d_i in rows:
                    cw.writerow(
                        [
                            f"{t_i:.6f}",
                            f"{kg_i:.6f}",
                            f"{kg_i * 1000.0:.3f}",
                            str(raw_i),
                            str(d_i),
                        ]
                    )
        except OSError as e:
            QMessageBox.critical(self, "导出失败", str(e))
            return
        QMessageBox.information(
            self,
            "导出成功",
            f"已写入 {len(rows)} 行（含 raw、d，见文件首行标定说明）：\n{path}",
        )

    def _parse_line(self, line: str) -> None:
        m = LINE_RE.search(line)
        if not m:
            self._lines_skip += 1
            return
        w_kg, _w_g, raw_s, d_s = m.groups()
        try:
            fk = float(w_kg)
            raw_i = int(raw_s, 10)
            d_i = int(d_s, 10)
        except ValueError:
            self._lines_skip += 1
            return

        if self._t0 is None:
            self._t0 = time.perf_counter()
        t = time.perf_counter() - self._t0

        self._t.append(t)
        self._w_kg.append(fk)
        self._raw.append(raw_i)
        self._d_raw.append(d_i)
        self._lines_ok += 1
        self._dirty = True

    def _redraw_plots(self) -> None:
        if not self._dirty and len(self._t) == 0:
            return
        self._dirty = False

        self._lbl_stats.setText(
            f"已解析: {self._lines_ok} 行 | 跳过: {self._lines_skip} 行 | 缓冲点数: {len(self._t)}"
        )

        if len(self._t) == 0:
            return

        ta = np.asarray(self._t, dtype=np.float64)
        self._curve_w.setData(ta, np.asarray(self._w_kg, dtype=np.float64))

        self._follow_x_if_needed(float(ta[-1]))

    def _follow_x_if_needed(self, last_t: float) -> None:
        if not self._chk_follow_x.isChecked():
            return
        vb = self._plot_weight.getViewBox()
        (xmin_vis, xmax_vis), _ = vb.viewRange()
        width = xmax_vis - xmin_vis
        if width <= 1e-12:
            return
        if last_t > xmax_vis:
            vb.setXRange(last_t - width, last_t, padding=0)

    def _calib_ds_in_interval(self, t0: float, t1: float) -> list[int]:
        if t0 > t1:
            t0, t1 = t1, t0
        return [d for t, d in zip(self._t, self._d_raw) if t0 <= t <= t1]

    def _calib_fill_from_xrange(self) -> None:
        vb = self._plot_weight.getViewBox()
        (xmin, xmax), _ = vb.viewRange()
        if xmax < xmin:
            xmin, xmax = xmax, xmin
        self._spin_cal_t0.setValue(float(xmin))
        self._spin_cal_t1.setValue(float(xmax))

    def _calib_macro_lines(self, mass_g: float, d_med: float) -> str:
        d_i = int(round(d_med))
        return (
            f"#define HX711_KNOWN_MASS_G           ({mass_g:.2f}f)\n"
            f"#define HX711_KNOWN_MASS_RAW_DIFF    ({d_i}L)\n"
        )

    def _calib_generate_single(self) -> None:
        if len(self._t) == 0:
            QMessageBox.information(self, "标定", "当前无缓冲数据，请先连接并采集。")
            return
        t0 = float(self._spin_cal_t0.value())
        t1 = float(self._spin_cal_t1.value())
        mass = float(self._spin_cal_mass.value())
        ds = self._calib_ds_in_interval(t0, t1)
        if not ds:
            self._txt_cal.setPlainText(
                f"区间 [{t0:.3f}, {t1:.3f}] s 内无采样点。请放大/拖选稳定段后「从 X 轴可见范围填入」。"
            )
            return
        d_med = _median_int(ds)
        macro = self._calib_macro_lines(mass, d_med)
        lines = [
            f"单段标定：区间 [{min(t0, t1):.3f}, {max(t0, t1):.3f}] s，点数 n={len(ds)}，d 中位数={d_med:.1f}",
            f"（固件用整数 d，建议 KNOWN_MASS_RAW_DIFF = {int(round(d_med))}）",
            "",
            macro.rstrip(),
        ]
        self._txt_cal.setPlainText("\n".join(lines))

    def _calib_two_point(self) -> None:
        if len(self._t) == 0:
            QMessageBox.information(self, "标定", "当前无缓冲数据，请先连接并采集。")
            return
        m1 = float(self._spin_cal_m1.value())
        m2 = float(self._spin_cal_m2.value())
        t1a, t1b = float(self._spin_cal_t1a.value()), float(self._spin_cal_t1b.value())
        t2a, t2b = float(self._spin_cal_t2a.value()), float(self._spin_cal_t2b.value())
        d1s = self._calib_ds_in_interval(t1a, t1b)
        d2s = self._calib_ds_in_interval(t2a, t2b)
        if not d1s or not d2s:
            self._txt_cal.setPlainText(
                "某一段区间内无采样点。请检查 t1a–t1b、t2a–t2b 是否落在当前缓冲时间范围内。"
            )
            return
        d1 = _median_int(d1s)
        d2 = _median_int(d2s)
        dm = m2 - m1
        dd = d2 - d1
        lines: list[str] = [
            f"段1：m1={m1:.2f} g，区间 [{min(t1a,t1b):.3f},{max(t1a,t1b):.3f}] s，n={len(d1s)}，d 中位={d1:.1f}",
            f"段2：m2={m2:.2f} g，区间 [{min(t2a,t2b):.3f},{max(t2a,t2b):.3f}] s，n={len(d2s)}，d 中位={d2:.1f}",
            f"Δm={dm:.2f} g，Δd={dd:.1f}",
        ]
        if abs(dd) < 1e-6:
            lines.append("警告：两段 d 几乎相同，无法估斜率；请拉长间隔或检查是否同重。")
        else:
            slope_g_per_unit = dm / dd
            lines.append(f"近似斜率：{slope_g_per_unit:.6f} g / raw（仅作一致性参考）")
        warn = []
        if m2 > m1 and d2 <= d1:
            warn.append("m2>m1 但 d2≤d1：两段可能不可比（零漂、冲击、或去皮基准不同）；优先看 CSV/单段。")
        elif m1 > m2 and d1 <= d2:
            warn.append("m1>m2 但 d1≤d2：同上，注意比较顺序与去皮。")
        if warn:
            lines.append("")
            lines.extend(warn)
        use_m, use_d = (m2, d2) if m2 >= m1 else (m1, d1)
        lines.append("")
        lines.append("建议写入 bsp_hx711.h（取较重或更可信的一段；若两点一致可任选）：")
        lines.append(self._calib_macro_lines(use_m, use_d).rstrip())
        self._txt_cal.setPlainText("\n".join(lines))

    def _calib_copy(self) -> None:
        txt = self._txt_cal.toPlainText().strip()
        if not txt:
            return
        QApplication.clipboard().setText(txt)

    def closeEvent(self, event) -> None:
        self._disconnect_serial()
        super().closeEvent(event)


def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = SerialPlotterWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
