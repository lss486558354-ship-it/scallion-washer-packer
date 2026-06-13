#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Jetson Orin Nano 与 STM32F103VE 通信协议
大葱切割-清洗-打包一体机

功能说明：
  - 与STM32串口通信，发送控制指令
  - 接收STM32传感器数据
  - 视觉识别后的位置信息发送给STM32
  - 运行YOLOv8视觉模型进行目标检测

硬件连接：
  Jetson Orin Nano UART (TX/RX) <-> STM32 USART2 (PA2/PA3)
  波特率: 115200
  电平: Jetson UART为3.3V TTL，与STM32可直接连接（STM32也是3.3V容忍）

Jetson Orin Nano 串口引脚（40Pin扩展口）：
  - Pin 8  (GPIO14/UART1_TX) -> STM32 USART2_RX (PA3)
  - Pin 10 (GPIO15/UART1_RX) -> STM32 USART2_TX (PA2)
  - Pin 6  (GND)             -> STM32 GND

  注意：使用前需确保GPIO权限正确，或使用USB转串口模块

作者：机械创新比赛电控组
版本：V2.0（Jetson Orin Nano版）
日期：2026-03-23
"""

import serial
import struct
import threading
import time
import logging
import os
import glob
from enum import Enum
from typing import Optional, Callable

# =============================================================================
// 配置参数
// =============================================================================
# Jetson Orin Nano 串口配置
# 优先使用USB转串口，否则使用40Pin扩展口UART
def find_serial_port() -> str:
    """
    自动查找可用的串口设备
    Jetson Orin Nano上按优先级查找：
    1. /dev/ttyUSB* (USB转串口模块)
    2. /dev/ttyTHS0 (40Pin扩展口UART)
    3. /dev/ttyTHS1 (备用UART)
    """
    # 首先查找USB转串口
    usb_ports = glob.glob('/dev/ttyUSB*')
    if usb_ports:
        # 按设备号排序，优先使用ttyUSB0
        usb_ports.sort()
        logging.info(f"发现USB转串口设备: {usb_ports[0]}")
        return usb_ports[0]
    
    # 使用40Pin扩展口UART
    if os.path.exists('/dev/ttyTHS0'):
        logging.info("使用40Pin扩展口UART: /dev/ttyTHS0")
        return '/dev/ttyTHS0'
    
    # 尝试其他UART
    for uart in ['/dev/ttyTHS1', '/dev/ttyTHS2', '/dev/ttyS0']:
        if os.path.exists(uart):
            logging.info(f"发现UART设备: {uart}")
            return uart
    
    # 默认使用ttyTHS0
    return '/dev/ttyTHS0'

SERIAL_PORT = find_serial_port()      # Jetson UART（自动检测）
BAUDRATE = 115200
TIMEOUT = 1.0                   # 串口超时时间(秒)

# 协议帧定义
FRAME_HEAD = 0xAA
FRAME_TAIL = 0x55

# 命令类型（与STM32协议对应）
class CMD(Enum):
    # STM32 -> Jetson
    SENSOR_DATA     = 0x01  # 传感器数据
    MOTOR_STATUS    = 0x02  # 电机状态
    LIMIT_SWITCH    = 0x03  # 限位开关状态
    SYSTEM_STATUS   = 0x04  # 系统状态
    TASK_COMPLETE   = 0x05  # 任务完成
    ERROR_REPORT    = 0x06  # 错误报告
    
    # Jetson -> STM32
    START_TASK      = 0x10  # 开始任务
    STOP_TASK       = 0x11  # 停止任务
    PAUSE_TASK      = 0x12  # 暂停任务
    RESUME_TASK     = 0x13  # 恢复任务
    SET_MOTOR_SPEED = 0x14  # 设置电机速度
    SET_MOTOR_POS   = 0x15  # 设置电机位置
    SET_SERVO_ANGLE = 0x16  # 设置舵机角度
    RESET_SYSTEM    = 0x17  # 复位系统
    CONFIG_PARAMS   = 0x18  # 配置参数

# 系统状态
class SYSTEM_STATE(Enum):
    IDLE       = 0x00  # 空闲
    READY      = 0x01  # 就绪
    RUNNING    = 0x02  # 运行中
    PAUSED     = 0x03  # 暂停
    ERROR      = 0x04  # 错误
    EMERGENCY  = 0x05  # 急停

# =============================================================================
// 协议类
// =============================================================================
class Protocol:
    """通信协议处理类"""
    
    @staticmethod
    def calculate_checksum(cmd: int, data: bytes) -> int:
        """
        计算校验和（异或校验）
        """
        checksum = cmd ^ len(data)
        for byte in data:
            checksum ^= byte
        return checksum
    
    @staticmethod
    def build_frame(cmd: int, data: bytes) -> bytes:
        """
        构建协议帧
        帧格式: [帧头0xAA] [命令] [长度] [数据...] [校验和]
        """
        checksum = Protocol.calculate_checksum(cmd, data)
        frame = bytes([FRAME_HEAD, cmd, len(data)]) + data + bytes([checksum])
        return frame
    
    @staticmethod
    def parse_frame(data: bytes) -> Optional[tuple]:
        """
        解析协议帧
        返回: (cmd, data) 或 None（解析失败）
        """
        if len(data) < 4:  # 最小帧: 帧头+命令+长度+校验
            return None
        
        if data[0] != FRAME_HEAD:
            return None
        
        cmd = data[1]
        length = data[2]
        
        if len(data) < 4 + length:
            return None
        
        payload = data[3:3+length]
        received_checksum = data[3+length]
        
        # 验证校验和
        calculated_checksum = Protocol.calculate_checksum(cmd, payload)
        if calculated_checksum != received_checksum:
            return None
        
        return (cmd, payload)

# =============================================================================
// STM32通信类
// =============================================================================
class STM32Communicator:
    """STM32通信管理类"""
    
    def __init__(self, port: str = None, baudrate: int = BAUDRATE):
        """
        初始化STM32通信器
        
        Args:
            port: 串口设备路径，None时自动检测
            baudrate: 波特率，默认115200
        """
        self.port = port if port else find_serial_port()
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.is_connected = False
        
        # 数据回调函数
        self.sensor_callback: Optional[Callable] = None
        self.system_status_callback: Optional[Callable] = None
        self.error_callback: Optional[Callable] = None
        self.task_complete_callback: Optional[Callable] = None
        self.limit_switch_callback: Optional[Callable] = None
        
        # 接收线程
        self.rx_thread: Optional[threading.Thread] = None
        self.running = False
        
        # 日志
        self.logger = logging.getLogger('STM32Comm')
        
        # 传感器数据缓存
        self.sensor_data = {
            'motor1_position': 0,
            'motor2_position': 0,
            'motor3_position': 0,
            'motor1_speed': 0,
            'motor2_speed': 0,
            'motor3_speed': 0,
            'servo1_angle': 0.0,
            'servo2_angle': 0.0,
            'adc_value': 0
        }
        
        # 系统状态
        self.system_status = SYSTEM_STATE.IDLE
        self.error_code = 0
    
    def connect(self) -> bool:
        """
        连接STM32串口
        """
        try:
            # 确保串口存在
            if not os.path.exists(self.port):
                self.logger.warning(f"串口 {self.port} 不存在，尝试自动检测...")
                self.port = find_serial_port()
            
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=TIMEOUT
            )
            self.is_connected = True
            self.logger.info(f"Jetson串口 {self.port} 连接成功")
            
            # 启动接收线程
            self.running = True
            self.rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.rx_thread.start()
            
            return True
            
        except serial.SerialException as e:
            self.logger.error(f"串口连接失败: {e}")
            self.logger.error(f"请检查：")
            self.logger.error(f"  1. STM32是否已连接")
            self.logger.error(f"  2. 串口权限是否足够 (运行: sudo chmod 666 {self.port})")
            self.logger.error(f"  3. 串口是否被其他程序占用")
            return False
    
    def disconnect(self):
        """
        断开串口连接
        """
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=2)
        
        if self.serial and self.serial.is_open:
            self.serial.close()
        
        self.is_connected = False
        self.logger.info("串口已断开")
    
    def _receive_loop(self):
        """
        接收数据线程
        """
        buffer = bytearray()
        
        while self.running:
            try:
                if self.serial and self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    buffer.extend(data)
                    
                    # 处理缓冲区数据
                    while len(buffer) >= 4:
                        # 查找帧头
                        frame_start = -1
                        for i in range(len(buffer)):
                            if buffer[i] == FRAME_HEAD:
                                frame_start = i
                                break
                        
                        if frame_start == -1:
                            buffer.clear()
                            break
                        
                        # 删除帧头前的无效数据
                        if frame_start > 0:
                            buffer = buffer[frame_start:]
                        
                        # 尝试解析帧
                        if len(buffer) >= 4:
                            length = buffer[2]
                            frame_len = 4 + length
                            
                            if len(buffer) >= frame_len:
                                frame_data = bytes(buffer[:frame_len])
                                result = Protocol.parse_frame(frame_data)
                                
                                if result:
                                    self._handle_frame(result[0], result[1])
                                    buffer = buffer[frame_len:]
                                else:
                                    # 校验失败，丢弃帧头继续寻找
                                    buffer = buffer[1:]
                            else:
                                # 数据不完整，等待更多数据
                                break
                                
            except Exception as e:
                self.logger.error(f"接收数据错误: {e}")
                time.sleep(0.1)
    
    def _handle_frame(self, cmd: int, data: bytes):
        """
        处理接收到的帧
        """
        try:
            if cmd == CMD.SENSOR_DATA.value:
                self._parse_sensor_data(data)
                if self.sensor_callback:
                    self.sensor_callback(self.sensor_data)
                    
            elif cmd == CMD.SYSTEM_STATUS.value:
                if len(data) >= 2:
                    self.system_status = SYSTEM_STATE(data[0])
                    self.error_code = data[1]
                    if self.system_status_callback:
                        self.system_status_callback(self.system_status, self.error_code)
                        
            elif cmd == CMD.ERROR_REPORT.value:
                if len(data) >= 1:
                    error_code = data[0]
                    self.logger.error(f"STM32错误报告: 0x{error_code:02X}")
                    if self.error_callback:
                        self.error_callback(error_code)
                        
            elif cmd == CMD.TASK_COMPLETE.value:
                self.logger.info("任务完成通知")
                if self.task_complete_callback:
                    self.task_complete_callback()
                    
            elif cmd == CMD.LIMIT_SWITCH.value:
                if len(data) >= 1:
                    limit_status = data[0]
                    self.logger.warning(f"限位开关状态: 0x{limit_status:02X}")
                    if self.limit_switch_callback:
                        self.limit_switch_callback(limit_status)
                        
        except Exception as e:
            self.logger.error(f"处理帧数据错误: {e}")
    
    def _parse_sensor_data(self, data: bytes):
        """
        解析传感器数据
        """
        if len(data) >= 13:
            self.sensor_data['motor1_position'] = (data[0] << 8) | data[1]
            self.sensor_data['motor2_position'] = (data[2] << 8) | data[3]
            self.sensor_data['motor3_position'] = (data[4] << 8) | data[5]
            self.sensor_data['motor1_speed'] = (data[6] << 8) | data[7]
            self.sensor_data['motor2_speed'] = (data[8] << 8) | data[9]
            self.sensor_data['motor3_speed'] = (data[10] << 8) | data[11]
            self.sensor_data['adc_value'] = data[12]
    
    # =========================================================================
    # 发送命令函数
    # =========================================================================
    
    def send_start_task(self):
        """发送开始任务命令"""
        frame = Protocol.build_frame(CMD.START_TASK.value, bytes([0]))
        self._send(frame)
        self.logger.info("发送: 开始任务")
    
    def send_stop_task(self):
        """发送停止任务命令"""
        frame = Protocol.build_frame(CMD.STOP_TASK.value, bytes([0]))
        self._send(frame)
        self.logger.info("发送: 停止任务")
    
    def send_pause_task(self):
        """发送暂停任务命令"""
        frame = Protocol.build_frame(CMD.PAUSE_TASK.value, bytes([0]))
        self._send(frame)
        self.logger.info("发送: 暂停任务")
    
    def send_resume_task(self):
        """发送恢复任务命令"""
        frame = Protocol.build_frame(CMD.RESUME_TASK.value, bytes([0]))
        self._send(frame)
        self.logger.info("发送: 恢复任务")
    
    def send_motor_speed(self, motor_id: int, speed: int):
        """
        发送电机速度设置命令
        
        Args:
            motor_id: 电机编号 (1-3)
            speed: 目标速度（有符号，负数为反转）
        """
        data = bytes([motor_id, (speed >> 8) & 0xFF, speed & 0xFF])
        frame = Protocol.build_frame(CMD.SET_MOTOR_SPEED.value, data)
        self._send(frame)
        self.logger.info(f"发送: 电机{motor_id}速度={speed}")
    
    def send_motor_position(self, motor_id: int, position: int):
        """
        发送电机位置设置命令
        
        Args:
            motor_id: 电机编号 (1-3)
            position: 目标位置（脉冲数）
        """
        data = bytes([motor_id, (position >> 8) & 0xFF, position & 0xFF, 1])  # 位置模式
        frame = Protocol.build_frame(CMD.SET_MOTOR_POS.value, data)
        self._send(frame)
        self.logger.info(f"发送: 电机{motor_id}位置={position}")
    
    def send_servo_angle(self, servo_id: int, angle: float):
        """
        发送舵机角度设置命令
        
        Args:
            servo_id: 舵机编号 (1-2)
            angle: 目标角度 (0-180度)
        """
        angle_int = int(angle * 10)  # 放大10倍传输
        data = bytes([servo_id, (angle_int >> 8) & 0xFF, angle_int & 0xFF])
        frame = Protocol.build_frame(CMD.SET_SERVO_ANGLE.value, data)
        self._send(frame)
        self.logger.info(f"发送: 舵机{servo_id}角度={angle}")
    
    def send_reset(self):
        """发送系统复位命令"""
        frame = Protocol.build_frame(CMD.RESET_SYSTEM.value, bytes([0]))
        self._send(frame)
        self.logger.info("发送: 系统复位")
    
    def _send(self, data: bytes):
        """
        发送数据
        """
        if self.serial and self.is_connected:
            try:
                self.serial.write(data)
            except Exception as e:
                self.logger.error(f"发送数据失败: {e}")

# =============================================================================
// 示例：视觉识别与大葱位置控制
// =============================================================================
class ScallionController:
    """
    大葱切割控制器
    结合YOLOv8视觉识别和STM32控制
    """
    
    def __init__(self):
        self.stm32 = STM32Communicator()
        self.target_positions = []  # 视觉识别到的目标位置
        self.yolo_model = None     # YOLOv8模型
    
    def load_yolo_model(self, model_path: str = 'yolov8n.pt'):
        """
        加载YOLOv8模型
        
        Args:
            model_path: 模型文件路径，默认使用yolov8n（nano版，轻量快速）
        
        Returns:
            bool: 加载是否成功
        """
        try:
            from ultralytics import YOLO
            self.logger.info(f"正在加载YOLOv8模型: {model_path}")
            self.yolo_model = YOLO(model_path)
            self.logger.info("YOLOv8模型加载成功")
            return True
        except ImportError:
            self.logger.error("未安装ultralytics库，请运行: pip install ultralytics")
            return False
        except Exception as e:
            self.logger.error(f"YOLOv8模型加载失败: {e}")
            return False
    
    def detect_scallion(self, frame):
        """
        使用YOLOv8检测大葱位置
        
        Args:
            frame: 摄像头帧数据（numpy数组）
        
        Returns:
            list: 检测到的目标列表，每个目标包含(x, y, width, height, confidence)
        """
        if self.yolo_model is None:
            self.logger.warning("YOLOv8模型未加载，跳过检测")
            return []
        
        try:
            results = self.yolo_model(frame, verbose=False)
            detections = []
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # 获取边界框信息
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].cpu().numpy()
                    cls = int(box.cls[0].cpu().numpy())
                    
                    # 计算中心点和尺寸
                    x = (x1 + x2) / 2
                    y = (y1 + y2) / 2
                    width = x2 - x1
                    height = y2 - y1
                    
                    detections.append({
                        'x': x,
                        'y': y,
                        'width': width,
                        'height': height,
                        'confidence': conf,
                        'class': cls
                    })
            
            return detections
            
        except Exception as e:
            self.logger.error(f"YOLOv8检测失败: {e}")
            return []
    
    def start(self):
        """启动控制器"""
        if not self.stm32.connect():
            print("无法连接到STM32")
            return False
        
        # 设置回调函数
        self.stm32.sensor_callback = self.on_sensor_data
        self.stm32.error_callback = self.on_error
        
        # 等待系统就绪
        time.sleep(0.5)
        
        # 发送复位命令
        self.stm32.send_reset()
        time.sleep(0.2)
        
        # 启动主循环
        self.main_loop()
        
        return True
    
    def main_loop(self):
        """主控制循环"""
        try:
            while True:
                # 1. 模拟视觉识别获取目标位置
                # 在实际应用中，这里应该调用视觉识别算法
                # target = self.detect_scallion(camera_frame)
                
                # 2. 如果识别到目标，发送位置指令
                # if target:
                #     self.stm32.send_motor_position(2, target.x)  # 传送带移动到目标位置
                #     self.stm32.send_servo_angle(1, target.cut_angle)  # 设置切割角度
                
                # 3. 等待一段时间后继续
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n用户中断")
        finally:
            self.stm32.disconnect()
    
    def on_sensor_data(self, data: dict):
        """传感器数据回调"""
        pass  # 可以在这里处理传感器数据
    
    def on_error(self, error_code: int):
        """错误回调"""
        print(f"STM32错误: 0x{error_code:02X}")
        
        # 根据错误代码处理
        if error_code == 0x10:  # 原点限位触发
            print("电机已回到原点")
        elif error_code >= 0x01 and error_code <= 0x03:
            print(f"电机{error_code}故障")
        elif error_code == 0x30:
            print("通信超时")
    
    # =========================================================================
    # 任务函数
    # =========================================================================
    
    def task_cut(self):
        """切割任务"""
        # 1. 启动传送带
        self.stm32.send_motor_speed(2, 1000)
        
        # 2. 等待到位
        time.sleep(2)
        
        # 3. 停止传送带
        self.stm32.send_motor_speed(2, 0)
        
        # 4. 切割
        self.stm32.send_motor_speed(1, 3000)
        time.sleep(0.5)
        self.stm32.send_motor_speed(1, 0)
    
    def task_wash(self):
        """清洗任务"""
        # 1. 启动水泵
        self.stm32.send_motor_speed(3, 2000)
        
        # 2. 设置清洗角度
        self.stm32.send_servo_angle(1, 45)
        
        # 3. 清洗
        time.sleep(3)
        
        # 4. 停止
        self.stm32.send_motor_speed(3, 0)
    
    def task_pack(self):
        """打包任务"""
        # 1. 夹紧
        self.stm32.send_servo_angle(2, 90)
        time.sleep(0.5)
        
        # 2. 封口
        self.stm32.send_servo_angle(3, 45)
        time.sleep(0.5)
        
        # 3. 释放
        self.stm32.send_servo_angle(2, 0)
        self.stm32.send_servo_angle(3, 0)

# =============================================================================
// Jetson GPIO 配置辅助函数
// =============================================================================
class JetsonGPIOHelper:
    """
    Jetson Orin Nano GPIO辅助类
    用于配置40Pin扩展口的GPIO功能
    """
    
    @staticmethod
    def check_uart_permissions(port: str = '/dev/ttyTHS0') -> bool:
        """
        检查并修复UART串口权限
        
        Args:
            port: 串口设备路径
        
        Returns:
            bool: 权限是否足够
        """
        try:
            # 检查当前用户是否有读写权限
            if os.access(port, os.R_OK | os.W_OK):
                return True
            
            # 尝试使用sudo临时修改权限
            os.system(f'sudo chmod 666 {port}')
            
            # 再次检查
            return os.access(port, os.R_OK | os.W_OK)
            
        except Exception as e:
            logging.error(f"权限检查失败: {e}")
            return False
    
    @staticmethod
    def list_available_uarts() -> list:
        """
        列出Jetson上所有可用的UART设备
        
        Returns:
            list: 可用UART设备列表
        """
        uarts = []
        
        # USB转串口
        for port in glob.glob('/dev/ttyUSB*'):
            uarts.append({'port': port, 'type': 'USB转串口'})
        
        # 40Pin扩展口UART
        for port in glob.glob('/dev/ttyTHS*'):
            uarts.append({'port': port, 'type': '40Pin扩展口'})
        
        # 标准串口
        for port in glob.glob('/dev/ttyS*'):
            uarts.append({'port': port, 'type': '标准串口'})
        
        return uarts

# =============================================================================
// 主程序入口
// =============================================================================
if __name__ == '__main__':
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 列出可用串口
    print("=" * 50)
    print("Jetson Orin Nano 串口配置")
    print("=" * 50)
    
    available_uarts = JetsonGPIOHelper.list_available_uarts()
    if available_uarts:
        print("发现以下UART设备:")
        for i, uart in enumerate(available_uarts, 1):
            print(f"  {i}. {uart['port']} ({uart['type']})")
    else:
        print("未发现UART设备")
    
    print()
    print(f"将使用的串口: {find_serial_port()}")
    print("=" * 50)
    print()
    
    # 创建并启动控制器
    controller = ScallionController()
    
    # 可选：加载YOLOv8模型
    # controller.load_yolo_model('yolov8n.pt')
    
    controller.start()
