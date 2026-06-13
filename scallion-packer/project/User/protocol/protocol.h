/**
 ******************************************************************************
 * STM32F103VET6 通信协议层
 *
 * ============================================================================
 * 功能：STM32与Jetson Orin Nano之间使用自定义的帧协议进行通信
 * 帧格式: [帧头0xAA] [命令] [长度] [数据N] [校验和]
 *
 * 注意：VET6版本支持所有6路步进电机
 * ============================================================================
 *
 * 使用示例：
 *   // 1. 初始化
 *   Protocol_Init();
 *
 *   // 2. 发送帧
 *   uint8_t data[3] = {0x01, 0x02, 0x03};
 *   Protocol_SendFrame(CHANNEL_USART2, CMD_SYSTEM_STATUS, data, 3);
 *
 *   // 3. 处理接收到的字节（在USART中断中调用）
 *   Protocol_CommChannel_HandleByte(USART2, byte);
 *
 * ============================================================================
 * 通信通道：
 *   CHANNEL_USART2 = 0  - Jetson通信串口
 *   CHANNEL_USART3 = 1  - 预留扩展串口
 *   CHANNEL_USART1 = 2  - 调试串口
 *
 * 命令类型：
 *   STM32 -> Jetson：
 *     CMD_SENSOR_DATA   = 0x01  - 传感器数据
 *     CMD_MOTOR_STATUS  = 0x02  - 电机状态
 *     CMD_LIMIT_SWITCH  = 0x03  - 限位状态
 *     CMD_SYSTEM_STATUS = 0x04  - 系统状态
 *     CMD_TASK_COMPLETE = 0x05  - 任务完成
 *     CMD_ERROR_REPORT  = 0x06  - 错误报告
 *
 *   Jetson -> STM32：
 *     CMD_START_TASK    = 0x10  - 开始任务
 *     CMD_STOP_TASK     = 0x11  - 停止任务
 *     CMD_PAUSE_TASK    = 0x12  - 暂停任务
 *     CMD_RESUME_TASK   = 0x13  - 恢复任务
 *     CMD_SET_MOTOR_SPEED = 0x14 - 设置电机速度
 *     CMD_SET_MOTOR_POS = 0x15  - 设置电机位置
 *     CMD_SET_SERVO_ANGLE = 0x16 - 设置舵机角度
 *     CMD_RESET_SYSTEM  = 0x17  - 系统复位
 *     CMD_ALL_STOP       = 0x2F  - 全部急停
 *     CMD_PROCESS_START  = 0x30  - 开始工艺流程
 *     CMD_PROCESS_STOP   = 0x31  - 停止工艺流程
 *
 * 错误类型：
 *   ERROR_LIMIT_ORIGIN   = 0x10  - 原点限位触发
 *   ERROR_LIMIT_FORWARD  = 0x11  - 正限位触发
 *   ERROR_LIMIT_BACKWARD = 0x12  - 负限位触发
 *   ERROR_SENSOR_TIMEOUT = 0x20  - 传感器超时
 *   ERROR_COMM_TIMEOUT   = 0x30  - 通信超时
 *
 * 帧格式：
 *   [帧头 0xAA] [命令 1字节] [长度 1字节] [数据 N字节] [校验和 1字节]
 *
 * ============================================================================
 * 作者：工程训练中心116电控组
 * 日期：2026-04-09（VET6版本）
 ******************************************************************************
 */

#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

/* VET6版本：已启用完整功能 */

#include "stm32f10x.h"
#include "process_control.h"
#include <string.h>

/* 通信通道
 *   CHANNEL_USART2 = 0  - Jetson通信串口
 *   CHANNEL_USART3 = 1  - 预留扩展串口
 *   CHANNEL_USART1 = 2  - 调试串口
 */
typedef enum {
    CHANNEL_USART2 = 0,
    CHANNEL_USART3 = 1,
    CHANNEL_USART1 = 2
} CommChannel_TypeDef;

/* 帧头和帧尾定义 */
#define FRAME_HEAD      0xAA
#define FRAME_TAIL      0x55
#define FRAME_MIN_SIZE  4

/* 最大数据长度 */
#define PROTOCOL_MAX_DATA_LEN  32

/* 通信缓冲区大小 */
#define COMM_TX_BUF_SIZE  256
#define COMM_RX_BUF_SIZE  256

/* ========== 命令类型 ========== */
typedef enum {
    /* STM32 -> Jetson */
    CMD_SENSOR_DATA   = 0x01,   /* 传感器数据 */
    CMD_MOTOR_STATUS  = 0x02,   /* 电机状态 */
    CMD_LIMIT_SWITCH  = 0x03,   /* 限位开关状态 */
    CMD_SYSTEM_STATUS = 0x04,   /* 系统状态 */
    CMD_TASK_COMPLETE = 0x05,   /* 任务完成 */
    CMD_ERROR_REPORT  = 0x06,   /* 错误报告 */

    /* Jetson -> STM32 */
    CMD_START_TASK       = 0x10,   /* 开始任务 */
    CMD_STOP_TASK        = 0x11,   /* 停止任务 */
    CMD_PAUSE_TASK       = 0x12,   /* 暂停任务 */
    CMD_RESUME_TASK      = 0x13,   /* 恢复任务 */
    CMD_SET_MOTOR_SPEED  = 0x14,   /* 设置电机速度 */
    CMD_SET_MOTOR_POS    = 0x15,   /* 设置电机位置 */
    CMD_SET_SERVO_ANGLE = 0x16,   /* 设置舵机角度 */
    CMD_RESET_SYSTEM     = 0x17,   /* 系统复位 */
    CMD_CONFIG_PARAMS    = 0x18,   /* 配置参数 */

    /* 上位机(语音模块) -> STM32 */
    CMD_DCM_START      = 0x20,   /* 直流电机启动    data:[ID][占空比] */
    CMD_DCM_STOP       = 0x21,   /* 直流电机停止    data:[ID] */
    CMD_STEPPER_START  = 0x22,   /* 步进电机启动    data:[ID][速度H][速度L] */
    CMD_STEPPER_STOP   = 0x23,   /* 步进电机停止    data:[ID] */
    CMD_SET_SPEED_DIR  = 0x24,   /* 设置转速+方向  data:[ID][类型][方向][值H][值L] */
    CMD_STEPPER_DIR    = 0x25,   /* 步进电机设置方向 data:[ID][方向] */
    CMD_DCM_DIR        = 0x26,   /* 直流电机设置方向 data:[ID][方向] */
    CMD_OFFSET_ADJUST  = 0x27,   /* 调整偏移量      data:[ID][类型][偏移H][偏移L] */
    CMD_OFFSET_RESET  = 0x28,   /* 清零偏移量      data:[ID] */
    CMD_ALL_STOP       = 0x2F,   /* 全部急停        */
    CMD_PROCESS_START  = 0x30,   /* 开始工艺流程    */
    CMD_PROCESS_STOP   = 0x31,   /* 停止工艺流程    */
    CMD_PROCESS_PAUSE  = 0x32,   /* 暂停工艺流程    */
    CMD_PROCESS_RESUME = 0x33,   /* 恢复工艺流程    */
    CMD_SET_CLEAN_MODE = 0x34,   /* 设置清洗模式    */
    CMD_SET_PACK_MODE  = 0x35,   /* 设置打包模式    */
    CMD_SET_PARAM      = 0x36,   /* 设置工艺参数    */
    CMD_GET_STATUS     = 0x37,   /* 查询工艺状态    */
    CMD_ULTRASONIC_TEST = 0x38    /* 超声波单独测试  */
} Protocol_CMD_TypeDef;

/* ========== 系统状态 ========== */
typedef enum {
    SYSTEM_IDLE     = 0x00,   /* 空闲状态 */
    SYSTEM_READY    = 0x01,   /* 就绪状态 */
    SYSTEM_RUNNING  = 0x02,   /* 运行中 */
    SYSTEM_PAUSED   = 0x03,   /* 暂停 */
    SYSTEM_ERROR    = 0x04,   /* 错误状态 */
    SYSTEM_EMERGENCY = 0x05    /* 急停状态 */
} SystemStatus_TypeDef;

/* ========== 错误类型 ========== */
typedef enum {
    ERROR_NONE           = 0x00,   /* 无错误 */
    ERROR_MOTOR_1_FAULT = 0x01,   /* 电机1故障 */
    ERROR_MOTOR_2_FAULT = 0x02,   /* 电机2故障 */
    ERROR_MOTOR_3_FAULT = 0x03,   /* 电机3故障 */
    ERROR_LIMIT_ORIGIN   = 0x10,   /* 原点限位触发 */
    ERROR_LIMIT_FORWARD  = 0x11,   /* 正向限位触发 */
    ERROR_LIMIT_BACKWARD = 0x12,   /* 反向限位触发 */
    ERROR_SENSOR_TIMEOUT = 0x20,   /* 传感器超时 */
    ERROR_COMM_TIMEOUT   = 0x30    /* 通信超时 */
} ErrorCode_TypeDef;

/* 通信缓冲区 */
typedef struct {
    uint8_t  tx_buf[COMM_TX_BUF_SIZE];
    uint16_t tx_head;
    uint16_t tx_tail;
    uint8_t  tx_buf_full;

    uint8_t  rx_buf[COMM_RX_BUF_SIZE];
    uint16_t rx_head;
    uint16_t rx_tail;
    uint8_t  rx_buf_full;
} CommBuffer_TypeDef;

/* 协议帧结构体 */
typedef struct {
    uint8_t  head;
    uint8_t  cmd;
    uint8_t  len;
    uint8_t  data[PROTOCOL_MAX_DATA_LEN];
    uint8_t  checksum;
} ProtocolFrame_TypeDef;

/* 传感器数据结构体（STM32 -> Jetson） */
typedef struct {
    uint16_t motor1_position;   /* 电机1当前位置 */
    uint16_t motor2_position;   /* 电机2当前位置 */
    uint16_t motor3_position;   /* 电机3当前位置 */
    uint16_t motor1_speed;   /* 电机1当前速度 */
    uint16_t motor2_speed;   /* 电机2当前速度 */
    uint16_t motor3_speed;   /* 电机3当前速度 */
    float    servo1_angle;   /* 舵机1角度 */
    float    servo2_angle;   /* 舵机2角度 */
    float    adc_value;      /* ADC采样值 */
} SensorData_TypeDef;

/* 电机控制结构体（Jetson -> STM32） */
typedef struct {
    uint8_t  motor_id;       /* 电机编号(1~3) */
    int16_t  target_speed;   /* 目标速度 */
    int16_t  target_pos;     /* 目标位置 */
    uint8_t  control_mode;   /* 控制模式：0-速度，1-位置 */
} MotorControl_TypeDef;

/* 外部变量声明 */
extern CommBuffer_TypeDef   comm_buffer;
extern SystemStatus_TypeDef g_system_status;
extern ErrorCode_TypeDef   g_error_code;
extern SensorData_TypeDef   g_sensor_data;
extern int8_t  g_dcm_offset[4];      /* 直流电机占空比偏移量(-100~+100) */
extern int16_t g_stepper_offset[3];  /* 步进电机速度偏移量(-20000~+20000) */
extern volatile uint8_t g_protocol_emergency_stop_request; /* 急停请求（ISR设置，主循环执行） */
extern volatile uint8_t g_protocol_process_stop_request;  /* 流程停止请求 */
extern volatile uint8_t g_protocol_process_pause_request; /* 流程暂停请求 */
extern volatile uint8_t g_protocol_process_start_request; /* 流程开始请求 */
extern volatile uint8_t g_protocol_process_resume_request; /* 流程恢复请求 */
extern volatile uint8_t g_protocol_pack_test_request;      /* 分区H打包测试请求 */
extern volatile uint8_t g_protocol_last_cmd;              /* 最后收到的命令（用于调试） */
extern volatile uint8_t g_protocol_cmd_ready;              /* 命令解析成功标志 */

/* 函数声明 */
void Protocol_Init(void);
void Protocol_SendFrame(CommChannel_TypeDef channel, uint8_t cmd, uint8_t* data, uint8_t len);
void Protocol_SendSensorData(CommChannel_TypeDef channel);
void Protocol_SendSystemStatus(CommChannel_TypeDef channel);
void Protocol_SendErrorReport(CommChannel_TypeDef channel, ErrorCode_TypeDef error);
void Protocol_CommChannel_HandleByte(USART_TypeDef* USARTx, uint8_t byte);
void Protocol_ReportSensorData(void);
void Protocol_SendTaskComplete(CommChannel_TypeDef channel);
uint8_t Protocol_CalculateChecksum(uint8_t* data, uint8_t len);

#endif /* __PROTOCOL_H__ */
