/**
 ******************************************************************************
 * 串口总线舵机（飞特 FT-SCS 协议，UART4）
 *
 * 硬件：UART4 PC10(TX) / PC11(RX)，波特率见 bsp_usart_obj.c 中 UART4_BAUDRATE
 *
 * 协议帧（与《舵机SCS通信协议》一致）：
 *   0xFF 0xFF | ID | LEN | INST | PARAM... | CHK
 *   LEN = 从 INST 到 CHK（含）的字节数
 *   CHK = ~(ID + LEN + INST + 所有 PARAM) & 0xFF
 *
 * 写寄存器：INST = 0x03，首字节为起始地址，其后为数据（多字节时顺序以具体型号内存表为准；
 *   SCS 磁编码/常见款目标区0x2A 起共 6 字节为：目标位置、目标时间、目标速度，小端）
 *
 * 连续旋转（360°轮模式）：
 *   代码初始化时会自动写入 EPROM 配置轮模式，无需手动用上位机设置。
 *   轮模式下向 0x2E 写入有符号 16 位目标速度（小端）：正值与负值为相反转向，0 为停转。
 *
 ******************************************************************************
 */

#ifndef __BSP_SERVO_SERIAL_H__
#define __BSP_SERVO_SERIAL_H__

#include "stm32f10x.h"

typedef struct _ServoBus_t ServoBus_t;

struct _ServoBus_t {
    uint8_t id;
    float target_angle;
    float current_angle;
    uint16_t speed;

    void (*Init)(ServoBus_t* self);
    void (*SetAngle)(ServoBus_t* self, float angle);
    void (*SetAngleWithSpeed)(ServoBus_t* self, float angle, uint16_t speed);
    void (*SetWheelSpeed)(ServoBus_t* self, int16_t speed);
    void (*SetID)(ServoBus_t* self, uint8_t new_id);
    float (*ReadAngle)(ServoBus_t* self);
};

extern ServoBus_t SERVO_BUS[4];

#define SCS_ADDR_GOAL_BLOCK 0x2A
#define SCS_ADDR_GOAL_SPEED     0x2Eu
/* STS/SCS 系常见：当前位置 2 字节小端（以型号手册为准，多为 0x38起） */
#define SCS_ADDR_PRESENT_POS_L  0x38u
/* 扭矩开关：SCS/STS 磁编码系列控制表常见为 0x28（0=卸载/无力矩输出，1=加载）；与具体型号 PDF 核对 */
#define SCS_ADDR_TORQUE_SWITCH  0x28u
/* 扭矩限制：地址 0x0E，2字节（低字节在前），范围 0~1023，默认通常为 1023 */
#define SCS_ADDR_TORQUE_LIMIT   0x0Eu

/* EPROM 寄存器（STS/SMS_STS/SCS 系列通用）*/
#define STS_EPROM_MINMAX_L  9u   /* 角度限位最小值低字节 */
#define STS_EPROM_MODE      33u  /* 运行模式寄存器（1=轮模式/恒速模式）*/
#define STS_EPROM_LOCK      55u  /* EPROM 锁存（0=解锁可写 / 1=加锁保护）*/
#define STS_MODE_WHEEL      1u   /* 轮模式值 */
#define STS_SRAM_ACC        41u  /* SRAM 加速度寄存器（ACC）*/

void ServoBus_InitAll(void);

/* 上电后须加载扭矩舵机才会响应目标位置/速度（与上位机「扭矩输出」勾选等效，以内存表为准） */
void ServoBus_SetTorqueSwitch(uint8_t scs_id, uint8_t on);

/* 设置舵机扭矩限制（地址 0x0E），范围 0~1023，值越大扭矩越大 */
void ServoBus_SetTorqueLimit(uint8_t scs_id, uint16_t limit);

/* 注册 TX 调试回调（可为 NULL）：每发一帧前会调用，便于用 UART1 打印 hex确认固件在发数 */
void ServoBus_SetTxDebugHook(void (*cb)(const uint8_t *data, uint16_t len));

/* PING（INST=0x01），用于测物理层：若接线/波特率正确，UART4 环形缓冲内应能收到应答字节 */
void ServoBus_SendPing(uint8_t scs_id);

/* 发 PING 并短等后根据 RX 更新状态；读完并丢弃环缓冲内数据。
 * 若收到与发出的6 字节 PING 完全一致，视为线路回显而非舵机应答，不计 OK。 */
void ServoBus_RefreshPingStatus(uint8_t scs_id);

/* 最近一次 RefreshPingStatus：1=收到疑似舵机应答帧头(FF FF ID)且非回显，0=否 */
uint8_t ServoBus_LastPingOk(void);

/* 底层：按 SCS「写数据」指令发送（UART4） */
void ServoBus_WriteReg(uint8_t scs_id, uint8_t start_addr, const uint8_t* data, uint8_t data_len);

/* 与手册例 4 一致：从 0x2A 写入 6 字节（位置、时间、速度，小端） */
void ServoBus_SetGoalBlock(uint8_t scs_id, uint16_t goal_pos, uint16_t goal_time, uint16_t goal_speed);

/* 读当前位置（关节模式，2 字节小端）；半双工总线短等收包。成功返回 1，失败0 */
uint8_t ServoBus_ReadPresentPosition(uint8_t scs_id, uint16_t *pos_out);

/* 轮模式连续转：写目标速度（需舵机已处于轮模式） */
void ServoBus_SetWheelSpeedId(uint8_t scs_id, int16_t speed);

/* STS/SMS 系列（如 STS3020）：写 EPROM 将运行模式切为「恒速/轮模式」(1)，否则仅写 0x2E 在关节模式下不会持续转。
 * 与官方 SMS_STS::WriteMode 一致：寄存器 33=模式，55=掉电区锁（0 解锁 / 1 加锁）。 */
void ServoBus_STS_EnterWheelMode(uint8_t scs_id);

/* STS 恒速：官方 WriteSpe — 从 SRAM 0x29(ACC) 起写 3 字节 [加速度, 目标速度低, 高]（小端速度） */
void ServoBus_STS_WriteSpeedAcc(uint8_t scs_id, int16_t speed, uint8_t acc);

/* 与上位机GUI一致的轮模式速度写入函数
 * GUI速度范围: -1000 ~ +1000
 * 正速(1~1000) → 寄存器写原值(1~1000)，正转
 * 负速(-1~-1000) → 寄存器写 1024+|v|（1023~2024），反转
 * 写入寄存器0x2E，2字节大端序（与GUI一致）
 * 用法示例: ServoBus_SetWheelSpeedGui(3, 6);     // 舵机3，正转，速度6
 *            ServoBus_SetWheelSpeedGui(8, -134);  // 舵机8，反转，速度134 */
void ServoBus_SetWheelSpeedGui(uint8_t scs_id, int16_t gui_speed);

/*===============================================================
 * ST3215/STS360/STS360T 膜杆（ID2/7）专用协议
 *
 * 与 SCS（ID3/8）的关键差异：
 *   - 速度编码：SCS 用 1024+|v| 大端，ST3215 直接 int16 符号小端
 *   - 速度范围：ST3215 Mode1 速度 ±10~±9999，0=停止
 *
 * 轮模式进入流程：
 *   1. 卸扭矩
 *   2. 解锁 EPROM (0x55=0)
 *   3. 清角限位 (0x09~0x0C=0)
 *   4. 写模式=1 (0x33=1)
 *   5. 锁存 EPROM (0x55=1)
 *   6. SRAM 写零速度 + 开扭矩
 *===============================================================*/

/* ST3215/STS360/STS360T 进入轮模式：写 EPROM 持久化配置 + SRAM 开扭矩
 * 流程：卸扭矩 → 解锁 → 清角限位 → 写模式=1 → 锁存 → 零速+开扭矩
 * 确保舵机上电后静止不飞转，且轮模式配置掉电不丢失。 */
void ServoBus_STS360_EnterWheelMode(uint8_t scs_id);

/* ST3215/STS360 速度写入：int16 小端，正=CW，负=CCW，范围 ±10~±9999，0=停止
 * 注意：|v| < 10 时舵机会忽略，须配合扭矩使用。 */
void ServoBus_STS360_SetWheelSpeed(uint8_t scs_id, int16_t speed);

/* ST3215/STS360 速度+ACC 写入：速度 int16 小端，ACC 写 0x29 */
void ServoBus_STS360_WriteSpeedAcc(uint8_t scs_id, int16_t speed, uint8_t acc);

/* 与 PWM 舵机 SVx_ 宏区分：总线舵机使用 SCS_BUSx_ 前缀 */
#define SCS_BUS1_SetAngle(a)           SERVO_BUS[0].SetAngle(&SERVO_BUS[0], (a))
#define SCS_BUS2_SetAngle(a)           SERVO_BUS[1].SetAngle(&SERVO_BUS[1], (a))
#define SCS_BUS3_SetAngle(a)           SERVO_BUS[2].SetAngle(&SERVO_BUS[2], (a))
#define SCS_BUS4_SetAngle(a)           SERVO_BUS[3].SetAngle(&SERVO_BUS[3], (a))

#define SCS_BUS1_SetAngleSpeed(a, s)   SERVO_BUS[0].SetAngleWithSpeed(&SERVO_BUS[0], (a), (s))
#define SCS_BUS2_SetAngleSpeed(a, s)   SERVO_BUS[1].SetAngleWithSpeed(&SERVO_BUS[1], (a), (s))
#define SCS_BUS3_SetAngleSpeed(a, s)   SERVO_BUS[2].SetAngleWithSpeed(&SERVO_BUS[2], (a), (s))
#define SCS_BUS4_SetAngleSpeed(a, s)   SERVO_BUS[3].SetAngleWithSpeed(&SERVO_BUS[3], (a), (s))

#define SCS_BUS1_SetWheelSpeed(spd)    SERVO_BUS[0].SetWheelSpeed(&SERVO_BUS[0], (spd))
#define SCS_BUS2_SetWheelSpeed(spd)    SERVO_BUS[1].SetWheelSpeed(&SERVO_BUS[1], (spd))
#define SCS_BUS3_SetWheelSpeed(spd)    SERVO_BUS[2].SetWheelSpeed(&SERVO_BUS[2], (spd))
#define SCS_BUS4_SetWheelSpeed(spd)    SERVO_BUS[3].SetWheelSpeed(&SERVO_BUS[3], (spd))

#define SCS_BUS1_ReadAngle()           SERVO_BUS[0].ReadAngle(&SERVO_BUS[0])
#define SCS_BUS2_ReadAngle()           SERVO_BUS[1].ReadAngle(&SERVO_BUS[1])
#define SCS_BUS3_ReadAngle()           SERVO_BUS[2].ReadAngle(&SERVO_BUS[2])
#define SCS_BUS4_ReadAngle()           SERVO_BUS[3].ReadAngle(&SERVO_BUS[3])

#endif

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
