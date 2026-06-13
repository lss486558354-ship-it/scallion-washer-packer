/**
 ******************************************************************************
 * 飞特 SCS 串口舵机 — UART4 发送实现
 ******************************************************************************
 */

#include "bsp_servo_serial.h"
#include "bsp_usart_obj.h"
#include <string.h>

/* SetAngle：把 0~180° 线性映射到 0~SCS_POS_MAX（默认 4095，与手册示例量纲一致） */
#define SCS_POS_MAX  4095u

#define SCS_INST_WRITE  0x03u
#define SCS_INST_READ   0x02u
#define SCS_INST_PING   0x01u

/*===============================================================
 * 寄存器地址定义
 *
 * STS/SMS_STS/SCS 系列通用 EPROM 地址：
 *   0x09~0x0C = 角度限位
 *   0x33      = 运行模式（1=轮模式/恒速模式）
 *   0x55      = EPROM 锁存（0=解锁可写 / 1=加锁保护）
 *===============================================================*/

/* 通用寄存器（两种型号通用）*/
#define SCS_ADDR_GOAL_SPEED     0x2Eu  /* 目标速度（两种型号通用）*/

static void (*s_scs_tx_debug)(const uint8_t *data, uint16_t len) = NULL;
static uint8_t s_scs_last_ping_ok;

/* 半双工总线：帧尾后留极短空闲，避免紧接下一帧被舵机/缓冲吃掉 */
static void scs_post_tx_idle_gap(void)
{
    volatile uint32_t n;

    for (n = 0u; n < 25000u; n++) {
    }
}

static void scs_uart4_send(const uint8_t *buf, uint16_t len)
{
    UART[UART4_ID].SendData(&UART[UART4_ID], (uint8_t *)buf, len);
    if (s_scs_tx_debug != NULL) {
        s_scs_tx_debug(buf, len);
    }
    scs_post_tx_idle_gap();
}

void ServoBus_SetTxDebugHook(void (*cb)(const uint8_t *data, uint16_t len))
{
    s_scs_tx_debug = cb;
}

static uint8_t scs_checksum_byte(uint8_t id, uint8_t len, uint8_t inst,
    const uint8_t* params, uint8_t nparam)
{
    uint16_t s = (uint16_t)id + len + inst;
    uint8_t i;

    for (i = 0; i < nparam; i++) {
        s += params[i];
    }
    return (uint8_t)(~((uint8_t)s));
}

static void scs_fill_ping_frame(uint8_t scs_id, uint8_t out[6])
{
    uint8_t len_field = 2u;

    out[0] = 0xFFu;
    out[1] = 0xFFu;
    out[2] = scs_id;
    out[3] = len_field;
    out[4] = SCS_INST_PING;
    out[5] = scs_checksum_byte(scs_id, len_field, SCS_INST_PING, NULL, 0);
}

void ServoBus_SendPing(uint8_t scs_id)
{
    uint8_t buf[6];

    scs_fill_ping_frame(scs_id, buf);
    scs_uart4_send(buf, 6u);
}

void ServoBus_RefreshPingStatus(uint8_t scs_id)
{
    uint8_t tx[6];
    uint8_t rx[24];
    uint8_t n = 0;
    volatile uint32_t w, spin;

    UART4_ClearBuf();
    scs_fill_ping_frame(scs_id, tx);
    scs_uart4_send(tx, 6u);
    for (w = 0u; w < 12u; w++) {
        for (spin = 0u; spin < 25000u; spin++) {
        }
    }
    while (UART4_Available() != 0u && n < (uint8_t)sizeof(rx)) {
        rx[n] = UART4_ReadByte();
        n++;
    }

    s_scs_last_ping_ok = 0;
    /* 无字节或不足一帧：视为未收到有效应答 */
    if (n < 6u) {
        return;
    }
    /* 与发出的 PING 完全一致时多为 TX→RX 回显，不能当作舵机应答 */
    if (memcmp(rx, tx, 6u) == 0) {
        return;
    }
    if (rx[0] == 0xFFu && rx[1] == 0xFFu && rx[2] == scs_id) {
        s_scs_last_ping_ok = 1u;
    }
}

uint8_t ServoBus_LastPingOk(void)
{
    return s_scs_last_ping_ok;
}

uint8_t ServoBus_ReadPresentPosition(uint8_t scs_id, uint16_t *pos_out)
{
    uint8_t  tx[8];
    uint8_t  rx[32];
    uint8_t  n = 0;
    uint8_t  len_field = 4u;
    uint8_t  params[2];
    volatile uint32_t w, spin;
    uint8_t  i;

    if (pos_out == NULL) {
        return 0u;
    }
    params[0] = SCS_ADDR_PRESENT_POS_L;
    params[1] = 2u;
    tx[0] = 0xFFu;
    tx[1] = 0xFFu;
    tx[2] = scs_id;
    tx[3] = len_field;
    tx[4] = SCS_INST_READ;
    tx[5] = params[0];
    tx[6] = params[1];
    tx[7] = scs_checksum_byte(scs_id, len_field, SCS_INST_READ, params, 2u);

    UART4_ClearBuf();
    scs_uart4_send(tx, 8u);
    for (w = 0u; w < 20u; w++) {
        for (spin = 0u; spin < 25000u; spin++) {
        }
        while (UART4_Available() != 0u && n < (uint8_t)sizeof(rx)) {
            rx[n] = UART4_ReadByte();
            n++;
        }
        if (n >= 9u) {
            break;
        }
    }
    *pos_out = 0;
    for (i = 0u; i + 8u <= n; i++) {
        if (rx[i] == 0xFFu && rx[i + 1u] == 0xFFu && rx[i + 2u] == scs_id) {
            /* FF FF ID LEN ERR POS_L POS_H CHK */
            if (i + 8u <= n) {
                *pos_out = (uint16_t)rx[i + 5u] | ((uint16_t)rx[i + 6u] << 8);
                return 1u;
            }
        }
    }
    return 0u;
}

void ServoBus_WriteReg(uint8_t scs_id, uint8_t start_addr, const uint8_t* data, uint8_t data_len)
{
    uint8_t buf[40];
    uint8_t len_field;
    uint8_t i;

    if (data_len > (sizeof(buf) - 7u)) {
        return;
    }

    len_field = (uint8_t)(1u + 1u + data_len + 1u);

    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = scs_id;
    buf[3] = len_field;
    buf[4] = SCS_INST_WRITE;
    buf[5] = start_addr;
    for (i = 0; i < data_len; i++) {
        buf[6u + i] = data[i];
    }
    buf[6u + data_len] = scs_checksum_byte(scs_id, len_field, SCS_INST_WRITE, buf + 5,
        (uint8_t)(1u + data_len));

    scs_uart4_send(buf, (uint16_t)(7u + data_len));
}

void ServoBus_SetGoalBlock(uint8_t scs_id, uint16_t goal_pos, uint16_t goal_time, uint16_t goal_speed)
{
    uint8_t d[6];

    d[0] = (uint8_t)(goal_pos & 0xFFu);
    d[1] = (uint8_t)((goal_pos >> 8) & 0xFFu);
    d[2] = (uint8_t)(goal_time & 0xFFu);
    d[3] = (uint8_t)((goal_time >> 8) & 0xFFu);
    d[4] = (uint8_t)(goal_speed & 0xFFu);
    d[5] = (uint8_t)((goal_speed >> 8) & 0xFFu);
    ServoBus_WriteReg(scs_id, SCS_ADDR_GOAL_BLOCK, d, 6);
}

void ServoBus_SetWheelSpeedId(uint8_t scs_id, int16_t speed)
{
    /* 与 PC 上位机 servo_debug_tool.py 的 _speed_to_reg 完全一致的编码：
     * 正速 → 原值；负速 → 1024 + |v|。
     * 写入 0x2E，大端（高字节在前），与 PC 上位机一致。 */
    uint16_t v;
    uint8_t d[2];

    if (speed >= 0) {
        v = (uint16_t)((speed > 1000) ? 1000 : speed);
    } else {
        int16_t absv = -speed;
        if (absv > 1000) absv = 1000;
        v = (uint16_t)(1024 + absv);
    }
    d[0] = (uint8_t)((v >> 8) & 0xFFu);
    d[1] = (uint8_t)(v & 0xFFu);
    ServoBus_WriteReg(scs_id, SCS_ADDR_GOAL_SPEED, d, 2);
}

void ServoBus_SetTorqueSwitch(uint8_t scs_id, uint8_t on)
{
    uint8_t v = on ? 1u : 0u;

    ServoBus_WriteReg(scs_id, SCS_ADDR_TORQUE_SWITCH, &v, 1u);
}

void ServoBus_SetTorqueLimit(uint8_t scs_id, uint16_t limit)
{
    uint8_t d[2];
    uint16_t v = (limit > 1023u) ? 1023u : limit;

    d[0] = (uint8_t)(v & 0xFFu);
    d[1] = (uint8_t)((v >> 8) & 0xFFu);
    ServoBus_WriteReg(scs_id, SCS_ADDR_TORQUE_LIMIT, d, 2);
}

/* EPROM 写入后短等；禁止用 BSP_GetTickMs：本模块会在 SysTick→AppHook1ms 里调用，会导致死等 */
static void sts_eprom_pause(void)
{
    volatile uint32_t i, j;

    for (i = 0u; i < 35u; i++) {
        for (j = 0u; j < 25000u; j++) {
        }
    }
}

void ServoBus_STS_EnterWheelMode(uint8_t scs_id)
{
    uint8_t v;
    uint8_t z4[4] = { 0u, 0u, 0u, 0u };

    /* 改 EPROM 前先卸扭矩，避免带载写模式异常 */
    ServoBus_SetTorqueSwitch(scs_id, 0u);
    sts_eprom_pause();

    v = 0u;
    ServoBus_WriteReg(scs_id, STS_EPROM_LOCK, &v, 1u);
    sts_eprom_pause();

    /* 角限位全 0：轮模式/连续转常见要求（与官方 WheelMode 流程一致） */
    ServoBus_WriteReg(scs_id, STS_EPROM_MINMAX_L, z4, 4u);
    sts_eprom_pause();

    v = STS_MODE_WHEEL;
    ServoBus_WriteReg(scs_id, STS_EPROM_MODE, &v, 1u);
    sts_eprom_pause();

    v = 1u;
    ServoBus_WriteReg(scs_id, STS_EPROM_LOCK, &v, 1u);
    sts_eprom_pause();
}

/* 与 PC 上位机 servo_debug_tool.py 的 _speed_to_reg 完全一致的速度编码：
 * 正速 → 原值；负速 → 1024 + |v|。
 * 写入 0x2E，大端（高字节在前），与 PC 上位机一致。ACC 写 0x29（1字节）。 */
void ServoBus_STS_WriteSpeedAcc(uint8_t scs_id, int16_t speed, uint8_t acc)
{
    uint16_t v;
    uint8_t d_speed[2];
    uint8_t d_acc[1];

    /* 速度编码 */
    if (speed >= 0) {
        v = (uint16_t)((speed > 1000) ? 1000 : speed);
    } else {
        int16_t absv = -speed;
        if (absv > 1000) absv = 1000;
        v = (uint16_t)(1024 + absv);
    }
    d_speed[0] = (uint8_t)((v >> 8) & 0xFFu);
    d_speed[1] = (uint8_t)(v & 0xFFu);
    ServoBus_WriteReg(scs_id, SCS_ADDR_GOAL_SPEED, d_speed, 2u);

    /* ACC 单独写 0x29（1字节）*/
    d_acc[0] = acc;
    ServoBus_WriteReg(scs_id, STS_SRAM_ACC, d_acc, 1u);
}

/* 与上位机GUI一致的轮模式速度写入函数
 * GUI速度范围: -1000 ~ +1000
 * 正速(1~1000) → 寄存器写原值，正转
 * 负速(-1~-1000) → 寄存器写 1024+|v|，反转
 * 写入寄存器0x2E，2字节大端序（与GUI _speed_to_reg 一致）
 * 用法: ServoBus_SetWheelSpeedGui(3, 6);    // 舵机3，正转，速度6
 *        ServoBus_SetWheelSpeedGui(8, -134); // 舵机8，反转，速度134
 */
void ServoBus_SetWheelSpeedGui(uint8_t scs_id, int16_t gui_speed)
{
    uint16_t v;
    uint8_t d[2];

    if (gui_speed >= 0) {
        v = (uint16_t)((gui_speed > 1000) ? 1000 : gui_speed);
    } else {
        int16_t absv = -gui_speed;
        if (absv > 1000) absv = 1000;
        v = (uint16_t)(1024 + absv);
    }

    d[0] = (uint8_t)((v >> 8) & 0xFFu);  /* 大端：高字节在前 */
    d[1] = (uint8_t)(v & 0xFFu);          /* 大端：低字节在后 */
    ServoBus_WriteReg(scs_id, SCS_ADDR_GOAL_SPEED, d, 2);
}

/* ST3215/STS360/STS360T 进入轮模式：先写 EPROM 持久化配置模式寄存器，再写 SRAM 开扭矩。
 * 关键：
 *   1. 写 EPROM 前先卸扭矩，避免带载写模式异常
 *   2. EPROM 配置：解锁 → 清角限位 → 写模式=1 → 锁存
 *   3. SRAM 最后写零速度 + 开扭矩，确保舵机上电后静止不飞转
 *
 * EPROM 寄存器地址（与 STS/SMS 系列兼容）：
 *   0x55 = EPROM 锁存（0=解锁可写 / 1=加锁保护）
 *   0x33 = 运行模式（1=轮模式/恒速模式）
 *   0x09~0x0C = 角度限位（全 0 表示不限制） */
void ServoBus_STS360_EnterWheelMode(uint8_t scs_id)
{
    uint8_t v;
    uint8_t z4[4] = { 0u, 0u, 0u, 0u };

    /* 写 EPROM 前先卸扭矩，避免带载写模式异常 */
    ServoBus_SetTorqueSwitch(scs_id, 0u);
    sts_eprom_pause();

    /* 解锁 EPROM */
    v = 0u;
    ServoBus_WriteReg(scs_id, STS_EPROM_LOCK, &v, 1u);
    sts_eprom_pause();

    /* 角限位全 0：轮模式/连续转不限制角度 */
    ServoBus_WriteReg(scs_id, STS_EPROM_MINMAX_L, z4, 4u);
    sts_eprom_pause();

    /* 写轮模式 = 1 */
    v = STS_MODE_WHEEL;
    ServoBus_WriteReg(scs_id, STS_EPROM_MODE, &v, 1u);
    sts_eprom_pause();

    /* 锁存 EPROM */
    v = 1u;
    ServoBus_WriteReg(scs_id, STS_EPROM_LOCK, &v, 1u);
    sts_eprom_pause();

    /* SRAM：写零速度清旧值 + 开扭矩，舵机静止 */
    {
        uint8_t d[2];
        d[0] = 0x00u;
        d[1] = 0x00u;
        ServoBus_WriteReg(scs_id, SCS_ADDR_GOAL_SPEED, d, 2u);
    }
    ServoBus_SetTorqueSwitch(scs_id, 1u);
}

/* ST3215/STS360 速度写入：int16 小端，正=CW，负=CCW，范围 ±10~±9999，0=停止
 * 关键：先写速度（SRAM），后开扭矩，避免上电时舵机以旧速度飞转。
 * 注意：|v| < 10 时舵机会忽略，须配合扭矩使用。 */
void ServoBus_STS360_SetWheelSpeed(uint8_t scs_id, int16_t speed)
{
    uint8_t d[2];

    /* 小端：低字节在前 */
    d[0] = (uint8_t)((uint16_t)speed & 0xFFu);
    d[1] = (uint8_t)(((uint16_t)speed >> 8) & 0xFFu);
    ServoBus_WriteReg(scs_id, SCS_ADDR_GOAL_SPEED, d, 2);
    /* 速度写入后立即开扭矩，使舵机立即以设定速度转动 */
    ServoBus_SetTorqueSwitch(scs_id, 1u);
}

/* ST3215/STS360 速度+ACC 写入：先写速度，再开扭矩
 * ACC 写 0x29（ST3215 ACC 寄存器）*/
void ServoBus_STS360_WriteSpeedAcc(uint8_t scs_id, int16_t speed, uint8_t acc)
{
    uint8_t d_speed[2];
    uint8_t d_acc[1];

    /* 速度写 SRAM（先于扭矩），避免上电时以旧速度飞转 */
    d_speed[0] = (uint8_t)((uint16_t)speed & 0xFFu);
    d_speed[1] = (uint8_t)(((uint16_t)speed >> 8) & 0xFFu);
    ServoBus_WriteReg(scs_id, SCS_ADDR_GOAL_SPEED, d_speed, 2u);

    /* ACC 写 SRAM */
    d_acc[0] = acc;
    ServoBus_WriteReg(scs_id, 0x29u, d_acc, 1u);

    /* 速度/ACC 写入后立即开扭矩，舵机立即以设定速度转动 */
    ServoBus_SetTorqueSwitch(scs_id, 1u);
}

static void ServoBus_InitImpl(ServoBus_t* self)
{
    (void)self;
}

static void ServoBus_SetAngleImpl(ServoBus_t* self, float angle)
{
    uint16_t pos;

    if (angle < 0.0f) {
        angle = 0.0f;
    }
    if (angle > 180.0f) {
        angle = 180.0f;
    }
    self->target_angle = angle;
    pos = (uint16_t)(angle * (float)SCS_POS_MAX / 180.0f + 0.5f);
    ServoBus_SetGoalBlock(self->id, pos, 0, self->speed);
    self->current_angle = angle;
}

static void ServoBus_SetAngleWithSpeedImpl(ServoBus_t* self, float angle, uint16_t speed)
{
    self->speed = speed;
    ServoBus_SetAngleImpl(self, angle);
}

static void ServoBus_SetWheelSpeedImpl(ServoBus_t* self, int16_t speed)
{
    ServoBus_SetWheelSpeedId(self->id, speed);
}

static void ServoBus_SetIDStub(ServoBus_t* self, uint8_t new_id)
{
    (void)self;
    (void)new_id;
}

static float ServoBus_ReadAngleImpl(ServoBus_t* self)
{
    return self->current_angle;
}

ServoBus_t SERVO_BUS[4];

void ServoBus_InitAll(void)
{
    uint8_t i;

    UART4_ClearBuf();
    for (i = 0; i < 4u; i++) {
        SERVO_BUS[i].id = (uint8_t)(i + 1u);
        SERVO_BUS[i].target_angle = 90.0f;
        SERVO_BUS[i].current_angle = 90.0f;
        SERVO_BUS[i].speed = 500u;
        SERVO_BUS[i].Init = ServoBus_InitImpl;
        SERVO_BUS[i].SetAngle = ServoBus_SetAngleImpl;
        SERVO_BUS[i].SetAngleWithSpeed = ServoBus_SetAngleWithSpeedImpl;
        SERVO_BUS[i].SetWheelSpeed = ServoBus_SetWheelSpeedImpl;
        SERVO_BUS[i].SetID = ServoBus_SetIDStub;
        SERVO_BUS[i].ReadAngle = ServoBus_ReadAngleImpl;
    }
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
