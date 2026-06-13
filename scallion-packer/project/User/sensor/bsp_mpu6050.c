/**
 ******************************************************************************
 * MPU6050 六轴传感器驱动实现（I2C 方式）
 *
 * ============================================================================
 * 支持 PB8/PB9（重映射）和 PB6/PB7（默认）两套接线。
 * Stepper_ReapplyTim1RemapAndPulGpio() 由 bsp_stepper.c 提供。
 * ============================================================================
 *
 * 作者：工程训练中心116电控组
 * 日期：2026-04-19
 ******************************************************************************
 */

#include "bsp_mpu6050.h"
#include "bsp_stepper.h"
#include "stm32f10x_i2c.h"
#include <stdlib.h>

/* ============================================================================
 * 内部变量
 * ============================================================================ */
static MPU6050_Bus_TypeDef s_bus = MPU6050_BUS_NONE;
static MPU6050_Calib_TypeDef s_calib;
static uint8_t s_calib_valid = 0;

/* 当前量程（影响原始值到物理量的换算） */
static MPU6050_AccelFS_TypeDef s_accel_fs = MPU6050_ACCEL_FS_2G;
static MPU6050_GyroFS_TypeDef  s_gyro_fs  = MPU6050_GYRO_FS_250;

/* ============================================================================
 * I2C 私有函数声明
 * ============================================================================ */
static void I2C_Configuration_PB8_PB9(void);
static void I2C_Configuration_PB6_PB7(void);
static uint8_t I2C_WriteReg(uint8_t reg, uint8_t data);
static uint8_t I2C_ReadReg(uint8_t reg, uint8_t *data);
static uint8_t I2C_ReadMultiRegs(uint8_t start_reg, uint8_t *buf, uint8_t len);

/* ============================================================================
 * I2C 延时（软件模拟 I2C，无硬件超时检测时使用）
 * ============================================================================ */
#define I2C_DELAY()  do { volatile uint32_t i = 72; while(i--) { __NOP(); } } while(0)

/* ============================================================================
 * 初始化 I2C 总线（PB8/PB9，带重映射）
 * ============================================================================ */
static void I2C_Configuration_PB8_PB9(void)
{
    GPIO_InitTypeDef g;

    /* 使能时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* I2C1 重映射到 PB8/PB9 */
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

    /* PB8=SCL, PB9=SDA，配置为复用开漏（I2C 必须开漏） */
    g.GPIO_Pin   = GPIO_Pin_8;          /* SCL */
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &g);

    g.GPIO_Pin   = GPIO_Pin_9;          /* SDA */
    GPIO_Init(GPIOB, &g);

    /* I2C1 配置 */
    I2C_InitTypeDef i2c;
    i2c.I2C_Mode            = I2C_Mode_I2C;
    i2c.I2C_DutyCycle       = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1     = 0x00;
    i2c.I2C_Ack             = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed      = 400000;  /* 400kHz 快速模式 */
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
}

/* ============================================================================
 * 初始化 I2C 总线（PB6/PB7，默认脚）
 * ============================================================================ */
static void I2C_Configuration_PB6_PB7(void)
{
    GPIO_InitTypeDef g;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* PB6=SCL, PB7=SDA，复用开漏 */
    g.GPIO_Pin   = GPIO_Pin_6;          /* SCL */
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &g);

    g.GPIO_Pin   = GPIO_Pin_7;          /* SDA */
    GPIO_Init(GPIOB, &g);

    I2C_InitTypeDef i2c;
    i2c.I2C_Mode            = I2C_Mode_I2C;
    i2c.I2C_DutyCycle       = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1     = 0x00;
    i2c.I2C_Ack             = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed      = 400000;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
}

/* ============================================================================
 * I2C 等待事件（带超时）
 * ============================================================================ */
static uint8_t I2C_WaitEvent(uint32_t event, uint32_t timeout_ms)
{
    uint32_t tick_start = I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) ? 0 : 1;
    (void)tick_start; /* 简化版：直接轮询 */
    uint32_t count = 0;
    while (count++ < 1000000) {
        if (I2C_CheckEvent(I2C1, event) == SUCCESS)
            return 0;
    }
    return 1; /* 超时 */
}

/* ============================================================================
 * 写单个寄存器
 * ============================================================================ */
static uint8_t I2C_WriteReg(uint8_t reg, uint8_t data)
{
    /* 发送起始位 */
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, 100)) return 1;

    /* 发送从机地址 + 写 */
    I2C_Send7bitAddress(I2C1, MPU6050_I2C_ADDR << 1, I2C_Direction_Transmitter);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, 100)) return 2;

    /* 发送寄存器地址 */
    I2C_SendData(I2C1, reg);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, 100)) return 3;

    /* 发送数据 */
    I2C_SendData(I2C1, data);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, 100)) return 4;

    /* 发送停止位 */
    I2C_GenerateSTOP(I2C1, ENABLE);
    return 0;
}

/* ============================================================================
 * 读单个寄存器
 * ============================================================================ */
static uint8_t I2C_ReadReg(uint8_t reg, uint8_t *data)
{
    /* 发送起始位 */
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, 100)) return 1;

    /* 发送从机地址 + 写 */
    I2C_Send7bitAddress(I2C1, MPU6050_I2C_ADDR << 1, I2C_Direction_Transmitter);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, 100)) return 2;

    /* 发送寄存器地址 */
    I2C_SendData(I2C1, reg);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, 100)) return 3;

    /* 重新发送起始位（restart）*/
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, 100)) return 4;

    /* 发送从机地址 + 读 */
    I2C_Send7bitAddress(I2C1, MPU6050_I2C_ADDR << 1, I2C_Direction_Receiver);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, 100)) return 5;

    /* 关闭 ACK，接收最后一个字节后发送 NACK */
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    (void)I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, 100);
    *data = I2C_ReceiveData(I2C1);

    /* 发送停止位 */
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);  /* 恢复默认 ACK */
    return 0;
}

/* ============================================================================
 * 连续读多个寄存器
 * ============================================================================ */
static uint8_t I2C_ReadMultiRegs(uint8_t start_reg, uint8_t *buf, uint8_t len)
{
    if (len == 0) return 0;

    /* 发送起始位 */
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, 100)) return 1;

    /* 发送从机地址 + 写 */
    I2C_Send7bitAddress(I2C1, MPU6050_I2C_ADDR << 1, I2C_Direction_Transmitter);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, 100)) return 2;

    /* 发送起始寄存器地址 */
    I2C_SendData(I2C1, start_reg);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, 100)) return 3;

    /* Restart */
    I2C_GenerateSTART(I2C1, ENABLE);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, 100)) return 4;

    /* 发送从机地址 + 读 */
    I2C_Send7bitAddress(I2C1, MPU6050_I2C_ADDR << 1, I2C_Direction_Receiver);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, 100)) return 5;

    while (len > 1) {
        I2C_AcknowledgeConfig(I2C1, ENABLE);
        (void)I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, 100);
        *buf++ = I2C_ReceiveData(I2C1);
        len--;
    }

    /* 最后一个字节：NACK */
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    (void)I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, 100);
    *buf = I2C_ReceiveData(I2C1);

    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 0;
}

/* ============================================================================
 * 检测 MPU6050 是否在线（读 WHO_AM_I）
 * ============================================================================ */
static uint8_t MPU6050_Ping(uint8_t *who_am_i)
{
    if (I2C_ReadReg(MPU6050_REG_WHO_AM_I, who_am_i) == 0) {
        return (*who_am_i == 0x68 || *who_am_i == 0x69) ? 1 : 0;
    }
    return 0;
}

/* ============================================================================
 * 主初始化函数（自动探测）
 * ============================================================================ */
MPU6050_Bus_TypeDef MPU6050_Init(void)
{
    uint8_t who;

    /* --- 尝试 PB8/PB9（重映射）--- */
    I2C_Configuration_PB8_PB9();
    if (MPU6050_Ping(&who)) {
        s_bus = MPU6050_BUS_PB8_PB9;
        goto init_ok;
    }

    /* I2C1 未使能，下一次 I2C_Configuration_PB6_PB7 会重新配置 */

    /* --- 尝试 PB6/PB7（默认脚）--- */
    /* 关闭已启用的 I2C1 */
    I2C_Cmd(I2C1, DISABLE);
    I2C_DeInit(I2C1);
    /* 恢复 PB8/PB9 为普通 GPIO（关闭重映射） */
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, DISABLE);
    {
        GPIO_InitTypeDef g;
        g.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
        g.GPIO_Speed = GPIO_Speed_50MHz;
        g.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOB, &g);
    }

    I2C_Configuration_PB6_PB7();
    if (MPU6050_Ping(&who)) {
        s_bus = MPU6050_BUS_PB6_PB7;
        goto init_ok;
    }

    /* 未检测到 */
    s_bus = MPU6050_BUS_NONE;
    return s_bus;

init_ok:
    /* 唤醒 MPU6050（清除 PWR_MGMT_1 的 SLEEP 位） */
    I2C_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00);
    I2C_DELAY();

    /* 配置采样率：SMPLRT_DIV = 7 → 1kHz / (7+1) = 125Hz */
    I2C_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x07);

    /* 配置数字低通滤波器：DLPF=6 → ~5Hz 带宽（陀螺仪） */
    I2C_WriteReg(MPU6050_REG_CONFIG, 0x06);

    /* 加速度计 ±2g */
    I2C_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00);
    s_accel_fs = MPU6050_ACCEL_FS_2G;

    /* 陀螺仪 ±250°/s */
    I2C_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x00);
    s_gyro_fs = MPU6050_GYRO_FS_250;

    /* PB8/PB9 模式下重映射会改写 AFIO->MAPR，
     * 须在 MPU6050_Init() 后重新应用 TIM1 FullRemap，
     * 避免步进 1/5/6 脉冲丢失。
     */
    if (s_bus == MPU6050_BUS_PB8_PB9) {
        Stepper_ReapplyTim1RemapAndPulGpio();
    }

    /* 初始化校准偏移为 0 */
    s_calib_valid = 0;
    return s_bus;
}

/* ============================================================================
 * 检测传感器是否就绪
 * ============================================================================ */
uint8_t MPU6050_IsReady(void)
{
    return (s_bus != MPU6050_BUS_NONE) ? 1 : 0;
}

/* ============================================================================
 * 获取当前总线模式
 * ============================================================================ */
MPU6050_Bus_TypeDef MPU6050_GetBus(void)
{
    return s_bus;
}

/* ============================================================================
 * 获取原始数据（高字节在前）
 * ============================================================================ */
void MPU6050_GetRawData(MPU6050_RawData_TypeDef *data)
{
    uint8_t buf[14];
    if (data == NULL) return;

    if (I2C_ReadMultiRegs(MPU6050_REG_ACCEL_XOUT_H, buf, 14) == 0) {
        data->accel_x      = (int16_t)((uint16_t)buf[0]  << 8 | buf[1]);
        data->accel_y      = (int16_t)((uint16_t)buf[2]  << 8 | buf[3]);
        data->accel_z      = (int16_t)((uint16_t)buf[4]  << 8 | buf[5]);
        data->temperature  = (int16_t)((uint16_t)buf[6]  << 8 | buf[7]);
        data->gyro_x       = (int16_t)((uint16_t)buf[8]  << 8 | buf[9]);
        data->gyro_y       = (int16_t)((uint16_t)buf[10] << 8 | buf[11]);
        data->gyro_z       = (int16_t)((uint16_t)buf[12] << 8 | buf[13]);
    } else {
        data->accel_x = data->accel_y = data->accel_z = 0;
        data->temperature = 0;
        data->gyro_x = data->gyro_y = data->gyro_z = 0;
    }
}

/* ============================================================================
 * 获取去除零偏后的数据
 * ============================================================================ */
void MPU6050_GetCalibratedData(MPU6050_RawData_TypeDef *data)
{
    if (data == NULL) return;
    if (!s_calib_valid) return;  /* 未校准则返回原始值 */

    MPU6050_GetRawData(data);
    data->accel_x -= s_calib.accel_x_offset;
    data->accel_y -= s_calib.accel_y_offset;
    data->accel_z -= s_calib.accel_z_offset;
    data->gyro_x  -= s_calib.gyro_x_offset;
    data->gyro_y  -= s_calib.gyro_y_offset;
    data->gyro_z  -= s_calib.gyro_z_offset;
}

/* ============================================================================
 * 校准（采集 N 次均值作为零偏）
 * ============================================================================ */
void MPU6050_Calibrate(uint16_t sample_count)
{
    MPU6050_RawData_TypeDef sum = {0};
    uint16_t i;

    for (i = 0; i < sample_count; i++) {
        MPU6050_RawData_TypeDef d;
        MPU6050_GetRawData(&d);
        sum.accel_x     += d.accel_x;
        sum.accel_y     += d.accel_y;
        sum.accel_z     += d.accel_z;
        sum.gyro_x      += d.gyro_x;
        sum.gyro_y      += d.gyro_y;
        sum.gyro_z      += d.gyro_z;
        /* 简单延时，避免 I2C 总线竞争 */
        for (volatile uint32_t t = 200; t; t--) { __NOP(); }
    }

    s_calib.accel_x_offset = sum.accel_x / (int16_t)sample_count;
    s_calib.accel_y_offset = sum.accel_y / (int16_t)sample_count;
    s_calib.accel_z_offset = sum.accel_z / (int16_t)sample_count;
    /* Z 轴零偏处理：静止时 Z 轴应等于 +1g */
    if (s_accel_fs == MPU6050_ACCEL_FS_2G) {
        s_calib.accel_z_offset -= 16384;  /* ±2g 时 1g ≈ 16384 LSB */
    } else if (s_accel_fs == MPU6050_ACCEL_FS_4G) {
        s_calib.accel_z_offset -= 8192;
    }
    s_calib.gyro_x_offset  = sum.gyro_x  / (int16_t)sample_count;
    s_calib.gyro_y_offset  = sum.gyro_y  / (int16_t)sample_count;
    s_calib.gyro_z_offset  = sum.gyro_z  / (int16_t)sample_count;

    s_calib_valid = 1;
}

/* ============================================================================
 * 获取校准偏移
 * ============================================================================ */
void MPU6050_GetCalibOffset(MPU6050_Calib_TypeDef *calib)
{
    if (calib == NULL) return;
    *calib = s_calib;
}

/* ============================================================================
 * 获取温度（摄氏度）
 * ============================================================================ */
float MPU6050_GetTemperature_C(void)
{
    uint8_t buf[2];
    if (I2C_ReadMultiRegs(MPU6050_REG_TEMP_OUT_H, buf, 2) == 0) {
        int16_t temp_raw = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
        return (float)temp_raw / 340.0f + 36.53f;  /* 手册公式 */
    }
    return 0.0f;
}

/* ============================================================================
 * 设置加速度计量程
 * ============================================================================ */
void MPU6050_SetAccelFS(MPU6050_AccelFS_TypeDef fs)
{
    uint8_t val;
    switch (fs) {
        case MPU6050_ACCEL_FS_2G:  val = 0x00; break;
        case MPU6050_ACCEL_FS_4G:  val = 0x08; break;
        case MPU6050_ACCEL_FS_8G:  val = 0x10; break;
        case MPU6050_ACCEL_FS_16G: val = 0x18; break;
        default: return;
    }
    I2C_WriteReg(MPU6050_REG_ACCEL_CONFIG, val);
    s_accel_fs = fs;
}

/* ============================================================================
 * 设置陀螺仪量程
 * ============================================================================ */
void MPU6050_SetGyroFS(MPU6050_GyroFS_TypeDef fs)
{
    uint8_t val;
    switch (fs) {
        case MPU6050_GYRO_FS_250:   val = 0x00; break;
        case MPU6050_GYRO_FS_500:  val = 0x08; break;
        case MPU6050_GYRO_FS_1000: val = 0x10; break;
        case MPU6050_GYRO_FS_2000: val = 0x18; break;
        default: return;
    }
    I2C_WriteReg(MPU6050_REG_GYRO_CONFIG, val);
    s_gyro_fs = fs;
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE*****/
