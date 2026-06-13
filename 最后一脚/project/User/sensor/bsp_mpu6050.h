/**
 ******************************************************************************
 * MPU6050 六轴传感器驱动（I2C 方式）
 *
 * ============================================================================
 * 硬件配置（STM32F103VET6 I2C1）：
 *   方案A（推荐）：PB8=SCL，PB9=SDA（TIM4_CH3/CH4 重映射脚）
 *   方案B（备选）：PB6=SCL，PB7=SDA（I2C1 默认脚）
 *
 *   单板只应焊接其中一套，勿两套同时焊接造成 I2C 总线冲突。
 *
 * 使用示例：
 *   MPU6050_Init();              // 初始化（自动探测 PB8/PB9 或 PB6/PB7）
 *   int16_t ax, ay, az;
 *   int16_t gx, gy, gz;
 *   MPU6050_GetRawData(&ax, &ay, &az, &gx, &gy, &gz);
 *
 * 注意：
 *   - 使用 PB8/PB9 时会启用 GPIO_Remap_I2C1，需在 MPU6050_Init() 之后
 *     调用 Stepper_ReapplyTim1RemapAndPulGpio() 重新配置 TIM1 FullRemap，
 *     否则步进1/5/6 可能无脉冲输出。
 *   - 如使用 PB6/PB7（I2C1 默认），无需重映射。
 *
 * ============================================================================
 *
 * 作者：工程训练中心116电控组
 * 日期：2026-04-19
 ******************************************************************************
 */

#ifndef __BSP_MPU6050_H__
#define __BSP_MPU6050_H__

#include "stm32f10x.h"

/* ============================================================================
 * MPU6050 I2C 地址（AD0 接地 = 0x68，AD0 接 VCC = 0x69）
 * ============================================================================ */
#define MPU6050_I2C_ADDR         0x68

/* ============================================================================
 * MPU6050 寄存器地址
 * ============================================================================ */
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_INT_STATUS   0x3A
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_ACCEL_XOUT_L 0x3C
#define MPU6050_REG_ACCEL_YOUT_H 0x3D
#define MPU6050_REG_ACCEL_YOUT_L 0x3E
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F
#define MPU6050_REG_ACCEL_ZOUT_L 0x40
#define MPU6050_REG_TEMP_OUT_H   0x41
#define MPU6050_REG_TEMP_OUT_L   0x42
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_GYRO_XOUT_L  0x44
#define MPU6050_REG_GYRO_YOUT_H  0x45
#define MPU6050_REG_GYRO_YOUT_L  0x46
#define MPU6050_REG_GYRO_ZOUT_H  0x47
#define MPU6050_REG_GYRO_ZOUT_L  0x48
#define MPU6050_REG_WHO_AM_I     0x75

/* ============================================================================
 * 初始化模式
 * ============================================================================ */
typedef enum {
    MPU6050_BUS_PB8_PB9 = 0,   /* PB8=SCL, PB9=SDA（需 I2C 重映射） */
    MPU6050_BUS_PB6_PB7 = 1,   /* PB6=SCL, PB7=SDA（I2C1 默认脚） */
    MPU6050_BUS_NONE     = 2    /* 未检测到传感器 */
} MPU6050_Bus_TypeDef;

/* ============================================================================
 * 加速度计满量程（g）
 * ============================================================================ */
typedef enum {
    MPU6050_ACCEL_FS_2G  = 0,   /* ±2g  */
    MPU6050_ACCEL_FS_4G  = 1,   /* ±4g  */
    MPU6050_ACCEL_FS_8G  = 2,   /* ±8g  */
    MPU6050_ACCEL_FS_16G = 3    /* ±16g */
} MPU6050_AccelFS_TypeDef;

/* ============================================================================
 * 陀螺仪满量程（°/s）
 * ============================================================================ */
typedef enum {
    MPU6050_GYRO_FS_250  = 0,   /* ±250°/s */
    MPU6050_GYRO_FS_500  = 1,   /* ±500°/s */
    MPU6050_GYRO_FS_1000 = 2,   /* ±1000°/s */
    MPU6050_GYRO_FS_2000 = 3    /* ±2000°/s */
} MPU6050_GyroFS_TypeDef;

/* ============================================================================
 * 原始数据结构
 * ============================================================================ */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature;   /* 温度原始值 */
} MPU6050_RawData_TypeDef;

/* ============================================================================
 * 校准偏移（由 MPU6050_Calibrate() 填充）
 * ============================================================================ */
typedef struct {
    int16_t accel_x_offset;
    int16_t accel_y_offset;
    int16_t accel_z_offset;
    int16_t gyro_x_offset;
    int16_t gyro_y_offset;
    int16_t gyro_z_offset;
} MPU6050_Calib_TypeDef;

/* ============================================================================
 * 函数声明
 * ============================================================================ */

/* 初始化（自动探测 PB8/PB9 再试 PB6/PB7）
 * 返回实际使用的总线模式
 */
MPU6050_Bus_TypeDef MPU6050_Init(void);

/* 检测传感器是否存在 */
uint8_t MPU6050_IsReady(void);

/* 获取当前使用的总线模式 */
MPU6050_Bus_TypeDef MPU6050_GetBus(void);

/* 获取原始数据（须先 Init） */
void MPU6050_GetRawData(MPU6050_RawData_TypeDef *data);

/* 获取去除校准偏移后的数据 */
void MPU6050_GetCalibratedData(MPU6050_RawData_TypeDef *data);

/* 校准：采集 N 次均值作为零偏
 * 采集期间传感器须保持静止
 */
void MPU6050_Calibrate(uint16_t sample_count);

/* 获取校准偏移（须先 Calibrate） */
void MPU6050_GetCalibOffset(MPU6050_Calib_TypeDef *calib);

/* 获取温度（摄氏度） */
float MPU6050_GetTemperature_C(void);

/* 设置加速度计量程 */
void MPU6050_SetAccelFS(MPU6050_AccelFS_TypeDef fs);

/* 设置陀螺仪量程 */
void MPU6050_SetGyroFS(MPU6050_GyroFS_TypeDef fs);

#endif /* __BSP_MPU6050_H__ */

/******************* (C) COPYRIGHT 2026 *****END OF FILE*****/
