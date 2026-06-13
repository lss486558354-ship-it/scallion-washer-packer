/**
 ******************************************************************************
 * HX711 24-bit ADC 称重传感器驱动
 *
 * 硬件接口：
 *   PD_SCK（时钟）→ PA4（推挽输出）
 *   DOUT（数据）  → PA5（浮空输入，下降沿触发）
 *
 * 通信协议：MCU 提供时钟脉冲，HX711 在时钟下降沿输出 24 位数据
 *   PD_SCK=LOW → DOUT 变为 LOW 表示数据就绪
 *   PD_SCK 产生 25~27 个脉冲读走数据
 *   第 25 个脉冲后：通道 A 增益 128（默认）
 *   第 26 个脉冲后：通道 B 增益 32
 *   第 27 个脉冲后：通道 A 增益 64
 *
 * 标定参数（编译时常量，掉电丢失；支持运行时一键校准）：
 *   HX711_RATIO_SCALE  — 每 raw 单位的克数（负数表示硬件反接）
 *   HX711_KNOWN_MASS_G — 校准砝码质量 g
 *   HX711_KNOWN_MASS_RAW — 放砝码时 raw 均值
 *
 * 命令（UART1）：
 *   T  → 去皮（记录当前 raw 为零位基准）
 *   R  → 取消去皮
 *   C  → 打印配置信息
 *   K<mass> → 一键校准（如 K682.5），写入 RATIO_SCALE 到 RAM
 *
 * 输出格式（与 tools/serial_plotter/main.py 正则匹配）：
 *   [HX711] Weight=X.XXX kg (X.XXX g)  raw=X d=X
 *     raw  : 原始 ADC 采样值（24 位有符号）
 *     d    : 去皮后的差值 raw - tare_raw
 ******************************************************************************
 */

#ifndef __BSP_HX711_H__
#define __BSP_HX711_H__

#include "stm32f10x.h"

/*-----------------------------------------------------------
 * 硬件配置
 *-----------------------------------------------------------*/
#define HX711_SCK_PORT    GPIOA
#define HX711_SCK_PIN     GPIO_Pin_4    /* PA4 = SCK 时钟 */
#define HX711_DOUT_PORT   GPIOA
#define HX711_DOUT_PIN    GPIO_Pin_5    /* PA5 = DOUT 数据 */

/*-----------------------------------------------------------
 * 标定参数（请根据实际砝码校准后修改）
 *-----------------------------------------------------------*/
/* 校准后填入：已知砝码克数 */
#define HX711_KNOWN_MASS_G         (682.0f)
/* 校准后填入：放砝码时的 raw 均值（已去皮差值）
 * 从CSV分析（20260422_005912）：
 *   - 682g 砝码稳定段 raw 均值 ≈ -244,000
 *   - d = raw - tare_raw，diff = -244,000 */
#define HX711_KNOWN_MASS_RAW_DIFF  (-244000L)
/* 比例系数：克/ raw。已知 m=g 时 RATIO = g / raw_diff
 * 正数 vs 负数取决于传感器接线方向，差值为负说明物体放上后 raw 下降 */
#define HX711_RATIO_SCALE          ((HX711_KNOWN_MASS_G) / ((float)(HX711_KNOWN_MASS_RAW_DIFF)))

/*-----------------------------------------------------------
 * 采样滤波
 *-----------------------------------------------------------*/
/* 滑动平均窗口大小（2 的幂次方便位运算）*/
#define HX711_AVG_SHIFT      3   /* 1<<3 = 8 点平均 */
#define HX711_AVG_MASK      ((1U << HX711_AVG_SHIFT) - 1U)

/*-----------------------------------------------------------
 * 函数接口
 *-----------------------------------------------------------*/
/* 初始化 GPIO */
void HX711_Init(void);

/* 读取原始采样值（有符号 24 位，返回 int32_t） */
int32_t HX711_ReadRaw(void);

/* 读取滤波后值（8 点滑动平均） */
int32_t HX711_ReadFiltered(void);

/* 去皮：记录当前 raw 均值为零位基准 */
void HX711_Tare(void);

/* 取消去皮 */
void HX711_ResetTare(void);

/* 获取当前去皮差值 d = filtered - tare */
int32_t HX711_GetDiff(void);

/* 差值转克数（返回 float g） */
float HX711_DiffToGram(int32_t diff);

/* 获取当前重量克数（已去皮，已转换） */
float HX711_GetWeightGram(void);

/* 获取当前重量千克数（已去皮，已转换） */
float HX711_GetWeightKg(void);

/* 运行时一键校准（需先放上砝码去皮）
 * mass_g : 标准砝码质量（克）
 * 内部自动更新 g_hx711_ratio_scale，下次调用即生效 */
void HX711_OneClickCal(float mass_g);

/* 打印当前配置（C 命令） */
void HX711_PrintConfig(void);

/* 打印当前采样（用于调试） */
void HX711_PrintSample(void);

/* UART1 字节入口（主循环将收到的字节喂给本函数） */
void HX711_OnByte(uint8_t ch);

/* 测试任务（可由主循环调用，持续打印重量） */
void HX711_TestTask(void);

/*-----------------------------------------------------------
 * 外部变量（供 main.c 直接读取）
 *-----------------------------------------------------------*/
extern volatile int32_t  g_hx711_raw;
extern volatile int32_t  g_hx711_filtered;
extern volatile int32_t  g_hx711_tare_raw;
extern volatile int32_t  g_hx711_diff;
extern volatile float    g_hx711_ratio_scale;  /* 运行时可写 */
extern volatile uint8_t  g_hx711_tare_valid;

#endif

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
