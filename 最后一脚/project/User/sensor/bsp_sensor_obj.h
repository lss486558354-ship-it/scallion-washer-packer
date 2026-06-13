/**
 ******************************************************************************
 * 传感器对象化封装 - ADC采集
 *
 * ============================================================================
 * 使用示例：
 *
 * // 1. 初始化
 * SENSOR_InitAll();                        // 初始化所有传感器
 *
 * // 2. 使用快捷宏（推荐）
 * float pressure = ADC_PRESSURE_GetValue();  // 获取压力值
 * float water = ADC_WATER_GetValue();       // 获取水位值
 * float weight = ADC_WEIGHT_GetValue();     // 获取重量值
 *
 * // 3. 获取原始ADC值（调试用）
 * uint16_t raw = ADC_PRESSURE_GetRaw();     // 获取原始ADC值(0~4095)
 *
 * // 4. 使用对象方法
 * float value = SENSOR[SENSOR_PRESSURE].GetValue(&SENSOR[SENSOR_PRESSURE]);
 *
 * // 5. 校准
 * ADC_PRESSURE_Calibrate(0.0f, 100.0f);   // 设置零点和满量程
 * ADC_PRESSURE_AutoZero();                  // 自动校零（当前值设为零点）
 *
 * // 6. 滤波设置
 * SENSOR[SENSOR_PRESSURE].SetFilterAlpha(&SENSOR[SENSOR_PRESSURE], 0.1f);
 * // alpha越小越平滑，范围0.0~1.0
 *
 * ============================================================================
 * Sensor_ID_TypeDef 类型说明：
 *   SENSOR_PRESSURE = 0    - 压力传感器，PB0，ADC1_IN8
 *   SENSOR_WATER_LEVEL = 1  - 水位传感器，PB1，ADC1_IN9
 *   SENSOR_WEIGHT = 2       - 重量传感器，PA4，ADC1_IN4
 *   SENSOR_MAX = 3          - 传感器总数
 *
 * ============================================================================
 * Sensor_Type_TypeDef 类型说明：
 *   SENSOR_TYPE_VOLTAGE = 0   - 电压输出型(0-3.3V)
 *   SENSOR_TYPE_CURRENT = 1   - 电流输出型(4-20mA)
 *   SENSOR_TYPE_RESISTIVE = 2  - 电阻型
 *
 * ============================================================================
 * 参数说明：
 * - GetValue()：返回float类型的物理量值（已滤波、校准）
 * - GetRawValue()：返回uint16_t类型的原始ADC值（0~4095）
 * - zero_offset：零点偏移量（用于校准）
 * - full_scale：满量程值（用于单位转换）
 * - alpha：滤波系数，范围0.0~1.0，越小越平滑
 *
 * ============================================================================
 * ADC参数：
 * - 分辨率：12位（0~4095）
 * - 参考电压：3.3V
 * - 采样时间：55.5周期
 *
 * ============================================================================
 * 作者：工程训练中心116电控组
 * 日期：2026-03-24
 ******************************************************************************
 */

#ifndef __BSP_SENSOR_OBJ_H__
#define __BSP_SENSOR_OBJ_H__

#include "stm32f10x.h"
#include "bsp_gpio.h"
#include "bsp_hx711.h"

/* 前向声明 - 统一struct标签与typedef名 */
typedef struct _SENSOR_t SENSOR_t;

/* 传感器通道编号
 *   SENSOR_PRESSURE = 0    - 压力传感器，PB0，ADC1_IN8
 *   SENSOR_WATER_LEVEL = 1  - 水位传感器，PB1，ADC1_IN9
 *   SENSOR_WEIGHT = 2       - 重量传感器，PA4，ADC1_IN4
 *   SENSOR_MAX = 3          - 传感器总数
 */
typedef enum {
    SENSOR_PRESSURE = 0,
    SENSOR_WATER_LEVEL = 1,
    SENSOR_MAX = 2
    /* SENSOR_WEIGHT = 2 已禁用，重量改由 HX711 模块接管（PA4/PA5） */
} Sensor_ID_TypeDef;

/* 传感器类型
 *   SENSOR_TYPE_VOLTAGE = 0   - 电压输出型(0-3.3V)
 *   SENSOR_TYPE_CURRENT = 1   - 电流输出型(4-20mA)
 *   SENSOR_TYPE_RESISTIVE = 2  - 电阻型
 */
typedef enum {
    SENSOR_TYPE_VOLTAGE = 0,
    SENSOR_TYPE_CURRENT = 1,
    SENSOR_TYPE_RESISTIVE = 2
} Sensor_Type_TypeDef;

/* 传感器引脚配置 */
typedef struct {
    GPIO_PortPin_TypeDef pin;   /* ADC引脚 */
    uint8_t adc_channel;        /* ADC通道号 */
} Sensor_Pins_TypeDef;

/* 传感器对象 */
struct _SENSOR_t {
    Sensor_ID_TypeDef id;       /* 传感器编号 */
    Sensor_Type_TypeDef type;    /* 传感器类型 */

    /* 原始值 */
    uint16_t raw_value;         /* 原始ADC值（0~4095） */

    /* 处理后的值 */
    float filtered_value;        /* 滤波后的值 */
    float last_value;            /* 上一次的值 */

    /* 校准参数 */
    float zero_offset;         /* 零点偏移 */
    float full_scale;           /* 满量程值 */
    float scale_factor;         /* 比例因子 */

    /* 滤波参数 */
    float alpha;                /* 滤波系数（0~1，越小越平滑） */
    uint16_t sample_count;      /* 采样计数 */
    uint32_t sample_sum;        /* 采样累加 */

    /* 引脚配置 */
    Sensor_Pins_TypeDef pins;

    /* 函数指针 */
    void (*Init)(SENSOR_t* self);
    uint16_t (*GetRawValue)(SENSOR_t* self);
    float (*GetValue)(SENSOR_t* self);
    void (*Calibrate)(SENSOR_t* self, float zero_offset, float full_scale);
    void (*AutoZero)(SENSOR_t* self);
    void (*SetFilterAlpha)(SENSOR_t* self, float alpha);
};

/* 外部声明 */
extern SENSOR_t SENSOR[SENSOR_MAX];

/* 初始化所有传感器 */
void SENSOR_InitAll(void);

/* =============================================================================
 * 快捷宏定义
 * ============================================================================= */

/* 获取原始ADC值（调试用） */
#define ADC_PRESSURE_GetRaw()   SENSOR[SENSOR_PRESSURE].GetRawValue(&SENSOR[SENSOR_PRESSURE])
#define ADC_WATER_GetRaw()      SENSOR[SENSOR_WATER_LEVEL].GetRawValue(&SENSOR[SENSOR_WATER_LEVEL])
#define ADC_WEIGHT_GetRaw()     (0)   /* 已由 HX711 接管，无 ADC Raw */
#define ADC_WEIGHT_GetValue()   (HX711_GetWeightGram())  /* 重定向到 HX711 */

/* 获取处理后的值（推荐使用） */
#define ADC_PRESSURE_GetValue() SENSOR[SENSOR_PRESSURE].GetValue(&SENSOR[SENSOR_PRESSURE])
#define ADC_WATER_GetValue()    SENSOR[SENSOR_WATER_LEVEL].GetValue(&SENSOR[SENSOR_WATER_LEVEL])

/* 校准快捷宏 */
#define ADC_PRESSURE_Calibrate(zero, full)  SENSOR[SENSOR_PRESSURE].Calibrate(&SENSOR[SENSOR_PRESSURE], zero, full)
#define ADC_WATER_Calibrate(zero, full)     SENSOR[SENSOR_WATER_LEVEL].Calibrate(&SENSOR[SENSOR_WATER_LEVEL], zero, full)
#define ADC_WEIGHT_Calibrate(zero, full)    ((void)0)  /* 已由 HX711_Tare() 替代 */

/* 自动校零 */
#define ADC_PRESSURE_AutoZero()  SENSOR[SENSOR_PRESSURE].AutoZero(&SENSOR[SENSOR_PRESSURE])
#define ADC_WATER_AutoZero()     SENSOR[SENSOR_WATER_LEVEL].AutoZero(&SENSOR[SENSOR_WATER_LEVEL])
#define ADC_WEIGHT_AutoZero()    HX711_Tare()  /* 重定向到 HX711 去皮 */

#endif /* __BSP_SENSOR_OBJ_H__ */
