/**
 ******************************************************************************
 * 传感器对象化封装实现 - ADC采集
 * 作者：电控组
 * 日期：2026-03-23
 *
 * ADC资源配置：
 *   ADC1_IN8  - PB0 (压力传感器)
 *   ADC1_IN9  - PB1 (水位传感器)
 *   ADC1_IN4  - PA4 (重量传感器)
 *
 * 使用12位ADC，参考电压3.3V
 ******************************************************************************
 */

#include "bsp_sensor_obj.h"
#include "stm32f10x_adc.h"
#include <stdio.h>

/* =============================================================================
 * 内部宏定义
 * ============================================================================= */

/* ADC参考电压 */
#define ADC_VREF          3.3f

/* ADC最大分辨率 (12位) */
#define ADC_MAX_VALUE     4095

/* =============================================================================
 * 传感器配置表
 * ============================================================================= */

typedef struct {
    GPIO_PortPin_TypeDef pin;
    uint8_t adc_channel;
    uint8_t adc_rank;
} Sensor_Config_TypeDef;

static const Sensor_Config_TypeDef g_sensor_config[SENSOR_MAX] = {
    /* SENSOR_PRESSURE - PB0, ADC1_IN8 */
    {PB0, ADC_Channel_8, ADC_Channel_8},
    /* SENSOR_WATER_LEVEL - PB1, ADC1_IN9 */
    {PB1, ADC_Channel_9, ADC_Channel_9},
};

/* =============================================================================
 * 内部函数声明
 * ============================================================================= */

static void SENSOR_InitImpl(SENSOR_t* self);
static uint16_t SENSOR_GetRawValueImpl(SENSOR_t* self);
static float SENSOR_GetValueImpl(SENSOR_t* self);
static void SENSOR_CalibrateImpl(SENSOR_t* self, float zero_offset, float full_scale);
static void SENSOR_AutoZeroImpl(SENSOR_t* self);
static void SENSOR_SetFilterAlphaImpl(SENSOR_t* self, float alpha);

/* =============================================================================
 * ADC硬件初始化
 * ============================================================================= */

static void SENSOR_ADC_HwInit(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t i;

    /* 使能ADC1和GPIO时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /* 配置所有传感器引脚为模拟输入 */
    for (i = 0; i < SENSOR_MAX; i++) {
        /* 使能GPIO时钟 */
        if (g_sensor_config[i].pin.port == GPIOA) {
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        } else if (g_sensor_config[i].pin.port == GPIOB) {
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        }

        /* 配置为模拟输入 */
        GPIO_InitStructure.GPIO_Pin = g_sensor_config[i].pin.pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_Init(g_sensor_config[i].pin.port, &GPIO_InitStructure);
    }

    /* ADC配置 */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = SENSOR_MAX;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* 设置ADC采样通道和采样时间 */
    for (i = 0; i < SENSOR_MAX; i++) {
        ADC_RegularChannelConfig(ADC1, g_sensor_config[i].adc_channel, i + 1, ADC_SampleTime_55Cycles5);
    }

    /* 使能ADC */
    ADC_Cmd(ADC1, ENABLE);

    /* ADC校准 */
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

/* =============================================================================
 * 读取单个ADC通道
 * ============================================================================= */

static uint16_t SENSOR_ADC_ReadChannel(uint8_t channel)
{
    /* 配置ADC通道 */
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_55Cycles5);

    /* 启动转换 */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    /* 等待转换完成 */
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    /* 返回结果 */
    return ADC_GetConversionValue(ADC1);
}

/* =============================================================================
 * 对象方法实现
 * ============================================================================= */

static void SENSOR_InitImpl(SENSOR_t* self)
{
    uint8_t id = (uint8_t)self->id;

    /* 设置引脚配置 */
    self->pins.pin = g_sensor_config[id].pin;
    self->pins.adc_channel = g_sensor_config[id].adc_channel;

    /* 设置默认校准参数 */
    self->zero_offset = 0.0f;
    self->full_scale = 100.0f;  /* 默认100% */
    self->scale_factor = 1.0f;

    /* 设置默认滤波参数 */
    self->alpha = 0.1f;
    self->filtered_value = 0.0f;
    self->last_value = 0.0f;

    /* 清零采样统计 */
    self->sample_count = 0;
    self->sample_sum = 0;

    /* 根据传感器类型设置默认比例 */
    switch (self->type) {
        case SENSOR_TYPE_VOLTAGE:
            self->scale_factor = ADC_VREF / ADC_MAX_VALUE;
            break;
        case SENSOR_TYPE_CURRENT:
            self->scale_factor = 20.0f / ADC_MAX_VALUE;  /* 4-20mA */
            break;
        case SENSOR_TYPE_RESISTIVE:
            self->scale_factor = 1000.0f / ADC_MAX_VALUE; /* 0-1000欧姆 */
            break;
    }
}

static uint16_t SENSOR_GetRawValueImpl(SENSOR_t* self)
{
    uint16_t raw;
    uint8_t id = (uint8_t)self->id;

    /* 读取ADC */
    raw = SENSOR_ADC_ReadChannel(g_sensor_config[id].adc_channel);

    /* 更新原始值 */
    self->raw_value = raw;

    return raw;
}

static float SENSOR_GetValueImpl(SENSOR_t* self)
{
    float voltage;
    float physical_value;

    /* 读取原始值 */
    (void)self->GetRawValue(self);

    /* 转换为电压 */
    voltage = (float)self->raw_value * (ADC_VREF / ADC_MAX_VALUE);

    /* 根据传感器类型转换为物理量 */
    switch (self->type) {
        case SENSOR_TYPE_VOLTAGE:
            /* 电压型传感器: 直接转换为百分比或实际物理量 */
            physical_value = voltage * self->full_scale / ADC_VREF;
            break;

        case SENSOR_TYPE_CURRENT:
            /* 电流型传感器: (4-20mA) */
            /* 电流 = 4 + (raw / 4095) * 16 mA */
            physical_value = 4.0f + (float)self->raw_value * 16.0f / ADC_MAX_VALUE;
            physical_value = (physical_value - 4.0f) * self->full_scale / 16.0f;
            break;

        case SENSOR_TYPE_RESISTIVE:
        default:
            /* 电阻型或默认 */
            physical_value = voltage * 1000.0f / (ADC_VREF - voltage);
            break;
    }

    /* 应用校准偏移 */
    physical_value -= self->zero_offset;

    /* 一阶低通滤波 */
    self->filtered_value = self->alpha * physical_value + (1.0f - self->alpha) * self->last_value;
    self->last_value = self->filtered_value;

    return self->filtered_value;
}

static void SENSOR_CalibrateImpl(SENSOR_t* self, float zero_offset, float full_scale)
{
    self->zero_offset = zero_offset;
    self->full_scale = full_scale;
}

static void SENSOR_AutoZeroImpl(SENSOR_t* self)
{
    uint32_t sum = 0;
    uint16_t i;
    float avg_value;

    /* 采样10次求平均 */
    for (i = 0; i < 10; i++) {
        sum += self->GetRawValue(self);
    }

    avg_value = (float)sum / 10.0f;

    /* 将当前值设为零点 */
    self->zero_offset = avg_value * (ADC_VREF / ADC_MAX_VALUE) * self->full_scale / ADC_VREF;
}

static void SENSOR_SetFilterAlphaImpl(SENSOR_t* self, float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    self->alpha = alpha;
}

/* =============================================================================
 * 传感器对象实例
 * ============================================================================= */

SENSOR_t SENSOR[SENSOR_MAX];

/* 初始化所有传感器 */
void SENSOR_InitAll(void)
{
    uint8_t i;

    /* 先初始化ADC硬件 */
    SENSOR_ADC_HwInit();

    /* 初始化每个传感器对象 */
    for (i = 0; i < SENSOR_MAX; i++) {
        /* 基本属性 */
        SENSOR[i].id = (Sensor_ID_TypeDef)i;

        /* 默认传感器类型 */
        switch (i) {
            case SENSOR_PRESSURE:
                SENSOR[i].type = SENSOR_TYPE_VOLTAGE;
                break;
            case SENSOR_WATER_LEVEL:
                SENSOR[i].type = SENSOR_TYPE_VOLTAGE;
                break;
            default:
                SENSOR[i].type = SENSOR_TYPE_VOLTAGE;
                break;
        }

        /* 绑定函数指针 */
        SENSOR[i].Init = SENSOR_InitImpl;
        SENSOR[i].GetRawValue = SENSOR_GetRawValueImpl;
        SENSOR[i].GetValue = SENSOR_GetValueImpl;
        SENSOR[i].Calibrate = SENSOR_CalibrateImpl;
        SENSOR[i].AutoZero = SENSOR_AutoZeroImpl;
        SENSOR[i].SetFilterAlpha = SENSOR_SetFilterAlphaImpl;

        /* 初始化对象 */
        SENSOR[i].Init(&SENSOR[i]);
    }
}

/* =============================================================================
 * 便捷函数（非对象方法）
 * ============================================================================= */

/**
 * 读取所有传感器并格式化输出
 * 返回字符串格式: "P:xx.x W:xx.x L:xx.x"
 */
void SENSOR_ReportAll(char* buffer, uint16_t buffer_size)
{
    float pressure, water, weight;

    pressure = SENSOR[SENSOR_PRESSURE].GetValue(&SENSOR[SENSOR_PRESSURE]);
    water = SENSOR[SENSOR_WATER_LEVEL].GetValue(&SENSOR[SENSOR_WATER_LEVEL]);
    weight = 0.0f;  /* SENSOR_WEIGHT 已由 HX711 模块接管 */

    (void)snprintf(buffer, buffer_size, "P:%.1f W:%.1f L:%.1f",
                   pressure, water, weight);
}

/**
 * 获取传感器原始ADC值（调试用）
 */
void SENSOR_ReportRaw(char* buffer, uint16_t buffer_size)
{
    uint16_t p_raw, w_raw, l_raw;

    p_raw = SENSOR[SENSOR_PRESSURE].GetRawValue(&SENSOR[SENSOR_PRESSURE]);
    w_raw = SENSOR[SENSOR_WATER_LEVEL].GetRawValue(&SENSOR[SENSOR_WATER_LEVEL]);
    l_raw = 0;  /* SENSOR_WEIGHT 已由 HX711 模块接管，无原始 ADC 值 */

    (void)snprintf(buffer, buffer_size, "P:%04d W:%04d L:%04d",
                   p_raw, w_raw, l_raw);
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
