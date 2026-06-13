/**
 ******************************************************************************
 * 舵机对象化封装实现
 * 作者：电控组
 * 日期：2026-03-23
 *
 * 舵机控制：50Hz PWM，周期20ms
 *   0度对应脉宽0.5ms，90度对应1.5ms，180度对应2.5ms
 ******************************************************************************
 */

#include "bsp_servo_obj.h"
#include "stm32f10x_tim.h"

/* STM32固件库中 TIM_CC1E/2E/3E/4E 不存在于 stm32f10x_tim.h，此处直接定义 */
#define TIM_CC1E  ((uint16_t)0x0001)
#define TIM_CC2E  ((uint16_t)0x0010)
#define TIM_CC3E  ((uint16_t)0x0100)
#define TIM_CC4E  ((uint16_t)0x1000)

    /* 舵机配置表 - TIM4通道 */
    typedef struct {
        TIM_TypeDef* tim;
        uint32_t tim_clock;
        uint8_t tim_channel;
        GPIO_PortPin_TypeDef pin;
    } Servo_Config_TypeDef;

    static const Servo_Config_TypeDef g_servo_config[SERVO_MAX] = {
        /* TIM4_CH1 - PB6 */
        {TIM4, RCC_APB1Periph_TIM4, TIM_Channel_1, PB6},
        /* TIM4_CH2 - PB7 */
        {TIM4, RCC_APB1Periph_TIM4, TIM_Channel_2, PB7},
        /* TIM4_CH3 - PB8 (避免与步进电机TIM1冲突) */
        {TIM4, RCC_APB1Periph_TIM4, TIM_Channel_3, PB8},
        /* TIM4_CH4 - PB9 (避免与步进电机TIM1冲突) */
        {TIM4, RCC_APB1Periph_TIM4, TIM_Channel_4, PB9},
    };

/* 定时器周期参数 - 72MHz / 72 = 1MHz, ARR = 20000 -> 20ms */
#define SERVO_PSC          72
#define SERVO_ARR          20000
#define SERVO_PULSE_0deg   500   /* 0.5ms */
#define SERVO_PULSE_90deg  1500  /* 1.5ms */
#define SERVO_PULSE_180deg 2500  /* 2.5ms */

/* 内部函数声明 */
static void SERVO_InitImpl(SERVO_t* self);
static void SERVO_SetAngleImpl(SERVO_t* self, float angle);
static void SERVO_SetAngleAbsoluteImpl(SERVO_t* self, float angle);
static float SERVO_GetAngleImpl(SERVO_t* self);
static void SERVO_EnableImpl(SERVO_t* self, uint8_t en);

/* 设置舵机CCR值 */
static void SERVO_SetCCR(SERVO_t* self, uint16_t ccr)
{
    switch (self->pins.tim_channel) {
        case TIM_Channel_1:
            self->pins.tim->CCR1 = ccr;
            break;
        case TIM_Channel_2:
            self->pins.tim->CCR2 = ccr;
            break;
        case TIM_Channel_3:
            self->pins.tim->CCR3 = ccr;
            break;
        case TIM_Channel_4:
            self->pins.tim->CCR4 = ccr;
            break;
    }
}

/* 初始化单个舵机 */
static void SERVO_InitSingle(uint8_t id)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    Servo_Config_TypeDef* cfg = (Servo_Config_TypeDef*)&g_servo_config[id];

    /* 使能定时器时钟（TIM1在APB2，其他在APB1） */
    if (cfg->tim == TIM1) {
        RCC_APB2PeriphClockCmd(cfg->tim_clock, ENABLE);
    } else {
        RCC_APB1PeriphClockCmd(cfg->tim_clock, ENABLE);
    }

    /* 配置GPIO为复用功能 */
    GPIO_Set(cfg->pin, MODE_AF_PP_50M);

    /* 定时器基础配置 */
    TIM_TimeBaseStructure.TIM_Period = SERVO_ARR - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = SERVO_PSC - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(cfg->tim, &TIM_TimeBaseStructure);

    /* PWM输出配置 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; /* 先关闭 */
    TIM_OCInitStructure.TIM_Pulse = SERVO_PULSE_90deg;              /* 默认90度 */
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    /* 根据通道配置 */
    switch (cfg->tim_channel) {
        case TIM_Channel_1:
            TIM_OC1Init(cfg->tim, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(cfg->tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(cfg->tim, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(cfg->tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(cfg->tim, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(cfg->tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(cfg->tim, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(cfg->tim, TIM_OCPreload_Enable);
            break;
    }

    TIM_ARRPreloadConfig(cfg->tim, ENABLE);

    /* TIM1需要使能主输出 */
    if (cfg->tim == TIM1) {
        TIM_CtrlPWMOutputs(cfg->tim, ENABLE);
    }

    TIM_Cmd(cfg->tim, ENABLE);
}

/* =============================================================================
 * 对象方法实现
 * ============================================================================= */

static void SERVO_InitImpl(SERVO_t* self)
{
    SERVO_InitSingle((uint8_t)self->id);
}

static void SERVO_SetAngleImpl(SERVO_t* self, float angle)
{
    uint16_t ccr;
    float scale;

    /* 限制角度范围 */
    if (angle < self->min_angle) angle = self->min_angle;
    if (angle > self->max_angle) angle = self->max_angle;

    self->target_angle = angle;

    /* 线性插值计算CCR值 */
    scale = angle / 180.0f;
    ccr = (uint16_t)(SERVO_PULSE_0deg + scale * (SERVO_PULSE_180deg - SERVO_PULSE_0deg));

    SERVO_SetCCR(self, ccr);
    self->current_angle = angle;
}

static void SERVO_SetAngleAbsoluteImpl(SERVO_t* self, float angle)
{
    self->SetAngle(self, angle);
}

static float SERVO_GetAngleImpl(SERVO_t* self)
{
    return self->current_angle;
}

static void SERVO_EnableImpl(SERVO_t* self, uint8_t en)
{
    switch (self->pins.tim_channel) {
        case TIM_Channel_1:
            if (en) self->pins.tim->CCER |= TIM_CC1E;
            else self->pins.tim->CCER &= ~TIM_CC1E;
            break;
        case TIM_Channel_2:
            if (en) self->pins.tim->CCER |= TIM_CC2E;
            else self->pins.tim->CCER &= ~TIM_CC2E;
            break;
        case TIM_Channel_3:
            if (en) self->pins.tim->CCER |= TIM_CC3E;
            else self->pins.tim->CCER &= ~TIM_CC3E;
            break;
        case TIM_Channel_4:
            if (en) self->pins.tim->CCER |= TIM_CC4E;
            else self->pins.tim->CCER &= ~TIM_CC4E;
            break;
    }
}

/* =============================================================================
 * 舵机对象实例
 * ============================================================================= */

SERVO_t SERVO[SERVO_MAX];

/* 初始化所有舵机 */
void SERVO_InitAll(void)
{
    uint8_t i;
    for (i = 0; i < SERVO_MAX; i++) {
        /* 基本属性 */
        SERVO[i].id = (Servo_ID_TypeDef)i;
        SERVO[i].current_angle = 90.0f;
        SERVO[i].target_angle = 90.0f;
        SERVO[i].min_angle = 0.0f;
        SERVO[i].max_angle = 180.0f;

        /* 引脚配置 */
        SERVO[i].pins.tim = g_servo_config[i].tim;
        SERVO[i].pins.tim_channel = g_servo_config[i].tim_channel;
        SERVO[i].pins.pin = g_servo_config[i].pin;

        /* PWM参数 */
        SERVO[i].pwm_min = SERVO_PULSE_0deg;
        SERVO[i].pwm_max = SERVO_PULSE_180deg;
        SERVO[i].pwm_center = SERVO_PULSE_90deg;

        /* 绑定函数指针 */
        SERVO[i].Init = SERVO_InitImpl;
        SERVO[i].SetAngle = SERVO_SetAngleImpl;
        SERVO[i].SetAngleAbsolute = SERVO_SetAngleAbsoluteImpl;
        SERVO[i].GetAngle = SERVO_GetAngleImpl;
        SERVO[i].Enable = SERVO_EnableImpl;

        /* 初始化硬件 */
        SERVO[i].Init(&SERVO[i]);
    }
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
