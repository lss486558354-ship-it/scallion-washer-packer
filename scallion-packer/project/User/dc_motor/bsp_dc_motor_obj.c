/**
 *******************************************************************************
 * 直流电机驱动（切膜 + 通用直流）
 *
 * VET6版本
 *
 * 引脚定义：
 *   DCM1: PC9 (TIM8_CH4) PWM(ENA) + PD0/PD1 IN1/IN2 硬接 3.3V/GND 方向固定
 *   DCM2: 切膜电机 PC7 (TIM8_CH2) PWM + PD8/PD9 IN1/IN2（L298N）
 *   DCM3: PC8 (TIM8_CH3) PWM + PD4/PD5 IN1/IN2
 *   DCM4: PC6 (TIM8_CH1) PWM + PD6/PD7 IN1/IN2
 *
 * TIM8参数: 72MHz, PSC=72 -> 1MHz, ARR=100 -> 10kHz PWM
 *******************************************************************************
 */
#include "bsp_dc_motor_obj.h"
#include "stm32f10x_tim.h"

/* STM32缺少的 TIM_CC1E/2E/3E/4E 宏定义 */
#define TIM_CC1E  ((uint16_t)0x0001)
#define TIM_CC2E  ((uint16_t)0x0010)
#define TIM_CC3E  ((uint16_t)0x0100)
#define TIM_CC4E  ((uint16_t)0x1000)

/* =============================================================================
 * 常量定义
 * ============================================================================= */

/* TIM8参数 */
#define DCM_TIM              TIM8
#define DCM_TIM_RCC         RCC_APB2Periph_TIM8
#define DCM_PSC             72        /* 72MHz / 72 = 1MHz */
#define DCM_ARR             100       /* 10kHz PWM频率 */
#define DCM_TIM_FREQ        (72000000 / DCM_PSC)  /* 1MHz */

/* SysTick系统节拍，外部声明，1ms递增一次 */
extern volatile uint32_t g_tick_ms;
#define DCM_GetTick()       g_tick_ms

/* =============================================================================
 * 配置结构体
 * ============================================================================= */

typedef struct {
    GPIO_PortPin_TypeDef pwm_pin;
    GPIO_PortPin_TypeDef in1_pin;
    GPIO_PortPin_TypeDef in2_pin;
    TIM_TypeDef* tim;
    uint32_t tim_clock;
    uint8_t tim_channel;
} DCM_Config_TypeDef;

static const DCM_Config_TypeDef g_dcm_config[DCM_MAX] = {
    /* DCM_1: PC9 PWM(ENA), IN1/IN2 硬接 3.3V/GND 方向固定 */
    {PC9, PD0, PD1, TIM8, RCC_APB2Periph_TIM8, TIM_Channel_4},
    /* DCM_2: 切膜电机 PC7 PWM, PD8/PD9 方向（L298N IN1/IN2） */
    {PC7, PD8, PD9, TIM8, RCC_APB2Periph_TIM8, TIM_Channel_2},
    /* DCM_3: PC8 PWM, PD4/PD5 方向 */
    {PC8, PD4, PD5, TIM8, RCC_APB2Periph_TIM8, TIM_Channel_3},
    /* DCM_4: PC6 PWM, PD6/PD7 方向 */
    {PC6, PD6, PD7, TIM8, RCC_APB2Periph_TIM8, TIM_Channel_1},
};

/* =============================================================================
 * 内部函数声明
 * ============================================================================= */

static void DCM_InitImpl(DCM_t* self);
static void DCM_RunImpl(DCM_t* self, uint8_t duty, Motor_Direction_TypeDef dir);
static void DCM_RunRevolutionsImpl(DCM_t* self, float revolutions, uint8_t duty, Motor_Direction_TypeDef dir);
static void DCM_StopImpl(DCM_t* self);
static void DCM_SetCalibrationImpl(DCM_t* self, float ms_per_rev, uint8_t calib_duty);
static uint8_t DCM_IsBusyImpl(DCM_t* self);
static void DCM_ProcessImpl(DCM_t* self);

/* =============================================================================
 * PWM占空比设置
 * ============================================================================= */

static void DCM_SetCCR(DCM_t* self, uint16_t ccr)
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

/* =============================================================================
 * 方向设置
 * ============================================================================= */

static void DCM_SetDirection(DCM_t* self, Motor_Direction_TypeDef dir)
{
    if (dir == DIR_FORWARD) {
        /* 正转: IN1=1, IN2=0 */
        GPIO_SetHigh(self->pins.in1_pin);
        GPIO_SetLow(self->pins.in2_pin);
    } else {
        /* 反转: IN1=0, IN2=1 */
        GPIO_SetLow(self->pins.in1_pin);
        GPIO_SetHigh(self->pins.in2_pin);
    }
}

/* =============================================================================
 * TIM8硬件初始化
 * ============================================================================= */

static void DCM_TIM8_HwInit(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    /* 使能TIM8时钟 */
    RCC_APB2PeriphClockCmd(DCM_TIM_RCC, ENABLE);

    /* 初始化时基单元 */
    TIM_TimeBaseStructure.TIM_Period = DCM_ARR - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = DCM_PSC - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(DCM_TIM, &TIM_TimeBaseStructure);

    /* 初始化全部4路PWM输出 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;  /* 先禁止 */
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(DCM_TIM, &TIM_OCInitStructure);
    TIM_OC2Init(DCM_TIM, &TIM_OCInitStructure);
    TIM_OC3Init(DCM_TIM, &TIM_OCInitStructure);
    TIM_OC4Init(DCM_TIM, &TIM_OCInitStructure);

    /* 使能预装载 */
    TIM_OC1PreloadConfig(DCM_TIM, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(DCM_TIM, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(DCM_TIM, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(DCM_TIM, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(DCM_TIM, ENABLE);

    /* TIM8高级定时器必须使能PWM输出主开关 */
    TIM_CtrlPWMOutputs(DCM_TIM, ENABLE);

    /* PC9(ENA) 由 L298N ENA使用，TIM8输出PWM由main或其他地方控制 */
    DCM_TIM->CCR4 = 0;

    TIM_Cmd(DCM_TIM, ENABLE);
}

/* =============================================================================
 * 电机控制实现
 * ============================================================================= */

static void DCM_InitImpl(DCM_t* self)
{
    uint8_t id = (uint8_t)self->id;
    DCM_Config_TypeDef* cfg = (DCM_Config_TypeDef*)&g_dcm_config[id];

    /* 复制配置 */
    self->pins.pwm_pin = cfg->pwm_pin;
    self->pins.in1_pin = cfg->in1_pin;
    self->pins.in2_pin = cfg->in2_pin;
    self->pins.tim = cfg->tim;
    self->pins.tim_channel = cfg->tim_channel;

    /* DCM_1 的 ENA 脚 PC9由main直接控制PWM输出，其他复用推挽 */
    GPIO_Set(cfg->pwm_pin, (id == 0) ? MODE_OUT_PP_50M : MODE_AF_PP_50M);
    GPIO_Set(cfg->in1_pin, MODE_OUT_PP_50M);  /* 方向引脚1 */
    GPIO_Set(cfg->in2_pin, MODE_OUT_PP_50M);  /* 方向引脚2 */

    /* 初始状态：占空比0，方向全低（制动） */
    DCM_SetCCR(self, 0);
    GPIO_SetLow(cfg->in1_pin);
    GPIO_SetLow(cfg->in2_pin);

    /* 标定参数默认值 */
    self->ms_per_revolution = 1000.0f;  /* 默认1圈1000ms */
    self->calib_duty = 100;

    /* 初始状态 */
    self->state = DCM_STATE_IDLE;
    self->duty_percent = 0;
}

static void DCM_RunImpl(DCM_t* self, uint8_t duty, Motor_Direction_TypeDef dir)
{
    uint16_t ccr;

    /* 占空比0视为停止 */
    if (duty == 0) {
        self->Stop(self);
        return;
    }
    if (duty > 100) duty = 100;

    self->duty_percent = duty;
    self->direction = dir;

    /* 设置方向 */
    DCM_SetDirection(self, dir);

    /* 计算并设置PWM占空比CCR值 */
    ccr = (uint16_t)((uint32_t)duty * DCM_ARR / 100);
    DCM_SetCCR(self, ccr);

    /* 使能PWM输出通道 */
    switch (self->pins.tim_channel) {
        case TIM_Channel_1:
            DCM_TIM->CCER |= TIM_CC1E;
            break;
        case TIM_Channel_2:
            DCM_TIM->CCER |= TIM_CC2E;
            break;
        case TIM_Channel_3:
            DCM_TIM->CCER |= TIM_CC3E;
            break;
        case TIM_Channel_4:
            DCM_TIM->CCER |= TIM_CC4E;
            break;
    }

    self->state = DCM_STATE_RUNNING;
}

static void DCM_RunRevolutionsImpl(DCM_t* self, float revolutions, uint8_t duty, Motor_Direction_TypeDef dir)
{
    uint32_t current_tick;
    float time_ms;

    /* 计算转动指定圈数需要的时间 */
    /* time = revolutions * ms_per_rev * (calib_duty / duty) */
    if (duty == 0 || self->ms_per_revolution <= 0) {
        return;
    }
    time_ms = revolutions * self->ms_per_revolution * ((float)self->calib_duty / (float)duty);
    if (time_ms < 1.0f) time_ms = 1.0f;

    /* 记录结束时刻 */
    current_tick = DCM_GetTick();
    self->end_tick = current_tick + (uint32_t)time_ms;

    /* 启动电机 */
    self->Run(self, duty, dir);
    self->state = DCM_STATE_TIMED;
}

static void DCM_StopImpl(DCM_t* self)
{
    /* 关闭PWM输出通道 */
    switch (self->pins.tim_channel) {
        case TIM_Channel_1:
            DCM_TIM->CCER &= ~TIM_CC1E;
            break;
        case TIM_Channel_2:
            DCM_TIM->CCER &= ~TIM_CC2E;
            break;
        case TIM_Channel_3:
            DCM_TIM->CCER &= ~TIM_CC3E;
            break;
        case TIM_Channel_4:
            DCM_TIM->CCER &= ~TIM_CC4E;
            break;
    }

    /* 方向引脚全低（制动） */
    GPIO_SetLow(self->pins.in1_pin);
    GPIO_SetLow(self->pins.in2_pin);

    self->state = DCM_STATE_IDLE;
    self->duty_percent = 0;
}

static void DCM_SetCalibrationImpl(DCM_t* self, float ms_per_rev, uint8_t calib_duty)
{
    self->ms_per_revolution = ms_per_rev;
    self->calib_duty = calib_duty;
}

static uint8_t DCM_IsBusyImpl(DCM_t* self)
{
    return (self->state != DCM_STATE_IDLE) ? 1 : 0;
}

static void DCM_ProcessImpl(DCM_t* self)
{
    uint32_t current_tick;

    if (self->state != DCM_STATE_TIMED) {
        return;
    }

    current_tick = DCM_GetTick();

    /* 定时时间到，自动停止 */
    if (current_tick >= self->end_tick) {
        self->Stop(self);
    }
}

/* =============================================================================
 * 全局电机对象实例
 * ============================================================================= */

DCM_t DCM[DCM_MAX];

/* =============================================================================
 * 初始化与控制接口
 * ============================================================================= */

void DCM_InitAll(void)
{
    uint8_t i;

    /* 初始化TIM8硬件 */
    DCM_TIM8_HwInit();

    /* 初始化每个电机 */
    for (i = 0; i < DCM_MAX; i++) {
        /* 基本属性 */
        DCM[i].id = (DCM_ID_TypeDef)i;
        DCM[i].state = DCM_STATE_IDLE;
        DCM[i].direction = DIR_FORWARD;
        DCM[i].duty_percent = 0;

        /* 函数指针绑定 */
        DCM[i].Init = DCM_InitImpl;
        DCM[i].Run = DCM_RunImpl;
        DCM[i].RunRevolutions = DCM_RunRevolutionsImpl;
        DCM[i].Stop = DCM_StopImpl;
        DCM[i].SetCalibration = DCM_SetCalibrationImpl;
        DCM[i].IsBusy = DCM_IsBusyImpl;
        DCM[i].Process = DCM_ProcessImpl;

        /* 执行初始化 */
        DCM[i].Init(&DCM[i]);
    }
}

/* 停止所有电机 */
void DCM_StopAll(void)
{
    uint8_t i;
    for (i = 0; i < DCM_MAX; i++) {
        DCM[i].Stop(&DCM[i]);
    }
}

/* 处理定时运行电机的自动停止 */
void DCM_ProcessAll(void)
{
    uint8_t i;
    for (i = 0; i < DCM_MAX; i++) {
        DCM[i].Process(&DCM[i]);
    }
}

/* =============================================================================
 * 标定接口
 * ============================================================================= */

/**
 * 标定电机：实测转动指定圈数的时间
 * 用标定结果自动修正按圈数转动时的预估时间
 *
 * @param id 电机编号
 * @param test_duty 测试占空比(1-100)
 * @param revolutions 转动的圈数
 * @param actual_ms 实际测量的耗时
 */
void DCM_CalibrateMotor(DCM_ID_TypeDef id, uint8_t test_duty, float revolutions, uint32_t actual_ms)
{
    float ms_per_rev;
    if (actual_ms == 0 || revolutions <= 0) {
        return;
    }
    /* 计算1圈需要的毫秒数 */
    ms_per_rev = (float)actual_ms / revolutions;
    /* 保存标定参数 */
    DCM[id].SetCalibration(&DCM[id], ms_per_rev, test_duty);
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
