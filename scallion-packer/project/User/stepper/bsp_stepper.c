/**
 ******************************************************************************
 * 步进电机底层驱动（VET6版本）
 *
 * ============================================================================
 * VET6 LQFP100 引脚配置（根据ST RM0008 F103xE核实）：
 *   步进1: TIM1_CH1 → PE9（须 AFIO FullRemap TIM1；不再用 PA8）
 *   步进2: TIM2_CH1 → PA0, DIR → PC2
 *   步进3: TIM3_CH1 → PA6, DIR → PC4（PB0 留给超声 TRIG）
 *   步进4: TIM5_CH2 → PA1, DIR → PC12
 *   步进5: TIM1_CH4 → PE14, DIR → PD2	全程800步运行12s
 *   步进6: TIM1_CH3 → PE13, DIR → PD10	全程800步运行12s
 *
 * 定时器分配：
 *   TIM1 (APB2): 步进1(CH1), 步进5(CH4), 步进6(CH3) — 高级定时器
 *   TIM2 (APB1): 步进2(CH1) — 通用定时器
 *   TIM3 (APB1): 步进3(CH1) — 通用定时器
 *   TIM5 (APB1): 步进4(CH2) — 通用定时器
 *
 * TIM1中断处理: TIM1_UP_IRQn → 步进1, 步进5, 步进6
 * TIM2中断处理: TIM2_IRQn → 步进2
 * TIM3中断处理: TIM3_IRQn → 步进3
 * TIM5中断处理: TIM5_IRQn → 步进4
 *
 * 作者：工程训练中心116电控组
 * 日期：2026-04-13（VET6 LQFP100修正版）
 ******************************************************************************
 */

#include "bsp_stepper.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include <stddef.h>

/* TIM时钟频率 72MHz */
#define STEPPER_TIM_CLK_HZ   72000000U

/* 步进电机工作模式 */
typedef enum {
    STEPPER_MODE_STOP = 0,
    STEPPER_MODE_CONTINUOUS,
    STEPPER_MODE_MOVE_STEPS
} Stepper_Mode_TypeDef;

/* 步进电机通道控制结构体 */
typedef struct {
    TIM_TypeDef      *TIMx;
    uint8_t           tim_channel;    /* 定时器通道：1=CH1, 2=CH2, 3=CH3, 4=CH4 */
    GPIO_TypeDef     *dir_port;
    uint16_t          dir_pin;
    volatile int32_t  position;       /* 当前位置（累计脉冲数） */
    volatile uint32_t remain;          /* 剩余脉冲数（走步模式） */
    Stepper_Mode_TypeDef mode;         /* 当前工作模式 */
    int8_t            dir_sign;        /* 方向符号：+1正转，-1反转 */
    uint8_t           dir_invert;      /* DIR电平取反标志 */
    uint16_t          current_hz;      /* 当前速度（Hz） */
} Stepper_Channel_TypeDef;

/* 步进电机配置表（VET6 LQFP100修正版）
 *
 * 修正说明（2026-04-13）：
 *   - 步进3: TIM3_CH1(PA6)，与 PB0 超声 TRIG 解耦
 *   - 步进4: 原PC12(无TIM!) → PA1(TIM5_CH2)，PC12改为DIR
 *   - TIM1 FullRemap：CH1→PE9，CH3→PE13，CH4→PE14（PE13/14 无波多为未重映射）
 *   - 步进5: PE14(TIM1_CH4)；步进6: PE13(TIM1_CH3)
 *
 * 定时器映射：
 *   TIM1: 步进1(CH1), 步进5(CH4), 步进6(CH3)
 *   TIM2: 步进2(CH1)
 *   TIM3: 步进3(CH1)
 *   TIM5: 步进4(CH2)
 */
static Stepper_Channel_TypeDef s_ch[STEPPER_MAX] =
{
    /* 步进1 - 同步带A：TIM1_CH1(PE9，FullRemap)，DIR=PC0 */
    { TIM1, 1, GPIOC, GPIO_Pin_0,  0, 0, STEPPER_MODE_STOP, 1, 0, 1 },
    /* 步进2 - 同步带B：TIM2_CH1(PA0)，DIR=PC2 */
    { TIM2, 1, GPIOC, GPIO_Pin_2,  0, 0, STEPPER_MODE_STOP, 1, 0, 1 },
    /* 步进3 - 传送带：TIM3_CH1(PA6)，DIR=PC4 */
    { TIM3, 1, GPIOC, GPIO_Pin_4,  0, 1, STEPPER_MODE_STOP, 1, 0, 0 },
    /* 步进4 - 打包：TIM5_CH2(PA1)，DIR=PC12 */
    { TIM5, 2, GPIOC, GPIO_Pin_12, 0, 0, STEPPER_MODE_STOP, 1, 0, 0 },
    /* 步进5 - 切割丝杆A：TIM1_CH4(PE14)，DIR=PD2 */
    { TIM1, 4, GPIOD, GPIO_Pin_2,   0, 0, STEPPER_MODE_STOP, 1, 0, 0 },
    /* 步进6 - 切割丝杆B：TIM1_CH3(PE13)，DIR=PD10 */
    { TIM1, 3, GPIOD, GPIO_Pin_10, 0, 0, STEPPER_MODE_STOP, 1, 0, 0 },
};

/* 同步带电机组（步进1+2）状态 */
static uint8_t s_sync_running = 0;
static Stepper_Dir_TypeDef s_sync_dir = STEPPER_DIR_CW;
static uint16_t s_sync_speed = 0;

/* 丝杆电机组（步进5+6）状态 */
static uint8_t s_screw_running = 0;
static Stepper_Dir_TypeDef s_screw_dir = STEPPER_DIR_CW;
static uint16_t s_screw_speed = 0;

/* 内部函数声明 */
static void Stepper_GPIO_Init(void);
static void Stepper_TIM_Init(void);
static void Stepper_HardwareStop(Stepper_ID_TypeDef id);
static void Stepper_SetFrequency(Stepper_ID_TypeDef id, uint16_t hz);
static void Stepper_StartTimer(Stepper_ID_TypeDef id);
static void Stepper_StopTimer(Stepper_ID_TypeDef id);
static Stepper_Channel_TypeDef *Stepper_Get(Stepper_ID_TypeDef id);
static void Stepper_TIM1_SetPulCcerMask(uint16_t mask);
static uint16_t Stepper_TIM1_ChannelBit(uint8_t tim_channel);

/* 获取步进电机通道指针 */
static Stepper_Channel_TypeDef *Stepper_Get(Stepper_ID_TypeDef id)
{
    if (id >= STEPPER_MAX) return NULL;
    return &s_ch[id];
}

/**
 * TIM1 三路 PUL 共用 CCER：只开当前需要的通道，避免其它脚误输出脉冲。
 * mask 为 TIM_CCER_CC1E / CC3E / CC4E 的组合；0 表示三路比较输出全关。
 */
static void Stepper_TIM1_SetPulCcerMask(uint16_t mask)
{
    uint16_t c;

    c  = TIM1->CCER;
    c &= (uint16_t)~(TIM_CCER_CC1E | TIM_CCER_CC3E | TIM_CCER_CC4E);
    c |= (mask & (TIM_CCER_CC1E | TIM_CCER_CC3E | TIM_CCER_CC4E));
    TIM1->CCER = c;
}

static uint16_t Stepper_TIM1_ChannelBit(uint8_t tim_channel)
{
    switch (tim_channel) {
        case 1: return TIM_CCER_CC1E;
        case 3: return TIM_CCER_CC3E;
        case 4: return TIM_CCER_CC4E;
        default: return 0;
    }
}

/* ============================================================================
 * GPIO 初始化（DIR 引脚和 PUL 引脚）
 * ============================================================================ */
static void Stepper_GPIO_Init(void)
{
    GPIO_InitTypeDef g;

    /* 使能 GPIO 时钟
     * 注意：PE端口在VET6 LQFP100上存在，需要使能
     * AFIO复用功能时钟也需使能 */
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
        RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO,
        ENABLE
    );

    /* PE13/PE14 为 TIM1_CH3/CH4 的唯一引脚：必须 FullRemap TIM1（CH1→PE9） */
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);

    /* 使能定时器时钟 */
    RCC_APB1PeriphClockCmd(
        RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM5,
        ENABLE
    );
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* ==================== DIR 引脚：推挽输出 ==================== */
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;

    /* PC0 - 步进1 DIR */
    g.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOC, &g);

    /* PC2 - 步进2 DIR */
    g.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOC, &g);

    /* PC4 - 步进3 DIR */
    g.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOC, &g);

    /* PC12 - 步进4 DIR */
    g.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOC, &g);

    /* PD2 - 步进5 DIR */
    g.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &g);

    /* PD10 - 步进6 DIR */
    g.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOD, &g);

    /* ==================== PUL 引脚：复用推挽输出 ==================== */
    g.GPIO_Mode = GPIO_Mode_AF_PP;

    /* PA0 - TIM2_CH1 → 步进2 */
    g.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &g);

    /* PA1 - TIM5_CH2 → 步进4 */
    g.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOA, &g);

    /* PA6 - TIM3_CH1 → 步进3 */
    g.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOA, &g);

    /* PE9 - TIM1_CH1 → 步进1（FullRemap 后 CH1 在 PE9，非 PA8） */
    g.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOE, &g);

    /* PE13 - TIM1_CH3 → 步进6 */
    g.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOE, &g);

    /* PE14 - TIM1_CH4 → 步进5 */
    g.GPIO_Pin = GPIO_Pin_14;
    GPIO_Init(GPIOE, &g);
}

/* ============================================================================
 * 定时器 PWM 初始化
 *
 * TIM1: 步进1(CH1), 步进5(CH4), 步进6(CH3)
 * TIM2: 步进2(CH1)
 * TIM3: 步进3(CH1)
 * TIM5: 步进4(CH2)
 * ============================================================================ */
static void Stepper_TIM_Init(void)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;
    NVIC_InitTypeDef nv;

    /* -------------------- TIM1 配置（步进1/5/6，共用TIM1） -------------------- */
    /* TIM1 是高级定时器，必须设置 BDTR 的 MOE 位 */
    tb.TIM_Prescaler         = 71;    /* 1MHz */
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = 999;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tb);

    /* CH1 - 步进1(同步带A) */
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState= TIM_OutputState_Disable;
    oc.TIM_Pulse       = 500;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* CH4 - 步进5(切割丝杆A) */
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState= TIM_OutputState_Disable;
    oc.TIM_Pulse       = 500;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC4Init(TIM1, &oc);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* CH3 - 步进6(切割丝杆B) */
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState= TIM_OutputState_Disable;
    oc.TIM_Pulse       = 500;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC3Init(TIM1, &oc);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);    /* TIM1 必须使能 MOE */
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    nv.NVIC_IRQChannel            = TIM1_UP_IRQn;
    nv.NVIC_IRQChannelPreemptionPriority = 2;
    nv.NVIC_IRQChannelSubPriority = 0;
    nv.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nv);
    TIM_Cmd(TIM1, DISABLE);

    /* -------------------- TIM2 配置（步进2：同步带B） -------------------- */
    tb.TIM_Prescaler = 71;
    tb.TIM_Period = 999;
    TIM_TimeBaseInit(TIM2, &tb);
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState= TIM_OutputState_Disable;
    oc.TIM_Pulse       = 500;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &oc);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    nv.NVIC_IRQChannel            = TIM2_IRQn;
    nv.NVIC_IRQChannelPreemptionPriority = 2;
    nv.NVIC_IRQChannelSubPriority = 1;
    nv.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nv);
    TIM_Cmd(TIM2, DISABLE);

    /* -------------------- TIM3 配置（步进3：传送带，CH1 / PA6） -------------------- */
    tb.TIM_Prescaler = 71;
    tb.TIM_Period = 999;
    TIM_TimeBaseInit(TIM3, &tb);
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState= TIM_OutputState_Disable;
    oc.TIM_Pulse       = 500;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &oc);   /* CH1 */
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    nv.NVIC_IRQChannel            = TIM3_IRQn;
    nv.NVIC_IRQChannelPreemptionPriority = 2;
    nv.NVIC_IRQChannelSubPriority = 2;
    nv.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nv);
    TIM_Cmd(TIM3, DISABLE);

    /* -------------------- TIM5 配置（步进4：打包，CH2） -------------------- */
    /* VET6 LQFP100: TIM5 存在，IRQ = 50 */
#ifndef TIM5_IRQn
    #define TIM5_IRQn ((IRQn_Type)50)
#endif
    tb.TIM_Prescaler = 71;
    tb.TIM_Period = 999;
    TIM_TimeBaseInit(TIM5, &tb);
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState= TIM_OutputState_Disable;
    oc.TIM_Pulse       = 500;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC2Init(TIM5, &oc);    /* CH2 */
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    nv.NVIC_IRQChannel            = TIM5_IRQn;
    nv.NVIC_IRQChannelPreemptionPriority = 2;
    nv.NVIC_IRQChannelSubPriority = 3;
    nv.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nv);
    TIM_Cmd(TIM5, DISABLE);
}

/* ============================================================================
 * 使能/禁用步进电机（ENA不接，此函数保留但不起作用）
 * ============================================================================ */
void Stepper_Enable(Stepper_ID_TypeDef id, uint8_t ena)
{
    (void)id;
    (void)ena;
    /* ENA不接，驱动器默认使能，此函数空实现 */
}

/* ============================================================================
 * 设置步进电机旋转方向
 * ============================================================================ */
void Stepper_SetDirLevel(Stepper_ID_TypeDef id, Stepper_Dir_TypeDef forward)
{
    Stepper_Channel_TypeDef *c = Stepper_Get(id);
    uint16_t level;
    if (!c) return;

    level = forward ? Bit_SET : Bit_RESET;
    if (c->dir_invert) {
        level = (level == Bit_SET) ? Bit_RESET : Bit_SET;
    }

    if (level == Bit_SET) {
        GPIO_SetBits(c->dir_port, c->dir_pin);
    } else {
        GPIO_ResetBits(c->dir_port, c->dir_pin);
    }
}

/* ============================================================================
 * 设置 PWM 频率
 * ============================================================================ */
static void Stepper_SetFrequency(Stepper_ID_TypeDef id, uint16_t hz)
{
    Stepper_Channel_TypeDef *c;
    TIM_TypeDef *TIMx;
    uint32_t arr;
    uint16_t cmp;
    const uint16_t psc = 71;

    c = Stepper_Get(id);
    if (!c) return;
    TIMx = c->TIMx;

    if (hz < 1U)      hz = 1U;
    if (hz > 20000U)  hz = 20000U;

    arr = (STEPPER_TIM_CLK_HZ / (uint32_t)(psc + 1U)) / (uint32_t)hz;
    if (arr < 2U)       arr = 2U;
    if (arr > 0xFFFFU)  arr = 0xFFFFU;

    TIM_SetAutoreload(TIMx, (uint16_t)(arr - 1U));

    /* 设置占空比约50% */
    cmp = (uint16_t)(arr / 2U);
    switch (c->tim_channel) {
        case 1: TIM_SetCompare1(TIMx, cmp); break;
        case 2: TIM_SetCompare2(TIMx, cmp); break;
        case 3: TIM_SetCompare3(TIMx, cmp); break;
        case 4: TIM_SetCompare4(TIMx, cmp); break;
    }
    c->current_hz = hz;
}

/* ============================================================================
 * 启动定时器
 * ============================================================================ */
static void Stepper_StartTimer(Stepper_ID_TypeDef id)
{
    Stepper_Channel_TypeDef *c = Stepper_Get(id);
    if (!c) return;
    TIM_ClearITPendingBit(c->TIMx, TIM_IT_Update);
    TIM_ITConfig(c->TIMx, TIM_IT_Update, ENABLE);
    TIM_Cmd(c->TIMx, ENABLE);
}

/* ============================================================================
 * 停止定时器
 * ============================================================================ */
static void Stepper_StopTimer(Stepper_ID_TypeDef id)
{
    Stepper_Channel_TypeDef *c = Stepper_Get(id);
    if (!c) return;
    TIM_Cmd(c->TIMx, DISABLE);
    TIM_ITConfig(c->TIMx, TIM_IT_Update, DISABLE);
}

/* ============================================================================
 * 硬件停止
 * ============================================================================ */
static void Stepper_HardwareStop(Stepper_ID_TypeDef id)
{
    Stepper_Channel_TypeDef *c = Stepper_Get(id);
    if (!c) return;
    TIM_Cmd(c->TIMx, DISABLE);
    TIM_ITConfig(c->TIMx, TIM_IT_Update, DISABLE);
    c->mode   = STEPPER_MODE_STOP;
    c->remain = 0;
}

/* ============================================================================
 * 初始化
 * ============================================================================ */
void Stepper_Init(void)
{
    Stepper_GPIO_Init();
    Stepper_TIM_Init();
}

/**
 * 在任意可能改写 AFIO->MAPR 的模块（如 MPU6050 配置 I2C 重映射）之后调用，
 * 恢复 TIM1 FullRemap，并再次配置 PE9/PE13/PE14 为复用输出，避免丝杆 PUL 丢失。
 */
void Stepper_ReapplyTim1RemapAndPulGpio(void)
{
    GPIO_InitTypeDef g;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);

    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOE, &g);
}

/* 供 main/调试：上电默认关闭 TIM1 三路 PUL 的 CC输出使能（与旧 tim1_enable_outputs(0) 一致） */
void Stepper_TIM1_ClearPulOutputs(void)
{
    Stepper_TIM1_SetPulCcerMask(0);
}

/* ============================================================================
 * 连续旋转
 * ============================================================================ */
void Stepper_RunAtSpeed(Stepper_ID_TypeDef id, uint16_t steps_per_sec, Stepper_Dir_TypeDef dir)
{
    Stepper_Channel_TypeDef *c = Stepper_Get(id);
    if (!c) return;

    if (steps_per_sec == 0U) {
        Stepper_Stop(id);
        return;
    }

    if (c->TIMx == TIM1) {
        Stepper_TIM1_SetPulCcerMask(Stepper_TIM1_ChannelBit(c->tim_channel));
    }

    Stepper_SetDirLevel(id, dir);
    c->dir_sign = (dir == STEPPER_DIR_CW) ? 1 : -1;
    c->mode     = STEPPER_MODE_CONTINUOUS;
    c->remain   = 0;

    Stepper_SetFrequency(id, steps_per_sec);
    Stepper_StartTimer(id);
}

/* ============================================================================
 * 定量走步
 * ============================================================================ */
void Stepper_MoveSteps(Stepper_ID_TypeDef id, uint32_t total_steps, Stepper_Dir_TypeDef dir, uint16_t steps_per_sec)
{
    Stepper_Channel_TypeDef *c = Stepper_Get(id);
    if (!c || total_steps == 0U) return;

    if (c->TIMx == TIM1) {
        Stepper_TIM1_SetPulCcerMask(Stepper_TIM1_ChannelBit(c->tim_channel));
    }

    Stepper_SetDirLevel(id, dir);
    c->dir_sign = (dir == STEPPER_DIR_CW) ? 1 : -1;
    c->mode     = STEPPER_MODE_MOVE_STEPS;
    c->remain   = total_steps;

    Stepper_SetFrequency(id, steps_per_sec);
    Stepper_StartTimer(id);
}

/* ============================================================================
 * 停止
 * ============================================================================ */
void Stepper_Stop(Stepper_ID_TypeDef id)
{
    Stepper_HardwareStop(id);
}

/* ============================================================================
 * 获取位置
 * ============================================================================ */
int32_t Stepper_GetPosition(Stepper_ID_TypeDef id)
{
    Stepper_Channel_TypeDef *c = Stepper_Get(id);
    return c ? c->position : 0;
}

/* ============================================================================
 * 重置位置
 * ============================================================================ */
void Stepper_ResetPosition(Stepper_ID_TypeDef id)
{
    Stepper_Channel_TypeDef *c = Stepper_Get(id);
    if (c) c->position = 0;
}

/* ============================================================================
 * 获取速度
 * ============================================================================ */
uint16_t Stepper_GetCurrentSpeed(Stepper_ID_TypeDef id)
{
    Stepper_Channel_TypeDef *c = Stepper_Get(id);
    if (!c) return 0;
    if (c->mode == STEPPER_MODE_STOP) return 0;
    return c->current_hz;
}

/* ============================================================================
 * 同步带双电机控制
 * ============================================================================ */
void Stepper_SyncRun(Stepper_ID_TypeDef id_a, Stepper_ID_TypeDef id_b,
                     uint16_t steps_per_sec, Stepper_Dir_TypeDef dir)
{
    (void)id_a;
    (void)id_b;

    if (steps_per_sec == 0) {
        Stepper_SyncStop();
        return;
    }

    s_sync_running = 1;
    s_sync_dir = dir;
    s_sync_speed = steps_per_sec;

    Stepper_TIM1_SetPulCcerMask(TIM_CCER_CC1E);

    /* 步进1 DIR=取反，步进2 DIR=原方向 → 两皮带物理反向 */
    Stepper_SetDirLevel(STEPPER_SYNC_A, 1);
    Stepper_SetDirLevel(STEPPER_SYNC_B, 0);

    s_ch[STEPPER_SYNC_A].dir_sign = 1;
    s_ch[STEPPER_SYNC_B].dir_sign = 1;

    s_ch[STEPPER_SYNC_A].mode = STEPPER_MODE_CONTINUOUS;
    s_ch[STEPPER_SYNC_B].mode = STEPPER_MODE_CONTINUOUS;
    s_ch[STEPPER_SYNC_A].remain = 0;
    s_ch[STEPPER_SYNC_B].remain = 0;

    Stepper_SetFrequency(STEPPER_SYNC_A, steps_per_sec);
    Stepper_SetFrequency(STEPPER_SYNC_B, steps_per_sec);
    Stepper_StartTimer(STEPPER_SYNC_A);
    Stepper_StartTimer(STEPPER_SYNC_B);
}

void Stepper_SyncRun_Motor1Reversed(uint16_t steps_per_sec, Stepper_Dir_TypeDef dir)
{
    if (steps_per_sec == 0) {
        Stepper_SyncStop();
        return;
    }

    s_sync_running = 1;
    s_sync_dir     = dir;
    s_sync_speed   = steps_per_sec;

    Stepper_TIM1_SetPulCcerMask(TIM_CCER_CC1E);

    /* 步进1 DIR=取反，步进2 DIR=原方向 → 两皮带物理反向 */
    Stepper_SetDirLevel(STEPPER_SYNC_A, (dir == STEPPER_DIR_CW) ? STEPPER_DIR_CW : STEPPER_DIR_CCW);
    Stepper_SetDirLevel(STEPPER_SYNC_B, dir);

    s_ch[STEPPER_SYNC_A].dir_sign = 1;
    s_ch[STEPPER_SYNC_B].dir_sign = 1;

    s_ch[STEPPER_SYNC_A].mode = STEPPER_MODE_CONTINUOUS;
    s_ch[STEPPER_SYNC_B].mode = STEPPER_MODE_CONTINUOUS;
    s_ch[STEPPER_SYNC_A].remain = 0;
    s_ch[STEPPER_SYNC_B].remain = 0;

    Stepper_SetFrequency(STEPPER_SYNC_A, steps_per_sec);
    Stepper_SetFrequency(STEPPER_SYNC_B, steps_per_sec);
    Stepper_StartTimer(STEPPER_SYNC_A);
    Stepper_StartTimer(STEPPER_SYNC_B);
}

void Stepper_SyncStop(void)
{
    s_sync_running = 0;
    s_sync_speed = 0;
    Stepper_Stop(STEPPER_SYNC_A);
    Stepper_Stop(STEPPER_SYNC_B);
}

/* ============================================================================
 * 丝杆双电机控制
 * ============================================================================ */
void Stepper_ScrewRun(Stepper_ID_TypeDef id_a, Stepper_ID_TypeDef id_b,
                      uint16_t steps_per_sec, Stepper_Dir_TypeDef dir)
{
    (void)id_a;
    (void)id_b;

    if (steps_per_sec == 0) {
        Stepper_ScrewStop();
        return;
    }

    s_screw_running = 1;
    s_screw_dir = dir;
    s_screw_speed = steps_per_sec;

    Stepper_TIM1_SetPulCcerMask(TIM_CCER_CC3E | TIM_CCER_CC4E);

    /* 步进5和步进6同方向 */
    Stepper_SetDirLevel(STEPPER_SCREW_A, dir);
    Stepper_SetDirLevel(STEPPER_SCREW_B, dir);

    /* CW=向上(dir_sign=+1), CCW=向下(dir_sign=-1) */
    s_ch[STEPPER_SCREW_A].dir_sign = (dir == STEPPER_DIR_CW) ? 1 : -1;
    s_ch[STEPPER_SCREW_B].dir_sign = (dir == STEPPER_DIR_CW) ? 1 : -1;

    s_ch[STEPPER_SCREW_A].mode = STEPPER_MODE_CONTINUOUS;
    s_ch[STEPPER_SCREW_B].mode = STEPPER_MODE_CONTINUOUS;
    s_ch[STEPPER_SCREW_A].remain = 0;
    s_ch[STEPPER_SCREW_B].remain = 0;

    Stepper_SetFrequency(STEPPER_SCREW_A, steps_per_sec);
    Stepper_SetFrequency(STEPPER_SCREW_B, steps_per_sec);

    Stepper_StartTimer(STEPPER_SCREW_A);
    Stepper_StartTimer(STEPPER_SCREW_B);
}

void Stepper_ScrewStop(void)
{
    s_screw_running = 0;
    s_screw_speed = 0;
    Stepper_Stop(STEPPER_SCREW_A);
    Stepper_Stop(STEPPER_SCREW_B);
}

/* ============================================================================
 * 丝杆正转（向上，朝E0方向）
 * PE0限位触发时自动停止
 * ============================================================================ */
void Stepper_ScrewUp(uint16_t steps_per_sec)
{
    /* 注意：限位检测由调用方在外部循环中完成，此函数只负责启动电机 */
    if (steps_per_sec == 0) {
        Stepper_ScrewStop();
        return;
    }

    /* 如果已经在运行，先停止再重新启动 */
    if (s_screw_running) {
        Stepper_ScrewStop();
    }

    s_screw_running = 1;
    s_screw_dir = STEPPER_DIR_CW;
    s_screw_speed = steps_per_sec;

    Stepper_TIM1_SetPulCcerMask(TIM_CCER_CC3E | TIM_CCER_CC4E);

    /* CW = 往上走（朝E0方向） */
    Stepper_SetDirLevel(STEPPER_SCREW_A, STEPPER_DIR_CW);
    Stepper_SetDirLevel(STEPPER_SCREW_B, STEPPER_DIR_CW);

    /* 向上走，位置增加 */
    s_ch[STEPPER_SCREW_A].dir_sign = 1;
    s_ch[STEPPER_SCREW_B].dir_sign = 1;

    s_ch[STEPPER_SCREW_A].mode = STEPPER_MODE_CONTINUOUS;
    s_ch[STEPPER_SCREW_B].mode = STEPPER_MODE_CONTINUOUS;
    s_ch[STEPPER_SCREW_A].remain = 0;
    s_ch[STEPPER_SCREW_B].remain = 0;

    Stepper_SetFrequency(STEPPER_SCREW_A, steps_per_sec);
    Stepper_SetFrequency(STEPPER_SCREW_B, steps_per_sec);

    Stepper_StartTimer(STEPPER_SCREW_A);
    Stepper_StartTimer(STEPPER_SCREW_B);
}

/* ============================================================================
 * 丝杆反转（向下，朝E1方向）
 * 注意：限位检测由调用方在外部循环中完成，此函数只负责启动电机
 * ============================================================================ */
void Stepper_ScrewDown(uint16_t steps_per_sec)
{
    if (steps_per_sec == 0) {
        Stepper_ScrewStop();
        return;
    }

    if (s_screw_running) {
        Stepper_ScrewStop();
    }

    s_screw_running = 1;
    s_screw_dir = STEPPER_DIR_CCW;
    s_screw_speed = steps_per_sec;

    Stepper_TIM1_SetPulCcerMask(TIM_CCER_CC3E | TIM_CCER_CC4E);

    /* CCW = 往下走（朝E1方向） */
    Stepper_SetDirLevel(STEPPER_SCREW_A, STEPPER_DIR_CCW);
    Stepper_SetDirLevel(STEPPER_SCREW_B, STEPPER_DIR_CCW);

    /* 向下走，位置减少 */
    s_ch[STEPPER_SCREW_A].dir_sign = -1;
    s_ch[STEPPER_SCREW_B].dir_sign = -1;

    s_ch[STEPPER_SCREW_A].mode = STEPPER_MODE_CONTINUOUS;
    s_ch[STEPPER_SCREW_B].mode = STEPPER_MODE_CONTINUOUS;
    s_ch[STEPPER_SCREW_A].remain = 0;
    s_ch[STEPPER_SCREW_B].remain = 0;

    Stepper_SetFrequency(STEPPER_SCREW_A, steps_per_sec);
    Stepper_SetFrequency(STEPPER_SCREW_B, steps_per_sec);

    Stepper_StartTimer(STEPPER_SCREW_A);
    Stepper_StartTimer(STEPPER_SCREW_B);
}

/* ============================================================================
 * 查询丝杆是否在运行
 * ============================================================================ */
uint8_t Stepper_ScrewIsRunning(void)
{
    return s_screw_running;
}

/* ============================================================================
 * 定时器更新中断回调
 * ============================================================================ */
void Stepper_OnTimUpdate(Stepper_ID_TypeDef id)
{
    Stepper_Channel_TypeDef *c = Stepper_Get(id);
    if (!c) return;

    if (c->mode == STEPPER_MODE_MOVE_STEPS) {
        if (c->remain > 0U) {
            c->remain--;
            c->position += (int32_t)c->dir_sign;
            if (c->remain == 0U) {
                Stepper_HardwareStop(id);
            }
        }
    } else if (c->mode == STEPPER_MODE_CONTINUOUS) {
        c->position += (int32_t)c->dir_sign;
    }
}

/* ============================================================================
 * 兼容层：MOTOR 数组和 MOTOR_InitAll
 * ============================================================================ */
Motor_State_Data_TypeDef MOTOR[STEPPER_MAX];

void MOTOR_InitAll(void)
{
    uint8_t i;
    Stepper_Init();
    for (i = 0; i < STEPPER_MAX; i++) {
        MOTOR[i].state = MOTOR_STATE_IDLE;
        MOTOR[i].current_speed = 0;
        MOTOR[i].position = 0;
    }
}

/* ============================================================================
 * 示波器测试：六路 PUL 连续硬件 PWM（关闭更新中断，避免占用 CPU）
 * 引脚：PE9 PA0 PA6 PA1 PE14 PE13
 * ============================================================================ */
void Stepper_StartContinuousPwmTest(uint16_t hz)
{
    if (hz < 10U) {
        hz = 10U;
    }
    if (hz > 20000U) {
        hz = 20000U;
    }

    Stepper_Init();

    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
    TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);

    Stepper_TIM1_SetPulCcerMask(TIM_CCER_CC1E | TIM_CCER_CC3E | TIM_CCER_CC4E);

    Stepper_SetFrequency(STEPPER_SYNC_A, hz);
    Stepper_SetFrequency(STEPPER_SCREW_A, hz);
    Stepper_SetFrequency(STEPPER_SCREW_B, hz);
    Stepper_SetFrequency(STEPPER_SYNC_B, hz);
    Stepper_SetFrequency(STEPPER_CONVEY, hz);
    Stepper_SetFrequency(STEPPER_PACK, hz);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
