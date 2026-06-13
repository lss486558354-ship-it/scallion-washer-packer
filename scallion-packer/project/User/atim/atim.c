/**
 ******************************************************************************
 * 高级定时器 TIM8 输出指定个数PWM 实现
 *
 * 原理详解：
 *
 *   STM32 高级定时器（TIM1/TIM8）有一个 RCR（Repetition Counter）寄存器。
 *   RCR 的作用是：每溢出 RCR+1 次才产生一次更新事件/中断。
 *
 *   例如：
 *   - RCR = 0：每次溢出都产生更新中断
 *   - RCR = 1：每2次溢出才产生一次更新中断
 *   - RCR = N：每 N+1 次溢出才产生一次更新中断
 *
 *   输出指定个数 PWM 的关键技巧（参考正点原子 HAL 库例程）：
 *
 *   假设 npwm = 5，即要输出 5 个 PWM 脉冲：
 *
 *   Step 1: 定时器使能，ARR = X，PWM 模式
 *            定时器开始从 0 计数，到 ARR 溢出，产生更新事件，计数器归零
 *            每溢出一次，输出一个 PWM 周期
 *
 *   Step 2: 当需要输出 5 个脉冲时：
 *            → RCR 写入 4 (npwm-1)
 *            → 触发更新事件（UEG=1），立即产生第1次更新中断
 *            → 第1次中断：RCR 已递减为 3，重新写入 RCR=3，触发更新
 *            → 第2次中断：RCR 已递减为 2，重新写入 RCR=2，触发更新
 *            → 第3次中断：RCR 已递减为 1，重新写入 RCR=1，触发更新
 *            → 第4次中断：RCR 已递减为 0，重新写入 RCR=0，触发更新
 *            → 第5次中断：RCR 递减到 0xFF（下溢），定时器停止
 *
 *   实际上，由于首次写入 RCR 后立即触发更新中断，RCR 被递减，
 *   所以每次中断时 RCR 已经少了 1。因此：
 *     首次中断：RCR = npwm - 2
 *     末次中断：RCR = 0xFF（溢出）
 *
 *   这样，恰好输出 npwm 个 PWM 脉冲后自动停止。
 *
 * ============================================================================
 * 作者：工程训练中心116电控组
 * 日期：2026-04-13（VET6版本，标准库移植版）
 ******************************************************************************
 */

#include "atim.h"
#include "stm32f10x_tim.h"

/* ============================================================================
 * 宏定义
 * ============================================================================ */
#define ATIM_TIMx             TIM8
#define ATIM_TIMx_CLK        RCC_APB2Periph_TIM8

/* GPIO 时钟和引脚定义 */
#define ATIM_GPIOx_CH1        GPIOC
#define ATIM_GPIOx_CH2        GPIOC
#define ATIM_GPIOx_CH3        GPIOC
#define ATIM_GPIOx_CH4        GPIOC
#define ATIM_GPIOx_CLK        RCC_APB2Periph_GPIOC

#define ATIM_PIN_CH1         GPIO_Pin_6   /* TIM8_CH1 → PC6 */
#define ATIM_PIN_CH2         GPIO_Pin_7   /* TIM8_CH2 → PC7 */
#define ATIM_PIN_CH3         GPIO_Pin_8   /* TIM8_CH3 → PC8 */
#define ATIM_PIN_CH4         GPIO_Pin_9   /* TIM8_CH4 → PC9 */

/* ============================================================================
 * 类型定义
 * ============================================================================ */

/* 各通道状态 */
typedef struct {
    volatile uint8_t busy;     /* 是否正在输出指定个数PWM */
    volatile uint8_t rcr_val;   /* 剩余需要输出的脉冲数（RCR值） */
    uint16_t pulse;            /* 当前占空比比较值 */
} ATIM_ChannelState_TypeDef;

/* ============================================================================
 * 静态变量
 * ============================================================================ */

static ATIM_ChannelState_TypeDef s_atim_ch[4] = {
    {0, 0, 500},
    {0, 0, 500},
    {0, 0, 500},
    {0, 0, 500}
};

/* ============================================================================
 * 内部函数声明
 * ============================================================================ */
static void ATIM_GPIO_Init(void);
static void ATIM_TIM8_Init(uint16_t arr, uint16_t psc);

/* ============================================================================
 * GPIO 初始化 - TIM8_CH1~CH4 → PC6~PC9
 * ============================================================================ */
static void ATIM_GPIO_Init(void)
{
    GPIO_InitTypeDef g;

    /* 使能 GPIOC 和 AFIO 时钟 */
    RCC_APB2PeriphClockCmd(ATIM_GPIOx_CLK | RCC_APB2Periph_AFIO, ENABLE);

    /* 配置 PC6/7/8/9 为复用推挽输出（TIM8_CH1~CH4） */
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;

    g.GPIO_Pin = ATIM_PIN_CH1;   /* PC6 */
    GPIO_Init(ATIM_GPIOx_CH1, &g);

    g.GPIO_Pin = ATIM_PIN_CH2;   /* PC7 */
    GPIO_Init(ATIM_GPIOx_CH2, &g);

    g.GPIO_Pin = ATIM_PIN_CH3;   /* PC8 */
    GPIO_Init(ATIM_GPIOx_CH3, &g);

    g.GPIO_Pin = ATIM_PIN_CH4;   /* PC9 */
    GPIO_Init(ATIM_GPIOx_CH4, &g);
}

/* ============================================================================
 * TIM8 定时器初始化
 * ============================================================================ */
static void ATIM_TIM8_Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef  tb;
    TIM_OCInitTypeDef        oc;
    NVIC_InitTypeDef         nv;

    /* 使能 TIM8 时钟（高级定时器在 APB2） */
    RCC_APB2PeriphClockCmd(ATIM_TIMx_CLK, ENABLE);

    /* -------------------- Time Base -------------------- */
    tb.TIM_Prescaler         = psc;                  /* 预分频 */
    tb.TIM_CounterMode       = TIM_CounterMode_Up;   /* 递增计数 */
    tb.TIM_Period            = arr;                  /* 自动重装载值 */
    tb.TIM_RepetitionCounter = 0;                    /* RCR = 0 */
    TIM_TimeBaseInit(ATIM_TIMx, &tb);

    /* -------------------- Channel 1 (PC6) -------------------- */
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;           /* PWM 模式1 */
    oc.TIM_OutputState = TIM_OutputState_Enable;    /* 输出使能 */
    oc.TIM_OutputNState= TIM_OutputState_Disable;   /* 互补输出禁止 */
    oc.TIM_Pulse       = s_atim_ch[0].pulse;       /* 比较值（占空比） */
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;      /* 高电平有效 */
    TIM_OC1Init(ATIM_TIMx, &oc);
    TIM_OC1PreloadConfig(ATIM_TIMx, TIM_OCPreload_Enable);

    /* -------------------- Channel 2 (PC7) -------------------- */
    oc.TIM_Pulse = s_atim_ch[1].pulse;
    TIM_OC2Init(ATIM_TIMx, &oc);
    TIM_OC2PreloadConfig(ATIM_TIMx, TIM_OCPreload_Enable);

    /* -------------------- Channel 3 (PC8) -------------------- */
    oc.TIM_Pulse = s_atim_ch[2].pulse;
    TIM_OC3Init(ATIM_TIMx, &oc);
    TIM_OC3PreloadConfig(ATIM_TIMx, TIM_OCPreload_Enable);

    /* -------------------- Channel 4 (PC9) -------------------- */
    oc.TIM_Pulse = s_atim_ch[3].pulse;
    TIM_OC4Init(ATIM_TIMx, &oc);
    TIM_OC4PreloadConfig(ATIM_TIMx, TIM_OCPreload_Enable);

    /* 使能 ARR 预装载（缓冲） */
    TIM_ARRPreloadConfig(ATIM_TIMx, ENABLE);

    /* 使能 TIM8 主输出（MOE）—— 高级定时器必须设置！ */
    TIM_CtrlPWMOutputs(ATIM_TIMx, ENABLE);

    /* 使能更新中断 */
    TIM_ITConfig(ATIM_TIMx, TIM_IT_Update, ENABLE);

    /* 配置 NVIC */
    nv.NVIC_IRQChannel            = TIM8_UP_IRQn;
    nv.NVIC_IRQChannelPreemptionPriority = 1;
    nv.NVIC_IRQChannelSubPriority = 0;
    nv.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&nv);

    /* 定时器初始状态：失能（由 StartPWM/OutputNPwm 控制启动） */
    TIM_Cmd(ATIM_TIMx, DISABLE);
}

/* ============================================================================
 * 公共函数实现
 * ============================================================================ */

/* 初始化 TIM8 四路 PWM */
void ATIM_Init(uint16_t arr, uint16_t psc)
{
    ATIM_GPIO_Init();
    ATIM_TIM8_Init(arr, psc);
}

/* 设置指定通道的 PWM 占空比 */
void ATIM_SetPulse(ATIM_Channel_TypeDef channel, uint16_t pulse)
{
    if (channel > ATIM_CH4) return;

    s_atim_ch[channel].pulse = pulse;

    /* 如果定时器已在运行，更新 CCR 值 */
    if (TIM_GetCounter(ATIM_TIMx) != 0 || (ATIM_TIMx->CR1 & 0x01)) {
        switch (channel) {
            case ATIM_CH1: TIM_SetCompare1(ATIM_TIMx, pulse); break;
            case ATIM_CH2: TIM_SetCompare2(ATIM_TIMx, pulse); break;
            case ATIM_CH3: TIM_SetCompare3(ATIM_TIMx, pulse); break;
            case ATIM_CH4: TIM_SetCompare4(ATIM_TIMx, pulse); break;
        }
    }
}

/* 启动指定通道 PWM 输出（连续） */
void ATIM_StartPWM(ATIM_Channel_TypeDef channel)
{
    if (channel > ATIM_CH4) return;

    TIM_ClearITPendingBit(ATIM_TIMx, TIM_IT_Update);
    TIM_Cmd(ATIM_TIMx, ENABLE);
}

/* 停止指定通道 PWM 输出 */
void ATIM_StopPWM(ATIM_Channel_TypeDef channel)
{
    uint8_t i;

    (void)channel;

    /* 关闭定时器 */
    TIM_Cmd(ATIM_TIMx, DISABLE);
    TIM_ITConfig(ATIM_TIMx, TIM_IT_Update, DISABLE);

    /* 清除所有通道的 busy 标志 */
    for (i = 0; i < 4; i++) {
        s_atim_ch[i].busy = 0;
    }
}

/* 输出指定个数 PWM 脉冲（核心功能） */
void ATIM_OutputNPwm(ATIM_Channel_TypeDef channel, uint8_t npwm)
{
    uint8_t i;

    if (channel > ATIM_CH4 || npwm == 0) return;

    /* 清除所有通道的 busy 标志 */
    for (i = 0; i < 4; i++) {
        s_atim_ch[i].busy = 0;
    }

    /* 标记当前通道忙 */
    s_atim_ch[channel].busy = 1;
    s_atim_ch[channel].rcr_val = npwm;

    /* ★ 核心操作：写入 RCR = npwm - 1
     * 由于 RCR=0 时每次溢出都产生更新，
     * 写入 RCR = npwm - 1 后，硬件会自动递减，
     * 直到 RCR 减到 0xFF（下溢）时才停止。 */
    ATIM_TIMx->RCR = npwm - 1;

    /* 清除更新标志，然后触发更新事件（UEG=1）
     * 这会立即产生一次更新中断作为第一次计数 */
    TIM_ClearITPendingBit(ATIM_TIMx, TIM_IT_Update);
    ATIM_TIMx->EGR = 0x01;  /* UG=1，立即产生更新事件 */

    /* 使能更新中断并启动定时器 */
    TIM_ITConfig(ATIM_TIMx, TIM_IT_Update, ENABLE);
    TIM_Cmd(ATIM_TIMx, ENABLE);
}

/* 查询通道是否忙 */
uint8_t ATIM_IsBusy(ATIM_Channel_TypeDef channel)
{
    if (channel > ATIM_CH4) return 0;
    return s_atim_ch[channel].busy;
}

/* 停止所有 PWM 输出 */
void ATIM_StopAll(void)
{
    uint8_t i;

    ATIM_TIMx->CR1 &= ~(1 << 0);   /* CEN=0，关闭定时器 */
    TIM_ITConfig(ATIM_TIMx, TIM_IT_Update, DISABLE);

    for (i = 0; i < 4; i++) {
        s_atim_ch[i].busy = 0;
        s_atim_ch[i].rcr_val = 0;
    }
}

/* 四路 PC6~PC9 同时连续 50% PWM（关闭 TIM8 更新中断） */
void ATIM_StartContinuousAll50(void)
{
    uint16_t arr = ATIM_TIMx->ARR;
    uint16_t half = (uint16_t)(((uint32_t)arr + 1U) / 2U);
    uint8_t i;

    for (i = 0; i < 4; i++) {
        s_atim_ch[i].busy = 0;
        s_atim_ch[i].rcr_val = 0;
    }

    TIM_SetCompare1(ATIM_TIMx, half);
    TIM_SetCompare2(ATIM_TIMx, half);
    TIM_SetCompare3(ATIM_TIMx, half);
    TIM_SetCompare4(ATIM_TIMx, half);

    ATIM_TIMx->RCR = 0;
    TIM_ClearITPendingBit(ATIM_TIMx, TIM_IT_Update);
    TIM_ITConfig(ATIM_TIMx, TIM_IT_Update, DISABLE);
    TIM_CtrlPWMOutputs(ATIM_TIMx, ENABLE);
    TIM_Cmd(ATIM_TIMx, ENABLE);
}

/**
 * TIM8 更新中断回调
 *
 * RCR（重复计数器）工作原理：
 *   RCR=N 表示每溢出 RCR+1 次才产生一次更新事件。
 *   RCR 由硬件自动递减（每次计数器溢出时递减）。
 *
 * 输出 npwm 个 PWM 脉冲的流程（参考正点原子 HAL 库例程）：
 *
 *   假设 npwm = 5:
 *   Step 1: RCR = npwm-1 = 4, 触发更新事件(UG=1)立即产生第1次更新中断
 *   Step 2: 第1次中断：RCR 已递减为 3，重装 RCR=3（因为 3>0），再次触发更新
 *   Step 3: 第2次中断：RCR 已递减为 2，重装 RCR=2，再次触发更新
 *   Step 4: 第3次中断：RCR 已递减为 1，重装 RCR=1，再次触发更新
 *   Step 5: 第4次中断：RCR 已递减为 0，重装 RCR=0，再次触发更新
 *   Step 6: 第5次中断：RCR 从 0 递减到 0xFF（下溢），停止定时器
 *
 *   即：RCR 从 npwm-1 递减到 0xFF，共经历 npwm 次溢出，
 *       恰好输出 npwm 个 PWM 脉冲后自动停止。
 *
 * 关键点：
 *   - 第1次中断由 UG=1 触发（手动），此时 RCR=4（未递减）
 *   - 第2~5次中断由硬件溢出触发，此时 RCR 已递减
 *   - 只有在 RCR>0 时才重装（防止无限循环）
 */
void ATIM_TIM8_UP_Callback(void)
{
    uint8_t ch;

    /* 检查更新中断标志 */
    if (TIM_GetITStatus(ATIM_TIMx, TIM_IT_Update) == RESET) {
        return;
    }
    TIM_ClearITPendingBit(ATIM_TIMx, TIM_IT_Update);

    /* 遍历所有通道，找出忙的那个 */
    for (ch = 0; ch < 4; ch++) {
        if (s_atim_ch[ch].busy) {
            /* 读取当前 RCR 值 */
            uint8_t current_rcr = (uint8_t)(ATIM_TIMx->RCR & 0xFF);

            if (current_rcr == 0xFF) {
                /* RCR 从 0 下溢到 0xFF → npwm 个脉冲已输出完毕 */
                s_atim_ch[ch].busy = 0;
                s_atim_ch[ch].rcr_val = 0;
                ATIM_TIMx->CR1 &= ~(1 << 0);   /* CEN=0，关闭定时器 */
                TIM_ITConfig(ATIM_TIMx, TIM_IT_Update, DISABLE);
            } else {
                /* RCR > 0，重装并触发更新事件（仅在此分支重装） */
                ATIM_TIMx->RCR = current_rcr;
                ATIM_TIMx->EGR = 0x01;  /* UG=1，产生更新事件 */
            }
            break;  /* 一次只处理一个通道 */
        }
    }
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
