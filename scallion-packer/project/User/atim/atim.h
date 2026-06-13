/**
 ******************************************************************************
 * 高级定时器 TIM8 输出指定个数PWM 实验
 *
 * ============================================================================
 * 芯片：STM32F103VE LQFP100
 *
 * TIM8 硬件资源（高级定时器，与TIM1完全相同）：
 *   TIM8_CH1 → PC6  (PWM脉冲输出)
 *   TIM8_CH2 → PC7  (PWM脉冲输出)
 *   TIM8_CH3 → PC8  (PWM脉冲输出)
 *   TIM8_CH4 → PC9  (PWM脉冲输出)
 *
 * TIM8_N 互补输出（预留）：
 *   TIM8_CH1N → PA7
 *   TIM8_CH2N → PB0
 *   TIM8_CH3N → PB1
 *
 * 注意：PC6~PC9 在 LQFP100 上固定为 TIM8 引脚，无其他复用选择。
 * ============================================================================
 * 功能说明：
 *   利用 TIM8 的 RCR（重复计数器）实现输出指定个数的 PWM 脉冲。
 *   RCR 的作用是：每溢出 RCR+1 次才产生一次更新中断。
 *   因此，只需在更新中断中写入目标 RCR 值，即可精确控制 PWM 输出个数。
 *
 * 原理：
 *   PWM频率 = TIM8_CLK / ((PSC+1) * (ARR+1))
 *   RCR=0：每次溢出都产生更新中断，即每 (ARR+1) 个计数输出1个PWM
 *   RCR=N：每 (N+1)*(ARR+1) 个计数输出1个PWM
 *   实际使用：首次触发更新中断时写入 RCR = npwm-1，后续由硬件自动递减
 *
 * ============================================================================
 * 作者：工程训练中心116电控组
 * 日期：2026-04-13（VET6版本）
 ******************************************************************************
 */

#ifndef __ATIM_H
#define __ATIM_H

#include "stm32f10x.h"

/* ============================================================================
 * TIM8 四路PWM通道编号
 * ============================================================================ */
typedef enum {
    ATIM_CH1 = 0,   /* TIM8_CH1 → PC6 */
    ATIM_CH2 = 1,   /* TIM8_CH2 → PC7 */
    ATIM_CH3 = 2,   /* TIM8_CH3 → PC8 */
    ATIM_CH4 = 3    /* TIM8_CH4 → PC9 */
} ATIM_Channel_TypeDef;

/* ============================================================================
 * 函数声明
 * ============================================================================ */

/* 初始化 TIM8 四路 PWM（各通道独立频率/占空比）
 * arr：自动重装载值（决定PWM周期）
 * psc：预分频值（决定计数频率）
 * 注：四路使用相同的 ARR 和 PSC，即相同的 PWM 频率 */
void ATIM_Init(uint16_t arr, uint16_t psc);

/* 设置指定通道的 PWM 占空比（占空比 = pulse/arr）
 * channel：ATIM_CH1~CH4
 * pulse：比较值（0 ~ arr） */
void ATIM_SetPulse(ATIM_Channel_TypeDef channel, uint16_t pulse);

/* 启动指定通道的 PWM 输出（连续输出，不限制个数） */
void ATIM_StartPWM(ATIM_Channel_TypeDef channel);

/* 停止指定通道的 PWM 输出 */
void ATIM_StopPWM(ATIM_Channel_TypeDef channel);

/* 输出指定个数 PWM 脉冲（核心功能）
 * channel：ATIM_CH1~CH4
 * npwm：期望输出的 PWM 脉冲个数（范围 1~255）
 *
 * 工作流程：
 *   1. 写入 RCR = npwm - 1
 *   2. 触发更新事件（UEG=1）立即产生第一次更新中断
 *   3. 在中断回调中，递减 RCR，硬件自动递减
 *   4. 当 RCR 减至 0 后，停止定时器
 *
 * 注意：调用此函数后，必须使能 TIM8 更新中断和 NVIC，
 *       在 HAL_TIM_PeriodElapsedCallback 中处理自动停止逻辑 */
void ATIM_OutputNPwm(ATIM_Channel_TypeDef channel, uint8_t npwm);

/* 获取当前通道是否正在输出指定个数PWM的过程中
 * channel：ATIM_CH1~CH4
 * 返回：1=正在输出中，0=已停止 */
uint8_t ATIM_IsBusy(ATIM_Channel_TypeDef channel);

/* 停止所有四路 PWM 输出 */
void ATIM_StopAll(void);

/* 四路同时连续输出约 50% 占空比 PWM（关闭更新中断，用于示波器/接线测试）
 * 须先调用 ATIM_Init()配置 ARR/PSC */
void ATIM_StartContinuousAll50(void);

/* TIM8 更新中断回调（供 stm32f10x_it.c 中的 TIM8_UP_IRQHandler 调用） */
void ATIM_TIM8_UP_Callback(void);

#endif /* __ATIM_H */

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
