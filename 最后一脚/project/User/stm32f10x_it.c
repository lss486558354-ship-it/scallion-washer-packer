/**
 ******************************************************************************
 * STM32F103VET6 中断服务程序
 *
 * ============================================================================
 * VET6版本说明：
 *   - 步进电机6路：
 *       TIM1_UP → Stepper_OnTimUpdate(STEPPER_SYNC_A, STEPPER_SCREW_A, STEPPER_SCREW_B)
 *       TIM2_IRQn → Stepper_OnTimUpdate(STEPPER_SYNC_B)
 *       TIM3_IRQn → Stepper_OnTimUpdate(STEPPER_CONVEY)
 *       TIM5_IRQn → Stepper_OnTimUpdate(STEPPER_PACK)
 *   - 高级定时器TIM8：
 *       TIM8_UP → ATIM_TIM8_UP_Callback()（输出指定个数PWM）
 *   - 移除了C8T6特有的TIM4配置
 *
 * 作者：工程训练中心116电控组
 * 日期：2026-04-13（VET6适配版）
 ******************************************************************************
 */

#include "stm32f10x_it.h"
#include "bsp_stepper.h"
#include "bsp_tick.h"
#include "atim.h"

/* 声明外部变量 */
extern volatile uint32_t g_tick_ms;

/* ============================================================================
 * Cortex-M3 异常处理（空实现，防止链接报错）
 * ============================================================================ */

void NMI_Handler(void)         { }
void HardFault_Handler(void)   { while(1) { } }
void MemManage_Handler(void)  { while(1) { } }
void BusFault_Handler(void)   { while(1) { } }
void UsageFault_Handler(void) { while(1) { } }
void DebugMon_Handler(void)   { }

/* ============================================================================
 * 步进电机定时器中断服务程序
 * ============================================================================ */

/**
 * TIM1 更新中断（TIM1_UP_IRQn）
 * 同步带A (TIM1_CH1/PE9 FullRemap)、切割丝杆A (TIM1_CH4/PE14)、切割丝杆B (TIM1_CH3/PE13)
 */
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        /* 同步带A */
        Stepper_OnTimUpdate(STEPPER_SYNC_A);
        /* 切割丝杆A */
        Stepper_OnTimUpdate(STEPPER_SCREW_A);
        /* 切割丝杆B */
        Stepper_OnTimUpdate(STEPPER_SCREW_B);
    }
}

/**
 * TIM2 更新中断（TIM2_IRQn）
 * 同步带B (TIM2_CH1/PA0)
 */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        Stepper_OnTimUpdate(STEPPER_SYNC_B);
    }
}

/**
 * TIM3 更新中断（TIM3_IRQn）
 * 传送带 (TIM3_CH1/PA6)
 */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        Stepper_OnTimUpdate(STEPPER_CONVEY);
    }
}

/**
 * TIM5 更新中断（TIM5_IRQn）
 * 打包电机 (TIM5_CH2/PA1)
 */
void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        Stepper_OnTimUpdate(STEPPER_PACK);
    }
}

/**
 * TIM8 更新中断（TIM8_UP_IRQn）
 * 高级定时器：TIM8_CH1(PC6)、TIM8_CH2(PC7)、TIM8_CH3(PC8)、TIM8_CH4(PC9)
 * 用于输出指定个数PWM（RCR重复计数器控制脉冲个数）
 */
void TIM8_UP_IRQHandler(void)
{
    ATIM_TIM8_UP_Callback();
}

/* ============================================================================
 * SysTick滴答定时器中断（裸机时基，1ms）
 * ============================================================================ */

void SysTick_Handler(void)
{
    BSP_Tick_OnSysTick();
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
