/**
 ******************************************************************************
 * STM32F103VET6 中断处理头文件
 *
 * ============================================================================
 * VET6版本说明：
 *   - 步进电机6路：TIM1/TIM2/TIM3/TIM5
 *   - 移除了C8T6特有的TIM4配置
 *
 * 作者：工程训练中心116电控组
 * 日期：2026-04-09（VET6适配版）
 ******************************************************************************
 */

#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

/* Exported functions ------------------------------------------------------- */
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void SysTick_Handler(void);
void TIM1_UP_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM5_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_IT_H */

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
