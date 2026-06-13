/**
 ******************************************************************************
 * 滴答定时器驱动（裸机版）
 *
 * ============================================================================
 * 功能说明：
 *   - 使用 SysTick 提供 1ms 时基
 *   - 供直流减速电机按标定时间折算圈数使用
 *   - 供统一设备层 Device.Timer_xxx 使用
 *
 * ============================================================================
 * 使用示例：
 *
 * // 1. 初始化（在main.c的main函数开始处调用）
 * BSP_Tick_Init();                              // 初始化SysTick 1ms中断
 *
 * // 2. 获取当前时间
 * uint32_t now = BSP_GetTickMs();               // 获取当前毫秒数
 *
 * // 3. 非阻塞延时
 * uint32_t start = BSP_GetTickMs();
 * while (BSP_GetTickMs() - start < 1000) { }  // 延时1秒
 *
 * // 4. 定时器工具（由Device统一封装）
 * SoftTimer_t myTimer;
 * Device.Timer_Start(&myTimer, 1000);          // 启动1秒定时器
 * if (Device.Timer_IsExpired(&myTimer)) {      // 检查是否到期
 *     Device.LED.Toggle();
 * }
 *
 * // 5. 直流电机定时运行
 * DCM_RunRevolutions(&DCM[DCM_1], 2.5, 80, DIR_FORWARD);
 * // 内部使用g_tick_ms计算运行时间
 *
 * ============================================================================
 * 参数说明：
 * - BSP_GetTickMs()：返回uint32_t，范围0~4294967295ms（约49.7天溢出）
 *
 * ============================================================================
 * 作者：工程训练中心116电控组
 * 日期：2026-04-12（裸机移植版）
 ******************************************************************************
 */

#include "bsp_tick.h"

/* 全局毫秒计数（从 BSP_Tick_Init 后递增） */
volatile uint32_t g_tick_ms = 0U;

/**
 * @brief 初始化SysTick为1ms中断
 * @note 72MHz系统时钟，LOAD = 72000 - 1 = 71999
 *       每1ms触发一次SysTick_Handler，g_tick_ms++
 */
void BSP_Tick_Init(void)
{
    /* 设置Reload值为1ms（72MHz / 72000 = 1kHz） */
    SysTick->LOAD = 72000 - 1;

    /* 清零当前计数值 */
    SysTick->VAL = 0;

    /* SysTick 优先级设为 6，确保 1ms 基准不被 TIM5 步进中断打乱。
     * USART 优先级1 > SysTick6 > TIM1/2/3/5 优先级2，串口接收优先，时基其次，步进最后。 */
    SCB->SHP[11] = (uint8_t)((6U) << 4);

    /* 使能：时钟源=AHB(72MHz)、开启中断、启动计数 */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk   |
                     SysTick_CTRL_ENABLE_Msk;
}

/**
 * @brief 获取当前毫秒数
 * @return uint32_t类型，计数值
 * @note 从0开始，每1ms加1，约49.7天溢出
 */
uint32_t BSP_GetTickMs(void)
{
    return g_tick_ms;
}

/**
 * @brief SysTick中断回调（供 stm32f10x_it.c 的 SysTick_Handler 调用）
 */
void BSP_Tick_OnSysTick(void)
{
    g_tick_ms++;
    BSP_Tick_AppHook1ms();
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
