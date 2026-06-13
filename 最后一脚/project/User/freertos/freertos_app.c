/**
 ******************************************************************************
 * FreeRTOS 任务桩文件（裸机版）
 *
 * 功能说明：
 *   - 本文件已不再用于FreeRTOS任务管理
 *   - 所有业务逻辑由 main.c 超循环调度器接管
 *   - 本文件保留函数声明供其他代码引用
 *
 * 作者：工程训练中心116电控组
 * 版本：V1.0（裸机移植版）
 * 日期：2026-04-12
 ******************************************************************************
 */

#include "freertos_app.h"

/*-----------------------------------------------------------
 * 任务创建函数（裸机模式下为空）
 *-----------------------------------------------------------*/
void Create_RTOS_Tasks(void)
{
    /* 裸机模式下任务调度由 main.c 超循环接管，此函数无需实现 */
}

/*-----------------------------------------------------------
 * 信号量创建（裸机模式下为空）
 *-----------------------------------------------------------*/
void Create_RTOS_Semaphores(void)
{
    /* 裸机模式下无信号量需求 */
}

/*-----------------------------------------------------------
 * 队列创建（裸机模式下为空）
 *-----------------------------------------------------------*/
void Create_RTOS_Queues(void)
{
    /* 裸机模式下无队列需求 */
}

/*-----------------------------------------------------------
 * 任务函数桩（裸机模式下由 main.c 超循环替代）
 *-----------------------------------------------------------*/
void vTask_Sensor(void *pvParameters)
{
    (void)pvParameters;
    /* 由 main.c 每100ms调度 */
}

void vTask_Motor(void *pvParameters)
{
    (void)pvParameters;
    /* 由 main.c 每10ms调度 */
}

void vTask_Protocol(void *pvParameters)
{
    (void)pvParameters;
    /* 由 main.c 每1ms调度 */
}

void vTask_Process(void *pvParameters)
{
    (void)pvParameters;
    /* 由 main.c 每10ms调度 */
}

void vTask_Servo(void *pvParameters)
{
    (void)pvParameters;
    /* 由 main.c 事件驱动 */
}

void vTask_Display(void *pvParameters)
{
    (void)pvParameters;
    /* 由 main.c 每500ms调度 */
}

void vTask_Heartbeat(void *pvParameters)
{
    (void)pvParameters;
    /* 由 main.c 每500ms调度 */
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
