/**
 ******************************************************************************
 * FreeRTOS 任务头文件（裸机版）
 *
 * 功能说明：
 *   - 本文件已不再用于FreeRTOS任务管理
 *   - 保留消息结构体定义供其他代码引用
 *   - 任务函数声明保留桩函数签名
 *
 * 作者：工程训练中心116电控组
 * 版本：V1.0（裸机移植版）
 * 日期：2026-04-12
 ******************************************************************************
 */

#ifndef __FREERTOS_APP_H__
#define __FREERTOS_APP_H__

#include "stm32f10x.h"
#include "device.h"

/*-----------------------------------------------------------
 * 任务优先级定义（裸机模式下仅供参考）
 *-----------------------------------------------------------*/
#define TASK_PRIORITY_SENSOR       ( 5 )
#define TASK_PRIORITY_MOTOR       ( 6 )
#define TASK_PRIORITY_PROTOCOL    ( 6 )
#define TASK_PRIORITY_PROCESS     ( 6 )
#define TASK_PRIORITY_SERVO       ( 4 )
#define TASK_PRIORITY_DISPLAY     ( 3 )
#define TASK_PRIORITY_HEARTBEAT   ( 2 )
#define TASK_PRIORITY_IDLE        ( 1 )

/*-----------------------------------------------------------
 * 任务堆栈大小定义
 *-----------------------------------------------------------*/
#define TASK_STACK_SIZE_SENSOR     ( 256 )
#define TASK_STACK_SIZE_MOTOR     ( 256 )
#define TASK_STACK_SIZE_PROTOCOL  ( 512 )
#define TASK_STACK_SIZE_PROCESS   ( 512 )
#define TASK_STACK_SIZE_SERVO     ( 128 )
#define TASK_STACK_SIZE_DISPLAY   ( 256 )
#define TASK_STACK_SIZE_HEARTBEAT ( 128 )

/*-----------------------------------------------------------
 * 消息结构体定义
 *-----------------------------------------------------------*/

/* 电机控制消息 */
typedef struct {
    uint8_t motor_id;
    int32_t speed;
    uint8_t direction;
} MotorControlMsg_t;

/* 舵机控制消息 */
typedef struct {
    uint8_t servo_id;
    float angle;
} ServoControlMsg_t;

/* 传感器数据消息 */
typedef struct {
    uint32_t timestamp;
    float pressure;
    float water_level;
    float weight;
    int32_t motor1_pos;
    int32_t motor2_pos;
    int32_t motor3_pos;
    int32_t motor4_pos;
    int32_t motor5_pos;
} SensorDataMsg_t;

/*-----------------------------------------------------------
 * 私有函数
 *-----------------------------------------------------------*/
void Create_RTOS_Tasks(void);
void Create_RTOS_Semaphores(void);
void Create_RTOS_Queues(void);

/*-----------------------------------------------------------
 * 任务桩函数（裸机模式下由 main.c 超循环替代）
 *-----------------------------------------------------------*/
void vTask_Sensor(void *pvParameters);
void vTask_Motor(void *pvParameters);
void vTask_Protocol(void *pvParameters);
void vTask_Process(void *pvParameters);
void vTask_Servo(void *pvParameters);
void vTask_Display(void *pvParameters);
void vTask_Heartbeat(void *pvParameters);

#endif /* __FREERTOS_APP_H__ */
