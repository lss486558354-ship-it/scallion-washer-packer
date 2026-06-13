/**
 ******************************************************************************
 * 统一设备管理层 - 实现文件
 * 
 * 提供面向对象的设备访问接口
 * 
 * 作者：工程训练中心116电控组
 * 日期：2026-03-24
 ******************************************************************************
 */

/* VET6裸机模式：已启用完整功能 */

#include "device.h"
#include "bsp_tick.h"
#include "bsp_usart_obj.h"
#include "bsp_dc_motor_obj.h"
#include "protocol\process_control.h"
#include "bsp_hx711.h"
#include <stdio.h>
#include <stdarg.h>

/* =============================================================================
 * 内部函数声明
 * ============================================================================= */

/* 系统 */
static void Device_InitAll(void);
static uint32_t Device_GetTick(void);
static uint8_t Device_GetStatus(void);

/* 电机1 */
static void Device_Motor1_Init(void);
static void Device_Motor1_Run(uint16_t speed, Motor_Direction_TypeDef dir);
static void Device_Motor1_Stop(void);
static int32_t Device_Motor1_GetPos(void);
static void Device_Motor1_ResetPos(void);
static void Device_Motor1_Enable(uint8_t en);
static void Device_Motor1_Brake(void);
static void Device_Motor1_SetSpeed(uint16_t speed);
static Motor_State_TypeDef Device_Motor1_GetState(void);

/* 电机2 */
static void Device_Motor2_Init(void);
static void Device_Motor2_Run(uint16_t speed, Motor_Direction_TypeDef dir);
static void Device_Motor2_Stop(void);
static int32_t Device_Motor2_GetPos(void);
static void Device_Motor2_ResetPos(void);
static void Device_Motor2_Enable(uint8_t en);
static void Device_Motor2_Brake(void);
static void Device_Motor2_SetSpeed(uint16_t speed);
static Motor_State_TypeDef Device_Motor2_GetState(void);

/* 电机3（传送带） */
static void Device_Motor3_Init(void);
static void Device_Motor3_Run(uint16_t speed, Motor_Direction_TypeDef dir);
static void Device_Motor3_Stop(void);
static int32_t Device_Motor3_GetPos(void);
static void Device_Motor3_ResetPos(void);
static void Device_Motor3_Enable(uint8_t en);
static void Device_Motor3_Brake(void);
static void Device_Motor3_SetSpeed(uint16_t speed);
static Motor_State_TypeDef Device_Motor3_GetState(void);

/* 电机4（打包电机） */
static void Device_Motor4_Init(void);
static void Device_Motor4_Run(uint16_t speed, Motor_Direction_TypeDef dir);
static void Device_Motor4_Stop(void);
static int32_t Device_Motor4_GetPos(void);
static void Device_Motor4_ResetPos(void);
static void Device_Motor4_Enable(uint8_t en);
static void Device_Motor4_Brake(void);
static void Device_Motor4_SetSpeed(uint16_t speed);
static Motor_State_TypeDef Device_Motor4_GetState(void);

/* 电机5 */
static void Device_Motor5_Init(void);
static void Device_Motor5_Run(uint16_t speed, Motor_Direction_TypeDef dir);
static void Device_Motor5_Stop(void);
static int32_t Device_Motor5_GetPos(void);
static void Device_Motor5_ResetPos(void);
static void Device_Motor5_Enable(uint8_t en);
static void Device_Motor5_Brake(void);
static void Device_Motor5_SetSpeed(uint16_t speed);
static Motor_State_TypeDef Device_Motor5_GetState(void);

/* 电机6 */
static void Device_Motor6_Init(void);
static void Device_Motor6_Run(uint16_t speed, Motor_Direction_TypeDef dir);
static void Device_Motor6_Stop(void);
static int32_t Device_Motor6_GetPos(void);
static void Device_Motor6_ResetPos(void);
static void Device_Motor6_Enable(uint8_t en);
static void Device_Motor6_Brake(void);
static void Device_Motor6_SetSpeed(uint16_t speed);
static Motor_State_TypeDef Device_Motor6_GetState(void);

/* 舵机1 */
static void Device_Servo1_Init(void);
static void Device_Servo1_SetAngle(uint16_t angle);
static uint16_t Device_Servo1_GetAngle(void);

/* 舵机2 */
static void Device_Servo2_Init(void);
static void Device_Servo2_SetAngle(uint16_t angle);
static uint16_t Device_Servo2_GetAngle(void);

/* 舵机3 */
static void Device_Servo3_Init(void);
static void Device_Servo3_SetAngle(uint16_t angle);
static uint16_t Device_Servo3_GetAngle(void);

/* 舵机4 */
static void Device_Servo4_Init(void);
static void Device_Servo4_SetAngle(uint16_t angle);
static uint16_t Device_Servo4_GetAngle(void);

/* 直流电机1 */
static void Device_DCMotor1_Init(void);
static void Device_DCMotor1_Run(uint8_t duty, uint8_t forward);
static void Device_DCMotor1_Stop(void);
static void Device_DCMotor1_RunRevolutions(float revs, uint8_t duty, uint8_t forward);
static uint8_t Device_DCMotor1_IsBusy(void);

/* 直流电机2 */
static void Device_DCMotor2_Init(void);
static void Device_DCMotor2_Run(uint8_t duty, uint8_t forward);
static void Device_DCMotor2_Stop(void);
static void Device_DCMotor2_RunRevolutions(float revs, uint8_t duty, uint8_t forward);
static uint8_t Device_DCMotor2_IsBusy(void);

/* 直流电机3 */
static void Device_DCMotor3_Init(void);
static void Device_DCMotor3_Run(uint8_t duty, uint8_t forward);
static void Device_DCMotor3_Stop(void);
static void Device_DCMotor3_RunRevolutions(float revs, uint8_t duty, uint8_t forward);
static uint8_t Device_DCMotor3_IsBusy(void);

/* 直流电机4 */
static void Device_DCMotor4_Init(void);
static void Device_DCMotor4_Run(uint8_t duty, uint8_t forward);
static void Device_DCMotor4_Stop(void);
static void Device_DCMotor4_RunRevolutions(float revs, uint8_t duty, uint8_t forward);
static uint8_t Device_DCMotor4_IsBusy(void);

/* 限位开关 */
static void Device_LimitSwitch_Init(void);
static uint8_t Device_LimitSwitch_IsTriggered(LimitSwitch_ID_TypeDef switch_id);
static uint8_t Device_LimitSwitch_ReadAll(void);
static uint8_t Device_LimitSwitch_EmergencyPressed(void);
static uint8_t Device_LimitSwitch_M1OriginTriggered(void);
static uint8_t Device_LimitSwitch_M1ForwardTriggered(void);
static uint8_t Device_LimitSwitch_M2OriginTriggered(void);
static uint8_t Device_LimitSwitch_M2ForwardTriggered(void);
static uint8_t Device_LimitSwitch_M3OriginTriggered(void);
static uint8_t Device_LimitSwitch_M3ForwardTriggered(void);

/* 串口1 */
static void Device_UART1_Init(void);
static void Device_UART1_SendString(const char* str);
static void Device_UART1_Printf(const char* fmt, ...);
static uint8_t Device_UART1_Available(void);
static uint8_t Device_UART1_ReadByte(void);

/* 串口2 */
static void Device_UART2_Init(void);
static void Device_UART2_SendString(const char* str);
static void Device_UART2_Printf(const char* fmt, ...);
static uint8_t Device_UART2_Available(void);
static uint8_t Device_UART2_ReadByte(void);

/* 串口3（串口屏） */
static void Device_UART3_Init(void);
static void Device_UART3_SendString(const char* str);
static void Device_UART3_Printf(const char* fmt, ...);
static uint8_t Device_UART3_Available(void);
static uint8_t Device_UART3_ReadByte(void);

static void Device_UART4_Init(void);
static void Device_UART4_SendString(const char* str);
static void Device_UART4_Printf(const char* fmt, ...);
static uint8_t Device_UART4_Available(void);
static uint8_t Device_UART4_ReadByte(void);

/* 传感器 */
static void Device_Sensor_Init(void);
static float Device_Sensor_ReadPressure(void);
static float Device_Sensor_ReadWaterLevel(void);
static float Device_Sensor_ReadWeight(void);

/* LED */
static void Device_LED_Init(void);
static void Device_LED_On(void);
static void Device_LED_Off(void);
static void Device_LED_Toggle(void);

/* =============================================================================
 * 系统实现
 * ============================================================================= */

static void Device_InitAll(void)
{
    /* 初始化所有设备 */
    BSP_Tick_Init();
    MOTOR_InitAll();
    SERVO_InitAll();
    DCM_InitAll();
    LimitSwitch_Init();
    UART_InitAll();
    ServoBus_InitAll();
    SENSOR_InitAll();
    LED_Init();
}

static uint32_t Device_GetTick(void)
{
    return BSP_GetTickMs();
}

static uint8_t Device_GetStatus(void)
{
    extern volatile uint8_t g_system_status;
    return g_system_status;
}

/* =============================================================================
 * 定时器工具实现
 * ============================================================================= */

/* 启动定时器
 * timer: 定时器结构体指针
 * interval_ms: 定时间隔(毫秒)
 * 
 * 使用示例:
 *   SoftTimer_t myTimer;
 *   Device.Timer_Start(&myTimer, 1000);  // 启动1秒定时器
 *   if (Device.Timer_IsExpired(&myTimer)) {
 *       // 定时到了
 *   }
 */
static void Device_Timer_Start(SoftTimer_t *timer, uint32_t interval_ms)
{
    timer->start = BSP_GetTickMs();
    timer->interval = interval_ms;
    timer->running = 1;
}

/* 检查定时器是否到期
 * 返回: 1=到期了, 0=未到期
 * 
 * 注意：到期后会立即重置定时器，实现连续定时
 */
static uint8_t Device_Timer_IsExpired(SoftTimer_t *timer)
{
    uint32_t now = BSP_GetTickMs();
    uint32_t elapsed = now - timer->start;
    
    if (timer->running && elapsed >= timer->interval) {
        timer->start += timer->interval;  // 重置起点，实现连续定时
        return 1;
    }
    return 0;
}

/* 重置定时器（重新开始计时） */
static void Device_Timer_Reset(SoftTimer_t *timer)
{
    timer->start = BSP_GetTickMs();
}

/* =============================================================================
 * 电机1实现
 * ============================================================================= */

static void Device_Motor1_Init(void)
{
    M1_Init();
}

static void Device_Motor1_Run(uint16_t speed, Motor_Direction_TypeDef dir)
{
    M1_Run(speed, dir);
}

static void Device_Motor1_Stop(void)
{
    M1_Stop();
}

static int32_t Device_Motor1_GetPos(void)
{
    return M1_GetPos();
}

static void Device_Motor1_ResetPos(void)
{
    M1_ResetPos();
}

static void Device_Motor1_Enable(uint8_t en)
{
    M1_Enable(en);
}

static void Device_Motor1_Brake(void)
{
    M1_Brake();
}

static void Device_Motor1_SetSpeed(uint16_t speed)
{
    M1_SetSpeed(speed);
}

static Motor_State_TypeDef Device_Motor1_GetState(void)
{
    return M1_GetState();
}

/* =============================================================================
 * 电机2实现
 * ============================================================================= */

static void Device_Motor2_Init(void)
{
    M2_Init();
}

static void Device_Motor2_Run(uint16_t speed, Motor_Direction_TypeDef dir)
{
    M2_Run(speed, dir);
}

static void Device_Motor2_Stop(void)
{
    M2_Stop();
}

static int32_t Device_Motor2_GetPos(void)
{
    return M2_GetPos();
}

static void Device_Motor2_ResetPos(void)
{
    M2_ResetPos();
}

static void Device_Motor2_Enable(uint8_t en)
{
    M2_Enable(en);
}

static void Device_Motor2_Brake(void)
{
    M2_Brake();
}

static void Device_Motor2_SetSpeed(uint16_t speed)
{
    M2_SetSpeed(speed);
}

static Motor_State_TypeDef Device_Motor2_GetState(void)
{
    return M2_GetState();
}

/* =============================================================================
 * 电机3实现
 * ============================================================================= */

static void Device_Motor3_Init(void)
{
    M3_Init();
}

static void Device_Motor3_Run(uint16_t speed, Motor_Direction_TypeDef dir)
{
    M3_Run(speed, dir);
}

static void Device_Motor3_Stop(void)
{
    M3_Stop();
}

static int32_t Device_Motor3_GetPos(void)
{
    return M3_GetPos();
}

static void Device_Motor3_ResetPos(void)
{
    M3_ResetPos();
}

static void Device_Motor3_Enable(uint8_t en)
{
    M3_Enable(en);
}

static void Device_Motor3_Brake(void)
{
    M3_Brake();
}

static void Device_Motor3_SetSpeed(uint16_t speed)
{
    M3_SetSpeed(speed);
}

static Motor_State_TypeDef Device_Motor3_GetState(void)
{
    return M3_GetState();
}

/* =============================================================================
 * 电机4实现
 * ============================================================================= */

static void Device_Motor4_Init(void)
{
    M4_Init();
}

static void Device_Motor4_Run(uint16_t speed, Motor_Direction_TypeDef dir)
{
    M4_Run(speed, dir);
}

static void Device_Motor4_Stop(void)
{
    M4_Stop();
}

static int32_t Device_Motor4_GetPos(void)
{
    return M4_GetPos();
}

static void Device_Motor4_ResetPos(void)
{
    M4_ResetPos();
}

static void Device_Motor4_Enable(uint8_t en)
{
    M4_Enable(en);
}

static void Device_Motor4_Brake(void)
{
    M4_Brake();
}

static void Device_Motor4_SetSpeed(uint16_t speed)
{
    M4_SetSpeed(speed);
}

static Motor_State_TypeDef Device_Motor4_GetState(void)
{
    return M4_GetState();
}

/* =============================================================================
 * 电机5实现（切割丝杆A）
 * ============================================================================= */

static void Device_Motor5_Init(void)
{
    /* 步进5通过MOTOR_InitAll统一初始化 */
}

static void Device_Motor5_Run(uint16_t speed, Motor_Direction_TypeDef dir)
{
    M5_Run(speed, dir);
}

static void Device_Motor5_Stop(void)
{
    M5_Stop();
}

static int32_t Device_Motor5_GetPos(void)
{
    return M5_GetPos();
}

static void Device_Motor5_ResetPos(void)
{
    M5_ResetPos();
}

static void Device_Motor5_Enable(uint8_t en)
{
    M5_Enable(en);
}

static void Device_Motor5_Brake(void)
{
    M5_Brake();
}

static void Device_Motor5_SetSpeed(uint16_t speed)
{
    M5_SetSpeed(speed);
}

static Motor_State_TypeDef Device_Motor5_GetState(void)
{
    return M5_GetState();
}

/* =============================================================================
 * 电机6实现（切割丝杆B）
 * ============================================================================= */

static void Device_Motor6_Init(void)
{
}

static void Device_Motor6_Run(uint16_t speed, Motor_Direction_TypeDef dir)
{
    M6_Run(speed, dir);
}

static void Device_Motor6_Stop(void)
{
    M6_Stop();
}

static int32_t Device_Motor6_GetPos(void)
{
    return M6_GetPos();
}

static void Device_Motor6_ResetPos(void)
{
    M6_ResetPos();
}

static void Device_Motor6_Enable(uint8_t en)
{
    M6_Enable(en);
}

static void Device_Motor6_Brake(void)
{
    M6_Brake();
}

static void Device_Motor6_SetSpeed(uint16_t speed)
{
    M6_SetSpeed(speed);
}

static Motor_State_TypeDef Device_Motor6_GetState(void)
{
    return M6_GetState();
}

/* =============================================================================
 * 舵机1实现
 * ============================================================================= */

static void Device_Servo1_Init(void)
{
    /* 舵机通过SERVO_InitAll统一初始化 */
}

static void Device_Servo1_SetAngle(uint16_t angle)
{
    SV1_SetAngle(angle);
}

static uint16_t Device_Servo1_GetAngle(void)
{
    return (uint16_t)SV1_GetAngle();
}

/* =============================================================================
 * 舵机2实现
 * ============================================================================= */

static void Device_Servo2_Init(void)
{
}

static void Device_Servo2_SetAngle(uint16_t angle)
{
    SV2_SetAngle(angle);
}

static uint16_t Device_Servo2_GetAngle(void)
{
    return (uint16_t)SV2_GetAngle();
}

/* =============================================================================
 * 舵机3实现
 * ============================================================================= */

static void Device_Servo3_Init(void)
{
}

static void Device_Servo3_SetAngle(uint16_t angle)
{
    SV3_SetAngle(angle);
}

static uint16_t Device_Servo3_GetAngle(void)
{
    return (uint16_t)SV3_GetAngle();
}

/* =============================================================================
 * 舵机4实现
 * ============================================================================= */

static void Device_Servo4_Init(void)
{
}

static void Device_Servo4_SetAngle(uint16_t angle)
{
    SV4_SetAngle(angle);
}

static uint16_t Device_Servo4_GetAngle(void)
{
    return (uint16_t)SV4_GetAngle();
}

/* =============================================================================
 * 直流电机1实现
 * ============================================================================= */

static void Device_DCMotor1_Init(void)
{
    /* 通过DCM_InitAll统一初始化 */
}

static void Device_DCMotor1_Run(uint8_t duty, uint8_t forward)
{
    DCM1_Run(duty, forward ? DIR_FORWARD : DIR_BACKWARD);
}

static void Device_DCMotor1_Stop(void)
{
    DCM1_Stop();
}

static void Device_DCMotor1_RunRevolutions(float revs, uint8_t duty, uint8_t forward)
{
    DCM1_RunRev(revs, duty, forward ? DIR_FORWARD : DIR_BACKWARD);
}

static uint8_t Device_DCMotor1_IsBusy(void)
{
    return DCM1_IsBusy();
}

/* =============================================================================
 * 直流电机2实现
 * ============================================================================= */

static void Device_DCMotor2_Init(void)
{
}

static void Device_DCMotor2_Run(uint8_t duty, uint8_t forward)
{
    DCM2_Run(duty, forward ? DIR_FORWARD : DIR_BACKWARD);
}

static void Device_DCMotor2_Stop(void)
{
    DCM2_Stop();
}

static void Device_DCMotor2_RunRevolutions(float revs, uint8_t duty, uint8_t forward)
{
    DCM2_RunRev(revs, duty, forward ? DIR_FORWARD : DIR_BACKWARD);
}

static uint8_t Device_DCMotor2_IsBusy(void)
{
    return DCM2_IsBusy();
}

/* =============================================================================
 * 直流电机3实现
 * ============================================================================= */

static void Device_DCMotor3_Init(void)
{
}

static void Device_DCMotor3_Run(uint8_t duty, uint8_t forward)
{
    DCM3_Run(duty, forward ? DIR_FORWARD : DIR_BACKWARD);
}

static void Device_DCMotor3_Stop(void)
{
    DCM3_Stop();
}

static void Device_DCMotor3_RunRevolutions(float revs, uint8_t duty, uint8_t forward)
{
    DCM3_RunRev(revs, duty, forward ? DIR_FORWARD : DIR_BACKWARD);
}

static uint8_t Device_DCMotor3_IsBusy(void)
{
    return DCM3_IsBusy();
}

/* =============================================================================
 * 直流电机4实现
 * ============================================================================= */

static void Device_DCMotor4_Init(void)
{
}

static void Device_DCMotor4_Run(uint8_t duty, uint8_t forward)
{
    DCM4_Run(duty, forward ? DIR_FORWARD : DIR_BACKWARD);
}

static void Device_DCMotor4_Stop(void)
{
    DCM4_Stop();
}

static void Device_DCMotor4_RunRevolutions(float revs, uint8_t duty, uint8_t forward)
{
    DCM4_RunRev(revs, duty, forward ? DIR_FORWARD : DIR_BACKWARD);
}

static uint8_t Device_DCMotor4_IsBusy(void)
{
    return DCM4_IsBusy();
}

/* =============================================================================
 * 限位开关实现
 * ============================================================================= */

static void Device_LimitSwitch_Init(void)
{
    LimitSwitch_Init();
}

static uint8_t Device_LimitSwitch_IsTriggered(LimitSwitch_ID_TypeDef switch_id)
{
    return LimitSwitch_IsTriggered(switch_id);
}

static uint8_t Device_LimitSwitch_ReadAll(void)
{
    return LimitSwitch_ReadAll();
}

static uint8_t Device_LimitSwitch_EmergencyPressed(void)
{
    return LimitSwitch_EmergencyPressed();
}

static uint8_t Device_LimitSwitch_M1OriginTriggered(void)
{
    return LimitSwitch_IsTriggered(LIMIT_M1_ORIGIN);
}

static uint8_t Device_LimitSwitch_M1ForwardTriggered(void)
{
    return LimitSwitch_IsTriggered(LIMIT_M1_FORWARD);
}

static uint8_t Device_LimitSwitch_M2OriginTriggered(void)
{
    return LimitSwitch_IsTriggered(LIMIT_M2_ORIGIN);
}

static uint8_t Device_LimitSwitch_M2ForwardTriggered(void)
{
    return LimitSwitch_IsTriggered(LIMIT_M2_FORWARD);
}

static uint8_t Device_LimitSwitch_M3OriginTriggered(void)
{
    return LimitSwitch_IsTriggered(LIMIT_M3_ORIGIN);
}

static uint8_t Device_LimitSwitch_M3ForwardTriggered(void)
{
    return LimitSwitch_IsTriggered(LIMIT_M3_FORWARD);
}

/* =============================================================================
 * 串口1实现
 * ============================================================================= */

static void Device_UART1_Init(void)
{
    /* 通过UART_InitAll统一初始化 */
}

static void Device_UART1_SendString(const char* str)
{
    UART1_SendString((char*)str);
}

static void Device_UART1_Printf(const char* fmt, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    UART1_SendString(buffer);
}

static uint8_t Device_UART1_Available(void)
{
    return UART1_Available();
}

static uint8_t Device_UART1_ReadByte(void)
{
    return UART1_ReadByte();
}

/* =============================================================================
 * 串口2实现
 * ============================================================================= */

static void Device_UART2_Init(void)
{
}

static void Device_UART2_SendString(const char* str)
{
    UART2_SendString((char*)str);
}

static void Device_UART2_Printf(const char* fmt, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    UART2_SendString(buffer);
}

static uint8_t Device_UART2_Available(void)
{
    return UART2_Available();
}

static uint8_t Device_UART2_ReadByte(void)
{
    return UART2_ReadByte();
}

/* =============================================================================
 * 串口3实现（串口屏）
 * ============================================================================= */

static void Device_UART3_Init(void)
{
}

static void Device_UART3_SendString(const char* str)
{
    if (str == NULL) {
        return;
    }
    UART1_SendString("[串口屏] TX ");
    UART1_SendString((char*)str);
    UART1_SendString("\r\n");
    UART3_SendString((char*)str);
}

static void Device_UART3_Printf(const char* fmt, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    UART1_SendString("[串口屏] TX ");
    UART1_SendString(buffer);
    UART1_SendString("\r\n");
    UART3_SendString(buffer);
}

static uint8_t Device_UART3_Available(void)
{
    return UART3_Available();
}

static uint8_t Device_UART3_ReadByte(void)
{
    return UART3_ReadByte();
}

/* =============================================================================
 * 串口4 - 重量传感器
 * =============================================================================*/

static void Device_UART4_Init(void)
{
}

static void Device_UART4_SendString(const char* str)
{
    UART4_SendString((char*)str);
}

static void Device_UART4_Printf(const char* fmt, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    UART4_SendString(buffer);
}

static uint8_t Device_UART4_Available(void)
{
    return UART4_Available();
}

static uint8_t Device_UART4_ReadByte(void)
{
    return UART4_ReadByte();
}

/* =============================================================================
 * 传感器实现
 * ============================================================================= */

static void Device_Sensor_Init(void)
{
    /* 通过SENSOR_InitAll统一初始化 */
}

static float Device_Sensor_ReadPressure(void)
{
    return ADC_PRESSURE_GetValue();
}

static float Device_Sensor_ReadWaterLevel(void)
{
    return ADC_WATER_GetValue();
}

static float Device_Sensor_ReadWeight(void)
{
    return HX711_GetWeightGram();
}

/* =============================================================================
 * LED实现
 * ============================================================================= */

static void Device_LED_Init(void)
{
    LED_Init();
}

static void Device_LED_On(void)
{
    LED_On();
}

static void Device_LED_Off(void)
{
    LED_Off();
}

static void Device_LED_Toggle(void)
{
    LED_Toggle();
}

/* =============================================================================
 * 全局设备实例
 * ============================================================================= */

DEVICE_t Device;

/* 初始化全局设备实例 */
void Device_InitInstance(void)
{
    /* 系统 */
    Device.Init = Device_InitAll;
    Device.GetTick = Device_GetTick;
    Device.GetStatus = Device_GetStatus;
    
    /* 定时器工具 */
    Device.Timer_Start = Device_Timer_Start;
    Device.Timer_IsExpired = Device_Timer_IsExpired;
    Device.Timer_Reset = Device_Timer_Reset;
    
    /* 电机1 */
    Device.Motor1.Init = Device_Motor1_Init;
    Device.Motor1.Run = Device_Motor1_Run;
    Device.Motor1.Stop = Device_Motor1_Stop;
    Device.Motor1.GetPos = Device_Motor1_GetPos;
    Device.Motor1.ResetPos = Device_Motor1_ResetPos;
    Device.Motor1.Enable = Device_Motor1_Enable;
    Device.Motor1.Brake = Device_Motor1_Brake;
    Device.Motor1.SetSpeed = Device_Motor1_SetSpeed;
    Device.Motor1.GetState = Device_Motor1_GetState;
    
    /* 电机2 */
    Device.Motor2.Init = Device_Motor2_Init;
    Device.Motor2.Run = Device_Motor2_Run;
    Device.Motor2.Stop = Device_Motor2_Stop;
    Device.Motor2.GetPos = Device_Motor2_GetPos;
    Device.Motor2.ResetPos = Device_Motor2_ResetPos;
    Device.Motor2.Enable = Device_Motor2_Enable;
    Device.Motor2.Brake = Device_Motor2_Brake;
    Device.Motor2.SetSpeed = Device_Motor2_SetSpeed;
    Device.Motor2.GetState = Device_Motor2_GetState;
    
    /* 电机3 */
    Device.Motor3.Init = Device_Motor3_Init;
    Device.Motor3.Run = Device_Motor3_Run;
    Device.Motor3.Stop = Device_Motor3_Stop;
    Device.Motor3.GetPos = Device_Motor3_GetPos;
    Device.Motor3.ResetPos = Device_Motor3_ResetPos;
    Device.Motor3.Enable = Device_Motor3_Enable;
    Device.Motor3.Brake = Device_Motor3_Brake;
    Device.Motor3.SetSpeed = Device_Motor3_SetSpeed;
    Device.Motor3.GetState = Device_Motor3_GetState;

    /* 电机4 */
    Device.Motor4.Init = Device_Motor4_Init;
    Device.Motor4.Run = Device_Motor4_Run;
    Device.Motor4.Stop = Device_Motor4_Stop;
    Device.Motor4.GetPos = Device_Motor4_GetPos;
    Device.Motor4.ResetPos = Device_Motor4_ResetPos;
    Device.Motor4.Enable = Device_Motor4_Enable;
    Device.Motor4.Brake = Device_Motor4_Brake;
    Device.Motor4.SetSpeed = Device_Motor4_SetSpeed;
    Device.Motor4.GetState = Device_Motor4_GetState;

    /* 电机5 */
    Device.Motor5.Init = Device_Motor5_Init;
    Device.Motor5.Run = Device_Motor5_Run;
    Device.Motor5.Stop = Device_Motor5_Stop;
    Device.Motor5.GetPos = Device_Motor5_GetPos;
    Device.Motor5.ResetPos = Device_Motor5_ResetPos;
    Device.Motor5.Enable = Device_Motor5_Enable;
    Device.Motor5.Brake = Device_Motor5_Brake;
    Device.Motor5.SetSpeed = Device_Motor5_SetSpeed;
    Device.Motor5.GetState = Device_Motor5_GetState;

    /* 电机6 */
    Device.Motor6.Init = Device_Motor6_Init;
    Device.Motor6.Run = Device_Motor6_Run;
    Device.Motor6.Stop = Device_Motor6_Stop;
    Device.Motor6.GetPos = Device_Motor6_GetPos;
    Device.Motor6.ResetPos = Device_Motor6_ResetPos;
    Device.Motor6.Enable = Device_Motor6_Enable;
    Device.Motor6.Brake = Device_Motor6_Brake;
    Device.Motor6.SetSpeed = Device_Motor6_SetSpeed;
    Device.Motor6.GetState = Device_Motor6_GetState;
    
    /* 舵机1 */
    Device.Servo1.Init = Device_Servo1_Init;
    Device.Servo1.SetAngle = Device_Servo1_SetAngle;
    Device.Servo1.GetAngle = Device_Servo1_GetAngle;
    
    /* 舵机2 */
    Device.Servo2.Init = Device_Servo2_Init;
    Device.Servo2.SetAngle = Device_Servo2_SetAngle;
    Device.Servo2.GetAngle = Device_Servo2_GetAngle;
    
    /* 舵机3 */
    Device.Servo3.Init = Device_Servo3_Init;
    Device.Servo3.SetAngle = Device_Servo3_SetAngle;
    Device.Servo3.GetAngle = Device_Servo3_GetAngle;
    
    /* 舵机4 */
    Device.Servo4.Init = Device_Servo4_Init;
    Device.Servo4.SetAngle = Device_Servo4_SetAngle;
    Device.Servo4.GetAngle = Device_Servo4_GetAngle;
    
    /* 直流电机1 */
    Device.DCMotor1.Init = Device_DCMotor1_Init;
    Device.DCMotor1.Run = Device_DCMotor1_Run;
    Device.DCMotor1.Stop = Device_DCMotor1_Stop;
    Device.DCMotor1.RunRevolutions = Device_DCMotor1_RunRevolutions;
    Device.DCMotor1.IsBusy = Device_DCMotor1_IsBusy;
    
    /* 直流电机2 */
    Device.DCMotor2.Init = Device_DCMotor2_Init;
    Device.DCMotor2.Run = Device_DCMotor2_Run;
    Device.DCMotor2.Stop = Device_DCMotor2_Stop;
    Device.DCMotor2.RunRevolutions = Device_DCMotor2_RunRevolutions;
    Device.DCMotor2.IsBusy = Device_DCMotor2_IsBusy;
    
    /* 直流电机3 */
    Device.DCMotor3.Init = Device_DCMotor3_Init;
    Device.DCMotor3.Run = Device_DCMotor3_Run;
    Device.DCMotor3.Stop = Device_DCMotor3_Stop;
    Device.DCMotor3.RunRevolutions = Device_DCMotor3_RunRevolutions;
    Device.DCMotor3.IsBusy = Device_DCMotor3_IsBusy;
    
    /* 直流电机4 */
    Device.DCMotor4.Init = Device_DCMotor4_Init;
    Device.DCMotor4.Run = Device_DCMotor4_Run;
    Device.DCMotor4.Stop = Device_DCMotor4_Stop;
    Device.DCMotor4.RunRevolutions = Device_DCMotor4_RunRevolutions;
    Device.DCMotor4.IsBusy = Device_DCMotor4_IsBusy;
    
    /* 限位开关 */
    Device.LimitSwitch.Init = Device_LimitSwitch_Init;
    Device.LimitSwitch.IsTriggered = Device_LimitSwitch_IsTriggered;
    Device.LimitSwitch.ReadAll = Device_LimitSwitch_ReadAll;
    Device.LimitSwitch.EmergencyPressed = Device_LimitSwitch_EmergencyPressed;
    Device.LimitSwitch.M1OriginTriggered = Device_LimitSwitch_M1OriginTriggered;
    Device.LimitSwitch.M1ForwardTriggered = Device_LimitSwitch_M1ForwardTriggered;
    Device.LimitSwitch.M2OriginTriggered = Device_LimitSwitch_M2OriginTriggered;
    Device.LimitSwitch.M2ForwardTriggered = Device_LimitSwitch_M2ForwardTriggered;
    Device.LimitSwitch.M3OriginTriggered = Device_LimitSwitch_M3OriginTriggered;
    Device.LimitSwitch.M3ForwardTriggered = Device_LimitSwitch_M3ForwardTriggered;
    
    /* UART1 */
    Device.UART1.Init = Device_UART1_Init;
    Device.UART1.SendString = Device_UART1_SendString;
    Device.UART1.Printf = Device_UART1_Printf;
    Device.UART1.Available = Device_UART1_Available;
    Device.UART1.ReadByte = Device_UART1_ReadByte;
    
    /* UART2 */
    Device.UART2.Init = Device_UART2_Init;
    Device.UART2.SendString = Device_UART2_SendString;
    Device.UART2.Printf = Device_UART2_Printf;
    Device.UART2.Available = Device_UART2_Available;
    Device.UART2.ReadByte = Device_UART2_ReadByte;

    /* UART3（串口屏） */
    Device.UART3.Init = Device_UART3_Init;
    Device.UART3.SendString = Device_UART3_SendString;
    Device.UART3.Printf = Device_UART3_Printf;
    Device.UART3.Available = Device_UART3_Available;
    Device.UART3.ReadByte = Device_UART3_ReadByte;

    /* UART4（重量传感器） */
    Device.UART4_Device.Init = Device_UART4_Init;
    Device.UART4_Device.SendString = Device_UART4_SendString;
    Device.UART4_Device.Printf = Device_UART4_Printf;
    Device.UART4_Device.Available = Device_UART4_Available;
    Device.UART4_Device.ReadByte = Device_UART4_ReadByte;
    
    /* 传感器 */
    Device.Sensor.Init = Device_Sensor_Init;
    Device.Sensor.ReadPressure = Device_Sensor_ReadPressure;
    Device.Sensor.ReadWaterLevel = Device_Sensor_ReadWaterLevel;
    Device.Sensor.ReadWeight = Device_Sensor_ReadWeight;
    
    /* LED */
    Device.LED.Init = Device_LED_Init;
    Device.LED.On = Device_LED_On;
    Device.LED.Off = Device_LED_Off;
    Device.LED.Toggle = Device_LED_Toggle;
    
    /* 工艺流程控制 */
    Device.Process_Init = Process_Init;
    Device.Process_RunSM = Process_Main_SM_Run;
    Device.Process_EmergencyStop = Process_EmergencyStop;
    
    /* 直流电机全局处理 */
    Device.DCM_ProcessAll = DCM_ProcessAll;
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
