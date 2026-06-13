/**
 ******************************************************************************
 * 统一设备管理层
 *
 * 提供面向对象的设备访问接口，通过 Device.Run() Device.Stop() 等方式调用
 *
 * ============================================================================
 * 使用示例：
 *
 * // 1. 在 main.c 中声明设备（只需要一次）
 * #include "device.h"
 * DEVICE_t Device;
 *
 * // 2. 在系统初始化中初始化所有设备
 * void System_Init(void)
 * {
 *     Device.Init();  // 初始化所有设备
 * }
 *
 * // 3. 步进电机控制 - 使用快捷宏（推荐）
 * M1_Run(1000, DIR_FORWARD);    // 同步带正转
 * M2_Run(500, DIR_BACKWARD);    // 切割丝杆反转
 * M3_Run(800, DIR_FORWARD);     // 传送带正转
 * M4_Run(1000, DIR_FORWARD);     // 打包电机正转
 * M1_Stop();                     // 停止同步带
 *
 * // 4. 舵机控制
 * Device.Servo1.SetAngle(90);    // 舵机1转到90度
 * Device.Servo2.SetAngle(45);    // 舵机2转到45度
 *
 * // 5. 直流电机控制
 * Device.DCMotor1.Run(80, 1);   // 直流电机1正转，80%占空比
 * Device.DCMotor1.Stop();
 *
 * // 6. 限位开关检测
 * if (Device.LimitSwitch.EmergencyPressed()) {
 *     M1_Stop(); M2_Stop(); M3_Stop(); M4_Stop();
 * }
 *
 * // 7. 传感器读取
 * float pressure = Device.Sensor.Pressure;
 * float water = Device.Sensor.WaterLevel;
 * float weight = Device.Sensor.Weight;
 *
 * // 8. 串口通信
 * Device.UART1.SendString("Hello");
 * Device.UART1.Printf("Value: %d", x);
 * Device.UART3.SendString("Page1");  // 发送到串口屏
 *
 * // 9. LED控制
 * Device.LED.On();
 * Device.LED.Off();
 * Device.LED.Toggle();
 *
 * ============================================================================
 * 作者：工程训练中心116电控组
 * 日期：2026-04-04
 ******************************************************************************
 */

#ifndef __DEVICE_H__
#define __DEVICE_H__

#include "stm32f10x.h"
#include "bsp_stepper.h"
#include "bsp_servo_obj.h"
#include "bsp_servo_serial.h"
#include "bsp_dc_motor_obj.h"
#include "bsp_limit_switch.h"
#include "bsp_usart_obj.h"
#include "bsp_sensor_obj.h"
#include "bsp_tick.h"
#include "bsp_gpio.h"

/* =============================================================================
 * 子设备结构体定义
 * ============================================================================= */

/* 电机1控制器 */
typedef struct {
    void (*Init)(void);
    void (*Run)(uint16_t speed, Motor_Direction_TypeDef dir);
    void (*Stop)(void);
    int32_t (*GetPos)(void);
    void (*ResetPos)(void);
    void (*Enable)(uint8_t en);
    void (*Brake)(void);
    void (*SetSpeed)(uint16_t speed);
    Motor_State_TypeDef (*GetState)(void);
} Motor_Device_t;

/* 电机2控制器 */
typedef struct {
    void (*Init)(void);
    void (*Run)(uint16_t speed, Motor_Direction_TypeDef dir);
    void (*Stop)(void);
    int32_t (*GetPos)(void);
    void (*ResetPos)(void);
    void (*Enable)(uint8_t en);
    void (*Brake)(void);
    void (*SetSpeed)(uint16_t speed);
    Motor_State_TypeDef (*GetState)(void);
} Motor2_Device_t;

/* 电机3控制器（传送带） */
typedef struct {
    void (*Init)(void);
    void (*Run)(uint16_t speed, Motor_Direction_TypeDef dir);
    void (*Stop)(void);
    int32_t (*GetPos)(void);
    void (*ResetPos)(void);
    void (*Enable)(uint8_t en);
    void (*Brake)(void);
    void (*SetSpeed)(uint16_t speed);
    Motor_State_TypeDef (*GetState)(void);
} Motor3_Device_t;

/* 电机4控制器（打包电机） */
typedef struct {
    void (*Init)(void);
    void (*Run)(uint16_t speed, Motor_Direction_TypeDef dir);
    void (*Stop)(void);
    int32_t (*GetPos)(void);
    void (*ResetPos)(void);
    void (*Enable)(uint8_t en);
    void (*Brake)(void);
    void (*SetSpeed)(uint16_t speed);
    Motor_State_TypeDef (*GetState)(void);
} Motor4_Device_t;

/* 电机5控制器（切割丝杆A） */
typedef struct {
    void (*Init)(void);
    void (*Run)(uint16_t speed, Motor_Direction_TypeDef dir);
    void (*Stop)(void);
    int32_t (*GetPos)(void);
    void (*ResetPos)(void);
    void (*Enable)(uint8_t en);
    void (*Brake)(void);
    void (*SetSpeed)(uint16_t speed);
    Motor_State_TypeDef (*GetState)(void);
} Motor5_Device_t;

/* 电机6控制器 */
typedef struct {
    void (*Init)(void);
    void (*Run)(uint16_t speed, Motor_Direction_TypeDef dir);
    void (*Stop)(void);
    int32_t (*GetPos)(void);
    void (*ResetPos)(void);
    void (*Enable)(uint8_t en);
    void (*Brake)(void);
    void (*SetSpeed)(uint16_t speed);
    Motor_State_TypeDef (*GetState)(void);
} Motor6_Device_t;

/* 舵机1控制器 */
typedef struct {
    void (*Init)(void);
    void (*SetAngle)(uint16_t angle);
    uint16_t (*GetAngle)(void);
} Servo1_Device_t;

/* 舵机2控制器 */
typedef struct {
    void (*Init)(void);
    void (*SetAngle)(uint16_t angle);
    uint16_t (*GetAngle)(void);
} Servo2_Device_t;

/* 舵机3控制器 */
typedef struct {
    void (*Init)(void);
    void (*SetAngle)(uint16_t angle);
    uint16_t (*GetAngle)(void);
} Servo3_Device_t;

/* 舵机4控制器 */
typedef struct {
    void (*Init)(void);
    void (*SetAngle)(uint16_t angle);
    uint16_t (*GetAngle)(void);
} Servo4_Device_t;

/* 直流电机1控制器 */
typedef struct {
    void (*Init)(void);
    void (*Run)(uint8_t duty, uint8_t forward);
    void (*Stop)(void);
    void (*RunRevolutions)(float revs, uint8_t duty, uint8_t forward);
    uint8_t (*IsBusy)(void);
} DCMotor1_Device_t;

/* 直流电机2控制器 */
typedef struct {
    void (*Init)(void);
    void (*Run)(uint8_t duty, uint8_t forward);
    void (*Stop)(void);
    void (*RunRevolutions)(float revs, uint8_t duty, uint8_t forward);
    uint8_t (*IsBusy)(void);
} DCMotor2_Device_t;

/* 直流电机3控制器 */
typedef struct {
    void (*Init)(void);
    void (*Run)(uint8_t duty, uint8_t forward);
    void (*Stop)(void);
    void (*RunRevolutions)(float revs, uint8_t duty, uint8_t forward);
    uint8_t (*IsBusy)(void);
} DCMotor3_Device_t;

/* 直流电机4控制器 */
typedef struct {
    void (*Init)(void);
    void (*Run)(uint8_t duty, uint8_t forward);
    void (*Stop)(void);
    void (*RunRevolutions)(float revs, uint8_t duty, uint8_t forward);
    uint8_t (*IsBusy)(void);
} DCMotor4_Device_t;

/* 限位开关控制器 */
typedef struct {
    void (*Init)(void);
    uint8_t (*IsTriggered)(LimitSwitch_ID_TypeDef switch_id);
    uint8_t (*ReadAll)(void);
    uint8_t (*EmergencyPressed)(void);
    uint8_t (*M1OriginTriggered)(void);
    uint8_t (*M1ForwardTriggered)(void);
    uint8_t (*M2OriginTriggered)(void);
    uint8_t (*M2ForwardTriggered)(void);
    uint8_t (*M3OriginTriggered)(void);
    uint8_t (*M3ForwardTriggered)(void);
} LimitSwitch_Device_t;

/* 串口1控制器（调试串口） */
typedef struct {
    void (*Init)(void);
    void (*SendString)(const char* str);
    void (*Printf)(const char* fmt, ...);
    uint8_t (*Available)(void);
    uint8_t (*ReadByte)(void);
} UART1_Device_t;

/* 串口2控制器（通信串口/Jetson） */
typedef struct {
    void (*Init)(void);
    void (*SendString)(const char* str);
    void (*Printf)(const char* fmt, ...);
    uint8_t (*Available)(void);
    uint8_t (*ReadByte)(void);
} UART2_Device_t;

/* 串口3控制器（串口屏） */
typedef struct {
    void (*Init)(void);
    void (*SendString)(const char* str);
    void (*Printf)(const char* fmt, ...);
    uint8_t (*Available)(void);
    uint8_t (*ReadByte)(void);
} UART3_Device_t;

/* 串口4控制器（重量传感器） */
typedef struct {
    void (*Init)(void);
    void (*SendString)(const char* str);
    void (*Printf)(const char* fmt, ...);
    uint8_t (*Available)(void);
    uint8_t (*ReadByte)(void);
} UART4_Device_t;

/* 传感器控制器 */
typedef struct {
    void (*Init)(void);
    float (*ReadPressure)(void);
    float (*ReadWaterLevel)(void);
    float (*ReadWeight)(void);
    float Pressure;      /* 直接访问属性 */
    float WaterLevel;
    float Weight;
} Sensor_Device_t;

/* LED控制器 */
typedef struct {
    void (*Init)(void);
    void (*On)(void);
    void (*Off)(void);
    void (*Toggle)(void);
} LED_Device_t;

/* 定时器控制器 */
typedef struct {
    uint32_t start;       /* 上次触发时间 */
    uint32_t interval;    /* 间隔时间(ms) */
    uint8_t running;     /* 是否运行 */
} SoftTimer_t;

/* =============================================================================
 * 统一设备结构体
 * ============================================================================= */
typedef struct {
    /* 初始化 */
    void (*Init)(void);
    
    /* 系统 */
    uint32_t (*GetTick)(void);       /* 获取系统运行时间(ms) */
    uint8_t (*GetStatus)(void);       /* 获取系统状态 */
    
    /* 定时器工具 */
    void (*Timer_Start)(SoftTimer_t *timer, uint32_t interval_ms);
    uint8_t (*Timer_IsExpired)(SoftTimer_t *timer);
    void (*Timer_Reset)(SoftTimer_t *timer);
    
    /* 步进电机 */
    Motor_Device_t Motor1;
    Motor2_Device_t Motor2;
    Motor3_Device_t Motor3;
    Motor4_Device_t Motor4;
    Motor5_Device_t Motor5;
    Motor6_Device_t Motor6;

    /* 舵机 */
    Servo1_Device_t Servo1;
    Servo2_Device_t Servo2;
    Servo3_Device_t Servo3;
    Servo4_Device_t Servo4;
    
    /* 直流电机 */
    DCMotor1_Device_t DCMotor1;
    DCMotor2_Device_t DCMotor2;
    DCMotor3_Device_t DCMotor3;
    DCMotor4_Device_t DCMotor4;
    
    /* 限位开关 */
    LimitSwitch_Device_t LimitSwitch;
    
    /* 串口 */
    UART1_Device_t UART1;
    UART2_Device_t UART2;
    UART3_Device_t UART3;  /* 串口屏 */
    UART4_Device_t UART4_Device;   /* UART4：飞特 SCS 总线 / 兼容外设 */
    
    /* 传感器 */
    Sensor_Device_t Sensor;
    
    /* LED */
    LED_Device_t LED;
    
    /* 工艺流程控制 */
    void (*Process_Init)(void);
    void (*Process_RunSM)(void);
    void (*Process_EmergencyStop)(void);
    
    /* 直流电机全局处理 */
    void (*DCM_ProcessAll)(void);
    
} DEVICE_t;

/* =============================================================================
 * 函数声明
 * ============================================================================= */
void Device_InitInstance(void);

/* =============================================================================
 * 外部变量声明
 * ============================================================================= */
extern DEVICE_t Device;

#endif /* __DEVICE_H__ */

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
