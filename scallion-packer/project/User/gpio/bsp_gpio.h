/**
 ******************************************************************************
 * GPIO封装层 - 简化引脚操作
 *
 * ============================================================================
 * 使用示例：
 *
 * // 1. 设置引脚模式
 * GPIO_Set(PA0, MODE_OUT_PP_50M);   // PA0配置为推挽输出50MHz
 * GPIO_Set(PB1, MODE_IN_PULLUP);     // PB1配置为上拉输入
 *
 * // 2. 读取输入
 * uint8_t val = GPIO_Read(PB1);      // 读取PB1电平，返回0或1
 *
 * // 3. 写入输出
 * GPIO_Write(PA0, 1);               // PA0输出高电平
 * GPIO_Write(PA0, 0);               // PA0输出低电平
 *
 * // 4. 便捷操作
 * GPIO_SetHigh(PA0);                // PA0输出高
 * GPIO_SetLow(PA0);                 // PA0输出低
 * GPIO_Toggle(PA0);                // PA0电平翻转
 *
 * // 5. 输入检测
 * if (GPIO_IsHigh(PB1)) { }         // PB1为高电平
 * if (GPIO_IsLow(PB1)) { }         // PB1为低电平
 *
 * // 6. LED操作（PC13）
 * LED_Init();                        // 初始化LED
 * LED_On();                          // LED亮
 * LED_Off();                         // LED灭
 * LED_Toggle();                      // LED翻转
 *
 * ============================================================================
 * GPIO_Mode_TypeDef 类型说明：
 *   MODE_IN_ANALOG      = 0x00  模拟输入（ADC专用）
 *   MODE_IN_FLOATING    = 0x04  浮空输入（不确定状态）
 *   MODE_IN_PULLUP      = 0x08  上拉输入（默认高电平）
 *   MODE_IN_PULLDOWN    = 0x08  下拉输入（默认低电平）
 *   MODE_OUT_PP_10M     = 0x02  推挽输出，最大10MHz
 *   MODE_OUT_PP_2M      = 0x06  推挽输出，最大2MHz
 *   MODE_OUT_PP_50M     = 0x03  推挽输出，最大50MHz（常用）
 *   MODE_AF_PP_50M      = 0x0B  复用推挽输出，50MHz（常用）
 *
 * ============================================================================
 * 可用引脚：PA0~PA15, PB0~PB15, PC0~PC15, PD0~PD15, PE0~PE15
 *
 * ============================================================================
 * 作者：工程训练中心116电控组
 * 日期：2026-03-24
 ******************************************************************************
 */

#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#include "stm32f10x.h"

/* GPIO模式定义 - 简化版 */
typedef enum {
    /* 输入模式 */
    MODE_IN_ANALOG     = 0x00,  /* 模拟输入，用于ADC采集 */
    MODE_IN_FLOATING   = 0x04,  /* 浮空输入，电平不确定 */
    MODE_IN_PULLUP    = 0x08,  /* 上拉输入，未接信号时默认高电平 */
    MODE_IN_PULLDOWN  = 0x08,  /* 下拉输入，未接信号时默认低电平 */

    /* 输出模式 */
    MODE_OUT_PP_10M    = 0x02,  /* 推挽输出，最大10MHz，适合低速设备 */
    MODE_OUT_PP_2M    = 0x06,  /* 推挽输出，最大2MHz */
    MODE_OUT_PP_50M    = 0x03,  /* 推挽输出，最大50MHz，常用 */

    MODE_OUT_OD_10M    = 0x06,  /* 开漏输出，10MHz，可用于线与 */
    MODE_OUT_OD_50M   = 0x07,  /* 开漏输出，50MHz */

    /* 复用功能 */
    MODE_AF_PP_10M    = 0x0A,  /* 复用推挽输出，10MHz */
    MODE_AF_PP_50M    = 0x0B,  /* 复用推挽输出，50MHz，常用 */

    MODE_AF_OD_10M    = 0x0E,  /* 复用开漏输出，10MHz */
    MODE_AF_OD_50M    = 0x0F,  /* 复用开漏输出，50MHz */
} GPIO_Mode_TypeDef;

/* GPIO端口定义 */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} GPIO_PortPin_TypeDef;

/* 端口定义（必须大写，用于宏） */
#define PA0   {GPIOA, GPIO_Pin_0}
#define PA1   {GPIOA, GPIO_Pin_1}
#define PA2   {GPIOA, GPIO_Pin_2}
#define PA3   {GPIOA, GPIO_Pin_3}
#define PA4   {GPIOA, GPIO_Pin_4}
#define PA5   {GPIOA, GPIO_Pin_5}
#define PA6   {GPIOA, GPIO_Pin_6}
#define PA7   {GPIOA, GPIO_Pin_7}
#define PA8   {GPIOA, GPIO_Pin_8}
#define PA9   {GPIOA, GPIO_Pin_9}
#define PA10  {GPIOA, GPIO_Pin_10}
#define PA11  {GPIOA, GPIO_Pin_11}
#define PA12  {GPIOA, GPIO_Pin_12}
#define PA13  {GPIOA, GPIO_Pin_13}
#define PA14  {GPIOA, GPIO_Pin_14}
#define PA15  {GPIOA, GPIO_Pin_15}

#define PB0   {GPIOB, GPIO_Pin_0}
#define PB1   {GPIOB, GPIO_Pin_1}
#define PB2   {GPIOB, GPIO_Pin_2}
#define PB3   {GPIOB, GPIO_Pin_3}
#define PB4   {GPIOB, GPIO_Pin_4}
#define PB5   {GPIOB, GPIO_Pin_5}
#define PB6   {GPIOB, GPIO_Pin_6}
#define PB7   {GPIOB, GPIO_Pin_7}
#define PB8   {GPIOB, GPIO_Pin_8}
#define PB9   {GPIOB, GPIO_Pin_9}
#define PB10  {GPIOB, GPIO_Pin_10}
#define PB11  {GPIOB, GPIO_Pin_11}
#define PB12  {GPIOB, GPIO_Pin_12}
#define PB13  {GPIOB, GPIO_Pin_13}
#define PB14  {GPIOB, GPIO_Pin_14}
#define PB15  {GPIOB, GPIO_Pin_15}

#define PC0   {GPIOC, GPIO_Pin_0}
#define PC1   {GPIOC, GPIO_Pin_1}
#define PC2   {GPIOC, GPIO_Pin_2}
#define PC3   {GPIOC, GPIO_Pin_3}
#define PC4   {GPIOC, GPIO_Pin_4}
#define PC5   {GPIOC, GPIO_Pin_5}
#define PC6   {GPIOC, GPIO_Pin_6}
#define PC7   {GPIOC, GPIO_Pin_7}
#define PC8   {GPIOC, GPIO_Pin_8}
#define PC9   {GPIOC, GPIO_Pin_9}
#define PC10  {GPIOC, GPIO_Pin_10}
#define PC11  {GPIOC, GPIO_Pin_11}
#define PC12  {GPIOC, GPIO_Pin_12}
#define PC13  {GPIOC, GPIO_Pin_13}
#define PC14  {GPIOC, GPIO_Pin_14}
#define PC15  {GPIOC, GPIO_Pin_15}

#define PD0   {GPIOD, GPIO_Pin_0}
#define PD1   {GPIOD, GPIO_Pin_1}
#define PD2   {GPIOD, GPIO_Pin_2}
#define PD3   {GPIOD, GPIO_Pin_3}
#define PD4   {GPIOD, GPIO_Pin_4}
#define PD5   {GPIOD, GPIO_Pin_5}
#define PD6   {GPIOD, GPIO_Pin_6}
#define PD7   {GPIOD, GPIO_Pin_7}
#define PD8   {GPIOD, GPIO_Pin_8}
#define PD9   {GPIOD, GPIO_Pin_9}
#define PD10  {GPIOD, GPIO_Pin_10}
#define PD11  {GPIOD, GPIO_Pin_11}
#define PD12  {GPIOD, GPIO_Pin_12}
#define PD13  {GPIOD, GPIO_Pin_13}
#define PD14  {GPIOD, GPIO_Pin_14}
#define PD15  {GPIOD, GPIO_Pin_15}

/* PE端口定义（用于限位开关，避免与PC冲突） */
#define PE0   {GPIOE, GPIO_Pin_0}
#define PE1   {GPIOE, GPIO_Pin_1}
#define PE2   {GPIOE, GPIO_Pin_2}
#define PE3   {GPIOE, GPIO_Pin_3}
#define PE4   {GPIOE, GPIO_Pin_4}
#define PE5   {GPIOE, GPIO_Pin_5}
#define PE6   {GPIOE, GPIO_Pin_6}
#define PE7   {GPIOE, GPIO_Pin_7}
#define PE8   {GPIOE, GPIO_Pin_8}
#define PE9   {GPIOE, GPIO_Pin_9}
#define PE10  {GPIOE, GPIO_Pin_10}
#define PE11  {GPIOE, GPIO_Pin_11}
#define PE12  {GPIOE, GPIO_Pin_12}
#define PE13  {GPIOE, GPIO_Pin_13}
#define PE14  {GPIOE, GPIO_Pin_14}
#define PE15  {GPIOE, GPIO_Pin_15}

/* 简化操作宏 */
#define GPIO_Set(pin, mode)       GPIO_InitPin(&(pin), mode)
#define GPIO_Read(pin)             GPIO_ReadPin((pin))
#define GPIO_Write(pin, val)       GPIO_WritePin(&(pin), val)
#define GPIO_Toggle(pin)           GPIO_TogglePin(&(pin))
#define GPIO_SetHigh(pin)          GPIO_WritePin(&(pin), 1)
#define GPIO_SetLow(pin)           GPIO_WritePin(&(pin), 0)
#define GPIO_IsHigh(pin)           (GPIO_ReadPin(&(pin)) == 1)
#define GPIO_IsLow(pin)            (GPIO_ReadPin((pin)) == 0)

/* LED操作 */
#define LED_Init()                 GPIO_InitLed()
#define LED_On()                   GPIO_SetLow(LED_PIN)
#define LED_Off()                  GPIO_SetHigh(LED_PIN)
#define LED_Toggle()               GPIO_Toggle(LED_PIN)

extern GPIO_PortPin_TypeDef LED_PIN;

/* 函数声明 */
void GPIO_InitPin(const GPIO_PortPin_TypeDef* pin, GPIO_Mode_TypeDef mode);
uint8_t GPIO_ReadPin(const GPIO_PortPin_TypeDef* pin);
void GPIO_WritePin(const GPIO_PortPin_TypeDef* pin, uint8_t value);
void GPIO_TogglePin(const GPIO_PortPin_TypeDef* pin);
uint16_t GPIO_ReadPort(GPIO_TypeDef* port);
void GPIO_WritePort(GPIO_TypeDef* port, uint16_t value);
uint32_t GPIO_GetPortClock(GPIO_TypeDef* port);
void GPIO_InitLed(void);

#endif /* __BSP_GPIO_H__ */
