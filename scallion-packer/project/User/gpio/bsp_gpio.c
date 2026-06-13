/**
 ******************************************************************************
 * GPIO封装层实现 - 简化引脚操作
 * 作者：电控组
 * 日期：2026-03-23
 ******************************************************************************
 */

#include "bsp_gpio.h"

/* LED引脚定义 */
GPIO_PortPin_TypeDef LED_PIN = PC13;

/* GPIO端口时钟映射表 */
typedef struct {
    GPIO_TypeDef* port;
    uint32_t clock;
} GPIO_ClockMap_TypeDef;

static const GPIO_ClockMap_TypeDef g_gpio_clock_map[] = {
    {GPIOA, RCC_APB2Periph_GPIOA},
    {GPIOB, RCC_APB2Periph_GPIOB},
    {GPIOC, RCC_APB2Periph_GPIOC},
    {GPIOD, RCC_APB2Periph_GPIOD},
    {GPIOE, RCC_APB2Periph_GPIOE},
};

/* 端口数量 */
#define GPIO_PORT_COUNT (sizeof(g_gpio_clock_map) / sizeof(GPIO_ClockMap_TypeDef))

/* =============================================================================
 * 内部函数实现
 * ============================================================================= */

/**
 * 获取端口对应的时钟
 */
uint32_t GPIO_GetPortClock(GPIO_TypeDef* port)
{
    uint8_t i;
    for (i = 0; i < GPIO_PORT_COUNT; i++) {
        if (g_gpio_clock_map[i].port == port) {
            return g_gpio_clock_map[i].clock;
        }
    }
    return 0;
}

/**
 * 初始化单个引脚
 */
void GPIO_InitPin(const GPIO_PortPin_TypeDef* pin, GPIO_Mode_TypeDef mode)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint32_t clock;

    /* 获取并使能时钟 */
    clock = GPIO_GetPortClock(pin->port);
    if (clock == 0) {
        return;
    }
    RCC_APB2PeriphClockCmd(clock, ENABLE);

    /* 配置GPIO模式
     * mode的高2位决定速度(CNF的bit1)，低2位决定模式(CNF的bit0和MODE)
     * 重新计算: mode的高2位CNF[1], 低位 = MODE[1:0]<<2 | CNF[0]
     */
    if (mode <= MODE_IN_ANALOG) {
        /* 输入模式 */
        GPIO_InitStructure.GPIO_Mode = (GPIOMode_TypeDef)mode;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    } else if (mode <= MODE_IN_PULLDOWN) {
        /* 输入模式 - 上下拉 */
        if (mode == MODE_IN_PULLUP) {
            /* 上拉输入: 浮空输入 + ODR置位 */
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        } else {
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
        }
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    } else {
        /* 输出模式 - 解析速度位 */
        switch ((mode >> 2) & 0x03) {
            case 0:
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
                break;
            case 1:
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
                break;
            case 2:
            case 3:
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                break;
        }
        /* 解析MODE_*编码：bit3-2决定类型(CNF)，bit1-0决定速度(MODE) */
        switch ((mode >> 2) & 0x03) {
            case 0:
            case 1:
                /* 输入模式 */
                if (mode == MODE_IN_PULLUP) {
                    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
                } else if (mode == MODE_IN_PULLDOWN) {
                    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
                } else {
                    GPIO_InitStructure.GPIO_Mode = (GPIOMode_TypeDef)mode;
                }
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
                break;
            case 2:
                /* 普通输出模式（MODE_OUT_*） */
                if ((mode & 0x03) == 2) {
                    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  /* 推挽 */
                } else {
                    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  /* 开漏 */
                }
                /* 根据bit1:0设置速度 */
                switch (mode & 0x03) {
                    case 0: GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; break;
                    case 1: GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; break;
                    case 2:
                    case 3: GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; break;
                }
                break;
            case 3:
                /* 复用功能模式（MODE_AF_*） */
                if ((mode & 0x03) == 2) {
                    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  /* 复用推挽 */
                } else {
                    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;  /* 复用开漏 */
                }
                /* 根据bit1:0设置速度 */
                switch (mode & 0x03) {
                    case 0: GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; break;
                    case 1: GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; break;
                    case 2:
                    case 3: GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; break;
                }
                break;
        }
    }

    GPIO_InitStructure.GPIO_Pin = pin->pin;
    GPIO_Init(pin->port, &GPIO_InitStructure);

    /* 如果是上拉模式，需要设置ODR */
    if (mode == MODE_IN_PULLUP) {
        pin->port->BSRR = pin->pin;
    }
}

/**
 * 读取引脚电平
 */
uint8_t GPIO_ReadPin(const GPIO_PortPin_TypeDef* pin)
{
    return (uint8_t)((pin->port->IDR & pin->pin) != 0);
}

/**
 * 写入引脚电平
 */
void GPIO_WritePin(const GPIO_PortPin_TypeDef* pin, uint8_t value)
{
    if (value) {
        pin->port->BSRR = pin->pin;
    } else {
        pin->port->BRR = pin->pin;
    }
}

/**
 * 翻转引脚电平
 */
void GPIO_TogglePin(const GPIO_PortPin_TypeDef* pin)
{
    if ((pin->port->ODR & pin->pin) != 0) {
        pin->port->BRR = pin->pin;
    } else {
        pin->port->BSRR = pin->pin;
    }
}

/**
 * 读取整个端口
 */
uint16_t GPIO_ReadPort(GPIO_TypeDef* port)
{
    return (uint16_t)(port->IDR);
}

/**
 * 写入整个端口
 */
void GPIO_WritePort(GPIO_TypeDef* port, uint16_t value)
{
    port->ODR = value;
}

/**
 * LED初始化（PC13）
 */
void GPIO_InitLed(void)
{
    /* 使能GPIOC时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    LED_PIN.port = GPIOC;
    LED_PIN.pin = GPIO_Pin_13;

    /* PC13配置为推挽输出 */
    GPIO_Set(LED_PIN, MODE_OUT_PP_50M);
    GPIO_SetHigh(LED_PIN);
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
