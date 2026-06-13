/**
 ******************************************************************************
 * 限位开关驱动 - 引脚已修改为PE端口，避免与直流电机TIM8 PWM冲突
 *
 * STM32F103VE 引脚分配（修改后）：
 * PE0  - 电机1原点限位
 * PE1  - 电机1正限位
 * PE2  - 电机2原点限位
 * PE3  - 电机2正限位
 * PE4  - 电机3原点限位
 * PE5  - 电机3正限位
 * PE6  - 预留急停
 *
 * 作者：电控组
 * 日期：2026-03-24
 ******************************************************************************
 */

#include "stm32f10x.h"
#include "bsp_limit_switch.h"
#include "bsp_stepper.h"
#include <stddef.h>

static uint32_t trigger_count[LIMIT_MAX] = {0};

static LimitSwitch_Config_TypeDef limit_switch_m1_origin = {
    LIMIT_M1_ORIGIN,
    GPIOE,
    GPIO_Pin_0,
    0,
    LIMIT_NOT_TRIGGERED
};

static LimitSwitch_Config_TypeDef limit_switch_m1_forward = {
    LIMIT_M1_FORWARD,
    GPIOE,
    GPIO_Pin_1,
    0,
    LIMIT_NOT_TRIGGERED
};

static LimitSwitch_Config_TypeDef limit_switch_m2_origin = {
    LIMIT_M2_ORIGIN,
    GPIOE,
    GPIO_Pin_2,
    0,
    LIMIT_NOT_TRIGGERED
};

static LimitSwitch_Config_TypeDef limit_switch_m2_forward = {
    LIMIT_M2_FORWARD,
    GPIOE,
    GPIO_Pin_3,
    0,
    LIMIT_NOT_TRIGGERED
};

static LimitSwitch_Config_TypeDef limit_switch_m3_origin = {
    LIMIT_M3_ORIGIN,
    GPIOE,
    GPIO_Pin_4,
    0,
    LIMIT_NOT_TRIGGERED
};

static LimitSwitch_Config_TypeDef limit_switch_m3_forward = {
    LIMIT_M3_FORWARD,
    GPIOE,
    GPIO_Pin_5,
    0,
    LIMIT_NOT_TRIGGERED
};

static LimitSwitch_Config_TypeDef limit_switch_emergency = {
    LIMIT_EMERGENCY,
    GPIOE,
    GPIO_Pin_6,
    0,
    LIMIT_NOT_TRIGGERED
};

static LimitSwitch_Config_TypeDef *s_limit_configs[LIMIT_MAX] = {
    &limit_switch_m1_origin,
    &limit_switch_m1_forward,
    &limit_switch_m2_origin,
    &limit_switch_m2_forward,
    &limit_switch_m3_origin,
    &limit_switch_m3_forward,
    &limit_switch_emergency
};

LimitSwitch_Config_TypeDef *LimitSwitch_GetConfig(LimitSwitch_ID_TypeDef switch_id)
{
    if (switch_id >= LIMIT_MAX) return NULL;
    return s_limit_configs[switch_id];
}

void LimitSwitch_Init(void)
{
    uint8_t i;
    GPIO_InitTypeDef g;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

    g.GPIO_Mode = GPIO_Mode_IPU;
    g.GPIO_Speed = GPIO_Speed_50MHz;

    for (i = 0; i < (uint8_t)LIMIT_MAX; i++)
    {
        g.GPIO_Pin = s_limit_configs[i]->pin;
        GPIO_Init(s_limit_configs[i]->port, &g);
    }
}

LimitSwitch_State_TypeDef LimitSwitch_Read(LimitSwitch_ID_TypeDef switch_id)
{
    LimitSwitch_Config_TypeDef *limit = LimitSwitch_GetConfig(switch_id);
    uint8_t pin_state;

    if (limit == NULL) return LIMIT_NOT_TRIGGERED;

    pin_state = GPIO_ReadInputDataBit(limit->port, limit->pin);
    if (pin_state == limit->active_level)
        limit->state = LIMIT_TRIGGERED;
    else
        limit->state = LIMIT_NOT_TRIGGERED;

    return limit->state;
}

uint8_t LimitSwitch_ReadAll(void)
{
    uint8_t status = 0;
    uint8_t i;

    for (i = 0; i < LIMIT_MAX; i++)
    {
        if (LimitSwitch_Read((LimitSwitch_ID_TypeDef)i) == LIMIT_TRIGGERED)
            status |= (1 << i);
    }
    return status;
}

uint8_t LimitSwitch_IsTriggered(LimitSwitch_ID_TypeDef switch_id)
{
    return (LimitSwitch_Read(switch_id) == LIMIT_TRIGGERED) ? 1 : 0;
}

void LimitSwitch_ClearState(LimitSwitch_ID_TypeDef switch_id)
{
    LimitSwitch_Config_TypeDef *limit = LimitSwitch_GetConfig(switch_id);
    if (limit != NULL)
        limit->state = LIMIT_NOT_TRIGGERED;
}

void LimitSwitch_ClearAll(void)
{
    uint8_t i;
    for (i = 0; i < LIMIT_MAX; i++)
        LimitSwitch_ClearState((LimitSwitch_ID_TypeDef)i);
}

void LimitSwitch_IRQHandler(LimitSwitch_ID_TypeDef switch_id)
{
    LimitSwitch_Config_TypeDef *limit = LimitSwitch_GetConfig(switch_id);
    if (limit == NULL) return;

    LimitSwitch_Read(switch_id);

    if (limit->state == LIMIT_TRIGGERED)
    {
        trigger_count[switch_id]++;

        switch (switch_id)
        {
            case LIMIT_M1_ORIGIN:
            case LIMIT_M1_FORWARD:
                Motor_Stop(MOTOR_1);
                Motor_ResetPosition(MOTOR_1);
                break;
            case LIMIT_M2_ORIGIN:
            case LIMIT_M2_FORWARD:
                Motor_Stop(MOTOR_2);
                Motor_ResetPosition(MOTOR_2);
                break;
            case LIMIT_M3_ORIGIN:
            case LIMIT_M3_FORWARD:
                Motor_Stop(MOTOR_3);
                Motor_ResetPosition(MOTOR_3);
                break;
            case LIMIT_EMERGENCY:
                Motor_Stop(MOTOR_1);
                Motor_Stop(MOTOR_2);
                Motor_Stop(MOTOR_3);
                break;
            default:
                break;
        }
    }
}

uint32_t LimitSwitch_GetTriggerCount(LimitSwitch_ID_TypeDef switch_id)
{
    if (switch_id >= LIMIT_MAX) return 0;
    return trigger_count[switch_id];
}

uint8_t LimitSwitch_EmergencyPressed(void)
{
    return LimitSwitch_IsTriggered(LIMIT_EMERGENCY);
}

uint8_t LimitSwitch_HomeReached(uint8_t motor_id)
{
    switch (motor_id)
    {
        case 1: return LimitSwitch_IsTriggered(LIMIT_M1_ORIGIN);
        case 2: return LimitSwitch_IsTriggered(LIMIT_M2_ORIGIN);
        case 3: return LimitSwitch_IsTriggered(LIMIT_M3_ORIGIN);
        default: return 0;
    }
}

uint8_t LimitSwitch_ForwardLimitReached(uint8_t motor_id)
{
    switch (motor_id)
    {
        case 1: return LimitSwitch_IsTriggered(LIMIT_M1_FORWARD);
        case 2: return LimitSwitch_IsTriggered(LIMIT_M2_FORWARD);
        case 3: return LimitSwitch_IsTriggered(LIMIT_M3_FORWARD);
        default: return 0;
    }
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
