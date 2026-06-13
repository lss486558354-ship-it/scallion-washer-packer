/**
 ******************************************************************************
 * 继电器驱动实现
 *
 * ============================================================================
 * 硬件配置：
 *   K_AIR  - PB12  - GPIO输出，推挽
 *   K_PUMP - PB13  - GPIO输出，推挽
 *   K_S    - PA15  - GPIO输出，推挽（须关闭JTAG）
 *
 * 注意：PA15 在 AFIO 复位后默认复用为 JTDI，须通过
 *       AFIO->MAPR 的 SWJ_CFG 位关闭 JTAG-DP，仅保留 SW-DP。
 * ============================================================================
 *
 * 作者：工程训练中心116电控组
 * 日期：2026-04-19
 ******************************************************************************
 */

#include "bsp_relay.h"

/* ============================================================================
 * 内部配置表
 * ============================================================================ */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} Relay_Config_TypeDef;

static const Relay_Config_TypeDef g_relay_config[RELAY_MAX] = {
    [RELAY_K_AIR]  = {GPIOB, GPIO_Pin_12},   /* K_AIR  - PB12 */
    [RELAY_K_PUMP] = {GPIOB, GPIO_Pin_13},   /* K_PUMP - PB13 */
    [RELAY_K_S]    = {GPIOA, GPIO_Pin_15},    /* K_S    - PA15 */
    [RELAY_FAN_E6] = {GPIOE, GPIO_Pin_6},    /* FAN_E6 - PE6  */
};

/* 当前继电器状态（0=关闭，1=打开） */
static uint8_t s_relay_state[RELAY_MAX] = {0};

/* ============================================================================
 * 初始化所有继电器
 * ============================================================================ */
void Relay_InitAll(void)
{
    GPIO_InitTypeDef g;

    /* 使能 GPIOA/GPIOB/GPIOE 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

    /* 关闭 JTAG-DP，仅保留 SW-DP（使 PA15 可用作普通 GPIO）
     * SWJ_CFG[2:0] = 010b
     * 0x070 = 0b111 << 24
     */
    AFIO->MAPR &= ~(uint32_t)(0x07 << 24);
    AFIO->MAPR |=  (uint32_t)(0x02 << 24);  /* SWJ_CFG = 010 */

    /* 配置所有继电器引脚为推挽输出，初始状态关闭 */
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;

    uint8_t i;
    for (i = 0; i < (uint8_t)RELAY_MAX; i++) {
        g.GPIO_Pin = g_relay_config[i].pin;
        GPIO_Init(g_relay_config[i].port, &g);
        /* 上电默认关闭继电器 */
        GPIO_ResetBits(g_relay_config[i].port, g_relay_config[i].pin);
        s_relay_state[i] = 0;
    }
}

/* ============================================================================
 * 打开继电器
 * ============================================================================ */
void Relay_On(Relay_ID_TypeDef relay)
{
    if (relay >= RELAY_MAX) return;
    GPIO_SetBits(g_relay_config[relay].port, g_relay_config[relay].pin);
    s_relay_state[relay] = 1;
}

/* ============================================================================
 * 关闭继电器
 * ============================================================================ */
void Relay_Off(Relay_ID_TypeDef relay)
{
    if (relay >= RELAY_MAX) return;
    GPIO_ResetBits(g_relay_config[relay].port, g_relay_config[relay].pin);
    s_relay_state[relay] = 0;
}

/* ============================================================================
 * 切换继电器状态
 * ============================================================================ */
void Relay_Toggle(Relay_ID_TypeDef relay)
{
    if (relay >= RELAY_MAX) return;
    if (s_relay_state[relay]) {
        Relay_Off(relay);
    } else {
        Relay_On(relay);
    }
}

/* ============================================================================
 * 获取继电器状态
 * ============================================================================ */
uint8_t Relay_GetState(Relay_ID_TypeDef relay)
{
    if (relay >= RELAY_MAX) return 0;
    return s_relay_state[relay];
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE*****/
