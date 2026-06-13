/**
 ******************************************************************************
 * HX711 24-bit ADC 称重传感器驱动实现
 *
 * 硬件接口：
 *   PD_SCK（时钟）→ PA4（推挽输出）
 *   DOUT（数据）  → PA5（浮空输入）
 *
 * 时序（与海芯官方参考代码一致）：
 *   1. PD_SCK = LOW，等待 DOUT 从 HIGH 变为 LOW（数据就绪）
 *   2. SCK 产生上升沿，下降沿时 DOUT 数据有效，读取 1 bit
 *   3. 24 次循环读走 24 位（MSB-first）
 *   4. 第 25 个脉冲：通道 A，增益 128（默认，保持）
 *   5. SCK 高电平时间 < 50μs（避免进入断电模式， datasheet T3 max=50μs）
 *
 * 命令（UART1）：
 *   T  → 去皮（记录当前 raw 均值为零位基准）
 *   R  → 取消去皮
 *   C  → 打印配置信息
 *   K<mass>\r\n → 一键校准
 *
 * 输出格式（与 tools/serial_plotter/main.py 正则匹配）：
 *   [HX711] Weight=X.XXX kg (X.XXX g)  raw=X d=X
 *
 * 参考：海芯科技 HX711 官方数据手册 Rev 1.0
 ******************************************************************************
 */

#include "bsp_hx711.h"
#include "bsp_usart_obj.h"
#include "bsp_tick.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*-----------------------------------------------------------
 * 标定参数（引用头文件中的编译期常量）
 *-----------------------------------------------------------*/
volatile int32_t  g_hx711_raw        = 0;
volatile int32_t  g_hx711_filtered   = 0;
volatile int32_t  g_hx711_tare_raw   = 0;
volatile int32_t  g_hx711_diff       = 0;
volatile float    g_hx711_ratio_scale = HX711_KNOWN_MASS_G / (float)HX711_KNOWN_MASS_RAW_DIFF;
volatile uint8_t  g_hx711_tare_valid = 0;

/*-----------------------------------------------------------
 * 内部变量
 *-----------------------------------------------------------*/
static uint8_t  s_hx711_powered = 0;

/* 环形滤波缓冲区 */
#define HX711_BUF_SIZE  16
static int32_t  s_hx711_buf[HX711_BUF_SIZE];
static uint32_t s_hx711_buf_head = 0;
static uint8_t  s_hx711_buf_count = 0;
static uint8_t  s_hx711_buf_full  = 0;

/* UART1 命令解析 */
#define HX711_CMD_SIZE  32
static char     s_hx711_cmd_buf[HX711_CMD_SIZE];
static uint8_t  s_hx711_cmd_len   = 0;
static void HX711_ProcessCmd(const char *cmd, uint8_t len);

/*-----------------------------------------------------------
 * GPIO 初始化
 *-----------------------------------------------------------*/
void HX711_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* PA4 = SCK, PA5 = DOUT */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* PD_SCK 推挽输出 */
    GPIO_InitStructure.GPIO_Pin  = HX711_SCK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HX711_SCK_PORT, &GPIO_InitStructure);

    /* DOUT 浮空输入（datasheet 要求不接任何上下拉电阻以减少干扰） */
    GPIO_InitStructure.GPIO_Pin  = HX711_DOUT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HX711_DOUT_PORT, &GPIO_InitStructure);

    /* 初始状态：SCK=LOW */
    GPIO_ResetBits(HX711_SCK_PORT, HX711_SCK_PIN);

    s_hx711_powered = 1;

    /* 清空缓冲区 */
    memset(s_hx711_buf, 0, sizeof(s_hx711_buf));
    s_hx711_buf_head  = 0;
    s_hx711_buf_count = 0;
    s_hx711_buf_full  = 0;

    g_hx711_tare_valid = 0;
    g_hx711_tare_raw   = 0;
}

/*-----------------------------------------------------------
 * 底层时序读取
 *
 * 时序逻辑（与海芯官方参考代码一致）：
 *   - PD_SCK LOW → DOUT 从 HIGH→LOW 表示数据就绪
 *   - SCK 置高产生上升沿，置低时 DOUT 有效（下降沿采样）
 *   - 24 个脉冲读走 24 位，第 25 个脉冲选择通道 A 增益 128
 *   - SCK 高电平时间 < 50μs（datasheet T3 max），避免进入断电模式
 *
 * 返回值：
 *   成功：24-bit 有符号整数（-8388608 ~ 8388607）
 *   超时：0x7FFFFFFF
 *-----------------------------------------------------------*/
static int32_t HX711_ReadOnce(void)
{
    uint8_t i;
    int32_t val = 0;

    if (!s_hx711_powered) return 0x7FFFFFFFL;

    /* 等待 DOUT = LOW（数据就绪）
     * HX711 片内振荡器 10Hz（100ms周期）或 80Hz（12.5ms周期）
     * 超时约 200ms，留足够余量 */
    uint32_t timeout = 200000;
    while (GPIO_ReadInputDataBit(HX711_DOUT_PORT, HX711_DOUT_PIN) != Bit_RESET) {
        if (timeout-- == 0) return 0x7FFFFFFFL;
        __NOP();
    }

    /* 24 次循环：SCK 高→低，在下降沿读取 1 bit（MSB-first） */
    for (i = 0; i < 24; i++) {
        GPIO_SetBits(HX711_SCK_PORT, HX711_SCK_PIN);    /* SCK = HIGH */
        __NOP();                                          /* T2: DOUT 数据有效 >0.1μs */
        val = (val << 1);                                /* 左移一位准备接收 */
        GPIO_ResetBits(HX711_SCK_PORT, HX711_SCK_PIN);   /* SCK = LOW，产生下降沿 */
        if (GPIO_ReadInputDataBit(HX711_DOUT_PORT, HX711_DOUT_PIN) != Bit_RESET) {
            val++;                                        /* 下降沿采样：DOUT=1 则 +1 */
        }
        __NOP();                                          /* T4: SCK 低电平时间 >0.2μs */
    }

    /* 第 25 个脉冲：通道 A，增益 128（默认配置） */
    GPIO_SetBits(HX711_SCK_PORT, HX711_SCK_PIN);
    __NOP();
    GPIO_ResetBits(HX711_SCK_PORT, HX711_SCK_PIN);
    __NOP();

    /* 有符号扩展：24-bit → 32-bit
     * 参考代码使用 XOR 0x800000 翻转符号位，效果相同
     * 此处用条件判断更直观 */
    if (val & 0x800000L) {
        val |= 0xFF000000L;
    }

    return val;
}

/*-----------------------------------------------------------
 * 环形缓冲区管理
 *-----------------------------------------------------------*/
static void HX711_PushSample(int32_t raw)
{
    s_hx711_buf[s_hx711_buf_head] = raw;
    s_hx711_buf_head = (s_hx711_buf_head + 1) % HX711_BUF_SIZE;

    if (!s_hx711_buf_full) {
        if (s_hx711_buf_count < HX711_BUF_SIZE) {
            s_hx711_buf_count++;
        }
    }
    if (s_hx711_buf_count >= HX711_BUF_SIZE) {
        s_hx711_buf_full = 1;
    }
}

static int32_t HX711_GetAverage(void)
{
    int32_t sum = 0;
    uint8_t i, n;

    if (s_hx711_buf_count == 0) return 0;

    n = s_hx711_buf_count;
    for (i = 0; i < n; i++) {
        sum += s_hx711_buf[i];
    }
    return sum / (int32_t)n;
}

/*-----------------------------------------------------------
 * 公共 API
 *-----------------------------------------------------------*/
int32_t HX711_ReadRaw(void)
{
    return HX711_ReadOnce();
}

int32_t HX711_ReadFiltered(void)
{
    int32_t raw = HX711_ReadOnce();
    if (raw == 0x7FFFFFFFL) return g_hx711_filtered;

    HX711_PushSample(raw);
    return HX711_GetAverage();
}

void HX711_Tare(void)
{
    int32_t sum = 0;
    uint8_t i, cnt = 0;

    /* 连续采样 8 次取平均作为去皮基准 */
    for (i = 0; i < 8; i++) {
        int32_t r = HX711_ReadOnce();
        if (r != 0x7FFFFFFFL) {
            sum += r;
            cnt++;
        }
    }
    if (cnt == 0) {
        UART1_Printf("[HX711] Tare failed: no valid samples\r\n");
        return;
    }
    g_hx711_tare_raw   = sum / (int32_t)cnt;
    g_hx711_tare_valid = 1;

    UART1_Printf("[HX711] Tare=%ld (avg of %u samples)\r\n",
                 (long)g_hx711_tare_raw, cnt);
}

void HX711_ResetTare(void)
{
    g_hx711_tare_valid = 0;
    g_hx711_tare_raw   = 0;
    UART1_Printf("[HX711] Tare cleared\r\n");
}

int32_t HX711_GetDiff(void)
{
    return g_hx711_diff;
}

float HX711_DiffToGram(int32_t diff)
{
    return (float)diff * g_hx711_ratio_scale;
}

float HX711_GetWeightKg(void)
{
    return HX711_GetWeightGram() / 1000.0f;
}

float HX711_GetWeightGram(void)
{
    return HX711_DiffToGram(g_hx711_diff);
}

void HX711_OneClickCal(float mass_g)
{
    int32_t diff = g_hx711_diff;
    if (diff == 0) {
        UART1_Printf("[HX711] Cal failed: diff=0, please TARE first\r\n");
        return;
    }
    g_hx711_ratio_scale = mass_g / (float)diff;
    UART1_Printf(
        "[HX711] One-click cal: mass=%.2fg -> ratio=%.6f g/d\r\n",
        (double)mass_g, (double)g_hx711_ratio_scale
    );
}

void HX711_PrintConfig(void)
{
    UART1_Printf(
        "[HX711] CFG: HX711_KNOWN_MASS_G=%.2f  "
        "HX711_KNOWN_MASS_RAW_DIFF=%ld  ratio=%.6f g/d\r\n",
        (double)HX711_KNOWN_MASS_G,
        (long)HX711_KNOWN_MASS_RAW_DIFF,
        (double)g_hx711_ratio_scale
    );
    if (g_hx711_tare_valid) {
        UART1_Printf(
            "[HX711]      TARE raw=%ld\r\n", (long)g_hx711_tare_raw
        );
    } else {
        UART1_Printf("[HX711]      TARE: none\r\n");
    }
}

void HX711_PrintSample(void)
{
    int32_t raw = HX711_ReadOnce();
    if (raw == 0x7FFFFFFFL) {
        UART1_Printf("[HX711] ERR: HX711 not ready (timeout)\r\n");
        return;
    }

    g_hx711_raw = raw;

    if (g_hx711_tare_valid) {
        g_hx711_diff = raw - g_hx711_tare_raw;
    } else {
        g_hx711_diff = raw;
    }

    float wg = HX711_DiffToGram(g_hx711_diff);
    UART1_Printf(
        "[HX711] Weight=%.3f kg (%.3f g)  raw=%ld d=%ld\r\n",
        (double)(wg / 1000.0f), (double)wg,
        (long)raw, (long)g_hx711_diff
    );
}

/*-----------------------------------------------------------
 * UART1 命令解析
 *-----------------------------------------------------------*/
void HX711_OnByte(uint8_t ch)
{
    if (ch == '\r' || ch == '\n') {
        if (s_hx711_cmd_len > 0) {
            s_hx711_cmd_buf[s_hx711_cmd_len] = '\0';
            HX711_ProcessCmd(s_hx711_cmd_buf, s_hx711_cmd_len);
            s_hx711_cmd_len = 0;
        }
    } else {
        if (s_hx711_cmd_len < HX711_CMD_SIZE - 1) {
            s_hx711_cmd_buf[s_hx711_cmd_len++] = (char)ch;
        }
    }
}

static void HX711_ProcessCmd(const char *cmd, uint8_t len)
{
    if (len == 0) return;

    switch (cmd[0]) {
        case 'T':
        case 't':
            HX711_Tare();
            break;

        case 'R':
        case 'r':
            HX711_ResetTare();
            break;

        case 'C':
        case 'c':
            HX711_PrintConfig();
            break;

        case 'K':
        case 'k':
            if (len > 1) {
                float m = (float)atof(cmd + 1);
                if (m > 0.0f) {
                    HX711_OneClickCal(m);
                } else {
                    UART1_Printf("[HX711] K: invalid mass '%s'\r\n", cmd + 1);
                }
            } else {
                UART1_Printf("[HX711] Usage: K<mass>\r\n");
            }
            break;

        default:
            UART1_Printf("[HX711] Unknown cmd '%s', use T/R/C/K<mass>\r\n", cmd);
            break;
    }
}

/*-----------------------------------------------------------
 * 测试任务（供主循环调用，2Hz 打印）
 *-----------------------------------------------------------*/
void HX711_TestTask(void)
{
    static uint32_t s_last_ms = 0;
    uint32_t now = BSP_GetTickMs();

    if ((uint32_t)(now - s_last_ms) < 500U) return;
    s_last_ms = now;

    int32_t raw = HX711_ReadOnce();
    if (raw == 0x7FFFFFFFL) {
        return;
    }

    g_hx711_raw = raw;

    if (g_hx711_tare_valid) {
        g_hx711_diff = raw - g_hx711_tare_raw;
    } else {
        g_hx711_diff = raw;
    }

    float wg = HX711_DiffToGram(g_hx711_diff);

    UART1_Printf(
        "[HX711] Weight=%.3f kg (%.3f g)  raw=%ld d=%ld\r\n",
        (double)(wg / 1000.0f),
        (double)wg,
        (long)raw,
        (long)g_hx711_diff
    );
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
