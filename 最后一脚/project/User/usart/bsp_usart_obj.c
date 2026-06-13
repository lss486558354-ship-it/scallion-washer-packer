/**
 ******************************************************************************
 * 串口对象化封装实现
 * 作者：电控组
 * 日期：2026-03-23
 *
 * 硬件配置：
 *   UART1: PA9(TX) / PA10(RX) - 调试串口
 *   UART2: PA2(TX) / PA3(RX)  - Jetson通信
 *   UART3: PB10(TX) / PB11(RX) - 串口屏
 *
 * 默认波特率：115200
 ******************************************************************************
 */

#include "bsp_usart_obj.h"
#include "stm32f10x_usart.h"
#include <string.h>

/* UART1 调试口：在裸机单线程模式下无需互斥锁，直接发送 */
#define UART1_TX_LOCK   0

/* =============================================================================
 * 内部宏定义
 * ============================================================================= */

#define UART1_BAUDRATE  115200  /* 调试串口（HX711 等）；串口助手请同步115200 */
#define UART2_BAUDRATE  115200
#define UART3_BAUDRATE    115200
#define UART4_BAUDRATE    115200 /* 飞特 SCS 串口舵机常用波特率（与 FT 上位机默认一致；若改过舵机波特率请同步修改） */

/* =============================================================================
 * 串口配置表
 * ============================================================================= */

typedef struct {
    USART_TypeDef* usart;
    uint32_t usart_clock;
    uint32_t baudrate;
    uint8_t nvic_irq;
    uint8_t nvic_priority;
    uint8_t nvic_subpriority;
    GPIO_PortPin_TypeDef tx_pin;
    GPIO_PortPin_TypeDef rx_pin;
    uint32_t gpio_clock;
} UART_Config_TypeDef;

static const UART_Config_TypeDef g_uart_config[UART_MAX] = {
    /* UART1: PA9/PA10 */
    {
        USART1,
        RCC_APB2Periph_USART1,
        UART1_BAUDRATE,
        USART1_IRQn,
        1,  /* 抢占优先级 */
        0,  /* 子优先级 */
        {GPIOA, GPIO_Pin_9},
        {GPIOA, GPIO_Pin_10},
        RCC_APB2Periph_GPIOA
    },
    /* UART2: PA2/PA3 */
    {
        USART2,
        RCC_APB1Periph_USART2,
        UART2_BAUDRATE,
        USART2_IRQn,
        1,
        1,
        {GPIOA, GPIO_Pin_2},
        {GPIOA, GPIO_Pin_3},
        RCC_APB2Periph_GPIOA
    },
    /* UART3: PB10/PB11 */
    {
        USART3,
        RCC_APB1Periph_USART3,
        115200, /* 串口屏波特率（115200，与 TJC/Nextion 串口屏配置一致） */
        USART3_IRQn,
        1,
        2,
        {GPIOB, GPIO_Pin_10},
        {GPIOB, GPIO_Pin_11},
        RCC_APB2Periph_GPIOB
    },
    /* UART4: PC10(TX)/PC11(RX) - 飞特 SCS 总线舵机 */
    {
        UART4,
        RCC_APB1Periph_UART4,
        UART4_BAUDRATE,
        UART4_IRQn,
        1,
        3,
        {GPIOC, GPIO_Pin_10},
        {GPIOC, GPIO_Pin_11},
        RCC_APB2Periph_GPIOC
    }
};

/* =============================================================================
 * 内部函数声明
 * ============================================================================= */

static void UART_InitImpl(UART_t* self);
static void UART_SendByteImpl(UART_t* self, uint8_t byte);
static void UART_SendStringImpl(UART_t* self, char* str);
static void UART_SendDataImpl(UART_t* self, uint8_t* data, uint16_t len);
static uint8_t UART_ReadByteImpl(UART_t* self);
static uint16_t UART_AvailableImpl(UART_t* self);
static void UART_ClearBufImpl(UART_t* self);
static void UART_PrintfImpl(UART_t* self, char* fmt, ...);
/* =============================================================================
 * UTF-8 → GBK 精准转换表
 * 仅覆盖源码中实际可能发送的中文字符（ASCII直接透传不查表）
 * 按Unicode码点升序排列，支持二分查找
 * ============================================================================= */

typedef struct {
    uint16_t unicode;
    uint8_t gbk_high;
    uint8_t gbk_low;
} UTF8_GBK_Map_Entry;

static const UTF8_GBK_Map_Entry g_utf8_gbk_map[] = {
    /* 中文字符（按Unicode排序，覆盖源码中所有中文） */
    {0x4E00, 0xD2, 0xBB},  /* 一 */
    {0x4EFB, 0xC8, 0xCE},  /* 任 */
    {0x4F4D, 0xCE, 0xBB},  /* 位 */
    {0x4F53, 0xCC, 0xE5},  /* 体 */
    {0x505C, 0xCD, 0xA3},  /* 停 */
    {0x5165, 0xC8, 0xEB},  /* 入 */
    {0x51FA, 0xB3, 0xF6},  /* 出 */
    {0x5207, 0xC7, 0xD0},  /* 切 */
    {0x521D, 0xB3, 0xF5},  /* 初 */
    {0x5230, 0xB5, 0xBD},  /* 到 */
    {0x5272, 0xB8, 0xEE},  /* 割 */
    {0x529F, 0xB9, 0xA6},  /* 功 */
    {0x52A1, 0xCE, 0xF1},  /* 务 */
    {0x52A8, 0xB6, 0xAF},  /* 动 */
    {0x5305, 0xB0, 0xFC},  /* 包 */
    {0x5316, 0xBB, 0xAF},  /* 化 */
    {0x53D1, 0xB7, 0xA2},  /* 发 */
    {0x542F, 0xC6, 0xF4},  /* 启 */
    {0x544A, 0xB8, 0xE6},  /* 告 */
    {0x590D, 0xB8, 0xB4},  /* 复 */
    {0x5927, 0xB4, 0xF3},  /* 大 */
    {0x5931, 0xCA, 0xA7},  /* 失 */
    {0x59CB, 0xCA, 0xBC},  /* 始 */
    {0x5B8C, 0xCD, 0xEA},  /* 完 */
    {0x6210, 0xB3, 0xC9},  /* 成 */
    {0x6253, 0xB4, 0xF2},  /* 打 */
    {0x673A, 0xBB, 0xFA},  /* 机 */
    {0x6B62, 0xD6, 0xB9},  /* 止 */
    {0x6D17, 0xCF, 0xB4},  /* 洗 */
    {0x6D4B, 0xB2, 0xE2},  /* 测 */
    {0x6E05, 0xC7, 0xE5},  /* 清 */
    {0x8471, 0xB4, 0xD0},  /* 葱 */
    {0x89E6, 0xB4, 0xA5},  /* 触 */
    {0x8B66, 0xBE, 0xAF},  /* 警 */
    {0x8BEF, 0xCE, 0xF3},  /* 误 */
    {0x8D25, 0xB0, 0xDC},  /* 败 */
    {0x8DDD, 0xBE, 0xE0},  /* 距 */
    {0x8FBE, 0xB4, 0xEF},  /* 达 */
    {0x8FDB, 0xBD, 0xF8},  /* 进 */
    {0x9000, 0xCD, 0xCB},  /* 退 */
    {0x9519, 0xB4, 0xED},  /* 错 */
    {0x9650, 0xCF, 0xDE},  /* 限 */
};

/* 转换函数声明（实现见下方） */
static uint16_t UART_UTF8_to_GBK(char* dst, const char* src, uint16_t dst_size);

/* =============================================================================
 * 初始化单个串口
 * ============================================================================= */

static void UART_InitSingle(uint8_t id)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    UART_Config_TypeDef* cfg = (UART_Config_TypeDef*)&g_uart_config[id];

    /* 使能GPIO和时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | cfg->gpio_clock, ENABLE);
    if (cfg->usart == USART1) {
        RCC_APB2PeriphClockCmd(cfg->usart_clock, ENABLE);
    } else {
        RCC_APB1PeriphClockCmd(cfg->usart_clock, ENABLE);
    }

    if (cfg->usart == UART4 && UART4_SCS_HALF_DUPLEX) {
        /* 半双工：仅 TX 脚参与 USART，PC11 作高阻避免与单线总线争用 */
        GPIO_InitStructure.GPIO_Pin = cfg->tx_pin.pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(cfg->tx_pin.port, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = cfg->rx_pin.pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(cfg->rx_pin.port, &GPIO_InitStructure);
    } else {
        /* 配置TX引脚：复用推挽输出 */
        GPIO_InitStructure.GPIO_Pin = cfg->tx_pin.pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(cfg->tx_pin.port, &GPIO_InitStructure);

        /* 配置RX引脚：浮空输入 */
        GPIO_InitStructure.GPIO_Pin = cfg->rx_pin.pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(cfg->rx_pin.port, &GPIO_InitStructure);
    }

    /* UART4 与 USART2/3 一样挂在 APB1，必须用 USART_Init 按 PCLK1 计算 BRR。
     * 旧代码写 SystemCoreClock/baud 会得到约2 倍于正确的 BRR，实际波特率约一半（如 57600），舵机在 115200 下无响应。 */
    USART_InitStructure.USART_BaudRate = cfg->baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(cfg->usart, &USART_InitStructure);

    if (cfg->usart == UART4 && UART4_SCS_HALF_DUPLEX) {
        USART_HalfDuplexCmd(UART4, ENABLE);
    }

    /* NVIC配置 */
    NVIC_InitStructure.NVIC_IRQChannel = cfg->nvic_irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = cfg->nvic_priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = cfg->nvic_subpriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 使能接收中断与 USART */
    USART_ITConfig(cfg->usart, USART_IT_RXNE, ENABLE);
    USART_Cmd(cfg->usart, ENABLE);
}

/* =============================================================================
 * 写入环形缓冲区
 * ============================================================================= */

void UART_ISR_FeedRx(UART_ID_TypeDef id, uint8_t byte)
{
    UART_RingBuf_TypeDef* buf;

    if (id >= UART_MAX) {
        return;
    }
    buf = &UART[id].rx_buf;
    if (buf->full) {
        return;
    }
    buf->buf[buf->tail] = byte;
    buf->tail = (buf->tail + 1) % UART_RX_BUF_SIZE;
    buf->full = (buf->tail == buf->head);
}

static void UART_WriteBuf(UART_t* self, uint8_t byte)
{
    UART_ISR_FeedRx(self->id, byte);
}

/* =============================================================================
 * 对象方法实现
 * ============================================================================= */

static void UART_InitImpl(UART_t* self)
{
    uint8_t id = (uint8_t)self->id;

    /* 复制配置（含 UART4，供 SendByte 等与 USART 共用 API） */
    self->usart = g_uart_config[id].usart;
    self->baudrate = g_uart_config[id].baudrate;
    self->pins.tx_pin = g_uart_config[id].tx_pin;
    self->pins.rx_pin = g_uart_config[id].rx_pin;

    /* 初始化接收缓冲区 */
    self->rx_buf.head = 0;
    self->rx_buf.tail = 0;
    self->rx_buf.full = 0;

    /* 发送标志清零 */
    self->tx_busy = 0;

    /* 初始化硬件 */
    UART_InitSingle(id);
}

static void UART_SendByteImpl(UART_t* self, uint8_t byte)
{
    USART_TypeDef* usart = self->usart;

    while (USART_GetFlagStatus(usart, USART_FLAG_TXE) == RESET);
    USART_SendData(usart, byte);
    while (USART_GetFlagStatus(usart, USART_FLAG_TC) == RESET);
}

static void UART_SendStringImpl(UART_t* self, char* str)
{
    char gbk_buf[256];
    char* p;

    /* UART1: 使用队列发送，不再抢占
     * 注意：GBK转换需要完整字符串，不能逐字节转换
     * 这里直接发送UTF-8（串口屏支持），避免队列问题 */
    UART_UTF8_to_GBK(gbk_buf, str, sizeof(gbk_buf));
    for (p = gbk_buf; *p != '\0'; p++) {
        self->SendByte(self, (uint8_t)*p);
    }
}

static void UART_SendDataImpl(UART_t* self, uint8_t* data, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++) {
        self->SendByte(self, data[i]);
    }
}

/* =============================================================================
 * UTF-8 -> GBK 转换实现
 * 使用二分查找，最多 ~log2(55) ≈ 6 次比较
 * ============================================================================= */

static uint16_t UART_UTF8_to_GBK(char* dst, const char* src, uint16_t dst_size)
{
    uint16_t si = 0, di = 0;
    uint16_t lo, hi, mid;

    while (src[si] != '\0' && di + 1 < dst_size) {
        /* ASCII: 直接透传 */
        if ((uint8_t)src[si] < 0x80) {
            dst[di++] = src[si++];
            continue;
        }

        /* UTF-8 3字节汉字: 0xE4xx 0x9Bxx ~ 0xE9xx 0xBExx */
        if (((uint8_t)src[si] & 0xF0) == 0xE0 &&
            ((uint8_t)src[si+1] & 0xC0) == 0x80 &&
            ((uint8_t)src[si+2] & 0xC0) == 0x80) {

            uint16_t uc = ((uint16_t)(src[si]   & 0x0F) << 12) |
                          ((uint16_t)(src[si+1] & 0x3F) <<  6) |
                           (uint16_t)(src[si+2] & 0x3F);

            /* 二分查找 Unicode->GBK 转换表 */
            lo = 0;
            hi = sizeof(g_utf8_gbk_map) / sizeof(g_utf8_gbk_map[0]);
            while (lo < hi) {
                mid = (lo + hi) >> 1;
                if (g_utf8_gbk_map[mid].unicode < uc) {
                    lo = mid + 1;
                } else if (g_utf8_gbk_map[mid].unicode > uc) {
                    hi = mid;
                } else {
                    dst[di++] = (char)g_utf8_gbk_map[mid].gbk_high;
                    dst[di++] = (char)g_utf8_gbk_map[mid].gbk_low;
                    break;
                }
            }
            /* 找不到则丢弃该汉字（避免乱码扩散） */
            si += 3;
            continue;
        }

        /* 不识别的序列：跳过 */
        si++;
    }

    dst[di] = '\0';
    return di;
}

static uint8_t UART_ReadByteImpl(UART_t* self)
{
    UART_RingBuf_TypeDef* buf = &self->rx_buf;
    uint8_t byte = 0;

    /* 缓冲区为空则返回0 */
    if ((buf->head == buf->tail) && !buf->full) {
        return 0;
    }

    byte = buf->buf[buf->head];
    buf->head = (buf->head + 1) % UART_RX_BUF_SIZE;
    buf->full = 0;

    return byte;
}

static uint16_t UART_AvailableImpl(UART_t* self)
{
    UART_RingBuf_TypeDef* buf = &self->rx_buf;

    if (buf->full) {
        return UART_RX_BUF_SIZE;
    }

    if (buf->head <= buf->tail) {
        return buf->tail - buf->head;
    } else {
        return UART_RX_BUF_SIZE - buf->head + buf->tail;
    }
}

static void UART_ClearBufImpl(UART_t* self)
{
    UART_RingBuf_TypeDef* buf = &self->rx_buf;

    buf->head = 0;
    buf->tail = 0;
    buf->full = 0;
}

static void UART_PrintfImpl(UART_t* self, char* fmt, ...)
{
    char buffer[256];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(buffer, sizeof(buffer), fmt, ap);
    va_end(ap);

    UART_SendStringImpl(self, buffer);
}

/* =============================================================================
 * 串口对象实例
 * ============================================================================= */

UART_t UART[UART_MAX];

/* 初始化所有串口 */
void UART_InitAll(void)
{
    uint8_t i;
    for (i = 0; i < UART_MAX; i++) {
        /* 基本属性 */
        UART[i].id = (UART_ID_TypeDef)i;

        /* 绑定函数指针 */
        UART[i].Init = UART_InitImpl;
        UART[i].SendByte = UART_SendByteImpl;
        UART[i].SendString = UART_SendStringImpl;
        UART[i].SendData = UART_SendDataImpl;
        UART[i].ReadByte = UART_ReadByteImpl;
        UART[i].Available = UART_AvailableImpl;
        UART[i].ClearBuf = UART_ClearBufImpl;
        UART[i].Printf = UART_PrintfImpl;

        /* 初始化硬件 */
        UART[i].Init(&UART[i]);
    }
}

void UART4_InitOnly(void)
{
    UART_t* u = &UART[UART4_ID];

    u->id = UART4_ID;
    u->Init = UART_InitImpl;
    u->SendByte = UART_SendByteImpl;
    u->SendString = UART_SendStringImpl;
    u->SendData = UART_SendDataImpl;
    u->ReadByte = UART_ReadByteImpl;
    u->Available = UART_AvailableImpl;
    u->ClearBuf = UART_ClearBufImpl;
    u->Printf = UART_PrintfImpl;
    u->Init(u);
}

/* =============================================================================
 * 中断处理函数（Rx/Tx 均转到应用层可覆盖的弱回调，便于 main.c 集中写协议）
 * ============================================================================= */

__weak void USART1_HW_OnRxByte(uint8_t b)
{
    UART_ISR_FeedRx(UART1, b);
}

__weak void USART2_HW_OnRxByte(uint8_t b)
{
    UART_ISR_FeedRx(UART2, b);
}

__weak void USART3_HW_OnRxByte(uint8_t b)
{
    UART_ISR_FeedRx(UART3, b);
}

__weak void UART4_HW_OnRxByte(uint8_t b)
{
    UART_ISR_FeedRx(UART4_ID, b);
}

__weak void USART1_HW_OnTxRegEmpty(void)
{
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
}

__weak void USART2_HW_OnTxRegEmpty(void)
{
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
}

__weak void USART3_HW_OnTxRegEmpty(void)
{
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
}

__weak void UART4_HW_OnTxRegEmpty(void)
{
    USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t b = (uint8_t)USART_ReceiveData(USART1);
        USART1_HW_OnRxByte(b);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
        USART1_HW_OnTxRegEmpty();
    }
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t b = (uint8_t)USART_ReceiveData(USART2);
        USART2_HW_OnRxByte(b);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
        USART2_HW_OnTxRegEmpty();
    }
}

void USART3_IRQHandler(void)
{
    /* 处理ORE（溢出错误）：丢弃损坏的字节，恢复接收 */
    if (USART3->SR & (1u << 3)) { /* ORE标志位 */
        (void)USART3->DR; /* 读取DR清除ORE */
    }
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        uint8_t b = (uint8_t)USART_ReceiveData(USART3);
        USART3_HW_OnRxByte(b);
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
    if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET) {
        USART3_HW_OnTxRegEmpty();
    }
}

void UART4_IRQHandler(void)
{
    if ((UART4->SR & (1u << 5)) != 0u) { /* RXNE */
        uint8_t b = (uint8_t)(UART4->DR & 0xFFu);
        UART4_HW_OnRxByte(b);
    }
    if (USART_GetITStatus(UART4, USART_IT_TXE) != RESET) {
        UART4_HW_OnTxRegEmpty();
    }
}

/* =============================================================================
 * 兼容旧API的宏（可选使用）
 * ============================================================================= */

/* 与旧版bsp_usart.h兼容的宏 */
#define USART_SendByte(u, b)    UART[u].SendByte(&UART[u], b)
#define USART_SendString(u, s)  UART[u].SendString(&UART[u], s)
#define USART_ReadByte(u)       UART[u].ReadByte(&UART[u])
#define USART_ClearBuf(u)       UART[u].ClearBuf(&UART[u])

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
