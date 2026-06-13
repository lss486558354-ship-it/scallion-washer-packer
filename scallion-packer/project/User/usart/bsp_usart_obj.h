/**
 ******************************************************************************
 * 串口对象化封装
 *
 * ============================================================================
 * 使用示例：
 *
 * // 1. 初始化
 * UART_InitAll();                        // 初始化所有串口
 *
 * // 2. 使用快捷宏发送
 * UART1_SendString("System ready\r\n");     // 发送字符串
 * UART2_SendByte(0xAA);                  // 发送单字节
 * UART1_SendByte('A');                   // 发送字符
 *
 * // 3. 格式化打印（直接发送，无需sprintf）
 * UART1_Printf("Value:%d\r\n", 123);     // 打印整数
 * UART1_Printf("Float:%.2f\r\n", 3.14); // 打印浮点数
 * UART1_Printf("Hex:0x%02X\r\n", 255);   // 打印十六进制
 * UART2_Printf("Sensor:%.1f\r\n", 25.5);  // 发送到Jetson
 *
 * // 4. 接收数据
 * uint8_t data;
 * if (UART2_Available() > 0) {          // 检查是否有数据
 *     data = UART2_ReadByte();           // 读取字节
 * }
 *
 * // 5. 使用对象方法（与快捷宏效果相同）
 * UART[UART1].SendString(&UART[UART1], "Hello");
 * UART[UART2].SendByte(&UART[UART2], 0x55);
 * data = UART[UART2].ReadByte(&UART[UART2]);
 *
 * // 6. 缓冲区管理
 * UART2_ClearBuf();                      // 清空接收缓冲区
 * uint16_t count = UART2_Available();    // 获取缓冲区数据数量
 *
 * ============================================================================
 * UART_ID_TypeDef 类型说明：
 *   UART1 = 0    - 调试串口，PA9/PA10
 *   UART2 = 1    - Jetson通信，PA2/PA3
 *   UART3 = 2    - 串口屏，PB10/PB11
 *   UART4_ID = 3 - 飞特 SCS，PC10/PC11
 *   UART_MAX = 4 - 串口总数
 *
 * ============================================================================
 * 硬件配置：
 * - UART1: PA9(TX) / PA10(RX) - 调试串口，波特率115200
 * - UART2: PA2(TX) / PA3(RX) - Jetson通信，波特率115200
 * - UART3: PB10(TX) / PB11(RX) - 串口屏，波特率115200
 * - UART4: PC10(TX) / PC11(RX) - 飞特 SCS 总线舵机，波特率115200
 *
 * ============================================================================
 * 作者：工程训练中心116电控组
 * 日期：2026-03-24
 ******************************************************************************
 */

#ifndef __BSP_USART_OBJ_H__
#define __BSP_USART_OBJ_H__

/* 飞特 SCS 物理层二选一：
 * 0=**全双工**（默认）：PC10=TX、PC11=RX；接 **URT-1 / USB-TTL 排针**、或 MCU 侧 TX/RX 经二极管合路到单线。
 * 1=**USART 半双工**：仅用 PC10 单脚直驱舵机 Signal（PC11 高阻）；接 URT-1 时勿开此选项，否则 RX 无效。 */
#ifndef UART4_SCS_HALF_DUPLEX
#define UART4_SCS_HALF_DUPLEX  0
#endif

#include "stm32f10x.h"
#include <stdarg.h>
#include <stdio.h>
#include "bsp_gpio.h"

/* 前向声明 - 统一struct标签与typedef名 */
typedef struct _UART_t UART_t;

/* 串口编号枚举
 *   UART1 = 0    - 调试串口，PA9/PA10
 *   UART2 = 1    - Jetson通信，PA2/PA3
 *   UART3 = 2    - 串口屏，PB10/PB11
 *   UART4_ID = 3  - 飞特 SCS 总线舵机等，PC10/PC11
 *   UART_MAX = 4   - 串口总数
 */
typedef enum {
    UART1 = 0,
    UART2 = 1,
    UART3 = 2,
    UART4_ID = 3,
    UART_MAX = 4
} UART_ID_TypeDef;

/* 串口引脚配置 */
typedef struct {
    GPIO_PortPin_TypeDef tx_pin;  /* TX引脚 */
    GPIO_PortPin_TypeDef rx_pin;  /* RX引脚 */
    uint8_t afio_remap;           /* AFIO重映射（未使用） */
} UART_Pins_TypeDef;

/* 环形缓冲区
 * 用于存储接收到的数据，支持循环覆盖
 */
#define UART_RX_BUF_SIZE  128
typedef struct {
    uint8_t buf[UART_RX_BUF_SIZE];  /* 缓冲区数组，大小128字节 */
    uint16_t head;                    /* 读指针位置 */
    uint16_t tail;                    /* 写指针位置 */
    uint8_t full;                     /* 缓冲区满标志：0-未满，1-已满 */
} UART_RingBuf_TypeDef;

/* 串口对象
 *
 * 使用方式：
 *   // 使用快捷宏（推荐）
 *   UART1_SendString("Hello");
 *   data = UART2_ReadByte();
 *
 *   // 使用对象方法
 *   UART[UART1].SendString(&UART[UART1], "Hello");
 */
struct _UART_t {
    UART_ID_TypeDef id;              /* 串口编号：UART1~UART4 */
    USART_TypeDef* usart;           /* 底层USART外设指针 */
    uint32_t baudrate;              /* 波特率，默认115200 */

    /* 引脚配置 */
    UART_Pins_TypeDef pins;

    /* 接收缓冲区 */
    UART_RingBuf_TypeDef rx_buf;

    /* 发送标志：0-空闲，1-发送中 */
    volatile uint8_t tx_busy;

    /* 函数指针 */
    void (*Init)(UART_t* self);
    void (*SendByte)(UART_t* self, uint8_t byte);
    void (*SendString)(UART_t* self, char* str);
    void (*SendData)(UART_t* self, uint8_t* data, uint16_t len);
    uint8_t (*ReadByte)(UART_t* self);
    uint16_t (*Available)(UART_t* self);
    void (*ClearBuf)(UART_t* self);
    void (*Printf)(UART_t* self, char* fmt, ...);
};

/* 外部声明：全局串口对象数组 */
extern UART_t UART[UART_MAX];

/* 初始化所有串口 */
void UART_InitAll(void);

/* 仅初始化 UART4（PC10/PC11，飞特 SCS）。用于 main 等已自行初始化 USART1 的工程，避免 UART_InitAll 重配调试口。 */
void UART4_InitOnly(void);

/* 在 USARTx_IRQHandler 里把收到的字节写入对应环形缓冲（也可在应用层 Rx 回调里调用以兼顾协议与缓冲） */
void UART_ISR_FeedRx(UART_ID_TypeDef id, uint8_t byte);

/*
 * 串口硬件中断回调（建议在 main.c 中提供强符号覆盖；本模块内为 __weak 默认：仅 FeedRx / 关闭 TXE）。
 * Rx：每收到 1 字节调用一次；Tx：发送寄存器空（USART_IT_TXE）时调用，用于非阻塞发送。
 */
void USART1_HW_OnRxByte(uint8_t b);
void USART2_HW_OnRxByte(uint8_t b);
void USART3_HW_OnRxByte(uint8_t b);
void UART4_HW_OnRxByte(uint8_t b);
void USART1_HW_OnTxRegEmpty(void);
void USART2_HW_OnTxRegEmpty(void);
void USART3_HW_OnTxRegEmpty(void);
void UART4_HW_OnTxRegEmpty(void);

/* FreeRTOS 下初始化 UART1 发送互斥（在创建任务前调用，避免多任务与协议 ACK 字节交错） */
void UART1_TxMutex_Init(void);

/* 阻塞轮询发 UART1 前调用：关 TXE 中断并清空 main.c 的 ISR 发送队列，避免与 usart1_tx_start_isr 争用 DR 死等 */
void UART1_PollingTxPreempt(void);

/* =============================================================================
 * 快捷宏定义
 * ============================================================================= */

/* UART1 快捷宏（调试串口） */
#define UART1_SendByte(b)     UART[UART1].SendByte(&UART[UART1], b)
#define UART1_SendString(s)   UART[UART1].SendString(&UART[UART1], s)
#define UART1_ReadByte()      UART[UART1].ReadByte(&UART[UART1])
#define UART1_Available()     UART[UART1].Available(&UART[UART1])
#define UART1_ClearBuf()     UART[UART1].ClearBuf(&UART[UART1])
#define UART1_Printf(f, ...)  UART[UART1].Printf(&UART[UART1], f, ##__VA_ARGS__)

/* UART2 快捷宏（Jetson通信） */
#define UART2_SendByte(b)     UART[UART2].SendByte(&UART[UART2], b)
#define UART2_SendString(s)   UART[UART2].SendString(&UART[UART2], s)
#define UART2_ReadByte()      UART[UART2].ReadByte(&UART[UART2])
#define UART2_Available()     UART[UART2].Available(&UART[UART2])
#define UART2_ClearBuf()     UART[UART2].ClearBuf(&UART[UART2])
#define UART2_Printf(f, ...)  UART[UART2].Printf(&UART[UART2], f, ##__VA_ARGS__)

/* UART3 快捷宏（串口屏） */
#define UART3_SendByte(b)     UART[UART3].SendByte(&UART[UART3], b)
#define UART3_SendString(s)   UART[UART3].SendString(&UART[UART3], s)
#define UART3_ReadByte()      UART[UART3].ReadByte(&UART[UART3])
#define UART3_Available()     UART[UART3].Available(&UART[UART3])
#define UART3_ClearBuf()     UART[UART3].ClearBuf(&UART[UART3])
#define UART3_Printf(f, ...)  UART[UART3].Printf(&UART[UART3], f, ##__VA_ARGS__)

/* UART4 快捷宏（SCS 总线舵机等） */
#define UART4_SendByte(b)     UART[UART4_ID].SendByte(&UART[UART4_ID], b)
#define UART4_SendString(s)   UART[UART4_ID].SendString(&UART[UART4_ID], s)
#define UART4_ReadByte()      UART[UART4_ID].ReadByte(&UART[UART4_ID])
#define UART4_Available()     UART[UART4_ID].Available(&UART[UART4_ID])
#define UART4_ClearBuf()     UART[UART4_ID].ClearBuf(&UART[UART4_ID])
#define UART4_Printf(f, ...)  UART[UART4_ID].Printf(&UART[UART4_ID], f, ##__VA_ARGS__)

#endif /* __BSP_USART_OBJ_H__ */
