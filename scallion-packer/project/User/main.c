/**
 ******************************************************************************
 * 新工艺流程：串口屏命令驱动，自动循环计数打包
 *
 * 文件布局：前置声明 → main() → BSP_Tick_AppHook1ms() → 各 USART 串口回调 →
 *           其余 static 辅助函数（GPIO、串口阻塞发送、USART1 非阻塞发送队列）。
 *
 * 串口：Rx/Tx 中断在 Libraries工程链路的 USARTx_IRQHandler（bsp_usart_obj.c）中
 *       触发，再调用本文件中的 USARTx_HW_OnRxByte / OnTxRegEmpty（每口单独实现）。
 *
 * 【工艺流程总览】：
 *   上电 → IDLE（等待串口屏 0x9A 命令）
 *         ↓ 收到 0x9A
 *   BELT_RUN     同步带运转，等待超声波检测
 *         ↓ 超声<5cm（us_cnt++）
 *   SCREW_REV    同步带停 + 丝杆反转（E1限位或6s）
 *         ↓
 *   CUT_RUN      切割电机+同步带正转（切割1.5s停，同步带跑满8s）
 *         ↓
 *   BELT_FWD     同步带正转驱动（大葱移动）
 *         ↓
 *   SCREW_FWD    丝杆正转（E0限位或6s）
 *         ↓
 *   BELT_RUN2    同步带再次运转 → 回到 BELT_RUN 继续计数
 *         ↓ 超声计数达到 3
 *   CONVEY_WAIT  等待3s（同步带已跑满8s，传送带不动）
 *         ↓
 *   CONVEY_ENTER → CONVEY_REV（反转4s）→ CONVEY_FWD（正转20s）→ IDLE
 *
 * 版本：V5.0
 * 日期：2026-04-20
 ******************************************************************************
 */

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include <stdarg.h>
#include <stdio.h>
#include "bsp_tick.h"
#include "bsp_stepper.h"
// #include "bsp_hx711.h"
#include "process_control.h"
#include "main.h"
#include "bsp_usart_obj.h"
#include "bsp_servo_serial.h"
#include "device.h"
#include "bsp_servo_obj.h"
#include "bsp_dc_motor_obj.h"
#include "protocol.h"
#include "bsp_relay.h"
#include "bsp_limit_switch.h"
/*-----------------------------------------------------------
 * 静态变量定义（main.c 私有）
 *-----------------------------------------------------------*/
volatile uint8_t g_process_run_request = 0;

/* 旧版流程状态（兼容用，实际上用新的 g_new_proc_state）*/
static volatile Process_State_TypeDef g_proc_state = PROC_IDLE;
static volatile uint16_t              g_proc_seg_ms;
static volatile uint8_t               g_proc_cycle;

/*=======================================================================
 * 【新工艺流程状态机变量】
 *
 * 完整流程：
 *   IDLE → BELT_RUN → SCREW_REV → CUT_RUN → BELT_FWD → SCREW_FWD → BELT_RUN2
 *                                                         ↓ (cnt<5)
 *                                                   BELT_RUN
 *                                                         ↓ (cnt=5)
 *                                          CUT_RUN(切割1.5s+同步带8s) → CONVEY_WAIT(等6.5s)
 *                                                        ↓
 *                                              CONVEY_ENTER → CONVEY_REV → CONVEY_FWD → IDLE
 * =======================================================================*/
static volatile Process_New_State_TypeDef g_new_proc_state = PROC_NEW_IDLE;
static volatile uint32_t                  g_new_proc_seg_ms;  /* 阶段内毫秒计数 */
static volatile uint8_t                   g_ir_reset_request; /* 0x9A触发时请求重置IR计数 */
static volatile uint8_t                   g_us_count;         /* 超声波检测计数（<5cm 触发），阈值3根 */
static volatile uint8_t                   g_us_count_flag;    /* 0=本次未计数 1=已计数（防重复计数）*/

/* 分区 D：电机3+4 上电自检 —— 传送带正/反各一段；打包电机两段均为正转 */
/* M3(传送带)保持400细分=250Hz；M4(打包)改为32细分=20Hz（250/12.5） */
static uint16_t s_m34_boot_cnt;
static uint8_t  s_m34_boot_flag; /* 0=正转段1=反转段 2=已结束，不再计数 */

/* 分区 D1：丝杆电机上电自检 —— 反转11.5s → 正转11.5s → 完成 */
/* 必须在 M3/M4 自检完成后才执行；丝杆自检完成后才允许启动主流程 */
typedef enum {
    SCRW_IDLE  = 0,
    SCRW_REV,        /* 反转（CCW） */
    SCRW_FWD,        /* 正转（CW） */
    SCRW_DONE        /* 完成 */
} ScrewBoot_State_TypeDef;
static volatile ScrewBoot_State_TypeDef s_screw_boot_st = SCRW_IDLE;
static volatile uint16_t                s_screw_boot_cnt;  /* 10ms 单位计数器 */

/* main() 未跑完 Stepper_Init 等之前，SysTick 已可能1ms 进钩子；须禁止提前动步进/流程 */
static volatile uint8_t s_app_tick_armed;

/* SysTick 每 10ms 递增；主循环取走并跑 Process_Main_SM_Run（称重读 HX711 耗时长，不可在 ISR 里跑满） */
static volatile uint8_t s_process_sm_pending_count;

/* ========================================================================
 * 【分区 H】打包测试分区状态机
 *
 * 测试时序（简化版，不依赖超声/红外传感器）：
 *   IDLE → IMPELLER_ON → ROD_HALF1 → ROD_HALF2 → STOP → IDLE
 *     - IMPELLER_ON：舵机3+8转动 + 电机4(M3传送带)转动，4s
 *     - ROD_HALF1：舵机2+7半圈，417ms
 *     - ROD_HALF2：舵机2+7半圈，417ms
 *     - STOP：电机4停止 + 舵机3+8停止
 * ======================================================================== */
static volatile PackTestState_TypeDef g_pack_test_state = PACK_TEST_IDLE;
static volatile uint32_t             g_pack_test_seg_ms;  /* 阶段内毫秒计数 */
static volatile uint32_t             g_pack_test_total_ms; /* 打包阶段绝对累计时间 */

        /* 膜杆方向计算辅助（实现在 main() 前）*/

#if MAIN_PACK_ULTRA_TEST_ENABLE
/* 分区 G：满 MAIN_PACK_ULTRA_TEST_PERIOD_MS 置 1，主循环测距（勿在 SysTick 内调 Process_ReadPackDistance） */
static volatile uint8_t s_pack_ultra_test_pending;
#endif

/* USART1 非阻塞发送队列（仅本文件 + USART1_HW_OnTxRegEmpty 使用） */
/* 改进：使用循环缓冲区，支持多消息排队，不再丢消息 */
#define UART1_TX_QUEUE_SIZE  8   /* 最多缓存8条消息 */
#define UART1_TX_BUF_SIZE   128  /* 每条消息最大长度 */

typedef struct {
    char  data[UART1_TX_BUF_SIZE];
    uint16_t len;
    uint16_t idx;  /* 当前发送位置 */
} Uart1TxMsg;

static Uart1TxMsg s_tx_queue[UART1_TX_QUEUE_SIZE];
static volatile uint8_t s_tx_head;  /* 下一条要发送的消息索引 */
static volatile uint8_t s_tx_tail;  /* 下一条新消息写入位置 */
static volatile uint8_t s_tx_count; /* 队列中消息数量 */

/* 非阻塞 Printf（格式化后入队，不阻塞） */
static void UART1_PutStr(const char *str);
static void UART1_PrintfNB(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void UART1_PrintfNB(const char *fmt, ...) {
    char buf[UART1_TX_BUF_SIZE];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    UART1_PutStr(buf);
}

/* -------------------------------------------------------------------------- */
/* 前置声明 */
/* -------------------------------------------------------------------------- */
static void GPIO_Init_All(void);
static void uart1_reapply_af_pins(void);
void usart1_tx_start_isr(const char *str);
static void M34_ScsBus_StopWheelImpellers(void);
static void PackTest_ImpellerKeepalive(void);

/* 新工艺流程辅助函数 */
static void delay_us(uint32_t us);
static uint16_t New_ReadUltrasonicDistance(void);

/* ========================================================================
 * 【分区 H】打包测试辅助函数（须放在 main() 前，因主循环中会调用）
 * ======================================================================== */

/* 膜杆方向计算：from→to 的最短弧方向 */
static int8_t PackTest_RodDirBetween(uint16_t from_pos, uint16_t to_pos)
{
    uint32_t d = ((uint32_t)to_pos + PACK_TEST_ROD_STEPS_PER_REV
                  - (uint32_t)from_pos) % PACK_TEST_ROD_STEPS_PER_REV;
    if (d == 0u) return 0;
    return (d <= PACK_TEST_ROD_SHORTARC_HALF) ? 1 : -1;
}

/* 膜杆2+7 同时运行（ST3215/STS360 轮模式）
 * 注意：WriteSpeedAcc 内部已包含扭矩开启，无需单独调用 SetTorqueSwitch */
static void PackTest_RodWheelRunBoth(int16_t sp2, int16_t sp7)
{
    /* ST3215：先写速度后开扭矩（WriteSpeedAcc 内部实现），避免上电时飞转 */
    ServoBus_STS360_WriteSpeedAcc(PACK_TEST_ROD_ID2, sp2, PACK_TEST_ROD_ACC);
    ServoBus_STS360_WriteSpeedAcc(PACK_TEST_ROD_ID7, sp7, PACK_TEST_ROD_ACC);
}

/* 膜杆2+7 同时停止（ST3215/STS360）
 * 执行顺序（确保 UART 命令顺序正确）：
 * 1. 先开扭矩（确保舵机处于受力状态）
 * 2. 延迟约 2ms（等待扭矩开启命令通过 UART 发出）
 * 3. 发零速命令
 * 4. 延迟约 5ms（等待零速命令通过 UART 发出）
 * 5. 关扭矩
 *
 * 注意：WriteSpeedAcc 内部会先写速度再开扭矩，所以 Run 时不用单独开扭矩；
 * 但 Stop 时必须手动控制顺序，否则零速命令发不出去舵机会继续飞转。 */
static void PackTest_RodWheelStopBoth(void)
{
    ServoBus_SetTorqueSwitch(PACK_TEST_ROD_ID2, 1u);
    ServoBus_SetTorqueSwitch(PACK_TEST_ROD_ID7, 1u);

    for (volatile uint32_t i = 0u; i < 2000u; i++) { __NOP(); }

    ServoBus_STS360_WriteSpeedAcc(PACK_TEST_ROD_ID2, 0, 0u);
    ServoBus_STS360_WriteSpeedAcc(PACK_TEST_ROD_ID7, 0, 0u);

    for (volatile uint32_t i = 0u; i < 5000u; i++) { __NOP(); }

    ServoBus_SetTorqueSwitch(PACK_TEST_ROD_ID2, 0u);
    ServoBus_SetTorqueSwitch(PACK_TEST_ROD_ID7, 0u);
}

/* 阶段切换时统一停止所有执行机构 */
static void PackTest_StopAll(void)
{
    M4_Stop();
    DCM4_Stop();
    M34_ScsBus_StopWheelImpellers();
    PackTest_RodWheelStopBoth();
    /* 停止切膜电机：PC9=ENA低，PD8=PD9=0 */
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
    GPIO_ResetBits(GPIOD, GPIO_Pin_8);
    GPIO_ResetBits(GPIOD, GPIO_Pin_9);
}

/* -------------------------------------------------------------------------- */
/* main */
/* -------------------------------------------------------------------------- */
int main(void)
{
    /* 上电后最先强制拉低 DCM1 方向引脚，防止 L298N ENA 常使能时电机乱转 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    {
        GPIO_InitTypeDef g;
        g.GPIO_Mode  = GPIO_Mode_Out_PP;
        g.GPIO_Speed = GPIO_Speed_10MHz;
        g.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
        GPIO_Init(GPIOD, &g);
        GPIO_ResetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_1);
    }

    BSP_Tick_Init();
    LimitSwitch_Init();
    UART_InitAll();
    ServoBus_InitAll();

    Stepper_Init();
    Stepper_TIM1_ClearPulOutputs();
    /* MPU6050 未接入本流程时可保持注释；需要 I2C 传感器时再初始化 bsp_mpu6050 */
    /* MPU6050_Init(); */
    Stepper_ReapplyTim1RemapAndPulGpio();

    /* STS3020（SCS，叶轮 ID3/8）：大端速度编码 */
    ServoBus_STS_EnterWheelMode(M34_SCS_WHEEL_ID1);   /* ID3：叶轮 */
    ServoBus_STS_EnterWheelMode(M34_SCS_WHEEL_ID2);  /* ID8：叶轮 */
    /* STS360（膜杆 ID2/7）：小端 int16 速度 */
    ServoBus_STS360_EnterWheelMode(PACK_TEST_ROD_ID2);  /* ID2：膜杆，舵机2 */
    ServoBus_STS360_EnterWheelMode(PACK_TEST_ROD_ID7);   /* ID7：膜杆，舵机7 */

    /*
     * Process_StopAll() 会调 Device.Servo1/2.SetAngle：须先 Device_InitInstance填函数表，
     * 否则 SetAngle 为 NULL → HardFault，表现为卡在 Process_Init。
     * SERVO_InitAll() 初始化 TIM4 PWM 舵机，否则 SVx_SetAngle 访问未配置外设。
     */
    Device_InitInstance();
    SERVO_InitAll();
    /*
     * Process_StopAll() 会 DCM1_Stop~4：须先 DCM_InitAll() 绑定 DCM[i].Stop，
     * 否则 Stop 为 NULL → HardFault，现象同「卡在 Process_Init」。
     */
    DCM_InitAll();
    /*
     * GPIO_Init_All() 必须在 DCM_InitAll() 之后，因为 DCM_InitAll() 将 PC6/7/8/9
     * 配置为 TIM8 PWM 复用功能，这里重新抢占 PC6 为普通 GPIO 推挽输出，
     * 供切割电机（高电平转/低电平停）使用。
     */
    GPIO_Init_All();
    /*
     * 继电器（K_AIR/PB12、K_PUMP/PB13、K_S/PA15）初始化。
     * Relay_InitAll() 会关闭 JTAG 复用（仅保留 SWD），使 PA15 可作 GPIO。
     */
    Relay_InitAll();

    /* 切膜电机方向测试：ENA硬接5V，PD8/PD9控制方向
     * PD8高+PD9低=正转，PD8低+PD9高=反转，PD8=PD9=停止
     * 正转0.5s → 反转0.4s → 停止
     * 注释掉此段代码前确保方向正确 */
    {
        extern volatile uint32_t g_tick_ms;
        uint32_t t = g_tick_ms;
        UART1_PrintfNB("[CUT] Test start, tick=%u\r\n", (unsigned)t);

        /* 正转：PD8=1, PD9=0 */
        GPIO_SetBits(GPIOD, GPIO_Pin_8);
        GPIO_ResetBits(GPIOD, GPIO_Pin_9);
        while ((g_tick_ms - t) < 500u) {}
        UART1_PrintfNB("[CUT] Forward done, tick=%u\r\n", (unsigned)g_tick_ms);

        /* 反转：PD8=0, PD9=1 */
        GPIO_ResetBits(GPIOD, GPIO_Pin_8);
        GPIO_SetBits(GPIOD, GPIO_Pin_9);
        t = g_tick_ms;
        while ((g_tick_ms - t) < 500u) {}
        UART1_PrintfNB("[CUT] Reverse done, tick=%u\r\n", (unsigned)g_tick_ms);

        /* 停止：PD8=PD9=0 */
        GPIO_ResetBits(GPIOD, GPIO_Pin_8);
        GPIO_ResetBits(GPIOD, GPIO_Pin_9);
        UART1_PrintfNB("[CUT] Test stop, tick=%u\r\n", (unsigned)g_tick_ms);
    }

    /* 工艺流程 GPIO/参数；主状态机在 SysTick 1ms 钩子里每 10ms 跑 Process_Main_SM_Run */
    Process_Init();

    /* 测试阶段：跳过 M3/M4 上电自检（丝杆自检在 BSP_Tick_AppHook1ms 分区 D1 运行） */
    s_m34_boot_flag  = 2u;   /* 2=自检已完成，分区 D 不会跑 */

    s_app_tick_armed = 1u;
    /* 确保 PA9/PA10 仍为 USART1（避免长初始化后引脚模式漂移）；再发上电提示 */
    uart1_reapply_af_pins();
    /*仅用 ASCII，避免串口助手按 GBK/ANSI 解码 UTF-8 中文成乱码 */
    usart1_tx_start_isr("\r\n[UART1] Idle: PC1=start once, or g_process_run_request=1\r\n");
    usart1_tx_start_isr("[SYS] HX711 disabled\r\n");
    for (;;) {
        /* 每2秒打印精简心跳 */
        {
            static uint32_t s_debug_heartbeat_last_ms;
            uint32_t now_ms = BSP_GetTickMs();
            if ((now_ms - s_debug_heartbeat_last_ms) >= 2000U) {
                s_debug_heartbeat_last_ms = now_ms;
                /* 精简打印：当前状态 */
                UART1_PrintfNB("[LOOP] st=%d\r\n", (int)g_new_proc_state);
            }
        }

        /* 处理串口屏请求（由USART3 ISR设置标志，主循环执行，避免中断内阻塞） */
        if (g_protocol_emergency_stop_request) {
            usart1_tx_start_isr("*[ESTOP]\r\n");
            g_protocol_emergency_stop_request = 0;
            /* 停止所有电机和设备 */
            Stepper_SyncStop();
            Stepper_ScrewStop();
            Stepper_Stop(STEPPER_CONVEY);
            Stepper_Stop(STEPPER_PACK);
            DCM1_Stop();
            DCM2_Stop();
            DCM3_Stop();
            DCM4_Stop();
            FAN_E6_Off();
            /* 复位到空闲，等待下一条 0x9A */
            g_new_proc_state = PROC_NEW_IDLE;
            g_new_proc_seg_ms = 0u;
            g_us_count = 0u;
            g_us_count_flag = 0u;
        }
        if (g_protocol_process_stop_request) {
            usart1_tx_start_isr("*[PROC_STOP]\r\n");
            g_protocol_process_stop_request = 0;
            Stepper_SyncStop();
            Stepper_ScrewStop();
            Stepper_Stop(STEPPER_CONVEY);
            DCM1_Stop();
            DCM2_Stop();
            FAN_E6_Off();
            g_new_proc_state = PROC_NEW_IDLE;
            g_new_proc_seg_ms = 0u;
            g_us_count = 0u;
            g_us_count_flag = 0u;
        }
        if (g_protocol_process_pause_request) {
            usart1_tx_start_isr("*[PROC_PAUSE]\r\n");
            g_protocol_process_pause_request = 0;
        }
        if (g_protocol_process_start_request) {
            usart1_tx_start_isr("*[PROC_START]\r\n");
            g_protocol_process_start_request = 0;
            /* 从 IDLE 切换到 BELT_RUN，启动新工艺流程 */
            g_new_proc_state = PROC_NEW_BELT_RUN;
            g_new_proc_seg_ms = 0u;
            g_us_count = 0u;
            g_us_count_flag = 0u;
            g_pack_test_total_ms = 0u;
            g_ir_reset_request = 1u;
        }
        if (g_protocol_pack_test_request) {
            usart1_tx_start_isr("*[PACK_TEST_START]\r\n");
            g_protocol_pack_test_request = 0;
            /* 仅在 IDLE 时允许触发，避免与主流程冲突 */
            if (g_new_proc_state == PROC_NEW_IDLE) {
                PackTest_StopAll();                          /* 先停止所有残留动作 */
                g_new_proc_state = PROC_NEW_PACK_IMPELLER;
                g_new_proc_seg_ms = 0u;
            }
        }
        if (g_protocol_process_resume_request) {
            usart1_tx_start_isr("*[PROC_RESUME]\r\n");
            g_protocol_process_resume_request = 0;
        }

        /* HX711 已注释掉，串口屏通信不再被阻塞 */
        // HX711_TestTask();
        while (s_process_sm_pending_count != 0u) {
            s_process_sm_pending_count--;
            /* 新工艺流程状态机在滴答钩子（BSP_Tick_AppHook1ms）中已完整实现 */
        }
    }
}

/* 叶轮总线舵机（M34_SCS_WHEEL_ID1/2）保持运行（舵机3/8在打包全程不停） */
static void PackTest_ImpellerKeepalive(void)
{
    /* 只写一次速度即可，参考 PC 上位机直接写扭矩+速度的方式 */
    ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID1, 1u);
    ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID2, 1u);
    ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID1, M34_SCS_WHEEL_SPD, M34_SCS_WHEEL_ACC);
    ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID2, M34_SCS_WHEEL_SPD_ID8, M34_SCS_WHEEL_ACC);
}

/* 叶轮总线舵机（M34_SCS_WHEEL_ID1/2）同时停轮：先卸扭矩，再写零速
 * 关键：先关扭矩，再写零速。
 * WriteSpeedAcc 内部会先写速度再开扭矩；若先写零速后开扭矩，在两次调用之间
 * 另一个舵机扭矩已开着，仍以旧速度飞转。故先统一卸扭矩，再清零速。 */
static void M34_ScsBus_StopWheelImpellers(void)
{
    ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID1, 0u);
    ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID2, 0u);
    ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID1, 0);
    ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID2, 0);
}

/* ========================================================================
 * 【分区 H 辅助函数】膜杆轮模式方向判断
 *
 * 膜杆位置范围 0~4095（STS 1圈=4096步）。
 * 计算 from→to 的最短弧方向：
 *   返回 1  = 短弧为正向（CW）
 *   返回 -1 = 短弧为反向（CCW）
 *   返回 0  = 已到位（停在目标位置）
 * ======================================================================== */
/* process_control.c 无法访问 main.c 中的 static g_proc_state，通过此函数清除 */
void Process_SetProcState_Clear(void)
{
    g_proc_state = PROC_IDLE;
}

/* -------------------------------------------------------------------------- */
/* SysTick 每 1ms 调用一次（bsp_tick.c → BSP_Tick_OnSysTick → 本函数） */
/* 下面按【分区 A～H】阅读，每区职责不同；注释尽量写细，方便对照代码理解。 */
/* 【新工艺流程状态机】完全替换旧版 PROC_FLOW1~FLOW3 */
/* -------------------------------------------------------------------------- */
void BSP_Tick_AppHook1ms(void)
{
    /* 系统Tick未使能时直接返回（防止初始化阶段误触发）*/
    if (!s_app_tick_armed) {
        return;
    }

    /* ========================================================================
     * 【分区 A】运行指示灯（PC13）
     *
     * 每 1ms 计数一次，满 500（约 0.5s）翻转一次 LED，表示 SysTick 在跑、程序活着。
     * ======================================================================== */
    {
        static uint16_t s_led_ms;

        if (++s_led_ms >= 500u) {
            s_led_ms = 0u;
            LED_TOGGLE();
        }
    }

    /* ========================================================================
     * 【分区 B】M3/M4 上电自检（传送带 + 打包步进）
     *
     * 仅在空闲（g_new_proc_state == PROC_NEW_IDLE）时运行自检，避免与主流程抢电机。
     * 每相位开始(cnt==0)以各自频率启动传送带+打包电机同向，之后仅计数；
     * M3传送带保持400细分按 M34_RUN_HZ_CONVEY；M4打包电机按 M34_RUN_HZ_PACK。
     * cnt 计满 M34_BOOT_FWD_MS / M34_BOOT_REV_MS 后停两路、cnt 清零、flag 加 1。
     *
     * PC5（M34_RST_BTN）二次消抖：按下沿且空闲时重置自检。
     * ======================================================================== */
//    if (g_new_proc_state == PROC_NEW_IDLE) {
//#define M34_BTN_STG_MS 20u
//        static uint8_t  s_rst_stg1;
//        static uint8_t  s_rst_same;
//        static uint8_t  s_rst_armed;
//        static uint8_t  s_rst_ever_pressed;
//        static uint8_t  s_m34_scs_wheel_armed;
//        static uint16_t s_m34_scs_refresh_ms;
//        uint8_t         rst_now;
//        uint8_t        rst_stable;
//        uint8_t        m34_active;

//        rst_now = M34_RST_BTN_DOWN() ? 1u : 0u;
//        if (rst_now == s_rst_stg1) {
//            if (s_rst_same < 255u) s_rst_same++;
//        } else {
//            s_rst_stg1 = rst_now;
//            s_rst_same = 0u;
//        }

//        rst_stable = 0u;
//        if (s_rst_same >= M34_BTN_STG_MS) {
//            rst_stable = s_rst_stg1;
//            if (rst_stable) {
//                s_rst_ever_pressed = 1u;
//                if (s_rst_armed) {
//                    Stepper_Stop(STEPPER_CONVEY);
//                    Stepper_Stop(STEPPER_PACK);
//                    s_m34_boot_cnt  = 0u;
//                    s_m34_boot_flag = 0u;
//                    s_rst_armed     = 0u;
//                }
//            } else {
//                s_rst_armed = 1u;
//            }
//        }

//        m34_active = (s_rst_ever_pressed && s_m34_boot_flag < 2u) ? 1u : 0u;

//        if (m34_active) {
//            if (!s_m34_scs_wheel_armed) {
//                s_m34_scs_wheel_armed = 1u;
//                s_m34_scs_refresh_ms  = 0u;
//                ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID1, 1u);
//                ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID2, 1u);
//                ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID1, M34_SCS_WHEEL_SPD, M34_SCS_WHEEL_ACC);
//                ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID2, M34_SCS_WHEEL_SPD_ID8, M34_SCS_WHEEL_ACC);
//                ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID1, M34_SCS_WHEEL_SPD);
//                ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID2, M34_SCS_WHEEL_SPD_ID8);
//            } else {
//                s_m34_scs_refresh_ms++;
//                if (s_m34_scs_refresh_ms >= M34_SCS_REFRESH_MS) {
//                    s_m34_scs_refresh_ms = 0u;
//                    ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID1, 1u);
//                    ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID2, 1u);
//                    ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID1, M34_SCS_WHEEL_SPD, M34_SCS_WHEEL_ACC);
//                    ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID2, M34_SCS_WHEEL_SPD_ID8, M34_SCS_WHEEL_ACC);
//                    ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID1, M34_SCS_WHEEL_SPD);
//                    ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID2, M34_SCS_WHEEL_SPD_ID8);
//                }
//            }
//            {
//                Stepper_Dir_TypeDef d;
//                uint16_t            seg_ms;

//                d     = (s_m34_boot_flag == 0u) ? STEPPER_DIR_CW : STEPPER_DIR_CCW;
//                seg_ms = (s_m34_boot_flag == 0u) ? M34_BOOT_FWD_MS : M34_BOOT_REV_MS;
//                if (s_m34_boot_cnt == 0u) {
//                    Stepper_RunAtSpeed(STEPPER_CONVEY, M34_RUN_HZ_CONVEY, d);
//                    Stepper_RunAtSpeed(STEPPER_PACK, M34_RUN_HZ_PACK, STEPPER_DIR_CW);
//                }
//                s_m34_boot_cnt++;
//                if (s_m34_boot_cnt >= seg_ms) {
//                    Stepper_Stop(STEPPER_CONVEY);
//                    Stepper_Stop(STEPPER_PACK);
//                    s_m34_boot_cnt = 0u;
//                    s_m34_boot_flag++;
//                }
//            }
//        } else {
//            if (s_m34_scs_wheel_armed) {
//                s_m34_scs_wheel_armed = 0u;
//                s_m34_scs_refresh_ms  = 0u;
//                M34_ScsBus_StopWheelImpellers();
//            }
//        }
//    }

    /* ========================================================================
     * 【分区 D1】丝杆电机上电自检 —— 反转11.5s → 正转11.5s → 完成
     *
     * 仅在 M3/M4 自检（s_m34_boot_flag==2）完成后运行。
     * 完成后 s_screw_boot_st == SCRW_DONE。
     * ======================================================================== */
    if (s_screw_boot_st != SCRW_FWD && s_m34_boot_flag == 2u) {
        switch (s_screw_boot_st) {
            case SCRW_IDLE:
                s_screw_boot_cnt = 0u;
                s_screw_boot_st = SCRW_REV;
                Stepper_ScrewRun(STEPPER_SCREW_A, STEPPER_SCREW_B, DFLT_SCREW_SPEED_HZ, STEPPER_DIR_CCW);
                usart1_tx_start_isr("[SCREW_BOOT] start CCW (11.5s)\r\n");
                break;

            case SCRW_REV:
                s_screw_boot_cnt++;
                if (s_screw_boot_cnt >= DFLT_SCREW_BOOT_DURATION_MS) {
                    Stepper_ScrewStop();
                    s_screw_boot_cnt = 0u;
                    s_screw_boot_st = SCRW_FWD;
                    Stepper_ScrewRun(STEPPER_SCREW_A, STEPPER_SCREW_B, DFLT_SCREW_SPEED_HZ, STEPPER_DIR_CW);
                    usart1_tx_start_isr("[SCREW_BOOT] CCW done, start CW (11.5s)\r\n");
                }
                break;

            default:
                break;
        }
    }
    if (s_screw_boot_st == SCRW_FWD) {
        s_screw_boot_cnt++;
        if (s_screw_boot_cnt >= DFLT_SCREW_BOOT_DURATION_MS) {
            Stepper_ScrewStop();
            s_screw_boot_st = SCRW_DONE;
            usart1_tx_start_isr("[SCREW_BOOT] CW done, boot complete\r\n");
        }
    }

    /* ========================================================================
     * 【分区 F】切割+打包联合状态机
     *
     * 触发：USART3 收到 0x9A 命令（或调试串口命令）
     *
     *   IDLE ──(0x9A)──→ BELT_RUN（同步带一直转，无超声触发）
     *     ↑                      │
     *     │                 IR计数≥3
     *     │                      ↓
     *     └──────PACK_STOP←────PACK_CUTFILM
     *                ↑              │
     *                │        gray1: 等
     *                │        gray2: M4停→切膜电机 CW→CCW
     *                └────PACK_ROD_ON_2
     *                          │PACK_TEST_ROD_LEG_MS
     *                    PACK_ROD_OFF
     *                    PACK_M4 ────→ PACK_IMPELLER
     *
     *=======================================================================*/
    {
        Process_New_State_TypeDef st = g_new_proc_state;
        uint32_t                 seg = g_new_proc_seg_ms;
        uint32_t                 now_tick = BSP_GetTickMs();

        /* 打包阶段绝对累计时间（仅打包阶段使用） */
        static uint32_t s_pack_start_tick;
        static uint32_t s_pack_m4_start_tick;
        static uint8_t  s_pack_m4_started;

        /* IR 红外计数：遮挡触发 + 1s 防抖，仅在 PROC_NEW_BELT_RUN 状态下计数 */
        static uint8_t  s_ir_cnt = 0u;
        static uint8_t  s_ir_triggered = 0u;
        static uint32_t s_ir_trigger_tick = 0u;
        static uint8_t  s_ir_counting_enabled = 0u;

        /* IR 低电平（遮挡）触发计数 */
        if (g_ir_reset_request) {
            s_ir_cnt = 0u;
            s_ir_triggered = 0u;
            s_ir_counting_enabled = 0u;
            g_ir_reset_request = 0u;
        }

        /* 状态机上下文：仅在同步带运输阶段（非打包阶段）计数，
         * 进入打包阶段立即禁止计数并清零，防止打包过程中误触发。 */
        if (st >= PROC_NEW_PACK_IMPELLER) {
            /* 进入打包阶段：清零计数、禁止计数，防止打包过程中误触发 */
            s_ir_cnt = 0u;
            s_ir_triggered = 0u;
            s_ir_counting_enabled = 0u;
        } else {
            /* IDLE / BELT_RUN / CUT_RUN 等同步带运输阶段：允许计数 */
            s_ir_counting_enabled = 1u;
        }

        if (s_ir_counting_enabled && IR_COUNTER_LOW() && !s_ir_triggered) {
            s_ir_cnt++;
            s_ir_triggered = 1u;
            s_ir_trigger_tick = now_tick;
            UART1_PrintfNB("[IR_CNT] +1 cnt=%u\r\n", (unsigned)s_ir_cnt);
        }
        /* IR 高电平后等待 1s 才允许下次计数 */
        if (s_ir_triggered && !IR_COUNTER_LOW()) {
            if ((uint32_t)(now_tick - s_ir_trigger_tick) >= 1000u) {
                s_ir_triggered = 0u;
            }
        }

        /* 每500ms打印计数和切割状态 */
        {
            static uint32_t s_print_tick;
            if ((uint32_t)(now_tick - s_print_tick) >= 500U) {
                s_print_tick = now_tick;
                const char* state_name;
                switch (st) {
                    case PROC_NEW_IDLE: state_name = "IDLE"; break;
                    case PROC_NEW_BELT_RUN: state_name = "BELT_RUN"; break;
                    case PROC_NEW_CUT_RUN: state_name = "CUT_RUN"; break;
                    case PROC_NEW_PACK_IMPELLER: state_name = "PACK_IMP"; break;
                    case PROC_NEW_PACK_M4: state_name = "PACK_M4"; break;
                    case PROC_NEW_PACK_ROD_ON_1: state_name = "PACK_ROD1"; break;
                    case PROC_NEW_PACK_ROD_OFF: state_name = "PACK_ROFF"; break;
                    case PROC_NEW_PACK_ROD_ON_2: state_name = "PACK_ROD2"; break;
                    case PROC_NEW_PACK_CUTFILM: state_name = "PACK_FILM"; break;
                    case PROC_NEW_PACK_STOP: state_name = "PACK_STOP"; break;
                    default: state_name = "UNKNOWN"; break;
                }
                UART1_PrintfNB("[MON] st=%s IR=%u seg=%u\r\n",
                    state_name, (unsigned)s_ir_cnt, (unsigned)seg);
            }
        }

        /* 计算打包阶段绝对累计时间（先算，保证switch内各状态都能读到最新值） */
        if (st >= PROC_NEW_PACK_IMPELLER && st <= PROC_NEW_PACK_STOP) {
            g_pack_test_total_ms = now_tick - s_pack_start_tick;
        }

        switch (st) {

        /* ------------------------------------------------------------------
         * PROC_NEW_IDLE：等待 0x9A 启动命令
         * ------------------------------------------------------------------ */
        case PROC_NEW_IDLE:
            break;

        /* ------------------------------------------------------------------
         * PROC_NEW_BELT_RUN：同步带 CW 一直运转，切割电机(PC6)全程高电平
         *   - PC6 一直高（切割电机持续转）
         *   - IR计数≥3 → PC6拉低，停止同步带，进入PACK流程
         * ------------------------------------------------------------------ */
        case PROC_NEW_BELT_RUN: {
            if (seg == 0u) {
                Stepper_SyncRun_Motor1Reversed(DFLT_BELT_SPEED_HZ, STEPPER_DIR_CW);
                FAN_E6_On();
                GPIOC->BSRR = GPIO_Pin_6;  /* PC6 高电平，切割电机启动 */
                UART1_PrintfNB("[NEW] BELT_RUN start (PC6 ON)\r\n");
            }

            /* IR≥3 → PC6拉低，停止同步带，进入PACK流程 */
            if (s_ir_cnt >= 3u) {
                FAN_E6_Off();
                GPIOC->BRR = GPIO_Pin_6;  /* PC6 低电平，切割电机停止 */
                Stepper_SyncStop();
                UART1_PrintfNB("[NEW] IR cnt=%u, enter PACK (PC6 OFF)\r\n", s_ir_cnt);
                s_pack_start_tick = now_tick;
                s_pack_m4_started = 0u;
                g_pack_test_total_ms = 0u;
                g_new_proc_state = PROC_NEW_PACK_IMPELLER;
                g_new_proc_seg_ms = 0u;
                break;
            }

            seg++;
            g_new_proc_seg_ms = seg;
            break;
        }

        /* ------------------------------------------------------------------
         * PROC_NEW_CUT_RUN：（保留，不再被BELT_RUN自动进入，仅作手动触发用）
         * ------------------------------------------------------------------ */
        case PROC_NEW_CUT_RUN: {
            if (seg == 0u) {
                FAN_E6_On();
                GPIOC->BSRR = GPIO_Pin_6;  /* PC6 高电平，切割电机启动 */
                UART1_PrintfNB("[NEW] CUT_RUN start (PC6)\r\n");
            }

            seg++;

            if (seg >= 1500u) {
                FAN_E6_Off();
                GPIOC->BRR = GPIO_Pin_6;  /* PC6 低电平，切割电机停止 */
                UART1_PrintfNB("[NEW] CUT_RUN stop (PC6)\r\n");
                g_new_proc_state = PROC_NEW_BELT_RUN;
                g_new_proc_seg_ms = 0u;
                seg = 0u;
                break;
            }

            g_new_proc_seg_ms = seg;
            break;
        }

        /* ------------------------------------------------------------------
         * PROC_NEW_PACK_IMPELLER：M3 + 叶轮3/8 启动
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_IMPELLER: {
            if (seg == 0u) {
                ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID1, 1u);
                ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID2, 1u);
                ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID1, M34_SCS_WHEEL_SPD, M34_SCS_WHEEL_ACC);
                ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID2, M34_SCS_WHEEL_SPD_ID8, M34_SCS_WHEEL_ACC);
                ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID1, M34_SCS_WHEEL_SPD);
                ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID2, M34_SCS_WHEEL_SPD_ID8);
                M3_Run(PACK_TEST_M3_SPEED_HZ, STEPPER_DIR_CW);
                UART1_PrintfNB("[NEW] PACK_IMPELLER start\r\n");
            }
            PackTest_ImpellerKeepalive();

            if (PACK_B7_LOW() && !s_pack_m4_started) {
                s_pack_m4_started = 1u;
                s_pack_m4_start_tick = now_tick;
                M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);
                g_new_proc_state = PROC_NEW_PACK_M4;
                g_new_proc_seg_ms = 0u;
                UART1_PrintfNB("[NEW] -> PACK_M4\r\n");
                break;
            }

            if (g_pack_test_total_ms >= PACK_TEST_IMPELLER_DURATION_MS) {
                PackTest_StopAll();
                g_new_proc_state = PROC_NEW_PACK_STOP;
                g_new_proc_seg_ms = 0u;
                break;
            }

            seg++;
            g_new_proc_seg_ms = seg;
            break;
        }

        /* ------------------------------------------------------------------
         * PROC_NEW_PACK_M4：M4 运行中（M3+叶轮继续）
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_M4: {
            static uint8_t s_entered;
            if (!s_entered) {
                s_entered = 1u;
                UART1_PrintfNB("[NEW] PACK_M4 started\r\n");
            }
            M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);

            if ((uint32_t)(now_tick - s_pack_m4_start_tick) >= (PACK_TEST_ROD_ON_MS - 1000u)) {
                s_pack_m4_started = 0u;
                s_entered = 0u;
                g_new_proc_state = PROC_NEW_PACK_ROD_ON_1;
                g_new_proc_seg_ms = 0u;
                UART1_PrintfNB("[NEW] -> PACK_ROD_ON_1\r\n");
                break;
            }

            if (g_pack_test_total_ms >= PACK_TEST_IMPELLER_DURATION_MS) {
                PackTest_StopAll();
                g_new_proc_state = PROC_NEW_PACK_STOP;
                g_new_proc_seg_ms = 0u;
                break;
            }

            seg++;
            g_new_proc_seg_ms = seg;
            break;
        }

        /* ------------------------------------------------------------------
         * PROC_NEW_PACK_ROD_ON_1：膜杆 2/7 第一节（PACK_TEST_ROD_LEG_MS）
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_ROD_ON_1: {
            static uint8_t s_rod_armed;
            static uint8_t s_entered;

            if (!s_entered) {
                s_entered = 1u;
                s_rod_armed = 0u;
                UART1_PrintfNB("[NEW] PACK_ROD_ON_1 entry\r\n");

                M3_Stop();
                M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);
                M34_ScsBus_StopWheelImpellers();

                int8_t d2 = PackTest_RodDirBetween(PACK_TEST_ROD_ID2_PULL, PACK_TEST_ROD_ID2_PUSH);
                int8_t d7 = PackTest_RodDirBetween(PACK_TEST_ROD_ID7_PULL, PACK_TEST_ROD_ID7_PUSH);
                int16_t sp2 = (d2 >= 0) ? (int16_t)PACK_TEST_ROD_SPEED_ID2 : (int16_t)(-((int16_t)PACK_TEST_ROD_SPEED_ID2));
                int16_t sp7 = (d7 >= 0) ? (int16_t)PACK_TEST_ROD_SPEED_ID7 : (int16_t)(-((int16_t)PACK_TEST_ROD_SPEED_ID7));

                if (d2 == 0 && d7 == 0) {
                    UART1_PrintfNB("[NEW] ROD_ON_1 aligned, skip\r\n");
                } else {
                    PackTest_RodWheelRunBoth(sp2, sp7);
                    s_rod_armed = 1u;
                }
            }

            if (seg >= PACK_TEST_ROD_LEG_MS) {
                s_entered = 0u;
                if (s_rod_armed) {
                    PackTest_RodWheelStopBoth();
                    s_rod_armed = 0u;
                }
                g_new_proc_state = PROC_NEW_PACK_ROD_OFF;
                g_new_proc_seg_ms = 0u;
                UART1_PrintfNB("[NEW] -> PACK_ROD_OFF\r\n");
                break;
            }

            seg++;
            g_new_proc_seg_ms = seg;
            break;
        }

        /* ------------------------------------------------------------------
         * PROC_NEW_PACK_ROD_OFF：B7上升沿等待（防误触延时）
         * 进入时启动M3+叶轮+M4（第一节膜杆结束后继续保持缠绕运动）
         * B7上升沿触发后停止M3+M4+叶轮，进入PACK_ROD_ON_2
         * 兜底：seg>=300000（5分钟超时）强制跳转
         * 注意：M3与叶轮绑定同步启停
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_ROD_OFF: {
            static uint8_t s_b7_wait_phase; /* 0=等低电平 1=等上升沿 */
            static uint8_t s_entered;

            /* ========== 初始化：启动M3+叶轮+M4 ========== */
            if (!s_entered) {
                s_entered = 1u;
                s_b7_wait_phase = 0u;
                UART1_PrintfNB("[NEW] PACK_ROD_OFF entry, starting M3+impeller+M4\r\n");

                /* 启动M3+叶轮+M4，保持缠绕运动 */
                M3_Run(PACK_TEST_M3_SPEED_HZ, STEPPER_DIR_CW);
                ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID1, 1u);
                ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID2, 1u);
                ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID1, M34_SCS_WHEEL_SPD, M34_SCS_WHEEL_ACC);
                ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID2, M34_SCS_WHEEL_SPD_ID8, M34_SCS_WHEEL_ACC);
                ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID1, M34_SCS_WHEEL_SPD);
                ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID2, M34_SCS_WHEEL_SPD_ID8);
                M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);
                seg = 1u;
                g_new_proc_seg_ms = seg;
                break;
            }

            /* ========== 持续运行M3+叶轮+M4 ========== */
            PackTest_ImpellerKeepalive();
            M3_Run(PACK_TEST_M3_SPEED_HZ, STEPPER_DIR_CW);
            M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);

            /* ========== 阶段0：强制等待B7低电平 ========== */
            if (s_b7_wait_phase == 0u) {
                if (PACK_B7_LOW()) {
                    s_b7_wait_phase = 1u;
                    UART1_PrintfNB("[NEW] B7 LOW, waiting for rising edge\r\n");
                }
            }
            /* ========== 阶段1：等待B7上升沿（低→高）========== */
            else if (s_b7_wait_phase == 1u) {
                if (PACK_B7_HIGH()) {
                    /* 上升沿触发：停止M3+M4+叶轮，进入膜杆第二节 */
                    M3_Stop();
                    M4_Stop();
                    M34_ScsBus_StopWheelImpellers();
                    s_entered = 0u;
                    s_b7_wait_phase = 0u;
                    g_new_proc_state = PROC_NEW_PACK_ROD_ON_2;
                    g_new_proc_seg_ms = 0u;
                    seg = 0u;
                    UART1_PrintfNB("[NEW] B7 rising edge -> PACK_ROD_ON_2\r\n");
                    break;
                }
            }

            /* ========== 超时兜底（5分钟）========== */
            if (seg >= 300000u) {
                M3_Stop();
                M4_Stop();
                M34_ScsBus_StopWheelImpellers();
                s_entered = 0u;
                s_b7_wait_phase = 0u;
                g_new_proc_state = PROC_NEW_PACK_ROD_ON_2;
                g_new_proc_seg_ms = 0u;
                seg = 0u;
                UART1_PrintfNB("[NEW] -> PACK_ROD_ON_2 (timeout)\r\n");
                break;
            }

            seg++;
            g_new_proc_seg_ms = seg;
            break;
        }

        /* ------------------------------------------------------------------
         * PROC_NEW_PACK_ROD_ON_2：膜杆 2/7 第二节（固定运行 PACK_TEST_ROD_LEG_MS）
         * 入口：停止M3/M4/叶轮 + 启动膜杆（第一节拉膜后的反向推回）
         * 运行 PACK_TEST_ROD_LEG_MS（510ms）后进入切膜流程
         * 注意：M3与叶轮绑定，已在PACK_ROD_OFF中停止
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_ROD_ON_2: {
            static uint8_t s_rod_armed;
            static uint8_t s_entered;

            /* ========== 初始化：启动膜杆第二节 ========== */
            if (!s_entered) {
                s_entered = 1u;
                s_rod_armed = 0u;
                UART1_PrintfNB("[NEW] PACK_ROD_ON_2 entry\r\n");

                M3_Stop();
                M4_Stop();
                M34_ScsBus_StopWheelImpellers();

                /* 启动膜杆第二节：计算从当前位置去目标位置的方向 */
                int8_t d2 = PackTest_RodDirBetween(PACK_TEST_ROD_ID2_PULL, PACK_TEST_ROD_ID2_PUSH);
                int8_t d7 = PackTest_RodDirBetween(PACK_TEST_ROD_ID7_PULL, PACK_TEST_ROD_ID7_PUSH);
                int16_t sp2 = (d2 >= 0) ? (int16_t)PACK_TEST_ROD_SPEED_ID2 : (int16_t)(-((int16_t)PACK_TEST_ROD_SPEED_ID2));
                int16_t sp7 = (d7 >= 0) ? (int16_t)PACK_TEST_ROD_SPEED_ID7 : (int16_t)(-((int16_t)PACK_TEST_ROD_SPEED_ID7));

                if (d2 == 0 && d7 == 0) {
                    UART1_PrintfNB("[NEW] ROD_ON_2 aligned, skip\r\n");
                } else {
                    PackTest_RodWheelRunBoth(sp2, sp7);
                    s_rod_armed = 1u;
                    UART1_PrintfNB("[NEW] ROD_ON_2: d2=%d d7=%d\r\n", d2, d7);
                }
            }

            /* ========== 固定运行 PACK_TEST_ROD_LEG_MS ========== */
            if (seg >= PACK_TEST_ROD_LEG_MS) {
                s_entered = 0u;
                if (s_rod_armed) {
                    PackTest_RodWheelStopBoth();
                    s_rod_armed = 0u;
                }
                g_new_proc_state = PROC_NEW_PACK_CUTFILM;
                g_new_proc_seg_ms = 0u;
                UART1_PrintfNB("[NEW] -> PACK_CUTFILM\r\n");
                break;
            }

            seg++;
            g_new_proc_seg_ms = seg;
            break;
        }

        /* ------------------------------------------------------------------
         * PROC_NEW_PACK_CUTFILM：膜杆停止后，进入此状态
         *   step0: 等灰度第1次触发
         *   step1: 等灰度第2次触发 → M4停
         *   step2: 切膜电机正转 500ms
         *   step3: 切膜电机反转 500ms → 停止 → 进入PACK_STOP
         * 兜底：超时直接进PACK_STOP
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_CUTFILM: {
            static uint8_t  s_film_step;
            static uint16_t s_film_delay_ms;
            static uint8_t  s_entered;

            if (!s_entered) {
                s_entered = 1u;
                s_film_step = 0u;
                s_film_delay_ms = 0u;
                UART1_PrintfNB("[FILM] entry, waiting gray 1st trigger\r\n");

                M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);
                ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID1, 1u);
                ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID2, 1u);
                ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID1, M34_SCS_WHEEL_SPD, M34_SCS_WHEEL_ACC);
                ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID2, M34_SCS_WHEEL_SPD_ID8, M34_SCS_WHEEL_ACC);
                ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID1, M34_SCS_WHEEL_SPD);
                ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID2, M34_SCS_WHEEL_SPD_ID8);
            }

            PackTest_ImpellerKeepalive();

            /* step0~step1: M4保持运转；step2开始M4已停 */
            if (s_film_step <= 1u) {
                M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);
            }

            /* step0: 等灰度第1次触发 */
            if (s_film_step == 0u) {
                if (GRAY_SENSOR_LOW) {
                    s_film_step = 1u;
                    UART1_PrintfNB("[FILM] gray1 trig, waiting gray 2nd trigger\r\n");
                }
            }
            /* step1: 等灰度第2次触发 → M4停 */
            else if (s_film_step == 1u) {
                if (GRAY_SENSOR_LOW) {
                    M4_Stop();
                    ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID1, 0u);
                    ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID2, 0u);
                    s_film_step = 2u;
                    s_film_delay_ms = 0u;
                    UART1_PrintfNB("[FILM] gray2 trig, M4 stopped, CW ON\r\n");
                    /* 立即启动切膜电机正转 */
                    GPIO_SetBits(GPIOC, GPIO_Pin_9);
                    GPIO_SetBits(GPIOD, GPIO_Pin_8);
                    GPIO_ResetBits(GPIOD, GPIO_Pin_9);
                }
            }
            /* step2: 切膜电机正转 500ms */
            else if (s_film_step == 2u) {
                if (s_film_delay_ms >= 500u) {
                    /* 切换反转：PD8=0, PD9=1 */
                    GPIO_ResetBits(GPIOD, GPIO_Pin_8);
                    GPIO_SetBits(GPIOD, GPIO_Pin_9);
                    s_film_step = 3u;
                    s_film_delay_ms = 0u;
                    UART1_PrintfNB("[FILM] CCW ON\r\n");
                }
            }
            /* step3: 切膜电机反转 500ms → 停止 → PACK_STOP */
            else if (s_film_step == 3u) {
                if (s_film_delay_ms >= 500u) {
                    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
                    GPIO_ResetBits(GPIOD, GPIO_Pin_8);
                    GPIO_ResetBits(GPIOD, GPIO_Pin_9);
                    UART1_PrintfNB("[FILM] motor stopped -> PACK_STOP\r\n");
                    s_entered = 0u;
                    g_new_proc_state = PROC_NEW_PACK_STOP;
                    g_new_proc_seg_ms = 0u;
                    seg = 0u;
                    break;
                }
            }

            /* 切膜电机计时 */
            if (s_film_step >= 2u && s_film_step <= 3u) {
                s_film_delay_ms++;
            }

            /* 超时兜底（30s），直接进PACK_STOP */
            seg++;
            g_new_proc_seg_ms = seg;
            if (seg >= 30000u) {
                M4_Stop();
                GPIO_ResetBits(GPIOC, GPIO_Pin_9);
                GPIO_ResetBits(GPIOD, GPIO_Pin_8);
                GPIO_ResetBits(GPIOD, GPIO_Pin_9);
                s_entered = 0u;
                g_new_proc_state = PROC_NEW_PACK_STOP;
                g_new_proc_seg_ms = 0u;
                seg = 0u;
                UART1_PrintfNB("[FILM] timeout -> PACK_STOP\r\n");
                break;
            }
            break;
        }

        /* ------------------------------------------------------------------
         * PROC_NEW_PACK_STOP：全部停止，等待2s后重新启动同步带切割计数流程
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_STOP: {
            static uint8_t s_entered;
            static uint8_t s_restart_delay_done;
            static uint32_t s_stop_tick;

            if (!s_entered) {
                s_entered = 1u;
                s_restart_delay_done = 0u;
                s_stop_tick = now_tick;
                UART1_PrintfNB("[NEW] PACK_STOP entry\r\n");
            }

            PackTest_StopAll();

            /* 2s后重启同步带切割计数流程 */
            if (!s_restart_delay_done) {
                if ((uint32_t)(now_tick - s_stop_tick) >= 2000u) {
                    s_restart_delay_done = 1u;
                    UART1_PrintfNB("[NEW] 2000ms delay done, restart BELT_RUN\r\n");
                }
            } else {
                /* 2s到，重启BELT_RUN */
                s_entered = 0u;
                s_restart_delay_done = 0u;
                g_new_proc_state = PROC_NEW_BELT_RUN;
                g_new_proc_seg_ms = 0u;
                g_pack_test_total_ms = 0u;
                s_pack_start_tick = now_tick;
                s_pack_m4_started = 0u;
                /* 重置IR计数，继续计数流程 */
                s_ir_cnt = 0u;
                s_ir_triggered = 0u;
                g_ir_reset_request = 1u;
                break;
            }

            seg++;
            g_new_proc_seg_ms = seg;
            break;
        }

        default:
            g_new_proc_state = PROC_NEW_IDLE;
            break;
        }
    }

    /* * 【分区 G】调试：每 MAIN_PACK_ULTRA_TEST_PERIOD_MS 置标志，
     * 主循环调用 Process_ReadPackDistance() 并打印距离。
     * ======================================================================== */
#if MAIN_PACK_ULTRA_TEST_ENABLE
    {
        static uint16_t s_ultra_test_div1ms;

        if (++s_ultra_test_div1ms >= MAIN_PACK_ULTRA_TEST_PERIOD_MS) {
            s_ultra_test_div1ms = 0u;
            s_pack_ultra_test_pending = 1u;
        }
    }
#endif
}
/* ============================================================================
 * 【新工艺流程辅助函数】
 * ============================================================================ */

/* 阻塞微秒延时（使用系统时钟）*/
static void delay_us(uint32_t us)
{
    volatile uint32_t i;
    for (i = 0; i < us * 72 / 3; i++) {
        __asm volatile ("nop");
    }
}

/* 读取超声波距离（HC-SR04，PB0 Trig / PD3 Echo）
 * 返回值：距离(mm)，0xFFFF 表示超时无回响 */
static uint16_t New_ReadUltrasonicDistance(void)
{
    uint32_t t1, t2, diff;
    uint32_t timeout;
    uint16_t distance;
    uint8_t echo_ever_high = 0;

    /* 读取 TRIG/ECHO 原始状态（用于调试打印）*/
    (void)GPIO_ReadInputDataBit(PACK_SENSOR_TRIG_PORT, PACK_SENSOR_TRIG_PIN);
    (void)GPIO_ReadInputDataBit(PACK_SENSOR_ECHO_PORT, PACK_SENSOR_ECHO_PIN);

    /* 发送 10us 触发脉冲 */
    GPIO_SetBits(PACK_SENSOR_TRIG_PORT, PACK_SENSOR_TRIG_PIN);
    delay_us(10);
    GPIO_ResetBits(PACK_SENSOR_TRIG_PORT, PACK_SENSOR_TRIG_PIN);

    t1 = t2 = 0;
    timeout = 25000;

    /* 等待 ECHO 上升沿 */
    while (GPIO_ReadInputDataBit(PACK_SENSOR_ECHO_PORT, PACK_SENSOR_ECHO_PIN) == Bit_RESET) {
        if (--timeout == 0) {
            return 0xFFFF;
        }
    }
    echo_ever_high = 1;
    t1 = SysTick->VAL;

    /* 等待 ECHO 下降沿 */
    timeout = 25000;
    while (GPIO_ReadInputDataBit(PACK_SENSOR_ECHO_PORT, PACK_SENSOR_ECHO_PIN) != Bit_RESET) {
        if (timeout-- == 0) {
            return 0xFFFF;
        }
    }
    t2 = SysTick->VAL;

    /* 计算时间差 */
    if (t1 >= t2) diff = t1 - t2;
    else diff = (SysTick->LOAD - t2) + t1;

    /* 转换为 mm：diff / 72MHz * 340m/s / 2 = diff / 4235 ≈ diff * 17 / 72000 * 10 */
    distance = (uint16_t)((diff * 17U) / 7200U);
    if (distance > 4000) distance = 4000;
    return distance;
}

/* ============================================================================
 * 【串口硬件回调区】放在滴答函数下面、其它 static 函数上面。
 * 说明：USARTx_IRQHandler（在 bsp_usart_obj.c）收到中断后会调用这里的函数。
 * 每个串口各写一组 Rx / Tx，互不混用，方便以后改协议。
 * ============================================================================ */

/**
 * USART1（PA9/PA10，调试口 + HX711 命令）
 * Rx：交给称重模块解析；同时写入环形缓冲，便于其它代码 UART1_ReadByte 取数据。
 * Tx：配合 s_usart1_tx_msg 队列，在「发送寄存器空」中断里逐字节发送。
 */
void USART1_HW_OnRxByte(uint8_t b)
{
    // HX711_OnByte(b);  /* HX711 已注释掉 */
    UART_ISR_FeedRx(UART1, b);
}

void USART1_HW_OnTxRegEmpty(void)
{
    uint8_t ch;

    /* 队列为空，关闭TXE中断 */
    if (s_tx_count == 0) {
        USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        return;
    }

    /* 获取当前消息和位置 */
    ch = (uint8_t)s_tx_queue[s_tx_head].data[s_tx_queue[s_tx_head].idx];

    /* 发送当前字节 */
    USART_SendData(USART1, (uint16_t)ch);
    s_tx_queue[s_tx_head].idx++;

    /* 检查当前消息是否发送完成 */
    if (s_tx_queue[s_tx_head].idx >= s_tx_queue[s_tx_head].len) {
        /* 移出当前消息 */
        s_tx_head = (s_tx_head + 1) % UART1_TX_QUEUE_SIZE;
        s_tx_count--;

        /* 如果还有消息，下一个字节会自动发送 */
        /* TXE中断保持开启，因为s_tx_count > 0 */
    }
    /* 如果还有数据待发送，TXE中断会自动再次触发 */
}

/**
 * USART2（PA2/PA3，例如接 Jetson）
 * 默认行为：只把字节放入 UART2 接收环形缓冲，协议由上层 ReadByte/Available 处理。
 */
void USART2_HW_OnRxByte(uint8_t b)
{
    UART_ISR_FeedRx(UART2, b);
}

void USART2_HW_OnTxRegEmpty(void)
{
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
}

/**
 * USART3（PB10/PB11，例如 TJC 串口屏）
 *
 * TJC 串口屏协议：直接发送命令字节，无复杂握手
 * 帧格式: [AA] [CMD] [LEN] [DATA...] [CS]
 *   例如: AA 30 00 9A  (开始流程)
 *
 * 注意：TJC 屏幕不处理 MCU 的 ACK 帧，所以这里只解析命令并设置标志，
 *       由主循环执行，不回传任何数据给 TJC。
 */
void USART3_HW_OnRxByte(uint8_t b)
{
    static uint8_t s_tjc_state = 0;    /* 0=等待帧头, 1=命令, 2=长度, 3=数据, 4=校验 */
    static uint8_t s_tjc_cmd = 0;
    static uint8_t s_tjc_len = 0;
    static uint8_t s_tjc_data[32];
    static uint8_t s_tjc_data_idx = 0;
    static uint8_t s_tjc_calc_cs = 0;

    switch (s_tjc_state) {
        case 0: /* 等待帧头 AA */
            if (b == 0xAA) {
                s_tjc_calc_cs = b;
                s_tjc_state = 1;
            }
            break;

        case 1: /* 命令 */
            s_tjc_cmd = b;
            s_tjc_calc_cs ^= b;
            s_tjc_state = 2;
            break;

        case 2: /* 长度 */
            s_tjc_len = b;
            s_tjc_calc_cs ^= b;
            if (s_tjc_len > 32) {
                s_tjc_len = 32;
            }
            s_tjc_data_idx = 0;
            s_tjc_state = (s_tjc_len > 0) ? 3 : 4;
            break;

        case 3: /* 数据 */
            s_tjc_data[s_tjc_data_idx++] = b;
            s_tjc_calc_cs ^= b;
            if (s_tjc_data_idx >= s_tjc_len) {
                s_tjc_state = 4;
            }
            break;

        case 4: /* 校验 */
            if (b == s_tjc_calc_cs) {
                /* 校验通过，处理 TJC 命令 */
                switch (s_tjc_cmd) {
                    /* ------------------------------------------------------------
                     * 0x9A：新工艺流程启动命令（串口屏发送此命令后开始计数流程）
                     * ------------------------------------------------------------*/
                    case 0x9A: /* 新工艺流程启动命令 */
                        if (g_new_proc_state == PROC_NEW_IDLE && !g_protocol_process_start_request) {
                            if (s_screw_boot_st == SCRW_DONE) {
                                g_protocol_process_start_request = 1;
                                g_protocol_last_cmd = s_tjc_cmd;
                                g_protocol_cmd_ready = 1;
                            }
                        }
                        break;

                    case 0x30: /* 旧版开始流程（兼容）*/
                        if (g_proc_state == PROC_IDLE && !g_protocol_process_start_request) {
                            if (s_screw_boot_st == SCRW_DONE) {
                                g_process_run_request = 1u;
                                g_protocol_process_start_request = 1;
                                g_protocol_last_cmd = s_tjc_cmd;
                                g_protocol_cmd_ready = 1;
                            }
                        }
                        break;
                    case 0x31: /* 停止流程 */
                        if (!g_protocol_process_stop_request) {
                            g_protocol_process_stop_request = 1;
                            g_protocol_last_cmd = s_tjc_cmd;
                            g_protocol_cmd_ready = 1;
                        }
                        break;
                    case 0x32: /* 暂停流程 */
                        if (!g_protocol_process_pause_request) {
                            g_protocol_process_pause_request = 1;
                            g_protocol_last_cmd = s_tjc_cmd;
                            g_protocol_cmd_ready = 1;
                        }
                        break;
                    case 0x33: /* 恢复流程 */
                        g_protocol_process_resume_request = 1;
                        g_protocol_last_cmd = s_tjc_cmd;
                        g_protocol_cmd_ready = 1;
                        break;
                    case 0x2F: /* 急停 */
                        if (!g_protocol_emergency_stop_request) {
                            g_protocol_emergency_stop_request = 1;
                            g_protocol_last_cmd = s_tjc_cmd;
                            g_protocol_cmd_ready = 1;
                        }
                        break;
                    /* ---- 清洗模式 CMD_SET_CLEAN_MODE (0x34) ---- */
                    case 0x34: /* 清洗模式设置 data:[模式] */
                        if (s_tjc_len >= 1) {
                            g_clean_mode = (CleanMode_TypeDef)s_tjc_data[0];
                            Process_SetParam(0xF1, (uint16_t)s_tjc_data[0]);
                            g_protocol_last_cmd = s_tjc_cmd;
                            g_protocol_cmd_ready = 1;
                        }
                        break;
                    /* ---- 打包模式 CMD_SET_PACK_MODE (0x35) ---- */
                    case 0x35: /* 打包模式设置 data:[模式] */
                        if (s_tjc_len >= 1) {
                            g_pack_mode = (PackTriggerMode_TypeDef)s_tjc_data[0];
                            Process_SetParam(0xF2, (uint16_t)s_tjc_data[0]);
                            g_protocol_last_cmd = s_tjc_cmd;
                            g_protocol_cmd_ready = 1;
                        }
                        break;
                    /* ---- 通用参数设置 CMD_SET_PARAM (0x36) ---- */
                    case 0x36: /* 设置工艺参数 data:[ID][VH][VL] */
                        if (s_tjc_len >= 3) {
                            uint8_t param_id = s_tjc_data[0];
                            uint16_t value = ((uint16_t)s_tjc_data[1] << 8) | s_tjc_data[2];
                            Process_SetParam(param_id, value);
                            g_protocol_last_cmd = s_tjc_cmd;
                            g_protocol_cmd_ready = 1;
                        }
                        break;

                    /* ------------------------------------------------------------
                     * 0x40：分区H 打包测试启动命令
                     *   时序：舵机3+8转动 + 电机4转动（4s）
                     *        → 舵机2+7半圈 → 舵机2+7半圈 → 电机4停 + 舵机3+8停
                     *   适用：台架调试，不依赖超声/红外传感器
                     * ------------------------------------------------------------*/
                    case 0x40:
                        if (g_new_proc_state == PROC_NEW_IDLE) {
                            g_protocol_pack_test_request = 1;
                            g_protocol_last_cmd = s_tjc_cmd;
                            g_protocol_cmd_ready = 1;
                        }
                        break;
                    default:
                        /* 未知命令，记录调试 */
                        g_protocol_last_cmd = s_tjc_cmd;
                        g_protocol_cmd_ready = 1;
                        break;
                }
            }
            s_tjc_state = 0;
            break;

        default:
            s_tjc_state = 0;
            break;
    }

    /* 写入环形缓冲（供其他代码查询） */
    UART_ISR_FeedRx(UART3, b);
}

void USART3_HW_OnTxRegEmpty(void)
{
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
}

/**
 * UART4（PC10/PC11，例如飞特总线舵机或兼容设备）
 */
void UART4_HW_OnRxByte(uint8_t b)
{
    UART_ISR_FeedRx(UART4_ID, b);
}

void UART4_HW_OnTxRegEmpty(void)
{
    USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
}

/* -------------------------------------------------------------------------- */
/* 其余 static 辅助函数 */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* USART1 TX 队列实现 - 支持多消息排队，不丢消息 */
/* -------------------------------------------------------------------------- */
static void UART1_PutStr(const char *str)
{
    uint16_t i, len;
    uint8_t next_tail;
    uint32_t primask;

    if (str == NULL || *str == '\0') {
        return;
    }

    /* 计算消息长度 */
    len = 0;
    while (str[len] != '\0' && len < UART1_TX_BUF_SIZE - 1) {
        len++;
    }

    /* 禁用中断保护队列操作 */
    primask = __get_PRIMASK();
    __disable_irq();

    /* 检查队列是否满 */
    if (s_tx_count >= UART1_TX_QUEUE_SIZE) {
        __set_PRIMASK(primask);
        return;  /* 队列满，丢弃消息（可以改为覆盖最旧的） */
    }

    /* 复制消息到队列 */
    for (i = 0; i < len; i++) {
        s_tx_queue[s_tx_tail].data[i] = str[i];
    }
    s_tx_queue[s_tx_tail].data[len] = '\0';
    s_tx_queue[s_tx_tail].len = len;
    s_tx_queue[s_tx_tail].idx = 0;

    /* 更新尾指针 */
    next_tail = (s_tx_tail + 1) % UART1_TX_QUEUE_SIZE;
    s_tx_tail = next_tail;
    s_tx_count++;

    /* 如果之前没有消息在发送，立即启动TX */
    if (s_tx_count == 1) {
        USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    }

    __set_PRIMASK(primask);
}

/* 兼容性别名 - 保留旧函数名以减少修改量 */
void usart1_tx_start_isr(const char *str)
{
    UART1_PutStr(str);
}

static void uart1_reapply_af_pins(void)
{
    GPIO_InitTypeDef g;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Pin   = GPIO_Pin_9;
    GPIO_Init(GPIOA, &g);
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    g.GPIO_Pin  = GPIO_Pin_10;
    GPIO_Init(GPIOA, &g);
}

void UART1_PollingTxPreempt(void)
{
    uint32_t primask;
    primask = __get_PRIMASK();
    __disable_irq();

    /* 清空整个TX队列 */
    s_tx_head = 0;
    s_tx_tail = 0;
    s_tx_count = 0;

    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

    __set_PRIMASK(primask);
}

static void GPIO_Init_All(void)
{
    GPIO_InitTypeDef g;

    /* 使能所有需要用到的GPIO时钟：GPIOC(LED/按钮)、GPIOE(红外+E0/E1限位)、GPIOB(超声Trig)、GPIOD(超声Echo) */
    RCC_APB2PeriphClockCmd(LED_RCC | START_BTN_RCC | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC, ENABLE);

    /* LED PC13 - 推挽输出 */
    g.GPIO_Pin   = LED_GPIO_PIN;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_GPIO_PORT, &g);
    GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN);

    /* PC6 切割电机 - 推挽输出（高电平转，低电平停，超声波触发）*/
    g.GPIO_Pin   = GPIO_Pin_6;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &g);
    GPIO_ResetBits(GPIOC, GPIO_Pin_6);  /* 初始低电平，切割电机停止 */

    /* PC9 切膜电机 ENA - 推挽输出（高电平使能，低电平禁用）*/
    g.GPIO_Pin   = GPIO_Pin_9;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &g);
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);  /* 初始低电平，切膜电机失能 */

    /* PD8/PD9 切膜电机方向 - 推挽输出
     * PD8高+PD9低=正转，PD8低+PD9高=反转，PD8=PD9=停止 */
    g.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &g);
    GPIO_ResetBits(GPIOD, GPIO_Pin_8 | GPIO_Pin_9);  /* 初始停止状态 */

    /* PC1(启动按钮) + PC5(自检复位按钮) - 输入上拉 */
    g.GPIO_Pin   = START_BTN_PIN | M34_RST_BTN_PIN;
    g.GPIO_Mode  = GPIO_Mode_IPU;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(START_BTN_PORT, &g);

    /* PE8 红外传感器 - 输入上拉（遮光=有葱，低电平触发）*/
    g.GPIO_Pin   = PACK_IR_SENSOR_PIN;
    g.GPIO_Mode  = GPIO_Mode_IPU;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(PACK_IR_SENSOR_PORT, &g);

    /* PB0 超声波 Trig - 推挽输出（HC-SR04触发信号） */
    g.GPIO_Pin   = PACK_SENSOR_TRIG_PIN;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PACK_SENSOR_TRIG_PORT, &g);
    GPIO_ResetBits(PACK_SENSOR_TRIG_PORT, PACK_SENSOR_TRIG_PIN);

    /* PD3 超声波 Echo - 浮空输入（HC-SR04回响信号，输出3.3V） */
    g.GPIO_Pin   = PACK_SENSOR_ECHO_PIN;
    g.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(PACK_SENSOR_ECHO_PORT, &g);

    /* PE0=E0上限位 / PE1=E1下限位 - 输入上拉（触碰接地=低电平=限位触发） */
    g.GPIO_Pin   = SCREW_LIMIT_E0_PIN | SCREW_LIMIT_E1_PIN;
    g.GPIO_Mode  = GPIO_Mode_IPU;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(SCREW_LIMIT_E0_PORT, &g);

    /* PE7 灰度传感器 - 输入上拉（悬空=高电平，遮挡=低电平）*/
    g.GPIO_Pin   = GPIO_Pin_7;
    g.GPIO_Mode  = GPIO_Mode_IPU;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOE, &g);

    /* PB6=IR计数器 / PB7=B7到位检测 - 输入上拉
     * 注意：PB6/PB7 在 SERVO_InitAll() 中被配置为 TIM4 PWM 复用功能，
     * 必须在此处重新抢占为普通 GPIO 输入，否则 IR/B7 检测不到电平变化。 */
    g.GPIO_Pin   = IR_COUNTER_PIN | PACK_B7_SENSOR_PIN;
    g.GPIO_Mode  = GPIO_Mode_IPU;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &g);
}
