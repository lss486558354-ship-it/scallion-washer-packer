/**
 ******************************************************************************
 * 工艺流程控制源文件（V3.0 - 4分区架构）
 *
 * 功能：
 *   - 分区A：夹入切割（超声检测+丝杆调整）
 *   - 分区B：清洗吹气
 *   - 分区C：等待到位（计数判断）
 *   - 分区D：打包（红外+超声双重检测）
 *
 * 版本：V3.0
 * 日期：2026-04-18
 ******************************************************************************
 */

#include "process_control.h"
#include "bsp_tick.h"
#include "bsp_stepper.h"
#include "bsp_dc_motor_obj.h"
#include "bsp_servo_serial.h"
#include "bsp_servo_obj.h"
#include "bsp_relay.h"
#include "device.h"
#include "main.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <string.h>
#include "bsp_usart_obj.h"

/* 外部变量声明 */
extern volatile uint32_t g_tick_ms;
extern volatile uint8_t g_protocol_emergency_stop_request;
extern volatile uint8_t g_protocol_process_pause_request;
extern void usart1_tx_start_isr(const char *str);

/*-----------------------------------------------------------
 * 宏定义
 *-----------------------------------------------------------*/
#define PROCESS_TICK()       (g_tick_ms)
#define PROCESS_ELAPSED(ms)  ((uint32_t)(g_tick_ms - g_process_status.tick_start) >= (uint32_t)(ms))
#define PACK_ELAPSED(ms, start_time)    ((uint32_t)(g_tick_ms - (start_time)) >= (uint32_t)(ms))

/* 飞特总线ID */
#define PACK_SCS_IMPELLER_ID_A   3u
#define PACK_SCS_IMPELLER_ID_B   8u
#define PACK_SCS_IMPELLER_SPD_B(sp) ((int16_t)(-(int16_t)(sp)))
#define PACK_SCS_ROD_ID_A        2u
#define PACK_SCS_ROD_ID_B        7u

/*-----------------------------------------------------------
 * 本地辅助函数声明
 *-----------------------------------------------------------*/
static void delay_us(uint32_t us);
static uint16_t read_ultrasonic_distance(GPIO_TypeDef* trig_port, uint16_t trig_pin,
                                          GPIO_TypeDef* echo_port, uint16_t echo_pin);
static void pack_scs_impeller_stop(void);
static void pack_scs_impeller_clamp_run(void);
static void pack_scs_impeller_eject_run(void);
static void pack_scs_rod_wheel_run_both(int16_t sp2, int16_t sp7);
static void pack_scs_rod_wheel_zero_both(void);
static void pack_scs_rod_wheel_stop_final(uint8_t torque_off_after);
static void pack_rod_pull_speeds(int16_t *out2, int16_t *out7);
static void pack_rod_return_speeds(int16_t *out2, int16_t *out7);
static uint8_t pack_impeller_acc_u8(void);
static uint32_t pack_rod_wheel_leg_ms(void);
static int16_t pack_rod_wheel_mag(void);

/*-----------------------------------------------------------
 * 全局变量定义
 *-----------------------------------------------------------*/

/* 工艺流程状态 */
ProcessStatus_TypeDef g_process_status = {
    PROCESS_IDLE,          /* phase */
    PROCESS_IDLE,          /* phase_saved */
    CLAMP_IDLE,           /* clamp_state */
    CLAMP_IDLE,           /* clamp_state_saved */
    CLEAN_IDLE,           /* clean_state */
    CLEAN_IDLE,           /* clean_state_saved */
    WAIT_IDLE,            /* wait_state */
    WAIT_IDLE,            /* wait_state_saved */
    PACK_ZONE_IDLE,       /* pack_zone_state */
    PACK_ZONE_IDLE,       /* pack_zone_state_saved */
    CLEAN_MODE_BOTH,      /* clean_mode */
    PACK_MODE_FULL,       /* pack_mode */
    0,                    /* tick_start */
    0,                    /* tick_start_saved */
    PROCESS_IDLE,          /* phase_emstop_backup */
    CLAMP_IDLE,           /* clamp_state_emstop_backup */
    CLEAN_IDLE,           /* clean_state_emstop_backup */
    WAIT_IDLE,            /* wait_state_emstop_backup */
    PACK_ZONE_IDLE,       /* pack_zone_state_emstop_backup */
    0,                    /* tick_start_emstop_backup */
    0,                    /* emstop_backup_valid */
    SCREW_STOPPED,        /* screw_dir */
    0,                    /* screw_tick_start */
    0,                    /* onion_count */
    0,                    /* onion_count_flag */
    0,                    /* pack_head_seen */
    0,                    /* pack_tail_seen */
    0,                    /* pack_wrap_done */
    0,                    /* pack_cut_done */
    0,                    /* pack_ir_detected */
    0,                    /* reserved */
    0,                    /* pack_wrap_count */
    0,                    /* pack_rod_tick_start */
    0,                    /* pack_rod_leg */
};

/* 工艺参数 */
ProcessParams_TypeDef g_process_params = {
    DFLT_BELT_SPEED_HZ,          /* belt_speed_hz */
    DFLT_BELT_FEED_DELAY_MS,    /* belt_feed_delay_ms */
    1500,                       /* clamp_motor_speed_hz */
    2000,                       /* clamp_grip_time_ms */
    CLAMP_US_THRESHOLD_MM,       /* clamp_us_threshold_mm */
    5000,                       /* clamp_us_timeout_ms */
    DFLT_SCREW_SPEED_HZ,         /* screw_speed_hz */
    DFLT_SCREW_DOWN_DELAY_MS,    /* screw_down_delay_ms */
    DFLT_SCREW_LIMIT_DOWN_MS,    /* screw_limit_down_ms */
    DFLT_SCREW_LIMIT_UP_MS,      /* screw_limit_up_ms */
    DFLT_SCREW_TARGET_FIXED,     /* screw_target_fixed */
    DFLT_CUTTER_SPEED_DUTY,      /* cutter_speed_duty */
    DFLT_CUTTING_TIME_MS,        /* cutting_time_ms */
    DFLT_CLEAN_AIR_TIME_MS,      /* clean_air_time_ms */
    DFLT_CLEAN_WATER_TIME_MS,    /* clean_water_time_ms */
    1000,                        /* clean_wait_enter_ms */
    DFLT_SPRAY_DELAY_MS,         /* spray_delay_ms */
    DFLT_WATER_DELAY_MS,         /* water_delay_ms */
    2000,                        /* wait_delay_ms */
    50,                          /* wait_us_trigger_mm */
    DFLT_CONVEYOR_SPEED_HZ,      /* conveyor_speed_hz */
    DFLT_PACK_SPEED_HZ,           /* pack_speed_hz */
    DFLT_PACK_WRAP_TIME_MS,      /* pack_wrap_time_ms */
    DFLT_PACK_MIDDLE_DELAY_MS,   /* pack_middle_delay_ms */
    DFLT_PACK_HEAD_INTERVAL_MS,   /* pack_head_interval_ms */
    DFLT_PACK_CUT_DELAY_MS,      /* pack_cut_delay_ms */
    DFLT_PACK_SERVO_START,        /* pack_servo_angle_start */
    DFLT_PACK_SERVO_CUT,         /* pack_servo_angle_cut */
    DFLT_PACK_DIST_HEAD,         /* pack_dist_head */
    DFLT_PACK_DIST_TAIL,         /* pack_dist_tail */
    DFLT_PACK_TIMEOUT_MS,         /* pack_timeout_ms */
    DFLT_PACK_ALIGN_REVERSE_MS,   /* pack_align_reverse_ms */
    DFLT_PACK_M3_MS_PER_REV,     /* pack_m3_ms_per_rev */
    DFLT_PACK_IMPELLER_SPD,       /* pack_impeller_wheel_spd */
    DFLT_PACK_IMPELLER_ACC,       /* pack_impeller_acc */
    DFLT_PACK_IMPELLER_EJECT_MS,  /* pack_impeller_eject_ms */
    DFLT_PACK_IMPELLER_EJ_SPD3,   /* pack_impeller_eject_spd_id3 */
    DFLT_PACK_IMPELLER_EJ_SPD8,   /* pack_impeller_eject_spd_id8 */
    DFLT_PACK_ROD_GOAL_TIME,      /* pack_scs_rod_goal_time */
    DFLT_PACK_ROD_GOAL_SPEED,     /* pack_scs_rod_goal_speed */
    DFLT_PACK_ROD_ID2_PULL,       /* pack_scs_rod_id2_pull */
    DFLT_PACK_ROD_ID2_PUSH,       /* pack_scs_rod_id2_push */
    DFLT_PACK_ROD_ID7_PULL,       /* pack_scs_rod_id7_pull */
    DFLT_PACK_ROD_ID7_PUSH,       /* pack_scs_rod_id7_push */
    2000,                         /* pack_film_cw_ms */
    2000,                         /* pack_film_ccw_ms */
    4000,                         /* pack_eject_run_ms */
    DFLT_PROCESS_PLAN,           /* process_plan */
    3,                          /* target_count */
};

/* 传感器数据 */
ProcessSensors_TypeDef g_process_sensors = {0};

/* 当前模式 */
CleanMode_TypeDef g_clean_mode = CLEAN_MODE_BOTH;
PackTriggerMode_TypeDef g_pack_mode = PACK_MODE_FULL;

/* 打包模拟变量（无超声时使用）*/
static uint32_t s_pack_us_sim_enter_tick;
static uint32_t s_pack_us_sim_leave_tick;

/*-----------------------------------------------------------
 * 本地辅助函数实现
 *-----------------------------------------------------------*/

static void delay_us(uint32_t us)
{
    volatile uint32_t i;
    for (i = 0; i < us * 72 / 3; i++) { __asm volatile ("nop"); }
}

static uint8_t pack_impeller_acc_u8(void)
{
    uint16_t a = g_process_params.pack_impeller_acc;
    if (a == 0u) return 40u;
    if (a > 255u) return 255u;
    return (uint8_t)a;
}

static uint32_t pack_rod_wheel_leg_ms(void)
{
    uint32_t t = (uint32_t)g_process_params.pack_scs_rod_goal_time;
    return (t == 0u) ? 960u : t;
}

static int16_t pack_rod_wheel_mag(void)
{
    uint16_t u = g_process_params.pack_scs_rod_goal_speed;
    if (u == 0u) return (int16_t)DFLT_PACK_ROD_GOAL_SPEED;
    if (u > 32767u) return 32767;
    return (int16_t)u;
}

#define PACK_ROD_STS_STEPS_PER_REV  4096u
#define PACK_ROD_SHORTARC_HALF       2048u

static int8_t pack_rod_dir_between(uint16_t from_pos, uint16_t to_pos)
{
    uint32_t d = ((uint32_t)to_pos + PACK_ROD_STS_STEPS_PER_REV -
                  (uint32_t)from_pos) % PACK_ROD_STS_STEPS_PER_REV;
    if (d == 0u) return 1;
    return (d <= PACK_ROD_SHORTARC_HALF) ? 1 : -1;
}

static void pack_rod_pull_speeds(int16_t *out2, int16_t *out7)
{
    uint16_t p2s = g_process_params.pack_scs_rod_id2_push;
    uint16_t p2p = g_process_params.pack_scs_rod_id2_pull;
    uint16_t p7s = g_process_params.pack_scs_rod_id7_push;
    uint16_t p7p = g_process_params.pack_scs_rod_id7_pull;
    int8_t   d2  = pack_rod_dir_between(p2s, p2p);
    int8_t   d7  = pack_rod_dir_between(p7s, p7p);
    int16_t  mag = pack_rod_wheel_mag();

    *out2 = (d2 > 0) ? mag : (int16_t)(-mag);
    *out7 = (d7 > 0) ? mag : (int16_t)(-mag);
}

static void pack_rod_return_speeds(int16_t *out2, int16_t *out7)
{
    uint16_t p2p = g_process_params.pack_scs_rod_id2_pull;
    uint16_t p2s = g_process_params.pack_scs_rod_id2_push;
    uint16_t p7p = g_process_params.pack_scs_rod_id7_pull;
    uint16_t p7s = g_process_params.pack_scs_rod_id7_push;
    int8_t   d2  = pack_rod_dir_between(p2p, p2s);
    int8_t   d7  = pack_rod_dir_between(p7p, p7s);
    int16_t  mag = pack_rod_wheel_mag();

    *out2 = (d2 > 0) ? mag : (int16_t)(-mag);
    *out7 = (d7 > 0) ? mag : (int16_t)(-mag);
}

static void pack_scs_impeller_stop(void)
{
    /* 关扭矩须在速度写入后执行，避免扭矩残留导致舵机继续转 */
    ServoBus_STS_WriteSpeedAcc(PACK_SCS_IMPELLER_ID_A, 0, pack_impeller_acc_u8());
    ServoBus_STS_WriteSpeedAcc(PACK_SCS_IMPELLER_ID_B, 0, pack_impeller_acc_u8());
    ServoBus_SetTorqueSwitch(PACK_SCS_IMPELLER_ID_A, 0u);
    ServoBus_SetTorqueSwitch(PACK_SCS_IMPELLER_ID_B, 0u);
}

static void pack_scs_impeller_clamp_run(void)
{
    uint8_t  acc = pack_impeller_acc_u8();
    int16_t sp = (int16_t)g_process_params.pack_impeller_wheel_spd;

    /* 只写一次速度即可 */
    ServoBus_SetTorqueSwitch(PACK_SCS_IMPELLER_ID_A, 1u);
    ServoBus_SetTorqueSwitch(PACK_SCS_IMPELLER_ID_B, 1u);
    ServoBus_STS_WriteSpeedAcc(PACK_SCS_IMPELLER_ID_A, sp, acc);
    ServoBus_STS_WriteSpeedAcc(PACK_SCS_IMPELLER_ID_B, PACK_SCS_IMPELLER_SPD_B(sp), acc);
}

static void pack_scs_impeller_eject_run(void)
{
    /* 速度方向：ID3正转=正，ID8反转=负（与打包传送带配合） */
    int16_t s3 = (int16_t)g_process_params.pack_impeller_eject_spd_id3;
    int16_t s8 = (int16_t)g_process_params.pack_impeller_eject_spd_id8;
    if (g_process_params.pack_impeller_eject_spd_id8 == 0u) {
        s8 = (int16_t)(-s3);
    }
    /* 只写一次速度即可，Gui 版本写大端速度 */
    ServoBus_SetTorqueSwitch(PACK_SCS_IMPELLER_ID_A, 1u);
    ServoBus_SetTorqueSwitch(PACK_SCS_IMPELLER_ID_B, 1u);
    ServoBus_SetWheelSpeedGui(PACK_SCS_IMPELLER_ID_A, s3);
    ServoBus_SetWheelSpeedGui(PACK_SCS_IMPELLER_ID_B, (int16_t)(-s8));
}

/* 膜杆 ID2/7 是 ST3215（STServoBus 小端 int16），调用 ST3215 专用函数
 * 注意：WriteSpeedAcc 内部已包含扭矩开启，无需单独调用 SetTorqueSwitch */
static void pack_scs_rod_wheel_run_both(int16_t sp2, int16_t sp7)
{
    uint8_t acc = pack_impeller_acc_u8();
    ServoBus_STS360_WriteSpeedAcc(PACK_SCS_ROD_ID_A, sp2, acc);
    ServoBus_STS360_WriteSpeedAcc(PACK_SCS_ROD_ID_B, sp7, acc);
}

static void pack_scs_rod_wheel_zero_both(void)
{
    /* 关扭矩须在速度写入后执行 */
    ServoBus_STS360_WriteSpeedAcc(PACK_SCS_ROD_ID_A, 0, 0u);
    ServoBus_STS360_WriteSpeedAcc(PACK_SCS_ROD_ID_B, 0, 0u);
    ServoBus_SetTorqueSwitch(PACK_SCS_ROD_ID_A, 0u);
    ServoBus_SetTorqueSwitch(PACK_SCS_ROD_ID_B, 0u);
}

static void pack_scs_rod_wheel_stop_final(uint8_t torque_off_after)
{
    pack_scs_rod_wheel_zero_both();
    if (torque_off_after) {
        ServoBus_SetTorqueSwitch(PACK_SCS_ROD_ID_A, 0u);
        ServoBus_SetTorqueSwitch(PACK_SCS_ROD_ID_B, 0u);
    }
}

static uint16_t read_ultrasonic_distance(GPIO_TypeDef* trig_port, uint16_t trig_pin,
                                          GPIO_TypeDef* echo_port, uint16_t echo_pin)
{
    uint32_t t1, t2, diff;
    uint16_t distance;
    uint32_t timeout;

    GPIO_SetBits(trig_port, trig_pin);
    delay_us(10);
    GPIO_ResetBits(trig_port, trig_pin);

    t1 = t2 = 0;
    timeout = 25000;
    while (GPIO_ReadInputDataBit(echo_port, echo_pin) == Bit_RESET) {
        if (timeout-- == 0) {
            return 0xFFFF;
        }
    }
    t1 = SysTick->VAL;
    timeout = 25000;
    while (GPIO_ReadInputDataBit(echo_port, echo_pin) != Bit_RESET) {
        if (timeout-- == 0) {
            return 0xFFFF;
        }
    }
    t2 = SysTick->VAL;

    if (t1 >= t2) diff = t1 - t2;
    else diff = (SysTick->LOAD - t2) + t1;

    distance = (uint16_t)((diff * 17U) / 7200U);
    if (distance > 4000) distance = 4000;
    return distance;
}

/*-----------------------------------------------------------
 * 工艺流程初始化
 *-----------------------------------------------------------*/
void Process_Init(void)
{
    memset(&g_process_status, 0, sizeof(g_process_status));
    g_process_status.phase = PROCESS_IDLE;
    g_process_status.clamp_state = CLAMP_IDLE;
    g_process_status.clean_state = CLEAN_IDLE;
    g_process_status.wait_state = WAIT_IDLE;
    g_process_status.pack_zone_state = PACK_ZONE_IDLE;
    g_process_status.pack_mode = PACK_MODE_FULL;
    g_process_status.clean_mode = CLEAN_MODE_BOTH;

    g_process_params.target_count = 3;
    g_pack_mode = PACK_MODE_FULL;
}

/*-----------------------------------------------------------
 * 主状态机（4分区架构）
 *-----------------------------------------------------------*/
void Process_Main_SM_Run(void)
{
    uint32_t now = PROCESS_TICK();

    if (g_protocol_emergency_stop_request) {
        g_protocol_emergency_stop_request = 0;
        Process_EmergencyStop();
        return;
    }
    if (g_protocol_process_pause_request) {
        g_protocol_process_pause_request = 0;
        Process_Pause();
        return;
    }

    switch (g_process_status.phase) {

        /* ======================== 空闲 ======================== */
        case PROCESS_IDLE:
            break;

        /* ======================== 分区A：夹入切割 ======================== */
        case PROCESS_ZONE_A_CLAMP:
            Process_Clamp_SM_Run();
            if (g_process_status.clamp_state == CLAMP_DONE) {
                g_process_status.tick_start = now;
                g_process_status.phase = PROCESS_ZONE_B_CLEAN;
            }
            break;

        /* ======================== 分区B：清洗 ======================== */
        case PROCESS_ZONE_B_CLEAN:
            Process_Clean_SM_Run();
            if (g_process_status.clean_state == CLEAN_DONE) {
                g_process_status.tick_start = now;
                g_process_status.phase = PROCESS_ZONE_C_WAIT;
            }
            break;

        /* ======================== 分区C：等待 ======================== */
        case PROCESS_ZONE_C_WAIT:
            Process_Wait_SM_Run();
            if (g_process_status.wait_state == WAIT_READY) {
                /* 检查是否达标 */
                if (g_process_status.onion_count >= g_process_params.target_count) {
                    /* 达标，进入打包 */
                    g_process_status.phase = PROCESS_ZONE_D_PACK;
                    g_process_status.onion_count = 0;
                } else {
                    /* 未达标，返回分区A继续 */
                    g_process_status.phase = PROCESS_ZONE_A_CLAMP;
                    g_process_status.clamp_state = CLAMP_IDLE;
                    g_process_status.wait_state = WAIT_IDLE;
                }
                    g_process_status.tick_start = now;
                }
            break;

        /* ======================== 分区D：打包 ======================== */
        case PROCESS_ZONE_D_PACK:
            Process_Pack_Zone_SM_Run();
            if (g_process_status.pack_zone_state == PACK_ZONE_DONE) {
                g_process_status.phase = PROCESS_DONE;
                g_process_status.tick_start = now;
            }
            break;

        /* ======================== 流程完成 ======================== */
        case PROCESS_DONE:
            Process_StopAll();
            g_process_status.phase = PROCESS_IDLE;
            break;
    }
}

/*-----------------------------------------------------------
 * 分区A：夹爪子状态机
 *
 * 数量模式流程（数量达标后打包）：
 *   CLAMP_IDLE → CLAMP_GRIP → CLAMP_US_SENSE → CLAMP_CUT → CLAMP_DONE → Zone B
 *
 * CLAMP_GRIP       : 同步带运转夹入 2000ms
 * CLAMP_US_SENSE   : 同步带运转 + 超声检测大葱，计数达到 target_count 后进入切割
 * CLAMP_CUT        : 切割电机运转 2000ms
 * CLAMP_DONE       : 通知主状态机进入 Zone B 清洗
 *-----------------------------------------------------------*/
void Process_Clamp_SM_Run(void)
{
    uint32_t now = PROCESS_TICK();

    if (g_process_status.phase == PROCESS_PAUSED) return;

    if (g_protocol_emergency_stop_request) {
        g_protocol_emergency_stop_request = 0;
        Process_EmergencyStop();
        return;
    }
    if (g_protocol_process_pause_request) {
        g_protocol_process_pause_request = 0;
        Process_Pause();
        return;
    }

    switch (g_process_status.clamp_state) {

        /*------------------------------------------------------------
         * CLAMP_IDLE : 每次进入前重置大葱计数，通知主循环启动
         *------------------------------------------------------------*/
        case CLAMP_IDLE:
            g_process_status.onion_count = 0;
            g_process_status.onion_count_flag = 0;
            g_process_status.tick_start = now;
            g_process_status.clamp_state = CLAMP_GRIP;
            break;

        /*------------------------------------------------------------
         * CLAMP_GRIP : 同步带运转夹入，等待夹入时间后进入超声检测
         *------------------------------------------------------------*/
        case CLAMP_GRIP:
            Stepper_SyncRun_Motor1Reversed(g_process_params.clamp_motor_speed_hz, STEPPER_DIR_CW);
            if (PROCESS_ELAPSED(g_process_params.clamp_grip_time_ms)) {
                Stepper_SyncStop();
                g_process_status.tick_start = now;
                g_process_status.clamp_state = CLAMP_US_SENSE;
            }
            break;

        /*------------------------------------------------------------
         * CLAMP_US_SENSE : 同步带运转 + 超声检测大葱
         *   - 每 60ms 读取超声距离
         *   - dist < threshold → 停止同步带 → 大葱计数 +1 → 判断是否达标
         *   - 达标（count >= target_count）→ 进入 CLAMP_CUT
         *   - 未达标 → 重启同步带继续运送下一个大葱
         *------------------------------------------------------------*/
        case CLAMP_US_SENSE: {
            static uint32_t s_us_poll_tick = 0;
                uint16_t dist;

            /* 每 60ms 读取一次超声 */
            if ((uint32_t)(now - s_us_poll_tick) >= 60U) {
                s_us_poll_tick = now;
                dist = Process_ReadClampDistance();
                g_process_sensors.clamp_distance_mm = dist;

                /* 检测到大葱（距离在阈值内，且未在本次已计数） */
                if (dist != 0xFFFF && dist < g_process_params.clamp_us_threshold_mm
                    && !g_process_status.onion_count_flag) {

                    /* 停止同步带，计数 +1 */
                    Stepper_SyncStop();
                    g_process_status.onion_count++;

                    /* 数量达标 → 进入切割 */
                    if (g_process_status.onion_count >= g_process_params.target_count) {
                        g_process_status.tick_start = now;
                        g_process_status.clamp_state = CLAMP_CUT;
                        s_us_poll_tick = 0;
                        break;
                    }

                    /* 未达标 → 重启同步带，继续运送下一个大葱 */
                    Stepper_SyncRun_Motor1Reversed(g_process_params.clamp_motor_speed_hz, STEPPER_DIR_CW);
                }
            }
            break;
        }

        /*------------------------------------------------------------
         * CLAMP_CUT : 切割电机运转，等待切割时间后进入完成状态
         *------------------------------------------------------------*/
        case CLAMP_CUT: {
            static uint8_t s_cutter_started = 0;

            if (!s_cutter_started) {
                s_cutter_started = 1;
                /* TODO: 启动切割电机 */
                (void)g_process_params.cutting_time_ms;
            }

            if (PROCESS_ELAPSED(g_process_params.cutting_time_ms)) {
                /* TODO: 停止切割电机 */
                g_process_status.onion_count_flag = 1;
                s_cutter_started = 0;
                g_process_status.tick_start = now;
                g_process_status.clamp_state = CLAMP_DONE;
            }
            break;
        }

        case CLAMP_DONE:
            break;

        default:
            break;
    }
}

/*-----------------------------------------------------------
 * 分区B：清洗子状态机
 *-----------------------------------------------------------*/
void Process_Clean_SM_Run(void)
{
    uint32_t now = PROCESS_TICK();

    if (g_process_status.phase == PROCESS_PAUSED) return;

    if (g_process_status.clean_state == CLEAN_IDLE) {
        g_process_status.tick_start = now;
        g_process_status.clean_state = CLEAN_ENTER;
    }

    switch (g_process_status.clean_state) {

        case CLEAN_ENTER:
            g_process_status.tick_start = now;
            if (g_clean_mode == CLEAN_MODE_WATER_ONLY) {
                g_process_status.clean_state = CLEAN_WATER_ON;
            } else {
                g_process_status.clean_state = CLEAN_SPRAY_ON;
            }
            break;

        case CLEAN_SPRAY_ON:
            AIR_VALVE_ON();
            Device.Servo1.SetAngle(45);
            if (PROCESS_ELAPSED(g_process_params.clean_air_time_ms)) {
                AIR_VALVE_OFF();
                Device.Servo1.SetAngle(g_process_params.pack_servo_angle_start);
                g_process_status.tick_start = now;
                if (g_clean_mode == CLEAN_MODE_AIR_ONLY) {
                    g_process_status.clean_state = CLEAN_DONE;
                } else {
                    g_process_status.clean_state = CLEAN_WATER_ON;
                }
            }
            break;

        case CLEAN_WATER_ON:
            WATER_PUMP_ON();
            if (PROCESS_ELAPSED(g_process_params.clean_water_time_ms)) {
                WATER_PUMP_OFF();
                g_process_status.tick_start = now;
                g_process_status.clean_state = CLEAN_DONE;
            }
            break;

        case CLEAN_DONE:
            AIR_VALVE_OFF();
            WATER_PUMP_OFF();
            Device.Servo1.SetAngle(g_process_params.pack_servo_angle_start);
            break;

        default:
            break;
    }
}

/*-----------------------------------------------------------
 * 分区C：等待子状态机
 *-----------------------------------------------------------*/
void Process_Wait_SM_Run(void)
{
    uint32_t now = PROCESS_TICK();

    if (g_process_status.phase == PROCESS_PAUSED) return;

    if (g_process_status.wait_state == WAIT_IDLE) {
        g_process_status.tick_start = now;
        g_process_status.wait_state = WAIT_DELAY;
        return;
    }

    switch (g_process_status.wait_state) {

        case WAIT_DELAY:
            /* 定时等待大葱到位 */
            if (PROCESS_ELAPSED(g_process_params.wait_delay_ms)) {
                g_process_status.tick_start = now;
                g_process_status.wait_state = WAIT_READY;
            }
            break;

        case WAIT_READY:
            /* 等待完成，由主状态机判断是否达标 */
            break;

        default:
            break;
    }
}

/*-----------------------------------------------------------
 * 分区D：打包子状态机（红外+超声双重检测）
 *-----------------------------------------------------------*/
void Process_Pack_Zone_SM_Run(void)
{
    PackZoneState_TypeDef pack_st0 = g_process_status.pack_zone_state;
    uint32_t          now      = PROCESS_TICK();
    uint16_t          dist_raw;
    uint16_t          dist;
    uint8_t           ir_detected;

    if (g_process_status.phase == PROCESS_PAUSED) return;

    dist_raw = Process_ReadPackDistance();
    dist     = dist_raw;
    ir_detected = PACK_IR_TRIGGERED();

    g_process_sensors.pack_distance_mm = dist;
    g_process_sensors.infrared_detected = ir_detected;

    switch (g_process_status.pack_zone_state) {

        case PACK_ZONE_IDLE:
            g_process_status.pack_head_seen = 0;
            g_process_status.pack_tail_seen = 0;
            g_process_status.pack_wrap_done = 0;
            g_process_status.pack_cut_done = 0;
            g_process_status.pack_ir_detected = 0;
            g_process_status.pack_rod_leg = 0u;
            g_process_status.pack_wrap_count = 0;
            ServoBus_SetTorqueSwitch(PACK_SCS_IMPELLER_ID_A, 1u);
            ServoBus_SetTorqueSwitch(PACK_SCS_IMPELLER_ID_B, 1u);
            pack_scs_impeller_stop();
            DCM4_Stop();
            Stepper_Stop(STEPPER_CONVEY);
            g_process_status.tick_start = now;
            g_process_status.pack_wrap_start = now;
            g_process_status.pack_zone_state = PACK_ZONE_ALIGN_REV;
            break;

        case PACK_ZONE_ALIGN_REV:
            /* M3反转2s对齐 */
            Stepper_RunAtSpeed(STEPPER_CONVEY, g_process_params.conveyor_speed_hz, STEPPER_DIR_CCW);
            if (PROCESS_ELAPSED(g_process_params.pack_align_reverse_ms)) {
                Stepper_Stop(STEPPER_CONVEY);
                g_process_status.tick_start = now;
                g_process_status.pack_zone_state = PACK_ZONE_FEED_FWD;
            }
            break;

        case PACK_ZONE_FEED_FWD:
            /* 叶轮先转，M3等3s再启动送葱 */
            pack_scs_impeller_clamp_run();
            if (PROCESS_ELAPSED(3000u)) {
                Stepper_RunAtSpeed(STEPPER_CONVEY, g_process_params.conveyor_speed_hz, STEPPER_DIR_CW);
                g_process_status.pack_wrap_start = now;
                g_process_status.pack_zone_state = PACK_ZONE_WAIT_PACK;
            }
            break;

        case PACK_ZONE_WAIT_PACK:
            /* 等超声波检测大葱到达 */
            Stepper_RunAtSpeed(STEPPER_CONVEY, g_process_params.conveyor_speed_hz, STEPPER_DIR_CW);
            pack_scs_impeller_clamp_run();

            if (g_pack_mode == PACK_MODE_FULL) {
                /* 超声波检测葱头 */
                if (dist < g_process_params.pack_dist_head && dist != 0xFFFF) {
                g_process_status.pack_head_seen = 1;
                    g_process_status.tick_start = now;
                    g_process_status.pack_zone_state = PACK_ZONE_WRAP_START;
                }
            } else if (g_pack_mode == PACK_MODE_MIDDLE) {
                /* 红外检测葱头 */
                if (ir_detected) {
                g_process_status.pack_head_seen = 1;
                    g_process_status.pack_ir_detected = 1;
                    g_process_status.tick_start = now;
                    g_process_status.pack_zone_state = PACK_ZONE_WRAP_START;
                }
            }

            if (PROCESS_ELAPSED(g_process_params.pack_timeout_ms)) {
                g_process_status.pack_zone_state = PACK_ZONE_DONE;
            }
            break;

        case PACK_ZONE_WRAP_START:
            /* 缠膜两圈后，套杆抽离(舵机2/7半圈) */
            Stepper_RunAtSpeed(STEPPER_CONVEY, g_process_params.pack_speed_hz, STEPPER_DIR_CW);
            pack_scs_impeller_clamp_run();

            /* 两圈定时 */
            if (PROCESS_ELAPSED((uint32_t)(2u * g_process_params.pack_m3_ms_per_rev))) {
                Stepper_Stop(STEPPER_CONVEY);
                pack_scs_impeller_stop();

                /* 抽杆：轮模式两节定时 */
                {
                    int16_t s2, s7;
                    pack_rod_pull_speeds(&s2, &s7);
                    g_process_status.pack_rod_leg = 1u;
                    g_process_status.pack_wrap_start = now;
                    pack_scs_rod_wheel_run_both(s2, s7);
                }
                g_process_status.pack_zone_state = PACK_ZONE_ROD_PULL;
            }
            break;

        case PACK_ZONE_ROD_PULL:
            /* 超声确认离开：套杆送入，然后缠膜一圈同时切膜电机正2s反2s */
            {
                uint32_t leg_ms = pack_rod_wheel_leg_ms();

                if (g_process_status.pack_rod_leg == 1u) {
                    if (PACK_ELAPSED(leg_ms, g_process_status.pack_wrap_start)) {
                        pack_scs_rod_wheel_zero_both();
                        {
                            int16_t s2, s7;
                            pack_rod_pull_speeds(&s2, &s7);
                            pack_scs_rod_wheel_run_both(s2, s7);
                        }
                        g_process_status.pack_rod_leg = 2u;
                        g_process_status.pack_wrap_start = now;
                    }
                } else if (g_process_status.pack_rod_leg == 2u) {
                    if (PACK_ELAPSED(leg_ms, g_process_status.pack_wrap_start)) {
                        pack_scs_rod_wheel_stop_final(0u);
                        g_process_status.pack_rod_leg = 0u;
                        pack_scs_impeller_clamp_run();
                        /* 缠膜一圈同时切膜电机正2s反2s */
                        Stepper_RunAtSpeed(STEPPER_CONVEY, g_process_params.pack_speed_hz, STEPPER_DIR_CW);
                        DCM4_Run(50, DIR_FORWARD);
                        g_process_status.pack_wrap_start = now;
                        g_process_status.pack_zone_state = PACK_ZONE_WRAP_TAIL;
                    }
                }
            }
            break;

        case PACK_ZONE_WRAP_TAIL:
            /* 等超声检测葱尾+红外确认离开 */
            pack_scs_impeller_clamp_run();

            if (dist > g_process_params.pack_dist_tail && dist != 0xFFFF) {
                /* 超声确认葱尾离开 */
                if (!ir_detected) {
                    /* 红外也确认离开 */
                    DCM4_Stop();
                    Stepper_Stop(STEPPER_CONVEY);
                pack_scs_impeller_stop();
                    g_process_status.pack_tail_seen = 1;
                    g_process_status.tick_start = now;
                    g_process_status.pack_zone_state = PACK_ZONE_WRAP_COMPLETE;
                }
            }

            /* HEAD_TAIL模式：缠膜两圈后切断，再缠两圈 */
            if (g_pack_mode == PACK_MODE_HEAD_TAIL && !g_process_status.pack_cut_done) {
                if (PROCESS_ELAPSED((uint32_t)(2u * g_process_params.pack_m3_ms_per_rev))) {
                    /* 切断 */
                    DCM4_Run(50, DIR_FORWARD);
            g_process_status.tick_start = now;
                    g_process_status.pack_cut_done = 1;
                }
            }
            if (g_pack_mode == PACK_MODE_HEAD_TAIL && g_process_status.pack_cut_done) {
                if (PROCESS_ELAPSED(g_process_params.pack_film_cw_ms)) {
                    DCM4_Run(50, DIR_BACKWARD);
                }
                if (PROCESS_ELAPSED(g_process_params.pack_film_cw_ms + g_process_params.pack_film_ccw_ms)) {
                    DCM4_Stop();
                }
            }

            if (PROCESS_ELAPSED(g_process_params.pack_timeout_ms)) {
                DCM4_Stop();
                Stepper_Stop(STEPPER_CONVEY);
                pack_scs_impeller_stop();
                g_process_status.pack_zone_state = PACK_ZONE_DONE;
            }
            break;

        case PACK_ZONE_WRAP_COMPLETE:
            /* 打包完成 */
            g_process_status.pack_wrap_done = 1;
            g_process_status.tick_start = now;
            g_process_status.pack_zone_state = PACK_ZONE_EJECT;
            break;

        case PACK_ZONE_EJECT:
            /* M3+叶轮运行4s抛出 */
            pack_scs_impeller_eject_run();
            if (PROCESS_ELAPSED(g_process_params.pack_eject_run_ms)) {
                pack_scs_impeller_stop();
                g_process_status.pack_zone_state = PACK_ZONE_DONE;
            }
            break;

        case PACK_ZONE_DONE:
            Stepper_Stop(STEPPER_CONVEY);
            DCM4_Stop();
            pack_scs_impeller_stop();
            ServoBus_SetTorqueSwitch(PACK_SCS_ROD_ID_A, 0u);
            ServoBus_SetTorqueSwitch(PACK_SCS_ROD_ID_B, 0u);
            AIR_VALVE_OFF();
            WATER_PUMP_OFF();
            break;

        default:
            g_process_status.pack_zone_state = PACK_ZONE_DONE;
            break;
    }

#if PROCESS_PACK_DBG_UART
    if (g_process_status.pack_zone_state != pack_st0) {
        UART1_Printf("[PACK] %u->%u d=%u ir=%d\r\n",
            (unsigned)pack_st0, (unsigned)g_process_status.pack_zone_state,
            (unsigned)dist, ir_detected);
    }
#endif
}

/*-----------------------------------------------------------
 * 测距驱动
 *-----------------------------------------------------------*/
uint16_t Process_ReadPackDistance(void)
{
    return read_ultrasonic_distance(PACK_SENSOR_TRIG_PORT, PACK_SENSOR_TRIG_PIN,
                                     PACK_SENSOR_ECHO_PORT, PACK_SENSOR_ECHO_PIN);
}

uint16_t Process_ReadClampDistance(void)
{
    return read_ultrasonic_distance(CLAMP_US_TRIG_PORT, CLAMP_US_TRIG_PIN,
                                     CLAMP_US_ECHO_PORT, CLAMP_US_ECHO_PIN);
}

/*-----------------------------------------------------------
 * 膜杆总线IDLE
 *-----------------------------------------------------------*/
void Process_PackRodBus_Idle(void)
{
    pack_scs_rod_wheel_stop_final(1u);
}

/*-----------------------------------------------------------
 * 停止所有设备
 *-----------------------------------------------------------*/
void Process_StopAll(void)
{
    Stepper_SyncStop();
    Stepper_ScrewStop();
    Stepper_Stop(STEPPER_CONVEY);
    Stepper_Stop(STEPPER_PACK);
    DCM1_Stop();
    DCM2_Stop();
    DCM3_Stop();
    DCM4_Stop();
    AIR_VALVE_OFF();
    WATER_PUMP_OFF();
    PWR_CTRL_OFF();
    pack_scs_impeller_stop();
    pack_scs_rod_wheel_stop_final(1u);
    Device.Servo1.SetAngle(g_process_params.pack_servo_angle_start);
    Device.Servo2.SetAngle(g_process_params.pack_servo_angle_start);
}

/*-----------------------------------------------------------
 * 紧急停止
 * 功能：停止所有电机，保持当前状态不变（支持恢复至急停前状态）
 *-----------------------------------------------------------*/
void Process_EmergencyStop(void)
{
    Process_StopAll();

    /* 保存急停前的状态快照（供后续恢复用） */
    g_process_status.phase_emstop_backup       = g_process_status.phase;
    g_process_status.clamp_state_emstop_backup   = g_process_status.clamp_state;
    g_process_status.clean_state_emstop_backup   = g_process_status.clean_state;
    g_process_status.wait_state_emstop_backup    = g_process_status.wait_state;
    g_process_status.pack_zone_state_emstop_backup = g_process_status.pack_zone_state;
    g_process_status.tick_start_emstop_backup    = g_process_status.tick_start;
    g_process_status.emstop_backup_valid         = 1;

    g_process_status.phase = PROCESS_IDLE;
    Process_SetProcState_Clear();
}

/*-----------------------------------------------------------
 * 流程启动/停止/暂停/恢复
 *-----------------------------------------------------------*/
void Process_Start(void)
{
    if (g_process_status.phase == PROCESS_IDLE) {
        /* 优先恢复急停前的状态（若存在），否则从头开始 */
        if (g_process_status.emstop_backup_valid) {
            g_process_status.phase                     = g_process_status.phase_emstop_backup;
            g_process_status.clamp_state                = g_process_status.clamp_state_emstop_backup;
            g_process_status.clean_state                = g_process_status.clean_state_emstop_backup;
            g_process_status.wait_state                 = g_process_status.wait_state_emstop_backup;
            g_process_status.pack_zone_state            = g_process_status.pack_zone_state_emstop_backup;
            g_process_status.tick_start                 = g_process_status.tick_start_emstop_backup;
            g_process_status.emstop_backup_valid         = 0;
        } else {
            g_process_status.phase = PROCESS_ZONE_A_CLAMP;
            g_process_status.tick_start = PROCESS_TICK();
        }
    }
}

void Process_Stop(void)
{
    Process_StopAll();
    g_process_status.phase = PROCESS_IDLE;
    Process_SetProcState_Clear();
}

void Process_Pause(void)
{
    if (g_process_status.phase != PROCESS_IDLE &&
        g_process_status.phase != PROCESS_PAUSED) {
        g_process_status.phase_saved            = g_process_status.phase;
        g_process_status.clamp_state_saved       = g_process_status.clamp_state;
        g_process_status.clean_state_saved       = g_process_status.clean_state;
        g_process_status.wait_state_saved        = g_process_status.wait_state;
        g_process_status.pack_zone_state_saved  = g_process_status.pack_zone_state;
        g_process_status.tick_start_saved        = g_process_status.tick_start;
        g_process_status.phase                   = PROCESS_PAUSED;
        Process_StopAll();
    }
}

void Process_Resume(void)
{
    if (g_process_status.phase == PROCESS_PAUSED) {
        g_process_status.phase                   = g_process_status.phase_saved;
        g_process_status.clamp_state              = g_process_status.clamp_state_saved;
        g_process_status.clean_state              = g_process_status.clean_state_saved;
        g_process_status.wait_state               = g_process_status.wait_state_saved;
        g_process_status.pack_zone_state           = g_process_status.pack_zone_state_saved;
        g_process_status.tick_start               = g_process_status.tick_start_saved;
    }
}

void Process_EmergencyStop_Resume(void)
{
    if (g_process_status.emstop_backup_valid) {
        g_process_status.phase                     = g_process_status.phase_emstop_backup;
        g_process_status.clamp_state                = g_process_status.clamp_state_emstop_backup;
        g_process_status.clean_state                = g_process_status.clean_state_emstop_backup;
        g_process_status.wait_state                 = g_process_status.wait_state_emstop_backup;
        g_process_status.pack_zone_state            = g_process_status.pack_zone_state_emstop_backup;
        g_process_status.tick_start                 = g_process_status.tick_start_emstop_backup;
        g_process_status.emstop_backup_valid         = 0;

        /* 同步 g_proc_state（供 main.c 的 TJC 停止条件判断） */
        Process_SetProcState_Clear();
    }
}

/*-----------------------------------------------------------
 * 传感器更新
 *-----------------------------------------------------------*/
void Process_UpdateSensors(ProcessSensors_TypeDef* sensors)
{
    if (sensors != NULL) {
        memcpy(&g_process_sensors, sensors, sizeof(ProcessSensors_TypeDef));
    }
}

/*-----------------------------------------------------------
 * 状态获取接口
 *-----------------------------------------------------------*/
ProcessPhase_TypeDef Process_GetPhase(void)
{
    return g_process_status.phase;
}

ClampState_TypeDef Process_GetClampState(void)
{
    return g_process_status.clamp_state;
}

CleanState_TypeDef Process_GetCleanState(void)
{
    return g_process_status.clean_state;
}

WaitState_TypeDef Process_GetWaitState(void)
{
    return g_process_status.wait_state;
}

PackZoneState_TypeDef Process_GetPackZoneState(void)
{
    return g_process_status.pack_zone_state;
}

CleanMode_TypeDef Process_GetCleanMode(void)
{
    return g_clean_mode;
}

PackTriggerMode_TypeDef Process_GetPackMode(void)
{
    return g_pack_mode;
}

/*-----------------------------------------------------------
 * 参数读写接口
 *-----------------------------------------------------------*/
uint16_t Process_GetParam(uint8_t param_id)
{
    switch (param_id) {
        case 0x01: return g_process_params.belt_speed_hz;
        case 0x02: return g_process_params.belt_feed_delay_ms;
        case 0x03: return g_process_params.clamp_motor_speed_hz;
        case 0x04: return g_process_params.clamp_grip_time_ms;
        case 0x05: return g_process_params.clamp_us_threshold_mm;
        case 0x06: return g_process_params.clamp_us_timeout_ms;
        case 0x07: return g_process_params.screw_speed_hz;
        case 0x08: return g_process_params.screw_limit_down_ms;
        case 0x09: return g_process_params.screw_limit_up_ms;
        case 0x0A: return g_process_params.cutter_speed_duty;
        case 0x0B: return g_process_params.cutting_time_ms;
        case 0x0C: return g_process_params.clean_air_time_ms;
        case 0x0D: return g_process_params.clean_water_time_ms;
        case 0x0E: return g_process_params.wait_delay_ms;
        case 0x0F: return g_process_params.wait_us_trigger_mm;
        case 0x10: return g_process_params.conveyor_speed_hz;
        case 0x11: return g_process_params.pack_speed_hz;
        case 0x12: return g_process_params.pack_dist_head;
        case 0x13: return g_process_params.pack_dist_tail;
        case 0x14: return g_process_params.pack_timeout_ms;
        case 0x15: return g_process_params.pack_align_reverse_ms;
        case 0x16: return g_process_params.pack_m3_ms_per_rev;
        case 0x17: return g_process_params.pack_impeller_wheel_spd;
        case 0x18: return g_process_params.pack_impeller_acc;
        case 0x19: return g_process_params.pack_impeller_eject_ms;
        case 0x1A: return g_process_params.pack_film_cw_ms;
        case 0x1B: return g_process_params.pack_film_ccw_ms;
        case 0x1C: return g_process_params.pack_eject_run_ms;
        case 0x1D: return g_process_params.target_count;
        case 0xF1: return (uint16_t)g_clean_mode;
        case 0xF2: return (uint16_t)g_pack_mode;
        default:   return 0;
    }
}

void Process_SetParam(uint8_t param_id, uint16_t value)
{
    switch (param_id) {
        case 0x01: g_process_params.belt_speed_hz = value; break;
        case 0x02: g_process_params.belt_feed_delay_ms = value; break;
        case 0x03: g_process_params.clamp_motor_speed_hz = value; break;
        case 0x04: g_process_params.clamp_grip_time_ms = value; break;
        case 0x05: g_process_params.clamp_us_threshold_mm = value; break;
        case 0x06: g_process_params.clamp_us_timeout_ms = value; break;
        case 0x07: g_process_params.screw_speed_hz = value; break;
        case 0x08: g_process_params.screw_limit_down_ms = value; break;
        case 0x09: g_process_params.screw_limit_up_ms = value; break;
        case 0x0A: g_process_params.cutter_speed_duty = value; break;
        case 0x0B: g_process_params.cutting_time_ms = value; break;
        case 0x0C: g_process_params.clean_air_time_ms = value; break;
        case 0x0D: g_process_params.clean_water_time_ms = value; break;
        case 0x0E: g_process_params.wait_delay_ms = value; break;
        case 0x0F: g_process_params.wait_us_trigger_mm = value; break;
        case 0x10: g_process_params.conveyor_speed_hz = value; break;
        case 0x11: g_process_params.pack_speed_hz = value; break;
        case 0x12: g_process_params.pack_dist_head = value; break;
        case 0x13: g_process_params.pack_dist_tail = value; break;
        case 0x14: g_process_params.pack_timeout_ms = value; break;
        case 0x15: g_process_params.pack_align_reverse_ms = value; break;
        case 0x16: g_process_params.pack_m3_ms_per_rev = value; break;
        case 0x17: g_process_params.pack_impeller_wheel_spd = value; break;
        case 0x18: g_process_params.pack_impeller_acc = value; break;
        case 0x19: g_process_params.pack_impeller_eject_ms = value; break;
        case 0x1A: g_process_params.pack_film_cw_ms = value; break;
        case 0x1B: g_process_params.pack_film_ccw_ms = value; break;
        case 0x1C: g_process_params.pack_eject_run_ms = value; break;
        case 0x1D:
            if (value < 1u) value = 1u;
            if (value > 7u) value = 7u;
            g_process_params.target_count = value;
            break;
        case 0xF1: g_clean_mode = (CleanMode_TypeDef)value; break;
        case 0xF2: g_pack_mode = (PackTriggerMode_TypeDef)value; break;
        default: break;
    }
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
