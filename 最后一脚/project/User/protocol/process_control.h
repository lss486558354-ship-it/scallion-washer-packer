/**
 ******************************************************************************
 * 工艺流程控制头文件
 *
 * 功能：
 *   - 大葱切割-清洗-打包一体机4分区状态机
 *   - 分区A：夹入切割（超声检测+丝杆调整）
 *   - 分区B：清洗吹气
 *   - 分区C：等待到位（计数判断）
 *   - 分区D：打包（红外+超声双重检测）
 *   - Device对象化统一调度接口
 *
 * 版本：V3.0
 * 日期：2026-04-18
 ******************************************************************************
 */

#ifndef __PROCESS_CONTROL_H__
#define __PROCESS_CONTROL_H__

#include "stm32f10x.h"
#include "main.h"
#include "bsp_relay.h"

/* VET6版本步进测试模式：已启用完整功能 */

/*-----------------------------------------------------------
 * 功能开关（0=启用测距 1=跳过测距，使用固定丝杆位置）
 *-----------------------------------------------------------*/
#define PROCESS_SKIP_HEIGHT_MEASURE  0

/*
 * 1：上电 Process_Init 后直接进入 PROCESS_PACKAGING（从 PACK_ENTER 起跑打包子状态机），
 *    便于台架调试；完整产线需先发协议启动或走 PC1 主流程时务必改为 0。
 * 原因：正常时 phase 一直保持 PROCESS_IDLE，只有 Process_Start() 等才会进入后续阶段，
 *    故默认不会自动进打包。
 */
#ifndef PROCESS_TEST_BOOT_DIRECT_PACK
#define PROCESS_TEST_BOOT_DIRECT_PACK  1
#endif

/*
 * 1：打包 HC-SR04 未接时用定时模拟「葱进入 / 葱离开」（见 pack_us_sim_enter/leave_ms）
 * 0：按 dist 与 pack_dist_head/tail 判断
 */
#ifndef PROCESS_PACK_USE_FAKE_ULTRASONIC
#define PROCESS_PACK_USE_FAKE_ULTRASONIC  1
#endif

/* 1：打包状态机 UART1 打印 [PACK] 行（状态/测距/舵机命令）；量产可改 0 */
#ifndef PROCESS_PACK_DBG_UART
#define PROCESS_PACK_DBG_UART  0
#endif

/*-----------------------------------------------------------
 * GPIO引脚定义（实际接线后请对应修改此处）
 *
 * 注意：以下为预分配引脚
 *-----------------------------------------------------------*/

/* ============================================================================
 * GPIO继电器操作 — 由 bsp_relay.c 的 Relay_InitAll() 统一初始化
 * ============================================================================ */

/* 喷气继电器 K_AIR → PB12 */
#define AIR_VALVE_ON()     Relay_On(RELAY_K_AIR)
#define AIR_VALVE_OFF()    Relay_Off(RELAY_K_AIR)

/* 水泵继电器 K_PUMP → PB13 */
#define WATER_PUMP_ON()     Relay_On(RELAY_K_PUMP)
#define WATER_PUMP_OFF()   Relay_Off(RELAY_K_PUMP)

/* 电源控制继电器 K_S → PA15（Relay_InitAll() 已关闭 JTAG） */
#define PWR_CTRL_ON()      Relay_On(RELAY_K_S)
#define PWR_CTRL_OFF()     Relay_Off(RELAY_K_S)

/* 同步带到位光电开关（PE7）*/
#define BELT_SENSOR_PORT      GPIOE
#define BELT_SENSOR_PIN       GPIO_Pin_7
#define BELT_SENSOR_TRIGGERED (GPIO_ReadInputDataBit(BELT_SENSOR_PORT, BELT_SENSOR_PIN) == Bit_RESET)

/* 打包测距传感器 HC-SR04（与现场接线一致：PB0=Trig，PD3=Echo） */
#define PACK_SENSOR_TRIG_PORT  GPIOB
#define PACK_SENSOR_TRIG_PIN  GPIO_Pin_0
#define PACK_SENSOR_ECHO_PORT GPIOD
#define PACK_SENSOR_ECHO_PIN  GPIO_Pin_3
#define PACK_SENSOR_TRIG_0()  GPIO_ResetBits(PACK_SENSOR_TRIG_PORT, PACK_SENSOR_TRIG_PIN)
#define PACK_SENSOR_TRIG_1()  GPIO_SetBits(PACK_SENSOR_TRIG_PORT, PACK_SENSOR_TRIG_PIN)
#define PACK_SENSOR_ECHO_READ GPIO_ReadInputDataBit(PACK_SENSOR_ECHO_PORT, PACK_SENSOR_ECHO_PIN)

/*-----------------------------------------------------------
 * 打包测距阈值定义（mm，可通过串口命令修改）
 * 原理：
 *   空闲时传感器对着传送带/固定背景，距离固定（较大）
 *   葱头到达，距离缩短（变小）
 *   葱身通过，距离持续较短
 *   葱尾离开，距离恢复（变大）
 *-----------------------------------------------------------*/
#define PACK_DIST_THRESH_HEAD   55   /* 葱头阈值：距离<此值认为葱头到达 */
#define PACK_DIST_THRESH_TAIL   65   /* 葱尾阈值：距离>此值认为葱尾离开 */
#define PACK_MEASURE_TIMEOUT_MS 30000 /* 打包测距总超时 ms */

/*-----------------------------------------------------------
 * 主流程状态枚举（已在main.h定义，此处引用）
 *-----------------------------------------------------------*/

/*-----------------------------------------------------------
 * 分区A 夹爪子状态枚举（已在main.h定义，此处引用）
 *-----------------------------------------------------------*/

/*-----------------------------------------------------------
 * 分区B 清洗子状态枚举（已在main.h定义，此处引用）
 *-----------------------------------------------------------*/

/*-----------------------------------------------------------
 * 分区C 等待子状态枚举（已在main.h定义，此处引用）
 *-----------------------------------------------------------*/

/*-----------------------------------------------------------
 * 分区D 打包子状态枚举（已在main.h定义，此处引用）
 *-----------------------------------------------------------*/

/*-----------------------------------------------------------
 * 打包触发模式枚举（已在main.h定义，此处引用）
 *-----------------------------------------------------------*/

/*-----------------------------------------------------------
 * 丝杆运动方向枚举（已在main.h定义，此处引用）
 *-----------------------------------------------------------*/

/*-----------------------------------------------------------
 * 工艺参数结构体（可通过串口命令修改）
 *-----------------------------------------------------------*/
typedef struct {
    /* 同步带参数 */
    uint16_t belt_speed_hz;        /* 同步带步进电机速度 Hz */
    uint16_t belt_feed_delay_ms;   /* 进料后稳定延时 ms */

    /* 分区A夹爪参数 */
    uint16_t clamp_motor_speed_hz;     /* 夹爪电机速度 */
    uint16_t clamp_grip_time_ms;       /* 夹入持续时间 */
    uint16_t clamp_us_threshold_mm;     /* 超声波检测阈值 */
    uint16_t clamp_us_timeout_ms;       /* 超声波检测超时 */

    /* 丝杆参数 */
    uint16_t screw_speed_hz;        /* 丝杆步进电机速度 Hz */
    uint16_t screw_down_delay_ms;     /* 丝杆下降稳定延时 ms */
    uint16_t screw_limit_down_ms;   /* 正转下降最大持续时间 ms */
    uint16_t screw_limit_up_ms;     /* 反转上升最大持续时间 ms */
    uint16_t screw_target_fixed;     /* 固定丝杆目标位置（步）*/

    /* 切割参数 */
    uint16_t cutter_speed_duty;    /* 切割电机 PWM 占空比 0~100 */
    uint16_t cutting_time_ms;      /* 切割持续时间 ms */

    /* 清洗参数 */
    uint16_t clean_air_time_ms;    /* 喷气清洗持续时间 ms */
    uint16_t clean_water_time_ms;  /* 水泵清洗持续时间 ms */
    uint16_t clean_wait_enter_ms;   /* 进入清洗前等待时间 */
    uint16_t spray_delay_ms;       /* 喷嘴启动前延时 ms */
    uint16_t water_delay_ms;       /* 水泵启动前延时 ms */

    /* 分区C等待参数 */
    uint16_t wait_delay_ms;             /* 等待时间 */
    uint16_t wait_us_trigger_mm;        /* 超声波触发距离阈值 */

    /* 传送带参数 */
    uint16_t conveyor_speed_hz;    /* 传送带步进电机速度 Hz */

    /* 打包参数 */
    uint16_t pack_speed_hz;        /* 打包卷膜电机速度 Hz */
    uint16_t pack_wrap_time_ms;    /* 每次卷膜持续时间 ms */
    uint16_t pack_middle_delay_ms; /* 中间模式：葱头后延时 ms 再卷 */
    uint16_t pack_head_interval_ms;/* 头尾模式：葱尾后间隔 ms 再卷 */
    uint16_t pack_cut_delay_ms;    /* 切断后等待时间 ms */
    uint16_t pack_servo_angle_start; /* 舵机复位角度（0~180）*/
    uint16_t pack_servo_angle_cut;   /* 舵机切断角度（0~180）*/
    uint16_t pack_dist_head;       /* 葱头阈值 mm */
    uint16_t pack_dist_tail;       /* 葱尾阈值 mm */
    uint16_t pack_timeout_ms;      /* 打包总超时 ms */

    /* 打包电机参数 */
    uint16_t pack_align_reverse_ms;   /* M3 反转对齐时间 ms */
    uint16_t pack_m3_ms_per_rev;     /* 打包线一圈 ms */
    uint16_t pack_impeller_wheel_spd; /* 叶轮夹入速度 */
    uint16_t pack_impeller_acc;      /* 叶轮加减速 */
    uint16_t pack_impeller_eject_ms; /* 送出阶段持续时间 ms */
    uint16_t pack_impeller_eject_spd_id3;  /* 抛出速度ID3 */
    uint16_t pack_impeller_eject_spd_id8;  /* 抛出速度ID8 */
    uint16_t pack_scs_rod_goal_time;  /* 膜杆每节运行时间 ms */
    uint16_t pack_scs_rod_goal_speed; /* 膜杆轮速幅值 */
    uint16_t pack_scs_rod_id2_pull;  /* 膜杆ID2抽出端 */
    uint16_t pack_scs_rod_id2_push;  /* 膜杆ID2复位端 */
    uint16_t pack_scs_rod_id7_pull;  /* 膜杆ID7抽出端 */
    uint16_t pack_scs_rod_id7_push;  /* 膜杆ID7复位端 */
    uint16_t pack_film_cw_ms;       /* 切膜电机正转时间 ms */
    uint16_t pack_film_ccw_ms;      /* 切膜电机反转时间 ms */
    uint16_t pack_eject_run_ms;      /* 抛出运行时间 ms */

    /* 数量方案参数 */
    uint16_t process_plan;            /* 0=称重 1=数量 */
    uint16_t target_count;          /* 目标根数 1~7 */
} ProcessParams_TypeDef;

/*-----------------------------------------------------------
 * 工艺流程全局状态结构体
 *-----------------------------------------------------------*/
typedef struct {
    ProcessPhase_TypeDef    phase;          /* 当前主流程阶段 */
    ProcessPhase_TypeDef    phase_saved;     /* 暂停前保留的阶段（用于恢复） */
    ClampState_TypeDef     clamp_state;     /* 分区A夹爪子状态 */
    ClampState_TypeDef     clamp_state_saved;     /* 暂停前保留的夹爪子状态 */
    CleanState_TypeDef     clean_state;     /* 分区B清洗子状态 */
    CleanState_TypeDef     clean_state_saved;     /* 暂停前保留的清洗子状态 */
    WaitState_TypeDef      wait_state;      /* 分区C等待子状态 */
    WaitState_TypeDef      wait_state_saved;      /* 暂停前保留的等待子状态 */
    PackZoneState_TypeDef  pack_zone_state; /* 分区D打包子状态 */
    PackZoneState_TypeDef  pack_zone_state_saved; /* 暂停前保留的打包子状态 */
    CleanMode_TypeDef      clean_mode;      /* 当前清洗模式 */
    PackTriggerMode_TypeDef pack_mode;      /* 当前打包触发模式 */

    uint32_t               tick_start;      /* 阶段开始时间戳 */
    uint32_t               tick_start_saved; /* 暂停前保留的阶段时间戳（用于恢复剩余时间） */

    /* 急停备份：按下急停按钮前保存所有状态，用于恢复 */
    ProcessPhase_TypeDef    phase_emstop_backup;
    ClampState_TypeDef     clamp_state_emstop_backup;
    CleanState_TypeDef     clean_state_emstop_backup;
    WaitState_TypeDef      wait_state_emstop_backup;
    PackZoneState_TypeDef  pack_zone_state_emstop_backup;
    uint32_t               tick_start_emstop_backup;
    uint8_t                emstop_backup_valid;  /* 1=已保存过急停前状态 */

    /* 丝杆运动方向状态 */
    ScrewDirection_TypeDef screw_dir;
    uint32_t               screw_tick_start;

    /* 大葱计数（分区A检测到+1，分区C进入时清零）*/
    uint8_t                onion_count;
    uint8_t                onion_count_flag; /* 计数标志位：1=已计数 */

    /* 打包状态机标志位 */
    uint8_t pack_head_seen : 1;   /* 葱头已检测到 */
    uint8_t pack_tail_seen : 1;   /* 葱尾已检测到 */
    uint8_t pack_wrap_done  : 1;  /* 本次卷膜完成 */
    uint8_t pack_cut_done   : 1;  /* 切断完成 */
    uint8_t pack_ir_detected : 1; /* 红外检测到葱 */
    uint8_t reserved        : 3;

    /* 打包圈数计数 */
    uint16_t pack_wrap_count;
    uint32_t pack_wrap_start;      /* 卷膜开始时间 */

    /* 膜杆定时 */
    uint32_t pack_rod_tick_start;
    uint8_t  pack_rod_leg;         /* 0=未分段 1=第一节 2=第二节 */
} ProcessStatus_TypeDef;

/*-----------------------------------------------------------
 * 传感器输入结构体
 *-----------------------------------------------------------*/
typedef struct {
    uint8_t  belt_arrived;        /* 同步带到位光电（0/1）*/
    uint8_t  height_valid;        /* 高度测量有效（0/1）*/
    uint16_t height_value_mm;      /* 超声波测量距离 mm */
    uint8_t  screw_up_limit;      /* 丝杆上限位（0/1）*/
    uint8_t  screw_down_limit;    /* 丝杆下限位（0/1）*/
    uint16_t weight_g;           /* 当前重量 g */
    uint16_t pack_distance_mm;    /* 打包测距 mm */
    uint16_t clamp_distance_mm;   /* 夹爪区域测距 mm */
    uint8_t  infrared_detected;   /* 红外传感器检测 */
} ProcessSensors_TypeDef;

/*-----------------------------------------------------------
 * 工艺参数默认值
 *-----------------------------------------------------------*/
#define DFLT_BELT_SPEED_HZ         750
#define DFLT_BELT_FEED_DELAY_MS     300
#define DFLT_MEASURE_DELAY_MS        200
#define DFLT_SCREW_TARGET_FIXED    2000  /* 固定丝杆位置（步）*/
#define DFLT_SCREW_SPEED_HZ          800
#define DFLT_SCREW_BOOT_DURATION_MS  11500  /* 丝杆上电自检：反转11.5s → 正转11.5s */
#define DFLT_SCREW_DOWN_DELAY_MS      500
#define DFLT_SCREW_LIMIT_DOWN_MS    10000  /* 正转最大10s，防止撞机 */
#define DFLT_SCREW_LIMIT_UP_MS      12000  /* 反转最大12s，回原点超时 */
#define DFLT_CUTTER_SPEED_DUTY       80
#define DFLT_CUTTING_TIME_MS        2000
#define DFLT_CLEAN_AIR_TIME_MS      3000
#define DFLT_CLEAN_WATER_TIME_MS    5000
#define DFLT_SPRAY_DELAY_MS          300
#define DFLT_WATER_DELAY_MS          200
#define DFLT_WEIGHT_TARGET_G         500
#define DFLT_WEIGH_TIMEOUT_MS      30000
#define DFLT_WEIGHT_STABLE_MS        600  /* 达标后至少稳定 0.6s 才进打包 */
#define DFLT_WEIGHT_HYSTERESIS_G      20 /* 允许在阈值附近 ±20g 内抖动不重置计时 */
#define DFLT_CONVEYOR_SPEED_HZ     2000   /* 传送带速度 Hz（400细分不变） */
#define DFLT_PACK_SPEED_HZ          1000   /* 打包卷膜速度 Hz（400细分不变） */
#define DFLT_PACK_WRAP_TIME_MS     2000
#define DFLT_PACK_MIDDLE_DELAY_MS   1000
#define DFLT_PACK_HEAD_INTERVAL_MS  1000
#define DFLT_PACK_CUT_DELAY_MS      500
#define DFLT_PACK_SERVO_START        0
#define DFLT_PACK_SERVO_CUT          90
#define DFLT_PACK_DIST_HEAD          55
#define DFLT_PACK_DIST_TAIL          65
#define DFLT_PACK_TIMEOUT_MS       30000
#define DFLT_PACK_ALIGN_REVERSE_MS 2000u
/* 打包电机(M4)转一圈标定 ms；与 main 分区D 自检正转段同值。改此处一处即可与自检、打包 PREROLL 对齐 */
#define M34_BOOT_FWD_MS            208u  /* M3传送带正转时长 ms（32细分，2600/12.5） */
#define DFLT_PACK_M3_MS_PER_REV    M34_BOOT_FWD_MS
#define DFLT_PACK_IMPELLER_SPD     1500u
#define DFLT_PACK_IMPELLER_ACC     40u
#define DFLT_PACK_IMPELLER_EJECT_MS 6000u
#define DFLT_PACK_IMPELLER_EJ_SPD3 6u      /* 叶轮抛出速度ID3（对应上位机GUI值） */
#define DFLT_PACK_IMPELLER_EJ_SPD8 134u     /* 叶轮抛出速度ID8（对应上位机GUI值） */
#define DFLT_PACK_ROD_ID2_PULL     512u
#define DFLT_PACK_ROD_ID2_PUSH     2560u
#define DFLT_PACK_ROD_ID7_PULL   512u
#define DFLT_PACK_ROD_ID7_PUSH   2560u
#define DFLT_PACK_ROD_GOAL_TIME   960u /* 膜杆每节 0.960s到时写 0 速（半圈×2≈1圈） */
#define DFLT_PACK_ROD_GOAL_SPEED  4095u /* 轮速幅值（与手册量纲一致时可到 4095） */
#define DFLT_PACK_ROD_SETTLE_MS   1500u
#define DFLT_PACK_US_SIM_ENTER_MS 2000u /* 无超声时代替「检测到大葱进入」*/
#define DFLT_PACK_US_SIM_LEAVE_MS 8000u  /* 无超声时代替「检测到大葱离开」，可串口改 */
#define DFLT_ENABLE_HEIGHT_SCREW    1u   /* 默认开启测高+丝杆 */
#define DFLT_PROCESS_PLAN             0u   /* 默认称重方案 */
#define DFLT_TARGET_COUNT             1u   /* 默认至少 1 根 */
#define QUANTITY_COUNT_DEBOUNCE_MS  300u  /* 两次计数最小间隔 */

/*-----------------------------------------------------------
 * 全局变量声明（由 process_control.c 定义）
 *-----------------------------------------------------------*/
extern ProcessStatus_TypeDef  g_process_status;
extern ProcessParams_TypeDef   g_process_params;
extern ProcessSensors_TypeDef g_process_sensors;
extern CleanMode_TypeDef      g_clean_mode;
extern PackTriggerMode_TypeDef g_pack_mode;

/*-----------------------------------------------------------
 * 串口命令参数ID定义（CMD_SET_PARAM 使用）
 *-----------------------------------------------------------*/
typedef enum {
    PARAM_BELT_SPEED_HZ        = 0x01,
    PARAM_BELT_FEED_DELAY_MS   = 0x02,
    PARAM_MEASURE_DELAY_MS     = 0x03,
    PARAM_SCREW_TARGET_FIXED  = 0x04,
    PARAM_SCREW_SPEED_HZ      = 0x05,
    PARAM_CUTTER_SPEED_DUTY   = 0x06,
    PARAM_CUTTING_TIME_MS     = 0x07,
    PARAM_CLEAN_AIR_TIME_MS   = 0x08,
    PARAM_CLEAN_WATER_TIME_MS = 0x09,
    PARAM_WEIGHT_TARGET_G     = 0x0A,
    PARAM_WEIGH_TIMEOUT_MS    = 0x0B,
    PARAM_CONVEYOR_SPEED_HZ   = 0x0C,
    PARAM_PACK_SPEED_HZ       = 0x0D,
    PARAM_PACK_WRAP_TIME_MS   = 0x0E,
    PARAM_PACK_MIDDLE_DELAY_MS = 0x0F,
    PARAM_PACK_HEAD_INTERVAL_MS = 0x10,
    PARAM_PACK_CUT_DELAY_MS   = 0x11,
    PARAM_PACK_SERVO_CUT      = 0x12,
    PARAM_PACK_DIST_HEAD      = 0x13,  /* 葱头阈值 mm */
    PARAM_PACK_DIST_TAIL      = 0x14,  /* 葱尾阈值 mm */
    PARAM_PACK_TIMEOUT_MS     = 0x15,
    PARAM_SCREW_LIMIT_DOWN_MS = 0x16,  /* 丝杆正转限位时间 ms */
    PARAM_SCREW_LIMIT_UP_MS   = 0x17,  /* 丝杆反转限位时间 ms */
    PARAM_WEIGHT_STABLE_MS    = 0x18,  /* 称重达标连续保持 ms */
    PARAM_WEIGHT_HYSTERESIS_G = 0x19,  /* 称重回差 g */
    PARAM_PACK_ALIGN_REVERSE_MS = 0x1A,
    PARAM_PACK_M3_MS_PER_REV = 0x1B,
    PARAM_PACK_IMPELLER_WHEEL_SPD = 0x1C,
    PARAM_PACK_IMPELLER_ACC       = 0x1D,
    PARAM_PACK_IMPELLER_EJECT_MS  = 0x1E,
    PARAM_PACK_IMPELLER_EJ_SPD3   = 0x1F,
    PARAM_PACK_IMPELLER_EJ_SPD8   = 0x20,
    PARAM_PACK_ROD_ID2_PULL       = 0x21,
    PARAM_PACK_ROD_ID2_PUSH       = 0x22,
    PARAM_PACK_ROD_ID7_PULL       = 0x23,
    PARAM_PACK_ROD_ID7_PUSH       = 0x24,
    PARAM_PACK_ROD_GOAL_TIME      = 0x25,
    PARAM_PACK_ROD_GOAL_SPEED     = 0x26,
    PARAM_PACK_ROD_SETTLE_MS      = 0x27,
    PARAM_PACK_US_SIM_ENTER_MS    = 0x28,
    PARAM_PACK_US_SIM_LEAVE_MS    = 0x29,
    PARAM_ENABLE_HEIGHT_SCREW     = 0x2A, /* 非0开测高+丝杆，0跳过 */
    PARAM_PROCESS_PLAN            = 0x2B, /* 0称重 1数量 */
    PARAM_TARGET_COUNT            = 0x2C, /* 1~7 */
    PARAM_CLEAN_MODE          = 0xF1,  /* 特殊：清洗模式 */
    PARAM_PACK_MODE           = 0xF2   /* 特殊：打包模式 */
} ProcessParamID_TypeDef;

/*-----------------------------------------------------------
 * 函数声明
 *-----------------------------------------------------------*/
void  Process_Init(void);
void  Process_Main_SM_Run(void);

/* 分区A夹爪子状态机 */
void  Process_Clamp_SM_Run(void);

/* 分区B清洗子状态机 */
void  Process_Clean_SM_Run(void);

/* 分区C等待子状态机 */
void  Process_Wait_SM_Run(void);

/* 分区D打包子状态机 */
void  Process_Pack_Zone_SM_Run(void);

void  Process_StopAll(void);
void  Process_PackRodBus_Idle(void);
void  Process_EmergencyStop(void);
void  Process_UpdateSensors(ProcessSensors_TypeDef* sensors);
void  Process_Start(void);
void  Process_Stop(void);
void  Process_Pause(void);
void  Process_Resume(void);
void  Process_EmergencyStop_Resume(void);
void  Process_SetProcState_Clear(void);

ProcessPhase_TypeDef    Process_GetPhase(void);
ClampState_TypeDef     Process_GetClampState(void);
CleanState_TypeDef     Process_GetCleanState(void);
WaitState_TypeDef      Process_GetWaitState(void);
PackZoneState_TypeDef  Process_GetPackZoneState(void);
CleanMode_TypeDef      Process_GetCleanMode(void);
PackTriggerMode_TypeDef Process_GetPackMode(void);

uint16_t Process_GetParam(uint8_t param_id);
void     Process_SetParam(uint8_t param_id, uint16_t value);

/* 测距驱动 */
uint16_t Process_ReadPackDistance(void);
uint16_t Process_ReadClampDistance(void);

#endif /* __PROCESS_CONTROL_H__ */
