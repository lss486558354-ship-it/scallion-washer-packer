/**
 ******************************************************************************
 * 大葱机主控头文件
 *
 * 功能：
 *   - 4分区状态机枚举定义
 *   - Device对象化设备调度接口
 *   - 传感器GPIO宏定义
 *   - 统一调度API
 *
 * 版本：V3.0
 * 日期：2026-04-18
 ******************************************************************************
 */

#ifndef __MAIN_H__
#define __MAIN_H__

#define FW_VERSION  "2026_04_05_ver_01.10.00"

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

/*-----------------------------------------------------------
 * 功能开关配置
 *-----------------------------------------------------------*/

/* 打包超声波测试打印：1=每1s打印距离(cm) */
#ifndef MAIN_PACK_ULTRA_TEST_ENABLE
#define MAIN_PACK_ULTRA_TEST_ENABLE  0
#endif
#define MAIN_PACK_ULTRA_TEST_PERIOD_MS  1000u

/*-----------------------------------------------------------
 * LED和按钮GPIO定义
 *-----------------------------------------------------------*/
#define LED_GPIO_PORT  GPIOC
#define LED_GPIO_PIN   GPIO_Pin_13
#define LED_RCC        RCC_APB2Periph_GPIOC
#define LED_TOGGLE() GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN, (BitAction)(1 - GPIO_ReadOutputDataBit(LED_GPIO_PORT, LED_GPIO_PIN)))

#define START_BTN_PORT   GPIOC
#define START_BTN_PIN    GPIO_Pin_1
#define START_BTN_RCC    RCC_APB2Periph_GPIOC
#define START_BTN_DOWN() (GPIO_ReadInputDataBit(START_BTN_PORT, START_BTN_PIN) == Bit_RESET)

/*-----------------------------------------------------------
 * M34电机自检相关定义
 *-----------------------------------------------------------*/

/* 电机3/4 自检复位：PC5 内部上拉，按下接地 */
#define M34_RST_BTN_PORT   GPIOC
#define M34_RST_BTN_PIN    GPIO_Pin_5
#define M34_RST_BTN_DOWN() (GPIO_ReadInputDataBit(M34_RST_BTN_PORT, M34_RST_BTN_PIN) == Bit_RESET)

#define M34_BOOT_REV_MS 206u  /* 传送带反转段时长 ms（32细分，2570/12.5） */
#define M34_RUN_HZ_CONVEY    40u  /* M3传送带自检频率 Hz（32细分，250/12.5） */
#define M34_RUN_HZ_PACK     250u  /* M4打包自检频率 Hz（400细分，不变） */

/* UART4 总线舵机 ID（飞特）*/
#define M34_SCS_WHEEL_ID1       3u
#define M34_SCS_WHEEL_ID2       8u
#define M34_SCS_WHEEL_SPD_ABS   2
#define M34_SCS_WHEEL_SPD       ((int16_t)M34_SCS_WHEEL_SPD_ABS)
#define M34_SCS_WHEEL_SPD_ID8   ((int16_t)(-130))
#define M34_SCS_WHEEL_ACC       40u
#define M34_SCS_REFRESH_MS      120u

/*-----------------------------------------------------------
 * 分区H：打包测试参数（可按需调整）
 *-----------------------------------------------------------*/
/**
 * 新时序（ms）：
 *   0ms      : 步进电机3 + 舵机叶轮(ID3/8) 启动（M3传送带联动）
 *   B7低电平 : 步进电机4 启动（M4单独启动）
 *   3200ms后 : 舵机膜杆(ID2/7) 第一节启动（叶轮+M3+M4停止），运行200ms
 *   ~3400ms  : 膜杆停止，叶轮+M3+M4 重新启动
 *   B7高电平 : 舵机膜杆(ID2/7) 第二节启动（M3+M4+叶轮停止），运行530ms
 *   B7高电平后510ms : M4@1200+叶轮运行，1000ms后全部停止
 *
 * 膜杆两节动作时间: 第一节=PACK_TEST_ROD_LEG_MS(510ms), 第二节=510ms
 */
#define PACK_TEST_IMPELLER_START_MS   0u       /* 叶轮+M3 启动时刻 */
#define PACK_TEST_IMPELLER_DURATION_MS  25400u  /* 总持续时间上限保护 */
#define PACK_TEST_M4_START_MS          1000u    /* M4 启动时刻（已废弃，由B7低电平触发）*/
#define PACK_TEST_ROD_ON_MS           3200u    /* 膜杆启动时刻（全部停止）*/
#define PACK_TEST_ROD_LEG_MS           510u    /* 膜杆每节运行时间（动作时长） */
#define PACK_TEST_M3_RECOVER_MS       4000u    /* M3+M4+叶轮恢复时刻（膜杆停止时）*/
#define PACK_TEST_ROD_LEG2_START_MS   18800u    /* 膜杆第二节触发（B7高电平，已废弃时间触发）*/
#define PACK_TEST_ROD_LEG2_STOP_MS    19600u   /* 膜杆第二节停止时刻（B7高电平后530ms，已废弃，以代码内530u为准）*/
/* M3 反转参数（独立计时段，触发前执行） */
#define PACK_TEST_M3_CCW_SPEED_HZ     200u    /* M3 反转速度 Hz */
/* 膜杆 ID2/ID7 轮速方向 */
#define PACK_TEST_ROD_ID2_PULL   512u
#define PACK_TEST_ROD_ID2_PUSH  2560u
#define PACK_TEST_ROD_ID7_PULL   512u
#define PACK_TEST_ROD_ID7_PUSH  2560u
/* 膜杆轮速幅值（舵机2:9999,舵机7:9999，方向由传感器决定符号）
 * 官方STS360满速: int16范围±10~±9999，当前设为最大值9999 */
#define PACK_TEST_ROD_SPEED_ID2    9999u     /* 舵机2 轮速幅值（满速） */
#define PACK_TEST_ROD_SPEED_ID7    9999u     /* 舵机7 轮速幅值（满速） */
#define PACK_TEST_ROD_ACC         254u     /* 膜杆加减速（最大值，加速越快停止越快） */
/* 膜杆（舵机2/7）对应的总线舵机 ID */
#define PACK_TEST_ROD_ID2        2u       /* 膜杆A → 总线 ID 2 */
#define PACK_TEST_ROD_ID7        7u       /* 膜杆B → 总线 ID 7 */
#define PACK_TEST_M4_SPEED_HZ    10000u    /* M4打包传送带速度 Hz（加倍） */
#define PACK_TEST_FINAL_M4_SPEED_HZ 16200u  /* M4最终阶段速度 Hz（加倍） */
#define PACK_TEST_M3_SPEED_HZ    200u     /* M3传送带速度 Hz（联动叶轮）*/
/* 膜杆两位置间的方向判断：取中间值比较，短弧方向为正 */
#define PACK_TEST_ROD_SHORTARC_HALF  2048u
#define PACK_TEST_ROD_STEPS_PER_REV  4096u

/*-----------------------------------------------------------
 * 全局运行标志（供外部访问）
 *-----------------------------------------------------------*/
extern volatile uint8_t g_process_run_request;

/* 旧版流程状态（main.c 自检用）*/
typedef enum {
    PROC_IDLE = 0,
    PROC_FLOW1_SYNC,
    PROC_FLOW2_SCREW_CCW,
    PROC_FLOW2_SYNC_2S,
    PROC_FLOW2_SCREW_CW,
    PROC_FLOW2_SYNC_10S,
    PROC_FLOW3_CONVEY,
    PROC_FLOW3_PACK1,
    PROC_FLOW3_GAP,
    PROC_FLOW3_PACK2,
    PROC_DONE
} Process_State_TypeDef;

extern volatile Process_State_TypeDef g_proc_state;
extern volatile uint16_t              g_proc_seg_ms;
extern volatile uint8_t               g_proc_cycle;

/*-----------------------------------------------------------
 * 打包测试辅助函数（分区F联合状态机调用）
 *-----------------------------------------------------------*/
typedef enum {
    PROCESS_IDLE           = 0x00,  /* 空闲，等待开始命令 */
    PROCESS_ZONE_A_CLAMP,           /* 分区A：夹入-超声-丝杆-切割 */
    PROCESS_ZONE_B_CLEAN,          /* 分区B：清洗+吹气 */
    PROCESS_ZONE_C_WAIT,           /* 分区C：定时等待大葱到位 */
    PROCESS_ZONE_D_PACK,           /* 分区D：打包（红外+超声双重检测）*/
    PROCESS_DONE,                    /* 本次流程完成 */
    PROCESS_PAUSED                   /* 已暂停，保留 phase_saved */
} ProcessPhase_TypeDef;

/*-----------------------------------------------------------
 * 分区A 夹爪子状态机枚举
 *-----------------------------------------------------------*/
typedef enum {
    CLAMP_IDLE = 0x00,
    CLAMP_GRIP,          /* M1/M2 夹入动作 */
    CLAMP_US_SENSE,      /* 超声波检测大葱（等待葱到后停止夹入动作），大葱计数+1，计数标志=1，进入分区C时置零 */
    CLAMP_SCREW_ADJUST,  /* 丝杆根据超声波距离调整位置 */
    CLAMP_SCREW_HOLD,    /* 丝杆到位后保持 */
    CLAMP_CUT,           /* 切割电机动作0.5s（同步带同时继续运行）*/
    CLAMP_DONE           /* 分区A完成，等待进入分区B */
} ClampState_TypeDef;

/*-----------------------------------------------------------
 * 分区B 清洗子状态机枚举
 *-----------------------------------------------------------*/
typedef enum {
    CLEAN_IDLE       = 0x00,  /* 未进入清洗 */
    CLEAN_ENTER,              /* 进入清洗，初始化 */
    CLEAN_SPRAY_ON,           /* 喷气开启 */
    CLEAN_SPRAY_OFF,          /* 喷气关闭 */
    CLEAN_WATER_ON,           /* 水泵开启 */
    CLEAN_WATER_OFF,          /* 水泵关闭 */
    CLEAN_DONE                /* 清洗完成 */
} CleanState_TypeDef;

/*-----------------------------------------------------------
 * 分区C 等待子状态机枚举
 *-----------------------------------------------------------*/
typedef enum {
    WAIT_IDLE = 0x00,
    WAIT_DELAY,          /* 定时等待大葱到位 */
    WAIT_READY           /* 等待完成 */
} WaitState_TypeDef;

/*-----------------------------------------------------------
 * 分区D 打包子状态机枚举
 *-----------------------------------------------------------*/
typedef enum {
    PACK_ZONE_IDLE = 0x00,
    PACK_ZONE_ALIGN_REV,     /* M3反转2s对齐 */
    PACK_ZONE_FEED_FWD,     /* M3正转+叶轮(ID3/8)夹入 */
    PACK_ZONE_WAIT_PACK,    /* 等超声波检测大葱到达 */
    PACK_ZONE_WRAP_START,   /* 缠膜两圈后，套杆抽离(舵机2/7半圈) */
    PACK_ZONE_ROD_PULL,     /* 超声确认离开：套杆送入(舵机2/7半圈)，然后缠膜一圈同时后切膜电机正2s反2s(M3暂停) */
    PACK_ZONE_WRAP_TAIL,    /* 等超声检测葱尾+红外确认离开 */
    PACK_ZONE_WRAP_COMPLETE,/* 打包完成 */
    PACK_ZONE_EJECT,        /* M3+叶轮运行4s抛出 */
    PACK_ZONE_DONE
} PackZoneState_TypeDef;

/*-----------------------------------------------------------
 * 打包触发模式枚举
 *-----------------------------------------------------------*/
typedef enum {
    PACK_MODE_FULL     = 0x01, /* 超声波开始，红外+超声结束 */
    PACK_MODE_MIDDLE  = 0x02, /* 红外开始，超声结束 */
    PACK_MODE_HEAD_TAIL = 0x03 /* 超声波开始2圈切断，超声波结束再2圈再切断 */
} PackTriggerMode_TypeDef;

/*-----------------------------------------------------------
 * 清洗模式枚举
 *-----------------------------------------------------------*/
typedef enum {
    CLEAN_MODE_NONE      = 0x00,  /* 未设置（默认全部）*/
    CLEAN_MODE_AIR_ONLY  = 0x01,  /* 仅喷气（高速气流去黄叶）*/
    CLEAN_MODE_WATER_ONLY = 0x02, /* 仅水泵清洗 */
    CLEAN_MODE_BOTH      = 0x03   /* 喷气+水泵都使用（默认）*/
} CleanMode_TypeDef;

/*-----------------------------------------------------------
 * 丝杆运动方向枚举
 *-----------------------------------------------------------*/
typedef enum {
    SCREW_STOPPED  = 0x00,  /* 停止 */
    SCREW_GOING_DOWN = 0x01, /* 正在下降（正转，向下限位方向）*/
    SCREW_GOING_UP   = 0x02  /* 正在上升（反转，向上限位方向）*/
} ScrewDirection_TypeDef;

/*-----------------------------------------------------------
 * 打包测距阈值定义（mm）
 *-----------------------------------------------------------*/
#define PACK_DIST_THRESH_HEAD   55   /* 葱头阈值：距离<此值认为葱头到达 */
#define PACK_DIST_THRESH_TAIL   65   /* 葱尾阈值：距离>此值认为葱尾离开 */
#define PACK_MEASURE_TIMEOUT_MS 30000 /* 打包测距总超时 ms */

/*-----------------------------------------------------------
 * 分区A超声波阈值定义（mm）
 *-----------------------------------------------------------*/
#define CLAMP_US_THRESHOLD_MM  50    /* 夹爪区域超声波检测阈值 */

/*-----------------------------------------------------------
 * 传感器GPIO定义
 *-----------------------------------------------------------*/

/* 分区A夹爪超声波 - HC-SR04 #1 */
#define CLAMP_US_TRIG_PORT   GPIOB
#define CLAMP_US_TRIG_PIN    GPIO_Pin_0
#define CLAMP_US_ECHO_PORT   GPIOD
#define CLAMP_US_ECHO_PIN    GPIO_Pin_3
#define CLAMP_US_ECHO_READ   GPIO_ReadInputDataBit(CLAMP_US_ECHO_PORT, CLAMP_US_ECHO_PIN)

/* 打包区域超声波 - HC-SR04 #2 */
#define PACK_US_TRIG_PORT    GPIOB
#define PACK_US_TRIG_PIN     GPIO_Pin_0
#define PACK_US_ECHO_PORT    GPIOD
#define PACK_US_ECHO_PIN     GPIO_Pin_4
#define PACK_US_ECHO_READ    GPIO_ReadInputDataBit(PACK_US_ECHO_PORT, PACK_US_ECHO_PIN)

/* 红外传感器 - PE8（遮光=有葱，低电平触发）*/
#define PACK_IR_SENSOR_PORT  GPIOE
#define PACK_IR_SENSOR_PIN   GPIO_Pin_8
#define PACK_IR_TRIGGERED()  (GPIO_ReadInputDataBit(PACK_IR_SENSOR_PORT, PACK_IR_SENSOR_PIN) == Bit_RESET)

/* 灰度传感器（PE7）：高电平→低电平跳变触发
 * 原 BELT_SENSOR 复用为灰度传感器，用于检测膜到位 */
#define GRAY_SENSOR_PORT      GPIOE
#define GRAY_SENSOR_PIN       GPIO_Pin_7
#define GRAY_SENSOR_HIGH      (GPIO_ReadInputDataBit(GRAY_SENSOR_PORT, GRAY_SENSOR_PIN) != Bit_RESET)
#define GRAY_SENSOR_LOW       (GPIO_ReadInputDataBit(GRAY_SENSOR_PORT, GRAY_SENSOR_PIN) == Bit_RESET)

/* B7 红外传感器 - PB7（低电平=大葱到位，高电平=大葱离开）*/
#define PACK_B7_SENSOR_PORT   GPIOB
#define PACK_B7_SENSOR_PIN    GPIO_Pin_7
#define PACK_B7_LOW()         (GPIO_ReadInputDataBit(PACK_B7_SENSOR_PORT, PACK_B7_SENSOR_PIN) == Bit_RESET)   /* 大葱到位 */
#define PACK_B7_HIGH()        (GPIO_ReadInputDataBit(PACK_B7_SENSOR_PORT, PACK_B7_SENSOR_PIN) == Bit_SET)   /* 大葱离开 */

/* PB6 红外计数器 - 输入上拉（遮挡=低电平，遮挡结束=高电平）*/
#define IR_COUNTER_PORT   GPIOB
#define IR_COUNTER_PIN    GPIO_Pin_6
#define IR_COUNTER_LOW()  (GPIO_ReadInputDataBit(IR_COUNTER_PORT, IR_COUNTER_PIN) == Bit_RESET)   /* 遮挡触发 */

/* 丝杆限位开关 E0/E1（GPIOE，PE0=E0上限位，PE1=E1下限位，上拉输入，触碰接地=低电平）*/
#define SCREW_LIMIT_E0_PORT    GPIOE
#define SCREW_LIMIT_E0_PIN    GPIO_Pin_0
/* E0=PE0：下限位（CCW反转方向）→ 高电平=安全可反转，低电平=撞到限位禁止反转 */
#define SCREW_LIMIT_E0_SAFE()  (GPIO_ReadInputDataBit(SCREW_LIMIT_E0_PORT, SCREW_LIMIT_E0_PIN) != Bit_RESET)

#define SCREW_LIMIT_E1_PORT    GPIOE
#define SCREW_LIMIT_E1_PIN    GPIO_Pin_1
/* E1=PE1：上限位（CW正转方向）→ 高电平=安全可正转，低电平=撞到限位禁止正转 */
#define SCREW_LIMIT_E1_SAFE()  (GPIO_ReadInputDataBit(SCREW_LIMIT_E1_PORT, SCREW_LIMIT_E1_PIN) != Bit_RESET)

/*=====================================================================
 * 切割+打包联合流程主状态机枚举
 *
 * 完整流程（0x9A命令启动）：
 *
 *   IDLE
 *     ↓ 收到 0x9A 命令
 *   BELT_RUN          同步带(电机1+2) CW运转
 *     ↓ 超声 dist<阈值
 *   CUT_RUN           切割电机正转 1.5s（同步带配合）
 *     ↓
 *   (count<3) → BELT_RUN 循环
 *   (count≥3) → PACK_IMPELLER_ON（进入打包流程）
 *     ↓ B7低
 *   PACK_M4_ON        M4启动
 *     ↓ 2200ms
 *   PACK_ROD_ON_1     膜杆第一节
 *     ↓ 800ms
 *   PACK_ROD_OFF      M3+M4+叶轮恢复
 *     ↓ B7高
 *   PACK_ROD_ON_2     膜杆第二节
 *     ↓ B7高1950ms
 *   PACK_STOP         全部停止 → IDLE
 *=====================================================================*/
typedef enum {
    /* 切割阶段 */
    PROC_NEW_IDLE = 0,        /* 等待 0x9A 命令 */
    PROC_NEW_BELT_RUN,        /* 同步带CW运转，红外计数 */
    PROC_NEW_CUT_RUN,         /* 切割电机1.5s */

    /* 打包阶段（复用分区H逻辑）*/
    PROC_NEW_PACK_IMPELLER,   /* M3+叶轮启动 */
    PROC_NEW_PACK_M4,         /* B7低→M4启动 */
    PROC_NEW_PACK_ROD_ON_1,   /* 膜杆第一节 */
    PROC_NEW_PACK_ROD_OFF,    /* M3+M4+叶轮恢复 */
    PROC_NEW_PACK_ROD_ON_2,   /* 膜杆第二节 */
    PROC_NEW_PACK_CUTFILM,    /* gray1触发→等gray2→M4停→切膜CW→CCW→PACK_STOP */
    PROC_NEW_PACK_STOP,        /* 全部停止，2s后返回BELT_RUN */
} Process_New_State_TypeDef;

/*-----------------------------------------------------------
 * 分区H：打包测试状态枚举
 *
 * 测试时序（B7红外传感器触发）：
 *   PACK_TEST_IDLE
 *     ↓ 触发（首次开机自动触发一次）
 *   PACK_TEST_IMPELLER_ON        // 0ms:    步进电机3+舵机叶轮(ID3/8)启动
 *     ↓ B7低电平（大葱到位）
 *   PACK_TEST_M4_ON              // M4启动（M3+叶轮继续）
 *     ↓ 2200ms后
 *   PACK_TEST_ROD_ON_1           // 舵机膜杆(ID2/7)第一节（全部停止）
 *     ↓ 800ms
 *   PACK_TEST_ROD_OFF            // 膜杆停止（M3+M4+叶轮恢复）
 *     ↓ B7高电平（大葱离开）
 *   PACK_TEST_ROD_ON_2           // 舵机膜杆(ID2/7)第二节（M3+M4+叶轮停止）
 *     ↓ B7高电平后1950ms
 *   PACK_TEST_STOP               // 全部停止
 *     ↓ 立即
 *   PACK_TEST_IDLE              // 返回空闲（不自动重启，等待下次触发）
 *-----------------------------------------------------------*/
typedef enum {
    PACK_TEST_IDLE = 0x00,
    PACK_TEST_IMPELLER_ON,       /* 0ms:     步进电机3 + 舵机叶轮(ID3/8)启动 */
    PACK_TEST_M4_ON,             /* B7低电平: 步进电机4启动（M3+叶轮继续）*/
    PACK_TEST_ROD_ON_1,          /* 2200ms后: 舵机膜杆(ID2/7)第一节（全部停止）*/
    PACK_TEST_ROD_OFF,           /* 3300ms:  膜杆停止（M3+M4+叶轮恢复）*/
    PACK_TEST_ROD_ON_2,          /* B7高电平: 舵机膜杆(ID2/7)第二节（M3+M4+叶轮停止）*/
    PACK_TEST_STOP               /* B7高电平后1950ms: 全部停止 */
} PackTestState_TypeDef;

/*-----------------------------------------------------------
 * Device对象化设备调度接口
 *-----------------------------------------------------------*/
typedef struct {
    void (*Init)(void);
    void (*Start)(void);
    void (*Stop)(void);
    void (*EmergencyStop)(void);
    uint8_t (*GetStatus)(void);
} Device_HandleTypeDef;

/* 外部设备句柄声明 */
extern Device_HandleTypeDef Device_Clamp;      /* 夹爪设备 */
extern Device_HandleTypeDef Device_Clean;      /* 清洗设备 */
extern Device_HandleTypeDef Device_Pack;       /* 打包设备 */
extern Device_HandleTypeDef Device_Cutter;     /* 切割设备 */
extern Device_HandleTypeDef Device_Screw;      /* 丝杆设备 */
extern Device_HandleTypeDef Device_Conveyor;   /* 传送带设备 */

/*-----------------------------------------------------------
 * 统一调度API声明
 *-----------------------------------------------------------*/
void Device_InitAll(void);
void Device_StopAll(void);
void Device_EmergencyStop(void);
void Device_ProcessLoop(void);

/*-----------------------------------------------------------
 * 状态获取API声明
 *-----------------------------------------------------------*/
ProcessPhase_TypeDef Device_GetPhase(void);
ClampState_TypeDef   Device_GetClampState(void);
CleanState_TypeDef   Device_GetCleanState(void);
WaitState_TypeDef    Device_GetWaitState(void);
PackZoneState_TypeDef Device_GetPackState(void);

#endif /* __MAIN_H__ */
