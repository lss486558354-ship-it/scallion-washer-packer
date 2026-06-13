#!/usr/bin/env python3
"""
Apply all 6 changes to main.c in one pass.

CHANGE 1: Add g_pack_test_total_ms declaration after g_pack_test_seg_ms
CHANGE 2: Replace PACK_TEST trigger to use new state machine
CHANGE 3: Delete old partition H boot auto-run
CHANGE 4: Replace partition D state machine with new one
CHANGE 5: Delete old partition E
CHANGE 6: Delete old partition H state machine
"""

import re

# Read source file
src_path = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00 - 副本 (2)\project\User\main.c"
with open(src_path, 'r', encoding='utf-8') as f:
    content = f.read()

lines = content.split('\n')
print(f"Original file: {len(lines)} lines")

# ============================================================================
# CHANGE 1: Add g_pack_test_total_ms declaration after g_pack_test_seg_ms
# ============================================================================
# Find the line with g_pack_test_seg_ms (line 107 area)
for i, line in enumerate(lines):
    if 'g_pack_test_seg_ms' in line and 'static volatile uint32_t' in line:
        # Insert the new declaration after this line
        lines.insert(i + 1, 'static volatile uint32_t             g_pack_test_total_ms; /* 打包阶段绝对累计时间 */')
        print(f"CHANGE 1: Added g_pack_test_total_ms at line {i+2}")
        break

# ============================================================================
# CHANGE 3: Delete old partition H boot auto-run (after s_app_tick_armed = 1u)
# ============================================================================
# Find "s_app_tick_armed = 1u;" and delete the block that follows until
# the "/* 确保 PA9/PA10" comment

new_lines = []
i = 0
while i < len(lines):
    line = lines[i]

    # Found the trigger point
    if 's_app_tick_armed = 1u;' in line:
        new_lines.append(line)
        i += 1
        # Skip lines until we find the comment "/* 确保 PA9/PA10"
        while i < len(lines):
            if '/* 确保 PA9/PA10' in lines[i] or '确保 PA9/PA10' in lines[i]:
                break
            i += 1
        print(f"CHANGE 3: Deleted partition H boot auto-run block")
        continue

    new_lines.append(line)
    i += 1

lines = new_lines

# ============================================================================
# CHANGE 2: Replace PACK_TEST trigger in main()
# ============================================================================
# Find the block where g_protocol_pack_test_request triggers PACK_TEST_IMPELLER_ON
# Original:
#     g_pack_test_state = PACK_TEST_IMPELLER_ON;
#     g_pack_test_seg_ms = 0u;
#
# Replace with:
#     PackTest_StopAll();
#     g_new_proc_state = PROC_NEW_PACK_IMPELLER;
#     g_new_proc_seg_ms = 0u;

for i, line in enumerate(lines):
    if 'g_pack_test_state = PACK_TEST_IMPELLER_ON;' in line and 'g_pack_test_seg_ms' not in line:
        # Check if previous line has PackTest_StopAll
        if i > 0 and 'PackTest_StopAll();' in lines[i-1]:
            # This is the one in the trigger block, replace it
            lines[i] = '                g_new_proc_state = PROC_NEW_PACK_IMPELLER;'
            lines[i+1] = '                g_new_proc_seg_ms = 0u;'
            print(f"CHANGE 2: Replaced PACK_TEST trigger at line {i+1}")
            break

# ============================================================================
# CHANGE 4: Replace partition D state machine with new one
# ============================================================================
# Find "【分区 D】新工艺流程主状态机" through just before "【分区 E】"
# Build the new partition F state machine

new_partition_f = '''    /* ========================================================================
     * 【分区 F】切割+打包联合状态机
     *
     * 触发：USART3 收到 0x9A 命令（或调试串口命令）
     *
     *   IDLE ──(0x9A)──→ BELT_RUN ──(超声<阈值)──→ CUT_RUN
     *     ↑                   ↑                        │
     *     │                  IR计数<3              IR计数≥3
     *     │                    │                    ↓
     *     └──────PACK_STOP←──────PACK_ROD_ON_2←────PACK_IMPELLER
     *                                       │B7低   │B7高
     *                                  PACK_M4     PACK_ROD_ON_1
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

        /* IR 红外计数：遮挡触发 + 1s 防抖 */
        static uint8_t  s_ir_cnt = 0u;
        static uint8_t  s_ir_triggered = 0u;
        static uint32_t s_ir_trigger_tick = 0u;

        /* IR 低电平（遮挡）触发计数 */
        if (IR_COUNTER_LOW() && !s_ir_triggered) {
            s_ir_cnt++;
            s_ir_triggered = 1u;
            s_ir_trigger_tick = now_tick;
            UART1_PrintfNB("[IR_CNT] +1 cnt=%u\\r\\n", (unsigned)s_ir_cnt);
        }
        /* IR 高电平后等待 1s 才允许下次计数 */
        if (s_ir_triggered && !IR_COUNTER_LOW()) {
            if ((uint32_t)(now_tick - s_ir_trigger_tick) >= 1000u) {
                s_ir_triggered = 0u;
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
         * PROC_NEW_BELT_RUN：同步带 CW 运转，红外计数
         * ------------------------------------------------------------------ */
        case PROC_NEW_BELT_RUN: {
            static uint32_t s_us_poll_tick;
            static uint8_t  s_us_triggered_this;

            if (seg == 0u) {
                Stepper_SyncRun_Motor1Reversed(DFLT_BELT_SPEED_HZ, STEPPER_DIR_CW);
                s_us_poll_tick = now_tick;
                s_us_triggered_this = 0u;
                UART1_PrintfNB("[NEW] BELT_RUN start\\r\\n");
            }

            if ((uint32_t)(now_tick - s_us_poll_tick) >= 60U) {
                s_us_poll_tick = now_tick;
                uint16_t dist = New_ReadUltrasonicDistance();

                if (dist != 0xFFFF && dist < 50 && !s_us_triggered_this) {
                    s_us_triggered_this = 1u;
                    g_new_proc_state = PROC_NEW_CUT_RUN;
                    g_new_proc_seg_ms = 0u;
                    UART1_PrintfNB("[NEW] US trig dist=%d, -> CUT_RUN\\r\\n", dist);
                    break;
                }
                if (dist == 0xFFFF || dist >= 50) {
                    s_us_triggered_this = 0u;
                }
            }

            seg++;
            g_new_proc_seg_ms = seg;
            break;
        }

        /* ------------------------------------------------------------------
         * PROC_NEW_CUT_RUN：切割电机 1.5s
         * ------------------------------------------------------------------ */
        case PROC_NEW_CUT_RUN: {
            if (seg == 0u) {
                GPIOC->BSRR = GPIO_Pin_9;
                Stepper_SyncRun_Motor1Reversed(DFLT_BELT_SPEED_HZ, STEPPER_DIR_CW);
                UART1_PrintfNB("[NEW] CUT_RUN start\\r\\n");
            }

            seg++;

            if (seg == 1500u) {
                GPIOC->BRR = GPIO_Pin_9;
                UART1_PrintfNB("[NEW] Cutter stop\\r\\n");
            }

            if (seg >= 8000u) {
                Stepper_SyncStop();

                if (s_ir_cnt >= 3u) {
                    UART1_PrintfNB("[NEW] IR cnt=%u, enter PACK flow\\r\\n", s_ir_cnt);
                    s_pack_start_tick = now_tick;
                    s_pack_m4_started = 0u;
                    s_pack_recovered = 0u;
                    g_pack_test_total_ms = 0u;
                    g_new_proc_state = PROC_NEW_PACK_IMPELLER;
                } else {
                    UART1_PrintfNB("[NEW] IR cnt=%u (<3), back to BELT_RUN\\r\\n", s_ir_cnt);
                    g_new_proc_state = PROC_NEW_BELT_RUN;
                }
                g_new_proc_seg_ms = 0u;
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
                ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID1, (int16_t)2, M34_SCS_WHEEL_ACC);
                ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID2, (int16_t)(-130), M34_SCS_WHEEL_ACC);
                ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID1, (int16_t)2);
                ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID2, (int16_t)(-130));
                M3_Run(PACK_TEST_M3_SPEED_HZ, STEPPER_DIR_CW);
                UART1_PrintfNB("[NEW] PACK_IMPELLER start\\r\\n");
            }
            PackTest_ImpellerKeepalive();

            if (PACK_B7_LOW() && !s_pack_m4_started) {
                s_pack_m4_started = 1u;
                s_pack_m4_start_tick = now_tick;
                M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);
                g_new_proc_state = PROC_NEW_PACK_M4;
                g_new_proc_seg_ms = 0u;
                UART1_PrintfNB("[NEW] -> PACK_M4\\r\\n");
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
            M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);

            if ((uint32_t)(now_tick - s_pack_m4_start_tick) >= (PACK_TEST_ROD_ON_MS - 1000u)) {
                s_pack_m4_started = 0u;
                g_new_proc_state = PROC_NEW_PACK_ROD_ON_1;
                g_new_proc_seg_ms = 0u;
                UART1_PrintfNB("[NEW] -> PACK_ROD_ON_1\\r\\n");
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
                UART1_PrintfNB("[NEW] PACK_ROD_ON_1 entry\\r\\n");

                M3_Stop();
                M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);
                M34_ScsBus_StopWheelImpellers();

                int8_t d2 = PackTest_RodDirBetween(PACK_TEST_ROD_ID2_PULL, PACK_TEST_ROD_ID2_PUSH);
                int8_t d7 = PackTest_RodDirBetween(PACK_TEST_ROD_ID7_PULL, PACK_TEST_ROD_ID7_PUSH);
                int16_t sp2 = (d2 >= 0) ? (int16_t)PACK_TEST_ROD_SPEED_ID2 : (int16_t)(-((int16_t)PACK_TEST_ROD_SPEED_ID2));
                int16_t sp7 = (d7 >= 0) ? (int16_t)PACK_TEST_ROD_SPEED_ID7 : (int16_t)(-((int16_t)PACK_TEST_ROD_SPEED_ID7));

                if (d2 == 0 && d7 == 0) {
                    UART1_PrintfNB("[NEW] ROD_ON_1 aligned, skip\\r\\n");
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
                UART1_PrintfNB("[NEW] -> PACK_ROD_OFF\\r\\n");
                break;
            }

            seg++;
            g_new_proc_seg_ms = seg;
            break;
        }

        /* ------------------------------------------------------------------
         * PROC_NEW_PACK_ROD_OFF：M3+M4+叶轮恢复
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_ROD_OFF: {
            static uint8_t s_b7_wait_phase; /* 0=等低电平 1=等上升沿 */
            static uint8_t s_entered;

            /* ========== 初始化：启动M4+叶轮 ========== */
            if (!s_entered) {
                s_entered = 1u;
                s_b7_wait_phase = 0u;
                UART1_PrintfNB("[NEW] PACK_ROD_OFF entry, starting M4+impeller\\r\\n");

                /* 启动M4+叶轮，保持缠绕运动 */
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

            /* ========== 持续运行M4+叶轮 ========== */
            PackTest_ImpellerKeepalive();
            M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);

            /* ========== 阶段0：强制等待B7低电平 ========== */
            if (s_b7_wait_phase == 0u) {
                if (PACK_B7_LOW()) {
                    s_b7_wait_phase = 1u;
                    UART1_PrintfNB("[NEW] B7 LOW, waiting for rising edge\\r\\n");
                }
            }
            /* ========== 阶段1：等待B7上升沿（低→高）========== */
            else if (s_b7_wait_phase == 1u) {
                if (PACK_B7_HIGH()) {
                    /* 上升沿触发：停止M4+叶轮，进入膜杆第二节 */
                    M4_Stop();
                    M34_ScsBus_StopWheelImpellers();
                    s_entered = 0u;
                    s_b7_wait_phase = 0u;
                    g_new_proc_state = PROC_NEW_PACK_ROD_ON_2;
                    g_new_proc_seg_ms = 0u;
                    seg = 0u;
                    UART1_PrintfNB("[NEW] B7 rising edge -> PACK_ROD_ON_2\\r\\n");
                    break;
                }
            }

            /* ========== 超时兜底（5分钟）========== */
            if (seg >= 300000u) {
                M4_Stop();
                M34_ScsBus_StopWheelImpellers();
                s_entered = 0u;
                s_b7_wait_phase = 0u;
                g_new_proc_state = PROC_NEW_PACK_ROD_ON_2;
                g_new_proc_seg_ms = 0u;
                seg = 0u;
                UART1_PrintfNB("[NEW] -> PACK_ROD_ON_2 (timeout)\\r\\n");
                break;
            }

            seg++;
            g_new_proc_seg_ms = seg;
            break;
        }

        /* ------------------------------------------------------------------
         * PROC_NEW_PACK_ROD_ON_2：膜杆 2/7 第二节
         * 入口：停止M3/M4/叶轮 + 启动膜杆（第一节拉膜后的反向推回）
         * 等待B7高电平持续PACK_TEST_ROD_LEG_MS（葱头完全离开）后进入切膜
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_ROD_ON_2: {
            static uint8_t s_rod_armed;
            static uint8_t s_entered;
            static uint8_t s_b7_high_started;
            static uint32_t s_b7_high_start_tick;

            /* ========== 初始化：启动膜杆第二节 ========== */
            if (!s_entered) {
                s_entered = 1u;
                s_b7_high_started = 0u;
                s_rod_armed = 0u;
                UART1_PrintfNB("[NEW] PACK_ROD_ON_2 entry\\r\\n");

                M3_Stop();
                M4_Stop();
                M34_ScsBus_StopWheelImpellers();

                /* 启动膜杆第二节：计算从当前位置去目标位置的方向 */
                int8_t d2 = PackTest_RodDirBetween(PACK_TEST_ROD_ID2_PULL, PACK_TEST_ROD_ID2_PUSH);
                int8_t d7 = PackTest_RodDirBetween(PACK_TEST_ROD_ID7_PULL, PACK_TEST_ROD_ID7_PUSH);
                int16_t sp2 = (d2 >= 0) ? (int16_t)PACK_TEST_ROD_SPEED_ID2 : (int16_t)(-((int16_t)PACK_TEST_ROD_SPEED_ID2));
                int16_t sp7 = (d7 >= 0) ? (int16_t)PACK_TEST_ROD_SPEED_ID7 : (int16_t)(-((int16_t)PACK_TEST_ROD_SPEED_ID7));

                if (d2 == 0 && d7 == 0) {
                    UART1_PrintfNB("[NEW] ROD_ON_2 aligned, skip\\r\\n");
                } else {
                    PackTest_RodWheelRunBoth(sp2, sp7);
                    s_rod_armed = 1u;
                    UART1_PrintfNB("[NEW] ROD_ON_2: d2=%d d7=%d\\r\\n", d2, d7);
                }
            }

            /* ========== 等待B7高电平持续PACK_TEST_ROD_LEG_MS ========== */
            if (PACK_B7_HIGH()) {
                if (!s_b7_high_started) {
                    s_b7_high_started = 1u;
                    s_b7_high_start_tick = now_tick;
                    UART1_PrintfNB("[NEW] B7 HIGH at %u, counting %ums...\\r\\n",
                        (unsigned)g_pack_test_total_ms, (unsigned)PACK_TEST_ROD_LEG_MS);
                }
                if ((uint32_t)(now_tick - s_b7_high_start_tick) >= PACK_TEST_ROD_LEG_MS) {
                    /* B7高电平持续足够时间，停止膜杆+进入切膜 */
                    s_entered = 0u;
                    s_b7_high_started = 0u;
                    if (s_rod_armed) {
                        PackTest_RodWheelStopBoth();
                        s_rod_armed = 0u;
                    }
                    g_new_proc_state = PROC_NEW_PACK_ROD_CUTFILM;
                    g_new_proc_seg_ms = 0u;
                    seg = 0u;
                    UART1_PrintfNB("[NEW] -> PACK_ROD_CUTFILM\\r\\n");
                    break;
                }
            } else {
                s_b7_high_started = 0u;
            }

            /* ========== 超时兜底（PACK_TEST_ROD_LEG_MS * 2）========== */
            seg++;
            g_new_proc_seg_ms = seg;
            if (seg >= (PACK_TEST_ROD_LEG_MS * 2u)) {
                s_entered = 0u;
                s_b7_high_started = 0u;
                if (s_rod_armed) {
                    PackTest_RodWheelStopBoth();
                    s_rod_armed = 0u;
                }
                g_new_proc_state = PROC_NEW_PACK_ROD_CUTFILM;
                g_new_proc_seg_ms = 0u;
                seg = 0u;
                UART1_PrintfNB("[NEW] -> PACK_ROD_CUTFILM (timeout)\\r\\n");
                break;
            }
            break;
        }

        /* ------------------------------------------------------------------
         * PROC_NEW_PACK_STOP：全部停止，返回 IDLE
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_STOP: {
            PackTest_StopAll();
            UART1_PrintfNB("[NEW] PACK done, back to IDLE\\r\\n");

            g_new_proc_state = PROC_NEW_IDLE;
            g_new_proc_seg_ms = 0u;
            g_us_count = 0u;
            g_us_count_flag = 0u;
            s_ir_cnt = 0u;
            s_ir_triggered = 0u;
            g_pack_test_total_ms = 0u;
            break;
        }

        default:
            g_new_proc_state = PROC_NEW_IDLE;
            break;
        }
    }

'''

# Find partition D start and partition E start
part_d_start = -1
part_e_start = -1
part_f_marker = -1
part_h_start = -1

for i, line in enumerate(lines):
    if '【分区 D】新工艺流程主状态机' in line:
        part_d_start = i
    if '【分区 E】超声波检测计数' in line:
        part_e_start = i
    if '【分区 F】USART3 0x9A 命令触发新状态机' in line:
        part_f_marker = i
    if '【分区 H】打包测试状态机' in line and '上电' not in line:
        part_h_start = i

print(f"CHANGE 4: Found partition D at line {part_d_start+1}")
print(f"CHANGE 5: Found partition E at line {part_e_start+1}")
print(f"CHANGE 6: Found partition H at line {part_h_start+1}")

# Rebuild lines:
# Keep everything before partition D
# Insert new partition F
# Skip partition D, E, and the old partition F/USART3 trigger
# Keep everything after old partition H

# First, find where partition D's closing brace ends (find matching })
# The partition D block starts with "【分区 D】新工艺流程主状态机"
# and contains a switch/case. Need to find the closing brace

if part_d_start >= 0 and part_e_start >= 0:
    # Lines from part_d_start to part_e_start-1 are partition D
    # Replace those with new partition F
    before_d = lines[:part_d_start]
    after_e = lines[part_e_start:]

    lines = before_d + [new_partition_f] + after_e
    print(f"CHANGE 4+5: Replaced partition D and deleted partition E")
else:
    print("WARNING: Could not find partition D or E boundaries!")

# Re-scan for partition H (now at different position)
for i, line in enumerate(lines):
    if '【分区 H】打包测试状态机' in line and '上电' not in line:
        part_h_start = i
        break

# Find the closing brace of partition H block
if part_h_start >= 0:
    # Find the matching closing brace
    brace_count = 0
    h_end = -1
    for i in range(part_h_start, len(lines)):
        line = lines[i]
        brace_count += line.count('{')
        brace_count -= line.count('}')
        if brace_count == 0 and i > part_h_start:
            h_end = i
            break

    if h_end >= 0:
        # Keep lines before partition H, skip partition H block
        lines = lines[:part_h_start] + lines[h_end+1:]
        print(f"CHANGE 6: Deleted partition H state machine (lines {part_h_start+1}-{h_end+1})")
    else:
        print("WARNING: Could not find end of partition H block!")

# Reconstruct content
content = '\n'.join(lines)
lines = content.split('\n')
print(f"\nFinal file: {len(lines)} lines")

# ============================================================================
# Verification
# ============================================================================
open_braces = content.count('{')
close_braces = content.count('}')
open_comments = content.count('/*')
close_comments = content.count('*/')

print(f"\n=== VERIFICATION ===")
print(f"Braces: {{ = {open_braces}, }} = {close_braces}, {'BALANCED' if open_braces == close_braces else 'UNBALANCED!'}")
print(f"Comments: /* = {open_comments}, */ = {close_comments}, {'BALANCED' if open_comments == close_comments else 'UNBALANCED!'}")

# Check if file ends with closing brace
ends_with_brace = content.rstrip().endswith('}')
print(f"Ends with closing brace: {'YES' if ends_with_brace else 'NO!'}")

if open_braces != close_braces:
    print(f"\nWARNING: Brace mismatch! {{ = {open_braces}, }} = {close_braces}")
if open_comments != close_comments:
    print(f"\nWARNING: Comment mismatch! /* = {open_comments}, */ = {close_comments}")

# Write output file
out_path = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c"
with open(out_path, 'w', encoding='utf-8') as f:
    f.write(content)

print(f"\nOutput written to: {out_path}")
