# -*- coding: utf-8 -*-
# This script applies all changes to main.c while maintaining brace balance
import sys
sys.stdout.reconfigure(encoding='utf-8')

main_path = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c"
with open(main_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

print(f"Original: {len(lines)} lines")

# ============================================================
# STEP 1: Add g_pack_test_total_ms declaration
# ============================================================
# Find: "static volatile uint32_t             g_pack_test_seg_ms;"
# Add after it: "static volatile uint32_t             g_pack_test_total_ms;"
lines_new = []
added_total_ms = False
for i, line in enumerate(lines):
    lines_new.append(line)
    if not added_total_ms and 'g_pack_test_seg_ms' in line and 'g_pack_test_total_ms' not in line:
        indent = len(line) - len(line.lstrip())
        lines_new.append(' ' * indent + 'static volatile uint32_t             g_pack_test_total_ms; /* 打包阶段绝对累计时间 */\n')
        added_total_ms = True
        print(f"Added g_pack_test_total_ms after line {i+1}")
lines = lines_new

# ============================================================
# STEP 2: Modify PACK_TEST trigger to use new state machine
# ============================================================
# Find the PACK_TEST trigger block and replace
for i in range(len(lines)):
    if 'g_protocol_pack_test_request' in lines[i] and 'request == 0' not in lines[i]:
        # Look for the block
        for j in range(i, min(i+15, len(lines))):
            if "g_pack_test_state = PACK_TEST_IMPELLER_ON" in lines[j]:
                # Found it - replace with new version
                # Find indent
                indent = '        '  # 8 spaces
                lines[j] = indent + 'g_new_proc_state = PROC_NEW_PACK_IMPELLER;\n'
                # Next line
                lines[j+1] = indent + 'g_new_proc_seg_ms = 0u;\n'
                # Next line: keep PackTest_StopAll
                lines[j-1] = indent + 'PackTest_StopAll();\n'
                print(f"Replaced PACK_TEST trigger at line {j+1}")
                break
        break

# ============================================================
# STEP 3: Delete old partition H boot auto-run code
# ============================================================
# Find: "s_app_tick_armed = 1u;" followed by partition H block
# Lines 263-274 approximately
for i in range(len(lines)):
    if 's_app_tick_armed = 1u;' in lines[i].strip():
        # Check if next ~15 lines contain "g_pack_test_state = PACK_TEST_IMPELLER_ON"
        found_h = False
        end_h = i + 1
        for j in range(i+1, min(i+20, len(lines))):
            if 'g_pack_test_state = PACK_TEST_IMPELLER_ON' in lines[j]:
                found_h = True
                end_h = j + 3  # include the usart1_tx_start_isr line too
            if found_h and '/* 确保 PA9/PA10' in lines[j]:
                # Delete from i+1 to j-1
                del lines[i+1:j]
                print(f"Deleted partition H boot auto-run (lines {i+2} to {j})")
                break
        break

# ============================================================
# STEP 4: Replace old state machine with new one
# ============================================================
# Find the start: "/* =========" followed by "【分区 D】新工艺流程主状态机"
# Find the end: "【分区 E】超声波检测计数"
start_idx = None
for i in range(len(lines)):
    if '【分区 D】新工艺流程主状态机' in lines[i]:
        # Find the /* ===== line before it
        for j in range(i-1, -1, -1):
            if '/* =' in lines[j]:
                start_idx = j
                break
        break

end_idx = None
for i in range(len(lines)):
    if '【分区 E】超声波检测计数' in lines[i]:
        end_idx = i
        break

if start_idx is None or end_idx is None:
    print(f"ERROR: Could not find state machine block. start={start_idx}, end={end_idx}")
else:
    print(f"State machine block: lines {start_idx+1}-{end_idx+1} (to delete)")

    # New state machine code (must maintain brace balance)
    new_block = r'''    /* ========================================================================
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
        static uint8_t  s_pack_b7_was_low;
        static uint8_t  s_pack_recovered;

        /* IR 红外计数：遮挡触发 + 1s 防抖 */
        static uint8_t  s_ir_cnt = 0u;
        static uint8_t  s_ir_triggered = 0u;
        static uint32_t s_ir_trigger_tick = 0u;

        /* IR 低电平（遮挡）触发计数 */
        if (IR_COUNTER_LOW() && !s_ir_triggered) {
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

        switch (st) {

        /* ------------------------------------------------------------------
         * PROC_NEW_IDLE：等待 0x9A 启动命令
         * ------------------------------------------------------------------ */
        case PROC_NEW_IDLE:
            break;

        /* ------------------------------------------------------------------
         * PROC_NEW_BELT_RUN：同步带 CW 运转，红外计数
         *   - 同步带 CW 运转
         *   - 超声距离 < 阈值 → 进入 CUT_RUN
         * ------------------------------------------------------------------ */
        case PROC_NEW_BELT_RUN: {
            static uint32_t s_us_poll_tick;
            static uint8_t  s_us_triggered_this;

            if (seg == 0u) {
                Stepper_SyncRun_Motor1Reversed(DFLT_BELT_SPEED_HZ, STEPPER_DIR_CW);
                s_us_poll_tick = now_tick;
                s_us_triggered_this = 0u;
                UART1_PrintfNB("[NEW] BELT_RUN start\r\n");
            }

            /* 每 60ms 读取一次超声距离 */
            if ((uint32_t)(now_tick - s_us_poll_tick) >= 60U) {
                s_us_poll_tick = now_tick;
                uint16_t dist = New_ReadUltrasonicDistance();

                /* 超声 < 阈值（首端触发）→ 进入切割 */
                if (dist != 0xFFFF && dist < 50 && !s_us_triggered_this) {
                    s_us_triggered_this = 1u;
                    g_new_proc_state = PROC_NEW_CUT_RUN;
                    g_new_proc_seg_ms = 0u;
                    UART1_PrintfNB("[NEW] US trig dist=%d, -> CUT_RUN\r\n", dist);
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
         *   - PC9 高电平驱动切割电机，同步带继续 CW
         *   - 1.5s 停止切割，8s 后同步带停
         *   - IR计数<3 → 返回 BELT_RUN；IR计数≥3 → 进入打包
         * ------------------------------------------------------------------ */
        case PROC_NEW_CUT_RUN: {
            if (seg == 0u) {
                GPIOC->BSRR = GPIO_Pin_9;
                Stepper_SyncRun_Motor1Reversed(DFLT_BELT_SPEED_HZ, STEPPER_DIR_CW);
                UART1_PrintfNB("[NEW] CUT_RUN start\r\n");
            }

            seg++;

            /* 1.5s 停止切割电机，同步带继续 */
            if (seg == 1500u) {
                GPIOC->BRR = GPIO_Pin_9;
                UART1_PrintfNB("[NEW] Cutter stop\r\n");
            }

            /* 8s 后：停止同步带，判断 IR 计数 */
            if (seg >= 8000u) {
                Stepper_SyncStop();

                if (s_ir_cnt >= 3u) {
                    UART1_PrintfNB("[NEW] IR cnt=%u, enter PACK flow\r\n", s_ir_cnt);
                    s_pack_start_tick = now_tick;
                    s_pack_m4_started = 0u;
                    s_pack_recovered = 0u;
                    g_pack_test_total_ms = 0u;
                    g_new_proc_state = PROC_NEW_PACK_IMPELLER;
                } else {
                    UART1_PrintfNB("[NEW] IR cnt=%u (<3), back to BELT_RUN\r\n", s_ir_cnt);
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
         *   - B7 低电平 → M4 启动
         *   - 总时长 20400ms 上限 → STOP
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
                UART1_PrintfNB("[NEW] PACK_IMPELLER start\r\n");
            }
            PackTest_ImpellerKeepalive();

            /* B7 低电平 → M4 启动 */
            if (PACK_B7_LOW() && !s_pack_m4_started) {
                s_pack_m4_started = 1u;
                s_pack_m4_start_tick = now_tick;
                M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);
                g_new_proc_state = PROC_NEW_PACK_M4;
                g_new_proc_seg_ms = 0u;
                UART1_PrintfNB("[NEW] -> PACK_M4\r\n");
                break;
            }

            /* 上限 20400ms */
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
         *   - 2200ms → 进入 ROD_ON_1
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_M4: {
            M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);

            if ((uint32_t)(now_tick - s_pack_m4_start_tick) >= (PACK_TEST_ROD_ON_MS - 1000u)) {
                s_pack_m4_started = 0u;
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
         * PROC_NEW_PACK_ROD_ON_1：膜杆 2/7 第一节（800ms）
         *   - M3/M4/叶轮全停，膜杆启动
         *   - 800ms 后进入 ROD_OFF
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_ROD_ON_1: {
            static uint8_t s_rod_armed;
            static uint8_t s_entered;

            if (!s_entered) {
                s_entered = 1u;
                s_rod_armed = 0u;
                UART1_PrintfNB("[NEW] PACK_ROD_ON_1 entry\r\n");

                M3_Stop();
                M4_Stop();
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

            if (seg >= (PACK_TEST_ROD_ON_MS + PACK_TEST_ROD_LEG_MS)) {
                s_entered = 0u;
                if (s_rod_armed) {
                    PackTest_RodWheelStopBoth();
                    s_rod_armed = 0u;
                }
                s_pack_b7_was_low = PACK_B7_LOW() ? 1u : 0u;
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
         * PROC_NEW_PACK_ROD_OFF：M3+M4+叶轮恢复
         *   - 4000ms 恢复 M3+M4+叶轮
         *   - B7 高电平 → 进入 ROD_ON_2
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_ROD_OFF: {
            if (!s_pack_recovered && g_pack_test_total_ms >= PACK_TEST_M3_RECOVER_MS) {
                s_pack_recovered = 1u;
                ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID1, 1u);
                ServoBus_SetTorqueSwitch(M34_SCS_WHEEL_ID2, 1u);
                ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID1, (int16_t)2, M34_SCS_WHEEL_ACC);
                ServoBus_STS_WriteSpeedAcc(M34_SCS_WHEEL_ID2, (int16_t)(-130), M34_SCS_WHEEL_ACC);
                ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID1, (int16_t)2);
                ServoBus_SetWheelSpeedId(M34_SCS_WHEEL_ID2, (int16_t)(-130));
                M3_Run(PACK_TEST_M3_SPEED_HZ, STEPPER_DIR_CW);
                M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);
                UART1_PrintfNB("[NEW] PACK recovery at %u\r\n", (unsigned)g_pack_test_total_ms);
            }

            if (s_pack_recovered) {
                PackTest_ImpellerKeepalive();
                M3_Run(PACK_TEST_M3_SPEED_HZ, STEPPER_DIR_CW);
                M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);
            }

            if (s_pack_b7_was_low && PACK_B7_HIGH()) {
                g_new_proc_state = PROC_NEW_PACK_ROD_ON_2;
                g_new_proc_seg_ms = 0u;
                UART1_PrintfNB("[NEW] -> PACK_ROD_ON_2\r\n");
                break;
            }
            s_pack_b7_was_low = PACK_B7_LOW() ? 1u : 0u;

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
         * PROC_NEW_PACK_ROD_ON_2：膜杆 2/7 第二节
         *   - M3/M4/叶轮停止，膜杆启动
         *   - B7 高电平后 1950ms → 全部停止
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_ROD_ON_2: {
            static uint8_t s_rod_armed;
            static uint8_t s_entered;
            static uint8_t s_b7_high_tick;
            static uint32_t s_b7_start_tick;

            if (!s_entered) {
                s_entered = 1u;
                s_rod_armed = 0u;
                s_b7_high_tick = 0u;
                UART1_PrintfNB("[NEW] PACK_ROD_ON_2 entry\r\n");

                M3_Stop();
                M4_Stop();
                M34_ScsBus_StopWheelImpellers();

                int8_t d2 = PackTest_RodDirBetween(PACK_TEST_ROD_ID2_PULL, PACK_TEST_ROD_ID2_PUSH);
                int8_t d7 = PackTest_RodDirBetween(PACK_TEST_ROD_ID7_PULL, PACK_TEST_ROD_ID7_PUSH);
                int16_t sp2 = (d2 >= 0) ? (int16_t)PACK_TEST_ROD_SPEED_ID2 : (int16_t)(-((int16_t)PACK_TEST_ROD_SPEED_ID2));
                int16_t sp7 = (d7 >= 0) ? (int16_t)PACK_TEST_ROD_SPEED_ID7 : (int16_t)(-((int16_t)PACK_TEST_ROD_SPEED_ID7));

                if (d2 == 0 && d7 == 0) {
                    UART1_PrintfNB("[NEW] ROD_ON_2 aligned, skip\r\n");
                } else {
                    PackTest_RodWheelRunBoth(sp2, sp7);
                    s_rod_armed = 1u;
                }
            }

            if (PACK_B7_HIGH()) {
                if (s_b7_high_tick == 0u) {
                    s_b7_high_tick = 1u;
                    s_b7_start_tick = now_tick;
                    UART1_PrintfNB("[NEW] B7_HIGH at %u, counting 1950ms...\r\n", (unsigned)g_pack_test_total_ms);
                }
                if ((uint32_t)(now_tick - s_b7_start_tick) >= 1950u) {
                    s_entered = 0u;
                    if (s_rod_armed) {
                        PackTest_RodWheelStopBoth();
                        s_rod_armed = 0u;
                    }
                    g_new_proc_state = PROC_NEW_PACK_STOP;
                    g_new_proc_seg_ms = 0u;
                    UART1_PrintfNB("[NEW] -> PACK_STOP\r\n");
                    break;
                }
            } else {
                s_b7_high_tick = 0u;
            }

            seg++;
            g_new_proc_seg_ms = seg;
            break;
        }

        /* ------------------------------------------------------------------
         * PROC_NEW_PACK_STOP：全部停止，返回 IDLE
         * ------------------------------------------------------------------ */
        case PROC_NEW_PACK_STOP: {
            PackTest_StopAll();
            UART1_PrintfNB("[NEW] PACK done, back to IDLE\r\n");

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

        /* 计算打包阶段绝对累计时间 */
        if (st >= PROC_NEW_PACK_IMPELLER && st <= PROC_NEW_PACK_STOP) {
            if (st == PROC_NEW_PACK_IMPELLER) {
                g_pack_test_total_ms = now_tick - s_pack_start_tick;
            }
        }
    }

'''

    # Rebuild: keep lines 0 to start_idx-1, add new_block, then from end_idx
    lines = lines[:start_idx] + [new_block] + lines[end_idx:]
    print(f"Replaced state machine: {start_idx+1}-{end_idx+1} deleted, new block inserted")

# ============================================================
# STEP 5: Delete old partition E ultrasonic counting code
# ============================================================
# Find: "【分区 E】超声波检测计数" and delete until the next section "【分区 F】USART3"
e_start = None
e_end = None
for i in range(len(lines)):
    if '【分区 E】超声波检测计数' in lines[i]:
        e_start = i
    if e_start is not None and '【分区 F】USART3 0x9A' in lines[i]:
        e_end = i
        break

if e_start is not None and e_end is not None:
    del lines[e_start:e_end]
    print(f"Deleted partition E: lines {e_start+1}-{e_end} ({e_end-e_start} lines)")
else:
    print(f"WARNING: Partition E not found properly. start={e_start}, end={e_end}")

# ============================================================
# STEP 6: Delete old partition H state machine
# ============================================================
# Find: "【分区 H】打包测试状态机" comment block and its code block
h_start = None
h_end = None
for i in range(len(lines)):
    if '【分区 H】打包测试状态机' in lines[i] and '【分区 H】打包测试状态机' in lines[i]:
        # Make sure it's the one in the BSP_Tick hook, not the one in the global vars
        # Find the /* === line before it
        for j in range(i-1, -1, -1):
            if '/* =' in lines[j]:
                h_start = j
                break
        break

if h_start is not None:
    # Find the closing brace of this block
    # Look for the "/* ===" line before "【新工艺流程辅助函数】"
    brace_count = 0
    for i in range(h_start, len(lines)):
        for ch in lines[i]:
            if ch == '{': brace_count += 1
            elif ch == '}': brace_count -= 1
        if brace_count == 0 and i > h_start:
            h_end = i + 1
            break

    if h_end is not None:
        del lines[h_start:h_end]
        print(f"Deleted partition H state machine: lines {h_start+1}-{h_end} ({h_end-h_start} lines)")
    else:
        print(f"WARNING: Could not find end of partition H. brace_count at end: {brace_count}")
else:
    print("WARNING: Could not find partition H start")

# ============================================================
# Verify brace balance
# ============================================================
in_comment = False
depth = 0
for i, line in enumerate(lines):
    j = 0
    while j < len(line):
        c1 = line[j:j+2]
        if not in_comment and c1 == '/*':
            in_comment = True
            j += 2
            continue
        if in_comment and c1 == '*/':
            in_comment = False
            j += 2
            continue
        if not in_comment:
            if line[j] == '{':
                depth += 1
            elif line[j] == '}':
                depth -= 1
        j += 1

print(f"\nFinal: {len(lines)} lines, brace depth={depth} (should be 0)")

# ============================================================
# Write output
# ============================================================
with open(main_path, 'w', encoding='utf-8') as f:
    f.writelines(lines)

print("Written to main.c")
