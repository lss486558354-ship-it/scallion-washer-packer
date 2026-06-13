# -*- coding: utf-8 -*-
# Rewrite the new_block string using a different approach - write to temp file first
# then reconstruct
import sys
sys.stdout.reconfigure(encoding='utf-8')

# New state machine code as a separate string
NEW_BLOCK = r'''    /* ========================================================================
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
            UART1_PrintfNB("[IR_CNT] +1 cnt=%u\\r\\n", (unsigned)s_ir_cnt);
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
                UART1_PrintfNB("[NEW] BELT_RUN start\\r\\n");
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
         *   - PC9 高电平驱动切割电机，同步带继续 CW
         *   - 1.5s 停止切割，8s 后同步带停
         *   - IR计数<3 → 返回 BELT_RUN；IR计数≥3 → 进入打包
         * ------------------------------------------------------------------ */
        case PROC_NEW_CUT_RUN: {
            if (seg == 0u) {
                GPIOC->BSRR = GPIO_Pin_9;
                Stepper_SyncRun_Motor1Reversed(DFLT_BELT_SPEED_HZ, STEPPER_DIR_CW);
                UART1_PrintfNB("[NEW] CUT_RUN start\\r\\n");
            }

            seg++;

            /* 1.5s 停止切割电机，同步带继续 */
            if (seg == 1500u) {
                GPIOC->BRR = GPIO_Pin_9;
                UART1_PrintfNB("[NEW] Cutter stop\\r\\n");
            }

            /* 8s 后：停止同步带，判断 IR 计数 */
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
                UART1_PrintfNB("[NEW] PACK_IMPELLER start\\r\\n");
            }
            PackTest_ImpellerKeepalive();

            /* B7 低电平 → M4 启动 */
            if (PACK_B7_LOW() && !s_pack_m4_started) {
                s_pack_m4_started = 1u;
                s_pack_m4_start_tick = now_tick;
                M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);
                g_new_proc_state = PROC_NEW_PACK_M4;
                g_new_proc_seg_ms = 0u;
                UART1_PrintfNB("[NEW] -> PACK_M4\\r\\n");
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
                UART1_PrintfNB("[NEW] PACK_ROD_ON_1 entry\\r\\n");

                M3_Stop();
                M4_Stop();
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

            if (seg >= (PACK_TEST_ROD_ON_MS + PACK_TEST_ROD_LEG_MS)) {
                s_entered = 0u;
                if (s_rod_armed) {
                    PackTest_RodWheelStopBoth();
                    s_rod_armed = 0u;
                }
                s_pack_b7_was_low = PACK_B7_LOW() ? 1u : 0u;
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
                UART1_PrintfNB("[NEW] PACK recovery at %u\\r\\n", (unsigned)g_pack_test_total_ms);
            }

            if (s_pack_recovered) {
                PackTest_ImpellerKeepalive();
                M3_Run(PACK_TEST_M3_SPEED_HZ, STEPPER_DIR_CW);
                M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);
            }

            if (s_pack_b7_was_low && PACK_B7_HIGH()) {
                g_new_proc_state = PROC_NEW_PACK_ROD_ON_2;
                g_new_proc_seg_ms = 0u;
                UART1_PrintfNB("[NEW] -> PACK_ROD_ON_2\\r\\n");
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
                UART1_PrintfNB("[NEW] PACK_ROD_ON_2 entry\\r\\n");

                M3_Stop();
                M4_Stop();
                M34_ScsBus_StopWheelImpellers();

                int8_t d2 = PackTest_RodDirBetween(PACK_TEST_ROD_ID2_PULL, PACK_TEST_ROD_ID2_PUSH);
                int8_t d7 = PackTest_RodDirBetween(PACK_TEST_ROD_ID7_PULL, PACK_TEST_ROD_ID7_PUSH);
                int16_t sp2 = (d2 >= 0) ? (int16_t)PACK_TEST_ROD_SPEED_ID2 : (int16_t)(-((int16_t)PACK_TEST_ROD_SPEED_ID2));
                int16_t sp7 = (d7 >= 0) ? (int16_t)PACK_TEST_ROD_SPEED_ID7 : (int16_t)(-((int16_t)PACK_TEST_ROD_SPEED_ID7));

                if (d2 == 0 && d7 == 0) {
                    UART1_PrintfNB("[NEW] ROD_ON_2 aligned, skip\\r\\n");
                } else {
                    PackTest_RodWheelRunBoth(sp2, sp7);
                    s_rod_armed = 1u;
                }
            }

            if (PACK_B7_HIGH()) {
                if (s_b7_high_tick == 0u) {
                    s_b7_high_tick = 1u;
                    s_b7_start_tick = now_tick;
                    UART1_PrintfNB("[NEW] B7_HIGH at %u, counting 1950ms...\\r\\n", (unsigned)g_pack_test_total_ms);
                }
                if ((uint32_t)(now_tick - s_b7_start_tick) >= 1950u) {
                    s_entered = 0u;
                    if (s_rod_armed) {
                        PackTest_RodWheelStopBoth();
                        s_rod_armed = 0u;
                    }
                    g_new_proc_state = PROC_NEW_PACK_STOP;
                    g_new_proc_seg_ms = 0u;
                    UART1_PrintfNB("[NEW] -> PACK_STOP\\r\\n");
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

        /* 计算打包阶段绝对累计时间 */
        if (st >= PROC_NEW_PACK_IMPELLER && st <= PROC_NEW_PACK_STOP) {
            if (st == PROC_NEW_PACK_IMPELLER) {
                g_pack_test_total_ms = now_tick - s_pack_start_tick;
            }
        }
    }

'''

# Now read the backup and modify it
backup_path = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00 - 副本 (2)\project\User\main.c"
main_path = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c"

with open(backup_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

print(f"Backup: {len(lines)} lines")

# Verify backup brace balance
raw_backup = open(backup_path, 'rb').read()
ob = raw_backup.count(b'{')
cb = raw_backup.count(b'}')
print(f"Backup: open_braces={ob}, close_braces={cb}")

# ============================================================
# STEP 1: Add g_pack_test_total_ms declaration
# ============================================================
for i, line in enumerate(lines):
    if 'g_pack_test_seg_ms' in line and 'g_pack_test_total_ms' not in line:
        indent = len(line) - len(line.lstrip())
        new_line = ' ' * indent + 'static volatile uint32_t             g_pack_test_total_ms; /* 打包阶段绝对累计时间 */\n'
        lines.insert(i + 1, new_line)
        print(f"STEP 1: Added g_pack_test_total_ms after line {i+1}")
        break

# ============================================================
# STEP 2: Replace PACK_TEST trigger
# ============================================================
for i in range(len(lines)):
    if 'g_protocol_pack_test_request' in lines[i]:
        for j in range(i, min(i + 20, len(lines))):
            if 'g_pack_test_state = PACK_TEST_IMPELLER_ON' in lines[j]:
                indent = '        '
                lines[j] = indent + 'g_new_proc_state = PROC_NEW_PACK_IMPELLER;\n'
                lines[j+1] = indent + 'g_new_proc_seg_ms = 0u;\n'
                lines[j-1] = indent + 'PackTest_StopAll();\n'
                print(f"STEP 2: Replaced PACK_TEST trigger at line {j+1}")
                break
        break

# ============================================================
# STEP 3: Delete old partition H boot auto-run
# ============================================================
for i in range(len(lines)):
    if 's_app_tick_armed = 1u;' in lines[i].strip():
        for j in range(i + 1, min(i + 20, len(lines))):
            if '/* 确保 PA9/PA10' in lines[j]:
                del lines[i+1:j]
                print(f"STEP 3: Deleted partition H boot (lines {i+2} to {j})")
                break
        break

# ============================================================
# STEP 4: Replace state machine
# ============================================================
d_start = None
for i, line in enumerate(lines):
    if '\u300a\u5206\u533a D\u300b\u65b0\u5de5\u827a\u6d41\u7a0b\u4e3b\u72b6\u6001\u673a' in line:  # 【分区 D】新工艺流程主状态机
        for j in range(i - 1, -1, -1):
            if '/* =' in lines[j] and '===' in lines[j]:
                d_start = j
                break
        break

e_start = None
for i, line in enumerate(lines):
    if '\u300a\u5206\u533a E\u300b\u8d85\u58f0\u6ce2\u68c0\u6d4b\u8ba1\u6570' in line:  # 【分区 E】超声波检测计数
        e_start = i
        break

if d_start is None or e_start is None:
    print(f"ERROR: d_start={d_start}, e_start={e_start}")
    sys.exit(1)

print(f"STEP 4: Replacing state machine lines {d_start+1} to {e_start}")
# Replace with NEW_BLOCK string
# Fix \\r\\n to actual \r\n
new_block_fixed = NEW_BLOCK.replace('\\r\\n', '\r\n')
lines = lines[:d_start] + [new_block_fixed] + lines[e_start:]
print(f"After replacement: {len(lines)} lines")

# ============================================================
# STEP 5: Delete old partition E
# ============================================================
for i in range(len(lines)):
    if '\u300a\u5206\u533a E\u300b\u8d85\u58f0\u6ce2\u68c0\u6d4b\u8ba1\u6570' in lines[i]:  # 【分区 E】超声波检测计数
        e_end = i
        for j in range(i, len(lines)):
            if '\u300a\u5206\u533a F\u300bUSART3 0x9A' in lines[j]:  # 【分区 F】USART3 0x9A
                del lines[e_end:j]
                print(f"STEP 5: Deleted partition E (lines {e_end+1} to {j})")
                break
        break

# ============================================================
# STEP 6: Delete old partition H state machine
# ============================================================
for i in range(len(lines)):
    if '\u300a\u5206\u533a H\u300b\u6253\u5305\u6d4b\u8bd5\u72b6\u6001\u673a' in lines[i]:  # 【分区 H】打包测试状态机
        for j in range(i - 1, -1, -1):
            if '/* =' in lines[j] and '===' in lines[j]:
                h_start = j
                break
        else:
            h_start = i
        brace_count = 0
        h_end = None
        for j in range(h_start, len(lines)):
            for ch in lines[j]:
                if ch == '{': brace_count += 1
                elif ch == '}': brace_count -= 1
            if brace_count == 0 and j > h_start:
                h_end = j + 1
                break
        if h_end:
            del lines[h_start:h_end]
            print(f"STEP 6: Deleted partition H state machine (lines {h_start+1} to {h_end})")
        else:
            print(f"WARNING: Partition H end not found (brace_count={brace_count})")
        break

# ============================================================
# Verify brace balance
# ============================================================
raw_out = ''.join(lines)
open_b = raw_out.count('{')
close_b = raw_out.count('}')
print(f"\nFinal: {len(lines)} lines")
print(f"Braces: {{={open_b} }}={close_b} balance={open_b-close_b}")

# ============================================================
# Write output
# ============================================================
with open(main_path, 'w', encoding='utf-8') as f:
    f.writelines(lines)

print("Written to main.c")
