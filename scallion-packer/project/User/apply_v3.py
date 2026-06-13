# -*- coding: utf-8 -*-
import sys
sys.stdout.reconfigure(encoding='utf-8')

# Chinese text as bytes for reliable matching
MARKER_D = b'\xe3\x80\x90\xe5\x88\x86\xe5\x8c\xba D\xe3\x80\x91'  # 【分区 D】
MARKER_D2 = b'\xe6\x96\xb0\xe5\xb7\xa5\xe8\x89\xba\xe6\xb5\x81\xe7\xa8\x8b\xe4\xb8\xbb\xe7\x8a\xb6\xe6\x80\x81\xe6\x9c\xba'  # 新工艺流程主状态机
MARKER_E = b'\xe3\x80\x90\xe5\x88\x86\xe5\x8c\xba E\xe3\x80\x91\xe8\xb6\x85\xe5\xa3\xb0\xe6\xb3\xa2'  # 【分区 E】超声波
MARKER_E2 = b'\xe6\xa3\x80\xe6\xb5\x8b\xe8\xae\xa1\xe6\x95\xb0'  # 检测计数
MARKER_F = b'\xe3\x80\x90\xe5\x88\x86\xe5\x8c\xba F\xe3\x80\x91USART3 0x9A'  # 【分区 F】USART3 0x9A
MARKER_H = b'\xe3\x80\x90\xe5\x88\x86\xe5\x8c\xba H\xe3\x80\x91'  # 【分区 H】
MARKER_H2 = b'\xe6\x89\x93\xe5\x8c\x85\xe6\xb5\x8b\xe8\xaf\x95'  # 打包测试
SEPARATOR = b'/* ==='  # === separator

backup_path = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00 - 副本 (2)\project\User\main.c"
main_path = r"C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\project\User\main.c"

with open(backup_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

print(f"Backup: {len(lines)} lines")

# STEP 1: Add g_pack_test_total_ms
for i, line in enumerate(lines):
    if 'g_pack_test_seg_ms' in line and 'g_pack_test_total_ms' not in line:
        indent = len(line) - len(line.lstrip())
        new_line = ' ' * indent + 'static volatile uint32_t             g_pack_test_total_ms; /* \u6253\u5305\u9636\u6bb5\u7edd\u5bf9\u7d2f\u8ba1\u65f6\u95f4 */\n'
        lines.insert(i + 1, new_line)
        print(f"STEP 1: Added g_pack_test_total_ms after line {i+1}")
        break

# STEP 2: Replace PACK_TEST trigger
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

# STEP 3: Delete partition H boot auto-run
for i in range(len(lines)):
    if 's_app_tick_armed = 1u;' in lines[i].strip():
        for j in range(i + 1, min(i + 20, len(lines))):
            if '\u786e\u4fdd PA9/PA10' in lines[j]:  # 确保 PA9/PA10
                del lines[i+1:j]
                print(f"STEP 3: Deleted partition H boot (lines {i+2} to {j})")
                break
        break

# STEP 4: Find and replace state machine block
# Read as bytes for reliable pattern matching
with open(backup_path, 'rb') as f:
    raw = f.read()

# Convert to list of lines (bytes)
raw_lines = raw.split(b'\n')

# Find d_start: the /* === line before 【分区 D】新工艺流程主状态机
d_line = None
for i, line in enumerate(raw_lines):
    if MARKER_D in line and MARKER_D2 in line:
        d_line = i
        break

if d_line is None:
    print("ERROR: Could not find 【分区 D】 marker")
    sys.exit(1)

d_start = None
for i in range(d_line - 1, -1, -1):
    if SEPARATOR in raw_lines[i]:
        d_start = i
        break

# Find e_start: 【分区 E】超声波检测计数
e_start = None
for i, line in enumerate(raw_lines):
    if MARKER_E in line and MARKER_E2 in line:
        e_start = i
        break

if d_start is None or e_start is None:
    print(f"ERROR: d_start={d_start}, e_start={e_start}")
    sys.exit(1)

print(f"STEP 4: State machine block: lines {d_start+1}-{e_start+1}")

# New state machine code (using standard strings, not r-prefixed to avoid quote issues)
new_block = """\
    /* ========================================================================
     * \u300a\u5206\u533a F\u300b\u5207\u5272+\u6253\u5305\u8054\u5408\u72b6\u6001\u673a
     *
     * \u89e6\u53d1\uff1aUSART3 \u6536\u5230 0x9A \u547d\u4ee4\uff08\u6216\u8c03\u8bd5\u4e32\u53e3\u547d\u4ee4\uff09
     *
     *   IDLE \u2500\u2500(0x9A)\u2500\u2500\u2192 BELT_RUN \u2500\u2500(\u8d85\u58f0<\u9608\u503c)\u2500\u2500\u2192 CUT_RUN
     *     \u2191                   \u2191                        |
     *     \u2502                  IR\u8ba1\u6570<3              IR\u8ba1\u6570\u22653
     *     \u2502                    \u2502                    \u2193
     *     \u2514\u2500\u2500\u2500\u2500PACK_STOP\u2190\u2500\u2500\u2500PACK_ROD_ON_2\u2190\u2500\u2500\u2500PACK_IMPELLER
     *                                       |\u4e0bB7   |\u9ad8B7
     *                                  PACK_M4     PACK_ROD_ON_1
     *
     *=======================================================================*/
    {
        Process_New_State_TypeDef st = g_new_proc_state;
        uint32_t                 seg = g_new_proc_seg_ms;
        uint32_t                 now_tick = BSP_GetTickMs();

        /* \u6253\u5305\u9636\u6bb5\u7edd\u5bf9\u7d2f\u8ba1\u65f6\u95f4\uff08\u4ec5\u6253\u5305\u9636\u6bb5\u4f7f\u7528\uff09 */
        static uint32_t s_pack_start_tick;
        static uint32_t s_pack_m4_start_tick;
        static uint8_t  s_pack_m4_started;
        static uint8_t  s_pack_b7_was_low;
        static uint8_t  s_pack_recovered;

        /* IR \u7ea2\u5916\u8ba1\u6570\uff1a\u906e\u6321\u89e6\u53d1 + 1s \u9632\u6296 */
        static uint8_t  s_ir_cnt = 0u;
        static uint8_t  s_ir_triggered = 0u;
        static uint32_t s_ir_trigger_tick = 0u;

        /* IR \u4f4e\u7535\u5e73\uff08\u906e\u6321\uff09\u89e6\u53d1\u8ba1\u6570 */
        if (IR_COUNTER_LOW() && !s_ir_triggered) {
            s_ir_cnt++;
            s_ir_triggered = 1u;
            s_ir_trigger_tick = now_tick;
            UART1_PrintfNB("[IR_CNT] +1 cnt=%u\\r\\n", (unsigned)s_ir_cnt);
        }
        /* IR \u9ad8\u7535\u5e73\u540e\u7b49\u5f85 1s \u624d\u5141\u8bb8\u4e0b\u6b21\u8ba1\u6570 */
        if (s_ir_triggered && !IR_COUNTER_LOW()) {
            if ((uint32_t)(now_tick - s_ir_trigger_tick) >= 1000u) {
                s_ir_triggered = 0u;
            }
        }

        switch (st) {

        /* ------------------------------------------------------------------
         * PROC_NEW_IDLE\uff1a\u7b49\u5f85 0x9A \u542f\u52a8\u547d\u4ee4
         * ------------------------------------------------------------------ */
        case PROC_NEW_IDLE:
            break;

        /* ------------------------------------------------------------------
         * PROC_NEW_BELT_RUN\uff1a\u540c\u6b65\u5e26 CW \u8fd0\u8f6c\uff0c\u7ea2\u5916\u8ba1\u6570
         *   - \u540c\u6b65\u5e26 CW \u8fd0\u8f6c
         *   - \u8d85\u58f0\u8ddd\u79bb < \u9608\u503c \u2192 \u8fdb\u5165 CUT_RUN
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

            /* \u6bcf 60ms \u8bfb\u53d6\u4e00\u6b21\u8d85\u58f0\u8ddd\u79bb */
            if ((uint32_t)(now_tick - s_us_poll_tick) >= 60U) {
                s_us_poll_tick = now_tick;
                uint16_t dist = New_ReadUltrasonicDistance();

                /* \u8d85\u58f0 < \u9608\u503c\uff08\u9996\u7aef\u89e6\u53d1\uff09\u2192 \u8fdb\u5165\u5207\u5272 */
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
         * PROC_NEW_CUT_RUN\uff1a\u5207\u5272\u7535\u673a 1.5s
         *   - PC9 \u9ad8\u7535\u5e73\u9a71\u52a8\u5207\u5272\u7535\u673a\uff0c\u540c\u6b65\u5e26\u7ee7\u7eed CW
         *   - 1.5s \u505c\u6b62\u5207\u5272\uff0c8s \u540e\u540c\u6b65\u5e26\u505c
         *   - IR\u8ba1\u6570<3 \u2192 \u8fd4\u56de BELT_RUN\uff1bIR\u8ba1\u6570\u22653 \u2192 \u8fdb\u5165\u6253\u5305
         * ------------------------------------------------------------------ */
        case PROC_NEW_CUT_RUN: {
            if (seg == 0u) {
                GPIOC->BSRR = GPIO_Pin_9;
                Stepper_SyncRun_Motor1Reversed(DFLT_BELT_SPEED_HZ, STEPPER_DIR_CW);
                UART1_PrintfNB("[NEW] CUT_RUN start\\r\\n");
            }

            seg++;

            /* 1.5s \u505c\u6b62\u5207\u5272\u7535\u673a\uff0c\u540c\u6b65\u5e26\u7ee7\u7eed */
            if (seg == 1500u) {
                GPIOC->BRR = GPIO_Pin_9;
                UART1_PrintfNB("[NEW] Cutter stop\\r\\n");
            }

            /* 8s \u540e\uff1a\u505c\u6b62\u540c\u6b65\u5e26\uff0c\u5224\u65ad IR \u8ba1\u6570 */
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
         * PROC_NEW_PACK_IMPELLER\uff1aM3 + \u53f6\u8f6e3/8 \u542f\u52a8
         *   - B7 \u4f4e\u7535\u5e73 \u2192 M4 \u542f\u52a8
         *   - \u603b\u65f6\u957f 20400ms \u4e0a\u9650 \u2192 STOP
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

            /* B7 \u4f4e\u7535\u5e73 \u2192 M4 \u542f\u52a8 */
            if (PACK_B7_LOW() && !s_pack_m4_started) {
                s_pack_m4_started = 1u;
                s_pack_m4_start_tick = now_tick;
                M4_Run(PACK_TEST_M4_SPEED_HZ, STEPPER_DIR_CW);
                g_new_proc_state = PROC_NEW_PACK_M4;
                g_new_proc_seg_ms = 0u;
                UART1_PrintfNB("[NEW] -> PACK_M4\\r\\n");
                break;
            }

            /* \u4e0a\u9650 20400ms */
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
         * PROC_NEW_PACK_M4\uff1aM4 \u8fd0\u884c\u4e2d\uff08M3+\u53f6\u8f6e\u7ee7\u7eed\uff09
         *   - 2200ms \u2192 \u8fdb\u5165 ROD_ON_1
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
         * PROC_NEW_PACK_ROD_ON_1\uff1a\u819c\u67f1 2/7 \u7b2c\u4e00\u8282\uff08800ms\uff09
         *   - M3/M4/\u53f6\u8f6e\u5168\u505c\uff0c\u819c\u67f1\u542f\u52a8
         *   - 800ms \u540e\u8fdb\u5165 ROD_OFF
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
         * PROC_NEW_PACK_ROD_OFF\uff1aM3+M4+\u53f6\u8f6e\u6062\u590d
         *   - 4000ms \u6062\u590d M3+M4+\u53f6\u8f6e
         *   - B7 \u9ad8\u7535\u5e73 \u2192 \u8fdb\u5165 ROD_ON_2
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
         * PROC_NEW_PACK_ROD_ON_2\uff1a\u819c\u67f1 2/7 \u7b2c\u4e8c\u8282
         *   - M3/M4/\u53f6\u8f6e\u505c\u6b62\uff0c\u819c\u67f1\u542f\u52a8
         *   - B7 \u9ad8\u7535\u5e73\u540e 1950ms \u2192 \u5168\u90e8\u505c\u6b62
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
         * PROC_NEW_PACK_STOP\uff1a\u5168\u90e8\u505c\u6b62\uff0c\u8fd4\u56de IDLE
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

        /* \u8ba1\u7b97\u6253\u5305\u9636\u6bb5\u7edd\u5bf9\u7d2f\u8ba1\u65f6\u95f4 */
        if (st >= PROC_NEW_PACK_IMPELLER && st <= PROC_NEW_PACK_STOP) {
            if (st == PROC_NEW_PACK_IMPELLER) {
                g_pack_test_total_ms = now_tick - s_pack_start_tick;
            }
        }
    }

"""

# Rebuild: use lines (text), convert raw_lines indexes to text indexes
# Lines 1-1596 in text = lines 0-1595 in list
# raw_lines index d_start -> text line d_start+1

# Convert: text_lines[i-1] corresponds to raw_lines[i] (since we split on \n)
# BUT: text_lines has \n removed, raw_lines has no \n

# Actually simpler: use text line numbers
# d_start (raw) -> d_start+1 (text, 1-indexed)
# But we need to find them in text lines

# Re-find in text lines
text_d_line = None
for i, line in enumerate(lines):
    if '\u300a\u5206\u533a D\u300b' in line and '\u65b0\u5de5\u827a\u6d41\u7a0b' in line:  # 【分区 D】新工艺流程
        text_d_line = i
        break

if text_d_line is None:
    print("ERROR: Could not find 【分区 D】 in text")
    sys.exit(1)

text_d_start = None
for i in range(text_d_line - 1, -1, -1):
    if '/* ===' in lines[i] and '===' in lines[i]:
        text_d_start = i
        break

# Find e_start text
text_e_line = None
for i, line in enumerate(lines):
    if '\u300a\u5206\u533a E\u300b\u8d85\u58f0\u6ce2' in line and '\u68c0\u6d4b\u8ba1\u6570' in line:  # 【分区 E】超声波检测计数
        text_e_line = i
        break

print(f"State machine: text_d_start={text_d_start+1 if text_d_start else 'None'}, text_e_line={text_e_line+1 if text_e_line else 'None'}")

# Replace lines[text_d_start:text_e_line] with new_block
# Convert new_block to have proper line endings (matching original file)
new_block_fixed = new_block.replace('\n', '\r\n')

lines = lines[:text_d_start] + [new_block_fixed] + lines[text_e_line:]
print(f"After replacement: {len(lines)} lines")

# STEP 5: Delete old partition E (now shifted)
# Find 【分区 E】超声波检测计数 in the new lines
for i, line in enumerate(lines):
    if '\u300a\u5206\u533a E\u300b\u8d85\u58f0\u6ce2' in line and '\u68c0\u6d4b\u8ba1\u6570' in line:
        text_e2_start = i
        for j in range(i, len(lines)):
            if '\u300a\u5206\u533a F\u300bUSART3 0x9A' in lines[j]:  # 【分区 F】USART3 0x9A
                del lines[text_e2_start:j]
                print(f"STEP 5: Deleted partition E (lines {text_e2_start+1} to {j})")
                break
        break

# STEP 6: Delete old partition H state machine
# Find 【分区 H】打包测试状态机
for i, line in enumerate(lines):
    if '\u300a\u5206\u533a H\u300b\u6253\u5305\u6d4b\u8bd5' in line:  # 【分区 H】打包测试
        h_start = i
        # Find the /* === line before it
        for j in range(i - 1, -1, -1):
            if '/* ===' in lines[j] and '===' in lines[j]:
                h_start = j
                break
        # Find the closing brace
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

# Verify brace balance
raw_out = ''.join(lines)
ob = raw_out.count('{')
cb = raw_out.count('}')
print(f"\nFinal: {len(lines)} lines")
print(f"Braces: open={ob}, close={cb}, balance={ob-cb}")

# Write output
with open(main_path, 'w', encoding='utf-8') as f:
    f.writelines(lines)

print("Written to main.c")
