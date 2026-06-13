/**
 ******************************************************************************
 * 通信协议层 - 与Jetson Orin Nano通信（裸机版）
 *
 * 版本：V2.1 (使用对象化驱动框架)
 * 日期：2026-04-12（裸机移植版）
 ******************************************************************************
 */

#include "protocol.h"
#include "device.h"
#include "bsp_usart_obj.h"
#include "bsp_stepper.h"
#include "bsp_dc_motor_obj.h"
#include "bsp_servo_obj.h"
#include "bsp_sensor_obj.h"
#include "process_control.h"
#include <string.h>

/*-----------------------------------------------------------
 * ACK状态码（必须放在使用之前）
 *-----------------------------------------------------------*/
#define ACK_OK     0x00  /* 执行成功 */
#define ACK_PARAM  0x01  /* 参数错误 */
#define ACK_UNSUP  0x02  /* 不支持 */
#define ACK_BUSY   0x03  /* 设备忙 */
#define ACK_IDLE   0x04  /* 设备已停止 */

/*-----------------------------------------------------------
 * 函数前向声明（必须在使用之前）
 *-----------------------------------------------------------*/
static void Protocol_SendACK(uint8_t original_cmd, uint8_t status, uint8_t device_id);

// =============================================================================
// 全局变量定义
// =============================================================================

/* 通信缓冲区（用于Jetson通信） */
CommBuffer_TypeDef comm_buffer = {0};

/* 系统状态全局变量 */
SystemStatus_TypeDef g_system_status = SYSTEM_IDLE;
ErrorCode_TypeDef   g_error_code = ERROR_NONE;

/* 直流电机占空比偏移量（用于语音加减控制） */
int8_t g_dcm_offset[4] = {0};   /* DCM1~DCM3 偏移，范围 -100~+100 */

/* 步进电机速度偏移量（用于语音加减控制） */
int16_t g_stepper_offset[3] = {0};  /* STEPPER1~3 偏移，范围 -20000~+20000 */

/* 传感器数据全局变量 */
SensorData_TypeDef  g_sensor_data = {0};

/* 当前接收帧所属的通信通道（供命令处理函数回传ACK用） */
static CommChannel_TypeDef s_current_channel = CHANNEL_USART2;

/* 急停请求标志：USART3 ISR 中设置，主循环中执行，避免中断内阻塞舵机总线 */
volatile uint8_t g_protocol_emergency_stop_request = 0;
volatile uint8_t g_protocol_process_stop_request = 0;
volatile uint8_t g_protocol_process_pause_request = 0;
volatile uint8_t g_protocol_process_start_request = 0;
volatile uint8_t g_protocol_process_resume_request = 0;
volatile uint8_t g_protocol_pack_test_request = 0;       /* 分区H打包测试请求 */
volatile uint8_t g_protocol_last_cmd = 0;  /* 最后收到的命令（用于调试） */
volatile uint8_t g_protocol_cmd_ready = 0;  /* 命令解析成功标志 */

// =============================================================================
// 协议初始化
// =============================================================================
void Protocol_Init(void)
{
    /* 初始化通信缓冲区 */
    memset(&comm_buffer, 0, sizeof(CommBuffer_TypeDef));

    /* 初始化全局状态 */
    g_system_status = SYSTEM_READY;
    g_error_code = ERROR_NONE;

    /* 发送系统就绪消息 */
    Protocol_SendSystemStatus(CHANNEL_USART2);
}

// =============================================================================
// 发送协议帧（通用函数）- 使用UART对象
// =============================================================================
void Protocol_SendFrame(CommChannel_TypeDef channel, uint8_t cmd, uint8_t* data, uint8_t len)
{
    uint8_t tx_frame[PROTOCOL_MAX_DATA_LEN + 4];
    uint8_t checksum;
    uint8_t i;

    /* 限制数据长度 */
    if (len > PROTOCOL_MAX_DATA_LEN) {
        len = PROTOCOL_MAX_DATA_LEN;
    }

    /* 计算校验和（异或校验，包括帧头） */
    checksum = FRAME_HEAD ^ cmd ^ len;
    for (i = 0; i < len; i++) {
        checksum ^= data[i];
    }

    /* 组装帧：帧头 + 命令 + 长度 + 数据 + 校验 */
    tx_frame[0] = FRAME_HEAD;
    tx_frame[1] = cmd;
    tx_frame[2] = len;
    for (i = 0; i < len; i++) {
        tx_frame[3 + i] = data[i];
    }
    tx_frame[3 + len] = checksum;

    /* 根据通道发送帧 - 使用UART对象 */
    switch(channel) {
        case CHANNEL_USART2:
            for (i = 0; i < (len + 4); i++) {
                UART2_SendByte(tx_frame[i]);
            }
            break;
        case CHANNEL_USART3:
            for (i = 0; i < (len + 4); i++) {
                UART3_SendByte(tx_frame[i]);
            }
            break;
        case CHANNEL_USART1:
        default:
            for (i = 0; i < (len + 4); i++) {
                UART1_SendByte(tx_frame[i]);
            }
            break;
    }
}

// =============================================================================
// 发送传感器数据
// =============================================================================
void Protocol_SendSensorData(CommChannel_TypeDef channel)
{
    uint8_t data[sizeof(SensorData_TypeDef)];
    SensorData_TypeDef* p = &g_sensor_data;

    /* 将传感器数据打包为字节数组 */
    data[0]  = (uint8_t)(p->motor1_position >> 8);
    data[1]  = (uint8_t)(p->motor1_position & 0xFF);
    data[2]  = (uint8_t)(p->motor2_position >> 8);
    data[3]  = (uint8_t)(p->motor2_position & 0xFF);
    data[4]  = (uint8_t)(p->motor3_position >> 8);
    data[5]  = (uint8_t)(p->motor3_position & 0xFF);
    data[6]  = (uint8_t)(p->motor1_speed >> 8);
    data[7]  = (uint8_t)(p->motor1_speed & 0xFF);
    data[8]  = (uint8_t)(p->motor2_speed >> 8);
    data[9]  = (uint8_t)(p->motor2_speed & 0xFF);
    data[10] = (uint8_t)(p->motor3_speed >> 8);
    data[11] = (uint8_t)(p->motor3_speed & 0xFF);
    data[12] = (uint8_t)(p->adc_value);

    Protocol_SendFrame(channel, CMD_SENSOR_DATA, data, 13);
}

// =============================================================================
// 发送系统状态
// =============================================================================
void Protocol_SendSystemStatus(CommChannel_TypeDef channel)
{
    uint8_t data[3];

    data[0] = (uint8_t)g_system_status;
    data[1] = (uint8_t)g_error_code;
    data[2] = 0;

    Protocol_SendFrame(channel, CMD_SYSTEM_STATUS, data, 3);
}

// =============================================================================
// 发送错误报告
// =============================================================================
void Protocol_SendErrorReport(CommChannel_TypeDef channel, ErrorCode_TypeDef error)
{
    uint8_t data[2];

    g_error_code = error;
    g_system_status = SYSTEM_ERROR;

    data[0] = (uint8_t)error;
    data[1] = 0;

    Protocol_SendFrame(channel, CMD_ERROR_REPORT, data, 2);
}

// =============================================================================
// 处理从Jetson接收到的命令 - 使用对象化驱动
// =============================================================================
void Protocol_ProcessCommand(uint8_t cmd, uint8_t* data, uint8_t len)
{
    MotorControl_TypeDef motor_ctrl;

    switch (cmd) {
        case CMD_START_TASK:
            /* 开始任务 */
            g_system_status = SYSTEM_RUNNING;
            Protocol_SendACK(CMD_START_TASK, ACK_OK, (uint8_t)g_system_status);
            break;

        case CMD_STOP_TASK:
            /* 停止所有电机 - 使用快捷宏 */
            M1_Stop();
            M2_Stop();
            M3_Stop();
            g_system_status = SYSTEM_IDLE;
            Protocol_SendACK(CMD_STOP_TASK, ACK_OK, (uint8_t)g_system_status);
            break;

        case CMD_PAUSE_TASK:
            /* 暂停（保持当前位置） */
            M1_Brake();
            M2_Brake();
            M3_Brake();
            g_system_status = SYSTEM_PAUSED;
            Protocol_SendACK(CMD_PAUSE_TASK, ACK_OK, (uint8_t)g_system_status);
            break;

        case CMD_RESUME_TASK:
            /* 恢复运行 */
            g_system_status = SYSTEM_RUNNING;
            Protocol_SendACK(CMD_RESUME_TASK, ACK_OK, (uint8_t)g_system_status);
            break;

        case CMD_SET_MOTOR_SPEED:
            /* 设置电机速度 - 使用快捷宏 */
            if (len >= 3) {
                motor_ctrl.motor_id = data[0];
                motor_ctrl.target_speed = (int16_t)((data[1] << 8) | data[2]);

                if (motor_ctrl.motor_id == 1) {
                    M1_Run((uint16_t)motor_ctrl.target_speed, DIR_FORWARD);
                } else if (motor_ctrl.motor_id == 2) {
                    M2_Run((uint16_t)motor_ctrl.target_speed, DIR_FORWARD);
                } else if (motor_ctrl.motor_id == 3) {
                    M3_Run((uint16_t)motor_ctrl.target_speed, DIR_FORWARD);
                }
            }
            break;

        case CMD_SET_MOTOR_POS:
            /* 设置电机目标位置 */
            if (len >= 4) {
                motor_ctrl.motor_id = data[0];
                motor_ctrl.target_pos = (int16_t)((data[1] << 8) | data[2]);
                motor_ctrl.control_mode = data[3];

                if (motor_ctrl.control_mode == 1) {
                    /* 位置模式 */
                    if (motor_ctrl.motor_id == 1) {
                        M1_Run(1000, DIR_FORWARD);
                    } else if (motor_ctrl.motor_id == 2) {
                        M2_Run(1000, DIR_FORWARD);
                    } else if (motor_ctrl.motor_id == 3) {
                        M3_Run(1000, DIR_FORWARD);
                    }
                }
            }
            break;

        case CMD_SET_SERVO_ANGLE:
            /* 设置舵机角度 - 使用快捷宏 */
            if (len >= 3) {
                uint8_t servo_id = data[0];
                uint16_t angle = (data[1] << 8) | data[2];
                float angle_f = (float)angle / 10.0f;

                if (servo_id == 1) {
                    SERVO[SERVO_1].SetAngle(&SERVO[SERVO_1], angle_f);
                } else if (servo_id == 2) {
                    SERVO[SERVO_2].SetAngle(&SERVO[SERVO_2], angle_f);
                }
            }
            break;

        case CMD_RESET_SYSTEM:
            /* 系统复位 - 停止所有电机 */
            M1_Stop();
            M2_Stop();
            M3_Stop();
            DCM_StopAll();
            g_error_code = ERROR_NONE;
            g_system_status = SYSTEM_READY;
            Protocol_SendACK(CMD_RESET_SYSTEM, ACK_OK, (uint8_t)g_system_status);
            break;

        // =======================================================================
        // 上位机(语音模块)命令处理
        // =======================================================================
        case CMD_DCM_START:
            /* 直流电机启动（绝对值设置）  data:[电机ID][占空比][方向(可选)] */
            if (len >= 2) {
                uint8_t dcm_id = data[0];
                uint8_t duty   = data[1];
                Motor_Direction_TypeDef dir = DIR_FORWARD;
                int8_t idx = (int8_t)(dcm_id - 0x11);
                if (len >= 3 && (data[2] == 0x00 || data[2] == 0x01)) {
                    dir = (data[2] == 0x01) ? DIR_BACKWARD : DIR_FORWARD;
                }
                if (idx >= 0 && idx < 4) {
                    g_dcm_offset[idx] = 0;
                }
                if      (dcm_id == 0x11) { DCM1_Run(duty, dir); Protocol_SendACK(CMD_DCM_START,  ACK_OK, dcm_id); }
                else if (dcm_id == 0x12) { DCM2_Run(duty, dir); Protocol_SendACK(CMD_DCM_START,  ACK_OK, dcm_id); }
                else if (dcm_id == 0x13) { DCM3_Run(duty, dir); Protocol_SendACK(CMD_DCM_START,  ACK_OK, dcm_id); }
                else if (dcm_id == 0x1F) {
                    g_dcm_offset[0] = g_dcm_offset[1] = g_dcm_offset[2] = 0;
                    DCM1_Run(duty, dir);
                    DCM2_Run(duty, dir);
                    DCM3_Run(duty, dir);
                    Protocol_SendACK(CMD_DCM_START, ACK_OK, dcm_id);
                } else {
                    Protocol_SendACK(CMD_DCM_START, ACK_PARAM, dcm_id);
                }
            } else {
                Protocol_SendACK(CMD_DCM_START, ACK_PARAM, 0);
            }
            break;

        case CMD_DCM_STOP:
            /* 直流电机停止  data:[电机ID] */
            if (len >= 1) {
                uint8_t dcm_id = data[0];
                if      (dcm_id == 0x1F) { DCM_StopAll();  Protocol_SendACK(CMD_DCM_STOP, ACK_OK, dcm_id); }
                else if (dcm_id == 0x11) { DCM1_Stop();    Protocol_SendACK(CMD_DCM_STOP, ACK_OK, dcm_id); }
                else if (dcm_id == 0x12) { DCM2_Stop();    Protocol_SendACK(CMD_DCM_STOP, ACK_OK, dcm_id); }
                else if (dcm_id == 0x13) { DCM3_Stop();    Protocol_SendACK(CMD_DCM_STOP, ACK_OK, dcm_id); }
                else                      { Protocol_SendACK(CMD_DCM_STOP, ACK_PARAM, dcm_id); }
            } else {
                Protocol_SendACK(CMD_DCM_STOP, ACK_PARAM, 0);
            }
            break;

        case CMD_STEPPER_START:
            /* 步进电机启动  data:[电机ID][速度H][速度L] */
            if (len >= 3) {
                uint8_t step_id = data[0];
                uint16_t speed  = ((uint16_t)data[1] << 8) | data[2];
                if      (step_id == 0x01) { M1_Run(speed, DIR_FORWARD); Protocol_SendACK(CMD_STEPPER_START, ACK_OK, step_id); }
                else if (step_id == 0x02) { M2_Run(speed, DIR_FORWARD); Protocol_SendACK(CMD_STEPPER_START, ACK_OK, step_id); }
                else if (step_id == 0x03) { M3_Run(speed, DIR_FORWARD); Protocol_SendACK(CMD_STEPPER_START, ACK_OK, step_id); }
                else                      { Protocol_SendACK(CMD_STEPPER_START, ACK_PARAM, step_id); }
            } else {
                Protocol_SendACK(CMD_STEPPER_START, ACK_PARAM, 0);
            }
            break;

        case CMD_STEPPER_STOP:
            /* 步进电机停止  data:[电机ID] */
            if (len >= 1) {
                if      (data[0] == 0x01) { M1_Stop();  Protocol_SendACK(CMD_STEPPER_STOP, ACK_OK, data[0]); }
                else if (data[0] == 0x02) { M2_Stop();  Protocol_SendACK(CMD_STEPPER_STOP, ACK_OK, data[0]); }
                else if (data[0] == 0x03) { M3_Stop();  Protocol_SendACK(CMD_STEPPER_STOP, ACK_OK, data[0]); }
                else                      { Protocol_SendACK(CMD_STEPPER_STOP, ACK_PARAM, data[0]); }
            } else {
                Protocol_SendACK(CMD_STEPPER_STOP, ACK_PARAM, 0);
            }
            break;

        case CMD_SET_SPEED_DIR:
            /* 通用设置转速+方向（绝对值）  data:[电机ID][类型][方向][值H][值L] */
            if (len >= 5) {
                uint8_t  dev_id   = data[0];
                uint8_t  dev_type = data[1];
                uint8_t  dir_byte = data[2];
                uint16_t value    = ((uint16_t)data[3] << 8) | data[4];
                Motor_Direction_TypeDef dir =
                    (dir_byte == 0x01) ? DIR_BACKWARD : DIR_FORWARD;
                int8_t idx;
                uint16_t actual_speed = value;
                uint8_t actual_duty = (uint8_t)value;
                uint8_t status = ACK_OK;

                if (dev_type == 0x01) {
                    idx = (int8_t)(dev_id - 0x01);
                    if (idx >= 0 && idx < 3) g_stepper_offset[idx] = 0;
                    if (idx >= 0 && idx < 3) {
                        int32_t tmp = (int32_t)value + g_stepper_offset[idx];
                        if (tmp < 1) tmp = 1;
                        if (tmp > 20000) tmp = 20000;
                        actual_speed = (uint16_t)tmp;
                    }
                    if      (dev_id == 0x01) { M1_Run(actual_speed, dir); }
                    else if (dev_id == 0x02) { M2_Run(actual_speed, dir); }
                    else if (dev_id == 0x03) { M3_Run(actual_speed, dir); }
                    else                      { status = ACK_PARAM; }
                } else if (dev_type == 0x02) {
                    idx = (int8_t)(dev_id - 0x11);
                    if (idx >= 0 && idx < 4) g_dcm_offset[idx] = 0;
                    if (idx >= 0 && idx < 4) {
                        int32_t tmp = (int32_t)value + g_dcm_offset[idx];
                        if (tmp < 0) tmp = 0;
                        if (tmp > 100) tmp = 100;
                        actual_duty = (uint8_t)tmp;
                    }
                    if      (dev_id == 0x11) DCM1_Run(actual_duty, dir);
                    else if (dev_id == 0x12) DCM2_Run(actual_duty, dir);
                    else if (dev_id == 0x13) DCM3_Run(actual_duty, dir);
                    else if (dev_id == 0x1F) {
                        DCM1_Run(actual_duty, dir);
                        DCM2_Run(actual_duty, dir);
                        DCM3_Run(actual_duty, dir);
                    } else {
                        status = ACK_PARAM;
                    }
                } else {
                    status = ACK_PARAM;
                }
                Protocol_SendACK(CMD_SET_SPEED_DIR, status, dev_id);
            } else {
                Protocol_SendACK(CMD_SET_SPEED_DIR, ACK_PARAM, 0);
            }
            break;

        case CMD_STEPPER_DIR:
            /* 步进电机单独设置方向  data:[电机ID][方向] */
            if (len >= 2) {
                uint8_t status = ACK_OK;
                Stepper_Dir_TypeDef dir =
                    (data[1] == 0x01) ? STEPPER_DIR_CCW : STEPPER_DIR_CW;
                if      (data[0] == 0x01) Stepper_SetDirLevel(STEPPER_1, dir);
                else if (data[0] == 0x02) Stepper_SetDirLevel(STEPPER_2, dir);
                else if (data[0] == 0x03) Stepper_SetDirLevel(STEPPER_3, dir);
                else                      status = ACK_PARAM;
                Protocol_SendACK(CMD_STEPPER_DIR, status, data[0]);
            } else {
                Protocol_SendACK(CMD_STEPPER_DIR, ACK_PARAM, 0);
            }
            break;

        case CMD_DCM_DIR:
            /* 直流电机单独设置方向（不改变速度，下次Run生效）  data:[电机ID][方向] */
            if (len >= 2) {
                uint8_t status = ACK_OK;
                Motor_Direction_TypeDef dir =
                    (data[1] == 0x01) ? DIR_BACKWARD : DIR_FORWARD;
                if      (data[0] == 0x11) DCM[DCM_1].direction = dir;
                else if (data[0] == 0x12) DCM[DCM_2].direction = dir;
                else if (data[0] == 0x13) DCM[DCM_3].direction = dir;
                else                      status = ACK_PARAM;
                Protocol_SendACK(CMD_DCM_DIR, status, data[0]);
            } else {
                Protocol_SendACK(CMD_DCM_DIR, ACK_PARAM, 0);
            }
            break;

        case CMD_OFFSET_ADJUST:
            /* 调整偏移量（加减，不清零）  data:[电机ID][类型][偏移H][偏移L] */
            if (len >= 4) {
                uint8_t  dev_id   = data[0];
                uint8_t  dev_type = data[1];
                int16_t  offset   = (int16_t)((uint16_t)data[2] << 8 | data[3]);
                uint8_t  status   = ACK_OK;

                if (dev_type == 0x01) {
                    int8_t idx = (int8_t)(dev_id - 0x01);
                    if (idx >= 0 && idx < 3) {
                        int32_t tmp = (int32_t)g_stepper_offset[idx] + offset;
                        if (tmp < -20000) tmp = -20000;
                        if (tmp >  20000) tmp =  20000;
                        g_stepper_offset[idx] = (int16_t)tmp;
                    } else { status = ACK_PARAM; }
                } else if (dev_type == 0x02) {
                    int8_t idx = (int8_t)(dev_id - 0x11);
                    if (idx >= 0 && idx < 4) {
                        int16_t tmp = g_dcm_offset[idx] + offset;
                        if (tmp < -100) tmp = -100;
                        if (tmp >  100) tmp =  100;
                        g_dcm_offset[idx] = (int8_t)tmp;
                    } else { status = ACK_PARAM; }
                } else {
                    status = ACK_PARAM;
                }
                Protocol_SendACK(CMD_OFFSET_ADJUST, status, dev_id);
            } else {
                Protocol_SendACK(CMD_OFFSET_ADJUST, ACK_PARAM, 0);
            }
            break;

        case CMD_OFFSET_RESET:
            /* 清零偏移量  data:[电机ID] */
            if (len >= 1) {
                uint8_t dev_id = data[0];
                if (dev_id == 0x1F) {
                    g_dcm_offset[0] = g_dcm_offset[1] = g_dcm_offset[2] = 0;
                    g_stepper_offset[0] = g_stepper_offset[1] = g_stepper_offset[2] = 0;
                    Protocol_SendACK(CMD_OFFSET_RESET, ACK_OK, dev_id);
                } else if (dev_id >= 0x11 && dev_id <= 0x13) {
                    g_dcm_offset[dev_id - 0x11] = 0;
                    Protocol_SendACK(CMD_OFFSET_RESET, ACK_OK, dev_id);
                } else if (dev_id >= 0x01 && dev_id <= 0x03) {
                    g_stepper_offset[dev_id - 0x01] = 0;
                    Protocol_SendACK(CMD_OFFSET_RESET, ACK_OK, dev_id);
                } else {
                    Protocol_SendACK(CMD_OFFSET_RESET, ACK_PARAM, dev_id);
                }
            } else {
                Protocol_SendACK(CMD_OFFSET_RESET, ACK_PARAM, 0);
            }
            break;

        case CMD_ALL_STOP:
            /* 全部急停：仅设置标志，由主循环执行，避免在USART3中断内阻塞舵机总线 */
            g_protocol_emergency_stop_request = 1;
            g_system_status = SYSTEM_IDLE;
            Protocol_SendACK(CMD_ALL_STOP, ACK_OK, 0x00);
            break;

        // =======================================================================
        // 工艺流程控制命令
        // =======================================================================
        case CMD_PROCESS_START:
            /* 开始工艺流程：设置标志由主循环执行，避免中断内阻塞 */
            g_protocol_process_start_request = 1;
            g_protocol_last_cmd = CMD_PROCESS_START;
            /* 立即打印调试信息，不依赖主循环 */
            g_protocol_cmd_ready = 1;
            break;

        case CMD_PROCESS_STOP:
            /* 停止工艺流程：设置标志由主循环执行，避免中断内阻塞舵机总线 */
            g_protocol_process_stop_request = 1;
            g_system_status = SYSTEM_IDLE;
            g_protocol_last_cmd = CMD_PROCESS_STOP;
            g_protocol_cmd_ready = 1;
            Protocol_SendACK(CMD_PROCESS_STOP, ACK_OK, (uint8_t)Process_GetPhase());
            break;

        case CMD_PROCESS_PAUSE:
            /* 暂停工艺流程：设置标志由主循环执行 */
            g_protocol_process_pause_request = 1;
            g_system_status = SYSTEM_PAUSED;
            g_protocol_last_cmd = CMD_PROCESS_PAUSE;
            g_protocol_cmd_ready = 1;
            Protocol_SendACK(CMD_PROCESS_PAUSE, ACK_OK, (uint8_t)Process_GetPhase());
            break;

        case CMD_PROCESS_RESUME:
            /* 恢复工艺流程 */
            Process_Resume();
            g_system_status = SYSTEM_RUNNING;
            g_protocol_last_cmd = CMD_PROCESS_RESUME;
            g_protocol_cmd_ready = 1;
            Protocol_SendACK(CMD_PROCESS_RESUME, ACK_OK, (uint8_t)Process_GetPhase());
            break;

        case CMD_SET_CLEAN_MODE:
            /* 设置清洗模式  data:[模式] */
            g_protocol_last_cmd = CMD_SET_CLEAN_MODE;
            g_protocol_cmd_ready = 1;
            if (len >= 1) {
                g_clean_mode = (CleanMode_TypeDef)data[0];
                Process_SetParam(0xF1, (uint16_t)data[0]);  /* 与 Process_SetParam 中0xF1 一致 */
                Protocol_SendACK(CMD_SET_CLEAN_MODE, ACK_OK, data[0]);
            } else {
                Protocol_SendACK(CMD_SET_CLEAN_MODE, ACK_PARAM, 0);
            }
            break;

        case CMD_SET_PACK_MODE:
            /* 设置打包模式  data:[模式] */
            g_protocol_last_cmd = CMD_SET_PACK_MODE;
            g_protocol_cmd_ready = 1;
            if (len >= 1) {
                g_pack_mode = (PackTriggerMode_TypeDef)data[0];
                Process_SetParam(0xF2, (uint16_t)data[0]);  /* 与 Process_SetParam 中 0xF2 一致 */
                Protocol_SendACK(CMD_SET_PACK_MODE, ACK_OK, data[0]);
            } else {
                Protocol_SendACK(CMD_SET_PACK_MODE, ACK_PARAM, 0);
            }
            break;

        case CMD_SET_PARAM:
            /* 设置工艺参数  data:[参数ID][值高8][值低8] */
            g_protocol_last_cmd = CMD_SET_PARAM;
            g_protocol_cmd_ready = 1;
            if (len >= 3) {
                uint8_t param_id = data[0];
                uint16_t value = ((uint16_t)data[1] << 8) | data[2];
                Process_SetParam(param_id, value);
                Protocol_SendACK(CMD_SET_PARAM, ACK_OK, param_id);
            } else {
                Protocol_SendACK(CMD_SET_PARAM, ACK_PARAM, 0);
            }
            break;

        case CMD_GET_STATUS:
            /* 查询工艺状态，回传详细状态帧 */
            {
                uint8_t st_data[8];
                st_data[0] = (uint8_t)Process_GetPhase();       /* 主流程阶段 */
                st_data[1] = (uint8_t)Process_GetCleanState();   /* 清洗子状态 */
                st_data[2] = (uint8_t)Process_GetPackZoneState();    /* 打包子状态 */
                st_data[3] = (uint8_t)g_clean_mode;              /* 清洗模式 */
                st_data[4] = (uint8_t)g_pack_mode;               /* 打包模式 */
                st_data[5] = (uint8_t)g_system_status;           /* 系统状态 */
                st_data[6] = 0;                                   /* 保留 */
                st_data[7] = 0;                                   /* 保留 */
                Protocol_SendFrame(s_current_channel, CMD_GET_STATUS, st_data, 8);
            }
            break;

        case CMD_ULTRASONIC_TEST:
            {
                uint8_t ultra_data[3];
                uint16_t dist_mm = Process_ReadPackDistance();
                ultra_data[0] = (uint8_t)(dist_mm >> 8);
                ultra_data[1] = (uint8_t)(dist_mm & 0xFF);
                ultra_data[2] = (dist_mm == 0xFFFF) ? 0xFF : 0x00;
                Protocol_SendFrame(s_current_channel, CMD_ULTRASONIC_TEST, ultra_data, 3);
            }
            break;

        default:
            break;
    }
}

// =============================================================================
// 计算校验和（异或校验）
// =============================================================================
uint8_t Protocol_CalculateChecksum(uint8_t* data, uint8_t len)
{
    uint8_t checksum = 0;
    uint8_t i;

    for (i = 0; i < len; i++) {
        checksum ^= data[i];
    }

    return checksum;
}

// =============================================================================
// 根据USARTx获取通信通道枚举
// =============================================================================
static CommChannel_TypeDef Protocol_GetChannelFromUSART(USART_TypeDef* USARTx)
{
    #if defined(STM32F10X_HD)
    if (USARTx == USART1) return CHANNEL_USART1;
    if (USARTx == USART2) return CHANNEL_USART2;
    if (USARTx == USART3) return CHANNEL_USART3;
    #endif
    (void)USARTx;
    return CHANNEL_USART2;
}

// =============================================================================
// 发送ACK响应（语音模块命令回传）
// ACK帧: [AA] [原命令+0x80] [长度=2] [状态][设备ID] [校验]
// =============================================================================
static void Protocol_SendACK(uint8_t original_cmd, uint8_t status, uint8_t device_id)
{
    uint8_t ack_cmd = (uint8_t)(original_cmd + 0x80);  /* ACK命令 = 原命令 + 0x80 */
    uint8_t data[2];
    data[0] = status;
    data[1] = device_id;
    Protocol_SendFrame(s_current_channel, ack_cmd, data, 2);
}

// =============================================================================
// 串口接收数据处理（应放在USART中断中调用）
// =============================================================================
void Protocol_CommChannel_HandleByte(USART_TypeDef* USARTx, uint8_t byte)
{
    static ProtocolFrame_TypeDef rx_frame;
    static uint8_t rx_state = 0;
    static uint8_t rx_data_count = 0;
    uint8_t calc_checksum;

    switch (rx_state) {
        case 0:
            if (byte == FRAME_HEAD) {
                rx_frame.head = byte;
                rx_state = 1;
            }
            break;

        case 1:
            rx_frame.cmd = byte;
            rx_state = 2;
            break;

        case 2:
            rx_frame.len = byte;
            if (rx_frame.len > PROTOCOL_MAX_DATA_LEN) {
                rx_frame.len = PROTOCOL_MAX_DATA_LEN;
            }
            rx_data_count = 0;
            rx_state = 3;
            break;

        case 3:
            rx_frame.data[rx_data_count++] = byte;
            if (rx_data_count >= rx_frame.len) {
                rx_state = 4;
            }
            break;

        case 4:
            rx_frame.checksum = byte;
            /* 校验和 = 帧头 ^ 命令 ^ 长度 ^ 数据 */
            calc_checksum = FRAME_HEAD ^ rx_frame.cmd ^ rx_frame.len;
            for (rx_data_count = 0; rx_data_count < rx_frame.len; rx_data_count++) {
                calc_checksum ^= rx_frame.data[rx_data_count];
            }
            if (calc_checksum == rx_frame.checksum) {
                s_current_channel = Protocol_GetChannelFromUSART(USARTx);
                Protocol_ProcessCommand(rx_frame.cmd, rx_frame.data, rx_frame.len);
            } else {
                /* 校验失败，静默丢弃（不清空 data 数组，避免后续字节解析被污染） */
            }
            rx_state = 0;
            break;

        default:
            rx_state = 0;
            break;
    }
}

// =============================================================================
// 主动上报传感器数据给Jetson（周期调用）- 使用对象化驱动
// =============================================================================
void Protocol_ReportSensorData(void)
{
    /* 更新传感器数据 - 使用快捷宏 */
    g_sensor_data.motor1_position = M1_GetPos();
    g_sensor_data.motor2_position = M2_GetPos();
    g_sensor_data.motor3_position = M3_GetPos();

    g_sensor_data.motor1_speed = MOTOR[STEPPER_1].current_speed;
    g_sensor_data.motor2_speed = MOTOR[STEPPER_2].current_speed;
    g_sensor_data.motor3_speed = MOTOR[STEPPER_3].current_speed;

    g_sensor_data.servo1_angle = SERVO[SERVO_1].GetAngle(&SERVO[SERVO_1]);
    g_sensor_data.servo2_angle = SERVO[SERVO_2].GetAngle(&SERVO[SERVO_2]);

    g_sensor_data.adc_value = (uint16_t)ADC_PRESSURE_GetValue();

    /* 发送给Jetson */
    Protocol_SendSensorData(CHANNEL_USART2);
}

// =============================================================================
// 发送任务完成通知
// =============================================================================
void Protocol_SendTaskComplete(CommChannel_TypeDef channel)
{
    uint8_t data[2];
    data[0] = 1;
    data[1] = 0;

    Protocol_SendFrame(channel, CMD_TASK_COMPLETE, data, 2);
}

/******************* (C) COPYRIGHT 2026 *****END OF FILE****/
