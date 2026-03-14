#include "can_receive.h"
#include "main.h"
#include "sbus.h"

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart1;
uint8_t send_buf[4];

motor_measure_t motor_chassis[5];
int16_t motor_rsp;
int16_t D_angel = 0;

#define get_motor_measure(ptr,data)                                      \
    {																	 \
		(ptr)->last_ecd = (ptr)->ecd;                                    \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);             \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);       \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);   \
        (ptr)->temperate = (data)[6];                                    \
    }                                                                    \
    
static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

int16_t fp32_to_int16(fp32 fp32_val, fp32 fp_min, fp32 fp_max, int16_t int16_min, int16_t int16_max)
{
    // 步骤1：先限制浮点值在[fp_min, fp_max]，避免超出映射范围
    if (fp32_val < fp_min) fp32_val = fp_min;
    if (fp32_val > fp_max) fp32_val = fp_max;

    // 步骤2：线性映射到目标int16范围（四舍五入）
    float ratio = (fp32_val - fp_min) / (fp_max - fp_min); // 0~1的比例
    float int16_float = ratio * (int16_max - int16_min) + int16_min;

    // 步骤3：四舍五入转换为int16_t（两种方式选其一）
    //int16_t int16_val = (int16_t)roundf(int16_float); // 方式1：用roundf（需包含math.h）
     int16_t int16_val = (int16_t)(int16_float + 0.5f); // 方式2：加0.5截断（无需math.h）

    // 步骤4：最终保护，确保不超出int16_t本身的范围（-32768~32767）
    if (int16_val < INT16_MIN) int16_val = INT16_MIN;
    if (int16_val > INT16_MAX) int16_val = INT16_MAX;

    return int16_val;
}
void udata(uint8_t i)
{
    int16_t tpp = fp32_to_int16(set_speed[i],-8000.0f,8000.0f,-8000,8000); // 2字节实际转速 + 2字节目标转速
    // 实际转速：int16_t拆分为2字节（小端）
    send_buf[0] = ((uint8_t*)&motor_chassis[i].speed_rpm)[0];
    send_buf[1] = ((uint8_t*)&motor_chassis[i].speed_rpm)[1];
    // 目标转速：int16_t拆分为2字节（小端）
    send_buf[2] = ((uint8_t*)&tpp)[0];
    send_buf[3] = ((uint8_t*)&tpp)[1];

    // 4. 单次DMA发送4字节（避免连续调用导致覆盖）
    HAL_UART_Transmit_DMA(&huart1, send_buf, 4);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);
            motor_rsp = motor_chassis[0].speed_rpm;
            break;
        }   
        default:
        {
            break;
        }
    }
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &gimbal_tx_message,gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	
}

uint8_t CAN_cmd_chassis(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)
{
    uint8_t statu;
	uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
    
    statu = HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
    return statu;
}

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
	return &motor_chassis[(i & 0x03)];
}
//检测编码器突变
void jump_check_up(void)
{
    if(motor_chassis[4].last_ecd > motor_chassis[4].ecd)
    {
        int16_t dis1,dis2;
        dis1 = 65536 - motor_chassis[4].last_ecd;
        dis2 = motor_chassis[4].ecd;
        D_angel += dis1 + dis2;  
    }
    else
    {
        D_angel += motor_chassis[4].ecd - motor_chassis[4].last_ecd;
    }
}
void jump_check_down(void)
{
    if(motor_chassis[4].last_ecd < motor_chassis[4].ecd)
    {
        int16_t dis1,dis2;
        dis1 = motor_chassis[4].last_ecd;
        dis2 = motor_chassis[4].ecd - 65536;
        D_angel += dis2 - dis1;  
    }
    else
    {
        D_angel += motor_chassis[4].ecd - motor_chassis[4].last_ecd;
    }
}
