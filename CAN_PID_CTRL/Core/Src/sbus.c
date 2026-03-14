#include "sbus.h"
#include "main.h"
#include "can_receive.h"


extern UART_HandleTypeDef huart3;
uint16_t g_sbus_channels[16] = {0};
uint8_t sbus_data[25] = {0};
fp32 set_speed[4] = {0};

void sbus_decode(uint8_t *sbus_buf, uint16_t *channels)
{
    // 1. 空指针校验
    if (sbus_buf == NULL || channels == NULL) 
    {
        return;
    }

    // 2. SBUS帧头校验（增强鲁棒性）
    if (sbus_buf[0] != 0x0F) 
    {
        memset(channels, 0, 16 * sizeof(uint16_t)); // 非法帧清空通道
        return;
    }

    // 3. 16个通道的11位数据解析（修正第5通道位移错误）
    channels[0]  = ((sbus_buf[1] | sbus_buf[2] << 8) & 0x07FF);          // CH1
    channels[1]  = ((sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF);     // CH2
    channels[2]  = ((sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF); // CH3
    channels[3]  = ((sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF);     // CH4
    channels[4]  = ((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF);     // CH5
    channels[5]  = ((sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 8) & 0x07FF);  // CH6（修正<<9为<<8）
    channels[6]  = ((sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF);    // CH7
    channels[7]  = ((sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF);   // CH8
    channels[8]  = ((sbus_buf[12] | sbus_buf[13] << 8) & 0x07FF);        // CH9
    channels[9]  = ((sbus_buf[13] >> 3 | sbus_buf[14] << 5) & 0x07FF);   // CH10
    channels[10] = ((sbus_buf[14] >> 6 | sbus_buf[15] << 2 | sbus_buf[16] << 10) & 0x07FF); // CH11
    channels[11] = ((sbus_buf[16] >> 1 | sbus_buf[17] << 7) & 0x07FF);   // CH12
    channels[12] = ((sbus_buf[17] >> 4 | sbus_buf[18] << 4) & 0x07FF);   // CH13
    channels[13] = ((sbus_buf[18] >> 7 | sbus_buf[19] << 1 | sbus_buf[20] << 8) & 0x07FF);  // CH14（同理修正位移）
    channels[14] = ((sbus_buf[20] >> 2 | sbus_buf[21] << 6) & 0x07FF);   // CH15
    channels[15] = ((sbus_buf[21] >> 5 | sbus_buf[22] << 3) & 0x07FF);   // CH16
}


//code below needs polish
void cul_rpm(uint16_t *channels, fp32 *set_speed) /*0 AND 1 L || 2 AND 3 R*/
{
  fp32 rspeed1 = channels[1] - 1024.0f;
  fp32 rspeed2 = channels[3] - 1024.0f;
  fp32 gap = 0.0f;
  const fp32 MAX_SPEED = 8000.0f;  // 最大转速
  const fp32 DEAD_ZONE = 30.0f;   // 死区

  // 死区处理
  if ((rspeed1 > -DEAD_ZONE) && (rspeed1 < DEAD_ZONE)) rspeed1 = 0.0f;
  if ((rspeed2 > -DEAD_ZONE) && (rspeed2 < DEAD_ZONE)) rspeed2 = 0.0f;

  // 初始化四轮转速为0（避免旧值影响）
  set_speed[0] = 0.0f;
  set_speed[1] = 0.0f;
  set_speed[2] = 0.0f;
  set_speed[3] = 0.0f;

  // 前进/后退控制（channels[0]：前后通道）
  if ((channels[1] > 360) && (channels[1] < 1680))
  {
    fp32 base_speed = rspeed1 * (MAX_SPEED / 1024.0f);
    // 限制转速范围，避免溢出
    base_speed = (base_speed > MAX_SPEED) ? MAX_SPEED : base_speed;
    base_speed = (base_speed < -MAX_SPEED) ? -MAX_SPEED : base_speed;
    
    set_speed[0] = base_speed;
    set_speed[1] = base_speed;
    set_speed[2] = base_speed;
    set_speed[3] = base_speed;
  }

  // 转向控制（channels[1]：左右通道）
  if ((channels[3] > 360) && (channels[1] < 1680))
  {
    gap = rspeed2 * (MAX_SPEED / 1024.0f);
    // 限制转向增量范围
    gap = (gap > MAX_SPEED/2) ? MAX_SPEED/2 : gap;
    gap = (gap < -MAX_SPEED/2) ? -MAX_SPEED/2 : gap;
  }

  // 左转向（gap<0）：左轮减速
  if (gap > 0)
  {
    set_speed[0] += gap;  // 左轮1减速
    set_speed[1] += gap;  // 左轮2减速
    // 限制最终转速不超过最大值
    set_speed[0] = (set_speed[0] < -MAX_SPEED) ? -MAX_SPEED : set_speed[0];
    set_speed[1] = (set_speed[1] < -MAX_SPEED) ? -MAX_SPEED : set_speed[1];
  }
  // 右转向（gap>0）：右轮减速
  if (gap < 0)
  {
    set_speed[2] -= gap;  // 右轮1减速
    set_speed[3] -= gap;  // 右轮2减速
    // 限制最终转速不低于最小值
    set_speed[2] = (set_speed[2] < -MAX_SPEED) ? -MAX_SPEED : set_speed[2];
    set_speed[3] = (set_speed[3] < -MAX_SPEED) ? -MAX_SPEED : set_speed[3];
  } 
}
void yaw_move(uint16_t *channels)
{
  fp32 rspeed3 = channels[2] - 1024.0f;
  if(rspeed3 < 0)
  {
    jump_check_up();
    CAN_cmd_gimbal(800,0,0,0);
  }
  else if(rspeed3 >= 0)
  {
    jump_check_down();
    CAN_cmd_gimbal(-800,0,0,0);
  }
}
