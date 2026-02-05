#include "sbus.h"
#include "main.h"


extern UART_HandleTypeDef huart3;

void sbus_decode(uint8_t *sbus_buf, uint16_t *channels) //channels uint16_t
{
    if (sbus_buf == NULL || channels == NULL) return; 
    
    channels[0]  = ((sbus_buf[1] | sbus_buf[2] << 8) & 0x07FF);
    channels[1]  = ((sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF);
    channels[2]  = ((sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF);
    channels[3]  = ((sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF);
    channels[4]  = ((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF);
    channels[5]  = ((sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07FF);
    channels[6]  = ((sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF);
    channels[7]  = ((sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF);
    channels[8]  = ((sbus_buf[12] | sbus_buf[13] << 8) & 0x07FF);
    channels[9]  = ((sbus_buf[13] >> 3 | sbus_buf[14] << 5) & 0x07FF);
    channels[10] = ((sbus_buf[14] >> 6 | sbus_buf[15] << 2 | sbus_buf[16] << 10) & 0x07FF);
    channels[11] = ((sbus_buf[16] >> 1 | sbus_buf[17] << 7) & 0x07FF);    
    channels[12] = ((sbus_buf[17] >> 4 | sbus_buf[18] << 4) & 0x07FF);    
    channels[13] = ((sbus_buf[18] >> 7 | sbus_buf[19] << 1 | sbus_buf[20] << 9) & 0x07FF); 
    channels[14] = ((sbus_buf[20] >> 2 | sbus_buf[21] << 6) & 0x07FF);                      
    channels[15] = ((sbus_buf[21] >> 5 | sbus_buf[22] << 3) & 0x07FF);
}


//code below needs polish
void cul_rpm(uint16_t *channels, fp32 *set_speed) /*0 AND 1 L || 2 AND 3 R*/
{
  fp32 rspeed1 = channels[0] - 1024.0f;
  fp32 rspeed2 = channels[1] - 1024.0f;
  fp32 gap = 0.0f;
  const fp32 MAX_SPEED = 8000.0f;  // 最大转速
  const fp32 DEAD_ZONE = 100.0f;   // 死区

  // 死区处理
  if ((rspeed1 > -DEAD_ZONE) && (rspeed1 < DEAD_ZONE)) rspeed1 = 0.0f;
  if ((rspeed2 > -DEAD_ZONE) && (rspeed2 < DEAD_ZONE)) rspeed2 = 0.0f;

  // 初始化四轮转速为0（避免旧值影响）
  set_speed[0] = 0.0f;
  set_speed[1] = 0.0f;
  set_speed[2] = 0.0f;
  set_speed[3] = 0.0f;

  // 前进/后退控制（channels[0]：前后通道）
  if ((channels[0] > 0) && (channels[0] < 2047))
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
  if ((channels[1] > 0) && (channels[1] < 2047))
  {
    gap = rspeed2 * (MAX_SPEED / 1024.0f);
    // 限制转向增量范围
    gap = (gap > MAX_SPEED/2) ? MAX_SPEED/2 : gap;
    gap = (gap < -MAX_SPEED/2) ? -MAX_SPEED/2 : gap;
  }

  // 左转向（gap<0）：左轮减速，右轮增速
  if (gap < 0)
  {
    set_speed[0] -= gap;  // 左轮1增速
    set_speed[1] -= gap;  // 左轮2增速
    // 限制最终转速不超过最大值
    set_speed[0] = (set_speed[0] > MAX_SPEED) ? MAX_SPEED : set_speed[0];
    set_speed[1] = (set_speed[1] > MAX_SPEED) ? MAX_SPEED : set_speed[1];
  }
  // 右转向（gap>0）：右轮减速，左轮增速
  if (gap > 0)
  {
    set_speed[2] -= gap;  // 右轮1减速
    set_speed[3] -= gap;  // 右轮2减速
    // 限制最终转速不低于最小值
    set_speed[2] = (set_speed[2] < -MAX_SPEED) ? -MAX_SPEED : set_speed[2];
    set_speed[3] = (set_speed[3] < -MAX_SPEED) ? -MAX_SPEED : set_speed[3];
  }
}

