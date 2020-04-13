
#ifndef __SINGLE_WIRE_MASTER_H__
#define __SINGLE_WIRE_MASTER_H__

#include "stm32f0xx_hal.h"

//单总线主机
typedef struct
{
  uint8_t slave_no;     //单总线从机的编号
  
  GPIO_TypeDef* port;   //信号线引脚port
  uint16_t pin;         //信号线引脚pin
  uint8_t pin_pos;      //信号线引脚pin在port中的位置
  uint8_t pin_dir;      //信号线方向，输入还是输出

  uint8_t rw_req;       //单总线读取请求状态
  uint8_t reg_addr;     //单总线正在操作的寄存器地址
  uint8_t reg_data;     //单总线正在操作的寄存器值
  uint8_t state;        //单总线状态机状态

  uint16_t cnt;         //单总线操作时序的计数器
  uint16_t bit_index;   //正在操作的bit的位置
  uint16_t sample_index;//采样点/输出点相对于时序计数器的位置
  uint8_t verify_bit;   //校验位
  uint8_t rd_wait_cnt;  //读取时等待计数
} SWM;

void SWMInit(SWM * master,  uint8_t no, GPIO_TypeDef* port, uint16_t pin);

void SWMStateMachine(SWM * master);

uint8_t SWMRead(SWM * master, uint8_t reg);

uint8_t SWMWrite(SWM * master, uint8_t reg, uint8_t data);

#endif
