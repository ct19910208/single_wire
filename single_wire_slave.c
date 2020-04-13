/**
 ******************************************************************************
 *
 *                版权所有 (C), 2020-2030, 上海阿柚信息科技有限公司
 *
 ******************************************************************************
 * @file single_wire.c
 * @brief 单总线协议从机程序
 * @details
 *  协议说明：
 *  1.模式：主从模式，半双工，即所有读写通信都是master端发起，slave端响应
 *  2.硬件结构：通信线上设置上拉电阻，无通信时主从机端引脚保持输入状态。需要通
 *    信时，发送端设置为OD输出模式，防止主从同时输出时造成硬件损坏。特别是在一
 *    主多从时，多个从机可能会同时输出，但输出电平不一致，如果使用PP的话容易造
 *    成大电流，损坏硬件。使用OD输出模式，高电平是靠上拉电阻拉高的，由于上拉电
 *    阻阻值较大，不会产生大电流。一般情况下，想要达到很快的通信速度，需要适当
 *    减小上拉电阻的阻值，以增加驱动能力，但是阻值减小后，功耗会相应增加。
 *  3.数据方向：小端模式，即bit0最先传输
 *  4.起始位：0，主机请求通信时，首先将引脚配置成输出，并拉低，从机检测到低电平
 *    即进入接收状态
 *  4.地址位：为从机寄存器地址，地址位长度可配置
 *  5.读写位：紧随地址位之后，0表示写，1表示读
 *  6.数据位：数据长度为8bit
 *  7.校验位：采用奇偶校验，1bit
 *  8.一主多从：暂不支持，单总线实现一主多从需要有器件地址，同时需要有通信发起
 *  过程，该通信发起过程一般比较耗时，后续可支持
 *  9.连续读写：即从某个寄存器地址开始，连续读写多个字节，暂不支持，后续可支持
 *  使用说明：
 *  1.需要使用单总线从机时，首先建立SWS对象，然后调用
 *    void SWSInit(SWS* slave, GPIO_TypeDef* port, uint16_t pin)
 *    对其进行初始化
 *  2.单总线协议需要精准的时序控制，其读写时序由定时器控制，通过在定时器中断中
 *    调用函数
 *    void SWSStateMachine(SWS* slave)
 *    来实现读写时序的控制
 *  3.单总线从机的寄存器实现在此文件中，可根据实际情况对寄存器进行相应的修改，
 *    也可将寄存器独立出来，在另外的文件中实现，在SWS结构体中增加对寄存器地址
 *    的引用，在主程序中，可以调用函数
 *    void SWSSetReg(uint8_t index, uint8_t data)
 *    来将当前的系统状态、数据等写入寄存器，供主机读取
      可以在主程序中调用函数
 *    uint8_t SWSGetReg(uint8_t index)
 *    来不断查询寄存器中的值，并进行相应的操作
 * @author Saber
 * @platform STM8S003，移植到其他平台需要对IO相关操作进行修改
 * @version V1.0.0
 * @date 2020年4月9日
 * @license 
 ******************************************************************************
 */

#include "single_wire_slave.h"
#include "user.h"

//单总线从机信号脚的方向，输入或输出
typedef enum
{
  kIn = 0,  //输入
  kOut,     //输出
}SingleWirePinDir;

//单总线从机状态
typedef enum
{
  kStateIdle = 0, //空闲
  kStateRegAddr,  //接收寄存器地址和读写位
  kStateRd,       //主机读状态，即从机发送状态
  kStateWr,       //主机写状态，即从机接收状态
} SWSState;

//采样数量，即一个bit数据可以被采样的次数，保证采样点尽量在bit电平的中间
#define SAMPLE_NUMS   3

//注意，以下3个宏相互关联，为了减小计算量，这里并没有进行关联计算，需要一起手动修改
#define REG_ADDR_BIT_LEN  1
#define REG_ADDR_BIT_MASK 1
#define REG_NUM           2
//读写位的电平
#define REG_WR      0
#define REG_RD      1
//数据长度，8bit
#define DATA_BIT_LEN      8

//单总线从机寄存器
static uint8_t reg_data[REG_NUM];

/*****************************************************************************
 * @brief 单总线从机初始化
 * @history 
 *  1.@data 2020年4月10日
 *    @author Saber
 *    @note 新生成函数
 * @todo 
 ****************************************************************************/
void SWSInit(SWS* slave, GPIO_TypeDef* port, uint16_t pin)
{
  slave->port = port;
  slave->pin = pin;
  slave->pin_pos = 0;
  while (((slave->pin) & (1 << slave->pin_pos)) == 0)
  {
    slave->pin_pos++;
  }
  slave->pin_dir = (uint8_t)kIn;

  slave->state = kStateIdle;
  GPIO_Init(slave->port, slave->pin, GPIO_MODE_IN_FL_NO_IT);
}

/*****************************************************************************
 * @brief 设定单总线从机引脚方向
 * @history 
 *  1.@data 2020年4月11日
 *    @author Saber
 *    @note 新生成函数
 * @todo 
 ****************************************************************************/
static void SetPinDir(SWS * slave, SingleWirePinDir dir)
{
  slave->pin_dir = (uint8_t)dir;

  if (dir == kIn)  //输入
  {
//    GPIO_Init(slave->port, slave->pin, GPIO_MODE_IN_FL_NO_IT);
    slave->port->CR2 &= (uint8_t)(~(slave->pin));
    slave->port->DDR &= (uint8_t)(~(slave->pin));
    slave->port->CR1 &= (uint8_t)(~(slave->pin));
//    slave->port->CR2 &= (uint8_t)(~(slave->pin));
  }
  else  //输出
  {
//    GPIO_Init(slave->port, slave->pin, GPIO_MODE_OUT_OD_HIZ_FAST);
    slave->port->CR2 &= (uint8_t)(~(slave->pin));
    slave->port->ODR |= (uint8_t)slave->pin;
    slave->port->DDR |= (uint8_t)slave->pin;
    slave->port->CR1 &= (uint8_t)(~(slave->pin));
    slave->port->CR2 |= (uint8_t)slave->pin;
  }
}

/*****************************************************************************
 * @brief 单总线从机状态机
 * @history 
 *  1.@data 2020年4月11日
 *    @author Saber
 *    @note 新生成函数
 * @todo 
 ****************************************************************************/
void SWSStateMachine(SWS* slave)
{
  slave->cnt++;
  
  switch (slave->state)
  {
    case kStateIdle:
      //检测到下拉，表明有主机发起通信
      if ((slave->port->IDR & slave->pin) == 0)
      {
        slave->cnt = 0;
        slave->bit_index = 0;
        slave->sample_index = SAMPLE_NUMS + 1;  //延迟1个脉冲采样，尽量使采样点在信号的中间
        slave->reg_addr = 0;
        slave->state = kStateRegAddr;
      }
      break;
    case kStateRegAddr:
      if (slave->cnt == slave->sample_index)
      {
        //采样信号电平
        if ((slave->port->IDR & slave->pin) != 0)
        {
          slave->reg_addr |= (1 << slave->bit_index);
        }
        slave->bit_index++;
        slave->sample_index += SAMPLE_NUMS;
        //判断地址和读写位是否读取完成
        if (slave->bit_index > REG_ADDR_BIT_LEN)
        {
          slave->verify_bit = 0;
          slave->bit_index = 0;
          if ((slave->reg_addr >> REG_ADDR_BIT_LEN) == REG_WR)  //匹配写地址，进入写流程
          {
            slave->state = kStateWr;
            slave->reg_addr &= REG_ADDR_BIT_MASK;  //去除读写位
            slave->reg_data = 0;
          }
          else  //匹配读地址，进入读流程
          {
            slave->state = kStateRd;
            slave->reg_addr &= REG_ADDR_BIT_MASK;  //去除读写位
            slave->reg_data = reg_data[slave->reg_addr];
            slave->sample_index += SAMPLE_NUMS; //多等待一个数据周期再进行数据发送
          }
        }
      }
      break;
    case kStateWr:
      if (slave->cnt == slave->sample_index)
      {
        //读取数据位
        if (slave->bit_index < DATA_BIT_LEN)
        {
          if ((slave->port->IDR & slave->pin) != 0)
          {
            slave->reg_data |= (1 << slave->bit_index);
            slave->verify_bit ^= 0x01;
          }
          slave->bit_index++;
          slave->sample_index += SAMPLE_NUMS;
        }
        else
        {
          //校验通过
          if (((slave->port->IDR) & (slave->pin)) != 0)
          {
            if (slave->verify_bit != 0)
            {
              reg_data[slave->reg_addr] = slave->reg_data;
            }
          }
          else
          {
            if (slave->verify_bit == 0)
            {
              reg_data[slave->reg_addr] = slave->reg_data;
            }
          }
          slave->state = kStateIdle;
        }
      }
      break;
    case kStateRd:
      if (slave->cnt == slave->sample_index)
      {
        //拉低信号线
        if (slave->pin_dir == kIn)
        {
          SetPinDir(slave, kOut);
          slave->port->ODR &= (uint8_t)(~slave->pin);
//          slave->bit_index = 0;
        }
        else
        {
          //发送数据
          if (slave->bit_index < DATA_BIT_LEN)
          {
            if ((slave->reg_data & 0x01) == 0x01)
            {
              slave->verify_bit ^= 0x01;
              slave->port->ODR |= slave->pin;
            }
            else
            {
              slave->port->ODR &= (uint8_t)(~slave->pin);
            }
            slave->reg_data >>= 1;
          }
          //发送校验位
          else if (slave->bit_index == DATA_BIT_LEN)
          {
            if (slave->verify_bit != 0x00)
            {
              slave->verify_bit ^= 0x01;
              slave->port->ODR |= slave->pin;
            }
            else
            {
              slave->port->ODR &= (uint8_t)(~slave->pin);
            }
          }
          else  //恢复IDLE状态，将单线引脚设置为输入浮空
          {
            SetPinDir(slave, kIn);
            slave->state = kStateIdle;
          }
          slave->bit_index++;
        }
        slave->sample_index += SAMPLE_NUMS;
      }
      break;
    default:
      
      break;
  }
}

/*****************************************************************************
 * @brief 设定单总线从机寄存器的值，在主程序中调用，设定寄存器值后可供主机读取
 * @history 
 *  1.@data 2020年4月11日
 *    @author Saber
 *    @note 新生成函数
 * @todo 
 ****************************************************************************/
void SWSSetReg(uint8_t index, uint8_t data)
{
  if (index < REG_NUM)
  {
    reg_data[index] = data;
  }
}

/*****************************************************************************
 * @brief 获取单总线从机寄存器的值，在主程序中调用，查询从主机写进来的数据
 * @history 
 *  1.@data 2020年4月11日
 *    @author Saber
 *    @note 新生成函数
 * @todo 
 ****************************************************************************/
uint8_t SWSGetReg(uint8_t index)
{
  if (index < REG_NUM)
  {
    return reg_data[index];
  }
  else
  {
    return 0xFF;
  }
}


