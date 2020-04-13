/**
 ******************************************************************************
 *
 *                版权所有 (C), 2020-2030, 上海阿柚信息科技有限公司
 *
 ******************************************************************************
 * @file single_wire_master.c
 * @brief 单总线协议主机程序
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
 *  1.需要与一个从机进行通信时，首先建立SWM变量，然后调用
 *    void SWMInit(SWM * master, uint8_t no, GPIO_TypeDef* port, uint16_t pin)
 *    对其进行初始化
 *  2.单总线协议需要精准的时序控制，其读写时序由定时器控制，通过在定时器中断中
 *    调用函数
 *    void SWMStateMachine(SWM * master)
 *    来实现读写时序的控制
 *  3.需要进行读写时，分别调用
 *    uint8_t SWMRead(SWM * master, uint8_t reg)
 *    uint8_t SWMWrite(SWM * master, uint8_t reg, uint8_t data)
 *    发起读写请求，如果当前处于IDLE状态，则请求成功，返回1，否则请求失败，返
 *    回0
 *  4.需要在用户程序中重写读取完成回调函数
 *    __weak void SingleWireRdFinishCallback(SWM * master)
 *    对读取到的数据进行处理，该函数是在读写状态机控制函数中调用的，也就是在
 *    时序控制定时器中断中调用的，注意重写的函数不能太耗时
 *  5.同一个主机对象不能同时进行读和写，也就是说该协议为半双工。但是多个主机
 *    对象与不同从机的通信可以同时进行，互不干扰。注意，主机对象越多，时序控
 *    制定时器中断的处理任务就会越多，对MCU的性能要求就越高。
 * @author Saber
 * @platform STM32F030 HAL，移植到其他平台需要对IO相关操作进行修改
 * @version V1.0.0
 * @date 2020年4月11日
 * @license 
 ******************************************************************************
 */

#include "single_wire_master.h"
#include "main.h"

//单总线信号脚的方向，输入还是输出
typedef enum
{
  kIn = 0,  //输入
  kOut,     //输出
}SingleWirePinDir;

//单总线读写请求状态
typedef enum
{
  kIdle = 0,  //空闲，可以申请读写
  kRdReq,     //正在读slave
  kWrReq,     //正在写slave
} SingleWireRwReq;

//单总线状态机的状态
typedef enum
{
  kStateIdle = 0, //空闲
  kStateRegAddr,  //写地址
  kStateRd,       //读
  kStateRdPre,    //读之前的操作，此时将引脚设定为输入，一旦检测到低电平即进入读状态
  kStateWr,       //写状态
} SWMState;

//采样数量，即一个bit数据可以被采样的次数，保证采样点尽量在bit电平的中间
#define SAMPLE_NUMS   3

//注意，以下3个宏相互关联，为了减小计算量，这里并没有进行关联计算，需要一起手动修改
#define REG_ADDR_BIT_LEN  1
#define REG_ADDR_BIT_MASK 1
#define REG_NUM           2
//读写位的电平
#define REG_WR      GPIO_PIN_RESET
#define REG_RD      GPIO_PIN_SET
//数据长度，8bit
#define DATA_BIT_LEN      8
//读等待最大周期数，读取时等待时间超过该时间即认为读取失败，返回IDLE状态
#define RD_WAIT_MAX_CNT   10

/*****************************************************************************
 * @brief 单总线初始化
 * @history 
 *  1.@data 2020年4月9日
 *    @author Saber
 *    @note 新生成函数
 * @todo 
 ****************************************************************************/
void SWMInit(SWM * master, uint8_t no, GPIO_TypeDef* port, uint16_t pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  master->slave_no = no;
  
  master->port = port;
  master->pin = pin;
  master->pin_pos = 0;
  while (((master->pin) & (1 << master->pin_pos)) == 0)
  {
    master->pin_pos++;
  }
  master->pin_dir = (uint8_t)kIn;

  GPIO_InitStruct.Pin = master->pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(master->port, &GPIO_InitStruct);

  master->rw_req = kIdle;
  master->state = kStateIdle;
}

/*****************************************************************************
 * @brief 设定单总线引脚方向，输入还是输出
 * @history 
 *  1.@data 2020年4月9日
 *    @author Saber
 *    @note 新生成函数
 * @todo 
 ****************************************************************************/
static void SetPinDir(SWM * master, SingleWirePinDir dir)
{
  uint32_t temp;
//  GPIO_InitTypeDef GPIO_InitStruct = {0};

  master->pin_dir = (uint8_t)dir;
//  GPIO_InitStruct.Pin = master->pin;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
  if (dir == kIn)  //输入
  {
//    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//    HAL_GPIO_Init(master->port, &GPIO_InitStruct);
    //设定为输入
    master->port->MODER &= ~(GPIO_MODER_MODER0 << (master->pin_pos * 2U));
//    //上拉，由于初始化已经设定为上拉，后面不会再改变，所以可以不执行，节约时间
//    temp = master->port->PUPDR;
//    CLEAR_BIT(temp, GPIO_PUPDR_PUPDR0 << (master->pin * 2U));
//    SET_BIT(temp, (GPIO_PULLUP) << (master->pin * 2U));
//    master->port->PUPDR = temp;
    //
  }
  else  //输出
  {
//    HAL_GPIO_WritePin(master->port, master->pin, GPIO_PIN_SET);
    //输出高电平
    master->port->BSRR = (uint32_t)master->pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
//    HAL_GPIO_Init(master->port, &GPIO_InitStruct);
    //设定为输出
    temp = master->port->MODER;
    CLEAR_BIT(temp, GPIO_MODER_MODER0 << (master->pin_pos * 2U));   
    SET_BIT(temp, (GPIO_MODE_OUTPUT_OD & 0x00000003U) << (master->pin_pos * 2U));
    master->port->MODER = temp;
    //IO口配置为高速
    master->port->OSPEEDR |= GPIO_SPEED_FREQ_HIGH << (master->pin_pos * 2U); 
    //设置为OD
    master->port->OTYPER |= ((GPIO_MODE_OUTPUT_OD & 0x00000010U) >> 4U)
      << master->pin_pos;
  }
}

/*****************************************************************************
 * @brief 单总线读取结束时的回调函数，这里定义为weak，需要在应用程序中重写该
 * 函数，以在读取结束时将读到的数据取出进行处理，注意，由于该函数会在单总线的
 * 定时器中断中调用，重写的该函数不能太耗时
 * @history 
 *  1.@data 2020年4月9日
 *    @author Saber
 *    @note 新生成函数
 * @todo 
 ****************************************************************************/
__weak void SingleWireRdFinishCallback(SWM * master)
{
  UNUSED(master);
}

/*****************************************************************************
 * @brief 单总线协议时序控制状态机，在单总线定时器中调用，定时器需要有精准的定
 * 时周期
 * @history 
 *  1.@data 2020年4月10日
 *    @author Saber
 *    @note 新生成函数
 * @todo 
 ****************************************************************************/
void SWMStateMachine(SWM * master)
{
  master->cnt++;
  
  switch (master->state)
  {
    case kStateIdle:
      //有读写请求，首先拉低信号线
      if (master->rw_req != kIdle)
      {
        master->cnt = 0;
        master->bit_index = 0;
        master->sample_index = SAMPLE_NUMS;
        master->state = kStateRegAddr;
        SetPinDir(master, kOut);
//        HAL_GPIO_WritePin(master->port, master->pin, GPIO_PIN_RESET);
        master->port->BRR = (uint32_t)master->pin;
      }
      break;
    case kStateRegAddr:
      if (master->cnt == master->sample_index)
      {
        if (master->bit_index < REG_ADDR_BIT_LEN) //写地址
        {
          if ((master->reg_addr & (1 << master->bit_index)) != 0)
          {
//            HAL_GPIO_WritePin(master->port, master->pin, GPIO_PIN_SET);
            master->port->BSRR = (uint32_t)master->pin;
          }
          else
          {
//            HAL_GPIO_WritePin(master->port, master->pin, GPIO_PIN_RESET);
            master->port->BRR = (uint32_t)master->pin;
          }
          master->bit_index++;
        }
        else  //写读写位
        {
          if (master->rw_req == kRdReq)
          {
//            HAL_GPIO_WritePin(master->port, master->pin, REG_RD);
            master->port->BSRR = (uint32_t)master->pin;
            master->state = kStateRdPre;
          }
          else
          {
//            HAL_GPIO_WritePin(master->port, master->pin, REG_WR);
            master->port->BRR = (uint32_t)master->pin;
            master->state = kStateWr;
          }
          master->bit_index = 0;
          master->verify_bit = 0;
        }
        master->sample_index += SAMPLE_NUMS;
      }
      break;
    case kStateWr:
      if (master->cnt == master->sample_index)
      {
        if (master->bit_index < DATA_BIT_LEN) //写数据
        {
          if ((master->reg_data & (1 << master->bit_index)) != 0)
          {
//            HAL_GPIO_WritePin(master->port, master->pin, GPIO_PIN_SET);
            master->port->BSRR = (uint32_t)master->pin;
            master->verify_bit ^= 0x01;
          }
          else
          {
//            HAL_GPIO_WritePin(master->port, master->pin, GPIO_PIN_RESET);
            master->port->BRR = (uint32_t)master->pin;
          }
        }
        else if (master->bit_index == DATA_BIT_LEN) //写校验位
        {
          if (master->verify_bit != 0)
          {
//            HAL_GPIO_WritePin(master->port, master->pin, GPIO_PIN_SET);
            master->port->BSRR = (uint32_t)master->pin;
          }
          else
          {
//            HAL_GPIO_WritePin(master->port, master->pin, GPIO_PIN_RESET);
            master->port->BRR = (uint32_t)master->pin;
          }
        }
        else
        {
          master->state = kStateIdle;
          SetPinDir(master, kIn);
          master->rw_req = kIdle;
        }

        master->bit_index++;
        master->sample_index += SAMPLE_NUMS;
      }
      break;
    case kStateRdPre:
      if (master->cnt == master->sample_index)
      {
        if (master->pin_dir == kOut)  //将信号线设置为输入
        {
          SetPinDir(master, kIn);
          master->rd_wait_cnt = 0;
        }
        else
        {
          //检测到低电平，表明从机开始发送数据了
//          if (HAL_GPIO_ReadPin(master->port, master->pin) == GPIO_PIN_RESET)
          if ((master->port->IDR & master->pin) == GPIO_PIN_RESET)
          {
            master->state = kStateRd;
            master->reg_data = 0;
            master->sample_index += SAMPLE_NUMS;
          }
          //长时间没有等到低电平，停止接收，恢复IDLE状态
          master->rd_wait_cnt++;
          if (master->rd_wait_cnt >= RD_WAIT_MAX_CNT)
          {
            master->state = kStateIdle;
            master->rw_req = kIdle;
          }
        }
        master->sample_index++;
      }
      break;
    case kStateRd:
      if (master->cnt == master->sample_index)
      {
        if (master->bit_index < DATA_BIT_LEN)
        {
//          if (HAL_GPIO_ReadPin(master->port, master->pin) == GPIO_PIN_SET)
          if ((master->port->IDR & master->pin) != GPIO_PIN_RESET)
          {
            master->reg_data |= (1 << master->bit_index);
            master->verify_bit ^= 0x01;
          }
          master->bit_index++;
          master->sample_index += SAMPLE_NUMS;
        }
        else
        {
          //校验通过
//          if ((uint8_t)(HAL_GPIO_ReadPin(master->port, master->pin))
//            == master->verify_bit)
          if ((master->port->IDR & master->pin) == 
            ((uint32_t)(master->verify_bit) << master->pin))
          {
            SingleWireRdFinishCallback(master);
          }
          master->state = kStateIdle;
          master->rw_req = kIdle;
        }
      }
      break;
    
    default:
      
      break;
  }
}

/*****************************************************************************
 * @brief 单总线读，请求读取时将需要读取的寄存器地址传入，读取时序由状态机控制，
 * 读取完成时会调用读取回调函数处理读到的数据，请求成功返回1，否则返回0
 * @history 
 *  1.@data 2020年4月10日
 *    @author Saber
 *    @note 新生成函数
 * @todo 
 ****************************************************************************/
uint8_t SWMRead(SWM * master, uint8_t reg)
{
  if (master->rw_req == kIdle)
  {
    master->rw_req = kRdReq;
    master->reg_addr = reg;
    return 1;
  }
  else
  {
    return 0;
  }
}

/*****************************************************************************
 * @brief 单总线写函数，请求写时将需要写的寄存器地址和值传入，写时序由状态机控
 * 制，请求成功返回1，否则返回0
 * @history 
 *  1.@data 2020年4月10日
 *    @author Saber
 *    @note 新生成函数
 * @todo 
 ****************************************************************************/
uint8_t SWMWrite(SWM * master, uint8_t reg, uint8_t data)
{
  if (master->rw_req == kIdle)
  {
    master->rw_req = kWrReq;
    master->reg_addr = reg;
    master->reg_data = data;
    return 1;
  }
  else
  {
    return 0;
  }
}


