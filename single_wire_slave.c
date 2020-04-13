/**
 ******************************************************************************
 *
 *                ��Ȩ���� (C), 2020-2030, �Ϻ�������Ϣ�Ƽ����޹�˾
 *
 ******************************************************************************
 * @file single_wire.c
 * @brief ������Э��ӻ�����
 * @details
 *  Э��˵����
 *  1.ģʽ������ģʽ����˫���������ж�дͨ�Ŷ���master�˷���slave����Ӧ
 *  2.Ӳ���ṹ��ͨ�����������������裬��ͨ��ʱ���ӻ������ű�������״̬����Ҫͨ
 *    ��ʱ�����Ͷ�����ΪOD���ģʽ����ֹ����ͬʱ���ʱ���Ӳ���𻵡��ر�����һ
 *    �����ʱ������ӻ����ܻ�ͬʱ������������ƽ��һ�£����ʹ��PP�Ļ�������
 *    �ɴ��������Ӳ����ʹ��OD���ģʽ���ߵ�ƽ�ǿ������������ߵģ�����������
 *    ����ֵ�ϴ󣬲�������������һ������£���Ҫ�ﵽ�ܿ��ͨ���ٶȣ���Ҫ�ʵ�
 *    ��С�����������ֵ������������������������ֵ��С�󣬹��Ļ���Ӧ���ӡ�
 *  3.���ݷ���С��ģʽ����bit0���ȴ���
 *  4.��ʼλ��0����������ͨ��ʱ�����Ƚ��������ó�����������ͣ��ӻ���⵽�͵�ƽ
 *    ���������״̬
 *  4.��ַλ��Ϊ�ӻ��Ĵ�����ַ����ַλ���ȿ�����
 *  5.��дλ�������ַλ֮��0��ʾд��1��ʾ��
 *  6.����λ�����ݳ���Ϊ8bit
 *  7.У��λ��������żУ�飬1bit
 *  8.һ����ӣ��ݲ�֧�֣�������ʵ��һ�������Ҫ��������ַ��ͬʱ��Ҫ��ͨ�ŷ���
 *  ���̣���ͨ�ŷ������һ��ȽϺ�ʱ��������֧��
 *  9.������д������ĳ���Ĵ�����ַ��ʼ��������д����ֽڣ��ݲ�֧�֣�������֧��
 *  ʹ��˵����
 *  1.��Ҫʹ�õ����ߴӻ�ʱ�����Ƚ���SWS����Ȼ�����
 *    void SWSInit(SWS* slave, GPIO_TypeDef* port, uint16_t pin)
 *    ������г�ʼ��
 *  2.������Э����Ҫ��׼��ʱ����ƣ����дʱ���ɶ�ʱ�����ƣ�ͨ���ڶ�ʱ���ж���
 *    ���ú���
 *    void SWSStateMachine(SWS* slave)
 *    ��ʵ�ֶ�дʱ��Ŀ���
 *  3.�����ߴӻ��ļĴ���ʵ���ڴ��ļ��У��ɸ���ʵ������ԼĴ���������Ӧ���޸ģ�
 *    Ҳ�ɽ��Ĵ���������������������ļ���ʵ�֣���SWS�ṹ�������ӶԼĴ�����ַ
 *    �����ã����������У����Ե��ú���
 *    void SWSSetReg(uint8_t index, uint8_t data)
 *    ������ǰ��ϵͳ״̬�����ݵ�д��Ĵ�������������ȡ
      �������������е��ú���
 *    uint8_t SWSGetReg(uint8_t index)
 *    �����ϲ�ѯ�Ĵ����е�ֵ����������Ӧ�Ĳ���
 * @author Saber
 * @platform STM8S003����ֲ������ƽ̨��Ҫ��IO��ز��������޸�
 * @version V1.0.0
 * @date 2020��4��9��
 * @license 
 ******************************************************************************
 */

#include "single_wire_slave.h"
#include "user.h"

//�����ߴӻ��źŽŵķ�����������
typedef enum
{
  kIn = 0,  //����
  kOut,     //���
}SingleWirePinDir;

//�����ߴӻ�״̬
typedef enum
{
  kStateIdle = 0, //����
  kStateRegAddr,  //���ռĴ�����ַ�Ͷ�дλ
  kStateRd,       //������״̬�����ӻ�����״̬
  kStateWr,       //����д״̬�����ӻ�����״̬
} SWSState;

//������������һ��bit���ݿ��Ա������Ĵ�������֤�����㾡����bit��ƽ���м�
#define SAMPLE_NUMS   3

//ע�⣬����3�����໥������Ϊ�˼�С�����������ﲢû�н��й������㣬��Ҫһ���ֶ��޸�
#define REG_ADDR_BIT_LEN  1
#define REG_ADDR_BIT_MASK 1
#define REG_NUM           2
//��дλ�ĵ�ƽ
#define REG_WR      0
#define REG_RD      1
//���ݳ��ȣ�8bit
#define DATA_BIT_LEN      8

//�����ߴӻ��Ĵ���
static uint8_t reg_data[REG_NUM];

/*****************************************************************************
 * @brief �����ߴӻ���ʼ��
 * @history 
 *  1.@data 2020��4��10��
 *    @author Saber
 *    @note �����ɺ���
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
 * @brief �趨�����ߴӻ����ŷ���
 * @history 
 *  1.@data 2020��4��11��
 *    @author Saber
 *    @note �����ɺ���
 * @todo 
 ****************************************************************************/
static void SetPinDir(SWS * slave, SingleWirePinDir dir)
{
  slave->pin_dir = (uint8_t)dir;

  if (dir == kIn)  //����
  {
//    GPIO_Init(slave->port, slave->pin, GPIO_MODE_IN_FL_NO_IT);
    slave->port->CR2 &= (uint8_t)(~(slave->pin));
    slave->port->DDR &= (uint8_t)(~(slave->pin));
    slave->port->CR1 &= (uint8_t)(~(slave->pin));
//    slave->port->CR2 &= (uint8_t)(~(slave->pin));
  }
  else  //���
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
 * @brief �����ߴӻ�״̬��
 * @history 
 *  1.@data 2020��4��11��
 *    @author Saber
 *    @note �����ɺ���
 * @todo 
 ****************************************************************************/
void SWSStateMachine(SWS* slave)
{
  slave->cnt++;
  
  switch (slave->state)
  {
    case kStateIdle:
      //��⵽��������������������ͨ��
      if ((slave->port->IDR & slave->pin) == 0)
      {
        slave->cnt = 0;
        slave->bit_index = 0;
        slave->sample_index = SAMPLE_NUMS + 1;  //�ӳ�1���������������ʹ���������źŵ��м�
        slave->reg_addr = 0;
        slave->state = kStateRegAddr;
      }
      break;
    case kStateRegAddr:
      if (slave->cnt == slave->sample_index)
      {
        //�����źŵ�ƽ
        if ((slave->port->IDR & slave->pin) != 0)
        {
          slave->reg_addr |= (1 << slave->bit_index);
        }
        slave->bit_index++;
        slave->sample_index += SAMPLE_NUMS;
        //�жϵ�ַ�Ͷ�дλ�Ƿ��ȡ���
        if (slave->bit_index > REG_ADDR_BIT_LEN)
        {
          slave->verify_bit = 0;
          slave->bit_index = 0;
          if ((slave->reg_addr >> REG_ADDR_BIT_LEN) == REG_WR)  //ƥ��д��ַ������д����
          {
            slave->state = kStateWr;
            slave->reg_addr &= REG_ADDR_BIT_MASK;  //ȥ����дλ
            slave->reg_data = 0;
          }
          else  //ƥ�����ַ�����������
          {
            slave->state = kStateRd;
            slave->reg_addr &= REG_ADDR_BIT_MASK;  //ȥ����дλ
            slave->reg_data = reg_data[slave->reg_addr];
            slave->sample_index += SAMPLE_NUMS; //��ȴ�һ�����������ٽ������ݷ���
          }
        }
      }
      break;
    case kStateWr:
      if (slave->cnt == slave->sample_index)
      {
        //��ȡ����λ
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
          //У��ͨ��
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
        //�����ź���
        if (slave->pin_dir == kIn)
        {
          SetPinDir(slave, kOut);
          slave->port->ODR &= (uint8_t)(~slave->pin);
//          slave->bit_index = 0;
        }
        else
        {
          //��������
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
          //����У��λ
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
          else  //�ָ�IDLE״̬����������������Ϊ���븡��
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
 * @brief �趨�����ߴӻ��Ĵ�����ֵ�����������е��ã��趨�Ĵ���ֵ��ɹ�������ȡ
 * @history 
 *  1.@data 2020��4��11��
 *    @author Saber
 *    @note �����ɺ���
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
 * @brief ��ȡ�����ߴӻ��Ĵ�����ֵ�����������е��ã���ѯ������д����������
 * @history 
 *  1.@data 2020��4��11��
 *    @author Saber
 *    @note �����ɺ���
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


