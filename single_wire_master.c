/**
 ******************************************************************************
 *
 *                ��Ȩ���� (C), 2020-2030, �Ϻ�������Ϣ�Ƽ����޹�˾
 *
 ******************************************************************************
 * @file single_wire_master.c
 * @brief ������Э����������
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
 *  1.��Ҫ��һ���ӻ�����ͨ��ʱ�����Ƚ���SWM������Ȼ�����
 *    void SWMInit(SWM * master, uint8_t no, GPIO_TypeDef* port, uint16_t pin)
 *    ������г�ʼ��
 *  2.������Э����Ҫ��׼��ʱ����ƣ����дʱ���ɶ�ʱ�����ƣ�ͨ���ڶ�ʱ���ж���
 *    ���ú���
 *    void SWMStateMachine(SWM * master)
 *    ��ʵ�ֶ�дʱ��Ŀ���
 *  3.��Ҫ���ж�дʱ���ֱ����
 *    uint8_t SWMRead(SWM * master, uint8_t reg)
 *    uint8_t SWMWrite(SWM * master, uint8_t reg, uint8_t data)
 *    �����д���������ǰ����IDLE״̬��������ɹ�������1����������ʧ�ܣ���
 *    ��0
 *  4.��Ҫ���û���������д��ȡ��ɻص�����
 *    __weak void SingleWireRdFinishCallback(SWM * master)
 *    �Զ�ȡ�������ݽ��д����ú������ڶ�д״̬�����ƺ����е��õģ�Ҳ������
 *    ʱ����ƶ�ʱ���ж��е��õģ�ע����д�ĺ�������̫��ʱ
 *  5.ͬһ������������ͬʱ���ж���д��Ҳ����˵��Э��Ϊ��˫�������Ƕ������
 *    �����벻ͬ�ӻ���ͨ�ſ���ͬʱ���У��������š�ע�⣬��������Խ�࣬ʱ���
 *    �ƶ�ʱ���жϵĴ�������ͻ�Խ�࣬��MCU������Ҫ���Խ�ߡ�
 * @author Saber
 * @platform STM32F030 HAL����ֲ������ƽ̨��Ҫ��IO��ز��������޸�
 * @version V1.0.0
 * @date 2020��4��11��
 * @license 
 ******************************************************************************
 */

#include "single_wire_master.h"
#include "main.h"

//�������źŽŵķ������뻹�����
typedef enum
{
  kIn = 0,  //����
  kOut,     //���
}SingleWirePinDir;

//�����߶�д����״̬
typedef enum
{
  kIdle = 0,  //���У����������д
  kRdReq,     //���ڶ�slave
  kWrReq,     //����дslave
} SingleWireRwReq;

//������״̬����״̬
typedef enum
{
  kStateIdle = 0, //����
  kStateRegAddr,  //д��ַ
  kStateRd,       //��
  kStateRdPre,    //��֮ǰ�Ĳ�������ʱ�������趨Ϊ���룬һ����⵽�͵�ƽ�������״̬
  kStateWr,       //д״̬
} SWMState;

//������������һ��bit���ݿ��Ա������Ĵ�������֤�����㾡����bit��ƽ���м�
#define SAMPLE_NUMS   3

//ע�⣬����3�����໥������Ϊ�˼�С�����������ﲢû�н��й������㣬��Ҫһ���ֶ��޸�
#define REG_ADDR_BIT_LEN  1
#define REG_ADDR_BIT_MASK 1
#define REG_NUM           2
//��дλ�ĵ�ƽ
#define REG_WR      GPIO_PIN_RESET
#define REG_RD      GPIO_PIN_SET
//���ݳ��ȣ�8bit
#define DATA_BIT_LEN      8
//���ȴ��������������ȡʱ�ȴ�ʱ�䳬����ʱ�伴��Ϊ��ȡʧ�ܣ�����IDLE״̬
#define RD_WAIT_MAX_CNT   10

/*****************************************************************************
 * @brief �����߳�ʼ��
 * @history 
 *  1.@data 2020��4��9��
 *    @author Saber
 *    @note �����ɺ���
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
 * @brief �趨���������ŷ������뻹�����
 * @history 
 *  1.@data 2020��4��9��
 *    @author Saber
 *    @note �����ɺ���
 * @todo 
 ****************************************************************************/
static void SetPinDir(SWM * master, SingleWirePinDir dir)
{
  uint32_t temp;
//  GPIO_InitTypeDef GPIO_InitStruct = {0};

  master->pin_dir = (uint8_t)dir;
//  GPIO_InitStruct.Pin = master->pin;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
  if (dir == kIn)  //����
  {
//    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//    HAL_GPIO_Init(master->port, &GPIO_InitStruct);
    //�趨Ϊ����
    master->port->MODER &= ~(GPIO_MODER_MODER0 << (master->pin_pos * 2U));
//    //���������ڳ�ʼ���Ѿ��趨Ϊ���������治���ٸı䣬���Կ��Բ�ִ�У���Լʱ��
//    temp = master->port->PUPDR;
//    CLEAR_BIT(temp, GPIO_PUPDR_PUPDR0 << (master->pin * 2U));
//    SET_BIT(temp, (GPIO_PULLUP) << (master->pin * 2U));
//    master->port->PUPDR = temp;
    //
  }
  else  //���
  {
//    HAL_GPIO_WritePin(master->port, master->pin, GPIO_PIN_SET);
    //����ߵ�ƽ
    master->port->BSRR = (uint32_t)master->pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
//    HAL_GPIO_Init(master->port, &GPIO_InitStruct);
    //�趨Ϊ���
    temp = master->port->MODER;
    CLEAR_BIT(temp, GPIO_MODER_MODER0 << (master->pin_pos * 2U));   
    SET_BIT(temp, (GPIO_MODE_OUTPUT_OD & 0x00000003U) << (master->pin_pos * 2U));
    master->port->MODER = temp;
    //IO������Ϊ����
    master->port->OSPEEDR |= GPIO_SPEED_FREQ_HIGH << (master->pin_pos * 2U); 
    //����ΪOD
    master->port->OTYPER |= ((GPIO_MODE_OUTPUT_OD & 0x00000010U) >> 4U)
      << master->pin_pos;
  }
}

/*****************************************************************************
 * @brief �����߶�ȡ����ʱ�Ļص����������ﶨ��Ϊweak����Ҫ��Ӧ�ó�������д��
 * ���������ڶ�ȡ����ʱ������������ȡ�����д���ע�⣬���ڸú������ڵ����ߵ�
 * ��ʱ���ж��е��ã���д�ĸú�������̫��ʱ
 * @history 
 *  1.@data 2020��4��9��
 *    @author Saber
 *    @note �����ɺ���
 * @todo 
 ****************************************************************************/
__weak void SingleWireRdFinishCallback(SWM * master)
{
  UNUSED(master);
}

/*****************************************************************************
 * @brief ������Э��ʱ�����״̬�����ڵ����߶�ʱ���е��ã���ʱ����Ҫ�о�׼�Ķ�
 * ʱ����
 * @history 
 *  1.@data 2020��4��10��
 *    @author Saber
 *    @note �����ɺ���
 * @todo 
 ****************************************************************************/
void SWMStateMachine(SWM * master)
{
  master->cnt++;
  
  switch (master->state)
  {
    case kStateIdle:
      //�ж�д�������������ź���
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
        if (master->bit_index < REG_ADDR_BIT_LEN) //д��ַ
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
        else  //д��дλ
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
        if (master->bit_index < DATA_BIT_LEN) //д����
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
        else if (master->bit_index == DATA_BIT_LEN) //дУ��λ
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
        if (master->pin_dir == kOut)  //���ź�������Ϊ����
        {
          SetPinDir(master, kIn);
          master->rd_wait_cnt = 0;
        }
        else
        {
          //��⵽�͵�ƽ�������ӻ���ʼ����������
//          if (HAL_GPIO_ReadPin(master->port, master->pin) == GPIO_PIN_RESET)
          if ((master->port->IDR & master->pin) == GPIO_PIN_RESET)
          {
            master->state = kStateRd;
            master->reg_data = 0;
            master->sample_index += SAMPLE_NUMS;
          }
          //��ʱ��û�еȵ��͵�ƽ��ֹͣ���գ��ָ�IDLE״̬
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
          //У��ͨ��
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
 * @brief �����߶��������ȡʱ����Ҫ��ȡ�ļĴ�����ַ���룬��ȡʱ����״̬�����ƣ�
 * ��ȡ���ʱ����ö�ȡ�ص�����������������ݣ�����ɹ�����1�����򷵻�0
 * @history 
 *  1.@data 2020��4��10��
 *    @author Saber
 *    @note �����ɺ���
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
 * @brief ������д����������дʱ����Ҫд�ļĴ�����ַ��ֵ���룬дʱ����״̬����
 * �ƣ�����ɹ�����1�����򷵻�0
 * @history 
 *  1.@data 2020��4��10��
 *    @author Saber
 *    @note �����ɺ���
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


