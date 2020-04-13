
#ifndef __SINGLE_WIRE_MASTER_H__
#define __SINGLE_WIRE_MASTER_H__

#include "stm32f0xx_hal.h"

//����������
typedef struct
{
  uint8_t slave_no;     //�����ߴӻ��ı��
  
  GPIO_TypeDef* port;   //�ź�������port
  uint16_t pin;         //�ź�������pin
  uint8_t pin_pos;      //�ź�������pin��port�е�λ��
  uint8_t pin_dir;      //�ź��߷������뻹�����

  uint8_t rw_req;       //�����߶�ȡ����״̬
  uint8_t reg_addr;     //���������ڲ����ļĴ�����ַ
  uint8_t reg_data;     //���������ڲ����ļĴ���ֵ
  uint8_t state;        //������״̬��״̬

  uint16_t cnt;         //�����߲���ʱ��ļ�����
  uint16_t bit_index;   //���ڲ�����bit��λ��
  uint16_t sample_index;//������/����������ʱ���������λ��
  uint8_t verify_bit;   //У��λ
  uint8_t rd_wait_cnt;  //��ȡʱ�ȴ�����
} SWM;

void SWMInit(SWM * master,  uint8_t no, GPIO_TypeDef* port, uint16_t pin);

void SWMStateMachine(SWM * master);

uint8_t SWMRead(SWM * master, uint8_t reg);

uint8_t SWMWrite(SWM * master, uint8_t reg, uint8_t data);

#endif
