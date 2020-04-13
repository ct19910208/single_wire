
#ifndef __SINGLE_WIRE_SLAVE_H__
#define __SINGLE_WIRE_SLAVE_H__

#include "stm8s.h"

typedef struct
{
  GPIO_TypeDef* port;   //�ź�������port
  uint16_t pin;         //�ź�������pin
  uint8_t pin_pos       //�ź�������pin��port�е�λ��
  uint8_t pin_dir;      //�ź��߷������뻹�����

  uint8_t reg_addr;     //���������ڲ����ļĴ�����ַ
  uint8_t reg_data;     //���������ڲ����ļĴ���ֵ
  uint8_t state;        //������״̬��״̬

  uint16_t cnt;         //�����߲���ʱ��ļ�����
  uint16_t bit_index;   //���ڲ�����bit��λ��
  uint16_t sample_index;//������/����������ʱ���������λ��
  uint8_t verify_bit;   //У��λ
} SWS;


void SWSInit(SWS* slave, GPIO_TypeDef* port, uint16_t pin);

void SWSStateMachine(SWS* slave);

void SWSSetReg(uint8_t index, uint8_t data);

uint8_t SWSGetReg(uint8_t index);

#endif