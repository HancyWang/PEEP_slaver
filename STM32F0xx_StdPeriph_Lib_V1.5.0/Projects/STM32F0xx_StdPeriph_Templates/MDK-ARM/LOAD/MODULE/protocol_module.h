/**
********************************************************************************
* ��ࣺ
* ģ�����ƣ�protocol.h
* ģ�鹦�ܣ�����λ�C�M��ͨ��
* �������ڣ�
* �� �� �ߣ�
* ��    ����
********************************************************************************
**/
#ifndef __PROTOCOL_H_
#define __PROTOCOL_H_
/***********************************
* ͷ�ļ�
***********************************/
#include "stm32f0xx.h"

/**********************************
*�궨��
***********************************/
#define SEND_DATA_BUF_LENGTH        300  // ������ 115200
#define CMD_BUFFER_LENGTH         350  //�궨���������ݰ��ĳ���

#define		PACK_HEAD_BYTE				 0xFF //ͷ�ļ���־
#define   MODULE_CMD_TYPE					0

//ģ������ʶ���
#define SEND_VOILD_EXP_FLAG_ID     0x01  //��Ч������
#define SEND_EXP_TRAIN_DATA_ID     0x02  //�洢����
#define SEND_EXP_TRAIN_DATA_CHECK_SUM_ID     0x03  //�洢����У���
#define SEND_BAT_PER_ID     0x04  //��ص���

//��������ʶ���
#define GET_EXP_TRAIN_DATA_ID     0x01  //��Ч������
//#define GET_BAT_PER_ID     				0x02  //��ص���
//#define PWM_VALUE_SET_ID   				0x03  //����PWM����
#define COMM_PARAMETER_ID					0x04  //��λ���·��Ĺ�����Ϣ
#define GET_FLASH_DATA_1_ID				0x05  //��λ���·��Ļ��flash�в���
#define GET_FLASH_DATA_2_ID				0x06  //��λ���·��Ļ��flash�в���
#define GET_FLASH_DATA_3_ID				0x0C

#define IS_RCV_PARA_FINISHED			0x07  //��λ���·���ѯ�������Ƿ�������
#define SEND_PARA_RCV_RESULT			0x08  //��λ�����ͽ��ղ����Ĵ�����
#define SEND_FLASH_DATA_1_ID      0x09  //����ԭʼ����̫��(434Bytes),�ֳ���֡��
#define SEND_FLASH_DATA_2_ID      0x0A  
#define SEND_FLASH_DATA_3_ID      0x0B
#define MODE1_PWM1_ID							0x11  //��λ���·���MODE1-PWM1
#define MODE1_PWM2_ID							0x12  //��λ���·���MODE1-PWM1
#define MODE1_PWM3_ID							0x13  //��λ���·���MODE1-PWM1
#define MODE2_PWM1_ID							0x21  //��λ���·���MODE1-PWM1
#define MODE2_PWM2_ID							0x22  //��λ���·���MODE1-PWM1
#define MODE2_PWM3_ID							0x23  //��λ���·���MODE1-PWM1
#define MODE3_PWM1_ID							0x31  //��λ���·���MODE1-PWM1
#define MODE3_PWM2_ID							0x32  //��λ���·���MODE1-PWM1
#define MODE3_PWM3_ID							0x33  //��λ���·���MODE1-PWM1

#define CAL_SENSOR_MMGH_1         0x50  //��λ���·���mmgh��ֵ1
#define CAL_SENSOR_MMGH_2         0x51	//��λ���·���mmgh��ֵ2
#define CAL_SENSOR_MMGH_3         0x52	//��λ���·���mmgh��ֵ3
#define CAL_SENSOR_SEND_TO_PC     0x60  //��λ���ش�ֵ����λ��

//������λ��д��flash����ʼ��ַ
#define FLASH_PAGE_SIZE      			((uint16_t)0x400)  //flashһҳ�Ĵ�СΪ1K
#define FLASH_START_ADDR   				((uint32_t)0x08000000) //flash��ʼ��ַ
#define FLASH_END_ADDR						((uint32_t)0x08004000) //flash������ַ
#define FLASH_WRITE_START_ADDR		((uint32_t)0x08000000+1024*30) //��ʼд��Ŀ�ʼ��ַ
//ע��070cb��page size��2K
#define FLASH_ADDR_RECORD_CYCLES	((uint32_t)0x08000000+1024*32) //��¼���Ե�Ȧ�������������Ϊ�˲��Ե�ص�����֧�Ŷ���Ȧ
#define FLASH_WRITE_END_ADDR      ((uint32_t)0x08004000)  //flashд��Ľ�����ַ

#define FLASH_PRESSURE_RATE_ADDR  ((uint32_t)0x08000000+1024*26) //��ʼд��Ŀ�ʼ��ַ

void FlashRead(uint32_t addr, uint32_t *p_data, uint16_t len);
uint32_t FlashReadWord(uint32_t addr);
//uint32_t FlashReadByte(char* addr);
uint8_t FlashReadByte(uint32_t addr);
//uint16_t FlashWrite(uint32_t addr, uint32_t *p_data, uint16_t len);
uint16_t FlashWrite(uint32_t addr, uint8_t *p_data, uint16_t len);
//uint16_t GetModeSelected(void);
/***********************************
* ���������
***********************************/


/***********************************
* ����
***********************************/
void protocol_module_process(uint8_t* pdata);

void protocol_module_send_exp_flag(uint8_t flag);
void protocol_module_send_train_data_one_page(uint8_t* buf, uint8_t len);
void protocol_module_send_train_data_check_sum(uint32_t check_sum);
void protocol_module_send_bat_per(uint8_t bat_per);
void get_comm_para_to_buf(uint8_t* pdata);
void get_parameter_to_buf_by_frameId(uint8_t* pdata,char frameId);
void send_prameter_fram1_to_PC(void);
void send_prameter_fram2_to_PC(void);
#endif
