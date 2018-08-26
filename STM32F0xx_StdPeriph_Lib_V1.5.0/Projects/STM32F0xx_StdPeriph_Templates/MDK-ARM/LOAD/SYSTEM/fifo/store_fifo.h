/**
********************************************************************************
* ��ࣺ�������ٺ��
* ģ�����ƣ�store_fifo.c
* ģ�鹦�ܣ����픵���惦fifo����������퓴惦�cflash�У��͑���ʹ������
1. ���ȣ����xһ������
	UINT32 head_offset;//�^�惦�^��EEPROM������ʼ��ַ
	UINT8 data_page_num;//�����惦�^��FLASH�����洢ҳ��
	UINT16 data_page_size;//�����惦�^����FLASH�����洢��ҳ�Ĵ�С
	UINT32 data_offset;//�����惦�^����FLASH������ʼ��ַ
2. ���x�^��Ϣ���w�Y��STORE_HEAD
3. ���x������Ϣ�Y�����wSTORE_DATA_PAGE���������¶��x�Y��
4. ��ʼ���^��Ϣ��init_store_head
5. �x������
ע�⣺
1)data_page_num��0~255
2)data_page_size:��FLASH����퓴�С��һ���128B,1KB,2KB
3)head_offset��EEPROM��
4)data_offset:FLASH��
* �������ڣ�
* �� �� �ߣ�
* ��    ����
********************************************************************************
**/

#ifndef __STORE_FIFO_H
#define __STORE_FIFO_H
 
#include "datatype.h" 
#include "stm32f0xx.h"

//�^�惦�^�򡪡����Д��^�Ƿ��ʼ���Ę�־ֵ
#define STORE_HEAD_HAVE_INIT			0x56

//FLASH�洢��Ϣͷ���ݽṹ�� 
typedef struct
{
	UINT8 init_flag;//�^��ʼ����־
	UINT8 data_page_num;//�����惦�^�򡪡��洢ҳ��
	UINT16 data_page_size;//�����惦�^�򡪡��洢��ҳ�Ĵ�С
	UINT32 head_offset;//�^�惦�^�򡪡���ʼ��ַ
	UINT32 data_offset;//�����惦�^�򡪡���ʼ��ַ
	UINT32 front;	//��ͷ��ַ,0~STORE_DATA_PAGE_NUM
	UINT32 rear; 	//��β��ַ,0~STORE_DATA_PAGE_NUM
}STORE_HEAD;

//ÿҳ���ݽṹ
typedef struct
{
	UINT8* data_page_buf;
}STORE_DATA_PAGE;

//���ⲿ���õĺ���
BOOL init_store_head(STORE_HEAD* p_store_head, \
	UINT32 head_offset, UINT32 data_offset,UINT8 data_page_num,UINT16 data_page_size);
void read_store_head(STORE_HEAD* p_store_head);
BOOL save_store_head(STORE_HEAD* p_store_head);
UINT16 get_store_data_page_count(STORE_HEAD* p_store_head);
BOOL save_store_data_one_page(STORE_HEAD* p_store_head, STORE_DATA_PAGE* p_store_data_page);
BOOL save_store_data_multi_page(STORE_HEAD* p_store_head, STORE_DATA_PAGE p_store_data_page[], UINT16 page_num);
BOOL read_store_data_one_page(STORE_HEAD* p_store_head, STORE_DATA_PAGE* p_store_data_page, UINT32 logic_addr);
#endif
