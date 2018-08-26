/**
********************************************************************************
* ��ࣺ�������ٺ��
* ģ�����ƣ�store_fifo.c
* ģ�鹦�ܣ����픵���惦fifo
* �������ڣ�
* �� �� �ߣ�
* ��    ����
********************************************************************************
**/

/***********************************
* ͷ�ļ�
***********************************/
#include "store_fifo.h"
#include "datatype.h"
#include "hardware.h"

/***********************************
* �ֲ�����
***********************************/

/***********************************
* �������x
***********************************/

//��ȡ�洢ͷ��Ϣ-EEPROM
//head_len: sizeof(store_head)
void read_store_head(STORE_HEAD* p_store_head)
{
	if(NULL == p_store_head)
		return;
	
	read_half_word_buf_from_eeprom(p_store_head->head_offset,(UINT16*)p_store_head, sizeof(STORE_HEAD)>>1);
}

//����洢ͷ��Ϣ-EEPROM
//head_len: sizeof(store_head)
BOOL save_store_head(STORE_HEAD* p_store_head)
{
	if(NULL == p_store_head)
		return FALSE;
	
	return save_half_word_buf_to_eeprom(p_store_head->head_offset,(UINT16*)p_store_head, sizeof(STORE_HEAD)>>1);
}

//��ʼ���洢ͷ
BOOL init_store_head(STORE_HEAD* p_store_head, \
	UINT32 head_offset, UINT32 data_offset,UINT8 data_page_num,UINT16 data_page_size)
{	
	if(NULL == p_store_head)
		return FALSE;
	
	p_store_head->head_offset = head_offset;
	p_store_head->data_offset = data_offset;
	p_store_head->data_page_num = data_page_num;
	p_store_head->data_page_size = data_page_size;
	
	//��ȡ��ʼ����־ 
	read_store_head(p_store_head);
	if(STORE_HEAD_HAVE_INIT == p_store_head->init_flag)
	{
		//�Ѿ���ʼ��,�t����
		return TRUE;
	}
	else
	{
		//δ��ʼ�������ʼ��
		p_store_head->init_flag = STORE_HEAD_HAVE_INIT;
		p_store_head->front = p_store_head->data_offset;
		p_store_head->rear = p_store_head->data_offset;
		
		//���浽flash
		return save_store_head(p_store_head);
	}
}

//���flash�洢������퓔���С
//����ֵ��len ��ʾ�洢��ҳ��
UINT16 get_store_data_page_count(STORE_HEAD* p_store_head)
{
	if(NULL == p_store_head)
		return 0;
	
	if(p_store_head->front <= p_store_head->rear)
	{
		return (p_store_head->rear - p_store_head->front) / p_store_head->data_page_size;
	}
	else 
	{
		//flash�洢��
		return p_store_head->data_page_num;
	}
}

//����һҳ����
BOOL save_store_data_one_page(STORE_HEAD* p_store_head, STORE_DATA_PAGE* p_store_data_page)
{	
	BOOL err;
	if(NULL == p_store_data_page || NULL == p_store_head)
		return FALSE;
	
	//����һ�дҳ
	err |= save_one_page_to_flash(p_store_head->rear, (UINT8*)p_store_data_page, p_store_head->data_page_size);
	
	//�����β��ַ
	p_store_head->rear += 1 * p_store_head->data_page_size;
	if(p_store_head->rear == p_store_head->data_page_num * p_store_head->data_page_size)//
	{
		p_store_head->rear = 0 + p_store_head->data_offset;//����
	}
	
	//������׵�ַ
	if(p_store_head->front == p_store_head->rear)
	{
		p_store_head->front += 1 * p_store_head->data_page_size;
	}	
	
	err |= save_store_head(p_store_head);
	
	return err;
}
//�����ҳ����
BOOL save_store_data_multi_page(STORE_HEAD* p_store_head, STORE_DATA_PAGE p_store_data_page[], UINT16 page_num)
{
	UINT16 cnt;
	BOOL err;
	
	if(NULL == p_store_data_page)
		return FALSE;
	
	//����ÿһҳ
	for(cnt = 0; cnt < page_num; cnt ++)
	{
		err |= save_store_data_one_page(p_store_head, &(p_store_data_page[cnt]));
	}
		
	//����ͷ��Ϣ
	err |= save_store_head(p_store_head);
	
	return err;
}

//�xһ퓔���
//�o�������t���� FALSE
//p_store_data_page�������x���Ĕ���
BOOL read_store_data_one_page(STORE_HEAD* p_store_head, STORE_DATA_PAGE* p_store_data_page, UINT32 logic_addr)
{
	UINT16 page_num;
	UINT32 read_addr;
	
	if(NULL == p_store_data_page || NULL == p_store_head)
		return FALSE;
	
	//�Д��Ƿ��Д���
	page_num = get_store_data_page_count(p_store_head);
	if(page_num == 0 || page_num > p_store_head->data_page_num)
		return FALSE;
	
	//�Д���Ч��ַ
	if(logic_addr > p_store_head->data_page_num)
		return FALSE;
	
	//�@���x��ַ
	read_addr = p_store_head->front + logic_addr * p_store_head->data_page_size;
	if(read_addr >= p_store_head->data_page_num)
	{
		read_addr -= p_store_head->data_page_num * p_store_head->data_page_size;
	}
	
	//�xһ퓔���
	read_one_page_from_flash(read_addr, (uint8_t*)p_store_data_page, sizeof(STORE_DATA_PAGE));
	
	return TRUE;
}



























