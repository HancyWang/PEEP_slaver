/*******************************************************************************
* ��Ȩ���� :  
* �汾��   :  1.0
* �ļ���   :  CMD_receive.h
* �������� : 
* ����     :  
* ����˵�� :  ���������

*******************************************************************************/
#ifndef  __CMD_RECEIVE_H__
#define  __CMD_RECEIVE_H__
/*******************************************************************************
*                                 ͷ�ļ�����
*******************************************************************************/
#include <DataType.h>

typedef enum
{
	BUF1_LENGTH = 64+8,
	BUF2_LENGTH = 64+8
}BUF_LENGTH;

typedef struct
{
	UINT8  m_Buf2[BUF2_LENGTH];                //��������ַ�������������
	INT32  m_Buf2Wptr;                         //��������дָ��
	INT32  m_Buf2Rptr;                         //���������ָ��
	INT32  m_Cnt;
	INT32  m_Position;

	UINT8  m_Buf1[BUF1_LENGTH];                //��������ַ���������  	  
}CMD_Receive;

void Init_Receive(CMD_Receive *p_RX);
void Buf2Write(CMD_Receive* p_RX, UINT8 data);
UINT8 Buf2Read(CMD_Receive* p_RX);
INT32 GetBuf2Length(CMD_Receive* p_RX);
void ReceiveData(CMD_Receive* p_RX);
#endif
/*************************************����*************************************/
