#include "iwtdg.h"
#include "stm32f0xx_iwdg.h"

void Init_iWtdg(unsigned char prer,unsigned short int rlr)
{
//	IWDG->KR=0X5555;       //����Ĵ�������        
//	IWDG->PR=prer;         // д���ʼ����Ƶֵ
//	IWDG->RLR=rlr;            // д���Զ�װ��ֵ
//	IWDG->KR=0XAAAA;     //�����Ĵ�������
//	IWDG->KR=0XCCCC;       //�������Ź�
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //??????? I ???  
	IWDG_SetPrescaler(prer);  //??? IWDG ????:?? IWDG ????  
	IWDG_SetReload(rlr);  //??? IWDG ????  
	IWDG_ReloadCounter(); //??? IWDG ??????????? IWDG ???  
	IWDG_Enable();  //??? IWDG 
}

void IWDG_Feed(void)
{
   IWDG_ReloadCounter();//reload                                 
}

