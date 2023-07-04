#include "Vofa.h"
#include <string.h>
#include <stdio.h>
/************************************************
*�ļ�˵����			����vofa+��λ�����������ͼ����պ�����JustFloat��������ͼƬ��
*ʱ    �䣺			2021.11.26
*��    ע��			δ�����������ԣ����ܻ���bug��ע������ã��������޸�
								��λ�����ص�ַhttps://www.vofa.plus/
***************************************************/

JustFloat UploadDataJF;
static VofaReceiveData_t VofaReceiveHead = VofaReceiveCreate("Head",NULL,UserFirstReceiveData);

/*************************************************
*	��������: VOFA+���ͺ�����������ʾ����
*	ʹ�÷���:	VOFAplusUpload_JustFloat(&UploadDataJF);
*	��ע    : UploadDataJF��ȫ�ֱ�����ֱ�Ӹ�ֵ���ͼ��ɣ�
*			      �磺UploadDataJF.CH_Data[0] = 1;
**************************************************/
void VOFAplusUpload_JustFloat(JustFloat* sendbuffer) 
{
	uint8_t Tail[4] = { 0x00, 0x00, 0x80, 0x7f };
	VofaSendStr((uint8_t*)sendbuffer->CH_Data, sizeof(float) * JFCH_COUNT);
	VofaSendStr(Tail, sizeof(Tail));
}

/*************************************************
*	��������: VOFA+���ս�����������ڵ�Ƭ�����ս��
*	ʹ�÷���:	���봮���ж���
*						VOFAReceiveProcess(data);
*	��ע    : ˼·���Ա����֣���ͬ��Ըõ�ַ��ֵ���и�ֵ
*			      ���ֽ��龡���̣����ٱȽ�ʱ�䡣
*						������ַΪfloat�͵�ַ����ͬ�ļ�Ҫextern
*						�������һ�������nextֵ����Ϊ�գ�����
*						�����±���ʱ��Ҫ��ͷ�ļ���������������
**************************************************/
/****************************************************

						����������

****************************************************/

//extern float kp,ki,kd;
//�������սṹ�����������ֱ�Ϊ��	  		����   ������ַ     ��һ�ṹ���ַ
/*
static VofaReceiveData_t p1 = VofaReceiveCreate("p1",&kp,&i1);
static VofaReceiveData_t i1 = VofaReceiveCreate("i1",&ki,&d1);
static VofaReceiveData_t d1 = VofaReceiveCreate("d1",&kd,&e1);
static VofaReceiveData_t e1 = VofaReceiveCreate("e1",&ex,NULL);
*/
/****************************************************

						������

****************************************************/
void VOFAReceiveProcess(uint8_t data)
{
	static uint8_t receiveData[20];	//���������ݳ���20�����ɼӴ��������������ͬ
	static uint8_t ucRxCnt = 0;	
	uint8_t _name[10],flag=0,_data[20],i=0;
	receiveData[ucRxCnt++] = data;
	
	if(data == '\n')	//֡βΪ\n
	{
		for(i=0;i<ucRxCnt-1;i++)		//�ֱ���ȡ�ֺ�ǰ��ֺź�
		{
			if(receiveData[i]==':'){flag=1;continue;}		//�ַ����г���:
			if(flag==0)
				_name[i] = receiveData[i];					//��ȡ����
			else
				_data[(flag++)-1] = receiveData[i];	//��ȡ��ֵ
		}
		if(flag==0)		//�ַ�����δ����ð�ţ�������
		{
			memset(receiveData,0,sizeof(receiveData));
			ucRxCnt=0;
			return;	//δ���֡�:���������󣬲����и�ֵ
		}
		else				//�������
		{
			for(VofaReceiveData_t *p = VofaReceiveHead.next;p!=NULL;p=p->next)	//����ѭ����ֱ�����һ������Ϊ��
				if(strcmp((char *)_name,(char *)p->name)==0)	//������ͬ��
				{
					*p->data = atof((char *)_data);			//��ֵ
					memset(receiveData,0,sizeof(receiveData));	//�����������һ�ν���
					ucRxCnt=0;
					return;
				}
		}
	}
	if(ucRxCnt>19)	//�������ݹ��������
	{
		memset(receiveData,0,sizeof(receiveData));
		ucRxCnt=0;
	}
}

/*************************************************
*	��������: VOFA+���ͺ�����������ʾͼ��
*	ʹ�÷���:	VOFASendImg(mt9v03x_csi_image[0]);
*	��ע    : ����ͨ������ID����ʾ���ͼ��δ����
*			      ������ߴ���ʵ��188*120,491msһ��ͼ�����µ羺
**************************************************/

void VOFASendImg(uint8_t *img)
{
	int preFrame[7] = {0, 0, 0, 0, 0, 0x7F800000, 0x7F800000};
	preFrame[0] = Vofa_IMG_ID;      // ��ID���ڱ�ʶ��ͬͼƬͨ��
  preFrame[1] = Vofa_IMG_SIZE;    // ͼƬ���ݴ�С
  preFrame[2] = Vofa_IMG_WIDTH;   // ͼƬ��� 
  preFrame[3] = Vofa_IMG_HEIGHT;  // ͼƬ�߶�
  preFrame[4] = Vofa_IMG_FORMAT;  // ͼƬ��ʽ
	VofaSendStr((uint8_t *)preFrame, 7 * sizeof(int));
	VofaSendStr(img,preFrame[1]);
}