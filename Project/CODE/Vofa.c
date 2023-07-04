#include "Vofa.h"
#include <string.h>
#include <stdio.h>
/************************************************
*文件说明：			基于vofa+上位机开发，发送及接收函数（JustFloat），发送图片。
*时    间：			2021.11.26
*备    注：			未经过大量测试，可能会有bug，注释已码好，可自行修复
								上位机下载地址https://www.vofa.plus/
***************************************************/

JustFloat UploadDataJF;
static VofaReceiveData_t VofaReceiveHead = VofaReceiveCreate("Head",NULL,UserFirstReceiveData);

/*************************************************
*	函数功能: VOFA+发送函数，用来显示波形
*	使用方法:	VOFAplusUpload_JustFloat(&UploadDataJF);
*	备注    : UploadDataJF是全局变量，直接赋值发送即可，
*			      如：UploadDataJF.CH_Data[0] = 1;
**************************************************/
void VOFAplusUpload_JustFloat(JustFloat* sendbuffer) 
{
	uint8_t Tail[4] = { 0x00, 0x00, 0x80, 0x7f };
	VofaSendStr((uint8_t*)sendbuffer->CH_Data, sizeof(float) * JFCH_COUNT);
	VofaSendStr(Tail, sizeof(Tail));
}

/*************************************************
*	函数功能: VOFA+接收解包函数，用于单片机接收解包
*	使用方法:	放入串口中断中
*						VOFAReceiveProcess(data);
*	备注    : 思路：对比名字，相同则对该地址的值进行赋值
*			      名字建议尽量短，减少比较时间。
*						变量地址为float型地址，不同文件要extern
*						链表，最后一个链表的next值必需为空！！！
*						创建新变量时，要在头文件进行声明！！！
**************************************************/
/****************************************************

						创建变量区

****************************************************/

//extern float kp,ki,kd;
//创建接收结构体链表，参数分别为：	  		名字   变量地址     下一结构体地址
/*
static VofaReceiveData_t p1 = VofaReceiveCreate("p1",&kp,&i1);
static VofaReceiveData_t i1 = VofaReceiveCreate("i1",&ki,&d1);
static VofaReceiveData_t d1 = VofaReceiveCreate("d1",&kd,&e1);
static VofaReceiveData_t e1 = VofaReceiveCreate("e1",&ex,NULL);
*/
/****************************************************

						函数区

****************************************************/
void VOFAReceiveProcess(uint8_t data)
{
	static uint8_t receiveData[20];	//若接收数据超过20个，可加大其数组个数，下同
	static uint8_t ucRxCnt = 0;	
	uint8_t _name[10],flag=0,_data[20],i=0;
	receiveData[ucRxCnt++] = data;
	
	if(data == '\n')	//帧尾为\n
	{
		for(i=0;i<ucRxCnt-1;i++)		//分别提取分号前与分号后
		{
			if(receiveData[i]==':'){flag=1;continue;}		//字符串中出现:
			if(flag==0)
				_name[i] = receiveData[i];					//提取名字
			else
				_data[(flag++)-1] = receiveData[i];	//提取数值
		}
		if(flag==0)		//字符串中未出现冒号，包错误
		{
			memset(receiveData,0,sizeof(receiveData));
			ucRxCnt=0;
			return;	//未出现‘:’，包错误，不进行赋值
		}
		else				//正常情况
		{
			for(VofaReceiveData_t *p = VofaReceiveHead.next;p!=NULL;p=p->next)	//链表循环，直到最后一个链表为空
				if(strcmp((char *)_name,(char *)p->name)==0)	//查找相同名
				{
					*p->data = atof((char *)_data);			//赋值
					memset(receiveData,0,sizeof(receiveData));	//清除，进行下一次接收
					ucRxCnt=0;
					return;
				}
		}
	}
	if(ucRxCnt>19)	//接收数据过长，清除
	{
		memset(receiveData,0,sizeof(receiveData));
		ucRxCnt=0;
	}
}

/*************************************************
*	函数功能: VOFA+发送函数，用来显示图像
*	使用方法:	VOFASendImg(mt9v03x_csi_image[0]);
*	备注    : 可以通过更改ID来显示多个图像，未适配
*			      逐飞无线串口实测188*120,491ms一张图，极致电竞
**************************************************/

void VOFASendImg(uint8_t *img)
{
	int preFrame[7] = {0, 0, 0, 0, 0, 0x7F800000, 0x7F800000};
	preFrame[0] = Vofa_IMG_ID;      // 此ID用于标识不同图片通道
  preFrame[1] = Vofa_IMG_SIZE;    // 图片数据大小
  preFrame[2] = Vofa_IMG_WIDTH;   // 图片宽度 
  preFrame[3] = Vofa_IMG_HEIGHT;  // 图片高度
  preFrame[4] = Vofa_IMG_FORMAT;  // 图片格式
	VofaSendStr((uint8_t *)preFrame, 7 * sizeof(int));
	VofaSendStr(img,preFrame[1]);
}