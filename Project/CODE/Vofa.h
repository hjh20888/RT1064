#ifndef _Vofa_H_
#define _Vofa_H_
#include "headfile.h"
//#include "main.h"

#define JFCH_COUNT 6	//����float�͸���

typedef struct {//����ͨ����ʽ
	float CH_Data[JFCH_COUNT];
} JustFloat;

typedef struct VofaReceiveData {//���ݽ��ո�ʽ
	const char* const name;
	float *data;
	struct VofaReceiveData *next;
	
} VofaReceiveData_t;

extern JustFloat UploadDataJF;
//extern VofaReceiveData_t p1,i1,d1,e1;//�����Ҫ�������ļ�ʹ����Щ������Ӧ��ɾ������ʱ�ľ�̬��������extern����,һ�㲻��Ҫ
static VofaReceiveData_t p1,i1,d1,e1;		//����Ҫ������
/*************************************************************

			��ֲ��Ҫ��������Ϊ��������define���Լ�include �ļ�

*************************************************************/
//��UART6_SendStr"Ϊ�����ַ������޸Ķ�Ӧ���ں��Լ��������ݡ�
#define VofaSendStr(data,size)	uart_putbuff(USART_8,data,size)
//��ֲʱ������UserFirstReceiveData��ֵ���ɸı��Զ���ĵ�һ�����սṹ�壨��ͷ��
#define UserFirstReceiveData	&p1



//ͼ���ʽö��
enum ImgFormat {
    Format_Invalid,
    Format_Mono,
    Format_MonoLSB,
    Format_Indexed8,
    Format_RGB32,
    Format_ARGB32,
    Format_ARGB32_Premultiplied,
    Format_RGB16,
    Format_ARGB8565_Premultiplied,
    Format_RGB666,
    Format_ARGB6666_Premultiplied,
    Format_RGB555,
    Format_ARGB8555_Premultiplied,
    Format_RGB888,
    Format_RGB444,
    Format_ARGB4444_Premultiplied,
    Format_RGBX8888,
    Format_RGBA8888,
    Format_RGBA8888_Premultiplied,
    Format_BGR30,
    Format_A2BGR30_Premultiplied,
    Format_RGB30,
    Format_A2RGB30_Premultiplied,
    Format_Alpha8,
    Format_Grayscale8,
    
    // ���¸�ʽ����ʱ��IMG_WIDTH��IMG_HEIGHT����Ҫǿ��ָ��������Ϊ-1����
    Format_BMP,
    Format_GIF,
    Format_JPG,
    Format_PNG,
    Format_PBM,
    Format_PGM,
    Format_PPM,
    Format_XBM,
    Format_XPM,
    Format_SVG,
};


#define Vofa_IMG_ID					1
#define Vofa_IMG_SIZE				188*120		
#define Vofa_IMG_WIDTH			188
#define Vofa_IMG_HEIGHT 		120
#define Vofa_IMG_FORMAT			Format_Grayscale8		

//�ṹ�������ʼ��
#define VofaReceiveCreate(_name,_data,_next)	{.name=_name,.data=_data,.next=_next}

void VOFAplusUpload_JustFloat(JustFloat* sendbuffer);
void VOFAReceiveProcess(uint8_t data);
void VOFASendImg(uint8_t *img);

#endif
