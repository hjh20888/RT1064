#ifndef _Vofa_H_
#define _Vofa_H_
#include "headfile.h"
//#include "main.h"

#define JFCH_COUNT 6	//发送float型个数

typedef struct {//数据通道格式
	float CH_Data[JFCH_COUNT];
} JustFloat;

typedef struct VofaReceiveData {//数据接收格式
	const char* const name;
	float *data;
	struct VofaReceiveData *next;
	
} VofaReceiveData_t;

extern JustFloat UploadDataJF;
//extern VofaReceiveData_t p1,i1,d1,e1;//如果需要在其他文件使用这些变量，应当删除定义时的静态声明，再extern声明,一般不需要
static VofaReceiveData_t p1,i1,d1,e1;		//必须要先声明
/*************************************************************

			移植主要更改内容为下面两个define，以及include 文件

*************************************************************/
//“UART6_SendStr"为发送字符串，修改对应串口号以及参数内容。
#define VofaSendStr(data,size)	uart_putbuff(USART_8,data,size)
//移植时，更改UserFirstReceiveData的值即可改变自定义的第一个接收结构体（表头）
#define UserFirstReceiveData	&p1



//图像格式枚举
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
    
    // 以下格式发送时，IMG_WIDTH和IMG_HEIGHT不需要强制指定，设置为-1即可
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

//结构体参数初始化
#define VofaReceiveCreate(_name,_data,_next)	{.name=_name,.data=_data,.next=_next}

void VOFAplusUpload_JustFloat(JustFloat* sendbuffer);
void VOFAReceiveProcess(uint8_t data);
void VOFASendImg(uint8_t *img);

#endif
