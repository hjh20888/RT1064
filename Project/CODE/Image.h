#ifndef Image_h
#define Image_h

#include "headfile.h"

#define black_size 30			 //宏定义用于存储黑点坐标的二维数组大小
#define Threshold 145 + 98 //二值化阈值
//#define Threshold 145 //二值化阈值
extern unsigned int threshold;

//辅助线 辅助寻找矩形边框 W 94 H 40
#define Guide_X		100
#define Guide_Y		20

//场地长宽
#define Ground_Length	7.0f
#define Ground_Width	5.0f


//Image信号量
extern rt_sem_t Image_sem;

extern unsigned char Binaryzation_image[MT9V03X_CSI_H][MT9V03X_CSI_W];	//用于存储二值化图像
extern float BlackPoint[black_size][2];   	 //存储黑点实际坐标
extern float Image_BlackPoint[black_size][2];//存储黑点图像上的相对坐标	用于描点 便于观察是否把点全部找到
extern unsigned char black_num;	//记录黑点数量
extern unsigned char current_black_num;	//记录当前黑点序号
extern int black_order[black_size];//存储黑点顺序
extern short int high_point[2], low_point[2], center_point[2];//图片中心坐标
extern short int left_edge_x,right_edge_x,top_edge_y,low_edge_y;//各边框的平均横坐标与平均纵坐标
extern float current_location_X, current_location_Y; //当前位置的X Y坐标
extern float next_location_X, next_location_Y; 		   //下一个位置的X Y坐标
extern unsigned char image_mode;	//摄像头模式选择 0 关闭摄像头		1 识别坐标点		2 路径规划

void image_binaryzation(void);//图像二值化

unsigned char find_Rectangle(void);	//寻找A4纸中的矩形边框

unsigned char find_blackPoint(unsigned char image[MT9V03X_CSI_H][MT9V03X_CSI_W]);	//寻找A4纸中的黑点个数与坐标

unsigned char Path_planning(void);//路径规划
unsigned char Path_planning2(void);//路径规划2

unsigned char find_image(unsigned char image[MT9V03X_CSI_H][MT9V03X_CSI_W]);	//寻找图片中心

void Image_init(void); //Image线程

#endif