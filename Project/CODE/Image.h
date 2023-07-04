#ifndef Image_h
#define Image_h

#include "headfile.h"

#define black_size 30			 //�궨�����ڴ洢�ڵ�����Ķ�ά�����С
#define Threshold 145 + 98 //��ֵ����ֵ
//#define Threshold 145 //��ֵ����ֵ
extern unsigned int threshold;

//������ ����Ѱ�Ҿ��α߿� W 94 H 40
#define Guide_X		100
#define Guide_Y		20

//���س���
#define Ground_Length	7.0f
#define Ground_Width	5.0f


//Image�ź���
extern rt_sem_t Image_sem;

extern unsigned char Binaryzation_image[MT9V03X_CSI_H][MT9V03X_CSI_W];	//���ڴ洢��ֵ��ͼ��
extern float BlackPoint[black_size][2];   	 //�洢�ڵ�ʵ������
extern float Image_BlackPoint[black_size][2];//�洢�ڵ�ͼ���ϵ��������	������� ���ڹ۲��Ƿ�ѵ�ȫ���ҵ�
extern unsigned char black_num;	//��¼�ڵ�����
extern unsigned char current_black_num;	//��¼��ǰ�ڵ����
extern int black_order[black_size];//�洢�ڵ�˳��
extern short int high_point[2], low_point[2], center_point[2];//ͼƬ��������
extern short int left_edge_x,right_edge_x,top_edge_y,low_edge_y;//���߿��ƽ����������ƽ��������
extern float current_location_X, current_location_Y; //��ǰλ�õ�X Y����
extern float next_location_X, next_location_Y; 		   //��һ��λ�õ�X Y����
extern unsigned char image_mode;	//����ͷģʽѡ�� 0 �ر�����ͷ		1 ʶ�������		2 ·���滮

void image_binaryzation(void);//ͼ���ֵ��

unsigned char find_Rectangle(void);	//Ѱ��A4ֽ�еľ��α߿�

unsigned char find_blackPoint(unsigned char image[MT9V03X_CSI_H][MT9V03X_CSI_W]);	//Ѱ��A4ֽ�еĺڵ����������

unsigned char Path_planning(void);//·���滮
unsigned char Path_planning2(void);//·���滮2

unsigned char find_image(unsigned char image[MT9V03X_CSI_H][MT9V03X_CSI_W]);	//Ѱ��ͼƬ����

void Image_init(void); //Image�߳�

#endif