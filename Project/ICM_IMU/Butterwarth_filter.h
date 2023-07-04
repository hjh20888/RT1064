#ifndef _Butterwarth_filter_H_
#define _Butterwarth_filter_H_
/*----------------------------------------------------------------------------------------------------------------------/
        *               ������ֻ����Уѧ��ѧϰʹ�ã���Ȩ����Ȩ�����人�Ƽ���ѧ��
        *               �人�Ƽ���ѧ���ɿس���Դ���ṩ����Уѧ����
        *               ��Уѧ��ҪΪ�人�Ƽ���ѧ�ṩ������
        *               δ��ѧУ��ɣ����ý�Դ�����ṩ������
        *               ���ý�Դ����ŵ����Ϲ�����������أ�
        *               �������Դ�����Ĳ�����緢��������Ϊ��
        *               �人�Ƽ���ѧ����֮�Է��ɽ��������
-----------------------------------------------------------------------------------------------------------------------/
        *               ������Ϣ���ܶ���ֹ��ǰ�����������˳���������
        *               ��Դ���ף���ѧ����ϧ��ף������Ϯ�����׳ɹ�������
-----------------------------------------------------------------------------------------------------------------------/
	*		�����ƴ���Դ�ɿ� V1.1	�人�Ƽ���ѧ  By.YuYi
	*		CSDN����: http://blog.csdn.net/u011992534
	*               �ſ�ID��NamelessCotrunWUST
	*               �����ƴ���Դ�ɿ�QQȺ��540707961
        *               https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
        *               �ٶ�����:�����ƴ���Դ�ɿ�
        *               �޸�����:2017/10/30
        *               �汾��V1.1
        *               ��Ȩ���У�����ؾ���
        *               Copyright(C) �人�Ƽ���ѧ�人�Ƽ���ѧ 2017-2019
        *               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/

#define M_PI_F 3.141592653589793f
typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;


typedef struct
{
 float Output_Butter[3];
}Notch_Filter_BufferData;


typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;


float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF);
void Butterworth_Parameter_Init(void);



#endif


