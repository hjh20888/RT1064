#ifndef _Butterwarth_filter_H_
#define _Butterwarth_filter_H_
/*----------------------------------------------------------------------------------------------------------------------/
        *               本程序只供本校学生学习使用，版权著作权属于武汉科技大学，
        *               武汉科技大学将飞控程序源码提供给本校学生，
        *               本校学生要为武汉科技大学提供保护，
        *               未经学校许可，不得将源代码提供给他人
        *               不得将源代码放到网上供他人免费下载，
        *               更不能以此销售牟利，如发现上述行为，
        *               武汉科技大学将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
        *               生命不息、奋斗不止；前人栽树，后人乘凉！！！
        *               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
-----------------------------------------------------------------------------------------------------------------------/
	*		无名科创开源飞控 V1.1	武汉科技大学  By.YuYi
	*		CSDN博客: http://blog.csdn.net/u011992534
	*               优酷ID：NamelessCotrunWUST
	*               无名科创开源飞控QQ群：540707961
        *               https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
        *               百度贴吧:无名科创开源飞控
        *               修改日期:2017/10/30
        *               版本：V1.1
        *               版权所有，盗版必究。
        *               Copyright(C) 武汉科技大学武汉科技大学 2017-2019
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


