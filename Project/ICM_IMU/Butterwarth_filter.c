/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/
                ��Դ����������ѣ������ߵ���ʷ�Ѿ�֤�����ڹ�����������£���������Ŀ�Դ������������˰����ߣ��Է��ز�����
								Ŀ���Ƶķ�ʽ�в�ͨ���õĿ�Դ��Ŀ��Ҫ��רְ��Ա�����ۺ�����ְ��ֽ�ѧ��ͳ���û�����������ʵ������ɶԲ�
								Ʒ��һ�δ���������������������ۺϿ��ǣ������ɿؿ�Դ�����У������������˾���漤��ܴ��룬�汾������ʵ��
								���ܣ����ڹ�˾��Ʒ���������������Ӱ��ͻ�ѧϰ���������ȫ�����Ÿ��ͻ�ѧϰ���ͻ���ֲ�Ͷ��ο��������뱣����
								���Ȩ��
-----------------------------------------------------------------------------------------------------------------------/
*               ������ֻ��������ѧϰʹ�ã���Ȩ����Ȩ���������ƴ��Ŷӣ������ƴ��Ŷӽ��ɿس���Դ���ṩ�������ߣ�
*               ������ҪΪ�����ƴ��Ŷ��ṩ������δ��������ɣ����ý�Դ�����ṩ�����ˣ����ý�Դ����ŵ����Ϲ�����������أ� 
*               �������Դ�����Ĳ�����緢��������Ϊ�������ƴ��Ŷӽ���֮�Է��ɽ��������
-----------------------------------------------------------------------------------------------------------------------
*                                                 Ϊʲôѡ���������£�
*                                         �ж����ļ۸�������׵Ŀ�Դ�ɿأ�
*                                         ����ҵ������֮������µ��ۺ����
*                                         ׷�����û����飬��Ч����ѧϰ֮·��
*                                         ���²��ٹµ�������������տ�Դ�߶ȣ�
*                                         ��Ӧ���ҷ�ƶ���٣��ٽ��������ƹ�ƽ��
*                                         ��ʱ���ܶ�����ʣ����������˹�ͬ�塣 
-----------------------------------------------------------------------------------------------------------------------
*               ������Ϣ���ܶ���ֹ��ǰ�����������˳���������
*               ��Դ���ף���ѧ����ϧ��ף������Ϯ�����׳ɹ�������
*               ѧϰ�����ߣ��������Ƽ���DJI��ZEROTECH��XAG��AEE��GDU��AUTEL��EWATT��HIGH GREAT�ȹ�˾��ҵ
*               ��ְ�����뷢�ͣ�15671678205@163.com���豸ע��ְ����λ����λ��������
*               �����ƴ���Դ�ɿ�QQȺ��2��Ⱥ465082224��1��Ⱥ540707961����Ա������
*               CSDN���ͣ�http://blog.csdn.net/u011992534
*               �ſ�ID��NamelessCotrun����С��
*               Bվ��ѧ��Ƶ��https://space.bilibili.com/67803559/#/video
*               �ͻ�ʹ���ĵá��Ľ������������http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               �Ա����̣�https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               �ٶ�����:�����ƴ���Դ�ɿ�
*               ��˾����:www.nameless.tech
*               �������¹����׿�TI��Դ�ɿ���Ƴ��ԡ�֪��ר��:https://zhuanlan.zhihu.com/p/54471146
*               �޸�����:2021/04/30                    
*               �汾��������PLUS����GankerPilot_V3.0
*               ��Ȩ���У�����ؾ���
*               Copyright(C) 2019-2025 �人�������¿Ƽ����޹�˾ 
*               All rights reserved
-----------------------------------------------------------------------------------------------------------------------
*               ��Ҫ��ʾ��
*               �����Ա�����ת�ֵķɿء��������ѡ�����ѧ�ܵĶ����Խ��ۺ�Ⱥѧϰ������
*               ����ֱ�����������������������ϣ��������´�����������Ȩ�����˲��ý�
*               ���ϴ��봫���Ϲ��������أ�������ı��ΪĿ���������ϴ��룬�����д˲�
*               ���ߣ���˾����ǰ��֪����1���ڼ�ʱ�����������ѧУ����λ����������
*               ������ַ��Ϣ�ᱻ�����ڹ�˾�������ٷ�΢�Ź���ƽ̨���ٷ��������͡�֪��
*               ר���Լ��Ա�������ҳ���Թ�ʾ���棬����������Ϊ�����Ϊ�����۵㣬Ӱ��
*               ��ѧ���ҹ���������������ܿ�ͺ������˻����������������ء�
*               �����Ϊ����˾����ش���ʧ�ߣ����Է���;���������л���ĺ�����лл������
----------------------------------------------------------------------------------------------------------------------*/
#include "headfile.h"
#include "Butterwarth_filter.h"

/****************************************
Butterworth��ͨ�˲���������ʼ����http://blog.csdn.net/u011992534/article/details/73743955
***************************************/
/***********************************************************
@��������Butterworth_Parameter_Init
@��ڲ�������
@���ڲ�������
����������������˹��ͨ�˲�����ʼ��
@���ߣ�WUST
@���ڣ�2019��01��27��
*************************************************************/
void Butterworth_Parameter_Init(void)
{
}

/*************************************************
������:	float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
˵��:	���ٶȼƵ�ͨ�˲���
���:	float curr_input ��ǰ������ٶȼ�,�˲����������˲�������
����:	��
��ע:	2��Butterworth��ͨ�˲���
*************************************************/
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
	if(Buffer->Output_Butter[0]==0&&
		 Buffer->Output_Butter[1]==0&&
		 Buffer->Output_Butter[2]==0&&
		 Buffer->Input_Butter[0]==0&&
		 Buffer->Input_Butter[1]==0&&
		 Buffer->Input_Butter[2]==0)
	{
		Buffer->Output_Butter[0]=curr_input;
		Buffer->Output_Butter[1]=curr_input;
		Buffer->Output_Butter[2]=curr_input;
		Buffer->Input_Butter[0]=curr_input;
		Buffer->Input_Butter[1]=curr_input;
		Buffer->Input_Butter[2]=curr_input;
		return curr_input;
	}
	
  /* ���ٶȼ�Butterworth�˲� */
  /* ��ȡ����x(n) */
  Buffer->Input_Butter[2]=curr_input;
  /* Butterworth�˲� */
  Buffer->Output_Butter[2]=Parameter->b[0] * Buffer->Input_Butter[2]
													+Parameter->b[1] * Buffer->Input_Butter[1]
													+Parameter->b[2] * Buffer->Input_Butter[0]
													-Parameter->a[1] * Buffer->Output_Butter[1]
													-Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) ���б��� */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) ���б��� */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
	
	for(uint16_t i=0;i<3;i++)
	{
	  if(isnan(Buffer->Output_Butter[i])==1
			||isnan(Buffer->Input_Butter[i])==1)		
			{		
				Buffer->Output_Butter[0]=curr_input;
				Buffer->Output_Butter[1]=curr_input;
				Buffer->Output_Butter[2]=curr_input;
				Buffer->Input_Butter[0]=curr_input;
				Buffer->Input_Butter[1]=curr_input;
				Buffer->Input_Butter[2]=curr_input;
				return curr_input;
			}
	}	
  return Buffer->Output_Butter[2];
}



//https://blog.csdn.net/sszhouplus/article/details/43113505
//https://blog.csdn.net/shengzhadon/article/details/46784509
//https://blog.csdn.net/shengzhadon/article/details/46791903
//https://blog.csdn.net/shengzhadon/article/details/46803401
/***********************************************************
@��������Set_Cutoff_Frequency
@��ڲ�����float sample_frequent, float cutoff_frequent,
Butter_Parameter *LPF
@���ڲ�������
����������������˹��ͨ�˲�����ʼ��
@���ߣ�WUST
@���ڣ�2019��01��27��
*************************************************************/
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
  float fr = sample_frequent / cutoff_frequent;
  float ohm = tanf(M_PI_F / fr);
  float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
  if (cutoff_frequent <= 0.0f) {
    // no filtering
    return;
  }
  LPF->b[0] = ohm * ohm / c;
  LPF->b[1] = 2.0f * LPF->b[0];
  LPF->b[2] = LPF->b[0];
  LPF->a[0]=1.0f;
  LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
  LPF->a[2] = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}


/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/

