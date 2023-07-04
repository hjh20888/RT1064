#include "headfile.h"

//Image�ź���
rt_sem_t Image_sem;

unsigned char Binaryzation_image[MT9V03X_CSI_H][MT9V03X_CSI_W];	//���ڴ洢��ֵ��ͼ��
float BlackPoint[black_size][2] = {0.f,0.f};			//�洢�ڵ�ʵ������
float Image_BlackPoint[black_size][2] = {0.f,0.f};//�洢�ڵ�ͼ���ϵ��������	������� ���ڹ۲��Ƿ�ѵ�ȫ���ҵ�
unsigned char black_num = 0;	//��¼�ڵ�����
unsigned char current_black_num = 0;	//��¼��ǰ�ڵ����
int black_order[black_size] = {0};//�洢�ڵ�˳��
short int high_point[2], low_point[2], center_point[2];//ͼƬ��������
short int left_edge_x = 0,right_edge_x = 0,top_edge_y = 0,low_edge_y = 0;//���߿��ƽ����������ƽ��������
float current_location_X = 0.2f, current_location_Y = -0.25f; //��ǰλ�õ�X Y����
float next_location_X = 0.f, next_location_Y = 0.f; 		  //��һ��λ�õ�X Y����
unsigned char image_mode = 1;	//����ͷģʽѡ�� 0 �ر�����ͷ		1 ʶ�������		2 ·���滮
	
//-------------------------------------------------------------------------------------------------------------------
//  brief      �Ҷ�ͼ���ֵ��
//  return     void
//  since      v1.0
//  Sample usage:		image_binaryzation();				�����������ֵ -40 + 145
//-------------------------------------------------------------------------------------------------------------------

unsigned int threshold = 200;

void image_binaryzation(void)//ͼ���ֵ��
{
		unsigned char H,W; 
			
		for(H=0; H<MT9V03X_CSI_H-1; H++)
		{
				for(W=0; W<MT9V03X_CSI_W-1; W++)
				{
						
						if(*(*(mt9v03x_csi_image+H)+W) >= threshold)
								*(*(Binaryzation_image+H)+W) = 255;
						else
								*(*(Binaryzation_image+H)+W) = 0;
				}
		}
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      Ѱ��A4ֽ�еľ��α߿�
//  return     unsigned char
//  since      v2.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------

unsigned char find_Rectangle(void)	//Ѱ��A4ֽ�еľ��α߿�
{
		unsigned char H,W;
		unsigned char white_num=0; //�����İ׵����
		unsigned char first_find=1;//�����ж��Ƿ��һ�μ�⵽�ñ�
		
		
		//Ѱ���±߿�
		for(H=Guide_Y; H<MT9V03X_CSI_H; H++)
		{

				if(*(*(Binaryzation_image+H-1)+Guide_X)==255 && *(*(Binaryzation_image+H)+Guide_X)==0)
				{
						low_edge_y = H;
						break;
				}	
		}
		//Ѱ���ϱ߿�
		for(H=Guide_Y; H>0; H--)
		{
				if(*(*(Binaryzation_image+H+1)+Guide_X)==255 && *(*(Binaryzation_image+H)+Guide_X)==0)
				{
						top_edge_y = H;
						break;
				}	
		}
		
		//Ѱ����߿�
		for(W=Guide_X; W>0; W--)
		{
				if(*(*(Binaryzation_image+Guide_Y)+W+1)==255 && *(*(Binaryzation_image+Guide_Y)+W)==0)
				{
						left_edge_x = W;
						break;
				}	
		}
		//Ѱ���ұ߿�
		for(W=Guide_X; W<MT9V03X_CSI_W; W++)
		{
				if(*(*(Binaryzation_image+Guide_Y)+W-1)==255 && *(*(Binaryzation_image+Guide_Y)+W)==0)
				{
						right_edge_x = W;
						break;
				}	
		}
		
		return 1;
}	


//-------------------------------------------------------------------------------------------------------------------
//  brief      Ѱ��A4ֽ�еĺڵ����������
//  return     unsigned char
//  since      v2.0
//  Sample usage:		find_blackPoint(image_2);
//									black_num��ʵ�ʺڵ�����1	��Ϊ������һ��ԭ��
//-------------------------------------------------------------------------------------------------------------------

unsigned char find_blackPoint(unsigned char image[MT9V03X_CSI_H][MT9V03X_CSI_W])	//Ѱ��A4ֽ�еĺڵ���������� 
{
		unsigned char H,W;
		unsigned char find_flag=0;	//�ж��Ƿ��ֺڵ�
		unsigned char i,j;
		black_num=0;	//�ڵ�������0
	
		for(H=top_edge_y+2; H<low_edge_y-2; H++)
		{	
				for(W=left_edge_x+2; W<right_edge_x-2; W++)
				{
						if(image[H][W] < image[H][W-1])
						{
								black_num++;
								//��ֹ����Խ��
								if(black_num > black_size-1)
										black_num = 0;
							
								//�洢�ڵ�ͼ���ϵ��������	������� ���ڹ۲��Ƿ�ѵ�ȫ���ҵ�
								Image_BlackPoint[black_num][0]= W;
								Image_BlackPoint[black_num][1]= H;
								
								
								//��¼�ºڵ�ʵ������
								BlackPoint[black_num][0]= (float)(W*1.0f - left_edge_x*1.0f) / (float)(right_edge_x*1.0f - left_edge_x*1.0f) * Ground_Length; //- 0.1;
								BlackPoint[black_num][1]= (float)(low_edge_y*1.0f - H*1.0f) / (float)(low_edge_y*1.0f - top_edge_y*1.0f) * Ground_Width; //- 0.05;
							
								W+=2;		//������ֺڵ㣬��뾶8cm�ڲ��ټ��
								//��ֹ����Խ��
								if(W >= MT9V03X_CSI_W)
										W = MT9V03X_CSI_W - 1;
								
								find_flag=1;
						}			
				}
				if(find_flag==1)
				{
						H+=1;
						//��ֹ����Խ��
						if(H >= MT9V03X_CSI_H)
								H = MT9V03X_CSI_H - 1;
					
						find_flag=0;
				}	
		}
		
		
		black_num++;
		//��ֹ����Խ��
		if(black_num > black_size-1)
				black_num = 0;
		
		//��һ���������һ����Ϊԭ��
		BlackPoint[0][0] =  0.1f;
		BlackPoint[0][1] =  -0.2f;
		
		BlackPoint[black_num][0] = 0.f;
		BlackPoint[black_num][1] = -0.2f;
		
		return black_num;
}	


//-------------------------------------------------------------------------------------------------------------------
//  brief      ·���滮
//  return     unsigned char
//  since      v1.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------

unsigned char Path_planning(void)//·���滮
{
		int i,j,k;
		float length=0; //��¼�ܾ���
		float temp;		//���ڱȽ�
	
		//�滮ǰ���洢˳�����������
		for(i=0; i<black_size; i++)
				black_order[i]=0;
		
		for(i=1; i<=black_num-1; i++)
		{
				temp = 99999.0;
				
				for(j=1; j<=black_num-1; j++)
				{
						if(sqrt( pow((BlackPoint[j][0]-BlackPoint[black_order[i-1]][0]),2) +	pow((BlackPoint[j][1]-BlackPoint[black_order[i-1]][1]),2) ) < temp)
						{
								for(k=1;k<=i;k++)
								{
										if(j == black_order[k])	//�������źڵ��Ѿ���¼ ���˳�ѭ��������Ƚ�
										{
												break;
										}	
								}
								
								if(k==i+1)	
								{
										black_order[i] = j;
										temp = sqrt( pow((BlackPoint[j][0]-BlackPoint[black_order[i-1]][0]),2) +	pow((BlackPoint[j][1]-BlackPoint[black_order[i-1]][1]),2));
								}	
						}
				}
				
				length += sqrt( pow((BlackPoint[j][0]-BlackPoint[black_order[i-1]][0]),2) +	pow((BlackPoint[j][1]-BlackPoint[black_order[i-1]][1]),2));
		}
		
		return 0;
}	

unsigned char Path_planning2(void)//·���滮2
{
		int i = 0,j = 0,k = 0;
		float temp = 0;		//���ڱȽ�
		static int current_point = 0;
	
		temp = 99999.0;
		for(i=1; i<=black_num-1; i++)//15
		{
				if((sqrt( pow((current_location_X-BlackPoint[i][0]),2) +	pow((current_location_Y-BlackPoint[i][1]),2) ) < temp) && BlackPoint[i][0]!=-10 && i!=current_point)
				{
						temp = sqrt( pow((current_location_X-BlackPoint[i][0]),2) +	pow((current_location_Y-BlackPoint[i][1]),2) );
						next_location_X = BlackPoint[i][0];
						next_location_Y = BlackPoint[i][1];
					
						k = i;
				}
		}
		
		//�Ѿ��ҹ���������¼����
		BlackPoint[k][0] = -10;
		BlackPoint[k][1] = -10;
		
		//��ԭ�㸳ֵ �����ظ�������һ��
		BlackPoint[current_point][0] = -10;
		BlackPoint[current_point][1] = -10;
		
		current_point = k;
		
		return 0;
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      Ѱ��ͼƬ����
//  return     unsigned char
//  since      v1.0
//  Sample usage:		Y:55-65		X:90-100
//-------------------------------------------------------------------------------------------------------------------
																	/***************************
																	˼·��
																	�����½������Ͻ�ͬʱ����ͼ��
																	�ҵ���һ���ڰ�������¼����
																	***************************/

unsigned char find_image(unsigned char image[MT9V03X_CSI_H][MT9V03X_CSI_W])	//Ѱ��ͼƬ����
{
		unsigned char H,W;
		unsigned char i, j, k;
		unsigned char low_point_flag = 0, high_point_flag = 0;//��͵��־λ Ϊ1ʱ�ҵ���͵�
	
		for(H=MT9V03X_CSI_H-1; H>0; H--)//Ѱ����͵�����
		{
				for(W=0; W<MT9V03X_CSI_W-1; W++)
				{
						if(*(*(image+H)+W)==255)	//Ѱ����͵�����
						{
								low_point_flag = 1;
								low_point[0] = W;
								low_point[1] = H;
								break;
						}	
				}
				if(low_point_flag == 1)
						break;
		}
		
		for(H=0; H<MT9V03X_CSI_H-1; H++)//Ѱ����ߵ�����
		{
				for(W=MT9V03X_CSI_W-1; W>0; W--)
				{
						if(*(*(image+H)+W)==255)	//Ѱ����ߵ�����
						{
								high_point_flag = 1;
								high_point[0] = W;
								high_point[1] = H;
								break;
						}	
				}
				if(high_point_flag == 1)
						break;
		}
		
		if(abs(low_point[1] - high_point[1]) > 10)//�ų������ϰ׵�ĸ���
		{
				center_point[0] = (low_point[0] + high_point[0])/2;
				center_point[1] = (low_point[1] + high_point[1])/2;
			
				unsigned char h,w;
		}	
	
		return 0;
}	

/*******************************************************************************************************************
*  brief      Image�߳�
*  return     void
*  since      v1.0
*  Sample usage:	  ��ȡ���ź�����ʼ����	10msһ��
*******************************************************************************************************************/

void Image_entry(void *parameter)
{
    while(1)
    {
        //��ȡImage�ź��������û��������ȴ����ͷ�CPU����Ȩ
        rt_sem_take(Image_sem,RT_WAITING_FOREVER);
			
		rt_enter_critical();    //API�������ٽ������˳�ǰϵͳ���ᷢ���������
			
//        //ѡȡ���ͼ������Ѱ������
//        if(gpio_get(SWITCH_2)==0)
//        {
//                image_binaryzation();//ͼ���ֵ��
//        }	
//        
//        find_Rectangle();	//Ѱ��A4ֽ�еľ��α߿�3
//        
        if(gpio_get(SWITCH_2)==1)
        {
            black_num=4;

            BlackPoint[0][0] =  0.0f;
            BlackPoint[0][1] =  0.0f;
            
            BlackPoint[1][0] =  1.0f;
            BlackPoint[1][1] =  1.0f;

			BlackPoint[2][0] =  1.0f;
            BlackPoint[2][1] =  1.8f;

			BlackPoint[3][0] =  1.6f;
            BlackPoint[3][1] =  1.4f;
            
            BlackPoint[black_num][0] = 0.f;
            BlackPoint[black_num][1] = -0.2f;	
        }
//        
//        //Path_planning();//·���滮
        

        
        rt_exit_critical();    //API���˳��ٽ���
    }
}

void Image_init(void) //Image�߳�
{
    rt_thread_t tid;
	
	//MT9V03X����ͷ��ʼ�� ʹ��CSI�ӿ�
	//mt9v03x_csi_init();
	
    //����Image���ź�����8ms�ж��ͷ�һ��
    Image_sem = rt_sem_create("Image", 0, RT_IPC_FLAG_FIFO);		
    
    //����Image���߳� ���ȼ�19
    tid = rt_thread_create("Image", Image_entry, RT_NULL, 4096, 19, 10);
    
    //�����߳�
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}
