#include "headfile.h"

//-------------------------------------------------------------------------------------------------------------------
//  brief      ���������� �߿� �����
//  return     void
//  since      v1.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------

void drawing_drawing_drawing(void) //���������� �߿� �����
{
		unsigned char i,j,H,W;
	
		//����������
		for(H=0; H<MT9V03X_CSI_H; H++)
		{
				ips114_drawpoint(Guide_X,H,RED);
		}
		for(W=0; W<MT9V03X_CSI_W; W++)
		{
				ips114_drawpoint(W,Guide_Y,RED);
		}
		
		//������α߿�
		for(H=top_edge_y; H<low_edge_y; H++)
		{
				ips114_drawpoint(left_edge_x,H,RED); //�����߿�
				ips114_drawpoint(right_edge_x,H,RED);//����ұ߿�
		}
		
		for(W=left_edge_x; W<right_edge_x; W++)
		{
				ips114_drawpoint(W,top_edge_y,RED);	//����ϱ߿�
				ips114_drawpoint(W,low_edge_y,RED);	//����±߿�
		}
		
		//����ڵ�
		for(i=1; i<black_num; i++)
		{
				for(H=Image_BlackPoint[i][1]-2; H<Image_BlackPoint[i][1]+2; H++)
				{
						for(W=Image_BlackPoint[i][0]-2; W<Image_BlackPoint[i][0]+2; W++)
						{
								ips114_drawpoint(W,H,RED);
						}
				}
		}
		
		//��ʾ�ڵ���� ��ʵ�ʺڵ������1(�����˸�ԭ��)
		ips114_showstr(0, 7, "black_num:");
		ips114_showint16(75, 7, black_num);
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      IMU������ʾ
//  return     void
//  since      v1.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------

void IMU_show(void)	//IMU������ʾ
{
		//IMU Yaw
		ips114_showstr(0, 0, "Yaw:");
		ips114_showint16(75, 0, Yaw);
		
		//imu_pos
		ips114_showstr(0, 1, "imu_pos.x:");
		ips114_showint16(75, 1, imu_pos.x);
		ips114_showstr(0, 2, "imu_pos.y:");
		ips114_showint16(75, 2, imu_pos.y);
	
		//imu_speed
		ips114_showstr(0, 3, "imu_speed.x:");
		ips114_showint16(80, 3, imu_speed.x);
		ips114_showstr(0, 4, "imu_speed.y:");
		ips114_showint16(80, 4, imu_speed.y);
}	

/********************************************************************************************************************
*  brief      display��ʾ�߳�
*  return     void
*  since      v1.0
*  Sample usage:	  
********************************************************************************************************************/

void display_entry(void *parameter)
{
	  //ips114_clear(BLACK);
	
		unsigned char i,j,H,W;
    while(1)
    {
				//��ȡdisplay_sem�ź��������û��������ȴ����ͷ�CPU����Ȩ
        rt_sem_take(display_sem,RT_WAITING_FOREVER);
			
				if(pages == 1)
				{
					if(cleardisplay == 1){cleardisplay = 0;ips114_clear(WHITE);}
					
					//ips114_displayimage032_zoom(Binaryzation_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H, MT9V03X_CSI_W, MT9V03X_CSI_H);	//��ʾ����ͷͼ���ֵ��

					//drawing_drawing_drawing(); //���������� �߿� �����
                    ips114_showstr(0, 0, "Uart[0]=:");
                    ips114_showint16(80,0,uart1_data[0]);
                    ips114_showstr(0, 1, "Uart[1]=:");
                    ips114_showint16(80,1,uart1_data[1]);
                    ips114_showstr(0, 2, "Uart[2]=:");
                    ips114_showint16(80,2,uart1_data[2]);
                    ips114_showstr(0, 3, "Uart[3]=:");
                    ips114_showint16(80,3,uart1_data[3]);

					ips114_showfloat(0,4,yaw,3,3);
                    //ips114_showint16(130,4,uart1_data[4]);
                    //ips114_showint16(130,5,uart1_data[5]);
                    
					
				}
				else if(pages == 2)
				{
					if(cleardisplay == 1){cleardisplay = 0;ips114_clear(WHITE);}
					
					ips114_showstr(0, 0, "threshold:");
					ips114_showint16(100, 0, threshold);				
//					ips114_showstr(0, 1, "L1speed:");
//					ips114_showint16(100, 1, chassis.L1);
//					ips114_showstr(0, 2, "R1speed:");
//					ips114_showint16(100, 2, chassis.R1);
//					ips114_showstr(0, 3, "L2speed:");
//					ips114_showint16(100, 3, chassis.L2);
//					ips114_showstr(0, 4, "R2speed:");
//					ips114_showint16(100, 4, chassis.R2);
					
					ips114_showstr(0, 1, "172-134");
					ips114_showstr(0, 2, "X=:");
					ips114_showint16(100, 2,uart1_data[1] + uart1_data[3]);
					ips114_showstr(0, 3, "Y=:");
					ips114_showint16(100, 3,uart1_data[2]);
					
					ips114_showint16(100, 4,picture_data[0]);
					
//					ips114_showuint8(0,5,uart4_data[0]);
//					ips114_showuint8(0,6,uart4_data[1]);
//					ips114_showuint8(0,7,uart4_data[2]);
					
				}
        //ips114_displayimage032_zoom(Binaryzation_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H, MT9V03X_CSI_W/2, MT9V03X_CSI_H/2);//��С��ʾ�������ͬ����ʾһЩ����
        //ips114_showint16(0, 4, key_P);
        //ips114_showint16(0, 5, key_I);
       // ips114_showint16(0, 6, uart4_data[1]);
        //ips114_showint16(0, 7, uart4_data[2]);threshold
			
			
				//IMU_show();	//IMU������ʾ
				
    }
    
}


void display_init(void)
{
    rt_thread_t tid;
    
    //��ʼ����Ļ
    ips114_init();
    
    //������ʾ�߳� ���ȼ�����Ϊ31
    tid = rt_thread_create("display", display_entry, RT_NULL, 1024, 19, 30);
    
    //������ʾ�߳�
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}