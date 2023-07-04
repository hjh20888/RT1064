#include "headfile.h"

//Image信号量
rt_sem_t Image_sem;

unsigned char Binaryzation_image[MT9V03X_CSI_H][MT9V03X_CSI_W];	//用于存储二值化图像
float BlackPoint[black_size][2] = {0.f,0.f};			//存储黑点实际坐标
float Image_BlackPoint[black_size][2] = {0.f,0.f};//存储黑点图像上的相对坐标	用于描点 便于观察是否把点全部找到
unsigned char black_num = 0;	//记录黑点数量
unsigned char current_black_num = 0;	//记录当前黑点序号
int black_order[black_size] = {0};//存储黑点顺序
short int high_point[2], low_point[2], center_point[2];//图片中心坐标
short int left_edge_x = 0,right_edge_x = 0,top_edge_y = 0,low_edge_y = 0;//各边框的平均横坐标与平均纵坐标
float current_location_X = 0.2f, current_location_Y = -0.25f; //当前位置的X Y坐标
float next_location_X = 0.f, next_location_Y = 0.f; 		  //下一个位置的X Y坐标
unsigned char image_mode = 1;	//摄像头模式选择 0 关闭摄像头		1 识别坐标点		2 路径规划
	
//-------------------------------------------------------------------------------------------------------------------
//  brief      灰度图像二值化
//  return     void
//  since      v1.0
//  Sample usage:		image_binaryzation();				桌子上最佳阈值 -40 + 145
//-------------------------------------------------------------------------------------------------------------------

unsigned int threshold = 200;

void image_binaryzation(void)//图像二值化
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
//  brief      寻找A4纸中的矩形边框
//  return     unsigned char
//  since      v2.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------

unsigned char find_Rectangle(void)	//寻找A4纸中的矩形边框
{
		unsigned char H,W;
		unsigned char white_num=0; //连续的白点个数
		unsigned char first_find=1;//用于判断是否第一次检测到该边
		
		
		//寻找下边框
		for(H=Guide_Y; H<MT9V03X_CSI_H; H++)
		{

				if(*(*(Binaryzation_image+H-1)+Guide_X)==255 && *(*(Binaryzation_image+H)+Guide_X)==0)
				{
						low_edge_y = H;
						break;
				}	
		}
		//寻找上边框
		for(H=Guide_Y; H>0; H--)
		{
				if(*(*(Binaryzation_image+H+1)+Guide_X)==255 && *(*(Binaryzation_image+H)+Guide_X)==0)
				{
						top_edge_y = H;
						break;
				}	
		}
		
		//寻找左边框
		for(W=Guide_X; W>0; W--)
		{
				if(*(*(Binaryzation_image+Guide_Y)+W+1)==255 && *(*(Binaryzation_image+Guide_Y)+W)==0)
				{
						left_edge_x = W;
						break;
				}	
		}
		//寻找右边框
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
//  brief      寻找A4纸中的黑点个数与坐标
//  return     unsigned char
//  since      v2.0
//  Sample usage:		find_blackPoint(image_2);
//									black_num比实际黑点数多1	因为加上了一个原点
//-------------------------------------------------------------------------------------------------------------------

unsigned char find_blackPoint(unsigned char image[MT9V03X_CSI_H][MT9V03X_CSI_W])	//寻找A4纸中的黑点个数与坐标 
{
		unsigned char H,W;
		unsigned char find_flag=0;	//判断是否发现黑点
		unsigned char i,j;
		black_num=0;	//黑点数量置0
	
		for(H=top_edge_y+2; H<low_edge_y-2; H++)
		{	
				for(W=left_edge_x+2; W<right_edge_x-2; W++)
				{
						if(image[H][W] < image[H][W-1])
						{
								black_num++;
								//防止数组越界
								if(black_num > black_size-1)
										black_num = 0;
							
								//存储黑点图像上的相对坐标	用于描点 便于观察是否把点全部找到
								Image_BlackPoint[black_num][0]= W;
								Image_BlackPoint[black_num][1]= H;
								
								
								//记录下黑点实际坐标
								BlackPoint[black_num][0]= (float)(W*1.0f - left_edge_x*1.0f) / (float)(right_edge_x*1.0f - left_edge_x*1.0f) * Ground_Length; //- 0.1;
								BlackPoint[black_num][1]= (float)(low_edge_y*1.0f - H*1.0f) / (float)(low_edge_y*1.0f - top_edge_y*1.0f) * Ground_Width; //- 0.05;
							
								W+=2;		//如果发现黑点，则半径8cm内不再检测
								//防止数组越界
								if(W >= MT9V03X_CSI_W)
										W = MT9V03X_CSI_W - 1;
								
								find_flag=1;
						}			
				}
				if(find_flag==1)
				{
						H+=1;
						//防止数组越界
						if(H >= MT9V03X_CSI_H)
								H = MT9V03X_CSI_H - 1;
					
						find_flag=0;
				}	
		}
		
		
		black_num++;
		//防止数组越界
		if(black_num > black_size-1)
				black_num = 0;
		
		//第一个点与最后一个点为原点
		BlackPoint[0][0] =  0.1f;
		BlackPoint[0][1] =  -0.2f;
		
		BlackPoint[black_num][0] = 0.f;
		BlackPoint[black_num][1] = -0.2f;
		
		return black_num;
}	


//-------------------------------------------------------------------------------------------------------------------
//  brief      路径规划
//  return     unsigned char
//  since      v1.0
//  Sample usage:		
//-------------------------------------------------------------------------------------------------------------------

unsigned char Path_planning(void)//路径规划
{
		int i,j,k;
		float length=0; //记录总距离
		float temp;		//用于比较
	
		//规划前将存储顺序的数组清零
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
										if(j == black_order[k])	//如果该序号黑点已经记录 则退出循环不参与比较
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

unsigned char Path_planning2(void)//路径规划2
{
		int i = 0,j = 0,k = 0;
		float temp = 0;		//用于比较
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
		
		//已经找过的坐标点记录下来
		BlackPoint[k][0] = -10;
		BlackPoint[k][1] = -10;
		
		//对原点赋值 后面重复操作上一次
		BlackPoint[current_point][0] = -10;
		BlackPoint[current_point][1] = -10;
		
		current_point = k;
		
		return 0;
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      寻找图片中心
//  return     unsigned char
//  since      v1.0
//  Sample usage:		Y:55-65		X:90-100
//-------------------------------------------------------------------------------------------------------------------
																	/***************************
																	思路：
																	从左下角与右上角同时遍历图像
																	找到第一个黑白跳变点记录下来
																	***************************/

unsigned char find_image(unsigned char image[MT9V03X_CSI_H][MT9V03X_CSI_W])	//寻找图片中心
{
		unsigned char H,W;
		unsigned char i, j, k;
		unsigned char low_point_flag = 0, high_point_flag = 0;//最低点标志位 为1时找到最低点
	
		for(H=MT9V03X_CSI_H-1; H>0; H--)//寻找最低点坐标
		{
				for(W=0; W<MT9V03X_CSI_W-1; W++)
				{
						if(*(*(image+H)+W)==255)	//寻找最低点坐标
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
		
		for(H=0; H<MT9V03X_CSI_H-1; H++)//寻找最高点坐标
		{
				for(W=MT9V03X_CSI_W-1; W>0; W--)
				{
						if(*(*(image+H)+W)==255)	//寻找最高点坐标
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
		
		if(abs(low_point[1] - high_point[1]) > 10)//排除赛道上白点的干扰
		{
				center_point[0] = (low_point[0] + high_point[0])/2;
				center_point[1] = (low_point[1] + high_point[1])/2;
			
				unsigned char h,w;
		}	
	
		return 0;
}	

/*******************************************************************************************************************
*  brief      Image线程
*  return     void
*  since      v1.0
*  Sample usage:	  获取到信号量后开始运行	10ms一次
*******************************************************************************************************************/

void Image_entry(void *parameter)
{
    while(1)
    {
        //获取Image信号量，如果没有则持续等待并释放CPU控制权
        rt_sem_take(Image_sem,RT_WAITING_FOREVER);
			
		rt_enter_critical();    //API：进入临界区，退出前系统不会发生任务调度
			
//        //选取最佳图像用于寻找坐标
//        if(gpio_get(SWITCH_2)==0)
//        {
//                image_binaryzation();//图像二值化
//        }	
//        
//        find_Rectangle();	//寻找A4纸中的矩形边框3
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
//        //Path_planning();//路径规划
        

        
        rt_exit_critical();    //API：退出临界区
    }
}

void Image_init(void) //Image线程
{
    rt_thread_t tid;
	
	//MT9V03X摄像头初始化 使用CSI接口
	//mt9v03x_csi_init();
	
    //创建Image的信号量，8ms中断释放一次
    Image_sem = rt_sem_create("Image", 0, RT_IPC_FLAG_FIFO);		
    
    //创建Image的线程 优先级19
    tid = rt_thread_create("Image", Image_entry, RT_NULL, 4096, 19, 10);
    
    //启动线程
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}
