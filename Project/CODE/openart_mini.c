#include "headfile.h"

//串口通信
uint8 uart1_rx_buffer;	//缓冲区数据
lpuart_transfer_t   uart1_receivexfer;
lpuart_handle_t     uart1_g_lpuartHandle;
uint8 uart1_data[6];	//接收串口缓冲区数据
uint8_t openart_data[1]; //串口发送数据

uint8 uart4_rx_buffer;	//缓冲区数据
lpuart_transfer_t   uart4_receivexfer;
lpuart_handle_t     uart4_g_lpuartHandle;
uint8 uart4_data[6];	//接收串口缓冲区数据

//-------------------------------------------------------------------------------------------------------------------
//  brief      串口通信 回调函数4
//  return     void
//  since      v2.0
//  Sample usage:	初始化时调用	用于图片识别
//-------------------------------------------------------------------------------------------------------------------
//串口接收： 帧头    大类		小类			

void uart4_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)//串口通信4 回调函数
{
    static unsigned char RX_count = 0;//记录接收到的数据个数
    if(kStatus_LPUART_RxIdle == status)
    {
        //数据已经被写入到了 之前设置的BUFF中
        //本例程使用的BUFF为 uart4_rx_buffer
			
				if(uart4_rx_buffer == 0xAA)	//帧头1
				{
						RX_count = 0;
						uart4_data[0] = uart4_rx_buffer;//将数据取出
						RX_count++;
				}	
				else if(RX_count < 6) //多少个字节
				{
						uart4_data[RX_count++] = uart4_rx_buffer;//将数据取出
				}	
    }
    		
    handle->rxDataSize = uart4_receivexfer.dataSize;  //还原缓冲区长度
    handle->rxData = uart4_receivexfer.data;          //还原缓冲区地址
}

//-------------------------------------------------------------------------------------------------------------------
//  brief      串口通信 回调函数
//  return     void
//  since      v2.0
//  Sample usage:	初始化时调用	
//-------------------------------------------------------------------------------------------------------------------
//串口接收： 帧头    图片中心点x	图片中心点y	

void uart1_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)//串口通信1 回调函数
{
    static unsigned char RX_count = 0;//记录接收到的数据个数
    if(kStatus_LPUART_RxIdle == status)
    {
        //数据已经被写入到了 之前设置的BUFF中
        //本例程使用的BUFF为 uart1_rx_buffer
			
				if(uart1_rx_buffer == 0xAA)	//帧头1
				{
						RX_count = 0;
						uart1_data[0] = uart1_rx_buffer;//将数据取出
						RX_count++;
				}	
				else if(RX_count < 4)
				{
						uart1_data[RX_count++] = uart1_rx_buffer;//将数据取出
				}	
    }
    		
    handle->rxDataSize = uart1_receivexfer.dataSize;  //还原缓冲区长度
    handle->rxData = uart1_receivexfer.data;          //还原缓冲区地址
}

void uart1_show(void)	//uart1数据显示
{
		//openart_send(); //串口通信 数据发送
		ips114_showint16(130,0,uart1_data[0]);
		ips114_showint16(130,1,uart1_data[1]);
		ips114_showint16(130,2,uart1_data[2]);
		ips114_showint16(130,3,uart1_data[3]);
		ips114_showint16(130,4,uart1_data[4]);
		ips114_showint16(130,5,uart1_data[5]);
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      串口通信 数据发送
//  return     void
//  since      v2.0
//  Sample usage:	
//        			  uart_putchar(USART_1,uart_send);//串口字节发送
//-------------------------------------------------------------------------------------------------------------------

void openart_send(void) //串口通信 数据发送
{
    static uint8_t openart_data[1];
    openart_data[0] = 4;
    uart_putbuff(USART_4, (uint8_t *) &openart_data, sizeof(openart_data));
}

//-------------------------------------------------------------------------------------------------------------------
//  brief      open art初始化
//  return     void
//  since      v2.0
//  Sample usage:	
//-------------------------------------------------------------------------------------------------------------------

void openart_Init(void) //open art初始化
{
		//初始换串口   波特率为115200 TX为B12 RX为B13
    uart_init (USART_1, 115200,UART1_TX_B12,UART1_RX_B13);
	
    //配置串口接收的缓冲区及缓冲区长度
    uart1_receivexfer.dataSize = 1;
    uart1_receivexfer.data = &uart1_rx_buffer;
	
    //设置中断函数及其参数
    uart_set_handle(USART_1, &uart1_g_lpuartHandle, uart1_uart_callback, NULL, 0, uart1_receivexfer.data, 1);
	NVIC_SetPriority(LPUART1_IRQn,14);         //设置串口中断优先级 范围0-15 越小优先级越高
    uart_rx_irq(USART_1,1);
	
	
	uart_init(USART_4, 115200, UART4_TX_C16, UART4_RX_C17);

    //配置串口接收的缓冲区及缓冲区长度
    uart4_receivexfer.dataSize = 1;
    uart4_receivexfer.data = &uart4_rx_buffer;

    //设置中断函数及其参数
    uart_set_handle(USART_4, &uart4_g_lpuartHandle, uart4_uart_callback, NULL, 0, uart4_receivexfer.data, 1);

    NVIC_SetPriority(LPUART4_IRQn, 10); //14        //设置串口中断优先级 范围0-15 越小优先级越高
    uart_rx_irq(USART_4, 1);
}