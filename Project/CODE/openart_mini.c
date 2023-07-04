#include "headfile.h"

//����ͨ��
uint8 uart1_rx_buffer;	//����������
lpuart_transfer_t   uart1_receivexfer;
lpuart_handle_t     uart1_g_lpuartHandle;
uint8 uart1_data[6];	//���մ��ڻ���������
uint8_t openart_data[1]; //���ڷ�������

uint8 uart4_rx_buffer;	//����������
lpuart_transfer_t   uart4_receivexfer;
lpuart_handle_t     uart4_g_lpuartHandle;
uint8 uart4_data[6];	//���մ��ڻ���������

//-------------------------------------------------------------------------------------------------------------------
//  brief      ����ͨ�� �ص�����4
//  return     void
//  since      v2.0
//  Sample usage:	��ʼ��ʱ����	����ͼƬʶ��
//-------------------------------------------------------------------------------------------------------------------
//���ڽ��գ� ֡ͷ    ����		С��			

void uart4_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)//����ͨ��4 �ص�����
{
    static unsigned char RX_count = 0;//��¼���յ������ݸ���
    if(kStatus_LPUART_RxIdle == status)
    {
        //�����Ѿ���д�뵽�� ֮ǰ���õ�BUFF��
        //������ʹ�õ�BUFFΪ uart4_rx_buffer
			
				if(uart4_rx_buffer == 0xAA)	//֡ͷ1
				{
						RX_count = 0;
						uart4_data[0] = uart4_rx_buffer;//������ȡ��
						RX_count++;
				}	
				else if(RX_count < 6) //���ٸ��ֽ�
				{
						uart4_data[RX_count++] = uart4_rx_buffer;//������ȡ��
				}	
    }
    		
    handle->rxDataSize = uart4_receivexfer.dataSize;  //��ԭ����������
    handle->rxData = uart4_receivexfer.data;          //��ԭ��������ַ
}

//-------------------------------------------------------------------------------------------------------------------
//  brief      ����ͨ�� �ص�����
//  return     void
//  since      v2.0
//  Sample usage:	��ʼ��ʱ����	
//-------------------------------------------------------------------------------------------------------------------
//���ڽ��գ� ֡ͷ    ͼƬ���ĵ�x	ͼƬ���ĵ�y	

void uart1_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)//����ͨ��1 �ص�����
{
    static unsigned char RX_count = 0;//��¼���յ������ݸ���
    if(kStatus_LPUART_RxIdle == status)
    {
        //�����Ѿ���д�뵽�� ֮ǰ���õ�BUFF��
        //������ʹ�õ�BUFFΪ uart1_rx_buffer
			
				if(uart1_rx_buffer == 0xAA)	//֡ͷ1
				{
						RX_count = 0;
						uart1_data[0] = uart1_rx_buffer;//������ȡ��
						RX_count++;
				}	
				else if(RX_count < 4)
				{
						uart1_data[RX_count++] = uart1_rx_buffer;//������ȡ��
				}	
    }
    		
    handle->rxDataSize = uart1_receivexfer.dataSize;  //��ԭ����������
    handle->rxData = uart1_receivexfer.data;          //��ԭ��������ַ
}

void uart1_show(void)	//uart1������ʾ
{
		//openart_send(); //����ͨ�� ���ݷ���
		ips114_showint16(130,0,uart1_data[0]);
		ips114_showint16(130,1,uart1_data[1]);
		ips114_showint16(130,2,uart1_data[2]);
		ips114_showint16(130,3,uart1_data[3]);
		ips114_showint16(130,4,uart1_data[4]);
		ips114_showint16(130,5,uart1_data[5]);
}	

//-------------------------------------------------------------------------------------------------------------------
//  brief      ����ͨ�� ���ݷ���
//  return     void
//  since      v2.0
//  Sample usage:	
//        			  uart_putchar(USART_1,uart_send);//�����ֽڷ���
//-------------------------------------------------------------------------------------------------------------------

void openart_send(void) //����ͨ�� ���ݷ���
{
    static uint8_t openart_data[1];
    openart_data[0] = 4;
    uart_putbuff(USART_4, (uint8_t *) &openart_data, sizeof(openart_data));
}

//-------------------------------------------------------------------------------------------------------------------
//  brief      open art��ʼ��
//  return     void
//  since      v2.0
//  Sample usage:	
//-------------------------------------------------------------------------------------------------------------------

void openart_Init(void) //open art��ʼ��
{
		//��ʼ������   ������Ϊ115200 TXΪB12 RXΪB13
    uart_init (USART_1, 115200,UART1_TX_B12,UART1_RX_B13);
	
    //���ô��ڽ��յĻ�����������������
    uart1_receivexfer.dataSize = 1;
    uart1_receivexfer.data = &uart1_rx_buffer;
	
    //�����жϺ����������
    uart_set_handle(USART_1, &uart1_g_lpuartHandle, uart1_uart_callback, NULL, 0, uart1_receivexfer.data, 1);
	NVIC_SetPriority(LPUART1_IRQn,14);         //���ô����ж����ȼ� ��Χ0-15 ԽС���ȼ�Խ��
    uart_rx_irq(USART_1,1);
	
	
	uart_init(USART_4, 115200, UART4_TX_C16, UART4_RX_C17);

    //���ô��ڽ��յĻ�����������������
    uart4_receivexfer.dataSize = 1;
    uart4_receivexfer.data = &uart4_rx_buffer;

    //�����жϺ����������
    uart_set_handle(USART_4, &uart4_g_lpuartHandle, uart4_uart_callback, NULL, 0, uart4_receivexfer.data, 1);

    NVIC_SetPriority(LPUART4_IRQn, 10); //14        //���ô����ж����ȼ� ��Χ0-15 ԽС���ȼ�Խ��
    uart_rx_irq(USART_4, 1);
}