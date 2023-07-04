#ifndef _openartart_mini_h
#define _openartart_mini_h

#include "headfile.h"

//����ͨ��
extern uint8 uart1_rx_buffer;	//����������
extern lpuart_transfer_t   uart1_receivexfer;
extern lpuart_handle_t     uart1_g_lpuartHandle;
extern uint8 uart1_data[6];	//���մ��ڻ���������
extern uint8_t openart_data[1]; //���ڷ�������

extern uint8 uart4_rx_buffer;	//����������
extern lpuart_transfer_t   uart4_receivexfer;
extern lpuart_handle_t     uart4_g_lpuartHandle;
extern uint8 uart4_data[6];	//���մ��ڻ���������

void uart4_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);//����ͨ��4 �ص�����
void uart1_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);//����ͨ�� �ص�����
void uart1_show(void);	 //uart1������ʾ
void openart_send(void); //����ͨ�� ���ݷ���
void openart_Init(void); //open art��ʼ��

#endif
