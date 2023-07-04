#ifndef _openartart_mini_h
#define _openartart_mini_h

#include "headfile.h"

//串口通信
extern uint8 uart1_rx_buffer;	//缓冲区数据
extern lpuart_transfer_t   uart1_receivexfer;
extern lpuart_handle_t     uart1_g_lpuartHandle;
extern uint8 uart1_data[6];	//接收串口缓冲区数据
extern uint8_t openart_data[1]; //串口发送数据

extern uint8 uart4_rx_buffer;	//缓冲区数据
extern lpuart_transfer_t   uart4_receivexfer;
extern lpuart_handle_t     uart4_g_lpuartHandle;
extern uint8 uart4_data[6];	//接收串口缓冲区数据

void uart4_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);//串口通信4 回调函数
void uart1_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);//串口通信 回调函数
void uart1_show(void);	 //uart1数据显示
void openart_send(void); //串口通信 数据发送
void openart_Init(void); //open art初始化

#endif
