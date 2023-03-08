#include "zf_systick.h"
#include "zf_gpio.h"
#include "zf_uart.h"
#include "laser.h"

void laser_init(void)
{
	uart_init (LASER_UART, LASER_UART_BAUD, LASER_UART_TX, LASER_UART_RX);	// 初始化串口
	uart_rx_irq(LASER_UART, ENABLE);

	uart_index[LASER_UART]->IER |= UART_IER_RX;													// 使能接收完成中断
	nvic_init(UART3_IRQn, 0x02, 0x01, (FunctionalState)ENABLE);
}

uint16 distance;
void laser_uart_callback(void)
{
	static uint8 laser_buff[9],count=0,check_data=0;
	
	laser_buff[count] = (uart_index[LASER_UART])->RDR;
	
	if(count<=1)
	{
		if(laser_buff[count]!=0x59)
		{
			count = 0;
			check_data=0;
		}
		else
		{
			check_data += laser_buff[count];
			count++;
		}
	}
	else
	{
		if(count == 8)
		{
			if(check_data == laser_buff[count])
				distance = laser_buff[2]+256*laser_buff[3];
			
			count=0;
			check_data=0;
		}
		else
		{
			check_data += laser_buff[count];
			count++;
		}
	}
	
	
	
}
