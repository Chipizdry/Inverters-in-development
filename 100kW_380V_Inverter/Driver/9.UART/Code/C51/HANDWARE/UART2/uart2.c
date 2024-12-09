#include "uart2.h"


#if(UART2_INT_EN)
xdata u16 uart2_rx_sta;//bit15Used to mark whether a complete data packet has been received, bit[14:0] is used to store the length of the current data packetxdata u8
xdata u8  uart2_buf[UART2_PACKET_MAX_LEN+2];//Leave 2 blank characters
xdata u8  uart2_step;

//Serial port 2 interrupt service routine
//When sending data, the interrupt must be turned off, here is only responsible for processing the receiving interrupt.
void uart2_isr()	interrupt 4
{
	u8 res;
	
	if(RI0)//The serial port accepting the interrupt
	{
		RI0 = 0;//Clear accept interrupt flag
		res = SBUF0;//Read serial data
		
		if(uart2_rx_sta & UART2_PACKET_OK)//The received data has not been processed yet
			return;
	
		if(uart2_step==0)//The process of accepting data
		{
			if(res=='\r')//If the "\r\n" end marker is received, it is considered that the packet acceptance is complete
				uart2_step = 1;//Enter the process of accepting the '\n' token
			else if(res=='\n')//If the '\n' end marker is received, it is also considered that the packet is accepted as complete
				uart2_rx_sta |= UART2_PACKET_OK;//Mark packet acceptance complete
			else//Accept data
			{
				if(uart2_rx_sta>=UART2_PACKET_MAX_LEN)
					uart2_rx_sta = 0;//The data is too large, discard it, and start receiving from the beginning

				uart2_buf[uart2_rx_sta++] = res;//Store valid data
			}
		}else if(uart2_step==1)//The process of judging the end tag
		{
			uart2_step = 0;
			if(res=='\n')
				uart2_rx_sta |= UART2_PACKET_OK;//Mark packet acceptance complete
			else
				uart2_rx_sta = 0;//The next character of '\r' is not '\n', it is considered that the reception is wrong, and the reception starts from the beginning
		}
		
	}	
}
#endif


//Serial port 2 initialization
void uart2_init(u32 baud)
{
	MUX_SEL |= 0x40;//Setting bit6 to 1 means to export the uart2 interface to P0.4 and P0.5
	P0MDOUT &= 0xCF;
	P0MDOUT |= 0x10;//Set the corresponding IO port output and input
	ADCON = 0x80;//Select SREL0H:L as baud rate generator
	SCON0 = 0x50;//Accept enable and mode settings
	PCON &= 0x7F;//SMOD=0
	//Baud rate setting, the formula is:
	//SMOD=0  SREL0H:L=1024-main frequency/(64*baud rate),SMOD=1	 SREL0H:L=1024-main frequency/(32*baud rate)
	baud = 1024-(u16)(3225600.0f/baud);
	SREL0H = (baud>>8)&0xff;  
	SREL0L = baud&0xff;
	
	#if(UART2_INT_EN)
		ES0 = 1;//Interrupt enable
		EA = 1;
		//xdata variables must be initialized in functions
		uart2_rx_sta = 0;
		uart2_step = 0;
	#else
		ES0 = 0;
	#endif

}

//Send a byte
void u2_send_byte(u8 byte)
{
	ES0 = 0;//Close the serial port 2 interrupt first
	SBUF0 = byte;
	while(!TI0);
	TI0 = 0;
	#if(UART2_INT_EN)
		ES0 = 1;//Re-open interrupt
	#endif
}



//Send data
void u2_send_bytes(u8 *bytes,u16 len)
{
	u16 i;
	
	ES0 = 0;//Close the serial port 2 interrupt first
	for(i=0;i<len;i++)
	{
		SBUF0 = bytes[i];
		while(!TI0);
		TI0 = 0;
	}
	#if(UART2_INT_EN)
		ES0 = 1;//Re-open interrupt
	#endif
}


//Implement printf function with uart2 serial port
char putchar(char c)
{
	u2_send_byte(c);
	
	return c;
}












