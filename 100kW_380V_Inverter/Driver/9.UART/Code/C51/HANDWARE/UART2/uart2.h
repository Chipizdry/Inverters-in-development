#ifndef __UART2_H__
#define __UART2_H__
#include "sys.h"
#include <stdio.h>

#define UART5_INT_EN							1
#define UART2_INT_EN							1				//Whether the serial port interrupt is enabled
#define UART2_PACKET_OK						0x8000  //Whether the complete packet was received
#define UART2_PACKET_LEN					0x7fff	//Length of packet
#define UART2_PACKET_MAX_LEN			100			//Defines the maximum length of the serial port package, excluding '\n' or "\r\n" end tags


//Variable declaration
extern xdata u16 uart2_rx_sta;
extern xdata u8  uart2_buf[];

//Function declaration
void uart2_init(u32 baud);
void u2_send_byte(u8 byte);
void u2_send_bytes(u8 *bytes,u16 len);

#endif


