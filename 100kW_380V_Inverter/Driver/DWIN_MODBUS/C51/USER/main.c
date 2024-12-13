#include "sys.h"
#include "uart2.h"
#include "uart5.h"

#define FIRST_TXT		 "DGUS Tool\0\0"
#define TEST_TXT		 "DGUS TEST TEXT\0\0"
#define INT_TXT		 "INERRUPT \0\0"
#define WHILE_TXT		 "WHILE___ \0\0"
void main(void)
{   
	
  //  u8 vp_buffer[16] = {0};           // Буфер для передачи (массив кратен 2 байтам)
	u8 send_buff[8]={0, };
	u16 send_len;
	
	u16 len;
	u16 i;
u8 buff[512]={0, };
u16 recv_len;
	sys_init();//System initialization
	
		
		sys_write_vp(0x2000,FIRST_TXT,sizeof(FIRST_TXT)/2+1);//ٸ֚һٶ"τѾДʾ"࠘ݾʨ׃τѾŚɝ
     sys_delay_ms(1000);
	  sys_write_vp(0x2002,TEST_TXT,sizeof(TEST_TXT)/2+1);
	uart2_init(9600);//Initialize serial port 2
	//uart5_init(9600);//Initialize serial port 5

	while(1){   
		
		
		if(uart2_rx_sta & UART2_PACKET_OK)
		{
			
		
			len = uart2_rx_sta&UART2_PACKET_LEN;
		//	u2_send_bytes(uart2_buf,len);
			
			recv_len = 0;
			for(i=0;i<len;i++)
			{
				recv_len += sprintf(buff+recv_len,"%02X ",(u16)uart2_buf[i]);
			}
		//	buff[recv_len++] = 0;
		//	buff[recv_len++] = 0;
			sys_write_vp(0x2000,buff,recv_len/2+1);
			
			
			uart2_rx_sta = 0;
			//u2_send_bytes(buff,len);
		}
	
		poll_modbus_devices();
		
		
	}
		
}


