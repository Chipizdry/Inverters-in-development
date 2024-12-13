#include "uart2.h"


#if(UART2_INT_EN)
xdata u16 uart2_rx_sta;//bit15Used to mark whether a complete data packet has been received, bit[14:0] is used to store the length of the current data packetxdata u8
xdata u8  uart2_buf[UART2_PACKET_MAX_LEN+2];//Leave 2 blank characters
xdata u8  uart2_step;

//Serial port 2 interrupt service routine
//When sending data, the interrupt must be turned off, here is only responsible for processing the receiving interrupt.
/*
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
*/

void uart2_isr() interrupt 4 {
    u8 res;

    if (RI0) {  // Проверяем флаг приема данных
        RI0 = 0;  // Сбрасываем флаг приема

        res = SBUF0;  // Читаем принятый байт данных из регистра

        // Если пакет уже обработан, игнорируем дальнейшие данные
        if (uart2_rx_sta & UART2_PACKET_OK) {
            return;
        }

        // Сохраняем данные в буфер
        if (uart2_rx_sta < UART2_PACKET_MAX_LEN) {
            uart2_buf[uart2_rx_sta++] = res;
        } else {
            uart2_rx_sta = 0;  // Если буфер переполнен, сбрасываем
            return;
        }

        // Процесс приема данных по шагам
        if (uart2_step<8) {  // Первый байт — адрес устройства
            uart2_step++;
        } 
				
			if(uart2_step==8)	{  // Данные регистров и контрольная сумма (не используем для вывода на экран)
            uart2_rx_sta |= UART2_PACKET_OK;  // Устанавливаем флаг пакета
					uart2_step =0;
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




u16 calculate_crc(unsigned char *buffer, unsigned char length) {
    unsigned int temp, temp2, flag;
    unsigned int i;               // Вынесение переменной `i`
    unsigned char j;              // Вынесение переменной `j`

    temp = 0xFFFF;

    for (i = 0; i < length; i++) {
        temp = temp ^ buffer[i];
        for (j = 0; j < 8; j++) {
            flag = temp & 0x0001;
            temp >>= 1;
            if (flag)
                temp ^= 0xA001;
        }
    }

    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;

    return temp;
}


// Функция формирования и отправки Modbus-запроса
void modbus_request(u8 address, u16 start_register, u16 num_registers) {
    u8 request[8];
    u16 crc;

    // Формируем запрос Modbus
    request[0] = address;                      // Адрес устройства
    request[1] = 0x03;                         // Код функции (чтение регистров)
    request[2] = (start_register >> 8) & 0xFF; // Старший байт начального регистра
    request[3] = start_register & 0xFF;        // Младший байт начального регистра
    request[4] = (num_registers >> 8) & 0xFF;  // Старший байт количества регистров
    request[5] = num_registers & 0xFF;         // Младший байт количества регистров

    // Вычисляем CRC
    crc = calculate_crc(request, 6);
    request[6] = crc & 0xFF;                   // Младший байт CRC
    request[7] = (crc >> 8) & 0xFF;            // Старший байт CRC

    // Отправляем запрос через UART
    u2_send_bytes(request, 8);
}

// Функция циклического опроса 5 адресов Modbus
void poll_modbus_devices() {
    u8 modbus_addresses[5] = {1, 2, 3, 4, 5}; // Адреса устройств
    u16 start_register = 0x0001;              // Начальный регистр
    u16 num_registers = 4;                    // Количество регистров
  unsigned int k; 
    while (1) {
        for (k = 0; k < 5; k++) {
            // Формируем и отправляем запрос для каждого адреса
            modbus_request(modbus_addresses[k], start_register, num_registers);

            // Задержка между запросами для предотвращения наложения
            sys_delay_ms(900);
        }

        // Задержка перед следующим циклом опроса всех устройств
        sys_delay_ms(500);
    }
}






