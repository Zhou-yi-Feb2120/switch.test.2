/*
 * schA2.c
 *
 * Created: 2024/5/25 10:47:18
 * Author : Pigeon
 */ 

/*
 * schA240518.c
 *
 * Created: 2024/5/18 14:34:01
 * Author : Pigeon
 */ 
#define F_CPU 16000000
#define BAUD 4800

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

/******* HLW8032系数 *******/
double Ue=1.88666;	//电压系数
double Ce=1.0;		//电流系数
double VP_REG, V_REG, CP_REG, C_REG, PP_REG, P_REG;
//unsigned char len;
static unsigned long int HLW[24];
//static unsigned char HLWR[72] = {0x52,0x84,0x51,0x11,0xe8,0x97,0x84,0x50,0x61,0x01,0xf6,0x1c,0xf2,0x5a,0x02,0xf5,0xd0,0x00,0x06,0x98,0x00,0x3f,0x93,0x02,0x52,0x84,0x51,0x11,0xe8,0x97,0x84,0x50,0x61,0x01,0xf6,0x1c,0xf2,0x5a,0x02,0xf5,0xd0,0x00,0x06,0x98,0x00,0x3f,0x93,0x02,0x52,0x84,0x51,0x11,0xe8,0x97,0x84,0x50,0x61,0x01,0xf6,0x1c,0xf2,0x5a,0x02,0xf5,0xd0,0x00,0x06,0x98,0x00,0x3f,0x93,0x02,};
static unsigned char UART_R[72];
static double data[4];
static double adc[2];
//static double data1[3] = {220,0.1,22};

/******* MQTT参数 *******/
static unsigned char MQTTRST[] = {"AT+RST"};
static unsigned char MQTTUSERCFG[] = {"AT+MQTTUSERCFG=0,1,\"B9067\",\"SWa2testDN&k1ckttG09e1\",\"90A4689368373E9DEFEE6D0F7B11E0FA7BBFC511\",0,0,\"\""};
static unsigned char MQTTCLIENTID[] = {"AT+MQTTCLIENTID=0,\"B9067|securemode=3\\,signmethod=hmacsha1|\""};
static unsigned char MQTTCONN[] = {"AT+MQTTCONN=0,\"k1ckttG09e1.iot-as-mqtt.cn-shanghai.aliyuncs.com\",1883,1"};
//static unsigned char MQTTSUB[] = {"AT+MQTTSUB=0,\"/k1ckttG09e1/SWa2testDN/user/get\",1"};
//static unsigned char MQTTSUB[] = {"AT+MQTTSUB=0,\"/k1ckttG09e1/SWa2testDN/user/get\",1\r\n"};
//static unsigned char MQTTPUB[] = {"data9067-TEST"};
	
/******* UART初始化 *******//***********************************************************************************************************************/
void uart_init(void)
{
	UCSR0A = 0x00;
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);//|(1<<UPM01);	//8位
	UBRR0L = (F_CPU/BAUD/16-1)%256;	//波特率寄存器
	UBRR0H = (F_CPU/BAUD/16-1)/256;
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);		//发送使能，接受使能
}

/******* UART传输单字节 *******/
unsigned char uart_Rx_byte(void)
{
	while(!(UCSR0A&(1<<RXC0)));			//检测接收是否结束
	return UDR0;
}
void USART_Tx_byte( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

/******* UART接收数据 *******/
void uart_Rx(uint8_t* pBuf, int len)
{
	
	for(;len > 0;len--, pBuf++)
	{
		*pBuf = uart_Rx_byte();
	}
}
/******* UART发送数据 *******/
void uart_Tx(unsigned char* pBuf, int len)
{
	for(;len > 0;len--, pBuf++)
	{
		USART_Tx_byte(*pBuf);
	}
}

/******* HLW数据处理 *******/
void HLW_Rx_P(void)
{
	int i, j;
	i = 0;
	while(UART_R[i] != 0x5A) {i++;}
	for (j = 0; j < 24; j++)
	{
		HLW[j] = UART_R[i - 1];
		i = i + 1;
	}
}
/******* 数据处理 *******/
void Data_Processing(void)
{
	VP_REG = HLW[2] * 65536 + HLW[3] * 256 + HLW[4];	//电压参数寄存器
	V_REG = HLW[5] * 65536 + HLW[6] * 256 + HLW[7];		//电压寄存器
	data[0] = (VP_REG / V_REG) * Ue;
	
	CP_REG = HLW[8] * 65536 + HLW[9] * 256 + HLW[10];	//电流参数寄存器
	C_REG = HLW[11] * 65536 + HLW[12] * 256 + HLW[13];	//电流寄存器
	data[1] = ((CP_REG * Ce*100) / C_REG) / 100.0;
	
	PP_REG = HLW[14] * 65536 + HLW[15] * 256 + HLW[16];	//功率参数寄存
	P_REG = HLW[17] * 65536 + HLW[18] * 256 + HLW[19];	//功率寄存器
	data[2] = (PP_REG / P_REG) * Ue * Ce;
}

/******* 重定向 *******/
int uart_putchar(char c, FILE *stream)
{
	if (c == '\n')
	{
		uart_putchar('\r', stream);
	}
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
	return 0;
}

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

/******* ESP8266初始化 *******/
void ESP_init(void)
{
	_delay_ms(777);
	printf("%s\n",MQTTRST);
	_delay_ms(777);
	printf("%s\n",MQTTUSERCFG);
	_delay_ms(777);
	printf("%s\n",MQTTCLIENTID);
	_delay_ms(777);
	printf("%s\n",MQTTCONN);
	_delay_ms(777);
}

/******* ADC初始化 *******/
void adc_init(void)
{
	// 设置参考电压为 AVcc
	ADMUX = (1 << REFS0);
	// 启用 ADC，并设置预分频器为 128（16 MHz / 128 = 125 KHz）
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

/******* ADC_DATA_P *******/
void adc_read(uint8_t channel)
{
	// 限制通道范围在 0 到 7 之间
	channel &= 0x07;
	// 选择 ADC 通道
	ADMUX = (ADMUX & 0xF8) | channel;
	// 启动 ADC 转换
	ADCSRA |= (1 << ADSC);
	// 等待转换完成
	while (ADCSRA & (1 << ADSC));
	// 返回 ADC 值
	adc[1] = ADCL;
	adc[0] = ADCH;
}
void adc_data_p(void)
{
	double adcR;
	adcR = adc[0] * 256 + adc[1];
	data[3] = adcR * 500 / 1024;
}
/***********************************************************************************************************************/
int main(void)
{
    /* Replace with your application code */
	stdout = &uart_output;
	_delay_ms(333);
	DDRC =(1<<DDC5);
	DDRD |= (1<<DDD1)|(1<<DDD4)|(1<<DDD6);
	PORTD |= (1<<PORTD1)|(1<<PORTD0);//|(1<<PORTD4);
	PORTC |=(1<<PORTC5);
	uart_init();
	ESP_init();
	adc_init();
	_delay_ms(177);
	PORTC &= !(1<<PORTC5);
	while (1) 
    {
		PORTC |=(1<<PORTC5);
		
		uart_Rx(UART_R, 72);	//采集1
		HLW_Rx_P();				//处理1.1
		Data_Processing();		//处理1.2
		
		adc_read(1);			//采集2
		adc_data_p();			//处理2
		
		_delay_ms(333);
		printf("AT+MQTTPUB=0,\"/k1ckttG09e1/SWa2testDN/user/update\",\"%d %d %d %d\",1,0\n",(int)(data[0]*100),(int)(data[1]*1000),(int)(data[2]*100),(int)(data[3] * 100));
		_delay_ms(333);		
		
		PORTC &= !(1<<PORTC5);
    }
}


//uart_Tx(HLW, 24);		//调试

//printf("AT+MQTTPUB=0,\"/k1ckttG09e1/SWa2testDN/user/update\",\"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\",1,0\n",(int)HLW[0],(int)HLW[1],(int)HLW[2],(int)HLW[3],(int)HLW[4],(int)HLW[5],(int)HLW[6],(int)HLW[7],(int)HLW[8],(int)HLW[9],(int)HLW[10],(int)HLW[11],(int)HLW[12],(int)HLW[13],(int)HLW[14],(int)HLW[15],(int)HLW[16],(int)HLW[17],(int)HLW[18],(int)HLW[19],(int)HLW[20],(int)HLW[21],(int)HLW[22],(int)HLW[23]);
//printf("AT+MQTTPUB=0,\"/k1ckttG09e1/SWa2testDN/user/update\",\"%d %d %d\",1,0\n",,i,p);
//_delay_ms(377);
//printf("AT+MQTTPUB=0,\"/k1ckttG09e1/SWa2testDN/user/update\",\"%ld %d %d\",1,0\n",(long int)(VP_REG),(int)V_REG,(int)(VP_REG/V_REG));
//uart_Tx(id, 6);