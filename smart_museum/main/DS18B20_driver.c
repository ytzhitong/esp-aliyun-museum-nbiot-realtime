
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"

#include "driver/gpio.h"
#include "esp_err.h"

#include "DS18B20_driver.h"

#define GPIO_18B20    33
#define GPIO_18B20_PIN_SEL  1ULL<<GPIO_18B20

void DS18B20_gpio_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO0
    io_conf.pin_bit_mask = GPIO_18B20_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

uint8_t Ds18b20Init()
{
	uint16_t cnt=0;

	gpio_set_direction(GPIO_18B20, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_18B20, 0);
    ets_delay_us(700);

    gpio_set_direction(GPIO_18B20, GPIO_MODE_INPUT);

    while ((gpio_get_level(GPIO_18B20) == 1) && (cnt < 200))
	{
    	ets_delay_us(1);
		cnt++;
	}
	if (cnt >= 200) return 1;
	cnt = 0;

	//3.2 ������⵽GPIO13�ĵ͵�ƽ�����ǵ͵�ƽ����ʱ��̫��������ΪDS18B20ģ���쳣
	while ((gpio_get_level(GPIO_18B20) == 0) && (cnt < 480))
	{
		ets_delay_us(1);
		cnt++;
	}
	if (cnt >= 480) return 1;
	return 0;//�ɹ�����0
}

void Ds18b20WriteData(uint8_t data)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		//1.д����һ��ʼ��Ϊ�����Ȱ��������� 1 ΢���ʾд���ڿ�ʼ
		gpio_set_direction(GPIO_18B20, GPIO_MODE_OUTPUT);
		gpio_set_level(GPIO_18B20, 0);//�������͵͵�ƽ

		ets_delay_us(1);
		gpio_set_level(GPIO_18B20, data & 0x01);
		ets_delay_us(80);//д��������Ϊ 60 ΢�룬������� 120 ΢��
					//�ͷ�����Ϊ�ߵ�ƽ
		gpio_set_direction(GPIO_18B20, GPIO_MODE_INPUT);
		data = data >> 1;
	}
}


uint8_t Ds18b20ReadData(void)
{
	uint8_t byte=0;
	uint8_t data=0;
	int  j;
	for (j = 8; j > 0; j--)
	{
		gpio_set_direction(GPIO_18B20, GPIO_MODE_OUTPUT);
		gpio_set_level(GPIO_18B20, 0);		//�Ƚ���������1us

		ets_delay_us(1);
		gpio_set_direction(GPIO_18B20, GPIO_MODE_INPUT);	  	//Ȼ���ͷ�����

		ets_delay_us(6);
		data = gpio_get_level(GPIO_18B20);	 	//��ȡ���ݣ������λ��ʼ��ȡ
													/*��byte����һλ��Ȼ����������7λ���bi��ע���ƶ�֮���Ƶ���λ��0��*/
		byte = (byte >> 1) | (data << 7);
		ets_delay_us(48); //��ȡ��֮��ȴ�48us�ٽ��Ŷ�ȡ��һ����
	}
	return byte;
}


//* �������ܣ���18b20��ʼת���¶�
void Ds18b20ChangTemp(void)
{
	Ds18b20Init();
	ets_delay_us(1000);
	Ds18b20WriteData(0xcc);		//����ROM��������
	Ds18b20WriteData(0x44);	    //�¶�ת������
								//	Delay1ms(100);	//�ȴ�ת���ɹ������������һֱˢ�ŵĻ����Ͳ��������ʱ��
}

//* �������ܣ����Ͷ�ȡ�¶�����
void Ds18b20ReadTempCom(void)
{
	Ds18b20Init();
	ets_delay_us(1000);
	Ds18b20WriteData(0xcc);	 //����ROM��������
	Ds18b20WriteData(0xbe);	 //���Ͷ�ȡ�¶�����
}

//* �������ܣ���ȡ�¶�
int Ds18b20ReadTemp(void)
{
	int temp = 0;
	int tem = 0;
	uint8_t tmh, tml;

	Ds18b20ChangTemp();			 	//��д��ת������
	Ds18b20ReadTempCom();			//Ȼ��ȴ�ת������Ͷ�ȡ�¶�����
	tml = Ds18b20ReadData();		//��ȡ�¶�ֵ��16λ���ȶ����ֽ�
	tmh = Ds18b20ReadData();		//�ٶ����ֽ�

    if(tmh>7)
    {
    	tmh=~tmh;
    	tml=~tml;
        temp=0;					//�¶�Ϊ��
    }else temp=1;				//�¶�Ϊ��
    tem=tmh; 					//��ø߰�λ
    tem<<=8;
    tem+=tml;					//��õװ�λ
    tem=(float)tem*0.625;		//ת��
	if(temp)return tem; 		//�����¶�ֵ
	else return -tem;

}
