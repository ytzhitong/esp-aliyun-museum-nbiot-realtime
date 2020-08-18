#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "restore.h"
#include "Other_sensor.h"

#include "esp_system.h"
#include "platform_hal.h"

#include "app_entry.h"

uint8_t debug_mode=0;

#define UART1_TXD  (GPIO_NUM_16)
#define UART1_RXD  (GPIO_NUM_4)
#define UART1_RTS  (UART_PIN_NO_CHANGE)
#define UART1_CTS  (UART_PIN_NO_CHANGE)

//uint8_t error_cnt=0;
//
//static void nbiot_task(void *arg)
//{
//    // Configure a temporary buffer for the incoming data
//    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
//
//    while (1)
//    {
//        // Read data from the UART
//        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 100 / portTICK_RATE_MS);
//
//        if(len>0)
//        {
//
//        }
//
//    }
//}

extern float NH4_value,NH4_temp;
extern int hum,tempA,tempB,lux;
extern uint32_t BT_ad;
extern float PHvalue;

#define wait_net_ms 100

void send_at_cmd(char *payload)
{
	int len=strlen ( payload );
	uart_write_bytes(UART_NUM_1,  payload, len);
}

void send_debug(char *payload)
{
	int len=strlen ( payload );
	uart_write_bytes(UART_NUM_0,  payload, len);
}

void uart1_nbiot_init(void)
{
	uint8_t i=0;
	uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
	int len=0;

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART1_TXD, UART1_RXD, UART1_RTS, UART1_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    for(i=0;i<10;i++) //
    {
    	send_at_cmd("AT+CGATT?\n");
    	len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 1000 / portTICK_RATE_MS);
    	uart_write_bytes(UART_NUM_0,  (char *)data, len);
    	if(len>0)
    	{
        	if(strstr((const char *)data, "+CGATT:1") != NULL)//开关
        	{
        		send_debug("CGATT OK\n");
        		break;
        	}
    	}
    }
    if(i>8)
    {
    	send_debug("CGATT FAIL\n");
    	send_at_cmd("AT+NRB\n");
    	vTaskDelay(100 / portTICK_RATE_MS);
    	esp_deep_sleep(DEEP_SLEEP_GAP);
    }

    strcpy((char *)data,"AT+QMTCFG=\"aliauth\",0,\"a1KjUd7JWxB\",\"SUZHOU_NB_001\",\"26f10ae448386c330c529103415b226e\"\n");
    strncpy((char *)data+37, decive_inf.name, 13);
    strncpy((char *)data+53, decive_inf.secret, 32);
    printf("%s\n",data);
    send_at_cmd((char *)data);//
	len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 300 / portTICK_RATE_MS);
	uart_write_bytes(UART_NUM_0,  (char *)data, len);
	if((len>0)&&(strstr((const char *)data, "OK") != NULL))
	{
		send_debug("QMTCFG OK\n");
	}
	else
	{
		send_debug("QMTCFG FAIL\n");
    	send_at_cmd("AT+NRB\n");
    	vTaskDelay(100 / portTICK_RATE_MS);
    	esp_deep_sleep(DEEP_SLEEP_GAP);
	}

    strcpy((char *)data,"AT+QMTOPEN=0,\"a1KjUd7JWxB.iot-as-mqtt.cn-shanghai.aliyuncs.com\",1883\n");
	send_at_cmd((char *)data);
    for(i=0;i<20;i++) //
    {
    	len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 1000 / portTICK_RATE_MS);
    	uart_write_bytes(UART_NUM_0,  (char *)data, len);
    	if(len>0)
    	{
        	if(strstr((const char *)data, "+QMTOPEN: 0,0") != NULL)//开关
        	{
        		send_debug("QMTOPEN OK\n");
        		break;
        	}
    	}
    }
    if(i>18)
    {
    	printf("QMTOPEN FAIL\n");
    	send_at_cmd("AT+NRB\n");
    	vTaskDelay(100 / portTICK_RATE_MS);
    	esp_deep_sleep(DEEP_SLEEP_GAP);
    }

    strcpy((char *)data,"AT+QMTCONN=0,\"clientExample\"\n");
	send_at_cmd((char *)data);
    for(i=0;i<10;i++) //
    {
    	len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 1000 / portTICK_RATE_MS);
    	uart_write_bytes(UART_NUM_0,  (char *)data, len);
    	if(len>0)
    	{
        	if(strstr((const char *)data, "+QMTCONN: 0,0,0") != NULL)//开关
        	{
        		send_debug("QMTCONN OK\n");
        		break;
        	}
    	}
    }
    if(i>8)
    {
    	send_debug("QMTCONN FAIL\n");
    	send_at_cmd("AT+NRB\n");
    	vTaskDelay(100 / portTICK_RATE_MS);
    	esp_deep_sleep(DEEP_SLEEP_GAP);
    }

}

uint16_t flash_gap = 950;
void LED_task(void *pvParameter)
{
    for(; ; )
    {
    	gpio_set_level(GPIO_LED_0, 1);
    	vTaskDelay(50 / portTICK_RATE_MS);
    	gpio_set_level(GPIO_LED_0, 0);
    	vTaskDelay(flash_gap / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

void LED_flash(uint16_t gap)
{
	gpio_set_level(GPIO_LED_0, 1);
	vTaskDelay(gap / portTICK_RATE_MS);
	gpio_set_level(GPIO_LED_0, 0);
}

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_LED_0))
#define GPIO_INPUT_PIN_SEL   ((1ULL<<GPIO_INPUT_MODE))

void control_gpio_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_set_level(GPIO_LED_0, 0);

    //xTaskCreate(LED_task, "test_task", 2048, NULL, 5, NULL);
}

#define UART0_BUF_SIZE (128)

void uart0_debug_task(void *pvParameter)
{
	char* ptr = NULL;
	int len;
	uint8_t *data = (uint8_t *) malloc(UART0_BUF_SIZE);

    for(; ; )
    {
    	int len = uart_read_bytes(UART_NUM_0, data, UART0_BUF_SIZE, 200 / portTICK_RATE_MS);
    	if((len>0))
    	{
        	if(strstr((const char *)data, "Read_ph_correction_para") != NULL)//开关
        	{
        		read_PH_correct();
        		printf("ph_correction_zero= %.3f , slope= %.3f \n",PH_correct_para.zero,PH_correct_para.slope);
        	}

        	if(strstr((const char *)data, "Write_ph_correction_para") != NULL)//开关
        	{
        	    if(strstr((const char *)data, "zero") != NULL)
        	    {
        	    	ptr=strstr((const char *)data, "zero")+5;
        	    	PH_correct_para.zero=atof(ptr);
        	    }
        	    if(strstr((const char *)data, "slope") != NULL)
        	    {
        	    	ptr=strstr((const char *)data, "slope")+6;
        	    	PH_correct_para.slope=atof(ptr);
        	    }
        	    write_PH_correct();
        		printf("ph_correction_zero= %.3f , slope= %.3f \n",PH_correct_para.zero,PH_correct_para.slope);
        	}

        	if(strstr((const char *)data, "Read_ph_value") != NULL)//开关
        	{
        		printf("ph_value= %.2f \n",PHvalue);
        	}

        	if(strstr((const char *)data, "Read_device_information") != NULL)//开关
        	{
        		read_devive_inf();
        		printf("device_name= %s , device_secret= %s \n",decive_inf.name,decive_inf.secret);
        	}
        	if(strstr((const char *)data, "Write_device_information") != NULL)//开关
        	{
        	    if(strstr((const char *)data, "name") != NULL)
        	    {
        	    	ptr=strstr((const char *)data, "name")+5;
        	    	strncpy(decive_inf.name, ptr, 13);
        	    }
        	    if(strstr((const char *)data, "secret") != NULL)
        	    {
        	    	ptr=strstr((const char *)data, "secret")+7;
        	    	strncpy(decive_inf.secret, ptr, 32);
        	    }
        	    write_device_secret();
        	    printf("device_name= %s , device_secret= %s \n",decive_inf.name,decive_inf.secret);
        	}

        	if(strstr((const char *)data, "Read_nh4_temp_value") != NULL)//开关
        	{
        		printf("nh4_value= %.3f ,temp= %.1f \n",NH4_value,NH4_temp);
        	}

        	if(strstr((const char *)data, "Read_hum_temp_ab_value") != NULL)//开关
        	{
        		printf("hum= %.1f ,tempA= %.1f ,tempB= %.1f \n",(float)hum/10,(float)tempA/10,(float)tempB/10);
        	}

    	}
    	len=0;
    	memset(data,0,UART0_BUF_SIZE);
    }
    vTaskDelete(NULL);
}

void uart0_debug_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    xTaskCreate(uart0_debug_task, "test_task", 4096, NULL, 5, NULL);
}


#define UART2_TXD  (GPIO_NUM_17)
#define UART2_RXD  (GPIO_NUM_18)
#define UART2_RTS  (UART_PIN_NO_CHANGE)
#define UART2_CTS  (UART_PIN_NO_CHANGE)

#define UART2_BUF_SIZE (128)

float HexToDecimal(uint8_t *Byte,int num)
{
	return *((float*)Byte);
}

void NH4_read(void)
{
    // Configure a temporary buffer for the incoming data
    char read_value_cmd[8]={0x01,0x03,0x00,0x01,0x00,0x02,0x95,0xcb};
    char read_temp_cmd[8]= {0x01,0x03,0x00,0x03,0x00,0x02,0x34,0x0b};
    uint8_t *data = (uint8_t *) malloc(UART2_BUF_SIZE);
    uint8_t Byte_temp[4];

	uart_write_bytes(UART_NUM_2,  read_value_cmd, 8);
	int len = uart_read_bytes(UART_NUM_2, data, UART2_BUF_SIZE, 250 / portTICK_RATE_MS);
	if((len>0)&&(data[0]==0x01)&&(data[1]==0x03)&&(data[2]==0x04))
	{
		Byte_temp[0]=data[4];
		Byte_temp[1]=data[3];
		Byte_temp[2]=data[6];
		Byte_temp[3]=data[5];

		NH4_value=HexToDecimal(Byte_temp,4);
		//printf("H20 NH4: %.3f \n",NH4_value);

		memset(data,0,UART2_BUF_SIZE);
		memset(Byte_temp,0,4);
	}

	vTaskDelay(250 / portTICK_RATE_MS);

	uart_write_bytes(UART_NUM_2,  read_temp_cmd, 8);
	len = uart_read_bytes(UART_NUM_2, data, UART2_BUF_SIZE, 250 / portTICK_RATE_MS);
	if((len>0)&&(data[0]==0x01)&&(data[1]==0x03)&&(data[2]==0x04))
	{
		Byte_temp[0]=data[4];
		Byte_temp[1]=data[3];
		Byte_temp[2]=data[6];
		Byte_temp[3]=data[5];

		NH4_temp=HexToDecimal(Byte_temp,4);
		//printf("H20 temp: %.1f \n",NH4_temp);

		memset(data,0,UART2_BUF_SIZE);
		memset(Byte_temp,0,4);
	}

	vTaskDelay(250 / portTICK_RATE_MS);
}

void uart2_NH4_init(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, UART2_TXD, UART2_RXD, UART2_RTS, UART2_CTS);
    uart_driver_install(UART_NUM_2, UART2_BUF_SIZE * 2, 0, 0, NULL, 0);
}


#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;

static const adc_unit_t unit = ADC_UNIT_1;

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}


uint32_t ADC_value_get(adc1_channel_t channel, adc_atten_t atten)
{
	uint32_t adc_reading = 0;

	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(channel, atten);

    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
#if debug
    print_char_val_type(val_type);
#endif
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
    	adc_reading += adc1_get_raw((adc1_channel_t)channel);
    }
    adc_reading /= NO_OF_SAMPLES;
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
#if debug
    printf("Raw: %d \n", adc_reading);
#endif
    return voltage;
}


