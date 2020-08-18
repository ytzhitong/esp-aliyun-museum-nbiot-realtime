/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP32 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_sleep.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"

#include "app_entry.h"

#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/rtc_io.h"

#include "restore.h"

#include "AM2320_driver.h"
#include "DS18B20_driver.h"
#include "BH1750_driver.h"
#include "Other_sensor.h"

float NH4_value,NH4_temp;
int hum,tempA,tempB,lux;
uint32_t BT_ad,PH_ad;
float PHvalue;


void sensor_read(void)
{

	AM2320_get_value(&hum,&tempA);

	tempB=Ds18b20ReadTemp();

	lux=bh1750_value_get();

	PH_ad=ADC_value_get(ADC_CHANNEL_7,ADC_ATTEN_DB_11);
	PHvalue=16.314-0.0054*PH_ad;
	PHvalue=PH_correct_para.zero+PH_correct_para.slope*PHvalue;

	BT_ad=ADC_value_get(ADC_CHANNEL_6,ADC_ATTEN_DB_11)*1.6;

	NH4_read();

	LED_flash(50);

	//	printf("hum=%d tempA=%d \n",hum,tempA);
	//	printf("tempB=%d \n",tempB);
	//	printf("lux=%d \n",lux);
	//	printf("PH: %.2f \n", PHvalue);
	//	printf("Voltage2: %dmV \n", BT_ad);

	//	vTaskDelay(2000 / portTICK_RATE_MS);
}


//TaskHandle_t Sensor_Task_Handler;

void app_main()
{
	uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
	int i=0;
	int len=0;

	//Spiffs_init();

    //Initialize NVS
	ESP_ERROR_CHECK( nvs_flash_init() );
	read_PH_correct();
	read_devive_inf();

	rtc_gpio_isolate(GPIO_NUM_12);

    //Initialize sensors
    control_gpio_init();
    DS18B20_gpio_init();
    AM2320_gpio_init();
    bh1750_i2c_init();

    uart0_debug_init();
    uart2_NH4_init();

    if(gpio_get_level(GPIO_INPUT_MODE))
    {
        vTaskDelay(5000 / portTICK_RATE_MS);
    	uart1_nbiot_init();
    }

    while(1)
    {
    	sensor_read();
    	sensor_read();

    	if(gpio_get_level(GPIO_INPUT_MODE))
    	{
    	    strcpy((char *)data,"AT+QMTPUB=0,1,1,0,\"/sys/a1KjUd7JWxB/SUZHOU_NB_001/thing/event/property/post\"\n");
    	    strncpy((char *)data+36, decive_inf.name, 13);
    		send_at_cmd((char *)data);
    	    for(i=0;i<5;i++) //
    	    {
    	    	len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 1000 / portTICK_RATE_MS);
    	    	uart_write_bytes(UART_NUM_0,  (char *)data, len);
    	    	if(len>0)
    	    	{
    	        	if(strstr((const char *)data, ">") != NULL)//开关
    	        	{
    	        		send_debug("> OK\n");
    	        		break;
    	        	}
    	    	}
    	    }
    	    if(i>3)
    	    {
    	    	send_debug("> FAIL\n");
    	    	send_at_cmd("AT+NRB\n");
    	    	vTaskDelay(100 / portTICK_RATE_MS);
    	    	esp_deep_sleep(DEEP_SLEEP_GAP);
    	    }

    		char msg_pub[512];
    		len = sprintf(msg_pub,"{\"id\": \"1\",\"version\":\"1.0\",\"params\": {\"value\": {\"CurrentTemperature\":%.1f, \"RelativeHumidity\":%.1f, \"TemperatureB\":%.1f, \"Voltage\":%d, \"Andan\":%.3f,\"H2oTemp\":%.1f,\"LightLuxValue\":%d,\"PH\":%.2f}},\"method\": \"thing.event.property.post\"}\n",
    				           (float)tempA/10,(float)hum/10,(float)tempB/10,BT_ad,NH4_value,NH4_temp,lux,PHvalue);
    		send_at_cmd((char *)msg_pub);

    		vTaskDelay(100 / portTICK_RATE_MS);
    		char Sendcmd[1]={0x1a};
    		uart_write_bytes(UART_NUM_1,  Sendcmd, 1);
    		vTaskDelay(100 / portTICK_RATE_MS);

    	    for(i=0;i<30;i++) //
    	    {
    	    	len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 500 / portTICK_RATE_MS);
    	    	uart_write_bytes(UART_NUM_0,  (char *)data, len);

    	    	LED_flash(50);

    	    	if(len>0)
    	    	{
    	        	if(strstr((const char *)data, "+QMTPUB: 0,1,0") != NULL)//开关
    	        	{
    	        		send_debug("PUB OK\n");
    	        		vTaskDelay(100 / portTICK_RATE_MS);
    	        		break;
    	        	}
    	    	}
    	    }
    	    if(i>28)
    	    {
    	    	send_debug("PUB FAIL\n");
    	    	send_at_cmd("AT+NRB\n");
    	    	vTaskDelay(100 / portTICK_RATE_MS);
    	    	esp_deep_sleep(DEEP_SLEEP_GAP);
    	    }

    	    vTaskDelay(30000 / portTICK_RATE_MS);
//			esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_GAP);
//			esp_light_sleep_start();
    	}
    }

}

