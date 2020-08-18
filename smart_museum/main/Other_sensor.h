#ifndef _OTHER_SENSOR_H_
#define _OTHER_SENSOR_H_

#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEEP_SLEEP_GAP    30000000
#define LIGHT_SLEEP_GAP    30000000

uint32_t ADC_value_get(adc1_channel_t channel, adc_atten_t atten);

#define BUF_SIZE (512)

#define GPIO_OUTPUT_IO_0    27
#define GPIO_LED_0          13
#define GPIO_UV_0           32
#define GPIO_NB_RES         26

#define GPIO_INPUT_MODE     21

void control_gpio_init(void);
void LED_flash(uint16_t gap);

void uart0_debug_init(void);
void uart1_nbiot_init(void);
void send_at_cmd(char *payload);

void send_debug(char *payload);

void uart2_ze03_init(void);
void uart2_NH4_init(void);
void NH4_read(void);

#endif
