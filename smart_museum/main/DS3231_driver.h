//#include "sys.h"

#include "esp_err.h"

#ifndef __DS3231_driver_H__
#define __DS3231_driver_H__


typedef struct
{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint32_t w_year;
	uint8_t  w_month;
	uint8_t  w_date;
	uint8_t  week;
}_calendar_obj;
extern _calendar_obj calendar;	//日历结构体

extern uint8_t const mon_table[12];	//月份日期数据表

esp_err_t DS3231_i2c_init(void);

void get_show_time(void);
//uint8_t RTC_Get_Week(u16 year,uint8_t month,uint8_t day);
void DS3231_Set(uint8_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec);//设置时间

#endif
