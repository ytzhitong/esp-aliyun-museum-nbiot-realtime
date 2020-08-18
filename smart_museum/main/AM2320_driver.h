#ifndef _AM2320_DRIVE_H_
#define _AM2320_DRIVE_H_


uint8_t  AM2320_get_value(int* hum,int* temp);
// esp_err_t am2320_i2c_init(void);

void AM2320_gpio_init(void);

#endif
