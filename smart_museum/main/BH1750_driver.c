#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "BH1750_driver.h"


#define I2C_BH1750_MASTER_SCL_IO          26                /*!< gpio number for I2C master clock */
#define I2C_BH1750_MASTER_SDA_IO          25               /*!< gpio number for I2C master data  */
#define I2C_BH1750_MASTER_NUM             I2C_NUM_0        /*!< I2C port number for master dev */

#define BH1750_SENSOR_ADDR                 0x46

/**
 * @brief i2c master initialization
 */
esp_err_t bh1750_i2c_init(void)
{
    int i2c_master_port = I2C_BH1750_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_BH1750_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_BH1750_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}


static esp_err_t bh1750_i2c_write(i2c_port_t i2c_num,  uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR  | WRITE_BIT, ACK_CHECK_EN);

    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t bh1750_i2c_read(i2c_port_t i2c_num,  uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR  | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

uint16_t bh1750_value_get(void)
{
	uint8_t cmd_temp[3]={0x01,0x10,0x20};
	uint8_t dat_temp[2];
    uint16_t lux_temp;
    float lux_temp1;

	bh1750_i2c_write(I2C_BH1750_MASTER_NUM, cmd_temp , 1);
	bh1750_i2c_write(I2C_BH1750_MASTER_NUM, cmd_temp+1 , 1);
	bh1750_i2c_write(I2C_BH1750_MASTER_NUM, cmd_temp+2 , 1);

	bh1750_i2c_read(I2C_BH1750_MASTER_NUM,  dat_temp, 2);

	lux_temp=dat_temp[0]<<8|dat_temp[1];
	lux_temp1=(float)lux_temp/1.2;
	//* lux=(uint16_t)lux_temp1;
	return (uint16_t)lux_temp1;
}
