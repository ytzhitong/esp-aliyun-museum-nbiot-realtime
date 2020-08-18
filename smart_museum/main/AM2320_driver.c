#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "AM2320_driver.h"

//#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
//#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
//#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
//#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
//#define ACK_VAL 0x0                             /*!< I2C ack value */
//#define NACK_VAL 0x1                            /*!< I2C nack value */
//
//#define I2C_AM2320_MASTER_NUM             I2C_NUM_0
//#define AM2320_SENSOR_ADDR                0xB8
//
//uint8_t IIC_RX_Buffer[10];
//
//
// static esp_err_t AM2320_wakeUp(i2c_port_t i2c_num)
// {
//	    int ret;
//	    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//
//	    i2c_master_start(cmd);
//	    i2c_master_write_byte(cmd, AM2320_SENSOR_ADDR  | WRITE_BIT, ACK_CHECK_EN);
//
//	    i2c_master_stop(cmd);
//	    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//	    i2c_cmd_link_delete(cmd);
//
//	    return ret;
// }
//
// static esp_err_t AM2320_send_cmd(i2c_port_t i2c_num)
// {
//	    int ret;
//	    uint8_t IIC_TX_Buffer[]={0x03,0x00,0x04}; //读温湿度命令（无CRC校验）
//
//	    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//	    i2c_master_start(cmd);
//	    i2c_master_write_byte(cmd, AM2320_SENSOR_ADDR  | WRITE_BIT, ACK_CHECK_EN);
//	    i2c_master_write(cmd, IIC_TX_Buffer, 3, ACK_CHECK_EN);
//	    i2c_master_stop(cmd);
//	    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//	    i2c_cmd_link_delete(cmd);
//
//	    return ret;
// }
//
// static esp_err_t AM2320_get_data(i2c_port_t i2c_num)
// {
//	    int ret;
//
//	    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//	    i2c_master_start(cmd);
//	    i2c_master_write_byte(cmd, AM2320_SENSOR_ADDR  | READ_BIT, ACK_CHECK_EN);
//	    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//	    i2c_cmd_link_delete(cmd);
//
//	    //os_delay_us(40);//等待至少30us。
//	    ets_delay_us(40);
//
//	    cmd = i2c_cmd_link_create();
//	    i2c_master_read(cmd, IIC_RX_Buffer, 8, ACK_CHECK_EN);
//	    i2c_master_stop(cmd);
//	    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//	    i2c_cmd_link_delete(cmd);
//
//	    return ret;
// }
//
// ///计算CRC校验码
// unsigned int CRC16(unsigned char *ptr, unsigned char len)
// {
//    unsigned int crc=0xffff;
//    unsigned char i;
//    while(len--)
//    {
//        crc ^=*ptr++;
//        for(i=0;i<8;i++)
// 	   {
// 	       if(crc & 0x1)
// 		   {
// 		      crc>>=1;
// 			  crc^=0xa001;
// 		   }
// 		   else
// 		   {
// 		      crc>>=1;
// 		   }
// 	   }
//    }
//    return crc;
// }
//
// ///检测CRC校验码是否正确
// unsigned char CheckCRC(unsigned char *ptr,unsigned char len)
// {
//   unsigned int crc;
// 	crc=(unsigned int)CRC16(ptr,len-2);
// 	if(ptr[len-1]==(crc>>8) && ptr[len-2]==(crc & 0x00ff))
// 	{
// 	    return 0xff;
// 	}
// 	else
// 	{
// 	   return 0x0;
// 	}
// }
//
// void  AM2320_get_value(int* hum,int* temp)
// {
//	    int a,b;
//
//	    AM2320_wakeUp(I2C_AM2320_MASTER_NUM);   //唤醒传感器
//	    AM2320_send_cmd(I2C_AM2320_MASTER_NUM); //发送读取指令
//	    vTaskDelay(2 / portTICK_RATE_MS);       //等待2ms
//	    AM2320_get_data(I2C_AM2320_MASTER_NUM); //读取寄存器
//
//	    if(CheckCRC(IIC_RX_Buffer,8))
//		 {
//			a =(( IIC_RX_Buffer[2]<<8 )+ IIC_RX_Buffer[3]);
//			b =(( IIC_RX_Buffer[4]<<8)+ IIC_RX_Buffer[5]);
//
//			*hum = a/10;
//			*temp = b/10;
//		 }
//		else
//		 {
//			printf("Data: CRC Wrong\n");
//		 }
// }
//
//
//#define AM2320_I2C_SDA_IO 17
//#define AM2320_I2C_SCL_IO 5
//#define AM2320_I2C_FREQ_HZ 10000
//#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
//#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
// /**
//  * @brief i2c master initialization
//  */
//  esp_err_t am2320_i2c_init(void)
// {
//     int i2c_master_port = I2C_AM2320_MASTER_NUM;
//     i2c_config_t conf;
//     conf.mode = I2C_MODE_MASTER;
//     conf.sda_io_num = AM2320_I2C_SDA_IO;
//     conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.scl_io_num = AM2320_I2C_SCL_IO;
//     conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.master.clk_speed = AM2320_I2C_FREQ_HZ;
//     i2c_param_config(i2c_master_port, &conf);
//     return i2c_driver_install(i2c_master_port, conf.mode,
//                               I2C_MASTER_RX_BUF_DISABLE,
//                               I2C_MASTER_TX_BUF_DISABLE, 0);
// }


#define GPIO_AM2320    32
#define GPIO_AM2320_PIN_SEL  1ULL<<GPIO_AM2320

void AM2320_gpio_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO0
    io_conf.pin_bit_mask = GPIO_AM2320_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}


uint8_t AM2320_read_byte(void)
{
	uint16_t j = 0;
	uint8_t data = 0, bit = 0;

    for(uint8_t i = 0; i < 8; i++)
    {
        // 检测上次低电平是否结束
        while(!gpio_get_level(GPIO_AM2320))
        {
            // 防止进入死循环
            if(++j>=50000)
            {
                break;
            }
        }
        // 延时Min=26us Max70us 跳过数据"0" 的高电平
        ets_delay_us(30);

        // 判断传感器发送数据位
        bit = gpio_get_level(GPIO_AM2320);
        j = 0;
        // 等待高电平结束
        while(gpio_get_level(GPIO_AM2320))
        {
            // 防止进入死循环
            if(++j >= 50000)
            {
                break;
            }
        }
        data <<= 1;
        data |= bit;
    }
    return data;
}


uint8_t  AM2320_get_value(int* hum,int* temp)
{
	int i=0;
	int j=0;
	uint8_t HumHigh, HumLow, TempHigh, TempLow, TempChecksum, Temp;

	gpio_set_direction(GPIO_AM2320, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_AM2320, 0);		//拉低1000us
	ets_delay_us(1000);

	gpio_set_level(GPIO_AM2320, 1);
	ets_delay_us(30); //释放总线30us

	gpio_set_direction(GPIO_AM2320, GPIO_MODE_INPUT);
	ets_delay_us(6); //延时6us，等待稳定
    while(gpio_get_level(GPIO_AM2320)==0) //判断从机是否有低电平响应信号
    {
       i++;
       ets_delay_us(1);
       if(i>85)
    	   break;
    }
    while(gpio_get_level(GPIO_AM2320)==1) //判断从机是否有高电平响应信号
    {
       j++;
       ets_delay_us(1);
       if(j>85)
    	   break;
    }

    if(i>80||j>80||i<30||j<30) //响应异常
    return 1;

    // 接收数据
    HumHigh   = AM2320_read_byte();
    HumLow    = AM2320_read_byte();
    TempHigh  = AM2320_read_byte();
    TempLow   = AM2320_read_byte();
    TempChecksum = AM2320_read_byte();

    Temp = (uint8_t)(HumHigh + HumLow + TempHigh + TempLow);
    if(Temp!=TempChecksum)
    return 2;

    *hum =(( HumHigh<<8 )+ HumLow);
    *temp =(( TempHigh<<8)+ TempLow);

    return 0;
}

