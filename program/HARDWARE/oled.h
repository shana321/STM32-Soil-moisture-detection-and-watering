//////////////////////////////////////////////////////////////////////////////////	 
//
//  文 件 名   : oled.h
//  版 本 号   : v1.0
//  作    者   : xq
//  生成日期   : 2021-11-05
//  最近修改   : 
//  功能描述   : SSD1306 OLED驱动
//              说明:   对象化封装，使用时提供I2C/SPI/8080/6080接口的回调函数用于重写方法
//                      因为I/O不同等原因，每个硬件对象都有独特的属性和方法，需要重写(最底处有例子/模板)
//              结构：OLED->I2C->GPIO
//              ----------------------------------------------------------------
//              GND   电源地
//              VCC   接5V或3.3v电源
//              SCL   I2C时钟线
//              SDA   接PB15（SDA） 
//              RES   接PB11 如果是用4针iic接口这个脚可以不接
//              ----------------------------------------------------------------
//Copyright(C) xq 2021/11/5
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////
#ifndef _OLED_H_
#define _OLED_H_

#ifdef __cplusplus
	extern "C"{
#endif

#include <stdint.h>		
/*
	头文件的包含	
	#include <stdint.h>	
*/
/***OLED功能命令/操作码***/
typedef enum
{
	SSD1306_SET_CONTRAST_CONTROL		= 0x81,		//设置亮度，后还需要发一个亮度大小的字节,范围0x00~0xFF
	SSD1306_ENTIRE_DISPLAY_OFF			= 0xA4,		//跟随GRAM内容显示 (RESET)
	SSD1306_ENTIRE_DISPLAY_ON				= 0xA5,		//填充屏幕而不管GRAM
	SSD1306_INVERSE_DISPLAY_OFF			= 0xA6,		//正常显示模式 (RESET)
	SSD1306_INVERSE_DISPLAY_ON			= 0xA7,		//反色显示
	SSD1306_DISPALY_OFF							= 0xAE,		//休眠模式 (RESET)
	SSD1306_DISPLAY_ON							= 0xAF,		//工作模式
	/*
	0x00~0x0f:页寻址模式下设置列地址低4位
	0x10~0x1f:页寻址模式下设置列地址低4位
	*/
	SSD1306_SET_ADDRESSING_MODE			= 0x20,		//设置寻址模式，后还需要发一个字节	0x00:水平寻址 0x01:竖直寻址	0x02:页寻址
	SSD1306_SET_COLUMN_ADDRESS			= 0x21,		//设置列开始/结束地址，后发两个字节，低7位有效，范围0-127d，仅水平/竖直寻址模式可用
	SSD1306_SET_PAGE_ADDRESS				=	0x22,		//设置页开始/结束地址，后发两个字节，低3位有效，范围0-7d，仅水平/竖直寻址模式可用
	SSD1306_ADDRESS_MODE_HORIZONTAL = 0x00,
	SSD1306_ADDRESS_MODE_VERTICAL		= 0x01,
	SSD1306_ADDRESS_MODE_PAGE				= 0x02,
	/*
	0xB0~0xB7:页寻址模式下设置页地址 0~7d
	0x40~0x7F:低6位有效， 0~63d 设置start line register,改变ROW和RAM之间的映射，设置为0x41时，ROW0映射为RAM1，0x48时，ROW0映射为RAM8
	*/
	SSD1306_COLUMN_REMAP_OFF				= 0xA0,		//COLUMN 0 IS MAPPED TO SEG0 (RESET)(仅影响配置之后的数据)
	SSD1306_COLUMN_REMAP_ON					= 0xA1,		//COLUMN 127 IS MAPPED TO SEG0 (仅影响配置之后的数据)
	SSD1306_SET_MULTIPLEX_RATIO			= 0xA8,		//设置COM显示范围，后需要发一个字节，低6位有效，范围15d~63d(RESET:63 代表64MUX)
	SSD1306_ROW_REMAP_OFF						= 0xC0,		//正常显示模式，COM0-COM[N-1] N:MULTIPLEX RATIO (RESET) (立即对已有数据影响)
	SSD1306_ROW_REMAP_ON						= 0xC8,		//显示图像竖直镜像翻转，扫描模式COM[N-1]-COM0(立即对已有数据影响)
	SSD1306_SET_DISPLAY_OFFSET			= 0xD3,		//设置COM和ROW之间的映射，后跟一个字节，低6位有效，范围0-63d(RESET:0),设置为8时，COM8映射为ROW0.
}ssd1306_op_code;


/***封装库的变量、函数定义及声明***/
//回调函数cb所使用的消息（用于调用不同底层接口）
typedef enum
{
    OLED_MSG_I2C_INIT,
    OLED_MSG_I2C_START,
    OLED_MSG_I2C_STOP,
    OLED_MSG_I2C_WAIT_ACK,
    OLED_MSG_I2C_WRITE_BYTE
}OLED_CB_MSG;
typedef struct _oled_t OLED_t;
typedef uint8_t (*oled_cb)(OLED_t *slf,OLED_CB_MSG msg, uint8_t data_byte);
struct _oled_t
{
    oled_cb cb;
};

/***类方法***/
//OLED类的实例化，即创建一个oled对象
//oled_cb: oled类包含操作底层I/O的方法，用户需重写该方法
void New_OLED(OLED_t *slf, oled_cb oled_cb);

/***半私有方法**/
void OLED_WriteCmd(OLED_t *slf, uint8_t byte_to_write);

void OLED_WriteData(OLED_t *slf, uint8_t byte_to_write);

/***功能函数**/
void OLED_Clear(OLED_t *slf);
//OLED设置显示位置
void OLED_SetPos(OLED_t *slf, uint8_t x, uint8_t y);

void OLED_ON(OLED_t *slf);

void OLED_OFF(OLED_t *slf);

void OLED_Entire_Display_On(OLED_t *slf);

void OLED_Entire_Display_Off(OLED_t *slf);

void OLED_Display_Inverse_Enable(OLED_t *slf);
	
void OLED_Display_Inverse_Disable(OLED_t *slf);

void OLED_Display_Flip_Enable(OLED_t *slf);

void OLED_Display_Flip_Disable(OLED_t *slf);

void fill_picture(OLED_t *slf, uint8_t fill_Data);

void OLED_ShowChar(OLED_t *slf, uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size);

void OLED_ShowString(OLED_t *slf, uint8_t x, uint8_t y, char *str, uint8_t Char_Size);

void OLED_Init(OLED_t *slf);


#endif

/*
#include "Sofei2cMaster.h"
//注：调用了同样方式封装的软i2c库，用户可根据需求改成硬件I2C或自己的软I2C
//i2c对象
Sofei2cMaster oled_i2c;
//i2c底层操作回调
uint8_t at89c52_i2c_cb(Sofei2cMaster *i2c, uint8_t msg)
{
	switch(msg)
	{
		case SI2C_MSG_INIT:  		break;
		case SI2C_MSG_SET_SDA:   P1 |= 0x01;break;		
		case SI2C_MSG_RESET_SDA: P1 &= ~(0x01);break;
		case SI2C_MSG_READ_SDA:	 return P1&0x01;break;
		case SI2C_MSG_OUTPUT_SDA:  break; 
		case SI2C_MSG_INPUT_SDA:	break;	

		case SI2C_MSG_OUTPUT_SCL: 	break;
		case SI2C_MSG_SET_SCL:		P1 |= 0x02; break;
		case SI2C_MSG_RESET_SCL:	P1 &= ~(0x02);break;
		case SI2C_MSG_BUF_DELAY:    //停止信号与开始信号间的总线释放时间（TMP75为1.3us）
		case SI2C_MSG_SUSTA_DELAY:	//重复开始信号的建立时间（SDA=SCL=1的最短时间，TMP75为0.6us）
		case SI2C_MSG_HDSTA_DELAY:	//开始信号的保持时间（SDA拉低的最短时间，在这之后产生第一个时钟信号，TMP75为0.6us）
		case SI2C_MSG_SUSTO_DELAY:	//停止信号的建立时间（SDA拉高前SCL=1，SDA=0的保持时间，TMP75为0.6us）
		case SI2C_MSG_HDDAT_DELAY:	//数据保持时间(SCL=0后数据需要保持不变的时间，4ns,很短,IO频率超过250M才需要?)
		case SI2C_MSG_SUDAT_DELAY:	//数据建立时间(SCL=1前SDA需要保持状态的时间，100ns，在读写时序中被SCL延时覆盖，在ack中需使用，若频率低于10M则不需要)
		case SI2C_MSG_SCLL_DELAY:	//SCL低电平时间(读写时序中的主要延时，TMP75为1.3us min)
		case SI2C_MSG_SCLH_DELAY:	//SCL高电平时间(读写时序中的主要延时, TMP75为0.6us min)
		break;//这里都不使用额外延时，因为51，调用回调的时间已经够长了。
		//对于有些外设，读的时候需要适当慢一点，要不然外设反应不过来
	}
	return 0;
}
//i2c 重写方法
New_Sofei2cMaster(&oled_i2c, at89c52_i2c_cb);


OLED_t my_oled;
uint8_t oled_cb(OLED_t *slf,OLED_CB_MSG msg, uint8_t data_byte)
{
    switch (msg)
    {
        case OLED_MSG_I2C_INIT:
        {
            Sofei2cMaster_init(&oled_i2c);
            break;
        }
        case OLED_MSG_I2C_START:
        {
            Sofei2cMaster_start(&oled_i2c);
            break;
        }
        case OLED_MSG_I2C_STOP:
        {
            Sofei2cMaster_stop(&oled_i2c);
            break;
        }
        case OLED_MSG_I2C_WAIT_ACK:
        {
            Sofei2cMaster_wait_ack(&oled_i2c);
            break;
        }
        case OLED_MSG_I2C_WRITE_BYTE:
        {
            Sofei2cMaster_send_byte(&oled_i2c, data_byte);
            break;
        }

        default: break;
    }
}
//重写方法
New_OLED(&my_oled, oled_cb);
*/
