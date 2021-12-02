//////////////////////////////////////////////////////////////////////////////////	 
//
//  文 件 名   : oled.c
//  版 本 号   : v1.0
//  作    者   : xq
//  生成日期   : 2021-11-05
//  最近修改   : 
//  功能描述   : SSD1306 OLED驱动
//              说明:   对象化封装，使用时提供I2C/SPI/8080/6080接口的回调函数用于重写方法
//                      因为I/O不同等原因，每个硬件都有独特的属性和方法，需要重写(在oled.h最底处有例子/模板)
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
#include "oled.h"
#include "oledfont.h"

#define I2C_Init()                  slf->cb(slf, OLED_MSG_I2C_INIT, 0)
#define I2C_Start()                 slf->cb(slf, OLED_MSG_I2C_START, 0)
#define I2C_Stop()                  slf->cb(slf, OLED_MSG_I2C_STOP, 0)
#define I2C_WaitAck()               slf->cb(slf, OLED_MSG_I2C_WAIT_ACK, 0)
#define I2C_WriteByte(Data_Byte)    slf->cb(slf, OLED_MSG_I2C_WRITE_BYTE, Data_Byte)

//OLED类的实例化，即创建一个oled对象
//oled_cb: oled类包含操作底层I/O的方法，用户需重写该方法
void New_OLED(OLED_t *slf, oled_cb oled_cb)
{
	slf->cb = oled_cb;
}
//OLED写命令
void OLED_WriteCmd(OLED_t *slf, uint8_t byte_to_write)
{
    I2C_Start();
    I2C_WriteByte(0x78);    //Slave address,SA0=0
    I2C_WaitAck();

    I2C_WriteByte(0x00);
    I2C_WaitAck();

    I2C_WriteByte(byte_to_write);
    I2C_WaitAck();
    I2C_Stop();
}
//OLED写数据
void OLED_WriteData(OLED_t *slf, uint8_t byte_to_write)
{
    I2C_Start();
    I2C_WriteByte(0x78);    //Slave address,SA0=0
    I2C_WaitAck();

    I2C_WriteByte(0x40);
    I2C_WaitAck();

    I2C_WriteByte(byte_to_write);
    I2C_WaitAck();
    I2C_Stop();
}


void OLED_Clear(OLED_t *slf)
{
  unsigned char i,n;
  for(i=0; i<8; i++)
  {
    OLED_WriteCmd(slf, 0xb0 + i); //设置页地址（0~7）
    OLED_WriteCmd(slf, 0x00); //设置显示位置—列低地址
    OLED_WriteCmd(slf, 0x10); //设置显示位置—列高地址
    for(n=0; n<128; n++)
        OLED_WriteData(slf, 0x00); //写0x00到屏幕寄存器上
  }
}

//OLED设置显示位置 (仅Page Addressing模式下)
void OLED_SetPos(OLED_t *slf, uint8_t x, uint8_t y)
{
	OLED_WriteCmd(slf, 0xb0 + y);
	OLED_WriteCmd(slf, ((x & 0xf0) >> 4) | 0x10);
	OLED_WriteCmd(slf, x & 0x0f);
}
 
//------将OLED从休眠中唤醒------
void OLED_ON(OLED_t *slf)
{
  OLED_WriteCmd(slf, 0X8D);  //设置电荷泵
  OLED_WriteCmd(slf, 0X14);  //开启电荷泵
  OLED_WriteCmd(slf, SSD1306_DISPLAY_ON);  //OLED唤醒
}
 
//------让OLED休眠 -- 休眠模式下,OLED功耗不到10uA------
void OLED_OFF(OLED_t *slf)
{
  OLED_WriteCmd(slf, 0X8D);  //设置电荷泵
  OLED_WriteCmd(slf, 0X10);  //关闭电荷泵
  OLED_WriteCmd(slf, SSD1306_DISPALY_OFF);  //OLED休眠
}

//OLED全填充显示
void OLED_Entire_Display_On(OLED_t *slf)
{
	OLED_WriteCmd(slf, SSD1306_ENTIRE_DISPLAY_ON);
}
//OLED全填充显示
void OLED_Entire_Display_Off(OLED_t *slf)
{
	OLED_WriteCmd(slf, SSD1306_ENTIRE_DISPLAY_OFF);
}

//OLED反色显示(bit0代表亮)
void OLED_Display_Inverse_Enable(OLED_t *slf)
{
	OLED_WriteCmd(slf, SSD1306_INVERSE_DISPLAY_ON);
}
//OLED正色显示(bit1代表亮)
void OLED_Display_Inverse_Disable(OLED_t *slf)
{
	OLED_WriteCmd(slf, SSD1306_INVERSE_DISPLAY_OFF);
}
//OLED翻转显示
void OLED_Display_Flip_Enable(OLED_t *slf)
{
	OLED_WriteCmd(slf, SSD1306_COLUMN_REMAP_ON);
	OLED_WriteCmd(slf, SSD1306_ROW_REMAP_ON);
}
//OLED不翻转显示
void OLED_Display_Flip_Disable(OLED_t *slf)
{
	OLED_WriteCmd(slf, SSD1306_COLUMN_REMAP_OFF);
	OLED_WriteCmd(slf, SSD1306_ROW_REMAP_OFF);
}
/********************************************
// fill_Picture
********************************************/

void fill_picture(OLED_t *slf, uint8_t fill_Data)
{
	uint8_t m,n;
	for(m=0;m<8;m++)
	{
		OLED_WriteCmd(slf, 0xb0+m);		//page0-page1
		OLED_WriteCmd(slf, 0x00);		//low column start address
		OLED_WriteCmd(slf, 0x10);		//high column start address
		for(n=0;n<128;n++)
			{
				OLED_WriteData(slf, fill_Data);
			}
	}
}
//在指定位置显示一个字符,包括部分字符
//x:0~63
//y:0~7				 
//size:选择字体 16/12 
void OLED_ShowChar(OLED_t *slf, uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size)
{      	
	uint8_t c=0,i=0;	
    c=chr-' ';//得到偏移后的值			
    if(x>125){x=0;y=y+2;}
        if(Char_Size ==16)
        {
            OLED_SetPos(slf, x, y);	
            for(i=0;i<8;i++)
                OLED_WriteData(slf, F8X16[c*16+i]);

            OLED_SetPos(slf, x, y+1);
            for(i=0;i<8;i++)
                OLED_WriteData(slf, F8X16[c*16+i+8]);
        }
        else
        {	
            OLED_SetPos(slf, x, y);
            for(i=0;i<6;i++)
                OLED_WriteData(slf, F6x8[c][i]);
            
        }
}
//显示一个字符串
void OLED_ShowString(OLED_t *slf, uint8_t x, uint8_t y, char *str, uint8_t Char_Size)
{
	uint8_t j=0;
	while (str[j]!='\0')
	{		OLED_ShowChar(slf, x, y, str[j], Char_Size);
			x+=8;
		if(x>120){x=0;y+=2;}
			j++;
	}
}

void OLED_Init(OLED_t *slf)
{
	I2C_Init();
 	OLED_WriteCmd(slf, SSD1306_DISPALY_OFF);//--turn off oled panel
	
	OLED_WriteCmd(slf, SSD1306_SET_CONTRAST_CONTROL);//--set contrast control register
	OLED_WriteCmd(slf, 0xCF); // Set SEG Output Current Brightness
	
	OLED_WriteCmd(slf, SSD1306_ENTIRE_DISPLAY_OFF);// Disable Entire Display On (0xa4/0xa5)
	OLED_WriteCmd(slf, SSD1306_INVERSE_DISPLAY_OFF);// Disable Inverse Display On (0xa6/a7) 
	
	OLED_WriteCmd(slf, SSD1306_SET_ADDRESSING_MODE);//Set Addressing Mode (0x00/0x01/0x02)
	OLED_WriteCmd(slf, SSD1306_ADDRESS_MODE_PAGE);//Page Addressing Mode
	
	OLED_WriteCmd(slf, 0x40);//--set start line address  Set Mapping RAM Display Start Line (0x40~0x7F)
	
	OLED_WriteCmd(slf, SSD1306_COLUMN_REMAP_ON);//--Set SEG/Column Mapping     0xa0???? 0xa1??
	OLED_WriteCmd(slf, SSD1306_ROW_REMAP_ON);//Set COM/Row Scan Direction   0xc0???? 0xc8??
	
	OLED_WriteCmd(slf, SSD1306_SET_MULTIPLEX_RATIO);//--set multiplex ratio(1 to 64)
	OLED_WriteCmd(slf, 0x3f);//--1/64 duty
	
	OLED_WriteCmd(slf, SSD1306_SET_DISPLAY_OFFSET);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WriteCmd(slf, 0x00);//-not offset
	
	OLED_WriteCmd(slf, 0xDA);//--set com pins hardware configuration
	OLED_WriteCmd(slf, 0x12);
	
	OLED_WriteCmd(slf, 0xd5);//--set display clock divide ratio/oscillator frequency
	OLED_WriteCmd(slf, 0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
	
	OLED_WriteCmd(slf, 0xD9);//--set pre-charge period
	OLED_WriteCmd(slf, 0xF1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	
	OLED_WriteCmd(slf, 0xDB);//--set vcomh
	OLED_WriteCmd(slf, 0x40);//Set VCOM Deselect Level
	
	OLED_WriteCmd(slf, 0x8D);//--set Charge Pump enable/disable
	OLED_WriteCmd(slf, 0x14);//--set(0x10) disable
	
	OLED_WriteCmd(slf, SSD1306_DISPLAY_ON);//--turn on oled panel
	
	OLED_SetPos(slf, 0, 0); 	
	OLED_Clear(slf);//OLED清屏
}			    


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
		case SI2C_MSG_SET_SDA:   OLED_I2C_SDA_GPIO_Port->ODR |= OLED_I2C_SDA_Pin;break;
		case SI2C_MSG_RESET_SDA: OLED_I2C_SDA_GPIO_Port->ODR &= ~OLED_I2C_SDA_Pin;break;
		case SI2C_MSG_READ_SDA:	 return OLED_I2C_SDA_GPIO_Port->IDR & OLED_I2C_SDA_Pin;
		case SI2C_MSG_OUTPUT_SDA:  break; 
		case SI2C_MSG_INPUT_SDA:	break;	

		case SI2C_MSG_OUTPUT_SCL: 	break;
		case SI2C_MSG_SET_SCL:		OLED_I2C_SCL_GPIO_Port->ODR |= OLED_I2C_SCL_Pin;break;
		case SI2C_MSG_RESET_SCL:	OLED_I2C_SCL_GPIO_Port->ODR &= ~OLED_I2C_SCL_Pin;break;
		case SI2C_MSG_BUF_DELAY:    //停止信号与开始信号间的总线释放时间（SSD1306、TMP75为1.3us）
		case SI2C_MSG_SUSTA_DELAY:	//重复开始信号的建立时间（SCL=1的最短时间，SSD1306、TMP75为0.6us）
		case SI2C_MSG_HDSTA_DELAY:	//开始信号的保持时间（SDA拉低的最短时间，在这之后产生第一个时钟信号，SSD1306、TMP75为0.6us）
		case SI2C_MSG_SUSTO_DELAY:	//停止信号的建立时间（SDA拉高前SCL=1，SDA=0的保持时间，SSD1306、TMP75为0.6us）
		case SI2C_MSG_HDDAT_DELAY:	//数据保持时间(SCL=0后数据需要保持不变的时间，TMP75为4ns,很短,IO频率超过250M才需要?)
		case SI2C_MSG_SUDAT_DELAY:	//数据建立时间(SCL=1前SDA需要保持状态的时间，SSD1306、TMP75为100ns，在读写时序中被SCL延时覆盖，在ack中需使用，若频率低于10M则不需要?)
		case SI2C_MSG_SCLL_DELAY:		//SCL低电平时间(读写时序中的主要延时，TMP75为1.3us min)
		case SI2C_MSG_SCLH_DELAY:		//SCL高电平时间(读写时序中的主要延时, TMP75为0.6us min)
		break;
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
						Sofei2cMaster oled_i2c;
						New_Sofei2cMaster(&oled_i2c, stm32_i2c_cb);
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

