#include "bsp_ov7725.h"
#include "bsp_sccb.h"
#include "bsp_ili9341_lcd.h"

typedef struct Reg
{
	uint8_t Address;			       /*寄存器地址*/
	uint8_t Value;		           /*寄存器值*/
}Reg_Info;


Reg_Info Sensor_Config[] =       //寄存器参数配置 
{
	{CLKRC,     0x00}, /*clock config*/
	{COM7,      0x46}, /*QVGA RGB565 */
	{HSTART,    0x3f},
	{HSIZE,     0x50},
	{VSTRT,     0x03},
	{VSIZE,     0x78},
	{HREF,      0x00},
	{HOutSize,  0x50},
	{VOutSize,  0x78},
	{EXHCH,     0x00},

	/*DSP control*/
	{TGT_B,     0x7f},
	{FixGain,   0x09},
	{AWB_Ctrl0, 0xe0},
	{DSP_Ctrl1, 0xff},
	{DSP_Ctrl2, 0x20},
	{DSP_Ctrl3,	0x00},
	{DSP_Ctrl4, 0x00},

	/*AGC AEC AWB*/
	{COM8,		  0xf0},
	{COM4,		  0x81}, /*Pll AEC CONFIG*/
	{COM6,		  0xc5},
	{COM9,		  0x21},
	{BDBase,	  0xFF},
	{BDMStep,	  0x01},
	{AEW,		  0x34},
	{AEB,		  0x3c},
	{VPT,		  0xa1},
	{EXHCL,		  0x00},
	{AWBCtrl3,    0xaa},
	{COM8,		  0xff},
	{AWBCtrl1,    0x5d},

	{EDGE1,		  0x0a},
	{DNSOff,	  0x01},
	{EDGE2,		  0x01},
	{EDGE3,		  0x01},

	{MTX1,		  0x5f},
	{MTX2,		  0x53},
	{MTX3,		  0x11},
	{MTX4,		  0x1a},
	{MTX5,		  0x3d},
	{MTX6,		  0x5a},
	{MTX_Ctrl,    0x1e},

	{BRIGHT,	  0x00},
	{CNST,		  0x25},
	{USAT,		  0x65},
	{VSAT,		  0x65},
	{UVADJ0,	  0x81},
	{SDE,		  0x06},
	
    /*GAMMA config*/
	{GAM1,		  0x0c},
	{GAM2,		  0x16},
	{GAM3,		  0x2a},
	{GAM4,		  0x4e},
	{GAM5,		  0x61},
	{GAM6,		  0x6f},
	{GAM7,		  0x7b},
	{GAM8,		  0x86},
	{GAM9,		  0x8e},
	{GAM10,		  0x97},
	{GAM11,		  0xa4},
	{GAM12,		  0xaf},
	{GAM13,		  0xc5},
	{GAM14,		  0xd7},
	{GAM15,		  0xe8},
	{SLOP,		  0x20},

	{HUECOS,	  0x80},
	{HUESIN,	  0x80},
	{DSPAuto,	  0xff},
	{DM_LNL,	  0x00},
	{BDBase,	  0x99},
	{BDMStep,	  0x03},
	{LC_RADI,	  0x00},
	{LC_COEF,	  0x13},
	{LC_XC,		  0x08},
	{LC_COEFB,    0x14},
	{LC_COEFR,    0x17},
	{LC_CTR,	  0x05},
	
	{COM3,		  0xd0},/*Horizontal mirror image*/

	/*night mode auto frame rate control*/
	{COM5,		  0xf5},	/*在夜视环境下，自动降低帧率，保证低照度画面质量*/
	//{COM5,	  0x31},	/*夜视环境帧率不变*/
};

u8 OV7725_REG_NUM = sizeof(Sensor_Config)/sizeof(Sensor_Config[0]);	  //结构体数组成员数目

uint8_t Ov7725_vsync = 0;	       //帧同步信号标志，在中断函数和main函数里面使用



static void FIFO_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD , ENABLE);
	                           
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
		
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;	//1W LED 灯控制 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	GPIO_SetBits(GPIOA, GPIO_Pin_8);
	
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;	//PD3(FIFO_WEN--FIFO写使能)
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;	//PB5(FIFO_WRST--FIFO写复位)
    GPIO_Init(GPIOB, &GPIO_InitStructure);
		
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;	//PA2(FIFO_RRST--FIFO读复位) PA3(FIFO_OE--FIFO输出使能)
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;//PC5(FIFO_RCLK-FIFO读时钟)
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //PB8-PB15(FIFO_DATA--FIFO输出数据)
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    FIFO_CS_L();	  					//拉低使FIFO输出使能
    FIFO_WE_H();   						//拉高使FIFO写允许
}


void Ov7725_GPIO_Config(void)
{
	SCCB_GPIO_Config();
	FIFO_GPIO_Config();
}


static void VSYNC_GPIO_Configuration(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	  /*PA0---VSYNC*/
	
    GPIO_InitStructure.GPIO_Pin   =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}


static void VSYNC_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


//==============================================================
//描述  ：OV7725 VSYNC中断管脚配置
//==============================================================
//               ___                            ___
// VSYNC:     __|   |__________________________|   |__     
///
static void VSYNC_EXTI_Configuration(void)
{
    EXTI_InitTypeDef   EXTI_InitStructure;

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);   //线中断口配置
    EXTI_InitStructure.EXTI_Line      = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode      = EXTI_Mode_Interrupt;      //中断模式
    //EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising ;     //上升沿触发
	EXTI_InitStructure.EXTI_Trigger   = EXTI_Trigger_Falling ; 
    EXTI_InitStructure.EXTI_LineCmd   = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_GenerateSWInterrupt(EXTI_Line0);                       	//中断挂到 EXTI_Line0  线
}


//==============================================================
// 描述  ：OV7725 VSYNC中断相关配置
//==============================================================
void VSYNC_Init(void)
{
    VSYNC_GPIO_Configuration();
    VSYNC_EXTI_Configuration();
    VSYNC_NVIC_Configuration();
}


//==============================================================
//描述  ：Sensor初始化
//输出  ：返回1成功，返回0失败
//==============================================================
ErrorStatus Ov7725_Init(void)
{
	uint16_t i = 0;
	uint8_t Sensor_IDCode = 0;	
		
	if( 0 == SCCB_WriteByte ( 0x12, 0x80 ) )             //复位sensor 	
		return ERROR ;
	
	if( 0 == SCCB_ReadByte( &Sensor_IDCode, 1, 0x0b ) )	 //读取sensor ID号
		return ERROR;
	
	if(Sensor_IDCode == OV7725_ID)
	{
		for( i = 0 ; i < OV7725_REG_NUM ; i++ )
		{
			if( 0 == SCCB_WriteByte(Sensor_Config[i].Address, Sensor_Config[i].Value) )
				return ERROR;
		}
	}
	else
		return ERROR;
	return SUCCESS;
}


//==============================================================
//       320
// -------------------
//|                   |
//|                   |
//|                   |  240
//|                   |
//|                   |
// -------------------
//==============================================================
void ImagDisp(void)
{
	uint16_t i, j;
	uint16_t Camera_Data;
	Lcd_GramScan(3);
	for(i = 0; i < 240; i++)
	{
		for(j = 0; j < 320; j++)
		{
			READ_FIFO_PIXEL(Camera_Data);	//从FIFO读出一个rgb565像素到Camera_Data变量   16bit
			LCD_WR_Data(Camera_Data);       //往LCD GRAM写入数据     
		}
	}
}
