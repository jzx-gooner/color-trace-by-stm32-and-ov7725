#include "stm32f10x.h"
#include "bsp_ov7725.h"
#include "bsp_ili9341_lcd.h"
#include "EasyTracer.h"
#include "bsp_led.h"  

extern uint8_t Ov7725_vsync;         
RESULT Resu;           
TARGET_CONDI Condition={50,80,20,250,20,200,40,40,320,240};


int main(void) 	
{
	LED_GPIO_Init();
	LCD_Init();                       	//液晶初始化 
	Ov7725_GPIO_Config();             	//ov7725 gpio 初始化 
	while(Ov7725_Init() != SUCCESS);  	//ov7725 寄存器配置初始化 	
	VSYNC_Init();	                    //ov7725 场信号线初始化 
	Ov7725_vsync = 0;
	
	while(1)
	{
		if( Ov7725_vsync == 2 )
		{
            Ov7725_vsync = 0;	
			if(Trace(&Condition, &Resu))
			{			
				LCD_Clear(Resu.x-Resu.w/2,Resu.y-Resu.h/2,Resu.w,1,0xf800);
				LCD_Clear(Resu.x-Resu.w/2,Resu.y-Resu.h/2,1,Resu.h,0xf800);
				LCD_Clear(Resu.x-Resu.w/2,Resu.y+Resu.h/2,Resu.w,1,0xf800);
				LCD_Clear(Resu.x+Resu.w/2,Resu.y-Resu.h/2,1,Resu.h,0xf800);
				LCD_Clear(Resu.x-2,Resu.y-2,4,4,0xf800);
			}	
		}
	}
}
