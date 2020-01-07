#include "sys.h"
#include "delay.h"  
#include "usart.h"   

#include "lcd.h"

#include "touch.h" 
#include "lvgl.h"
#include "lv_examples.h"
#include "timer.h"
//读点函数实现
bool my_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data)
{
    static lv_coord_t last_x = 0;
    static lv_coord_t last_y = 0;

    /*Save the state and save the pressed coordinate*/
	data->state = tp_dev.sta&TP_PRES_DOWN ?  LV_INDEV_STATE_PR : LV_INDEV_STATE_REL; 
	//    if(data->state == LV_INDEV_STATE_PR) touchpad_get_xy(&last_x, &last_y);
   if(tp_dev.sta&TP_PRES_DOWN)			//触摸屏被按下,判断里的内容要自己实现
		{	
		 	if(tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{	
				//TP_Draw_Big_Point(tp_dev.x[0],tp_dev.y[0],RED);		//画图
				last_x=tp_dev.x[0];
				last_y=tp_dev.y[0];
			}
		}
    /*Set the coordinates (if released use the last pressed coordinates)*/
    data->point.x = last_x;
    data->point.y = last_y;

    return false; /*Return `false` because we are not buffering and no more data to read*/
}
//打点函数实现
void my_disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p)
{
    int32_t x, y;
    for(y = area->y1; y <= area->y2; y++) {
        for(x = area->x1; x <= area->x2; x++) {
            //set_pixel(x, y, *color_p);  /* Put a pixel to the display.*/
            LCD_Fast_DrawPoint( x, y,color_p->full );//自己的打点函数
			color_p++;
        }
    }

    lv_disp_flush_ready(disp);         /* Indicate you are ready with the flushing*/
}

int main(void)
{ 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200
	TIM3_Int_Init(50-1,8400-1);

 	LCD_Init();					//LCD初始化 
	tp_dev.init();				//触摸屏初始化
	
 	
//初始化littlevGL	
	lv_init();
//显示缓冲区
	static lv_disp_buf_t disp_buf;
	static lv_color_t buf[LV_HOR_RES_MAX * 10];                     /*Declare a buffer for 10 lines*/
	lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);    /*Initialize the display buffer*/
//像素打点
	lv_disp_drv_t disp_drv;               /*Descriptor of a display driver*/
	lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
	disp_drv.flush_cb = my_disp_flush;    /*Set your driver function*/
	disp_drv.buffer = &disp_buf;          /*Assign the buffer to the display*/
	lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/	
//输入
	lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv);             /*Descriptor of a input device driver*/
	indev_drv.type = LV_INDEV_TYPE_POINTER;    /*Touch pad is a pointer-like device*/
	indev_drv.read_cb = my_touchpad_read;      /*Set your driver function*/
	lv_indev_drv_register(&indev_drv);         /*Finally register the driver*/
	
	
//建立一按钮
	lv_theme_t * th = lv_theme_night_init(20, NULL);
	lv_test_theme_1(th);

	while(1)
	{
	 	lv_task_handler();
		tp_dev.scan(0); 	
		
		   
	}
}
