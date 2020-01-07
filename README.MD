#以正点原子探索者开发为例，其他板子和屏幕也是可以的，因为只需要读点和写点
这是我已经移植成功的例子基于STM32F407，修改读点函数和写点函数其他不用变就变成你的



#第一步，准备工作
- 先准备一个工程
- 自己要先实现屏幕的写点，如果要触摸功能还要实现读点。


#第二步，下载和移植
1. 官方的github项目地址[https://github.com/littlevgl/lvgl](https://github.com/littlevgl/lvgl)
2. 下载的解压后名称为lvgl-master，重命名为lvgl（下面也用lvgl也是这个文件夹）
3. 将lvgl文件夹放到你的项目中
4. 将lvgl文件夹中的lv_conf_template.h复制到与lvgl同级目录，并重命名为lv_conf
5. 打开lv_conf，将#if 0改为 # if 1，设置你的命名的宽高像素，以及颜色模式，如下代码所示
```
/**
 * @file lv_conf.h
 *
 */

/*
 * COPY THIS FILE AS `lv_conf.h` NEXT TO the `lvgl` FOLDER
 */

#if 1 /*Set it to "1" to enable content*///改为1

#ifndef LV_CONF_H
#define LV_CONF_H
/* clang-format off */

#include <stdint.h>

/*====================
   Graphical settings
 *====================*/

/* Maximal horizontal and vertical resolution to support by the library.*/
#define LV_HOR_RES_MAX          (480)//屏幕宽
#define LV_VER_RES_MAX          (800)//屏幕高

/* Color depth:
 * - 1:  1 byte per pixel//如果是黑白屏就设置这个
 * - 8:  RGB233
 * - 16: RGB565
 * - 32: ARGB8888
 */
#define LV_COLOR_DEPTH     16//这里选择 RGB565模式

/* Swap the 2 bytes of RGB565 color.

......
```
6. 调用lv_init();函数
7. 设置显示缓冲区
```
//显示缓冲区,如下代码
	static lv_disp_buf_t disp_buf;
	static lv_color_t buf[LV_HOR_RES_MAX * 10];                     /*Declare a buffer for 10 lines*/
	lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);    /*Initialize the display buffer*/
```
8. 注册屏幕写点函数
```
//像素打点,函数注册
	lv_disp_drv_t disp_drv;               /*Descriptor of a display driver*/
	lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
	disp_drv.flush_cb = my_disp_flush;    /*Set your driver function*/
	disp_drv.buffer = &disp_buf;          /*Assign the buffer to the display*/
	lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/	
```
```
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
```
9.如果需要触摸输入就进行输入函数注册和实现，不需要就算了
```
//触摸输入注册
	lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv);             /*Descriptor of a input device driver*/
	indev_drv.type = LV_INDEV_TYPE_POINTER;    /*Touch pad is a pointer-like device*/
	indev_drv.read_cb = my_touchpad_read;      /*Set your driver function*/
	lv_indev_drv_register(&indev_drv);         /*Finally register the driver*/
```
 ```
//读点函数实现
bool my_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data)
{
    static lv_coord_t last_x = 0;
    static lv_coord_t last_y = 0;

    /*Save the state and save the pressed coordinate*/
	data->state = tp_dev.sta&TP_PRES_DOWN ?  LV_INDEV_STATE_PR : LV_INDEV_STATE_REL; 
	//    if(data->state == LV_INDEV_STATE_PR) touchpad_get_xy(&last_x, &last_y);
   if(LV_INDEV_STATE_PR)			//触摸屏被按下
		{	
		 	if(tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{	
				//TP_Draw_Big_Point(tp_dev.x[0],tp_dev.y[0],RED);		//画图
				last_x=tp_dev.x[0];//将触摸的点的位置高速littlevGL
				last_y=tp_dev.y[0];
			}
		}
    /*Set the coordinates (if released use the last pressed coordinates)*/
    data->point.x = last_x;
    data->point.y = last_y;

    return false; /*Return `false` because we are not buffering and no more data to read*/
}
 ```
10. 在while(1)里调用lv_task_handler();
```
while(1)
	{
	 	lv_task_handler();
		tp_dev.scan(0); //这是我自己的屏幕触摸扫描，	
	}
```
11. 配置一个1~10ms的定时器调用lv_tick_inc(x);x是定时器周期
```
//我这里是一个5ms的定时器
//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		lv_tick_inc(5);
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}
```
#通过上述步骤就移植完成了，
>编译可能会有很多警告，如果警告是在littlevGL的文件里可以不用管，因为官方总不可能出错的，应该是编译器的问题，

##例子测试 ---你可以在while(1) 之前 尝试如下添加如下代码，会创建1个按钮 ，按钮可以有回调函数,回调函数要写在main函数外面，这里的回调函数是弹出一个消息框，消息框按钮点击会自动关闭，不用设置
```
	
//建立一按钮
	lv_obj_t * label;

    lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);
    lv_obj_set_event_cb(btn1, event_handler);//按钮按下回调函数，可以不用设置，可以多按钮同一个回调函数
    lv_obj_align(btn1, NULL, LV_ALIGN_CENTER, 0, -40);

    label = lv_label_create(btn1, NULL);
    lv_label_set_text(label, "Button");

```
```
//按钮回调函数
static void event_handler(lv_obj_t * obj, lv_event_t event)
{
	
    if(event == LV_EVENT_CLICKED) {//如果按钮按下
		
		//建立一个消息框
		 lv_obj_t * mbox = lv_mbox_create(lv_scr_act(), NULL);//创建消息框
        lv_obj_set_width(mbox, 200);//设置宽度
        static const char * mbox_btns[] = {"ok","cancel", ""};//设置按钮名字
        lv_mbox_add_btns(mbox, mbox_btns);    /*The default action is close*///给消息框添加一个按钮,默然按钮事件会关闭消息框
        lv_obj_align(mbox, lv_scr_act(), LV_ALIGN_IN_TOP_MID, 0, LV_DPI / 2);//按钮居中
    }

}
```
#效果如如下


![1578392140568.gif](C:\Users\leaf\Desktop\littlevGL STM32F4\6769622-be9f3e1ce0730deb.gif)