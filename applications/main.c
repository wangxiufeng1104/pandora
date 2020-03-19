/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-09-09     RT-Thread    first version
 */

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include "drv_lcd.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* PLEASE DEFINE the LED0 pin for your board, such as: PA5 */
#define LED0_PIN    GET_PIN(E, 7)
rt_uint8_t abs0(rt_int32_t num)
{
    if(num>0) return num;

    num = -num;
    return (rt_uint8_t) num;
}
rt_uint32_t ColorToColor(rt_uint32_t color0, rt_uint32_t color1)//待研究
{
    rt_uint8_t Red0, Green0, Blue0;  // 起始三原色
    rt_uint8_t Red1, Green1, Blue1;  // 结果三原色
    rt_int32_t           RedMinus, GreenMinus, BlueMinus;  // 颜色差（color1 - color0）
    rt_uint8_t NStep;                            // 需要几步
    float         RedStep, GreenStep, BlueStep;     // 各色步进值
    unsigned long color;                            // 结果色
    unsigned char i;
    static rt_uint16_t WsDatTemp;
    // 绿 红 蓝 三原色分解
    Red0   = color0>>11;
    Green0 = (color0 & 0x7E0) >> 5;
    Blue0  = (color0 & 0x1F);

    Red1   = color1>>11;
    Green1 = (color1 & 0x7E0) >> 5;
    Blue1  = (color1 & 0x1F);

    // 计算需要多少步（取差值的最大值）
    RedMinus   = (Red1 - Red0);
    GreenMinus = (Green1 - Green0);
    BlueMinus  = (Blue1 - Blue0);

    NStep = ( abs0(RedMinus) > abs0(GreenMinus) ) ? abs0(RedMinus):abs0(GreenMinus);
    NStep = ( NStep > abs0(BlueMinus) ) ? NStep:abs0(BlueMinus);

    // 计算出各色步进值
    RedStep   = (float)RedMinus   / NStep;
    GreenStep = (float)GreenMinus / NStep;
    BlueStep  = (float)BlueMinus  / NStep;

    // 渐变开始
    for(i=0; i<NStep; i++)
    {
        Red1   = Red0   + (rt_int32_t)(RedStep   * i);
        Green1 = Green0 + (rt_int32_t)(GreenStep * i);
        Blue1  = Blue0  + (rt_int32_t)(BlueStep  * i);

        color  = Green1 << 5 | Red1 << 11 | Blue1;  // 合成  绿红蓝
        WsDatTemp = color;

            lcd_fill(20, 20, 220, 220, WsDatTemp);
      //WS_Reset();
        rt_thread_mdelay(10);                      // 渐变速度
    }
    // 渐变结束

    return color;
}



int main(void)
{
    /* set LED0 pin mode to output */
/*    rt_uint16_t colorArr[] = {BLACK,BLUE,GREEN,GBLUE,CYAN,GRAY,GRAY151,GRAY175,BROWN,GRAY187,GRAY240,RED,BRED,BRRED,YELLOW,WHITE};
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    for(int i = 1;i < (sizeof(colorArr)/sizeof(colorArr[0])); i++)
    {
        ColorToColor(colorArr[i - 1],colorArr[i]);
    }*/
/*    while (count++)
    {
        colorNum ++;
        if(colorNum <= BLUE)
        {
            lcd_fill(20, 20, 220, 220, BLUE);
        }
        else if (colorNum > BLUE && colorNum <= (GREEN + BLUE)) {
            lcd_fill(20, 20, 220, 220, GREEN);
        }
        else if(colorNum >= (GREEN + BLUE)){
            lcd_fill(20, 20, 220, 220, RED);
        }


        rt_pin_write(LED0_PIN, PIN_HIGH);
        lcd_fill(20, 20, 220, 220, RED);
        lcd_fill(40, 40, 200, 200, GREEN);
        lcd_fill(60, 60, 180, 180, BLUE);
        lcd_fill(80, 80, 160, 160, BLACK);
        lcd_fill(100, 100, 140, 140, WHITE);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        lcd_fill(0, 0, 240, 240, WHITE);
        rt_thread_mdelay(500);
    }*/

    return RT_EOK;
}
