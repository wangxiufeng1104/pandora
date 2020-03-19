/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-08-15     misonyo      first implementation.
 */
/*
 * 程序清单：这是一个 I2C 设备使用例程
 * 例程导出了 i2c_aht10_sample 命令到控制终端
 * 命令调用格式：i2c_aht10_sample i2c1
 * 命令解释：命令第二个参数是要使用的I2C总线设备名称，为空则使用默认的I2C总线设备
 * 程序功能：通过 I2C 设备读取温湿度传感器 aht10 的温湿度数据并打印
 */
#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include "drv_lcd.h"
#include <stdio.h>
#include <string.h>
#include "iot_logo.h"
#include <board.h>

#define AHT10_I2C_BUS_NAME          "i2c1"  /* 传感器连接的I2C总线设备名称 */
#define AHT10_ADDR                  0x38    /* 从机地址 */
#define AHT10_CALIBRATION_CMD       0xE1    /* 校准命令 */
#define AHT10_NORMAL_CMD            0xA8    /* 一般命令 */
#define AHT10_GET_DATA              0xAC    /* 获取数据命令 */
static struct rt_i2c_bus_device *i2c_bus = RT_NULL; /* I2C总线设备句柄 */

static struct rt_thread aht10_thread;
static rt_uint32_t aht10_thread_stack[1024];


/* 写传感器寄存器 */
static rt_err_t write_reg(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t *data)
{
    rt_uint8_t buf[3];
    struct rt_i2c_msg msgs;
    buf[0] = reg; //cmd
    buf[1] = data[0];
    buf[2] = data[1];
    msgs.addr = AHT10_ADDR;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = 3;
    /* 调用I2C设备接口传输数据 */
    if (rt_i2c_transfer(bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}
/* 读传感器寄存器数据 */
static rt_err_t read_regs(struct rt_i2c_bus_device *bus, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msgs;
    msgs.addr = AHT10_ADDR;
    msgs.flags = RT_I2C_RD;
    msgs.buf = buf;
    msgs.len = len;
    /* 调用I2C设备接口传输数据 */
    if (rt_i2c_transfer(bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}
static void read_temp_humi(float *cur_temp, float *cur_humi)
{
    rt_uint8_t temp[6];
    write_reg(i2c_bus, AHT10_GET_DATA, 0); /* 发送命令 */
    read_regs(i2c_bus, 6, temp); /* 获取传感器数据 */
    /* 湿度数据转换 */
    *cur_humi = (temp[1] << 12 | temp[2] << 4 | (temp[3] & 0xf0) >> 4) * 100.0 / (1 << 20);
    /* 温度数据转换 */
    *cur_temp = ((temp[3] & 0xf) << 16 | temp[4] << 8 | temp[5]) * 200.0 / (1 << 20) - 50;
}
static void aht10_init(const char *name)
{
    rt_uint8_t temp[2] = { 0, 0 };
    /* 查找I2C总线设备，获取I2C总线设备句柄 */
    i2c_bus = (struct rt_i2c_bus_device *) rt_device_find(name);
    if (i2c_bus == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", name);
    }
    else
    {
        write_reg(i2c_bus, AHT10_NORMAL_CMD, temp);
        rt_thread_mdelay(400);
        temp[0] = 0x08;
        temp[1] = 0x00;
        write_reg(i2c_bus, AHT10_CALIBRATION_CMD, temp);
        rt_thread_mdelay(400);
    }
}
void aht10_thread_entry(void *parameter)
{
    aht10_init(AHT10_I2C_BUS_NAME);

    char humidity_string[50];
    char temperature_string[50];
    float humidity = 0.0, temperature = 0.0;
    lcd_set_color(WHITE, BLACK);
    lcd_show_image(70, 0, 100, 106, gImage_iot_logo);
    while (1)
    {
        read_temp_humi(&temperature, &humidity);
        rt_kprintf("read aht10 sensor humidity   : %d.%d %%\n", (int) humidity, (int) (humidity * 10) % 10);
        rt_kprintf("read aht10 sensor temperature: %d.%d \n", (int) temperature, (int) (temperature * 10) % 10);
        rt_memset(humidity_string, 0, sizeof(humidity_string));
        rt_memset(temperature_string, 0, sizeof(temperature_string));
        sprintf(temperature_string,"temperature: %d.%d", (int) temperature, (int) (temperature * 10) % 10);
        sprintf(humidity_string,"humidity:%d.%d %%", (int) humidity, (int) (humidity * 10) % 10);
        lcd_show_string(10,110, 24, humidity_string);
        lcd_show_string(10,140, 24, temperature_string);
        rt_thread_delay(1000);
    }
}
int ath10_thread_init(void)
{

    rt_thread_init(&aht10_thread,
                    "ath10",
                    aht10_thread_entry,
                    RT_NULL,
                    aht10_thread_stack,
                    sizeof(aht10_thread_stack),
                    9, 5);
    rt_thread_startup(&aht10_thread);
    return 0;
}
INIT_APP_EXPORT(ath10_thread_init);

