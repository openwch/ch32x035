/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/03/02
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32x035.h"
#include <rtthread.h>
#include <rthw.h>
#include "drivers/pin.h"
#include "ch32x035_conf.h"

/* The printf pin is PA2 (UART2) */


/* Global typedef */

/* Global define */

/* LED0 is driven by the pin driver interface of rt  */
#define LED0_PIN  5   //PA0

/* Global Variable */
/*CH32X035F8 PB1 and PB5 are the same physical pin
 *  PB1 and PB5 cannot be used at the same time. */
#define  PB1  0
#define  PB5  1
#define  PB1_5  PB1

/*********************************************************************
 * @fn      LED1_BLINK_INIT
 *
 * @brief   LED1 directly calls the underlying driver
 *
 * @return  none
 */
void LED1_BLINK_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/* main is just one of the threads, in addition to tshell,idle
   *    main is just an LED blinking, the main thread is registered in rtthread_startup,
   *   tshell uses the serial port to receive interrupts, and the interrupt stack and thread stack are
   *   used separately.Note that when entering an interrupt, the 16caller register needs to be pushed 
   *   into the thread stack
 * */

int main(void)
{

    rt_kprintf("\r\n MCU: ch32x035\r\n");
	rt_kprintf(" SysClk: %dHz\r\n",SystemCoreClock);
    rt_kprintf(" www.wch.cn\r\n");
	LED1_BLINK_INIT();
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	while(1)
	{
	    GPIO_SetBits(GPIOA,GPIO_Pin_0);
	    rt_thread_mdelay(500);
	    GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	    rt_thread_mdelay(500);
	}
}

/* Test using the driver interface to operate the I/O port  */
int led(void)
{
    rt_uint8_t count;

    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    printf("led_SP:%08x\r\n",__get_SP());
    for(count = 0 ; count < 10 ;count++)
    {
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_kprintf("led on, count : %d\r\n", count);
        rt_thread_mdelay(500);

        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_kprintf("led off\r\n");
        rt_thread_mdelay(500);
    }
    return 0;
}
MSH_CMD_EXPORT(led,  led sample by using I/O drivers);



