/*
 * File      : drv_gpio.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author            Notes
 * 2017-10-20     ZYH            the first version
 * 2017-11-15     ZYH            update to 3.0.0
 */

#include <rthw.h>
#include <rtdevice.h>
#include <board.h>
#include <rtthread.h>
#include "drivers/pin.h"
#include "ch32x035_conf.h"


#ifdef RT_USING_PIN
#define __CH32_PIN(index, gpio, gpio_index) {index, GPIO##gpio##_CLK_ENABLE, GPIO##gpio, GPIO_Pin_##gpio_index}
#define __CH32_PIN_DEFAULT  {-1, 0, 0, 0}


static void GPIOA_CLK_ENABLE(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}

static void GPIOB_CLK_ENABLE(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}


static void GPIOC_CLK_ENABLE(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}


/* GPIO driver */
struct pin_index
{
    int index;
    void (*rcc)(void);
    GPIO_TypeDef *gpio;
    uint32_t pin;
};

static const struct pin_index pins[] =
{
#if (ch32x035_PIN_NUMBERS == 28)
    __CH32_PIN_DEFAULT,
    __CH32_PIN(1, C, 15),
    __CH32_PIN_DEFAULT,
    __CH32_PIN(3, C, 0),
    __CH32_PIN(4, C, 3),
    __CH32_PIN(5, A, 0),
    __CH32_PIN(6, A, 1),
    __CH32_PIN(7, A, 2),
    __CH32_PIN(8, A, 3),
    __CH32_PIN(9, A, 4),
    __CH32_PIN(10, A, 5),
    __CH32_PIN(11, A, 6),
    __CH32_PIN(12, A, 7),
    __CH32_PIN(13, B, 0),
    __CH32_PIN(14, B, 3),
    __CH32_PIN(15, B, 4),
#if PB1_5==PB1
    __CH32_PIN(16, B, 1),
#elif PB1_5==PB5
    __CH32_PIN(16, B, 5),
#endif
    __CH32_PIN(17, B, 6),
    __CH32_PIN(18, B, 7),
    __CH32_PIN(19, B, 8),
    __CH32_PIN(20, B, 9),
    __CH32_PIN(21, B, 10),
    __CH32_PIN(22, B, 11),
    __CH32_PIN(23, B, 12),
    __CH32_PIN(24, C, 19),
    __CH32_PIN(25, C, 18),
    __CH32_PIN(26, C, 16),
    __CH32_PIN(27, C, 17),
    __CH32_PIN(28, C, 14),

#endif
#if (ch32x035_PIN_NUMBERS == 48)
    __CH32_PIN_DEFAULT,
    __CH32_PIN(1, A, 15),
    __CH32_PIN(2, A, 16),
    __CH32_PIN(3, A, 17),
    __CH32_PIN(4, A, 18),
    __CH32_PIN(5, A, 19),
    __CH32_PIN(6, A, 20),
    __CH32_PIN(7, A, 21),
    __CH32_PIN(8, A, 22),
    __CH32_PIN(9, A, 23),
    __CH32_PIN(10, A, 0),
    __CH32_PIN(11, A, 1),
    __CH32_PIN(12, A, 2),
    __CH32_PIN(13, A, 3),
    __CH32_PIN(14, A, 4),
    __CH32_PIN(15, A, 5),
    __CH32_PIN(16, A, 6),
    __CH32_PIN(17, A, 7),
    __CH32_PIN(18, C, 6),
    __CH32_PIN(19, C, 7),
    __CH32_PIN(20, B, 0),
    __CH32_PIN(21, B, 1),
    __CH32_PIN(22, B, 2),
    __CH32_PIN(23, B, 3),
    __CH32_PIN(24, B, 4),
    __CH32_PIN(25, B, 5),
    __CH32_PIN(26, B, 6),
    __CH32_PIN(27, B, 7),
    __CH32_PIN(28, B, 8),
    __CH32_PIN(29, B, 9),
    __CH32_PIN(30, B, 10),
    __CH32_PIN(31, B, 11),
    __CH32_PIN(32, C, 16),
    __CH32_PIN(33, C, 17),
    __CH32_PIN(34, C, 18),
    __CH32_PIN(35, B, 12),
    __CH32_PIN(36, B, 13),
    __CH32_PIN(37, C, 19),
    __CH32_PIN(38, C, 14),
    __CH32_PIN(39, C, 15),
    __CH32_PIN(40, A, 8),
    __CH32_PIN(41, A, 9),
    __CH32_PIN(42, A, 10),
    __CH32_PIN(43, A, 11),
    __CH32_PIN(44, A, 12),
    __CH32_PIN(45, A, 13),
    __CH32_PIN(46, A, 14),
    __CH32_PIN_DEFAULT,
    __CH32_PIN_DEFAULT,

#endif

#if (ch32x035_PIN_NUMBERS == 64)
    __CH32_PIN_DEFAULT,
    __CH32_PIN(1, A, 15),
    __CH32_PIN(2, A, 16),
    __CH32_PIN(3, A, 17),
    __CH32_PIN(4, A, 18),
    __CH32_PIN(5, A, 19),
    __CH32_PIN(6, A, 20),
    __CH32_PIN(7, A, 21),
    __CH32_PIN(8, C, 0),
    __CH32_PIN(9, C, 1),
    __CH32_PIN(10, C, 2),
    __CH32_PIN(11, C, 3),
    __CH32_PIN(12, A, 22),
    __CH32_PIN(13, A, 23),
    __CH32_PIN(14, A, 0),
    __CH32_PIN(15, A, 1),
    __CH32_PIN(16, A, 2),
    __CH32_PIN(17, A, 3),
    __CH32_PIN(18, C, 4),
    __CH32_PIN(19, C, 5),
    __CH32_PIN(20, A, 4),
    __CH32_PIN(21, A, 5),
    __CH32_PIN(22, A, 6),
    __CH32_PIN(23, A, 7),
    __CH32_PIN(24, C, 6),
    __CH32_PIN(25, C, 7),
    __CH32_PIN(26, B, 0),
    __CH32_PIN(27, B, 1),
    __CH32_PIN(28, B, 2),
    __CH32_PIN(29, B, 3),
    __CH32_PIN(30, B, 4),
    __CH32_PIN_DEFAULT,
    __CH32_PIN_DEFAULT,
    __CH32_PIN(33, B, 5),
    __CH32_PIN(34, B, 6),
    __CH32_PIN(35, B, 7),
    __CH32_PIN(36, B, 8),
    __CH32_PIN(37, B, 16),
    __CH32_PIN(38, B, 17),
    __CH32_PIN(39, B, 18),
    __CH32_PIN(40, B, 19),
    __CH32_PIN(41, B, 9),
    __CH32_PIN(42, B, 10),
    __CH32_PIN(43, B, 11),
    __CH32_PIN(44, C, 16),
    __CH32_PIN(45, C, 17),
    __CH32_PIN(46, C, 18),
    __CH32_PIN(47, B, 12),
    __CH32_PIN(48, B, 13),
    __CH32_PIN(49, C, 19),
    __CH32_PIN(50, B, 14),
    __CH32_PIN(51, B, 20),
    __CH32_PIN(52, B, 21),
    __CH32_PIN(53, B, 15),
    __CH32_PIN(54, C, 14),
    __CH32_PIN(55, C, 15),
    __CH32_PIN(56, A, 8),
    __CH32_PIN(57, A, 9),
    __CH32_PIN(58, A, 10),
    __CH32_PIN(59, A, 11),
    __CH32_PIN(60, A, 12),
    __CH32_PIN(61, A, 13),
    __CH32_PIN(62, A, 14),
    __CH32_PIN_DEFAULT,
    __CH32_PIN_DEFAULT,

#endif

};

struct pin_irq_map
{
    rt_uint32_t pinbit;
    IRQn_Type irqno;
};
static const struct pin_irq_map pin_irq_map[] =
{
    {GPIO_Pin_0, EXTI7_0_IRQn},
    {GPIO_Pin_1, EXTI7_0_IRQn},
    {GPIO_Pin_2, EXTI7_0_IRQn},
    {GPIO_Pin_3, EXTI7_0_IRQn},
    {GPIO_Pin_4, EXTI7_0_IRQn},
    {GPIO_Pin_5, EXTI7_0_IRQn},
    {GPIO_Pin_6, EXTI7_0_IRQn},
    {GPIO_Pin_7, EXTI7_0_IRQn},
    {GPIO_Pin_8, EXTI15_8_IRQn},
    {GPIO_Pin_9, EXTI15_8_IRQn},
    {GPIO_Pin_10, EXTI15_8_IRQn},
    {GPIO_Pin_11, EXTI15_8_IRQn},
    {GPIO_Pin_12, EXTI15_8_IRQn},
    {GPIO_Pin_13, EXTI15_8_IRQn},
    {GPIO_Pin_14, EXTI15_8_IRQn},
    {GPIO_Pin_15, EXTI15_8_IRQn},
    {GPIO_Pin_16, EXTI25_16_IRQn},
    {GPIO_Pin_17, EXTI25_16_IRQn},
    {GPIO_Pin_18, EXTI25_16_IRQn},
    {GPIO_Pin_19, EXTI25_16_IRQn},
    {GPIO_Pin_20, EXTI25_16_IRQn},
    {GPIO_Pin_21, EXTI25_16_IRQn},
    {GPIO_Pin_22, EXTI25_16_IRQn},
    {GPIO_Pin_23, EXTI25_16_IRQn},



};
struct rt_pin_irq_hdr pin_irq_hdr_tab[] =
{
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},
    { -1, 0, RT_NULL, RT_NULL},

};

#define ITEM_NUM(items) sizeof(items) / sizeof(items[0])
const struct pin_index *get_pin(uint8_t pin)
{
    const struct pin_index *index;
    if (pin < ITEM_NUM(pins))
    {
        index = &pins[pin];
        if (index->index == -1)
            index = RT_NULL;
    }
    else
    {
        index = RT_NULL;
    }
    return index;
};

void ch32_pin_write(rt_device_t dev, rt_base_t pin, rt_base_t value)
{
    const struct pin_index *index;
    index = get_pin(pin);
    if (index == RT_NULL)
    {
        return;
    }
    GPIO_WriteBit(index->gpio, index->pin, (BitAction)value);
}

int ch32_pin_read(rt_device_t dev, rt_base_t pin)
{
    int value;
    const struct pin_index *index;
    value = PIN_LOW;
    index = get_pin(pin);
    if (index == RT_NULL)
    {
        return value;
    }
    value = GPIO_ReadInputDataBit(index->gpio, index->pin);
    return value;
}

void ch32_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    const struct pin_index *index;
    GPIO_InitTypeDef GPIO_InitStruct;
    index = get_pin(pin);
    if (index == RT_NULL)
    {
        return;
    }
    rt_kprintf("get pin index\r\n");
    /* GPIO Periph clock enable */
    index->rcc();
    /* Configure GPIO_InitStructure */
    GPIO_InitStruct.GPIO_Pin = index->pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    if (mode == PIN_MODE_OUTPUT)
    {
        /* output setting */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    }
    else if (mode == PIN_MODE_INPUT_AIN)
    {
        /* input setting: not pull. */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    }
    else if (mode == PIN_MODE_INPUT)
    {
        /* input setting: pull up. */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    else if (mode == PIN_MODE_INPUT_PULLDOWN)
    {
        /* input setting: pull down. */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
//        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    }
    else if (mode == PIN_MODE_INPUT_PULLUP)
    {
        /* output setting: od. */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
    }
    else if (mode == PIN_MODE_OUTPUT_OD)
    {
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    }
    else if (mode == PIN_MODE_OUTPUT_AF_OD)
    {
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    }
    else if (mode == PIN_MODE_OUTPUT_AF_PP)
    {
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    }

    GPIO_Init(index->gpio, &GPIO_InitStruct);
}


rt_inline rt_int32_t bit2bitno(rt_uint32_t bit)
{
    int i;
    for (i = 0; i < 32; i++)
    {
        if ((0x01 << i) == bit)
        {
            return i;
        }
    }
    return -1;
}

rt_inline const struct pin_irq_map *get_pin_irq_map(uint32_t pinbit)
{
    rt_int32_t mapindex = bit2bitno(pinbit);
    if (mapindex < 0 || mapindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_NULL;
    }
    return &pin_irq_map[mapindex];
};
rt_err_t ch32_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
                              rt_uint32_t mode, void (*hdr)(void *args), void *args)
{
    const struct pin_index *index;
    rt_base_t level;
    rt_int32_t irqindex = -1;
    index = get_pin(pin);
    if (index == RT_NULL)
    {
        return RT_ENOSYS;
    }
    irqindex = bit2bitno(index->pin);
    if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_ENOSYS;
    }
    level = rt_hw_interrupt_disable();
    if (pin_irq_hdr_tab[irqindex].pin == pin &&
            pin_irq_hdr_tab[irqindex].hdr == hdr &&
            pin_irq_hdr_tab[irqindex].mode == mode &&
            pin_irq_hdr_tab[irqindex].args == args)
    {
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }
    if (pin_irq_hdr_tab[irqindex].pin != -1)
    {
        rt_hw_interrupt_enable(level);
        return RT_EBUSY;
    }
    pin_irq_hdr_tab[irqindex].pin = pin;
    pin_irq_hdr_tab[irqindex].hdr = hdr;
    pin_irq_hdr_tab[irqindex].mode = mode;
    pin_irq_hdr_tab[irqindex].args = args;
    rt_hw_interrupt_enable(level);
    return RT_EOK;
}

rt_err_t ch32_pin_dettach_irq(struct rt_device *device, rt_int32_t pin)
{
    const struct pin_index *index;
    rt_base_t level;
    rt_int32_t irqindex = -1;
    index = get_pin(pin);
    if (index == RT_NULL)
    {
        return RT_ENOSYS;
    }
    irqindex = bit2bitno(index->pin);
    if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_ENOSYS;
    }
    level = rt_hw_interrupt_disable();
    if (pin_irq_hdr_tab[irqindex].pin == -1)
    {
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }
    pin_irq_hdr_tab[irqindex].pin = -1;
    pin_irq_hdr_tab[irqindex].hdr = RT_NULL;
    pin_irq_hdr_tab[irqindex].mode = 0;
    pin_irq_hdr_tab[irqindex].args = RT_NULL;
    rt_hw_interrupt_enable(level);
    return RT_EOK;
}

rt_err_t ch32_pin_irq_enable(struct rt_device *device, rt_base_t pin,
                              rt_uint32_t enabled)
{
    const struct pin_index *index;
    const struct pin_irq_map *irqmap;
    rt_base_t level;
    rt_int32_t irqindex = -1;
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStructure;

    index = get_pin(pin);
    if (index == RT_NULL)
    {
        return RT_ENOSYS;
    }
    if (enabled == PIN_IRQ_ENABLE)
    {
        irqindex = bit2bitno(index->pin);
        if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
        {
            return RT_ENOSYS;
        }
        level = rt_hw_interrupt_disable();
        if (pin_irq_hdr_tab[irqindex].pin == -1)
        {
            rt_hw_interrupt_enable(level);
            return RT_ENOSYS;
        }
        irqmap = &pin_irq_map[irqindex];
        /* GPIO Periph clock enable */
        index->rcc();
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

        /* Configure GPIO_InitStructure */
        GPIO_InitStruct.GPIO_Pin = index->pin;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

        EXTI_InitStructure.EXTI_Line=index->pin;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        switch (pin_irq_hdr_tab[irqindex].mode)
        {
        case PIN_IRQ_MODE_RISING:
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
            EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
            break;
        case PIN_IRQ_MODE_FALLING:
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
            EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
            break;
        case PIN_IRQ_MODE_RISING_FALLING:
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
            EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
            break;
        }

        GPIO_Init(index->gpio, &GPIO_InitStruct);
        EXTI_Init(&EXTI_InitStructure);
        NVIC_SetPriority(irqmap->irqno,5<<4);
        NVIC_EnableIRQ( irqmap->irqno );

        rt_hw_interrupt_enable(level);
    }
    else if (enabled == PIN_IRQ_DISABLE)
    {
        irqmap = get_pin_irq_map(index->pin);
        if (irqmap == RT_NULL)
        {
            return RT_ENOSYS;
        }
        NVIC_DisableIRQ(irqmap->irqno);
    }
    else
    {
        return RT_ENOSYS;
    }
    return RT_EOK;
}

const static struct rt_pin_ops _ch32_pin_ops =
{
    ch32_pin_mode,
    ch32_pin_write,
    ch32_pin_read,
    ch32_pin_attach_irq,
    ch32_pin_dettach_irq,
    ch32_pin_irq_enable,
};

int rt_hw_pin_init(void)
{
    int result;
    result = rt_device_pin_register("pin", &_ch32_pin_ops, RT_NULL);
    return result;
}
INIT_BOARD_EXPORT(rt_hw_pin_init);

rt_inline void pin_irq_hdr(int irqno)
{
    if (pin_irq_hdr_tab[irqno].hdr)
    {
        pin_irq_hdr_tab[irqno].hdr(pin_irq_hdr_tab[irqno].args);
    }
}

void HAL_GPIO_EXTI_Callback(uint32_t GPIO_Pin)
{
    pin_irq_hdr(bit2bitno(GPIO_Pin));
}


void EXTI0_IRQHandler(void) __attribute__((interrupt()));
void EXTI1_IRQHandler(void) __attribute__((interrupt()));
void EXTI2_IRQHandler(void) __attribute__((interrupt()));
void EXTI3_IRQHandler(void) __attribute__((interrupt()));
void EXTI4_IRQHandler(void) __attribute__((interrupt()));
void EXTI9_5_IRQHandler(void) __attribute__((interrupt()));
void EXTI25_16_IRQHandler(void) __attribute__((interrupt()));

void EXTI0_IRQHandler(void)
{
    GET_INT_SP();
    rt_interrupt_enter();

    if(EXTI_GetITStatus(EXTI_Line0)!=RESET)
    {
        HAL_GPIO_EXTI_Callback(GPIO_Pin_0);
        EXTI_ClearITPendingBit(EXTI_Line0);
    }

    rt_interrupt_leave();
    FREE_INT_SP();
}

void EXTI1_IRQHandler(void)
{
    GET_INT_SP();
    rt_interrupt_enter();
    if(EXTI_GetITStatus(EXTI_Line1)!=RESET)
    {
        HAL_GPIO_EXTI_Callback(GPIO_Pin_1);
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
    rt_interrupt_leave();
    FREE_INT_SP();
}

void EXTI2_IRQHandler(void)
{
    GET_INT_SP();
    rt_interrupt_enter();
    if(EXTI_GetITStatus(EXTI_Line2)!=RESET)
    {
        HAL_GPIO_EXTI_Callback(GPIO_Pin_2);
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
    rt_interrupt_leave();
    FREE_INT_SP();
}

void EXTI3_IRQHandler(void)
{
    GET_INT_SP();
    rt_interrupt_enter();
    if(EXTI_GetITStatus(EXTI_Line3)!=RESET)
    {
        HAL_GPIO_EXTI_Callback(GPIO_Pin_3);
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
    rt_interrupt_leave();
    FREE_INT_SP();
}

void EXTI4_IRQHandler(void)
{
    GET_INT_SP();
    rt_interrupt_enter();
    if(EXTI_GetITStatus(EXTI_Line4)!=RESET)
    {
        HAL_GPIO_EXTI_Callback(GPIO_Pin_4);
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
    rt_interrupt_leave();
    FREE_INT_SP();
}

void EXTI9_5_IRQHandler(void)
{
    GET_INT_SP();
    rt_interrupt_enter();
    if( (EXTI_GetITStatus(EXTI_Line5)!=RESET)|| \
        (EXTI_GetITStatus(EXTI_Line6)!=RESET)|| \
        (EXTI_GetITStatus(EXTI_Line7)!=RESET)|| \
        (EXTI_GetITStatus(EXTI_Line8)!=RESET)|| \
        (EXTI_GetITStatus(EXTI_Line9)!=RESET) )
    {
    HAL_GPIO_EXTI_Callback(GPIO_Pin_5);
    HAL_GPIO_EXTI_Callback(GPIO_Pin_6);
    HAL_GPIO_EXTI_Callback(GPIO_Pin_7);
    HAL_GPIO_EXTI_Callback(GPIO_Pin_8);
    HAL_GPIO_EXTI_Callback(GPIO_Pin_9);
    EXTI_ClearITPendingBit(EXTI_Line5|EXTI_Line6|EXTI_Line7|EXTI_Line8|EXTI_Line9);
    }
    rt_interrupt_leave();
    FREE_INT_SP();
}

void EXTI25_16_IRQHandler(void)
{
    GET_INT_SP();
    rt_interrupt_enter();
    if( (EXTI_GetITStatus(EXTI_Line16)!=RESET)|| \
        (EXTI_GetITStatus(EXTI_Line17)!=RESET)|| \
        (EXTI_GetITStatus(EXTI_Line18)!=RESET)|| \
        (EXTI_GetITStatus(EXTI_Line19)!=RESET)|| \
        (EXTI_GetITStatus(EXTI_Line20)!=RESET)|| \
        (EXTI_GetITStatus(EXTI_Line21)!=RESET)|| \
        (EXTI_GetITStatus(EXTI_Line22)!=RESET)|| \
        (EXTI_GetITStatus(EXTI_Line23)!=RESET) )
    {
    HAL_GPIO_EXTI_Callback(GPIO_Pin_16);
    HAL_GPIO_EXTI_Callback(GPIO_Pin_17);
    HAL_GPIO_EXTI_Callback(GPIO_Pin_18);
    HAL_GPIO_EXTI_Callback(GPIO_Pin_19);
    HAL_GPIO_EXTI_Callback(GPIO_Pin_20);
    HAL_GPIO_EXTI_Callback(GPIO_Pin_21);
    HAL_GPIO_EXTI_Callback(GPIO_Pin_22);
    HAL_GPIO_EXTI_Callback(GPIO_Pin_23);
    EXTI_ClearITPendingBit(EXTI_Line16|EXTI_Line17|EXTI_Line18|EXTI_Line19|EXTI_Line20|EXTI_Line21|EXTI_Line22|EXTI_Line23);
    }
    rt_interrupt_leave();
    FREE_INT_SP();
}
#endif
