/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/26
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *Analog watchdog routine include Non scanning mode , scanning mode and Analog Watchdog reset mode:
 *NoSCAN_MODE_WDT-ADC channel 1 (PA1), detect that the ADC conversion data on the rule group channel
 *is outside 2000 - 3500 and trigger the simulation Watchdog interrupt or reset.
 *SCAN_MODE_WDT-ADC channel 1 (PA1), detect that the ADC conversion data on the rule group channel
 *is outside 2000 - 3500 and trigger the simulation Watchdog interrupt or reset.
 */

#include "debug.h"

/* ADC WWDG Mode Definition*/
#define NoSCAN_MODE_WDT   0
#define SCAN_MODE_WDT     1

/* ADC WWDG Mode Selection*/
//#define ADC_MODE_WDT   NoSCAN_MODE_WDT
#define ADC_MODE_WDT   SCAN_MODE_WDT

/* WWDG Reset Enable Definition */
#define WDT_RST_ENABLE   0
#define WDT_RST_DISABLE  1

/* WWDG Reset Enable Selection */
#define WDT_RST   WDT_RST_DISABLE
//#define WDT_RST   WDT_RST_ENABLE


/*********************************************************************
 * @fn      ADC_Function_Init
 *
 * @brief   Initializes ADC collection.
 *
 * @return  none
 */
void ADC_Function_Init(void)
{
    ADC_InitTypeDef  ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

#if  (ADC_MODE_WDT == NoSCAN_MODE_WDT)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
#else
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5;
#endif

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit(ADC1);

    ADC_CLKConfig(ADC1, ADC_CLK_Div6);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
#if  (ADC_MODE_WDT == NoSCAN_MODE_WDT)
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
#else
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_NbrOfChannel = 4;
#endif

    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_Init(ADC1, &ADC_InitStructure);

#if  (ADC_MODE_WDT == NoSCAN_MODE_WDT)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_11Cycles);
    ADC_AnalogWatchdogSingleChannelConfig(ADC1, ADC_Channel_1);
    ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);

#else
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_11Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_11Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_11Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_11Cycles);
    ADC_AnalogWatchdogScanCmd(ADC1, ENABLE);

    ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_AllRegEnable);
#endif

    /* Higher Threshold:3500, Lower Threshold:2000 */
    ADC_AnalogWatchdogThresholdsConfig(ADC1, 3500, 2000);

#if (WDT_RST == WDT_RST_ENABLE)
    ADC_AnalogWatchdogResetCmd(ADC1, ADC_AnalogWatchdog_0_RST_EN, ENABLE);
#else
    ADC_AnalogWatchdogResetCmd(ADC1, ADC_AnalogWatchdog_0_RST_EN, DISABLE);
#endif

#if  (ADC_MODE_WDT == SCAN_MODE_WDT)

    /* Higher Threshold:3500, Lower Threshold:2000 */
    ADC_AnalogWatchdog1ThresholdsConfig(ADC1, 3500, 2000);

#if (WDT_RST == WDT_RST_ENABLE)
    ADC_AnalogWatchdogResetCmd(ADC1, ADC_AnalogWatchdog_1_RST_EN, ENABLE);
#else
    ADC_AnalogWatchdogResetCmd(ADC1, ADC_AnalogWatchdog_1_RST_EN, DISABLE);
#endif

    /* Higher Threshold:3500, Lower Threshold:2000 */
    ADC_AnalogWatchdog2ThresholdsConfig(ADC1, 3500, 2000);

#if (WDT_RST == WDT_RST_ENABLE)
    ADC_AnalogWatchdogResetCmd(ADC1, ADC_AnalogWatchdog_2_RST_EN, ENABLE);
#else
    ADC_AnalogWatchdogResetCmd(ADC1, ADC_AnalogWatchdog_2_RST_EN, DISABLE);
#endif

    /* Higher Threshold:3500, Lower Threshold:2000 */
    ADC_AnalogWatchdog3ThresholdsConfig(ADC1, 3500, 2000);

#if (WDT_RST == WDT_RST_ENABLE)
    ADC_AnalogWatchdogResetCmd(ADC1, ADC_AnalogWatchdog_3_RST_EN, ENABLE);
#else
    ADC_AnalogWatchdogResetCmd(ADC1, ADC_AnalogWatchdog_3_RST_EN, DISABLE);
#endif

#endif

    NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    printf("CTLR3-%08x\r\n", ADC1->CTLR3);
    Delay_Ms(1000);
}

/*********************************************************************
 * @fn      Get_ADC_Val
 *
 * @brief   Returns ADCx conversion result data.
 *
 * @param   ch - ADC channel.
 *            ADC_Channel_0 - ADC Channel0 selected.
 *            ADC_Channel_1 - ADC Channel1 selected.
 *            ADC_Channel_2 - ADC Channel2 selected.
 *            ADC_Channel_3 - ADC Channel3 selected.
 *            ADC_Channel_4 - ADC Channel4 selected.
 *            ADC_Channel_5 - ADC Channel5 selected.
 *            ADC_Channel_6 - ADC Channel6 selected.
 *            ADC_Channel_7 - ADC Channel7 selected.
 *            ADC_Channel_8 - ADC Channel8 selected.
 *            ADC_Channel_9 - ADC Channel9 selected.
 *            ADC_Channel_10 - ADC Channel10 selected.
 *            ADC_Channel_11 - ADC Channel11 selected.
 *            ADC_Channel_12 - ADC Channel12 selected.
 *            ADC_Channel_13 - ADC Channel13 selected.
 *            ADC_Channel_14 - ADC Channel14 selected.
 *            ADC_Channel_15 - ADC Channel15 selected.
 *
 * @return  none
 */
u16 Get_ADC_Val(u8 ch)
{
    u16 val;

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

    val = ADC_GetConversionValue(ADC1);

    return val;
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    u16 ADC_val;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );

    Delay_Ms(1000);
    ADC_Function_Init();

    while(1)
    {
        ADC_val = Get_ADC_Val(ADC_Channel_5);
        Delay_Ms(500);
        printf("%04d\r\n", ADC_val);
        Delay_Ms(2);
    }
}

void ADC1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      ADC1_IRQHandler
 *
 * @brief   ADC1 Interrupt Service Function.
 *
 * @return  none
 */
void ADC1_IRQHandler()
{
    if(ADC_GetITStatus( ADC1, ADC_IT_AWD)){
        printf( "Enter AnalogWatchdog Interrupt\r\n" );
    }

    ADC_ClearITPendingBit( ADC1, ADC_IT_AWD);
}
