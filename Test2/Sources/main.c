/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : Main program body
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "misc.h"
#include "extern.h"

#define cifra_0 0b01110111 //0b01110111
#define cifra_1 0b01000001
#define cifra_2 0b00111011
#define cifra_3 0b01101011
#define cifra_4 0b01001101
#define cifra_5 0b01101110
#define cifra_6 0b01111110
#define cifra_7 0b01000011
#define cifra_8 0b01111111
#define cifra_9 0b01101111

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_ICInitTypeDef  TIM_ICInitStructure;
ErrorStatus HSEStartUpStatus;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
extern vu16 IC2Value ;
extern vu16 DutyCycle ;
extern vu32 Frequency ;
extern int TempInt;
extern float Temp;

extern volatile u8 READ_flag;

extern float val;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void WriteLCD(float);
void RCC_TIMConfig(void);
void EXTI_Config(void);
void LCDInit(void);
u16 PID_val();

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
#ifdef DEBUG
  debug();
#endif

  /* System Clocks Configuration */
  RCC_Configuration();

  /* NVIC configuration */
  NVIC_Configuration();

  /* Configure the GPIO ports */
  GPIO_Configuration();
  RCC_TIMConfig();
  EXTI_Config();
  InitSysTick();
  //LCDInit();
 
  float printMe;
  while (1)
  { 
    if(TimeoutExp2() == 0)
       printMe = val;
    else
        printMe = Temp;
    WriteLCD(printMe);
    if(READ_flag)
        TIM3->CCR1 = PID_val(); 
    wasteTime(300);
  }
}

u16 PID_val()
{
    u16 ret;
    ret = 80*(val - Temp)/100;
    ret = 80 - ret;
    return ret;
}


/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);

    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {}

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while (RCC_GetSYSCLKSource() != 0x08)
    {}
  }

  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOA clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA , ENABLE);
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configure the GPIOD Pins.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM2 channel 2 pin (PA.01) configuration */
//temperature sensor
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
//

//rotary encoder
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
//

//led RED
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
//

//LCD
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
//
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configure the nested vectored interrupt controller.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
  /* Set the Vector Table base location at 0x20000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void RCC_TIMConfig(void)
{       
    /* TIM2 configuration: PWM Input mode ------------------------
     The external signal is connected to TIM2 CH2 pin (PA.01), 
     The Rising edge is used as active edge,
     The TIM2 CCR2 is used to compute the frequency value 
     The TIM2 CCR1 is used to compute the duty cycle value
  ------------------------------------------------------------ */

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);

  /* Select the TIM2 Input Trigger: TI2FP2 */
  TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);

  /* Enable the Master/Slave Mode */
  TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);

  /* TIM enable counter */
  TIM_Cmd(TIM4, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);



      /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 79; //se face +1
  TIM_TimeBaseStructure.TIM_Prescaler = 35;//se face +1
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  // t = 0.5 micro ( 72MHz/36 = 2MHz )
  // frecv PWM vrem sa fie 25 khz ( 2MHz / 25 KHz ) =>
  // T = 80 pasi
  

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

 /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

void EXTI_Config(void)
{
      EXTI_InitTypeDef  EXTI_InitStructure;
      GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);

      /* Configure Key Button EXTI Line to generate an interrupt on falling edge */  
      EXTI_InitStructure.EXTI_Line = EXTI_Line8 ;
      EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
      EXTI_InitStructure.EXTI_LineCmd = ENABLE;
      EXTI_Init(&EXTI_InitStructure);

}

void LCDInit(void)
{
//     WriteLCD(".");
}

void WriteLCD(float t)
{
    
    volatile int i, a, b, c, d;
    u8 cifra_a,cifra_b,cifra_c,cifra_d;
    u32 numar=0;

    t*=100;
    t=(int)t;
    
    a=t/1000;
    b=t/100;
    b=(int)b%10;
    c=t/10;
    c%=10;
    d=(int)t%10;
    
    switch(a)
    {
        case 0: 
             cifra_a=cifra_0;
             break;
        case 1: 
             cifra_a=cifra_1;
             break;
        case 2:
             cifra_a=cifra_2;
             break;
        case 3:
             cifra_a=cifra_3;
             break;
        case 4:
             cifra_a=cifra_4;
             break;
        case 5:
             cifra_a=cifra_5;
             break;
        case 6:
             cifra_a=cifra_6;
             break;
        case 7:
             cifra_a=cifra_7;
             break;
        case 8:
             cifra_a=cifra_8;
             break;
        case 9:
             cifra_a=cifra_9;
             break;
        default:
             cifra_a=0b10000000;
             break;
    }
    
    switch(b)
    {
        case 0: 
             cifra_b=cifra_0;
             break;
        case 1: 
            cifra_b=cifra_1;
             break;
        case 2:
             cifra_b=cifra_2;
             break;
        case 3:
             cifra_b=cifra_3;
             break;
        case 4:
             cifra_b=cifra_4;
             break;
        case 5:
             cifra_b=cifra_5;
             break;
        case 6:
             cifra_b=cifra_6;
             break;
        case 7:
             cifra_b=cifra_7;
             break;
        case 8:
             cifra_b=cifra_8;
             break;
        case 9:
             cifra_b=cifra_9;
             break;
        default:
             cifra_b=0b10000000;
             break;
    }

    switch(c)
    {
        case 0: 
             cifra_c=cifra_0;
             break;
        case 1: 
             cifra_c=cifra_1;
             break;
        case 2:
             cifra_c=cifra_2;
             break;
        case 3:
             cifra_c=cifra_3;
             break;
        case 4:
             cifra_c=cifra_4;
             break;
        case 5:
             cifra_c=cifra_5;
             break;
        case 6:
             cifra_c=cifra_6;
             break;
        case 7:
             cifra_c=cifra_7;
             break;
        case 8:
             cifra_c=cifra_8;
             break;
        case 9:
             cifra_c=cifra_9;
             break;
        default:
             cifra_c=0b10000000;
             break;
    }

    switch(d)
    {
        case 0: 
             cifra_d=cifra_0;
             break;
        case 1: 
             cifra_d=cifra_1;
             break;
        case 2:
             cifra_d=cifra_2;
             break;
        case 3:
             cifra_d=cifra_3;
             break;
        case 4:
             cifra_d=cifra_4;
             break;
        case 5:
             cifra_d=cifra_5;
             break;
        case 6:
             cifra_d=cifra_6;
             break;
        case 7:
             cifra_d=cifra_7;
             break;
        case 8:
             cifra_d=cifra_8;
             break;
        case 9:
             cifra_d=cifra_9;
             break;
        default:
             cifra_d=0b10000000;
             break;
    }

    cifra_b|=0b10000000;

    numar=(numar|cifra_d)<<8;
    numar=(numar|cifra_c)<<8;
    numar=(numar|cifra_b)<<8;
    numar=(numar|cifra_a);

    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
    
    for(i=0;i<32;i++)
        {
        
        GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
        
        
        if(numar & 1)
            {
                GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_SET);
            }
        else 
            {
                GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET);
            }
        GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
        numar>>=1;
        }

    // for(i=0;i<8;i++)
        // {
        
        // GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
        
        
        // if(cifra_a & 0b00000001)
            // {
                // GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_SET);
            // }
        // else 
            // {
                // GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET);
            // }
        // GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
        // cifra_a>>=1;
        // }
    
    // for(i=0;i<8;i++)
        // {
        // GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
        
        
        // if(cifra_b & 0b00000001)
            // {
                // GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_SET);
            // }
        // else 
            // {
                // GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET);
            // }
        // GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
        // cifra_b>>=1;
        // }

    
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
}


#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
