/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_it.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : Main Interrupt Service Routines.
*                      This file provides template for all exceptions handler
*                      and peripherals interrupt service routine.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "sys_time.h"
#include "extern.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
 vu16 IC2Value = 0;
 volatile float DutyCycle = 0;
 vu32 Frequency = 0;
 volatile float Temp = 0;
 volatile float val =25;
 //volatile int TempInt = 0;
 volatile u8 READ_flag = 0;
 volatile int counter;
 volatile float aux_temp;
 
 vu8 state;
 //vu16 val = 0; 

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/*******************************************************************************
* Function Name  : EXTI9_5_IRQHandler
* Description    : This function handles External lines 9 to 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void EXTI9_5_IRQHandler(void){
   if(EXTI_GetITStatus(EXTI_Line8) != RESET)  {
  
    state = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9);
  
    if(state == 1)
    {
        val = val - 0.5;
    }
    else
    {
       val = val + 0.5;
    }
    TimeoutSet2(1000);
    EXTI_ClearITPendingBit(EXTI_Line8);
  }
}

void update(float t)
{
    aux_temp = ((counter * aux_temp) + t)/(counter+1);    
    counter = (++counter)%20;
    if(counter == 0)
    {
        Temp = aux_temp;
        READ_flag = 1;
    }
}

/*******************************************************************************
* Function Name  : TIM4_IRQHandler
* Description    : This function handles TIM4 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM4_IRQHandler(void)
{
 /* Clear TIM2 Capture compare interrupt pending bit */

  if(TIM_GetFlagStatus(TIM4, TIM_FLAG_CC2) == SET)
   {
    
  TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);

  /* Get the Input Capture value */
  IC2Value = TIM_GetCapture2(TIM4);

  if (IC2Value != 0)
  {
    /* Duty cycle computation */
    DutyCycle = 100 - (TIM_GetCapture1(TIM4) * 100.0)/ IC2Value;
    update((DutyCycle - 32) / 0.47);
    
    /* Frequency computation */
    Frequency = 72000000 / IC2Value;
  }
  else
  {
    DutyCycle = 0;
    Frequency = 0;
  }
    }
}

void TIM2_IRQHandler(void)
{

  if(TIM_GetFlagStatus(TIM2, TIM_IT_Update) == SET)
   {
    
   TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
   sys_time++;
   }
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/