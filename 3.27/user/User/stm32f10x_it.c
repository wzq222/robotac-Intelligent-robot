/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "Delay.h"
#include "stm32f10x_exti.h"
#include "encoder.h"

extern volatile uint8_t g_control_flag;

/* ── 编码器引脚宏 ─────────────────────────────────────── */
#define M1_A()  GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15)
#define M1_B()  GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12)
#define M2_A()  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)
#define M2_B()  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)
#define M3_A()  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)
#define M3_B()  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)
#define M4_A()  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)
#define M4_B()  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
// 控制周期计数器（每10ms触发一次巡线控制）
static uint32_t s_control_counter = 0U;

// 巡线控制就绪标志，主循环中查询
volatile uint8_t g_control_flag = 0U;

void SysTick_Handler(void)
{
  Delay_IncTick();

  s_control_counter++;
  if (s_control_counter >= 10U)
  {
    s_control_counter = 0U;
    g_control_flag = 1U;
  }
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/******************************************************************************/
/*            编码器 EXTI 中断 (4路 4倍频正交解码)                            */
/*                                                                            */
/*  引脚 → EXTI 映射:                                                         */
/*    M1_A: PA15 → EXTI15    M1_B: PA12 → EXTI12                             */
/*    M2_A: PB4  → EXTI4     M2_B: PB3  → EXTI3                              */
/*    M3_A: PB8  → EXTI8     M3_B: PB5  → EXTI5                              */
/*    M4_A: PB10 → EXTI10    M4_B: PB9  → EXTI9                              */
/******************************************************************************/

void EXTI3_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
        Encoder_ISR_Handler(1, M2_A(), M2_B());  /* Motor2 */
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

void EXTI4_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) != RESET) {
        Encoder_ISR_Handler(1, M2_A(), M2_B());  /* Motor2 */
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
        Encoder_ISR_Handler(2, M3_A(), M3_B());  /* Motor3 */
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
        Encoder_ISR_Handler(2, M3_A(), M3_B());  /* Motor3 */
        EXTI_ClearITPendingBit(EXTI_Line8);
    }
    if (EXTI_GetITStatus(EXTI_Line9) != RESET) {
        Encoder_ISR_Handler(3, M4_A(), M4_B());  /* Motor4 */
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
        Encoder_ISR_Handler(0, M1_A(), M1_B());  /* Motor1 */
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
    if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
        Encoder_ISR_Handler(0, M1_A(), M1_B());  /* Motor1 */
        EXTI_ClearITPendingBit(EXTI_Line15);
    }
    if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
        Encoder_ISR_Handler(3, M4_A(), M4_B());  /* Motor4 */
        EXTI_ClearITPendingBit(EXTI_Line10);
    }
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
