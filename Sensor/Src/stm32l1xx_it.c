/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// FIXME: Calibrar los valores del servo


uint8_t measure_en = 1;

extern uint8_t send_en;
extern uint8_t start_measuring;


extern uint32_t flank_detect_en;
extern uint32_t start_time, end_time;
extern uint32_t value;
extern uint32_t vin;
extern uint32_t vin,last_vin,didnotmove_pot;
extern uint8_t modo;
extern uint8_t turn;
extern uint16_t distances[NUM_SAMPLES];
uint8_t counter_samples = 0;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Manda un pulso al sensor
void send_pulse() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    for(int w = 0; w < 300; ++w) {}
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim3);
    flank_detect_en = RISING_EDGE;
}

void cambio_posicion() {

  if (turn == 0) {
  	// De -90 a +90
  	vin = vin + INC;
  	if (vin >= D_P90 || counter_samples == 6) {
  	    TIM2->CCR1 = D_P90;
  	    turn = 1;
  	} else {
  	    TIM2->CCR1 = vin;
  	}

      } else if (turn == 1 || counter_samples == 6) {
  	// De + 90 a -90
  	vin = vin - INC;
  	if(vin <= D_M90){
  	    TIM2->CCR1 = D_M90;
  	    turn = 0;
  	} else {
  	    TIM2->CCR1 = vin;
  	}
}

}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim9;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC global interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */
    // Obtenemos el valor del potenciometro para mover el sensor
    // Funciona solo en el modo manual
    // Se toman datos cada vez que se pulse el boton

    if (modo == MODO_A) {
	return;
    }

    value = HAL_ADC_GetValue(&hadc);

//    last_vin = vin; //comprobamos estos valores, si es igual que el anterior para apagar el adc

    vin = value * 3300/4096; // Valor en el fondo de escala

    if (vin >= D_P90) vin = D_P90;
    if (vin <= D_M90) vin = D_M90;

    TIM2->CCR1 = D_M90 + vin;


//    if(last_vin==vin){
//	didnotmove_pot++;
//    } else if(didnotmove_pot>=5000){
//	start_measuring=1;
//	didnotmove_pot=0;
//	HAL_ADC_Stop_IT(&hadc);
//    }

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc);
  /* USER CODE BEGIN ADC1_IRQn 1 */
  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles TIM9 global interrupt.
  */
void TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM9_IRQn 0 */

    // Mueve el servo en el modo automatico

    if (modo == MODO_M) {
	return;
    }

    if (counter_samples > 0) cambio_posicion();
    send_pulse();

  /* USER CODE END TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM9_IRQn 1 */
  /* USER CODE END TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  __disable_irq();
    // Lectura de la signal del sensor

    HAL_TIM_Base_Stop_IT(&htim9);



    if(flank_detect_en == RISING_EDGE) {
	start_time = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
	flank_detect_en = FALLING_EDGE;

    } else if (flank_detect_en == FALLING_EDGE) {
	end_time = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
	TIM3->SR &= ~1;

	// Si estamos en modo manual, tomamos solo un dato
	// En modo manual tomamos NUM_SAMPLES y los ponemos

	if ( modo == MODO_M) {
	    // Deshabilitamos la captura de datos hasta que se vuelva a pulsar el boton
	    distances[counter_samples] = ((end_time - start_time) / 58);
	    HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_1);
	    HAL_TIM_Base_Stop_IT(&htim3);

	    counter_samples = 0;
	    flank_detect_en = NONE_EDGE;
	    send_en = 1;


	} else if (modo == MODO_A) {

	    distances[counter_samples] = (end_time - start_time) / 58;


	    if (counter_samples < NUM_SAMPLES) {
		counter_samples++;
		flank_detect_en = RISING_EDGE;
		HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_1);
		HAL_TIM_Base_Stop_IT(&htim3);
		send_en = 0;
	    } else {
		counter_samples = 0;
		flank_detect_en = NONE_EDGE;
		HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
		HAL_TIM_Base_Stop_IT(&htim3);
		send_en = 1;

	    }

	    HAL_TIM_Base_Start_IT(&htim9);

	}

    }
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */
    __enable_irq();
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  // En el modo manual, habilitamos la captura de un dato
  // En el modo automatico, no hace nada

  __disable_irq();
  EXTI->PR |= GPIO_PIN_13;

  if (modo == MODO_A) {
      __enable_irq();
      return;

  } else if (modo == MODO_M) {

//      if(flank_detect_en == NONE_EDGE && start_measuring == 1) {
	  HAL_TIM_Base_Stop(&htim9);
          HAL_TIM_Base_Stop_IT(&htim9);

          __HAL_ADC_DISABLE(&hadc);
          HAL_ADC_Stop_IT(&hadc);
          send_pulse();

          flank_detect_en = RISING_EDGE;
//     }
  }

  __enable_irq();
  /* USER CODE END EXTI15_10_IRQn 0 */
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
