/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Actuators_Init();
void TIM2_Init();
void TIM3_Init();
volatile int timePassed = 0;
volatile int timeServo = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  Actuators_Init();
  TIM2_Init();
  TIM3_Init();
  int direction = 0;
  typedef enum {
	  LEFT = 50,
	  CENTER = 150,
	  RIGHT = 250,
  }servoState;
  servoState currentState = CENTER;
    /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (timePassed > 35) {
		  if (!direction) {
			  TIM2->CCR1 += 15; // increase slower
			  if (TIM2->CCR1 > 950) { // 100% duty cycle
				  direction = !direction; // start dimming led
			  }
		  }
		  else {
			  TIM2->CCR1 -= 25; // decrease faster
			  if (TIM2->CCR1 < 100) { // 10% duty cycle
				  direction = !direction; // start lighting up led
			  }
		  }
		  timePassed = 0; // reset timer
	  }

	  switch(currentState) {
	  	  case CENTER:
	  		  if (timeServo > 1000){
	  			  TIM3->CCR2 = LEFT;
	  			  currentState = LEFT;
	  			  timeServo = 0;
	  		  }
	  		  break;
	  	  case RIGHT:
	  		  if (timeServo > 1000) {
	  			  TIM3->CCR2 = CENTER;
	  			  currentState = CENTER;
	  			  timeServo = 0;
	  		  }
	  		  break;
	  	  case LEFT:
	  		  if (timeServo > 1000) {
	  			  TIM3->CCR2 = RIGHT;
	  			  currentState = RIGHT;
	  			  timeServo = 0;
	  		  }
	  		  break;
	  }

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}
void Actuators_Init() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER &= ~GPIO_MODER_MODER5;
	GPIOA->MODER |= 0b10 << GPIO_MODER_MODER5_Pos; // aternate function pa5
	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL5);
	GPIOA->AFR[0] |= (0b0001 << GPIO_AFRL_AFRL5_Pos);

	GPIOA->MODER &= ~GPIO_MODER_MODER0; // PA0 external led
	GPIOA->MODER |= 0b01 << GPIO_MODER_MODER0_Pos; // GPO

	GPIOA->MODER &= ~GPIO_MODER_MODER1; // PA1 external
	GPIOA->MODER |= 0b10 << GPIO_MODER_MODER1_Pos; // alternate function
	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL1);
	GPIOA->AFR[0] |= (0b0001 << GPIO_AFRL_AFRL1_Pos);

	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_0;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_1;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_4;
	//SERVO PA4
	GPIOA->MODER &= ~GPIO_MODER_MODER4;
	GPIOA->MODER |= 0b10 << GPIO_MODER_MODER4_Pos;
	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL4);
	GPIOA->AFR[0] |= (0b0010<< GPIO_AFRL_AFRL4_Pos);
}

void TIM3_Init() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable timer 3
	TIM3->PSC = 160 - 1; // (16 * 10 ^ 6) / (16 * 10) = 10 ^ 5 = 100khZ frequency
	TIM3->ARR = 2000 - 1; // (10 ^ 5) / (2 * 10 ^ 3) = 50 hZ frequency
	TIM3->CCR2 = 150 - 1; // start servo in center

	TIM3->CNT = 0; //reset counter
	TIM3->CR1 |= 0b1; //enable counter

	TIM3->CCMR1 |= 0b110 << 12; // pwm mode 1 for ch 2 tim 3
	TIM3->CCER |= 0b1 << 4; // enable output compare ch2
}

/* USER CODE BEGIN 4 */
void TIM2_Init() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable timer clock
	TIM2->PSC = 16 - 1; // set prescaler to 16, so that we get a convenient reload value. PSC of 16 gives us a clock frequency of 1mhZ
	TIM2->ARR = 1000 - 1; // Clock/prescaler * arr = (16 * 10 ^ 6) / (16 * (10 ^ 3)) = 10 ^ 3 = 1khZ frequency PWM signal (1000 cycles in a second or a tick each millisecond)
	TIM2->CCR1 = 1000 - 1; // start pulsing at 100% duty cycle on ch1

	TIM2->CCR2 = 250 - 1; // compare value ch2 - external LED is at constant 75% duty cycle (we're using pwm mode 2 for ch2)

	TIM2->CNT = 0; // reset counter
	TIM2->CR1 |= 0b1; // enable counter for timer2

	TIM2->CCMR1 |= 0b110 << 4; // set pwm mode 1 for ch1 tim 2 (OC1M)
	TIM2->CCMR1 |= 0b111 << 12; // pwm mode 2 (low - high) for ch2 tim 2

	TIM2->CCER |= 0b1; // enable output compare for channel 1
	TIM2->CCER |= 0b1 << 4; // enable output compare ch 2

	TIM2->DIER |= TIM_DIER_CC1IE; // enable interrupts channel 1
	NVIC_EnableIRQ(TIM2_IRQn); // enable interrupt vector

}

void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_CC1IF) { // check which channel generated the interrupt
		TIM2->SR &= ~TIM_SR_CC1IF; // reset flag
		GPIOA->ODR |= GPIO_ODR_0; // turn on external led using interrupts and general purpose outputt
		timePassed++; // USING TIM2 AS BASIC TIMER AS WELL
		timeServo++;
	}
}

void TIM3_IRQHandler(void) {
	if (TIM3->SR & TIM_SR_CC2IF) { // check which channel generated the interrupt
		TIM3->SR &= ~TIM_SR_CC2IF; // reset flag
	}
}

void SysTick_Handler(void)
{

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}



#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
