/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  uint16_t led_location[4]={0x0001,0x0002,0x0004,0x0400};
  int led_index=3;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)){

	  		  //스위치(PB12) 눌리면 LED PB0, PB1, PB2, PB10 우측 이동

	  	led_index++;

	  	if (led_index==4){

	  		led_index=0;

	  	}

	  	for (int i=0;i<4;i++){

	  		 if (led_index==i){

	  		  		HAL_GPIO_WritePin(GPIOB,led_location[i],GPIO_PIN_RESET);

	  		 }

	  	else{

	  		  HAL_GPIO_WritePin(GPIOB, led_location[i],GPIO_PIN_SET);

	  		}

	  	}

	  }

	  	  else{

	  		  led_index--;

	  		  if (led_index==-1){

	  			  led_index=3;

	  		  }

	  		  for (int i=0;i<4;i++){

	  			  if (led_index==i){

	  				  HAL_GPIO_WritePin(GPIOB,led_location[i],GPIO_PIN_RESET);

	  			  }

	  			  else{

	  				  HAL_GPIO_WritePin(GPIOB, led_location[i],GPIO_PIN_SET);

	  			  }

	  		  }

	  	  }



	  	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)){

	  		  //스위치(PB14) 눌리면 한번에 shift되는 LED 개수 2개
	  		  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)){
	  			  led_index-=2;
	  		  }
	  		  else{
	  			  led_index--;
	  		  }

	  		  if (led_index<0){

	  			  led_index=4+led_index;

	  		  }

	  		  for (int i=0;i<4;i++){

	  		  			  if (led_index==i){

	  		  				  HAL_GPIO_WritePin(GPIOB,led_location[i],GPIO_PIN_RESET);

	  		  			  }

	  		  			  else{

	  		  				  HAL_GPIO_WritePin(GPIOB, led_location[i],GPIO_PIN_SET);

	  		  			  }

	  		  		  }

	  	  }
	  	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)){

	  		  		  //스위치(PB13) 눌리면 LED ON OFF 반전

	  		  		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

	  		  		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);

	  		  		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);

	  		  		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);

	  		  	  }
	  if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)){

	  		  //스위치(PB15) 눌리면 PD2 TOGGLE 일시정지 (LED의 TOGGLE 직전 상태 유지)

	  	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);

	  }
	  HAL_Delay(500);


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
