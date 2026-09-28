/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : DAY3 과제1/2 - TIM1 CH1(PA8) PWM 20kHz DC 모터 제어
  *
  *  [보드 실측 결과]  PA8 = 드라이버 PWM,  PC8 = 드라이버 DIR
  *                   DIR LOW = CW(시계), DIR HIGH = CCW(반시계)  (모터를 바라본 기준)
  *
  *  [CubeMX 설정]
  *   - TIM8 Channel3       : Disable
  *   - PC8                 : GPIO_Output (초기 Low)
  *   - TIM1 Channel1       : PWM Generation CH1 (PA8)
  *   - Prescaler           : 9-1
  *   - Counter Period(ARR) : 1000-1
  *     -> 180,000,000 / 9 / 1000 = 20,000 Hz (20kHz)
  *     -> CCR1 = 0 ~ 1000 (0 = duty 0%, 1000 = duty 100%)
  *   - CH Polarity High, PWM mode 1
  *
  *  [과제1] ASSIGNMENT 1 : PB12 누를 때마다 duty 0 -> 30 -> 50 -> 70 -> 0 ...
  *  [과제2] ASSIGNMENT 2 : Live Expressions 에서 motor_cmd 에 -1.0 ~ 1.0 입력
  *                         (+1 = CW 70%, -1 = CCW 70%, 0 = 정지)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 실행할 과제 선택: 1 또는 2 */
#define ASSIGNMENT      2

#define PWM_PERIOD      1000    /* ARR + 1 */
#define MAX_DUTY_RATIO  1.0

/* 방향 핀 (실측: LOW = CW, HIGH = CCW) */
#define DIR_PORT        GPIOC
#define DIR_PIN         GPIO_PIN_8
#define DIR_CW          GPIO_PIN_RESET
#define DIR_CCW         GPIO_PIN_SET

#define DEBOUNCE_MS     50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t ccr_value = 0;          /* 실제 CCR1 값 (Live Expressions 확인용) */

#if ASSIGNMENT == 1
static const uint8_t duty_steps[] = { 0, 30, 50, 70 };
volatile uint8_t duty_index   = 0;
volatile uint8_t duty_percent = 0;        /* 현재 duty % */
static GPIO_PinState btn_last = GPIO_PIN_RESET;
static uint32_t      btn_tick = 0;
#else
volatile double motor_cmd = 0.0;          /* -1.0 ~ 1.0 */
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#if ASSIGNMENT == 1
static void    set_duty_percent(uint8_t percent);
static uint8_t pb12_pressed(void);
#else
static void    motor_drive(double cmd);
#endif
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
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* 방향 CW, duty 0% 로 PWM 시작 */
  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, DIR_CW);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

#if ASSIGNMENT == 1
  btn_last = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);  /* 시작 시 오동작 방지 */
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if ASSIGNMENT == 1
    if (pb12_pressed())
    {
      duty_index = (duty_index + 1) % (sizeof(duty_steps) / sizeof(duty_steps[0]));
      set_duty_percent(duty_steps[duty_index]);
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);        /* 눌림 확인용 LED */
    }
#else
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)){
    	motor_cmd=0.3;
    }
    else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)){
        	motor_cmd=0.5;
        }
    else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)){
        	motor_cmd=0.7;
        }
    else{
    	motor_cmd=0;
    }
    motor_drive(motor_cmd);
#endif
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
#if ASSIGNMENT == 1
/* 과제1: duty % -> CCR1 (방향은 CW 고정) */
static void set_duty_percent(uint8_t percent)
{
  if (percent > 100) percent = 100;
  duty_percent = percent;
  ccr_value = (uint16_t)((uint32_t)PWM_PERIOD * percent / 100);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_value);
}

/* PB12 가 눌린 순간에만 1 반환 (누르면 HIGH 인 회로 기준) */
static uint8_t pb12_pressed(void)
{
  GPIO_PinState now = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);

  if (now != btn_last && (HAL_GetTick() - btn_tick) >= DEBOUNCE_MS)
  {
    btn_last = now;
    btn_tick = HAL_GetTick();
    return (now == GPIO_PIN_SET);
  }
  return 0;
}
#else
/* 과제2: 부호 = 방향, 크기 = 속도 (최대 70%) */
static void motor_drive(double cmd)
{
  if (cmd >  1.0) cmd =  1.0;
  if (cmd < -1.0) cmd = -1.0;

  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, (cmd >= 0.0) ? DIR_CW : DIR_CCW);

  ccr_value = (uint16_t)(fabs(cmd) * MAX_DUTY_RATIO * PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_value);
}
#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
