/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : DAY3 과제3 - Robstride RS00 액추에이터 CW/CCW 제어 (CAN, 속도 모드)
  *
  *  [조건]  Vin 24V, 모터 ID 14, 1Mbps, 속도는 데이터시트 최대(33 rad/s)의 절반 미만
  *
  *  [CubeMX CAN1 설정]  APB1 = 45MHz
  *   - Prescaler 3, BS1 12 Times, BS2 2 Times, SJW 1 Time  -> 1,000,000 bit/s
  *   - Automatic Bus-Off Management : Enable
  *   - Automatic Retransmission     : Enable
  *   - Operating Mode               : Normal
  *   - 핀: PA11 = CAN1_RX, PA12 = CAN1_TX
  *
  *  [조작]
  *   - PB12 : CW  /  PB13 : CCW  /  PB14 : 정지
  *   - 또는 Live Expressions 에서 motor_dir 에 1 / -1 / 0 입력
  *   - 속도 크기는 speed_mag (rad/s, 최대 SPEED_MAX 로 제한)
  *
  *  [RS00 프로토콜 요약]  29bit 확장 ID = (통신타입 << 24) | (data2 << 8) | 대상 ID
  *   - 타입 3  : 인에이블      타입 4 : 정지
  *   - 타입 18 : 파라미터 쓰기 (Byte0~1 = index, Byte4~7 = 값, 리틀 엔디안)
  *   - 타입 2  : 모터 피드백   (각도/속도/토크/온도, 빅 엔디안)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_ID        1u			//id를 1로 수정
#define MASTER_ID       0xFDu

/* 통신 타입 */
#define COMM_FEEDBACK   2u
#define COMM_ENABLE     3u
#define COMM_STOP       4u
#define COMM_WRITE      18u

/* 파라미터 index */
#define IDX_RUN_MODE    0x7005u   /* uint8 : 2 = 속도 모드 */
#define IDX_SPD_REF     0x700Au   /* float : rad/s        */
#define IDX_LIMIT_CUR   0x7018u   /* float : A            */
#define IDX_ACC_RAD     0x7022u   /* float : rad/s^2      */

#define RUN_MODE_SPEED  2u

/* 안전 설정 */
#define SPEED_MAX       15.0f     /* 데이터시트 33 rad/s 의 절반(16.5) 미만 */
#define SPEED_DEFAULT   5.0f      /* 처음엔 천천히 */
#define CURRENT_LIMIT   3.0f      /* 정격 상전류 4.7Apk 이하 */
#define ACCEL           10.0f     /* 속도 모드 가속도 */

/* 양수 spd_ref 가 CW 인지는 실측으로 확인 후, 반대면 -1.0f 로 변경 */
#define CW_SIGN         1.0f

#define SEND_PERIOD_MS  20u
#define PI_F            3.14159265f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 명령 (Live Expressions 로 변경 가능) */
volatile int8_t  motor_dir = 0;             /* 1 = CW, -1 = CCW, 0 = 정지 */
volatile float   speed_mag = SPEED_DEFAULT; /* rad/s */
volatile float   spd_cmd   = 0.0f;          /* 실제로 보낸 속도 명령 */

volatile uint32_t rx_any = 0;
volatile uint32_t last_rx_id = 0;

/* 피드백 (Live Expressions 확인용) */
volatile float   fb_pos    = 0.0f;          /* rad   */
volatile float   fb_vel    = 0.0f;          /* rad/s */
volatile float   fb_torque = 0.0f;          /* Nm    */
volatile float   fb_temp   = 0.0f;          /* 섭씨  */
volatile uint8_t fb_mode   = 0;             /* 0 Reset, 1 Cali, 2 Motor(운전 중) */
volatile uint8_t fb_fault  = 0;             /* 0 이면 정상 */
volatile uint32_t rx_count = 0;             /* 피드백 수신 횟수 (통신 확인용) */
volatile uint32_t tx_error = 0;             /* 송신 실패 횟수 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void    can_filter_accept_all(void);
static void    rs_send(uint8_t type, uint16_t data2, const uint8_t data[8]);
static void    rs_enable(void);
static void    rs_stop(void);
static void    rs_write_u8(uint16_t index, uint8_t value);
static void    rs_write_float(uint16_t index, float value);
static void    rs_poll_feedback(void);
static float   uint_to_float(uint16_t x, float x_min, float x_max);
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  can_filter_accept_all();
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_Delay(500);                               /* 모터 부팅 대기 */

  /* 매뉴얼 순서: 정지 -> run_mode=2 -> 인에이블 -> limit_cur -> (가속도) -> spd_ref */
  rs_stop();                                    /* 모드 변경은 정지 상태에서만 가능 */
  HAL_Delay(5);
  rs_write_u8(IDX_RUN_MODE, RUN_MODE_SPEED);
  HAL_Delay(5);
  rs_enable();
  HAL_Delay(5);
  rs_write_float(IDX_LIMIT_CUR, CURRENT_LIMIT);
  HAL_Delay(5);
  rs_write_float(IDX_ACC_RAD, ACCEL);
  HAL_Delay(5);
  rs_write_float(IDX_SPD_REF, 0.0f);            /* 처음엔 정지 */

  uint32_t last_send = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 버튼 (누르면 HIGH) */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) motor_dir =  1;   /* CW   */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) motor_dir = -1;   /* CCW  */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)) motor_dir =  0;   /* 정지 */

    /* 주기적으로 속도 명령 전송 */
    if (HAL_GetTick() - last_send >= SEND_PERIOD_MS)
    {
      last_send = HAL_GetTick();

      float mag = speed_mag;
      if (mag < 0.0f)      mag = 0.0f;
      if (mag > SPEED_MAX) mag = SPEED_MAX;

      int8_t dir = motor_dir;
      if (dir > 1)  dir = 1;
      if (dir < -1) dir = -1;

      spd_cmd = CW_SIGN * (float)dir * mag;
      rs_write_float(IDX_SPD_REF, spd_cmd);
    }

    rs_poll_feedback();
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
/* 모든 CAN 프레임을 FIFO0 으로 받기 */
static void can_filter_accept_all(void)
{
  CAN_FilterTypeDef f = {0};
  f.FilterBank           = 0;
  f.FilterMode           = CAN_FILTERMODE_IDMASK;
  f.FilterScale          = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh         = 0x0000;
  f.FilterIdLow          = 0x0000;
  f.FilterMaskIdHigh     = 0x0000;
  f.FilterMaskIdLow      = 0x0000;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation     = ENABLE;
  f.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK)
  {
    Error_Handler();
  }
}

/* 확장 ID = (type << 24) | (data2 << 8) | MOTOR_ID 로 8바이트 전송 */
static void rs_send(uint8_t type, uint16_t data2, const uint8_t data[8])
{
  CAN_TxHeaderTypeDef h = {0};
  uint32_t mailbox;
  uint8_t  buf[8];

  memcpy(buf, data, 8);
  h.ExtId = ((uint32_t)(type & 0x1F) << 24) | ((uint32_t)data2 << 8) | MOTOR_ID;
  h.IDE   = CAN_ID_EXT;
  h.RTR   = CAN_RTR_DATA;
  h.DLC   = 8;
  h.TransmitGlobalTime = DISABLE;

  /* 빈 메일박스 대기 (최대 10ms) */
  uint32_t t0 = HAL_GetTick();
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
  {
    if (HAL_GetTick() - t0 > 10) { tx_error++; return; }
  }
  if (HAL_CAN_AddTxMessage(&hcan1, &h, buf, &mailbox) != HAL_OK)
  {
    tx_error++;
  }
}

/* 타입 3: 인에이블 (data 전부 0) */
static void rs_enable(void)
{
  uint8_t d[8] = {0};
  rs_send(COMM_ENABLE, MASTER_ID, d);
}

/* 타입 4: 정지 (Byte0 = 0, 1 이면 고장 클리어) */
static void rs_stop(void)
{
  uint8_t d[8] = {0};
  rs_send(COMM_STOP, MASTER_ID, d);
}

/* 타입 18: 1바이트 파라미터 쓰기 (값은 Byte4) */
static void rs_write_u8(uint16_t index, uint8_t value)
{
  uint8_t d[8] = {0};
  memcpy(&d[0], &index, 2);    /* 리틀 엔디안 */
  d[4] = value;
  rs_send(COMM_WRITE, MASTER_ID, d);
}

/* 타입 18: float 파라미터 쓰기 (값은 Byte4~7, IEEE-754 리틀 엔디안) */
static void rs_write_float(uint16_t index, float value)
{
  uint8_t d[8] = {0};
  memcpy(&d[0], &index, 2);
  memcpy(&d[4], &value, 4);
  rs_send(COMM_WRITE, MASTER_ID, d);
}

/* 0~65535 -> 실수 범위 */
static float uint_to_float(uint16_t x, float x_min, float x_max)
{
  return (float)x * (x_max - x_min) / 65535.0f + x_min;
}

/* 수신 FIFO 비우면서 타입 2 피드백 해석 */
static void rs_poll_feedback(void)
{
  CAN_RxHeaderTypeDef rh;
  uint8_t d[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rh, d) != HAL_OK) break;
    rx_any++;
    last_rx_id = rh.ExtId;
    if (rh.IDE != CAN_ID_EXT || rh.DLC < 8) continue;

    uint8_t type   = (rh.ExtId >> 24) & 0x1F;
    uint8_t src_id = (rh.ExtId >> 8)  & 0xFF;     /* bit15~8 = 모터 ID */
    if (type != COMM_FEEDBACK || src_id != MOTOR_ID) continue;

    fb_fault  = (rh.ExtId >> 16) & 0x3F;
    fb_mode   = (rh.ExtId >> 22) & 0x03;

    /* 피드백 데이터는 상위 바이트가 먼저 */
    fb_pos    = uint_to_float((uint16_t)((d[0] << 8) | d[1]), -4.0f * PI_F, 4.0f * PI_F);
    fb_vel    = uint_to_float((uint16_t)((d[2] << 8) | d[3]), -33.0f, 33.0f);
    fb_torque = uint_to_float((uint16_t)((d[4] << 8) | d[5]), -14.0f, 14.0f);
    fb_temp   = (float)((d[6] << 8) | d[7]) / 10.0f;
    rx_count++;
  }
}
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
