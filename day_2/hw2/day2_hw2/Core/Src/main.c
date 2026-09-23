/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body  (Torque-only test)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t PCrxBuffer[RX_BUFFER_SIZE];

/* 디버깅용: 마지막으로 보낸 패킷 확인 */
uint8_t lastPacket[16];
uint8_t lastPacketLen = 0;
volatile uint8_t packetReady = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
void writePacket(uint8_t packet[], int length);
void setTorque(uint8_t id, uint8_t on);
void write1Byte(uint8_t id, uint16_t addr, uint8_t data);
void setGoalPosition(uint8_t id, uint32_t position);
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
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)PCrxBuffer);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)&USART3->DR);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, RX_BUFFER_SIZE);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
  LL_USART_EnableDMAReq_RX(USART3);
  LL_USART_EnableIT_IDLE(USART3);

  /* 모터 부팅 대기 후 토크 ON 한 번만 전송 */
  LL_mDelay(500);            // 모터 부팅 대기
  setTorque(14, 0);          // 1. 토크 OFF
  LL_mDelay(10);
  write1Byte(14, 11, 3);     // 2. Operating Mode = 3 (위치 모드)
  LL_mDelay(50);             //    EEPROM 저장 대기
  setTorque(14, 1);          // 3. 토크 ON
  LL_mDelay(10);
  setGoalPosition(14, 2048); // 4. 테스트: 180도로 이동

  /* setTorque 패킷 기대값:
     FF FF FD 00 0E 06 00 03 40 00 01 [CRC_L] [CRC_H]
     -> lastPacket 에서 확인 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  setTorque(14, 1);          // 3. 토크 ON
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)){
		  setGoalPosition(14, 1024); // 4. 테스트: 90도로 이동
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  }
	  else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)){
	  		  setGoalPosition(14, 2048); // 4. 테스트: 90도로 이동
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  	  }
	  else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)){
	  		  setGoalPosition(14, 10); // 4. 테스트: 90도로 이동
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  	  }
	  else{
		  setTorque(14, 0);      // 토크 명령 반복 전송
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	  }
	      LL_mDelay(1000);       // 1초마다


    /* 아무것도 안 함. 토크가 걸리는지 손으로 모터를 돌려 확인.
       뻑뻑하면 토크 ON 성공, 스르륵 돌아가면 실패. */
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
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
void write1Byte(uint8_t id, uint16_t addr, uint8_t data) {
    uint8_t packet[13];
    uint16_t crc;
    packet[0]=0xFF; packet[1]=0xFF; packet[2]=0xFD; packet[3]=0x00;
    packet[4]=id;
    packet[5]=6; packet[6]=0;
    packet[7]=0x03;
    packet[8]=addr & 0xFF; packet[9]=(addr >> 8) & 0xFF;
    packet[10]=data;
    crc=update_crc(0, packet, 11);
    packet[11]=crc & 0xFF;
    packet[12]=(crc >> 8) & 0xFF;
    writePacket(packet, 13);
}
void setTorque(uint8_t id, uint8_t on) {
    uint8_t packet[13];
    uint16_t crc;
    packet[0]=0xFF; packet[1]=0xFF; packet[2]=0xFD; packet[3]=0x00;
    packet[4]=id;
    packet[5]=6; packet[6]=0;        // Length = 파라미터3 + 3 = 6
    packet[7]=0x03;                  // Write
    packet[8]=64; packet[9]=0;       // Torque Enable 주소 64 (하위=64, 상위=0)
    packet[10]=on;                   // 1=on, 0=off
    crc=update_crc(0, packet, 11);
    packet[11] = crc & 0xFF;
    packet[12] = (crc >> 8) & 0xFF;								//(crc>>8) & 0xFF;
    writePacket(packet, 13);
}
void setGoalPosition(uint8_t id, uint32_t position) {
    uint8_t packet[16];
    uint16_t crc;
    packet[0]=0xFF; packet[1]=0xFF; packet[2]=0xFD; packet[3]=0x00;
    packet[4]=id;
    packet[5]=9; packet[6]=0;                 // Length = 파라미터6 + 3 = 9
    packet[7]=0x03;                           // Write
    packet[8]=116; packet[9]=0;               // Goal Position 주소 116
    packet[10]=position & 0xFF;               // 위치값 리틀엔디안 4바이트
    packet[11]=(position >> 8) & 0xFF;
    packet[12]=(position >> 16) & 0xFF;
    packet[13]=(position >> 24) & 0xFF;
    crc=update_crc(0, packet, 14);
    packet[14]=crc & 0xFF;
    packet[15]=(crc >> 8) & 0xFF;
    writePacket(packet, 16);
}
void writePacket(uint8_t packet[], int length){
    // 디버깅용: 만든 패킷을 전역 배열에 복사
    for (int i = 0; i < length && i < 16; i++) {
        lastPacket[i] = packet[i];
    }
    lastPacketLen = length;
    packetReady = 1;

    // 전송
    for (int i=0;i<length;i++){
        while(!LL_USART_IsActiveFlag_TXE(USART3));
        LL_USART_TransmitData8(USART3, packet[i]);
    }
    // 마지막 바이트(CRC)까지 완전히 나갈 때까지 대기 (AutoDirection 안전)
    while(!LL_USART_IsActiveFlag_TC(USART3));
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
