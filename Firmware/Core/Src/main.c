/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <SMS_Sensors.h>
#include <DHT11_Sensors.h>
#include <protocol.h>
#include "SX1278.h"
#include <DS18B20_Sensors.h>
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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
SMS_Data_t result;
SX1278_hw_t SX1278_hw;
SX1278_t SX1278;
int master;
int ret;
uint8_t buffer[512];
int message;
int message_length;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void ReadAllSensors(void);
void Set_RTC_Alarm(uint32_t seconds);
// Chuyển hướng printf sang UART1
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, HAL_MAX_DELAY);
    return len;
}

void ReadAllSensors(void) {
    // 1. Đọc cảm biến độ ẩm đất (Analog)
    SMS_Data_t soil_data = read_SMS_V1(&hadc1);

    // 2. Đọc cảm biến DHT11 (Môi trường)
    DHT11_Data_t dht_data = DHT11_Read(DHT_11_GPIO_Port, DHT_11_Pin);

    // 3. LẤY KẾT QUẢ DS18B20 (Đã được kích hoạt đo từ lúc nhận SYNC)
    float ds_temp = DS18B20_GetTempResult();

    // 4. In dữ liệu debug qua UART (Dùng cách tách nguyên/thập phân để an toàn)
    printf("\r\n--- GUI DU LIEU SLOT %d ---\r\n", Node_assigned_slot);

    // In độ ẩm đất
    printf("Soil Moist: %d%%\r\n", soil_data.moist_pct);

    // In DHT11 (Nhiệt độ/Độ ẩm môi trường)
    int t_int = (int)dht_data.Temperature;
    int t_dec = abs((int)(dht_data.Temperature * 10) % 10);
    printf("Env: %d.%d C | Hum: %d%%\r\n", t_int, t_dec, (int)dht_data.Humidity);

    // In DS18B20 (Nhiệt độ đất/nước)
    int ds_int = (int)ds_temp;
    int ds_dec = abs((int)(ds_temp * 10) % 10);
    printf("Soil Temp: %d.%d C\r\n", ds_int, ds_dec);

    // 5. GỬI DỮ LIỆU QUA LORA (Chỉ còn 5 tham số, đã bỏ Pin)
    // Thứ tự: ID, Temp_Env, Hum_Env, Temp_Soil, Hum_Soil
    Send_Sensor_Data(Node_assigned_slot,
                     dht_data.Temperature,
                     dht_data.Humidity,
                     ds_temp,
                     (float)soil_data.moist_pct);

    printf(">>> LoRa: Da gui goi DATA (11 bytes) len Gateway\r\n");
    printf("--------------------------\r\n");
}

void Check_STM32_UUID(void) {
      // STM32F103 có 96-bit UID nằm tại địa chỉ 0x1FFFF7E8
      uint32_t uid0 = *(uint32_t*)(0x1FFFF7E8);
      uint32_t uid1 = *(uint32_t*)(0x1FFFF7EC);
      uint32_t uid2 = *(uint32_t*)(0x1FFFF7F0);

      // In ra dạng Hex để dễ quan sát
      printf("STM32 Unique ID: %08X-%08X-%08X\r\n", uid0, uid1, uid2);
  }
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
//  master = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
//
//  if (master == GPIO_PIN_SET) {
//	  printf("Mode: Master (Sender)\r\n");
//  } else {
//	  printf("Mode: Slave (Receiver)\r\n");
//  }

  // Gán cấu hình phần cứng cho thư viện SX1278
  SX1278_hw.dio0.port = GPIOB;
  SX1278_hw.dio0.pin = GPIO_PIN_1;
  SX1278_hw.nss.port = GPIOA;
  SX1278_hw.nss.pin = GPIO_PIN_4;
  SX1278_hw.reset.port = GPIOB;
  SX1278_hw.reset.pin = GPIO_PIN_0;
  SX1278_hw.spi = &hspi1;

  SX1278.hw = &SX1278_hw;
  printf("Checking LoRa Connection...\r\n");
  printf("Configuring LoRa module...\r\n");
  // Cấu hình: 433MHz, 17dBm, SF7, BW 125kHz, CR 4/5, CRC ON
  SX1278_init(&SX1278, 433000000, SX1278_POWER_17DBM, SX1278_LORA_SF_7,
			  SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 12);
  printf("Done configuring LoRa Module.\r\n");


//  if (ret > 0) {
//	  printf("LoRa Connection: OK! (Return code: %d)\r\n", ret);
//  } else {
//	  printf("LoRa Connection: FAILED! (Return code: %d)\r\n", ret);
//	  printf("Please check wiring and SPI configuration.\r\n");
//  }

  if(DS18B20_Init()) {
      printf("DS18B20 Found!\n");
  }
  Check_STM32_UUID();
//  SX1278_receive(&SX1278, (uint8_t)sizeof(buffer), 1000);
//  printf("LoRa listening for SYNC...\n");
  SX1278_receive(&SX1278, 12, 1000); // Bắt đầu nghe
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Infinite loop */
    /* USER CODE BEGIN WHILE */
  /* Infinite loop */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        // 1. Ưu tiên lắng nghe LoRa
        if (SX1278_available(&SX1278)) {
            // Đọc đủ 16 bytes để không sót dữ liệu JOIN_ACK
            int len = SX1278_read(&SX1278, buffer, 16);

            // Kiểm tra Header 0xAA để lọc nhiễu
            if (len > 0 && buffer[0] == 0xAA) {
                if (buffer[1] == PKT_SYNC) {
                    handle_sync_packet(buffer);
                }
                else if (buffer[1] == PKT_JOIN_ACK) {
                    handle_join_ack(buffer);
                }
            }

            // Quay lại chế độ nhận 16 bytes ngay lập tức
            SX1278_receive(&SX1278, 16, 1000);
        }

        // 2. Kiểm tra đến mốc thời gian hành động
        if (target_action_time != 0 && now >= target_action_time) {
            uint32_t current_action_time = target_action_time; // Lưu lại mốc hiện tại
            target_action_time = 0; // Xóa mốc ngay để tránh re-trigger

            // TRƯỜNG HỢP A: Gửi JOIN REQUEST
            if (node_state == STATE_WAIT_TO_JOIN) {
                printf(">>> Đang gửi Join Request (96-bit UUID)...\r\n");
                Send_Join_Request();
                node_state = STATE_IDLE; // Chuyển sang trạng thái chờ phản hồi

                // Cực kỳ quan trọng: Mở cửa sổ nhận JOIN_ACK 16 bytes
                SX1278_receive(&SX1278, 16, 2000);
            }

            // TRƯỜNG HỢP B: Gửi DATA cảm biến
            else if (node_state == STATE_WAIT_MY_SLOT) {
                printf(">>> Đến Slot %d! Đang gửi DATA...\r\n", Node_assigned_slot);
                ReadAllSensors();
                node_state = STATE_IDLE;

                // Quay lại nghe để chờ gói SYNC của chu kỳ tiếp theo
                SX1278_receive(&SX1278, 16, 1000);
            }
        }
    }

      /* USER CODE BEGIN 3 */
    /* USER CODE END 3 */
  }
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LORA_RST_Pin|DHT_11_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_CS_Pin */
  GPIO_InitStruct.Pin = LORA_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LORA_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_RST_Pin */
  GPIO_InitStruct.Pin = LORA_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LORA_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT_11_Pin */
  GPIO_InitStruct.Pin = DHT_11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT_11_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
