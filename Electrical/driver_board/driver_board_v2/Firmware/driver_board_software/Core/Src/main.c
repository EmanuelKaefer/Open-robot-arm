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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Die Konstanten für die State Machine
typedef enum {
    STATE_INIT,
	STATE_HOMING,
    STATE_READY,
    STATE_ERROR
} SystemState;

SystemState currentState = STATE_INIT;

// --- SPI Write Funktion ---
void TMC_WriteReg(uint8_t addr, uint32_t value) {
    uint8_t data[5];
    data[0] = addr | 0x80; // MSB auf 1 setzen für Write
    data[1] = (value >> 24) & 0xFF;
    data[2] = (value >> 16) & 0xFF;
    data[3] = (value >> 8)  & 0xFF;
    data[4] = value & 0xFF;

    // CS LOW, Senden, CS HIGH
    HAL_GPIO_WritePin(TMC1_CS_GPIO_Port, TMC1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, data, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(TMC1_CS_GPIO_Port, TMC1_CS_Pin, GPIO_PIN_SET);
}

// --- SPI Read Funktion ---
uint32_t TMC_ReadReg(uint8_t addr) {
    uint8_t tx_data[5] = {addr & 0x7F, 0, 0, 0, 0}; // MSB auf 0 für Read
    uint8_t rx_data[5] = {0};

    // 1. Transfer: Dem TMC5160 sagen, welches Register wir wollen
    HAL_GPIO_WritePin(TMC1_CS_GPIO_Port, TMC1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx_data, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(TMC1_CS_GPIO_Port, TMC1_CS_Pin, GPIO_PIN_SET);

    // 2. Transfer: Die eigentlichen Daten abholen
    HAL_GPIO_WritePin(TMC1_CS_GPIO_Port, TMC1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(TMC1_CS_GPIO_Port, TMC1_CS_Pin, GPIO_PIN_SET);

    // Bytes zusammenbauen (rx_data[0] ist die Dummy-Antwort der Adresse)
    return ((uint32_t)rx_data[1] << 24) | ((uint32_t)rx_data[2] << 16) |
           ((uint32_t)rx_data[3] << 8)  | rx_data[4];
}
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /* USER CODE BEGIN 3 */
	  switch (currentState) {

	      case STATE_INIT:
	          printf("\r\n--- Starte TMC5160 Initialisierung ---\r\n");

	          // Clear latched faults
	          TMC_WriteReg(0x01, 0x00000007);
	          HAL_Delay(100); // 0.1 Sekunden warten

	          // Configure TMC5160
	          TMC_WriteReg(0x00, 0x00000004);   // GCONF
	          TMC_WriteReg(0x10, 0x00061008);   // IHOLD_IRUN
	          TMC_WriteReg(0x6C, 0x14010053);   // CHOPCONF
	          TMC_WriteReg(0x70, 0xC10D0024);   // PWMCONF
	          HAL_Delay(100);

	          // Verify
	          uint32_t gstat = TMC_ReadReg(0x01);
	          uint32_t drvstatus = TMC_ReadReg(0x6F);
	          uint32_t cs_actual = (drvstatus >> 16) & 0x1F;
	          uint8_t undervoltage = (gstat & (1<<2)) ? 1 : 0;

	          printf("TMC5160 Konfiguration abgeschlossen\r\n");
	          printf("  GSTAT:     0x%08lX\r\n", gstat);
	          printf("  CS actual: %lu\r\n", cs_actual);
	          printf("  Undervoltage: %d\r\n", undervoltage);

	          if (gstat == 0 && cs_actual > 0) {
	              printf("  >>> Treiber bereit! <<<\r\n");
	              currentState = STATE_HOMING; // Gehe in den nächsten State!
	          } else {
	              printf("  >>> FEHLER - Verdrahtung/Strom prüfen! <<<\r\n");
	              currentState = STATE_ERROR; // Bleibe im Fehler-State
	          }
	          break;
	      case STATE_HOMING:
	    	  //Hier wird das Homing procedere Stattfinden, er fährt mit jedem joint in einen endschalter
	    	  break;
	      case STATE_READY:
	          // Hier kommt später der Code rein, der die Simulink-UART-Daten liest
	          // und die Timer für die STEP-Pulse füttert.
	          break;

	      case STATE_ERROR:
	          // Hier gefangen. LED blinken lassen o.ä.
	          HAL_Delay(1000);
	          printf("Warte auf Fehlerbehebung...\r\n");
	          break;
	  }
	  /* USER CODE END 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TMC1_CS_GPIO_Port, TMC1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : TMC1_CS_Pin */
  GPIO_InitStruct.Pin = TMC1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TMC1_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
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
