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
#include <string.h>
#include "ota_metadata.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_ERROR_Pin             GPIO_PIN_14
#define LED_GPIO_Port             GPIOD

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SRAM_BASE_ADDRESS         0x20000000U
#define SRAM_END_ADDRESS          0x20020000U
#define FLASH_BASE_ADDRESS        0x08000000U
#define FLASH_END_ADDRESS         0x08100000U
#define BOOT_DEFAULT_MAX_ATTEMPTS 3U

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static OtaMetadata_t bootMetadata;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Debug_PrintLine(const char *line);
static uint8_t Boot_IsValidApplication(uint32_t appAddress);
static void Boot_JumpToApplication(uint32_t appAddress);
static uint32_t Boot_GetSlotAddress(uint32_t slot);
static uint8_t Boot_CanBootSlot(uint32_t slot);
static uint8_t Boot_TryJumpSlot(uint32_t slot, const char *slotName);
static void Boot_SeedDefaultMetadata(void);
static void Boot_Process(void);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Debug_PrintLine("Bootloader started\r\n");
  Boot_Process();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_ERROR_Pin);
    HAL_Delay(250);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LED_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void Debug_PrintLine(const char *line)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100U);
}

static uint8_t Boot_IsValidApplication(uint32_t appAddress)
{
  uint32_t initialSp = *(volatile uint32_t *)appAddress;
  uint32_t resetHandler = *(volatile uint32_t *)(appAddress + 4U);

  if ((initialSp < SRAM_BASE_ADDRESS) || (initialSp > SRAM_END_ADDRESS))
  {
    return 0U;
  }

  if ((resetHandler < FLASH_BASE_ADDRESS) || (resetHandler >= FLASH_END_ADDRESS))
  {
    return 0U;
  }

  if ((resetHandler & 1U) == 0U)
  {
    return 0U;
  }

  return 1U;
}

static uint32_t Boot_GetSlotAddress(uint32_t slot)
{
  if (slot == OTA_SLOT_B)
  {
    return OTA_APP_SLOT_B_ADDRESS;
  }

  return OTA_APP_SLOT_A_ADDRESS;
}

static uint8_t Boot_CanBootSlot(uint32_t slot)
{
  return (Boot_IsValidApplication(Boot_GetSlotAddress(slot)) != 0U) ? 1U : 0U;
}

static void Boot_SeedDefaultMetadata(void)
{
  if (OtaMetadata_Save(&gOtaMetadataDefault) == HAL_OK)
  {
    Debug_PrintLine("Default metadata initialized\r\n");
  }
  else
  {
    Debug_PrintLine("Default metadata init failed\r\n");
  }
}

static uint8_t Boot_TryJumpSlot(uint32_t slot, const char *slotName)
{
  uint32_t appAddress;

  appAddress = Boot_GetSlotAddress(slot);
  if (Boot_IsValidApplication(appAddress) == 0U)
  {
    Debug_PrintLine("Selected slot vector invalid\r\n");
    return 0U;
  }

  if (slotName != NULL)
  {
    Debug_PrintLine(slotName);
  }
  Debug_PrintLine("Jumping to application\r\n");
  Boot_JumpToApplication(appAddress);

  return 1U;
}

static void Boot_Process(void)
{
  uint8_t metadataValid;
  uint32_t preferredSlot;
  uint8_t metadataDirty = 0U;

  metadataValid = OtaMetadata_Load(&bootMetadata);

  if (metadataValid != 0U)
  {
    if ((bootMetadata.confirmedSlot != OTA_SLOT_A) && (bootMetadata.confirmedSlot != OTA_SLOT_B))
    {
      bootMetadata.confirmedSlot = OTA_SLOT_A;
      metadataDirty = 1U;
    }

    if ((bootMetadata.bootSlot != OTA_SLOT_A) && (bootMetadata.bootSlot != OTA_SLOT_B))
    {
      bootMetadata.bootSlot = bootMetadata.confirmedSlot;
      metadataDirty = 1U;
    }

    if (bootMetadata.maxBootAttempts == 0U)
    {
      bootMetadata.maxBootAttempts = BOOT_DEFAULT_MAX_ATTEMPTS;
      metadataDirty = 1U;
    }

    if (bootMetadata.updateInProgress != 0U)
    {
      if (bootMetadata.bootAttemptCount >= bootMetadata.maxBootAttempts)
      {
        bootMetadata.bootSlot = bootMetadata.confirmedSlot;
        bootMetadata.updateInProgress = 0U;
        bootMetadata.bootAttemptCount = 0U;
        bootMetadata.lastErrorCode = OTA_UPDATE_ERROR_FLASH;
        metadataDirty = 1U;
        Debug_PrintLine("Rollback: max boot attempts reached\r\n");
      }
      else
      {
        bootMetadata.bootAttemptCount++;
        metadataDirty = 1U;
      }
    }

    if (metadataDirty != 0U)
    {
      bootMetadata.metadataVersion++;
      if (OtaMetadata_Save(&bootMetadata) != HAL_OK)
      {
        Debug_PrintLine("Metadata save failed\r\n");
      }
    }

    preferredSlot = bootMetadata.bootSlot;

    if ((preferredSlot == OTA_SLOT_A) || (preferredSlot == OTA_SLOT_B))
    {
      if (Boot_CanBootSlot(preferredSlot) != 0U)
      {
        if (preferredSlot == OTA_SLOT_A)
        {
          if (Boot_TryJumpSlot(OTA_SLOT_A, "Preferred slot: A\r\n") != 0U)
          {
            return;
          }
        }
        else
        {
          if (Boot_TryJumpSlot(OTA_SLOT_B, "Preferred slot: B\r\n") != 0U)
          {
            return;
          }
        }
      }
    }

    if ((preferredSlot != OTA_SLOT_A) && (Boot_CanBootSlot(OTA_SLOT_A) != 0U))
    {
      if (Boot_TryJumpSlot(OTA_SLOT_A, "Fallback slot: A\r\n") != 0U)
      {
        return;
      }
    }

    if ((preferredSlot != OTA_SLOT_B) && (Boot_CanBootSlot(OTA_SLOT_B) != 0U))
    {
      if (Boot_TryJumpSlot(OTA_SLOT_B, "Fallback slot: B\r\n") != 0U)
      {
        return;
      }
    }
  }
  else
  {
    Debug_PrintLine("Metadata invalid, try default slots\r\n");
    Boot_SeedDefaultMetadata();
    if (Boot_TryJumpSlot(OTA_SLOT_A, "Default slot: A\r\n") != 0U)
    {
      return;
    }

    if (Boot_TryJumpSlot(OTA_SLOT_B, "Default slot: B\r\n") != 0U)
    {
      return;
    }
  }

  Debug_PrintLine("No valid application, staying in bootloader\r\n");
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
}

static void Boot_JumpToApplication(uint32_t appAddress)
{
  uint32_t appStack = *(volatile uint32_t *)appAddress;
  uint32_t appResetHandler = *(volatile uint32_t *)(appAddress + 4U);
  void (*AppEntry)(void) = (void (*)(void))appResetHandler;
  uint32_t index;

  HAL_SuspendTick();
  __disable_irq();

  for (index = 0U; index < 8U; index++)
  {
    NVIC->ICER[index] = 0xFFFFFFFFU;
    NVIC->ICPR[index] = 0xFFFFFFFFU;
  }

  HAL_RCC_DeInit();
  HAL_DeInit();

  SCB->VTOR = appAddress;
  __set_MSP(appStack);
  __enable_irq();
  __DSB();
  __ISB();
  AppEntry();

  while (1)
  {
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
