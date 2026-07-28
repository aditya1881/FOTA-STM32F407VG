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
#include <string.h>
#include "ota_metadata.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef CAN_NODE_ID
#define CAN_NODE_ID 7U
#endif

#define CAN_TX_STD_ID             (0x120U + (CAN_NODE_ID & 0x7U))
#define CAN_HEARTBEAT_PERIOD_MS   1000U
#define OTA_CAN_ID_SYNC           0x320U
#define OTA_CAN_ID_START          0x321U
#define OTA_CAN_ID_DATA           0x322U
#define OTA_CAN_ID_END            0x323U
#define OTA_CAN_ID_ACK            0x324U
#define OTA_CAN_ID_NACK           0x325U
#define OTA_CAN_ID_QUERY          0x326U
#define OTA_CAN_ID_STATUS         0x327U

#define OTA_QUERY_ACTIVE_STATUS   0U
#define OTA_QUERY_SLOT_A_META     1U
#define OTA_QUERY_SLOT_B_META     2U

#define OTA_SYNC_TOKEN_0          'O'
#define OTA_SYNC_TOKEN_1          'T'
#define OTA_SYNC_TOKEN_2          'A'
#define OTA_SYNC_TOKEN_3          'S'
#define OTA_SYNC_TOKEN_4          'Y'
#define OTA_SYNC_TOKEN_5          'N'
#define OTA_SYNC_TOKEN_6          'C'
#define OTA_SYNC_TOKEN_7          '!'

#define OTA_FOOTER_MAGIC          0x454E4431U
#define OTA_DATA_SEQ_BYTES        2U
#define OTA_DATA_PAYLOAD_SIZE     4U

#define OTA_RX_STATE_IDLE         0U
#define OTA_RX_STATE_WAIT_HEADER  1U
#define OTA_RX_STATE_RECEIVING    2U
#define OTA_RX_STATE_DONE         3U

#define OTA_ERR_OK                0U
#define OTA_ERR_PROTOCOL          1U
#define OTA_ERR_FLASH             2U
#define OTA_ERR_SIZE              3U
#define OTA_ERR_CRC               4U
#define LED_TX_Pin                GPIO_PIN_12
#define LED_RX_Pin                GPIO_PIN_15
#define LED_ERROR_Pin             GPIO_PIN_14
#define LED_GPIO_Port             GPIOD

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//===================================

//===================================
static volatile uint8_t canRxDebugPending;
static CAN_RxHeaderTypeDef canRxDebugHeader;
static uint8_t canRxDebugData[8];
static volatile uint8_t canFrameReady;
static volatile CAN_RxHeaderTypeDef canFrameHeader;
static volatile uint8_t canFrameData[8];

static uint8_t otaRxState;
static uint32_t otaExpectedSize;
static uint32_t otaExpectedCrc;
static uint32_t otaExpectedVersion;
static uint32_t otaReceivedSize;
static uint32_t otaRunningCrc;
static uint32_t otaWriteAddress;
static uint16_t otaExpectedDataSeq;
static uint32_t otaTargetSlot;
static uint32_t otaTargetSlotBase;
static uint32_t otaTargetSlotSize;
static uint32_t otaTargetSlotSector;
static uint32_t otaTargetSlotSectorCount;
static uint8_t otaRebootPending;
static uint32_t otaRebootTick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_App_Init(void);
static void Debug_PrintLine(const char *line);
static void Debug_PrintCanFrame(const char *label, uint32_t stdId, uint32_t dlc, const uint8_t *data);
static void CAN_PrintPendingRxFrame(void);
static void OTA_Init(void);
static void OTA_ProcessIncomingFrame(void);
static uint32_t OTA_Crc32_Update(uint32_t runningCrc, const uint8_t *data, uint32_t length);
static void OTA_SelectTargetSlot(void);
static HAL_StatusTypeDef OTA_EraseTargetSlot(void);
static HAL_StatusTypeDef OTA_WriteChunk(uint32_t address, const uint8_t *data, uint32_t length);
static void OTA_SendStatusFrame(uint32_t stdId, uint8_t code, uint32_t value);
static void OTA_SendRawFrame(uint32_t stdId, const uint8_t *data, uint8_t dlc);
static void OTA_ConfirmPendingImageEarly(void);

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

  /* Confirm as early as possible to survive resets before full peripheral bring-up. */
  OTA_ConfirmPendingImageEarly();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN_App_Init();
  Debug_PrintLine("CAN debug UART ready\r\n");
  Debug_PrintLine("Build: OTA RX\r\n");
  OTA_Init();
  uint32_t lastAppBlinkTick = HAL_GetTick();  
  HAL_UART_Transmit(&huart2, (uint8_t *)"GPIO_PIN_15 Blinking SLOT B\r\n",30U,100U);

 //========================================================================

//=======================================================

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    CAN_PrintPendingRxFrame();
    OTA_ProcessIncomingFrame();
    if ((HAL_GetTick() - lastAppBlinkTick) >= 1000U)
    {
      lastAppBlinkTick += 1000U;
     
      // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_TX_Pin);
      // HAL_GPIO_TogglePin(LED_GPIO_Port, GPIO_PIN_13);
      // HAL_GPIO_TogglePin(LED_GPIO_Port, GPIO_PIN_14);
      HAL_GPIO_TogglePin(LED_GPIO_Port, GPIO_PIN_15);
      HAL_Delay(50);


    }
    // if ((HAL_GetTick() - lastRxStatusTick) >= 2000U)
    // {
    //   lastRxStatusTick += 2000U;
    //   Debug_PrintLine("OTA RX waiting\r\n");
    // }

    if ((otaRebootPending != 0U) && ((int32_t)(HAL_GetTick() - otaRebootTick) >= 0))
    {
      HAL_Delay(20U);
      NVIC_SystemReset();
    }
//=====================================================================================================

//==========================================================================
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  /* Enable GPIOE clock for CS pin */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(LED_GPIO_Port, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_12, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */
static void Debug_PrintLine(const char *line)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100U);
}

static void OTA_ConfirmPendingImageEarly(void)
{
  OtaMetadata_t meta;

  if ((OtaMetadata_Load(&meta) != 0U) && (meta.updateInProgress != 0U))
  {
    (void)OtaMetadata_ConfirmActiveImage();
  }
}

static void Debug_PrintCanFrame(const char *label, uint32_t stdId, uint32_t dlc, const uint8_t *data)
{
  char line[128];
  int length = snprintf(line, sizeof(line),
                        "%s ID=0x%03lX DLC=%lu DATA=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                        label,
                        (unsigned long)stdId,
                        (unsigned long)dlc,
                        data[0], data[1], data[2], data[3],
                        data[4], data[5], data[6], data[7]);

  if (length > 0)
  {
    size_t bytesToSend = (length < (int)sizeof(line)) ? (size_t)length : strlen(line);
    HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)bytesToSend, 100U);
  }
}

static void CAN_App_Init(void)
{
  CAN_FilterTypeDef canFilterConfig = {0};

  canFilterConfig.FilterBank = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF | CAN_IT_ERROR) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  if (hcan->Instance != CAN1)
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_ERROR_Pin);
    return;
  }

  if ((rxHeader.IDE == CAN_ID_STD) && (rxHeader.RTR == CAN_RTR_DATA))
  {
    canRxDebugHeader = rxHeader;
    memcpy(canRxDebugData, rxData, sizeof(canRxDebugData));
    canRxDebugPending = 1U;

    if (canFrameReady == 0U)
    {
      canFrameHeader = rxHeader;
      memcpy((void *)canFrameData, rxData, sizeof(canFrameData));
      canFrameReady = 1U;
    }

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_RX_Pin);
  }
}

static void CAN_PrintPendingRxFrame(void)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  if (canRxDebugPending == 0U)
  {
    return;
  }

  __disable_irq();
  rxHeader = canRxDebugHeader;
  memcpy(rxData, canRxDebugData, sizeof(rxData));
  canRxDebugPending = 0U;
  __enable_irq();

  Debug_PrintCanFrame("RX", rxHeader.StdId, rxHeader.DLC, rxData);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_ERROR_Pin);

  
  }
}

static void OTA_Init(void)
{
  otaRxState = OTA_RX_STATE_IDLE;
  otaExpectedSize = 0U;
  otaExpectedCrc = 0U;
  otaExpectedVersion = 0U;
  otaReceivedSize = 0U;
  otaRunningCrc = 0xFFFFFFFFU;
  otaWriteAddress = 0U;
  otaExpectedDataSeq = 0U;
  otaTargetSlot = OTA_SLOT_B;
  otaTargetSlotBase = OTA_APP_SLOT_B_ADDRESS;
  otaTargetSlotSize = OTA_APP_SLOT_B_SIZE_BYTES;
  otaTargetSlotSector = FLASH_SECTOR_6;
  otaTargetSlotSectorCount = 2U;
  otaRebootPending = 0U;
  otaRebootTick = 0U;
}

static void OTA_SelectTargetSlot(void)
{
  OtaMetadata_t metadata;
  uint8_t metadataValid;
  uint32_t activeSlot = OTA_SLOT_A;

  memcpy(&metadata, &gOtaMetadataDefault, sizeof(OtaMetadata_t));
  metadataValid = OtaMetadata_Load(&metadata);

  if (metadataValid != 0U)
  {
    if ((metadata.activeSlot == OTA_SLOT_A) || (metadata.activeSlot == OTA_SLOT_B))
    {
      activeSlot = metadata.activeSlot;
    }
  }

  otaTargetSlot = (activeSlot == OTA_SLOT_A) ? OTA_SLOT_B : OTA_SLOT_A;

  if (otaTargetSlot == OTA_SLOT_A)
  {
    otaTargetSlotBase = OTA_APP_SLOT_A_ADDRESS;
    otaTargetSlotSize = OTA_APP_SLOT_A_SIZE_BYTES;
    otaTargetSlotSector = FLASH_SECTOR_4;
    otaTargetSlotSectorCount = 2U;
    otaExpectedVersion = metadata.appAVersion + 1U;
  }
  else
  {
    otaTargetSlotBase = OTA_APP_SLOT_B_ADDRESS;
    otaTargetSlotSize = OTA_APP_SLOT_B_SIZE_BYTES;
    otaTargetSlotSector = FLASH_SECTOR_6;
    otaTargetSlotSectorCount = 2U;
    otaExpectedVersion = metadata.appBVersion + 1U;
  }
}

static uint32_t OTA_Crc32_Update(uint32_t runningCrc, const uint8_t *data, uint32_t length)
{
  uint32_t crc = runningCrc;
  uint32_t i;

  for (i = 0U; i < length; i++)
  {
    uint32_t value = data[i];
    uint32_t bit;
    crc ^= value;
    for (bit = 0U; bit < 8U; bit++)
    {
      uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }

  return crc;
}

static HAL_StatusTypeDef OTA_EraseTargetSlot(void)
{
  FLASH_EraseInitTypeDef eraseConfig = {0};
  uint32_t sectorError = 0U;
  HAL_StatusTypeDef status;

  eraseConfig.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseConfig.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  eraseConfig.Sector = otaTargetSlotSector;
  eraseConfig.NbSectors = otaTargetSlotSectorCount;

  HAL_FLASH_Unlock();
  status = HAL_FLASHEx_Erase(&eraseConfig, &sectorError);
  HAL_FLASH_Lock();

  return (status == HAL_OK) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef OTA_WriteChunk(uint32_t address, const uint8_t *data, uint32_t length)
{
  uint32_t i;
  uint32_t paddedLength = (length + 3U) & ~3U;
  uint8_t localBuffer[OTA_DATA_PAYLOAD_SIZE] = {0};

  if (length > sizeof(localBuffer))
  {
    return HAL_ERROR;
  }

  memcpy(localBuffer, data, length);

  HAL_FLASH_Unlock();
  for (i = 0U; i < paddedLength; i += 4U)
  {
    uint32_t word = ((uint32_t)localBuffer[i + 0U] << 0U) |
                    ((uint32_t)localBuffer[i + 1U] << 8U) |
                    ((uint32_t)localBuffer[i + 2U] << 16U) |
                    ((uint32_t)localBuffer[i + 3U] << 24U);

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, word) != HAL_OK)
    {
      HAL_FLASH_Lock();
      return HAL_ERROR;
    }
  }
  HAL_FLASH_Lock();

  return HAL_OK;
}

static void OTA_SendStatusFrame(uint32_t stdId, uint8_t code, uint32_t value)
{
  uint8_t payload[8];

  payload[0] = code;
  payload[1] = (uint8_t)(value >> 0U);
  payload[2] = (uint8_t)(value >> 8U);
  payload[3] = (uint8_t)(value >> 16U);
  payload[4] = (uint8_t)(value >> 24U);
  payload[5] = 0U;
  payload[6] = 0U;
  payload[7] = 0U;

  OTA_SendRawFrame(stdId, payload, 8U);
}

static void OTA_SendRawFrame(uint32_t stdId, const uint8_t *data, uint8_t dlc)
{
  CAN_TxHeaderTypeDef txHeader = {0};
  uint32_t mailbox;

  txHeader.StdId = stdId;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = dlc;
  txHeader.TransmitGlobalTime = DISABLE;

  (void)HAL_CAN_AddTxMessage(&hcan1, &txHeader, (uint8_t *)data, &mailbox);
}

static void OTA_ProcessIncomingFrame(void)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  uint32_t payloadLen;
  uint16_t rxSeq;

  if (canFrameReady == 0U)
  {
    return;
  }

  __disable_irq();
  header = canFrameHeader;
  memcpy(data, (const void *)canFrameData, sizeof(data));
  canFrameReady = 0U;
  __enable_irq();

  if (header.DLC > 8U)
  {
    OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_PROTOCOL, 0U);
    return;
  }

  if (header.StdId == OTA_CAN_ID_QUERY)
  {
    OtaMetadata_t metadata;
    uint8_t payload[8] = {0};

    if ((header.DLC != 1U) || (OtaMetadata_Load(&metadata) == 0U))
    {
      memcpy(&metadata, &gOtaMetadataDefault, sizeof(metadata));
    }

    if (header.DLC != 1U)
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_PROTOCOL, 0U);
      return;
    }

    if (data[0] == OTA_QUERY_ACTIVE_STATUS)
    {
      payload[0] = (uint8_t)metadata.activeSlot;
      payload[1] = (uint8_t)metadata.bootSlot;
      payload[2] = (uint8_t)metadata.confirmedSlot;
      payload[3] = (uint8_t)metadata.updateInProgress;
      payload[4] = (uint8_t)metadata.bootAttemptCount;
      payload[5] = (uint8_t)metadata.maxBootAttempts;
      payload[6] = (uint8_t)metadata.lastErrorCode;
      payload[7] = (uint8_t)metadata.metadataVersion;
      OTA_SendRawFrame(OTA_CAN_ID_STATUS, payload, 8U);
      return;
    }

    if (data[0] == OTA_QUERY_SLOT_A_META)
    {
      payload[0] = (uint8_t)(metadata.appASizeBytes >> 0U);
      payload[1] = (uint8_t)(metadata.appASizeBytes >> 8U);
      payload[2] = (uint8_t)(metadata.appASizeBytes >> 16U);
      payload[3] = (uint8_t)(metadata.appASizeBytes >> 24U);
      payload[4] = (uint8_t)(metadata.appACrc32 >> 0U);
      payload[5] = (uint8_t)(metadata.appACrc32 >> 8U);
      payload[6] = (uint8_t)(metadata.appACrc32 >> 16U);
      payload[7] = (uint8_t)(metadata.appACrc32 >> 24U);
      OTA_SendRawFrame(OTA_CAN_ID_STATUS, payload, 8U);
      return;
    }

    if (data[0] == OTA_QUERY_SLOT_B_META)
    {
      payload[0] = (uint8_t)(metadata.appBSizeBytes >> 0U);
      payload[1] = (uint8_t)(metadata.appBSizeBytes >> 8U);
      payload[2] = (uint8_t)(metadata.appBSizeBytes >> 16U);
      payload[3] = (uint8_t)(metadata.appBSizeBytes >> 24U);
      payload[4] = (uint8_t)(metadata.appBCrc32 >> 0U);
      payload[5] = (uint8_t)(metadata.appBCrc32 >> 8U);
      payload[6] = (uint8_t)(metadata.appBCrc32 >> 16U);
      payload[7] = (uint8_t)(metadata.appBCrc32 >> 24U);
      OTA_SendRawFrame(OTA_CAN_ID_STATUS, payload, 8U);
      return;
    }

    OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_PROTOCOL, data[0]);
    return;
  }

  if (header.StdId == OTA_CAN_ID_SYNC)
  {
    if ((header.DLC == 8U) &&
        (data[0] == (uint8_t)OTA_SYNC_TOKEN_0) &&
        (data[1] == (uint8_t)OTA_SYNC_TOKEN_1) &&
        (data[2] == (uint8_t)OTA_SYNC_TOKEN_2) &&
        (data[3] == (uint8_t)OTA_SYNC_TOKEN_3) &&
        (data[4] == (uint8_t)OTA_SYNC_TOKEN_4) &&
        (data[5] == (uint8_t)OTA_SYNC_TOKEN_5) &&
        (data[6] == (uint8_t)OTA_SYNC_TOKEN_6) &&
        (data[7] == (uint8_t)OTA_SYNC_TOKEN_7))
    {
      OTA_Init();
      OTA_SelectTargetSlot();
      otaRxState = OTA_RX_STATE_WAIT_HEADER;
      OTA_SendStatusFrame(OTA_CAN_ID_ACK, OTA_ERR_OK, otaTargetSlot);
      Debug_PrintLine("OTA sync accepted\r\n");
    }
    else
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_PROTOCOL, 0U);
    }
    return;
  }

  if (header.StdId == OTA_CAN_ID_START)
  {
    uint32_t rxSize = ((uint32_t)data[0] << 0U) |
                      ((uint32_t)data[1] << 8U) |
                      ((uint32_t)data[2] << 16U) |
                      ((uint32_t)data[3] << 24U);
    uint32_t rxCrc = ((uint32_t)data[4] << 0U) |
                     ((uint32_t)data[5] << 8U) |
                     ((uint32_t)data[6] << 16U) |
                     ((uint32_t)data[7] << 24U);

    // Idempotent START: if sender retries START while we already moved to
    // RECEIVING and no data has been written yet, ACK it again instead of NACK.
    if ((otaRxState == OTA_RX_STATE_RECEIVING) &&
        (otaReceivedSize == 0U) &&
        (otaExpectedSize == rxSize) &&
        (otaExpectedCrc == rxCrc))
    {
      OTA_SendStatusFrame(OTA_CAN_ID_ACK, OTA_ERR_OK, otaExpectedSize);
      return;
    }

    if (otaRxState != OTA_RX_STATE_WAIT_HEADER)
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_PROTOCOL, 0U);
      return;
    }

    otaExpectedSize = rxSize;
    otaExpectedCrc = rxCrc;

    if ((otaExpectedSize == 0U) || (otaExpectedSize > otaTargetSlotSize))
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_SIZE, otaExpectedSize);
      (void)OtaMetadata_RecordLastError(OTA_UPDATE_ERROR_SIZE);
      OTA_Init();
      return;
    }

    if (OTA_EraseTargetSlot() != HAL_OK)
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_FLASH, 0U);
      (void)OtaMetadata_RecordLastError(OTA_UPDATE_ERROR_FLASH);
      OTA_Init();
      return;
    }

    otaRxState = OTA_RX_STATE_RECEIVING;
    otaReceivedSize = 0U;
    otaRunningCrc = 0xFFFFFFFFU;
    otaWriteAddress = otaTargetSlotBase;
    OTA_SendStatusFrame(OTA_CAN_ID_ACK, OTA_ERR_OK, otaExpectedSize);
    return;
  }

  if (header.StdId == OTA_CAN_ID_DATA)
  {
    if (otaRxState != OTA_RX_STATE_RECEIVING)
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_PROTOCOL, 0U);
      return;
    }

    if (header.DLC <= OTA_DATA_SEQ_BYTES)
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_PROTOCOL, 0U);
      OTA_Init();
      return;
    }

    rxSeq = ((uint16_t)data[0] << 0U) |
            ((uint16_t)data[1] << 8U);

    if (rxSeq != otaExpectedDataSeq)
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_PROTOCOL, otaExpectedDataSeq);
      OTA_Init();
      return;
    }

    payloadLen = header.DLC - OTA_DATA_SEQ_BYTES;
    if ((otaReceivedSize + payloadLen) > otaExpectedSize)
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_SIZE, otaReceivedSize);
      (void)OtaMetadata_RecordLastError(OTA_UPDATE_ERROR_SIZE);
      OTA_Init();
      return;
    }

    if (OTA_WriteChunk(otaWriteAddress, &data[OTA_DATA_SEQ_BYTES], payloadLen) != HAL_OK)
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_FLASH, otaReceivedSize);
      (void)OtaMetadata_RecordLastError(OTA_UPDATE_ERROR_FLASH);
      OTA_Init();
      return;
    }

    otaRunningCrc = OTA_Crc32_Update(otaRunningCrc, &data[OTA_DATA_SEQ_BYTES], payloadLen);
    otaReceivedSize += payloadLen;
    otaExpectedDataSeq++;
    otaWriteAddress += ((payloadLen + 3U) & ~3U);
    OTA_SendStatusFrame(OTA_CAN_ID_ACK, OTA_ERR_OK, otaReceivedSize);
    return;
  }

  if (header.StdId == OTA_CAN_ID_END)
  {
    uint32_t finalCrc;
    uint32_t footerMagic;
    uint32_t footerSize;

    if (otaRxState != OTA_RX_STATE_RECEIVING)
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_PROTOCOL, 0U);
      return;
    }

    if (header.DLC != 8U)
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_PROTOCOL, 0U);
      OTA_Init();
      return;
    }

    footerMagic = ((uint32_t)data[0] << 0U) |
                  ((uint32_t)data[1] << 8U) |
                  ((uint32_t)data[2] << 16U) |
                  ((uint32_t)data[3] << 24U);
    footerSize = ((uint32_t)data[4] << 0U) |
                 ((uint32_t)data[5] << 8U) |
                 ((uint32_t)data[6] << 16U) |
                 ((uint32_t)data[7] << 24U);

    if ((footerMagic != OTA_FOOTER_MAGIC) || (footerSize != otaExpectedSize))
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_PROTOCOL, footerSize);
      OTA_Init();
      return;
    }

    finalCrc = ~otaRunningCrc;
    if ((otaReceivedSize != otaExpectedSize) || (finalCrc != otaExpectedCrc))
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_CRC, finalCrc);
      (void)OtaMetadata_RecordLastError(OTA_UPDATE_ERROR_CRC);
      OTA_Init();
      return;
    }

    if (OtaMetadata_SetUpdatePending(otaTargetSlot, otaExpectedSize, finalCrc, otaExpectedVersion) != HAL_OK)
    {
      OTA_SendStatusFrame(OTA_CAN_ID_NACK, OTA_ERR_FLASH, 0U);
      OTA_Init();
      return;
    }

    otaRxState = OTA_RX_STATE_DONE;
    OTA_SendStatusFrame(OTA_CAN_ID_ACK, OTA_ERR_OK, finalCrc);
    if (otaTargetSlot == OTA_SLOT_A)
    {
      Debug_PrintLine("OTA image stored in Slot A\r\n");
    }
    else
    {
      Debug_PrintLine("OTA image stored in Slot B\r\n");
    }

    Debug_PrintLine("OTA success. Rebooting to bootloader...\r\n");
    otaRebootPending = 1U;
    otaRebootTick = HAL_GetTick() + 300U;
    return;
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
