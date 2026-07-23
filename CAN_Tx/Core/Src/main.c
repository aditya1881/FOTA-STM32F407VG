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
#include <stdbool.h>
#include <string.h>
#include "flash_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef CAN_NODE_ID
#define CAN_NODE_ID 5U
#endif

#define CAN_TX_STD_ID             (0x120U + (CAN_NODE_ID & 0x7U))
#define OTA_CAN_ID_SYNC           0x320U
#define OTA_CAN_ID_START          0x321U
#define OTA_CAN_ID_DATA           0x322U
#define OTA_CAN_ID_END            0x323U
#define OTA_CAN_ID_ACK            0x324U
#define OTA_CAN_ID_NACK           0x325U

#define OTA_SYNC_TOKEN_0          'O'
#define OTA_SYNC_TOKEN_1          'T'
#define OTA_SYNC_TOKEN_2          'A'
#define OTA_SYNC_TOKEN_3          'S'
#define OTA_SYNC_TOKEN_4          'Y'
#define OTA_SYNC_TOKEN_5          'N'
#define OTA_SYNC_TOKEN_6          'C'
#define OTA_SYNC_TOKEN_7          '!'

#define OTA_FOOTER_MAGIC          0x454E4431U
#define OTA_STORED_MAGIC          0x3241544FU  // 'OTA2'

#define OTA_UART_MAGIC            0x3141544FU
#define OTA_UART_HEADER_SIZE      12U
#define OTA_UART_MAX_IMAGE_SIZE   0x00040000U

#define OTA_DATA_SEQ_BYTES        2U
#define OTA_DATA_PAYLOAD_SIZE     4U
#define OTA_SEND_FRAME_TIMEOUT_MS 200U
#define OTA_SEND_SYNC_TIMEOUT_MS  500U
#define OTA_SEND_START_TIMEOUT_MS 3000U
#define OTA_SEND_DATA_TIMEOUT_MS  500U
#define OTA_SEND_END_TIMEOUT_MS   1500U
#define OTA_SEND_FRAME_RETRY_MAX  3U
#define OTA_REPLY_QUEUE_LEN       8U
#define LED_TX_Pin                GPIO_PIN_12
#define LED_RX_Pin                GPIO_PIN_15
#define LED_ERROR_Pin             GPIO_PIN_14
#define LED_GPIO_Port             GPIOD
#define USER_BUTTON_Pin           GPIO_PIN_0
#define USER_BUTTON_GPIO_Port     GPIOA
#define USER_BUTTON_ACTIVE_STATE  GPIO_PIN_SET
#define USER_BUTTON2_Pin          GPIO_PIN_13
#define USER_BUTTON2_GPIO_Port    GPIOC

// Storage partition for CAN_Rx image (reserved Sector 11 on STM32F407, 128KB)
#define OTA_STORED_IMAGE_ADDRESS  ((uint32_t)0x080E0000U)
#define OTA_STORED_AREA_SIZE      (128U * 1024U)

typedef struct
{
  uint32_t magic;
  uint32_t size;
  uint32_t crc;
  uint32_t reserved;
} OtaStoredHeader_t;

#define OTA_STORED_HEADER_SIZE    ((uint32_t)sizeof(OtaStoredHeader_t))
#define OTA_MAX_IMAGE_SIZE        (OTA_STORED_AREA_SIZE - OTA_STORED_HEADER_SIZE)

// UART upload protocol: send 'U' then 4-byte LE size, then size bytes of data
static uint32_t stored_image_size = 0;
static uint32_t stored_image_crc = 0;

static uint8_t *GetStoredImage(void)
{
  return (uint8_t*)(OTA_STORED_IMAGE_ADDRESS + OTA_STORED_HEADER_SIZE);
}

// Check user button (assumes configured in MX_GPIO_Init as PA0)
static bool UserButtonPressed(void)
{
  GPIO_PinState b1 = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
  GPIO_PinState b2 = HAL_GPIO_ReadPin(USER_BUTTON2_GPIO_Port, USER_BUTTON2_Pin);
  // PA0 commonly active-high, PC13 commonly active-low.
  return ((b1 == USER_BUTTON_ACTIVE_STATE) || (b2 == GPIO_PIN_RESET));
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define STR_HELPER(x) #x
#define XSTR(x) STR_HELPER(x)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static CAN_TxHeaderTypeDef canTxHeader;
static uint8_t canTxData[8];
static uint32_t canTxMailbox;
static volatile uint8_t canTxAckPending;
static volatile uint8_t canTxErrorPending;
static volatile uint32_t canTxAckMailbox;
static volatile uint8_t canRxReplyWr;
static volatile uint8_t canRxReplyRd;
static CAN_RxHeaderTypeDef canRxReplyHeaderQ[OTA_REPLY_QUEUE_LEN];
static uint8_t canRxReplyDataQ[OTA_REPLY_QUEUE_LEN][8];

static uint8_t otaHeaderBuffer[OTA_UART_HEADER_SIZE];
static uint32_t otaHeaderIndex;
static uint8_t otaStreamActive;
static uint8_t otaSyncSent;
static uint8_t otaStartSent;
static uint8_t otaEndSent;
static uint32_t otaExpectedSize;
static uint32_t otaExpectedCrc;
static uint32_t otaReceivedSize;
static uint32_t otaRunningCrc;
static uint8_t otaChunkBuffer[OTA_DATA_PAYLOAD_SIZE];
static uint8_t otaChunkLength;
static uint16_t otaDataSeq;
static volatile uint8_t otaTransferAborted;
static uint8_t buttonLatched;

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
static void CAN_PrintTxStatus(void);
static void CAN_HandleTxComplete(uint32_t txMailbox);
static uint32_t OTA_Crc32Update(uint32_t runningCrc, uint8_t byteValue);
static void OTA_AppInit(void);
static void OTA_LoadStoredImageInfo(void);
static void OTA_PrintStoredImageStatus(void);
static void OTA_ProcessUartInput(void);
static void OTA_ResetSession(void);
static void OTA_ParseHeaderAndStart(void);
static void OTA_FlushDataChunkIfReady(void);
static void OTA_FinalizeIfComplete(void);
static void OTA_SendSyncFrame(void);
static void OTA_SendStartFrame(void);
static void OTA_SendEndFrame(void);
static uint8_t OTA_SendDataFrame(const uint8_t *payload, uint8_t payloadLen);
static void OTA_PollReplies(void);
static void OTA_ClearReplies(void);
static uint8_t OTA_TryPopReply(CAN_RxHeaderTypeDef *header, uint8_t data[8]);
static uint8_t OTA_WaitForAckNack(uint32_t timeoutMs, uint8_t *code, uint32_t *value);
static uint8_t OTA_SendFrameWithRetry(uint32_t stdId, const uint8_t *data, uint8_t dlc, uint8_t maxRetries, uint32_t timeoutMs, uint8_t ignoreProtocolNack, uint8_t *nackCode, uint32_t *nackValue);
static void OTA_SendStoredImageOverCan(void);
static uint8_t OTA_SendFrame(uint32_t stdId, const uint8_t *data, uint8_t dlc);

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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN_App_Init();
  Debug_PrintLine("CAN debug UART ready\r\n");
  Debug_PrintLine("CAN OTA TX node ready. NODE=" XSTR(CAN_NODE_ID) "\r\n");
  Debug_PrintLine("UART header format: [magic OTA1][size u32 LE][crc32 u32 LE]\r\n");

  OTA_AppInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    CAN_PrintTxStatus();
    OTA_PollReplies();
    OTA_FlushDataChunkIfReady();
    OTA_FinalizeIfComplete();
    OTA_ProcessUartInput();
    // If user button pressed, begin sending stored image (if present)
    if ((stored_image_size != 0U) && (otaStreamActive == 0U))
    {
      uint8_t pressed = (UserButtonPressed() != false) ? 1U : 0U;

      if ((pressed != 0U) && (buttonLatched == 0U))
      {
        char line[72];
        snprintf(line, sizeof(line), "Button detected PA0=%u PC13=%u\r\n",
                 (unsigned int)HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin),
                 (unsigned int)HAL_GPIO_ReadPin(USER_BUTTON2_GPIO_Port, USER_BUTTON2_Pin));
        Debug_PrintLine(line);
        OTA_SendStoredImageOverCan();
        buttonLatched = 1U;
        HAL_Delay(60U);
      }
      else if (pressed == 0U)
      {
        buttonLatched = 0U;
      }
    }
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

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LED_TX_Pin|LED_RX_Pin|LED_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USER_BUTTON2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BUTTON2_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_TX_Pin|LED_RX_Pin|LED_ERROR_Pin, GPIO_PIN_RESET);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void Debug_PrintLine(const char *line)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 100U);
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

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF | CAN_IT_ERROR) != HAL_OK)
  {
    Error_Handler();
  }

  canTxHeader.StdId = CAN_TX_STD_ID;
  canTxHeader.ExtId = 0x00;
  canTxHeader.IDE = CAN_ID_STD;
  canTxHeader.RTR = CAN_RTR_DATA;
  canTxHeader.DLC = 8;
  canTxHeader.TransmitGlobalTime = DISABLE;
}

static void CAN_PrintTxStatus(void)
{
  if (canTxAckPending != 0U)
  {
    char line[48];
    uint32_t txMailbox;

    __disable_irq();
    txMailbox = canTxAckMailbox;
    canTxAckPending = 0U;
    __enable_irq();

    snprintf(line, sizeof(line), "ACK received for TX mailbox %lu\r\n", (unsigned long)txMailbox);
    Debug_PrintLine(line);
  }

  if (canTxErrorPending != 0U)
  {
    __disable_irq();
    canTxErrorPending = 0U;
    __enable_irq();

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_ERROR_Pin);
    Debug_PrintLine("TX failed: no ACK or CAN error\r\n");
  }
}

static void CAN_HandleTxComplete(uint32_t txMailbox)
{
  canTxAckMailbox = txMailbox;
  canTxAckPending = 1U;
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_RX_Pin);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CAN_HandleTxComplete(0U);
  }
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CAN_HandleTxComplete(1U);
  }
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CAN_HandleTxComplete(2U);
  }
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    canTxErrorPending = 1U;
  }
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    canTxErrorPending = 1U;
  }
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    canTxErrorPending = 1U;
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    canTxErrorPending = 1U;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];

  if (hcan->Instance != CAN1)
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
  {
    return;
  }

  if ((header.StdId == OTA_CAN_ID_ACK) || (header.StdId == OTA_CAN_ID_NACK))
  {
    uint8_t wr = canRxReplyWr;
    uint8_t next = (uint8_t)((wr + 1U) % OTA_REPLY_QUEUE_LEN);

    if (next == canRxReplyRd)
    {
      canRxReplyRd = (uint8_t)((canRxReplyRd + 1U) % OTA_REPLY_QUEUE_LEN);
    }

    canRxReplyHeaderQ[wr] = header;
    memcpy(canRxReplyDataQ[wr], data, sizeof(canRxReplyDataQ[wr]));
    canRxReplyWr = next;
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_RX_Pin);
  }
}

static void OTA_ClearReplies(void)
{
  __disable_irq();
  canRxReplyRd = canRxReplyWr;
  __enable_irq();
}

static uint8_t OTA_TryPopReply(CAN_RxHeaderTypeDef *header, uint8_t data[8])
{
  uint8_t rd;

  __disable_irq();
  if (canRxReplyRd == canRxReplyWr)
  {
    __enable_irq();
    return 0U;
  }

  rd = canRxReplyRd;
  *header = canRxReplyHeaderQ[rd];
  memcpy(data, canRxReplyDataQ[rd], 8U);
  canRxReplyRd = (uint8_t)((rd + 1U) % OTA_REPLY_QUEUE_LEN);
  __enable_irq();
  return 1U;
}

static void OTA_AppInit(void)
{
  OTA_LoadStoredImageInfo();
  OTA_ResetSession();
}

static void OTA_LoadStoredImageInfo(void)
{
  const OtaStoredHeader_t *hdr = (const OtaStoredHeader_t *)OTA_STORED_IMAGE_ADDRESS;

  if ((hdr->magic == OTA_STORED_MAGIC) &&
      (hdr->size > 0U) &&
      (hdr->size <= OTA_MAX_IMAGE_SIZE))
  {
    char msg[64];
    stored_image_size = hdr->size;
    stored_image_crc = hdr->crc;
    Debug_PrintLine("Stored image header valid\r\n");
    snprintf(msg, sizeof(msg), "Stored size: %lu bytes\r\n", (unsigned long)stored_image_size);
    Debug_PrintLine(msg);
    Debug_PrintLine("Use button or send 'S' over UART to start OTA\r\n");
  }
  else
  {
    stored_image_size = 0U;
    stored_image_crc = 0U;
    Debug_PrintLine("No valid stored image\r\n");
  }
}

static void OTA_PrintStoredImageStatus(void)
{
  char msg[96];
  snprintf(msg, sizeof(msg), "Stored status: size=%lu crc=0x%08lX\r\n",
           (unsigned long)stored_image_size,
           (unsigned long)stored_image_crc);
  Debug_PrintLine(msg);
}

static void OTA_ResetSession(void)
{
  otaHeaderIndex = 0U;
  otaStreamActive = 0U;
  otaSyncSent = 0U;
  otaStartSent = 0U;
  otaEndSent = 0U;
  otaExpectedSize = 0U;
  otaExpectedCrc = 0U;
  otaReceivedSize = 0U;
  otaRunningCrc = 0xFFFFFFFFU;
  otaChunkLength = 0U;
  otaDataSeq = 0U;
  otaTransferAborted = 0U;
  buttonLatched = 0U;
}

static void OTA_SendSyncFrame(void)
{
  uint8_t payload[8] =
  {
    (uint8_t)OTA_SYNC_TOKEN_0,
    (uint8_t)OTA_SYNC_TOKEN_1,
    (uint8_t)OTA_SYNC_TOKEN_2,
    (uint8_t)OTA_SYNC_TOKEN_3,
    (uint8_t)OTA_SYNC_TOKEN_4,
    (uint8_t)OTA_SYNC_TOKEN_5,
    (uint8_t)OTA_SYNC_TOKEN_6,
    (uint8_t)OTA_SYNC_TOKEN_7
  };

  if (OTA_SendFrame(OTA_CAN_ID_SYNC, payload, 8U) != 0U)
  {
    otaSyncSent = 1U;
    Debug_PrintLine("OTA SYNC sent\r\n");
  }
}

static uint8_t OTA_SendFrame(uint32_t stdId, const uint8_t *data, uint8_t dlc)
{
  canTxHeader.StdId = stdId;
  canTxHeader.DLC = dlc;
  memset(canTxData, 0, sizeof(canTxData));
  memcpy(canTxData, data, dlc);

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
  {
    return 0U;
  }

  if (HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, canTxData, &canTxMailbox) == HAL_OK)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_TX_Pin);
    Debug_PrintCanFrame("TX", canTxHeader.StdId, canTxHeader.DLC, canTxData);
    return 1U;
  }

  return 0U;
}

static uint32_t OTA_Crc32Update(uint32_t runningCrc, uint8_t byteValue)
{
  uint32_t crc = runningCrc ^ byteValue;
  uint32_t bit;

  for (bit = 0U; bit < 8U; bit++)
  {
    uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
    crc = (crc >> 1U) ^ (0xEDB88320U & mask);
  }

  return crc;
}

static void OTA_SendStartFrame(void)
{
  uint8_t payload[8];

  payload[0] = (uint8_t)(otaExpectedSize >> 0U);
  payload[1] = (uint8_t)(otaExpectedSize >> 8U);
  payload[2] = (uint8_t)(otaExpectedSize >> 16U);
  payload[3] = (uint8_t)(otaExpectedSize >> 24U);
  payload[4] = (uint8_t)(otaExpectedCrc >> 0U);
  payload[5] = (uint8_t)(otaExpectedCrc >> 8U);
  payload[6] = (uint8_t)(otaExpectedCrc >> 16U);
  payload[7] = (uint8_t)(otaExpectedCrc >> 24U);

  if (OTA_SendFrame(OTA_CAN_ID_START, payload, 8U) != 0U)
  {
    otaStartSent = 1U;
    Debug_PrintLine("OTA START sent\r\n");
  }
}

static void OTA_SendEndFrame(void)
{
  uint8_t payload[8];

  payload[0] = (uint8_t)(OTA_FOOTER_MAGIC >> 0U);
  payload[1] = (uint8_t)(OTA_FOOTER_MAGIC >> 8U);
  payload[2] = (uint8_t)(OTA_FOOTER_MAGIC >> 16U);
  payload[3] = (uint8_t)(OTA_FOOTER_MAGIC >> 24U);
  payload[4] = (uint8_t)(otaExpectedSize >> 0U);
  payload[5] = (uint8_t)(otaExpectedSize >> 8U);
  payload[6] = (uint8_t)(otaExpectedSize >> 16U);
  payload[7] = (uint8_t)(otaExpectedSize >> 24U);

  if (OTA_SendFrame(OTA_CAN_ID_END, payload, 8U) != 0U)
  {
    otaEndSent = 1U;
    Debug_PrintLine("OTA FOOTER sent\r\n");
  }
}

static uint8_t OTA_SendDataFrame(const uint8_t *payload, uint8_t payloadLen)
{
  uint8_t frame[8];

  if ((payload == NULL) || (payloadLen == 0U) || (payloadLen > OTA_DATA_PAYLOAD_SIZE))
  {
    return 0U;
  }

  frame[0] = (uint8_t)(otaDataSeq >> 0U);
  frame[1] = (uint8_t)(otaDataSeq >> 8U);
  memset(&frame[2], 0, OTA_DATA_PAYLOAD_SIZE);
  memcpy(&frame[2], payload, payloadLen);

  if (OTA_SendFrame(OTA_CAN_ID_DATA, frame, (uint8_t)(payloadLen + OTA_DATA_SEQ_BYTES)) == 0U)
  {
    return 0U;
  }

  otaDataSeq++;
  return 1U;
}

static void OTA_ParseHeaderAndStart(void)
{
  uint32_t magic = ((uint32_t)otaHeaderBuffer[0] << 0U) |
                   ((uint32_t)otaHeaderBuffer[1] << 8U) |
                   ((uint32_t)otaHeaderBuffer[2] << 16U) |
                   ((uint32_t)otaHeaderBuffer[3] << 24U);

  if (magic != OTA_UART_MAGIC)
  {
    Debug_PrintLine("UART header magic invalid\r\n");
    otaHeaderIndex = 0U;
    return;
  }

  otaExpectedSize = ((uint32_t)otaHeaderBuffer[4] << 0U) |
                    ((uint32_t)otaHeaderBuffer[5] << 8U) |
                    ((uint32_t)otaHeaderBuffer[6] << 16U) |
                    ((uint32_t)otaHeaderBuffer[7] << 24U);
  otaExpectedCrc = ((uint32_t)otaHeaderBuffer[8] << 0U) |
                   ((uint32_t)otaHeaderBuffer[9] << 8U) |
                   ((uint32_t)otaHeaderBuffer[10] << 16U) |
                   ((uint32_t)otaHeaderBuffer[11] << 24U);

  if ((otaExpectedSize == 0U) || (otaExpectedSize > OTA_UART_MAX_IMAGE_SIZE))
  {
    Debug_PrintLine("UART image size invalid\r\n");
    OTA_ResetSession();
    return;
  }

  otaStreamActive = 1U;
  otaHeaderIndex = 0U;
  otaRunningCrc = 0xFFFFFFFFU;
  otaReceivedSize = 0U;
  otaChunkLength = 0U;
  otaSyncSent = 0U;
  otaStartSent = 0U;
  otaEndSent = 0U;
  Debug_PrintLine("UART OTA stream accepted\r\n");
  OTA_SendSyncFrame();
}

static void OTA_FlushDataChunkIfReady(void)
{
  if ((otaStreamActive == 0U) || (otaSyncSent == 0U) || (otaStartSent == 0U))
  {
    return;
  }

  if (otaChunkLength == OTA_DATA_PAYLOAD_SIZE)
  {
    if (OTA_SendDataFrame(otaChunkBuffer, otaChunkLength) != 0U)
    {
      otaChunkLength = 0U;
    }
  }
}

static void OTA_FinalizeIfComplete(void)
{
  uint32_t finalCrc;

  if ((otaStreamActive == 0U) || (otaSyncSent == 0U) || (otaStartSent == 0U))
  {
    return;
  }

  if (otaReceivedSize != otaExpectedSize)
  {
    return;
  }

  if (otaChunkLength != 0U)
  {
    if (OTA_SendDataFrame(otaChunkBuffer, otaChunkLength) != 0U)
    {
      otaChunkLength = 0U;
    }
    else
    {
      return;
    }
  }

  finalCrc = ~otaRunningCrc;
  if (finalCrc != otaExpectedCrc)
  {
    Debug_PrintLine("UART image CRC mismatch\r\n");
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
    OTA_ResetSession();
    return;
  }

  if (otaEndSent == 0U)
  {
    OTA_SendEndFrame();
    return;
  }

  Debug_PrintLine("UART OTA stream complete\r\n");
  OTA_ResetSession();
}

static void OTA_ProcessUartInput(void)
{
  uint8_t rxByte;

  if ((otaStreamActive != 0U) && (otaSyncSent == 0U))
  {
    OTA_SendSyncFrame();
    return;
  }

  if ((otaStreamActive != 0U) && (otaStartSent == 0U))
  {
    OTA_SendStartFrame();
    return;
  }

  if ((otaStreamActive != 0U) && (otaChunkLength == OTA_DATA_PAYLOAD_SIZE))
  {
    return;
  }

  if (HAL_UART_Receive(&huart2, &rxByte, 1U, 0U) != HAL_OK)
  {
    return;
  }

  if (otaStreamActive == 0U)
  {
    // Support UART upload command 'U' (0x55): U + size u32 LE + data[size]
    if (rxByte == 'U')
    {
      uint8_t sizeBuf[4];
      uint32_t imgSize = 0;
      // read 4-byte size
      if (HAL_UART_Receive(&huart2, sizeBuf, 4U, 1000U) != HAL_OK) return;
      imgSize = ((uint32_t)sizeBuf[0]) | ((uint32_t)sizeBuf[1] << 8) | ((uint32_t)sizeBuf[2] << 16) | ((uint32_t)sizeBuf[3] << 24);
      if ((imgSize == 0U) || (imgSize > OTA_MAX_IMAGE_SIZE))
      {
        Debug_PrintLine("Image size too large for stored area\r\n");
        return;
      }
      Debug_PrintLine("Receiving image over UART to store in flash...\r\n");
      if (FLASH_If_Erase(OTA_STORED_IMAGE_ADDRESS, OTA_STORED_AREA_SIZE) != 0)
      {
        Debug_PrintLine("Flash erase failed before upload\r\n");
        return;
      }
      // Handshake: tell host we are ready to receive payload after erase.
      Debug_PrintLine("READY\r\n");
      uint32_t remaining = imgSize;
      uint32_t writeAddr = OTA_STORED_IMAGE_ADDRESS + OTA_STORED_HEADER_SIZE;
      uint32_t runningCrc = 0xFFFFFFFFU;
      uint8_t tmpBuf[128];
      while (remaining)
      {
        uint32_t toRead = (remaining > sizeof(tmpBuf)) ? sizeof(tmpBuf) : remaining;
        if (HAL_UART_Receive(&huart2, tmpBuf, toRead, 5000U) != HAL_OK)
        {
          Debug_PrintLine("UART receive timeout during image upload\r\n");
          return;
        }
        if (FLASH_If_Write(writeAddr, tmpBuf, toRead) != 0)
        {
          Debug_PrintLine("Flash write failed during upload\r\n");
          return;
        }

        for (uint32_t i = 0U; i < toRead; i++)
        {
          runningCrc = OTA_Crc32Update(runningCrc, tmpBuf[i]);
        }

        // Per-chunk handshake ACK to avoid UART overruns while flash programming.
        Debug_PrintLine("K");

        writeAddr += toRead;
        remaining -= toRead;
      }

      OtaStoredHeader_t hdr;
      hdr.magic = OTA_STORED_MAGIC;
      hdr.size = imgSize;
      hdr.crc = ~runningCrc;
      hdr.reserved = 0xFFFFFFFFU;

      if (FLASH_If_Write(OTA_STORED_IMAGE_ADDRESS, (const uint8_t *)&hdr, sizeof(hdr)) != 0)
      {
        Debug_PrintLine("Header write failed after upload\r\n");
        return;
      }

      stored_image_size = imgSize;
      stored_image_crc = hdr.crc;
      Debug_PrintLine("Image stored in flash\r\n");
      Debug_PrintLine("UPLOAD_OK\r\n");
      Debug_PrintLine("Send 'S' over UART or press button to start OTA\r\n");
      return;
    }

    if (rxByte == 'S')
    {
      if (stored_image_size == 0U)
      {
        Debug_PrintLine("No stored image. Upload first.\r\n");
      }
      else
      {
        Debug_PrintLine("UART command S received. Starting OTA\r\n");
        OTA_SendStoredImageOverCan();
      }
      return;
    }

    if (rxByte == 'Q')
    {
      OTA_LoadStoredImageInfo();
      OTA_PrintStoredImageStatus();
      return;
    }

    otaHeaderBuffer[otaHeaderIndex++] = rxByte;
    if (otaHeaderIndex >= OTA_UART_HEADER_SIZE)
    {
      OTA_ParseHeaderAndStart();
    }
    return;
  }

  if (otaReceivedSize >= otaExpectedSize)
  {
    return;
  }

  otaRunningCrc = OTA_Crc32Update(otaRunningCrc, rxByte);
  otaChunkBuffer[otaChunkLength++] = rxByte;
  otaReceivedSize++;
}

static void OTA_PollReplies(void)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  char line[96];

  if (OTA_TryPopReply(&header, data) == 0U)
  {
    return;
  }

  snprintf(line, sizeof(line), "RX %s code=%u value=%lu\r\n",
           (header.StdId == OTA_CAN_ID_ACK) ? "ACK" : "NACK",
           (unsigned int)data[0],
           (unsigned long)(((uint32_t)data[1] << 0U) |
                           ((uint32_t)data[2] << 8U) |
                           ((uint32_t)data[3] << 16U) |
                           ((uint32_t)data[4] << 24U)));
  Debug_PrintLine(line);

  if (header.StdId == OTA_CAN_ID_NACK)
  {
    Debug_PrintLine("Receiver rejected frame. Resetting stream.\r\n");
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
    otaTransferAborted = 1U;
    OTA_ResetSession();
  }
}

static uint8_t OTA_WaitForAckNack(uint32_t timeoutMs, uint8_t *code, uint32_t *value)
{
  uint32_t startTick = HAL_GetTick();

  while ((HAL_GetTick() - startTick) < timeoutMs)
  {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (OTA_TryPopReply(&header, data) == 0U)
    {
      continue;
    }

    if (code != NULL)
    {
      *code = data[0];
    }

    if (value != NULL)
    {
      *value = ((uint32_t)data[1] << 0U) |
               ((uint32_t)data[2] << 8U) |
               ((uint32_t)data[3] << 16U) |
               ((uint32_t)data[4] << 24U);
    }

    if (header.StdId == OTA_CAN_ID_ACK)
    {
      return 1U;
    }

    if (header.StdId == OTA_CAN_ID_NACK)
    {
      return 2U;
    }
  }

  return 0U;
}

static uint8_t OTA_SendFrameWithRetry(uint32_t stdId, const uint8_t *data, uint8_t dlc, uint8_t maxRetries, uint32_t timeoutMs, uint8_t ignoreProtocolNack, uint8_t *nackCode, uint32_t *nackValue)
{
  uint8_t attempt;

  OTA_ClearReplies();

  for (attempt = 0U; attempt < maxRetries; attempt++)
  {
    uint8_t result;

    if (OTA_SendFrame(stdId, data, dlc) == 0U)
    {
      continue;
    }

    result = OTA_WaitForAckNack(timeoutMs, nackCode, nackValue);
    if (result == 1U)
    {
      return 1U;
    }

    if (result == 2U)
    {
      if ((ignoreProtocolNack != 0U) && (nackCode != NULL) && (*nackCode == 1U))
      {
        continue;
      }
      return 2U;
    }
  }

  return 0U;
}

static void OTA_SendStoredImageOverCan(void)
{
  uint8_t *img = GetStoredImage();
  uint32_t size = stored_image_size;
  uint32_t crc = stored_image_crc;
  uint32_t offset = 0U;
  uint16_t seq = 0U;
  uint8_t nackCode = 0U;
  uint32_t nackValue = 0U;
  uint8_t frame[8];

  uint8_t syncFrame[8] = {
      (uint8_t)OTA_SYNC_TOKEN_0, (uint8_t)OTA_SYNC_TOKEN_1,
      (uint8_t)OTA_SYNC_TOKEN_2, (uint8_t)OTA_SYNC_TOKEN_3,
      (uint8_t)OTA_SYNC_TOKEN_4, (uint8_t)OTA_SYNC_TOKEN_5,
      (uint8_t)OTA_SYNC_TOKEN_6, (uint8_t)OTA_SYNC_TOKEN_7};

  uint8_t startFrame[8];
  uint8_t endFrame[8];

  if (size == 0U)
  {
    Debug_PrintLine("No stored image to send\r\n");
    return;
  }

  Debug_PrintLine("User button pressed, sending stored image over CAN\r\n");
  otaTransferAborted = 0U;

  if (crc == 0U)
  {
    crc = 0xFFFFFFFFU;
    while (offset < size)
    {
      crc = OTA_Crc32Update(crc, img[offset]);
      offset++;
    }
    crc = ~crc;
  }

  startFrame[0] = (uint8_t)(size >> 0U);
  startFrame[1] = (uint8_t)(size >> 8U);
  startFrame[2] = (uint8_t)(size >> 16U);
  startFrame[3] = (uint8_t)(size >> 24U);
  startFrame[4] = (uint8_t)(crc >> 0U);
  startFrame[5] = (uint8_t)(crc >> 8U);
  startFrame[6] = (uint8_t)(crc >> 16U);
  startFrame[7] = (uint8_t)(crc >> 24U);

  if (OTA_SendFrameWithRetry(OTA_CAN_ID_SYNC, syncFrame, 8U, OTA_SEND_FRAME_RETRY_MAX, OTA_SEND_SYNC_TIMEOUT_MS, 0U, &nackCode, &nackValue) != 1U)
  {
    Debug_PrintLine("SYNC failed\r\n");
    return;
  }

  if (OTA_SendFrameWithRetry(OTA_CAN_ID_START, startFrame, 8U, OTA_SEND_FRAME_RETRY_MAX, OTA_SEND_START_TIMEOUT_MS, 1U, &nackCode, &nackValue) != 1U)
  {
    Debug_PrintLine("START failed\r\n");
    return;
  }

  offset = 0U;
  seq = 0U;
  while (offset < size)
  {
    uint32_t remain = size - offset;
    uint8_t payloadLen = (remain >= OTA_DATA_PAYLOAD_SIZE) ? OTA_DATA_PAYLOAD_SIZE : (uint8_t)remain;
    uint8_t status;

    frame[0] = (uint8_t)(seq >> 0U);
    frame[1] = (uint8_t)(seq >> 8U);
    memset(&frame[2], 0, OTA_DATA_PAYLOAD_SIZE);
    memcpy(&frame[2], &img[offset], payloadLen);

    status = OTA_SendFrameWithRetry(OTA_CAN_ID_DATA, frame, (uint8_t)(payloadLen + OTA_DATA_SEQ_BYTES), OTA_SEND_FRAME_RETRY_MAX, OTA_SEND_DATA_TIMEOUT_MS, 0U, &nackCode, &nackValue);
    if (status == 1U)
    {
      offset += payloadLen;
      seq++;
      continue;
    }

    if ((status == 2U) && (nackCode == 1U))
    {
      uint16_t expectedSeq = (uint16_t)(nackValue & 0xFFFFU);
      if (expectedSeq > seq)
      {
        seq = expectedSeq;
        offset = ((uint32_t)seq) * OTA_DATA_PAYLOAD_SIZE;
        if (offset > size)
        {
          offset = size;
        }
        continue;
      }
    }

    Debug_PrintLine("DATA transfer failed\r\n");
    return;
  }

  endFrame[0] = (uint8_t)(OTA_FOOTER_MAGIC >> 0U);
  endFrame[1] = (uint8_t)(OTA_FOOTER_MAGIC >> 8U);
  endFrame[2] = (uint8_t)(OTA_FOOTER_MAGIC >> 16U);
  endFrame[3] = (uint8_t)(OTA_FOOTER_MAGIC >> 24U);
  endFrame[4] = (uint8_t)(size >> 0U);
  endFrame[5] = (uint8_t)(size >> 8U);
  endFrame[6] = (uint8_t)(size >> 16U);
  endFrame[7] = (uint8_t)(size >> 24U);

  if (OTA_SendFrameWithRetry(OTA_CAN_ID_END, endFrame, 8U, OTA_SEND_FRAME_RETRY_MAX, OTA_SEND_END_TIMEOUT_MS, 0U, &nackCode, &nackValue) != 1U)
  {
    Debug_PrintLine("END failed\r\n");
    return;
  }

  Debug_PrintLine("Stored image OTA transfer complete\r\n");
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
