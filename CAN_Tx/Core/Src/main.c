/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include <stdlib.h>
#include <ctype.h>
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
#define OTA_CAN_ID_QUERY          0x326U
#define OTA_CAN_ID_STATUS         0x327U

#define OTA_QUERY_ACTIVE_STATUS   0U
#define OTA_QUERY_SLOT_A_META     1U
#define OTA_QUERY_SLOT_B_META     2U

#define OTA_SLOT_A                0U
#define OTA_SLOT_B                1U

#define OTA_SYNC_TOKEN_0          'O'
#define OTA_SYNC_TOKEN_1          'T'
#define OTA_SYNC_TOKEN_2          'A'
#define OTA_SYNC_TOKEN_3          'S'
#define OTA_SYNC_TOKEN_4          'Y'
#define OTA_SYNC_TOKEN_5          'N'
#define OTA_SYNC_TOKEN_6          'C'
#define OTA_SYNC_TOKEN_7          '!'

#define OTA_FOOTER_MAGIC          0x454E4431U
#define OTA_STORED_MAGIC          0x3241544FU  // "OTA2" in ASCII means stored image header is valid

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
#define SIM7670_APN               "airtelgprs.com"
#define SIM7670_FIXED_BAUD        115200U
#define SIM7670_DEFAULT_URL_A      "https://can-rx-firmware-storage.s3.eu-north-1.amazonaws.com/CAN_Rx.bin"
#define SIM7670_DEFAULT_URL_B      "https://can-firmware-storage.s3.eu-north-1.amazonaws.com/CAN_Rx_slot_b.bin"
#define SIM7670_DEFAULT_MANIFEST_URL "https://can-firmware-storage.s3.eu-north-1.amazonaws.com/manifest.txt"
#define SIM7670_CMD_TIMEOUT_MS    5000U
#define SIM7670_HTTP_TIMEOUT_MS   60000U
#define SIM7670_HTTP_CHUNK_SIZE   256U
#define SIM7670_MANIFEST_MAX_BYTES 1024U
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

typedef struct
{
  uint32_t activeSlot;
  uint32_t bootSlot;
  uint32_t confirmedSlot;
  uint32_t updateInProgress;
  uint32_t bootAttemptCount;
  uint32_t maxBootAttempts;
  uint32_t lastErrorCode;
  uint32_t metadataVersion;
} OtaRxStatus_t;

typedef struct
{
  uint32_t size;
  uint32_t crc;
} OtaSlotMeta_t;

// Return the address where the stored OTA image payload begins.
// The firmware stores a small header at the start of the OTA area,
// so the actual image data starts immediately after that header.
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
UART_HandleTypeDef huart3;

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
static char simApn[64] = SIM7670_APN;
static char simFirmwareUrl[192] = SIM7670_DEFAULT_URL_B;
static char simManifestUrl[192] = SIM7670_DEFAULT_MANIFEST_URL;
static uint32_t simManifestVersion;
static uint32_t simManifestSize;
static uint32_t simManifestCrc; 
static uint8_t simManifestValid;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
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
static uint8_t OTA_WaitForReplyId(uint32_t stdId, uint32_t timeoutMs, CAN_RxHeaderTypeDef *header, uint8_t data[8]);
static uint8_t OTA_WaitForAckNack(uint32_t timeoutMs, uint8_t *code, uint32_t *value);
static uint8_t OTA_SendFrameWithRetry(uint32_t stdId, const uint8_t *data, uint8_t dlc, uint8_t maxRetries, uint32_t timeoutMs, uint8_t ignoreProtocolNack, uint8_t *nackCode, uint32_t *nackValue);
static uint8_t OTA_PrepareActiveSlotImageForTransfer(void);
static void OTA_SendStoredImageOverCan(void);
static uint8_t OTA_SendFrame(uint32_t stdId, const uint8_t *data, uint8_t dlc);
static uint8_t OTA_QueryRxStatus(OtaRxStatus_t *status);
static uint8_t OTA_QueryRxSlotMeta(uint8_t slot, OtaSlotMeta_t *meta);
static int32_t SIM7670_ReadLine(char *line, uint32_t lineSize, uint32_t timeoutMs);
static void SIM7670_DrainRx(uint32_t durationMs);
static int32_t UART2_ReadLine(char *line, uint32_t lineSize, uint32_t timeoutMs);
static uint8_t SIM7670_SendCmdExpectOk(const char *cmd, uint32_t timeoutMs);
static uint8_t SIM7670_WaitForHttpAction(uint16_t *httpCode, uint32_t *contentLen, uint32_t timeoutMs);
static uint8_t SIM7670_ParseHttpReadHeader(const char *line, uint32_t *dataLen);
static uint8_t SIM7670_ReadHttpChunk(uint32_t offset, uint32_t req, uint8_t *dataBuf, uint32_t *got);
static uint8_t SIM7670_HttpGetToBuffer(const char *url, uint8_t *buffer, uint32_t bufferMax, uint32_t *outLen);
static uint8_t SIM7670_ParseManifest(const uint8_t *buffer, uint32_t length, uint32_t *version, uint32_t *size, uint32_t *crc, char *urlOut, uint32_t urlOutSize);
static uint8_t SIM7670_FetchManifest(void);
static void SIM7670_PrintManifestStatus(void);
static uint8_t SIM7670_DownloadToStoredPartition(const char *url, uint32_t expectedSize, uint32_t expectedCrc);
static uint8_t SIM7670_SetHttpCid(void);
static uint8_t SIM7670_CheckCpinReady(void);
static void SIM7670_PrepareDataSession(void);
static uint8_t SIM7670_HttpInitWithRecovery(void);
static uint8_t SIM7670_CheckReady(void);
static uint8_t SIM7670_TryBaud(uint32_t baud);
static uint8_t SIM7670_WaitForRawToken(const char *token, uint32_t timeoutMs);
static uint8_t UrlLooksHttp(const char *url);
static void NormalizeGithubUrlInPlace(char *url, uint32_t urlSize);
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN_App_Init();
  Debug_PrintLine("CAN debug UART ready\r\n");
  Debug_PrintLine("CAN OTA TX node ready. NODE=" XSTR(CAN_NODE_ID) "\r\n");
  Debug_PrintLine("FW debug: UART_CMD_DIAG_V1\r\n");
  Debug_PrintLine("UART header format: [magic OTA1][size u32 LE][crc32 u32 LE]\r\n");
  Debug_PrintLine("UART cmd 'G': download firmware via SIM7670 and store\r\n");
  Debug_PrintLine("UART cmd 'R': check SIM7670 AT response\r\n");
  Debug_PrintLine("UART cmd 'W': set firmware URL\r\n");
  Debug_PrintLine("UART cmd 'T': set manifest URL\r\n");
  Debug_PrintLine("UART cmd 'M': fetch manifest (version,size,crc,url)\r\n");
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

    if (otaStreamActive == 0U)
    {
      uint8_t pressed = (UserButtonPressed() != false) ? 1U : 0U;

      if ((pressed != 0U) && (buttonLatched == 0U))
      {
        char line[72];
        uint8_t status;

        snprintf(line, sizeof(line), "Button detected PA0=%u PC13=%u\r\n",
                 (unsigned int)HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin),
                 (unsigned int)HAL_GPIO_ReadPin(USER_BUTTON2_GPIO_Port, USER_BUTTON2_Pin));
        Debug_PrintLine(line);
        buttonLatched = 1U;
        status = OTA_PrepareActiveSlotImageForTransfer();
        if (status == 1U)
        {
          Debug_PrintLine("Prepared image for OTA. Sending over CAN...\r\n");
          OTA_SendStoredImageOverCan();
        }
        else if (status == 2U)
        {
          Debug_PrintLine("RX already has the selected image. OTA transfer skipped.\r\n");
        }
        else
        {
          Debug_PrintLine("Active-slot-aware OTA preparation failed\r\n");
        }
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
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

  canTxHeader.StdId = CAN_TX_STD_ID;// standard id
  canTxHeader.ExtId = 0x00;// no ext id is used
  canTxHeader.IDE = CAN_ID_STD; // std id 
  canTxHeader.RTR = CAN_RTR_DATA; // data frame
  canTxHeader.DLC = 8; // data length code
  canTxHeader.TransmitGlobalTime = DISABLE;

  /* CAN Tx Header fields:
   * StdId: Standard (11-bit) Identifier used when IDE == CAN_ID_STD.
   * ExtId: Extended (29-bit) Identifier used when IDE == CAN_ID_EXT.
   * IDE: Identifier type (CAN_ID_STD or CAN_ID_EXT) selects StdId or ExtId.
   * RTR: Remote Transmission Request (CAN_RTR_DATA for data frame,
   *      CAN_RTR_REMOTE for remote frame requesting data).
   * DLC: Data Length Code - number of data bytes in the CAN frame (0..8 for classic CAN).
   * TransmitGlobalTime: If ENABLE, requests controller to append/get global time (for some hardware);
   *    typically DISABLE for normal transmissions.
   */
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


/*
* CAN_HandleTxComplete()
* @param : txMailbox - the mailbox number (0, 1, or 2) that completed transmission
* @brief : This function is called when a CAN transmission completes successfully. 
* It sets the canTxAckMailbox variable to the completed mailbox number, sets the canTxAckPending flag to indicate that an acknowledgment has been received, and toggles the LED_RX_Pin to provide visual feedback of the successful transmission.

*/
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

  if ((header.StdId == OTA_CAN_ID_ACK) || (header.StdId == OTA_CAN_ID_NACK) || (header.StdId == OTA_CAN_ID_STATUS))
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

static uint8_t OTA_WaitForReplyId(uint32_t stdId, uint32_t timeoutMs, CAN_RxHeaderTypeDef *header, uint8_t data[8])
{
  uint32_t startTick = HAL_GetTick();

  while ((HAL_GetTick() - startTick) < timeoutMs)
  {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (OTA_TryPopReply(&rxHeader, rxData) == 0U)
    {
      continue;
    }

    if (rxHeader.StdId == stdId)
    {
      if (header != NULL)
      {
        *header = rxHeader;
      }

      memcpy(data, rxData, 8U);
      return 1U;
    }
  }

  return 0U;
}

/*
* OTA_AppInit()
* @param : None
* @brief : This function initializes the OTA application by loading any stored image information and resetting the OTA session state.
*/
static void OTA_AppInit(void)
{
  OTA_LoadStoredImageInfo();
  OTA_ResetSession();
}

/*
* OTA_LoadStoredImageInfo()
* @param : None
* @brief : This function loads the stored image information from the designated memory location.
* It checks the validity of the stored image header by verifying the magic number, size, and CRC.
* If valid, it updates the stored image size and CRC variables and prints relevant information to the debug UART. 
* If invalid, it resets the stored image size and CRC to zero and indicates that no valid stored image is present.
*/
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

/*
* OTA_QueryRxStatus()
* @param : status - Pointer to the structure to store the received status information
* @brief : This function queries the receive status of the OTA module. It sends a query command over CAN and waits for a reply with the status information.
* If successful, it populates the provided status structure with the received data and returns 1. If unsuccessful, it returns 0.
*/
static uint8_t OTA_QueryRxStatus(OtaRxStatus_t *status)
{
  uint8_t cmd[1] = { OTA_QUERY_ACTIVE_STATUS };
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];

  if (status == NULL)
  {
    return 0U;
  }

  OTA_ClearReplies();
  if (OTA_SendFrame(OTA_CAN_ID_QUERY, cmd, 1U) == 0U)
  {
    return 0U;
  }

  if (OTA_WaitForReplyId(OTA_CAN_ID_STATUS, 1000U, &header, data) == 0U)
  {
    return 0U;
  }

  status->activeSlot = data[0];
  status->bootSlot = data[1];
  status->confirmedSlot = data[2];
  status->updateInProgress = data[3];
  status->bootAttemptCount = data[4];
  status->maxBootAttempts = data[5];
  status->lastErrorCode = data[6];
  status->metadataVersion = data[7];
  return 1U;
}


/*
* OTA_QueryRxSlotMeta()
* @param : slot - The slot number to query (0 or 1)
* @brief : This function queries the metadata of a specific slot in the OTA module. It sends a query command over CAN with the specified slot number and waits for a reply containing the metadata information.
* If successful, it populates the provided meta structure with the received data (size and CRC) and returns 1. If unsuccessful, it returns 0.
*/
static uint8_t OTA_QueryRxSlotMeta(uint8_t slot, OtaSlotMeta_t *meta)
{
  uint8_t cmd[1] = { slot };
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];

  if (meta == NULL)
  {
    return 0U;
  }

  OTA_ClearReplies();
  if (OTA_SendFrame(OTA_CAN_ID_QUERY, cmd, 1U) == 0U)
  {
    return 0U;
  }

  if (OTA_WaitForReplyId(OTA_CAN_ID_STATUS, 1000U, &header, data) == 0U)
  {
    return 0U;
  }

  meta->size = ((uint32_t)data[0] << 0U) |
               ((uint32_t)data[1] << 8U) |
               ((uint32_t)data[2] << 16U) |
               ((uint32_t)data[3] << 24U);
  meta->crc = ((uint32_t)data[4] << 0U) |
              ((uint32_t)data[5] << 8U) |
              ((uint32_t)data[6] << 16U) |
              ((uint32_t)data[7] << 24U);
  return 1U;
}
/*
* OTA_PrintStoredImageStatus()
* @param : None
* @brief : This function prints the status of the stored image, including its size and CRC, to the debug UART. It formats the information into a string and sends it for debugging purposes.

*/
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

/*
* OTA_SendSyncFrame()
* @param : None
* @brief : This function sends a synchronization frame over CAN to initiate the OTA process. 
* It constructs a payload containing the synchronization tokens and sends it using the OTA_SendFrame function.
*/
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
/*
* OTA_SendFrame()
* @param : stdId - The standard identifier for the CAN frame
* @param : data - Pointer to the data payload to be sent
* @param : dlc - The data length code (number of bytes in the payload)
* @return : 1 if the frame was sent successfully, 0 otherwise
* @brief : This function sends a CAN frame with the specified standard identifier, data payload, and data length code.
* It first checks if there are free mailboxes available for transmission.
* If a mailbox is available, it prepares the CAN header and data, and attempts to send the frame using HAL_CAN_AddTxMessage.
* If the transmission is successful, it toggles the LED_TX_Pin to indicate a successful transmission and prints the transmitted frame details to the debug UART.
* If the transmission fails or no mailboxes are available, it returns 0 to indicate failure.
*/
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
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_TX_Pin);//toggle TX LED to indicate a successful transmission
    Debug_PrintCanFrame("TX", canTxHeader.StdId, canTxHeader.DLC, canTxData);
    return 1U;
  }

  return 0U;
}

/*  
* OTA_Crc32Update()
* @param : runningCrc - The current CRC value to be updated fetched from the binary data of the OTA image
* @param : byteValue - The byte value to be processed
* @return : The updated CRC value
* @brief : This function updates the CRC value with the given byte value using the CRC32 algorithm.
*/
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

/*
* OTA_SendStartFrame()
* @param : None 
* @brief : This function sends a start frame over CAN to indicate the beginning of the OTA process.
* It constructs a payload containing the expected size and CRC of the OTA image, and sends it using the OTA_SendFrame function.
*/
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
/*
* OTA_SendEndFrame()
* @param : None
* @brief : This function sends an end frame over CAN to indicate the completion of the OTA process.
* It constructs a payload containing the OTA footer magic number and the expected size of the OTA image, and sends it using the OTA_SendFrame function.
* If the frame is sent successfully, it sets the otaEndSent flag to indicate that the end frame has been sent and prints a debug message.
*/
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

/*
* OTA_SendDataFrame()
* @param : payload - Pointer to the data payload to be sent
* @param : payloadLen - The length of the data payload
* @return : 1 if the data frame was sent successfully, 0 otherwise
* @brief : This function sends a data frame over CAN containing a portion of the OTA image data.
* It constructs a frame with the sequence number and the provided payload, and sends it using the
* OTA_SendFrame function. The sequence number is incremented after each successful transmission.
*/
static uint8_t OTA_SendDataFrame(const uint8_t *payload, uint8_t payloadLen)
{
  uint8_t frame[8]; 
  // OTA data frame is 8 bytes: 2 bytes for sequence number and 6 bytes for payload 
  //payload: means the actual data being sent in the OTA data frame
  //sequence number: means the order of the data frame in the OTA transfer, used to ensure that the receiver can reconstruct the original data correctly

  if ((payload == NULL) || (payloadLen == 0U) || (payloadLen > OTA_DATA_PAYLOAD_SIZE)) // Validate payload length and not null pointer and not zero length
  {
    return 0U;
  }

  frame[0] = (uint8_t)(otaDataSeq >> 0U);
  frame[1] = (uint8_t)(otaDataSeq >> 8U);
  memset(&frame[2], 0, OTA_DATA_PAYLOAD_SIZE);
  memcpy(&frame[2], payload, payloadLen);

  if (OTA_SendFrame(OTA_CAN_ID_DATA, frame, (uint8_t)(payloadLen + OTA_DATA_SEQ_BYTES)) == 0U) // return 0 if sending the frame failed
  {
    return 0U;
  }

  otaDataSeq++; //at extreme last the seqnum == total number of data frames sent, so that the receiver can know how many frames to expect
  return 1U;
}

/*
* OTA_ParseHeaderAndStart()
* @param : None
* @brief : This function parses the OTA header and starts the OTA transfer.
* It checks the magic number, expected size, and expected CRC from the received header.
* If the header is valid, it initializes the OTA transfer state and sends a synchronization frame to the receiver to indicate the start of the OTA process. If the header is invalid, it resets the OTA session.
*/
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
//after parsing the header, set the otaStreamActive flag to indicate that the OTA transfer is active,
// reset the header index, running CRC, received size, chunk length, and flags for sync, start, and end frames.

  otaStreamActive = 1U;
  otaHeaderIndex = 0U;
  otaRunningCrc = 0xFFFFFFFFU;
  otaReceivedSize = 0U;
  otaChunkLength = 0U;
  otaSyncSent = 0U;
  otaStartSent = 0U;
  otaEndSent = 0U;
  Debug_PrintLine("UART OTA stream accepted\r\n");
  OTA_SendSyncFrame(); //after parsing the header, send a sync frame to the receiver to indicate that the OTA transfer is starting
}

/*
 * OTA_FlushDataChunkIfReady()
 * @param None
 * @brief Flushes the data chunk if it is ready to be sent.
 * 
 */
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
      otaChunkLength = 0U; //reset the chunk length to 0 after successfully sending the data frame
    }
  }
}

/*
 * OTA_FinalizeIfComplete()
 * @param None
 * @brief Finalizes the OTA transfer if it is complete.
 * by checking if the received size matches the expected size, and if the running CRC matches the expected CRC.
 * If both conditions are met, it sends an end frame to indicate the completion of the OTA transfer. 
 * If the CRC does not match, it resets the OTA session and indicates an error.
 * 
 */
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

/*
 * OTA_ProcessUartInput()
 * @param None
 * @brief Processes incoming UART data for OTA updates.
 * then checks if the OTA stream is active and whether the sync and start frames have been sent.
 * If the OTA stream is active and the sync frame has not been sent, it sends the sync frame. 
 * If the OTA stream is active and the start frame has not been sent, it sends the start frame.
 * If the OTA stream is active and the chunk length is equal to the maximum payload size, it returns without processing further data.
 * If there is data available on the UART, it reads a byte and processes it.
 * 
 */
static void OTA_ProcessUartInput(void)
{
  uint8_t rxByte;
  char rxMsg[32];

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

  if (HAL_UART_Receive(&huart2, &rxByte, 1U, 20U) != HAL_OK)
  {
    return;
  }

  snprintf(rxMsg, sizeof(rxMsg), "UART RX 0x%02X\r\n", (unsigned int)rxByte);
  Debug_PrintLine(rxMsg);

  if ((rxByte >= 'a') && (rxByte <= 'z'))
  {
    rxByte = (uint8_t)(rxByte - ('a' - 'A'));
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
        uint32_t toRead = (remaining > sizeof(tmpBuf)) ? sizeof(tmpBuf) : remaining ;
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
      uint8_t status;

      buttonLatched = 1U;
      status = OTA_PrepareActiveSlotImageForTransfer();
      if (status == 1U)
      {
        
        Debug_PrintLine("Prepared image for OTA. Sending over CAN...\r\n");
        OTA_SendStoredImageOverCan();
      }
      else if (status == 2U)
      {
        Debug_PrintLine("RX already has the selected image. OTA transfer skipped.\r\n");
      }
      else
      {
        Debug_PrintLine("Active-slot-aware OTA preparation failed\r\n"); // it knows that the stored image is invalid or not present, so it cannot prepare for OTA transfer
      }
      return;
    }

    if (rxByte == 'Q')
    {
      OTA_LoadStoredImageInfo();// reload the stored image info from flash, in case it was updated by a previous upload
      OTA_PrintStoredImageStatus();// print the stored image status to the debug UART like size and CRC
      return;
    }

    if (rxByte == 'R')
    {
      uint32_t t0 = HAL_GetTick();
      uint32_t dt;
      uint8_t ready;

      Debug_PrintLine("Checking SIM7670...\r\n");
      ready = SIM7670_CheckReady();
      dt = HAL_GetTick() - t0;

      if (ready != 0U)
      {
        char line[64];
        snprintf(line, sizeof(line), "SIM7670 ready (%lu ms)\r\n", (unsigned long)dt);
        Debug_PrintLine(line);
      }
      else
      {
        char line[80];
        snprintf(line, sizeof(line), "SIM7670 not responding (%lu ms)\r\n", (unsigned long)dt);
        Debug_PrintLine(line);
      }
      return;
    }

    if (rxByte == 'W')
    {
      char input[192];
      char line[240];
      Debug_PrintLine("Enter firmware URL then newline:\r\n");
      if (UART2_ReadLine(input, sizeof(input), 30000U) > 0)
      {
        if (UrlLooksHttp(input) != 0U)
        {
          strncpy(simFirmwareUrl, input, sizeof(simFirmwareUrl) - 1U);
          simFirmwareUrl[sizeof(simFirmwareUrl) - 1U] = '\0';
          NormalizeGithubUrlInPlace(simFirmwareUrl, sizeof(simFirmwareUrl));
          snprintf(line, sizeof(line), "Firmware URL stored: %s\r\n", simFirmwareUrl);
          Debug_PrintLine(line);
        }
        else
        {
          Debug_PrintLine("Invalid URL, keeping previous firmware URL\r\n");
        }
      }
      else
      {
        Debug_PrintLine("URL input timeout\r\n");
        snprintf(line, sizeof(line), "Using stored firmware URL: %s\r\n", simFirmwareUrl);
        Debug_PrintLine(line);
      }
      return;
    }

    if (rxByte == 'T')
    {
      char input[192];
      char line[240];
      Debug_PrintLine("Enter manifest URL then newline:\r\n");
      if (UART2_ReadLine(input, sizeof(input), 30000U) > 0)
      {
        if (UrlLooksHttp(input) != 0U)
        {
          strncpy(simManifestUrl, input, sizeof(simManifestUrl) - 1U);
          simManifestUrl[sizeof(simManifestUrl) - 1U] = '\0';
          NormalizeGithubUrlInPlace(simManifestUrl, sizeof(simManifestUrl));
          snprintf(line, sizeof(line), "Manifest URL stored: %s\r\n", simManifestUrl);
          Debug_PrintLine(line);
        }
        else
        {
          Debug_PrintLine("Invalid URL, keeping previous manifest URL\r\n");
        }
      }
      else
      {
        Debug_PrintLine("Manifest URL input timeout\r\n");
        snprintf(line, sizeof(line), "Using stored manifest URL: %s\r\n", simManifestUrl);
        Debug_PrintLine(line);
      }
      return;
    }

    if (rxByte == 'M')
    {
      if (SIM7670_FetchManifest() != 0U)
      {
        SIM7670_PrintManifestStatus();
      }
      else
      {
        Debug_PrintLine("Manifest fetch failed\r\n");
      }
      return;
    }

    if (rxByte == 'G')
    {
      uint8_t status;

      status = OTA_PrepareActiveSlotImageForTransfer();
      if (status == 1U)
      {
        Debug_PrintLine("Prepared image for OTA. Sending over CAN...\r\n");
        OTA_SendStoredImageOverCan();
      }
      else if (status == 2U)
      {
        Debug_PrintLine("RX already has the selected image. OTA transfer skipped.\r\n");
      }
      else
      {
        Debug_PrintLine("Active-slot-aware OTA preparation failed\r\n");
      }
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

/*
 * OTA_PollReplies()
 * @param None
 * @brief Polls for replies from the OTA receiver over CAN.
 * It checks for ACK or NACK messages and handles them accordingly.
 * If a NACK is received, it resets the OTA session and indicates an error.
 */
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
/*
 * OTA_WaitForAckNack()
 * @param timeoutMs - The timeout in milliseconds to wait for an ACK or NACK response
 * @param code - Pointer to store the received code (if any)
 * @param value - Pointer to store the received value (if any)
 * @return 1 if ACK received, 2 if NACK received, 0 if timeout occurred
 * @brief Waits for an ACK or NACK response from the OTA receiver over CAN.
 * It polls for replies until either an ACK or NACK is received or the timeout expires.
 * If a reply is received, it extracts the code and value from the data and returns the appropriate status.
 */
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
/*
 * OTA_SendFrameWithRetry()
 * @param stdId - The standard identifier for the CAN frame
 * @param data - Pointer to the data payload to be sent
 * @param dlc - The data length code (number of bytes in the payload)
 * @param maxRetries - The maximum number of retries for sending the frame
 * @param timeoutMs - The timeout in milliseconds to wait for an ACK or NACK response
 * @param ignoreProtocolNack - Flag to indicate whether to ignore protocol NACKs (1 to ignore, 0 to not ignore)
 * @param nackCode - Pointer to store the received NACK code (if any)
 * @param nackValue - Pointer to store the received NACK value (if any)
 * @return 1 if ACK received, 2 if NACK received, 0 if all retries failed
 * @brief Sends a CAN frame with retries and waits for an ACK or NACK response.
 * It attempts to send the frame up to maxRetries times, waiting for an ACK or NACK after each attempt.
 * If an ACK is received, it returns 1. If a NACK is received, it returns 2. If all retries fail, it returns 0.
 */
static uint8_t OTA_SendFrameWithRetry(uint32_t stdId, const uint8_t *data, uint8_t dlc, uint8_t maxRetries, uint32_t timeoutMs, uint8_t ignoreProtocolNack, uint8_t *nackCode, uint32_t *nackValue)
{
  uint8_t attempt;
  char line[96];

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
      if ((nackCode != NULL) && (nackValue != NULL))
      {
        snprintf(line, sizeof(line), "CAN NACK id=0x%03lX code=%u value=%lu\r\n",
                 (unsigned long)stdId,
                 (unsigned int)(*nackCode),
                 (unsigned long)(*nackValue));
        Debug_PrintLine(line);
      }

      if ((ignoreProtocolNack != 0U) && (nackCode != NULL) && (*nackCode == 1U))
      {
        continue;
      }
      return 2U;
    }

    snprintf(line, sizeof(line), "CAN ACK timeout id=0x%03lX attempt=%u\r\n",
             (unsigned long)stdId,
             (unsigned int)(attempt + 1U));
    Debug_PrintLine(line);
  }

  return 0U;
}
/*
 * OTA_SendStoredImageOverCan()
 * @param None
 * @brief Sends the stored OTA image over CAN to the receiver.
 * It retrieves the stored image, calculates its CRC if not already done, and sends it in chunks over CAN.
 * The function handles synchronization, start, data, and end frames, and manages retries and acknowledgments.
 */
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

  Debug_PrintLine("Starting stored image OTA over CAN\r\n");
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

/*
 * OTA_PrepareActiveSlotImageForTransfer()
 * @param None
 * @return 1 if preparation successful, 0 if failed
 * @brief Prepares the active slot image for OTA transfer.
 * It checks the status of the active slot and prepares the appropriate firmware URL for download.
 * like it check then download Slot A or Slot B firmware based on the active slot of the receiver.
 * It also checks if the remote already has a matching image and skips transfer if so.
 */
static uint8_t OTA_PrepareActiveSlotImageForTransfer(void)
{
  char line[240];
  uint8_t useManifest = 1U;
  OtaRxStatus_t rxStatus;
  OtaSlotMeta_t slotAMeta;
  OtaSlotMeta_t slotBMeta;
  uint8_t remoteMatchSlot = 0xFFU;
  uint8_t queryOk = 0U;
  const char *selectedFirmwareUrl = simFirmwareUrl;

  if (UrlLooksHttp(simFirmwareUrl) == 0U)
  {
    strncpy(simFirmwareUrl, SIM7670_DEFAULT_URL_B, sizeof(simFirmwareUrl) - 1U);
    simFirmwareUrl[sizeof(simFirmwareUrl) - 1U] = '\0';
    Debug_PrintLine("Stored firmware URL invalid, restored default\r\n");
  }

  NormalizeGithubUrlInPlace(simManifestUrl, sizeof(simManifestUrl));
  NormalizeGithubUrlInPlace(simFirmwareUrl, sizeof(simFirmwareUrl));

  if (strncmp(simManifestUrl, "https://", 8U) == 0)
  {
    useManifest = 0U;
    simManifestValid = 0U;
    Debug_PrintLine("Manifest URL is HTTPS; skipping manifest fetch in HTTP-only mode\r\n");
  }

  if ((useManifest != 0U) && (SIM7670_FetchManifest() != 0U))
  {
    SIM7670_PrintManifestStatus();
  }
  else if (useManifest != 0U)
  {
    Debug_PrintLine("Manifest not available, using direct firmware URL\r\n");
  }

  if (OTA_QueryRxStatus(&rxStatus) == 0U)
  {
    Debug_PrintLine("Failed to query RX active slot\r\n");
    return 0U;
  }

  snprintf(line, sizeof(line), "Remote active slot: %lu boot=%lu confirmed=%lu\r\n",
           (unsigned long)rxStatus.activeSlot,
           (unsigned long)rxStatus.bootSlot,
           (unsigned long)rxStatus.confirmedSlot);
  Debug_PrintLine(line);

  if (rxStatus.activeSlot == OTA_SLOT_A) // if slot A active -> download to slot B
  {
    selectedFirmwareUrl = SIM7670_DEFAULT_URL_B;
  }
  else if (rxStatus.activeSlot == OTA_SLOT_B) // if slot B active -> download to slot A
  {
    selectedFirmwareUrl = SIM7670_DEFAULT_URL_A;
  }
  else
  {
    Debug_PrintLine("Unknown RX active slot\r\n");
    return 0U;
  }

  strncpy(simFirmwareUrl, selectedFirmwareUrl, sizeof(simFirmwareUrl) - 1U);
  simFirmwareUrl[sizeof(simFirmwareUrl) - 1U] = '\0';

  snprintf(line, sizeof(line), "Selected download URL for inactive slot: %s\r\n", simFirmwareUrl);//prints string stored in simFirmwareUrl to the debug UART
  Debug_PrintLine(line);

  if (simManifestValid != 0U) 
  {
    if ((OTA_QueryRxSlotMeta(OTA_QUERY_SLOT_A_META, &slotAMeta) != 0U) &&
        (OTA_QueryRxSlotMeta(OTA_QUERY_SLOT_B_META, &slotBMeta) != 0U))
    {
      queryOk = 1U;

      if ((slotAMeta.size == simManifestSize) && (slotAMeta.crc == simManifestCrc))
      {
        remoteMatchSlot = OTA_QUERY_SLOT_A_META;
      }
      else if ((slotBMeta.size == simManifestSize) && (slotBMeta.crc == simManifestCrc))
      {
        remoteMatchSlot = OTA_QUERY_SLOT_B_META;
      }

      if (remoteMatchSlot != 0xFFU) //0xFFU indicates that no matching slot was found
      {
        snprintf(line, sizeof(line), "Remote already has matching image in Slot %c. Skipping transfer.\r\n",
                 (remoteMatchSlot == OTA_QUERY_SLOT_A_META) ? 'A' : 'B');
        Debug_PrintLine(line);
        return 2U;
      }

      snprintf(line, sizeof(line), "Remote slots differ. Will transfer to inactive slot %c.\r\n",
               (rxStatus.activeSlot == OTA_SLOT_A) ? 'B' : 'A');
      Debug_PrintLine(line);
    }
  }

  if ((simManifestValid != 0U) && (queryOk == 0U))
  {
    Debug_PrintLine("Remote slot metadata query failed, proceeding with download\r\n");
  }

  if (SIM7670_DownloadToStoredPartition(simFirmwareUrl,
                                         (simManifestValid != 0U) ? simManifestSize : 0U,//if the manifest is valid, use the size from the manifest; otherwise, use 0
                                         (simManifestValid != 0U) ? simManifestCrc : 0U) != 0U) //if the simManifestValid is true, use the CRC from the manifest; otherwise, use 0
  {
    return 1U; // indicates that the download was successful and the image is ready for OTA transfer 
  }

  Debug_PrintLine("Active-slot-aware download failed\r\n");
  return 0U;
}

static int32_t SIM7670_ReadLine(char *line, uint32_t lineSize, uint32_t timeoutMs)
{
  uint32_t startTick = HAL_GetTick();
  uint32_t index = 0U;
  uint8_t ch;

  if ((line == NULL) || (lineSize < 2U))
  {
    return -1;
  }

  while ((HAL_GetTick() - startTick) < timeoutMs)
  {
    if (HAL_UART_Receive(&huart3, &ch, 1U, 10U) != HAL_OK)
    {
      continue;
    }

    if (ch == '\r')
    {
      continue;
    }

    if (ch == '\n')
    {
      if (index == 0U)
      {
        continue;
      }

      line[index] = '\0';
      return (int32_t)index;
    }

    if (index < (lineSize - 1U))
    {
      line[index++] = (char)ch;
    }
  }

  return -1;
}

static void SIM7670_DrainRx(uint32_t durationMs)
{
  uint32_t startTick = HAL_GetTick();
  uint8_t ch;

  while ((HAL_GetTick() - startTick) < durationMs)
  {
    if (HAL_UART_Receive(&huart3, &ch, 1U, 10U) != HAL_OK)
    {
      continue;
    }
  }
}

static int32_t UART2_ReadLine(char *line, uint32_t lineSize, uint32_t timeoutMs)
{
  uint32_t startTick = HAL_GetTick();
  uint32_t index = 0U;
  uint8_t ch;

  if ((line == NULL) || (lineSize < 2U))
  {
    return -1;
  }

  while ((HAL_GetTick() - startTick) < timeoutMs)
  {
    if (HAL_UART_Receive(&huart2, &ch, 1U, 10U) != HAL_OK)
    {
      continue;
    }

    if ((ch == '\r') || (ch == '\n'))
    {
      if (index == 0U)
      {
        continue;
      }
      line[index] = '\0';
      Debug_PrintLine("\r\n");
      return (int32_t)index;
    }

    if (index < (lineSize - 1U))
    {
      line[index++] = (char)ch;
      (void)HAL_UART_Transmit(&huart2, &ch, 1U, 20U);
    }
  }

  return -1;
}

static uint8_t SIM7670_SendCmdExpectOk(const char *cmd, uint32_t timeoutMs)
{
  char tx[192];
  char line[192];
  char dbg[224];
  uint32_t deadline;

  if (cmd == NULL)
  {
    return 0U;
  }

  (void)snprintf(tx, sizeof(tx), "%s\r\n", cmd);
  (void)snprintf(dbg, sizeof(dbg), "SIM> %s\r\n", cmd);
  Debug_PrintLine(dbg);

  /* Drop any stale modem bytes before sending next command. */
  SIM7670_DrainRx(80U);

  if (HAL_UART_Transmit(&huart3, (uint8_t *)tx, (uint16_t)strlen(tx), 500U) != HAL_OK)
  {
    Debug_PrintLine("SIM TX failed\r\n");
    return 0U;
  }

  deadline = HAL_GetTick() + timeoutMs;
  while ((int32_t)(deadline - HAL_GetTick()) > 0)
  {
    int32_t len = SIM7670_ReadLine(line, sizeof(line), 350U);
    if (len <= 0)
    {
      continue;
    }

    (void)snprintf(dbg, sizeof(dbg), "SIM< %s\r\n", line);
    Debug_PrintLine(dbg);

    if (strcmp(line, "OK") == 0)
    {
      return 1U;
    }

    /* Some firmware variants first echo command text (e.g., "AT") and then send OK later. */
    if (strcmp(line, cmd) == 0)
    {
      deadline = HAL_GetTick() + timeoutMs;
      continue;
    }

    if (strstr(line, "ERROR") != NULL)
    {
      return 0U;
    }
  }

  Debug_PrintLine("SIM cmd timeout\r\n");

  return 0U;
}

static uint8_t SIM7670_SetHttpCid(void)
{
  if (SIM7670_SendCmdExpectOk("AT+HTTPPARA=\"CID\",1", SIM7670_CMD_TIMEOUT_MS) != 0U)
  {
    return 1U;
  }

  if (SIM7670_SendCmdExpectOk("AT+HTTPPARA=\"CID\",\"1\"", SIM7670_CMD_TIMEOUT_MS) != 0U)
  {
    return 1U;
  }

  Debug_PrintLine("HTTP CID=1 failed, trying CID=0\r\n");
  if (SIM7670_SendCmdExpectOk("AT+HTTPPARA=\"CID\",0", SIM7670_CMD_TIMEOUT_MS) != 0U)
  {
    return 1U;
  }

  if (SIM7670_SendCmdExpectOk("AT+HTTPPARA=\"CID\",\"0\"", SIM7670_CMD_TIMEOUT_MS) != 0U)
  {
    return 1U;
  }

  Debug_PrintLine("HTTP CID unsupported on this FW, continuing without explicit CID\r\n");

  return 1U;
}

static void SIM7670_PrepareDataSession(void)
{
  /* Best-effort prep across SIM76xx firmware variants. Commands may return ERROR on some builds. */
  (void)SIM7670_SendCmdExpectOk("AT+NETCLOSE", SIM7670_HTTP_TIMEOUT_MS);
  (void)SIM7670_SendCmdExpectOk("AT+CGATT=1", SIM7670_HTTP_TIMEOUT_MS);
  (void)SIM7670_SendCmdExpectOk("AT+NETOPEN", SIM7670_HTTP_TIMEOUT_MS);
  (void)SIM7670_SendCmdExpectOk("AT+NETOPEN?", SIM7670_CMD_TIMEOUT_MS);
}

static uint8_t SIM7670_HttpInitWithRecovery(void)
{
  if (SIM7670_SendCmdExpectOk("AT+HTTPINIT", SIM7670_CMD_TIMEOUT_MS) != 0U)
  {
    return 1U;
  }

  Debug_PrintLine("HTTPINIT timeout, trying NET recovery\r\n");
  (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
  (void)SIM7670_SendCmdExpectOk("AT+NETCLOSE", SIM7670_HTTP_TIMEOUT_MS);
  HAL_Delay(300U);
  (void)SIM7670_SendCmdExpectOk("AT+CGATT=1", SIM7670_HTTP_TIMEOUT_MS);
  (void)SIM7670_SendCmdExpectOk("AT+NETOPEN", SIM7670_HTTP_TIMEOUT_MS);

  return SIM7670_SendCmdExpectOk("AT+HTTPINIT", SIM7670_HTTP_TIMEOUT_MS);
}

static uint8_t SIM7670_CheckCpinReady(void)
{
  char tx[24];
  char line[192];
  char dbg[224];
  uint32_t startTick = HAL_GetTick();

  (void)snprintf(tx, sizeof(tx), "AT+CPIN?\r\n");
  Debug_PrintLine("SIM> AT+CPIN?\r\n");

  if (HAL_UART_Transmit(&huart3, (uint8_t *)tx, (uint16_t)strlen(tx), 500U) != HAL_OK)
  {
    Debug_PrintLine("SIM TX failed\r\n");
    return 0U;
  }

  while ((HAL_GetTick() - startTick) < SIM7670_CMD_TIMEOUT_MS)
  {
    int32_t len = SIM7670_ReadLine(line, sizeof(line), 700U);
    if (len <= 0)
    {
      continue;
    }

    (void)snprintf(dbg, sizeof(dbg), "SIM< %s\r\n", line);
    Debug_PrintLine(dbg);

    if (strstr(line, "+CPIN: READY") != NULL)
    {
      return 1U;
    }

    if ((strstr(line, "+CPIN: SIM PIN") != NULL) ||
        (strstr(line, "+CPIN: NOT INSERTED") != NULL) ||
        (strstr(line, "ERROR") != NULL))
    {
      return 0U;
    }
  }

  Debug_PrintLine("SIM CPIN check timeout\r\n");
  return 0U;
}

static uint8_t SIM7670_WaitForHttpAction(uint16_t *httpCode, uint32_t *contentLen, uint32_t timeoutMs)
{
  char line[192];
  uint32_t startTick = HAL_GetTick();

  while ((HAL_GetTick() - startTick) < timeoutMs)
  {
    int32_t len = SIM7670_ReadLine(line, sizeof(line), timeoutMs);
    unsigned int method = 0U;
    unsigned int status = 0U;
    unsigned long length = 0UL;

    if (len <= 0)
    {
      continue;
    }

    if (sscanf(line, "+HTTPACTION: %u,%u,%lu", &method, &status, &length) == 3)
    {
      if (httpCode != NULL)
      {
        *httpCode = (uint16_t)status;
      }

      if (contentLen != NULL)
      {
        *contentLen = (uint32_t)length;
      }

      return 1U;
    }
  }

  return 0U;
}

static uint8_t SIM7670_ParseHttpReadHeader(const char *line, uint32_t *dataLen)
{
  const char *p;
  const char *end;
  const char *start;
  unsigned long parsed;

  if ((line == NULL) || (dataLen == NULL))
  {
    return 0U;
  }

  if (strstr(line, "+HTTPREAD") == NULL)
  {
    return 0U;
  }

  end = line + strlen(line);
  p = end;
  while ((p > line) && !isdigit((unsigned char)p[-1]))
  {
    p--;
  }

  start = p;
  while ((start > line) && isdigit((unsigned char)start[-1]))
  {
    start--;
  }

  if (start == p)
  {
    return 0U;
  }

  parsed = strtoul(start, NULL, 10);
  *dataLen = (uint32_t)parsed;
  return 1U;
}

static uint8_t SIM7670_ReadHttpChunk(uint32_t offset, uint32_t req, uint8_t *dataBuf, uint32_t *got)
{
  char cmd[64];
  char line[192];
  uint32_t lineDeadline = HAL_GetTick() + SIM7670_CMD_TIMEOUT_MS;
  uint32_t payloadLen = 0U;

  if ((dataBuf == NULL) || (got == NULL) || (req == 0U))
  {
    return 0U;
  }

  snprintf(cmd, sizeof(cmd), "AT+HTTPREAD=%lu,%lu\r\n", (unsigned long)offset, (unsigned long)req);
  if (HAL_UART_Transmit(&huart3, (uint8_t *)cmd, (uint16_t)strlen(cmd), 500U) != HAL_OK)
  {
    return 0U;
  }

  while ((int32_t)(lineDeadline - HAL_GetTick()) > 0)
  {
    int32_t len = SIM7670_ReadLine(line, sizeof(line), 500U);
    if (len <= 0)
    {
      continue;
    }

    if (SIM7670_ParseHttpReadHeader(line, &payloadLen) != 0U)
    {
      break;
    }

    if (strstr(line, "ERROR") != NULL)
    {
      return 0U;
    }
  }

  if ((payloadLen == 0U) || (payloadLen > req))
  {
    return 0U;
  }

  if (HAL_UART_Receive(&huart3, dataBuf, (uint16_t)payloadLen, SIM7670_CMD_TIMEOUT_MS) != HAL_OK)
  {
    return 0U;
  }

  /* Drain trailing blank/OK lines if present in this modem FW variant. */
  (void)SIM7670_ReadLine(line, sizeof(line), 300U);
  (void)SIM7670_ReadLine(line, sizeof(line), 300U);

  *got = payloadLen;
  return 1U;
}

static uint8_t SIM7670_HttpGetToBuffer(const char *url, uint8_t *buffer, uint32_t bufferMax, uint32_t *outLen)
{
  char cmd[256];
  uint16_t httpCode = 0U;
  uint32_t contentLen = 0U;
  uint32_t offset = 0U;

  if ((url == NULL) || (buffer == NULL) || (outLen == NULL) || (bufferMax == 0U))
  {
    return 0U;
  }

  if (SIM7670_CheckReady() == 0U)//
  {
    return 0U; //not ready 
  }

  (void)snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", simApn);
  if (SIM7670_SendCmdExpectOk(cmd, SIM7670_CMD_TIMEOUT_MS) == 0U)
  {
    return 0U;
  }

  SIM7670_PrepareDataSession();

  (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);

  if (SIM7670_HttpInitWithRecovery() == 0U)
  {
    return 0U;
  }

  (void)SIM7670_SetHttpCid();

  (void)snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
  if (SIM7670_SendCmdExpectOk(cmd, SIM7670_CMD_TIMEOUT_MS) == 0U)
  {
    (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
    return 0U;
  }

  if (SIM7670_SendCmdExpectOk("AT+HTTPACTION=0", SIM7670_CMD_TIMEOUT_MS) == 0U)
  {
    (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
    return 0U;
  }

  if (SIM7670_WaitForHttpAction(&httpCode, &contentLen, SIM7670_HTTP_TIMEOUT_MS) == 0U)
  {
    (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
    return 0U;
  }

  if ((httpCode != 200U) || (contentLen == 0U) || (contentLen > bufferMax))
  {
    (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
    return 0U;
  }

  while (offset < contentLen)
  {
    uint32_t req = contentLen - offset;
    uint32_t got = 0U;

    if (req > SIM7670_HTTP_CHUNK_SIZE)
    {
      req = SIM7670_HTTP_CHUNK_SIZE;
    }

    if (SIM7670_ReadHttpChunk(offset, req, &buffer[offset], &got) == 0U)
    {
      (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
      return 0U;
    }

    offset += got;
  }

  (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
  *outLen = contentLen;
  return 1U;
}

static uint8_t SIM7670_ParseManifest(const uint8_t *buffer, uint32_t length, uint32_t *version, uint32_t *size, uint32_t *crc, char *urlOut, uint32_t urlOutSize)
{
  char text[SIM7670_MANIFEST_MAX_BYTES + 1U];
  char *p;
  uint8_t foundSize = 0U;
  uint8_t foundCrc = 0U;
  uint8_t foundUrl = 0U;

  if ((buffer == NULL) || (length == 0U) || (length > SIM7670_MANIFEST_MAX_BYTES) ||
      (size == NULL) || (crc == NULL))
  {
    return 0U;
  }

  memcpy(text, buffer, length);
  text[length] = '\0';

  p = strstr(text, "version");
  if ((p != NULL) && (version != NULL))
  {
    while ((*p != '\0') && (*p != ':') && (*p != '='))
    {
      p++;
    }
    if ((*p == ':') || (*p == '='))
    {
      p++;
      *version = (uint32_t)strtoul(p, NULL, 0);
    }
  }

  p = strstr(text, "size");
  if (p != NULL)
  {
    while ((*p != '\0') && (*p != ':') && (*p != '='))
    {
      p++;
    }
    if ((*p == ':') || (*p == '='))
    {
      p++;
      *size = (uint32_t)strtoul(p, NULL, 0);
      foundSize = 1U;
    }
  }

  p = strstr(text, "crc");
  if (p != NULL)
  {
    while ((*p != '\0') && (*p != ':') && (*p != '='))
    {
      p++;
    }
    if ((*p == ':') || (*p == '='))
    {
      p++;
      while ((*p == ' ') || (*p == '"'))
      {
        p++;
      }
      *crc = (uint32_t)strtoul(p, NULL, 0);
      foundCrc = 1U;
    }
  }

  p = strstr(text, "url");
  if ((p != NULL) && (urlOut != NULL) && (urlOutSize >= 8U))
  {
    char *q;
    while ((*p != '\0') && (*p != ':') && (*p != '='))
    {
      p++;
    }
    if ((*p == ':') || (*p == '='))
    {
      p++;
      while ((*p == ' ') || (*p == '"'))
      {
        p++;
      }
      q = p;
      while ((*q != '\0') && (*q != '"') && (*q != '\r') && (*q != '\n'))
      {
        q++;
      }
      if (q > p)
      {
        uint32_t len = (uint32_t)(q - p);
        if (len >= urlOutSize)
        {
          len = urlOutSize - 1U;
        }
        memcpy(urlOut, p, len);
        urlOut[len] = '\0';
        foundUrl = 1U;
      }
    }
  }

  if ((urlOut != NULL) && (urlOutSize >= 1U) && (foundUrl == 0U))
  {
    urlOut[0] = '\0';
  }

  return (uint8_t)((foundSize != 0U) && (foundCrc != 0U));
}

static uint8_t SIM7670_FetchManifest(void)
{
  uint8_t manifestBuf[SIM7670_MANIFEST_MAX_BYTES];
  uint32_t manifestLen = 0U;
  uint32_t version = 0U;
  uint32_t size = 0U;
  uint32_t crc = 0U;
  char url[192] = {0};

  if (SIM7670_HttpGetToBuffer(simManifestUrl, manifestBuf, sizeof(manifestBuf), &manifestLen) == 0U)
  {
    simManifestValid = 0U;
    return 0U;
  }

  if (SIM7670_ParseManifest(manifestBuf, manifestLen, &version, &size, &crc, url, sizeof(url)) == 0U)
  {
    simManifestValid = 0U;
    return 0U;
  }

  simManifestVersion = version;
  simManifestSize = size;
  simManifestCrc = crc;
  if (url[0] != '\0')
  {
    strncpy(simFirmwareUrl, url, sizeof(simFirmwareUrl) - 1U);
    simFirmwareUrl[sizeof(simFirmwareUrl) - 1U] = '\0';
  }
  simManifestValid = 1U;
  return 1U;
}

static void SIM7670_PrintManifestStatus(void)
{
  char line[256];
  if (simManifestValid == 0U)
  {
    Debug_PrintLine("Manifest: invalid\r\n");
    return;
  }

  snprintf(line, sizeof(line), "Manifest v=%lu size=%lu crc=0x%08lX\r\n",
           (unsigned long)simManifestVersion,
           (unsigned long)simManifestSize,
           (unsigned long)simManifestCrc);
  Debug_PrintLine(line);

  if (simFirmwareUrl[0] != '\0')
  {
    snprintf(line, sizeof(line), "Manifest URL: %s\r\n", simFirmwareUrl);
    Debug_PrintLine(line);
  }
}

static uint8_t SIM7670_CheckReady(void)
{
  static const char atPingCrlf[] = "AT\r\n";
  char line[80];
  uint32_t attempt;

  if (SIM7670_TryBaud(SIM7670_FIXED_BAUD) != 0U)
  {
    return 1U;//if the SIM responds at the fixed baud rate, return 1 (ready)
  }

  Debug_PrintLine("SIM fixed-baud recovery path...\r\n");
  huart3.Init.BaudRate = SIM7670_FIXED_BAUD;
  (void)HAL_UART_DeInit(&huart3);
  (void)HAL_UART_Init(&huart3);
  HAL_Delay(120U);
  SIM7670_DrainRx(120U);

  for (attempt = 0U; attempt < 5U; attempt++)
  {
    if (HAL_UART_Transmit(&huart3, (uint8_t *)atPingCrlf, (uint16_t)(sizeof(atPingCrlf) - 1U), 250U) == HAL_OK)
    {
      if (SIM7670_WaitForRawToken("OK", 1400U) != 0U)
      {
        (void)SIM7670_SendCmdExpectOk("ATE0", 1200U);
        Debug_PrintLine("SIM recovered at fixed baud\r\n");
        return 1U;
      }
    }

    HAL_Delay(150U);
  }

  snprintf(line, sizeof(line), "SIM no response at fixed baud %lu\r\n", (unsigned long)SIM7670_FIXED_BAUD);
  Debug_PrintLine(line);

  return 0U;
}

static uint8_t SIM7670_WaitForRawToken(const char *token, uint32_t timeoutMs)
{
  uint32_t startTick = HAL_GetTick();
  uint8_t ch;
  char window[8] = {0};
  uint32_t i;
  uint32_t tokenLen;

  if (token == NULL)
  {
    return 0U;
  }

  tokenLen = strlen(token);
  if ((tokenLen == 0U) || (tokenLen >= sizeof(window)))
  {
    return 0U;
  }

  while ((HAL_GetTick() - startTick) < timeoutMs)
  {
    if (HAL_UART_Receive(&huart3, &ch, 1U, 20U) != HAL_OK)
    {
      continue;
    }

    for (i = 0U; i < (sizeof(window) - 1U); i++)
    {
      window[i] = window[i + 1U];
    }
    window[sizeof(window) - 1U] = (char)ch;

    if (strstr(window, token) != NULL)
    {
      return 1U;
    }
  }

  return 0U;
}

static uint8_t UrlLooksHttp(const char *url)
{
  if (url == NULL)
  {
    return 0U;
  }

  if ((strncmp(url, "http://", 7U) == 0) || (strncmp(url, "https://", 8U) == 0))
  {
    return 1U;
  }

  return 0U;
}

static void NormalizeGithubUrlInPlace(char *url, uint32_t urlSize)
{
  const char *githubPrefix = "https://github.com/";
  const char *blobTag = "/blob/";
  const char *repoStart;
  const char *blobPos;
  const char *afterBlob;
  char out[192];

  if ((url == NULL) || (urlSize < 2U))
  {
    return;
  }

  if (strncmp(url, githubPrefix, strlen(githubPrefix)) == 0)
  {
    repoStart = url + strlen(githubPrefix);
    blobPos = strstr(repoStart, blobTag);
    if (blobPos == NULL)
    {
      return;
    }

    afterBlob = blobPos + strlen(blobTag);
    (void)snprintf(out, sizeof(out), "https://raw.githubusercontent.com/%.*s/%s",
                   (int)(blobPos - repoStart), repoStart, afterBlob);

    strncpy(url, out, urlSize - 1U);
    url[urlSize - 1U] = '\0';
    return;
  }
}

static uint8_t SIM7670_TryBaud(uint32_t baud)
{
  static const char atPing[] = "AT\r";
  char line[64];
  uint32_t attempt;

  huart3.Init.BaudRate = baud;
  if (HAL_UART_DeInit(&huart3) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    return 0U;
  }

  snprintf(line, sizeof(line), "SIM probe baud %lu...\r\n", (unsigned long)baud);
  Debug_PrintLine(line);

  HAL_Delay(80U);
  SIM7670_DrainRx(80U);

  for (attempt = 0U; attempt < 3U; attempt++)
  {
    SIM7670_DrainRx(40U);
    if (HAL_UART_Transmit(&huart3, (uint8_t *)atPing, (uint16_t)(sizeof(atPing) - 1U), 200U) == HAL_OK)
    {
      if (SIM7670_WaitForRawToken("OK", 1200U) != 0U)
      {
        (void)SIM7670_SendCmdExpectOk("ATE0", 1200U);
        snprintf(line, sizeof(line), "SIM active baud %lu\r\n", (unsigned long)baud);
        Debug_PrintLine(line);
        return 1U;
      }
    }

    if (SIM7670_SendCmdExpectOk("AT", 1800U) != 0U)
    {
      (void)SIM7670_SendCmdExpectOk("ATE0", 1200U);
      snprintf(line, sizeof(line), "SIM active baud %lu\r\n", (unsigned long)baud);
      Debug_PrintLine(line);
      return 1U;
    }

    HAL_Delay(120U);
  }

  return 0U;
}
/**
* SIM7670_DownloadToStoredPartition()
* @param url: The URL to download the firmware image from.
* @param expectedSize: The expected size of the firmware image. If 0, size check is skipped.
* @param expectedCrc: The expected CRC32 of the firmware image. If 0, CRC check is skipped.
* @return: 1 if the download and verification were successful, 0 otherwise.
* this fun does every thing needed to download a firmware image from a given URL(function: SIM7670_PrepareDataSession) and store it in the designated flash partition(function: SIM7670_EraseFlash ). 
* It handles HTTP communication(function: SIM7670_HttpInitWithRecovery ), flash erasure(function: SIM7670_EraseFlash ), writing, and CRC verification. 
* 
*/
static uint8_t SIM7670_DownloadToStoredPartition(const char *url, uint32_t expectedSize, uint32_t expectedCrc)
{
  char cmd[256];
  uint16_t httpCode = 0U;
  uint32_t contentLen = 0U;
  uint32_t offset = 0U;
  uint32_t writeAddress = OTA_STORED_IMAGE_ADDRESS + OTA_STORED_HEADER_SIZE; // writeAddress:The firmware image will be written starting from address 0x08041000 in flash memory of the microcontroller. 
  //The first 0x1000 bytes (4KB) of the OTA_STORED_IMAGE_ADDRESS are reserved for the OTA header, which contains metadata about the firmware image, such as its size, version, and CRC. 
  ///The actual firmware image is written after this header.
  uint32_t runningCrc = 0xFFFFFFFFU;

  if ((url == NULL) || (url[0] == '\0'))
  {
    Debug_PrintLine("SIM URL empty\r\n");
    return 0U;
  } 

  if (SIM7670_CheckReady() == 0U)
  {
    Debug_PrintLine("SIM AT init failed\r\n");
    return 0U;
  }

  if (SIM7670_CheckCpinReady() == 0U)
  {
    Debug_PrintLine("SIM not ready (CPIN)\r\n");
    return 0U;
  }

  (void)snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", simApn);
  if (SIM7670_SendCmdExpectOk(cmd, SIM7670_CMD_TIMEOUT_MS) == 0U) 
  {
    Debug_PrintLine("SIM APN setup failed\r\n");
    return 0U;
  }

  SIM7670_PrepareDataSession(); 

  (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);

  if (SIM7670_HttpInitWithRecovery() == 0U)
  {
    Debug_PrintLine("SIM HTTPINIT failed\r\n");
    return 0U;
  }

  (void)SIM7670_SetHttpCid(); 

  (void)snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
  if (SIM7670_SendCmdExpectOk(cmd, SIM7670_CMD_TIMEOUT_MS) == 0U)
  {
    Debug_PrintLine("SIM HTTP URL set failed\r\n");
    (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
    return 0U;
  }

  if (SIM7670_SendCmdExpectOk("AT+HTTPACTION=0", SIM7670_CMD_TIMEOUT_MS) == 0U)
  {
    Debug_PrintLine("SIM HTTPACTION failed\r\n");
    (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
    return 0U;
  }

  if (SIM7670_WaitForHttpAction(&httpCode, &contentLen, SIM7670_HTTP_TIMEOUT_MS) == 0U)
  {
    Debug_PrintLine("SIM HTTPACTION timeout\r\n");
    (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
    return 0U;
  }

  if ((httpCode != 200U) || (contentLen == 0U) || (contentLen > OTA_MAX_IMAGE_SIZE))
  {
    char failLine[96];
    snprintf(failLine, sizeof(failLine), "HTTP status=%u len=%lu invalid\r\n",
             (unsigned int)httpCode,
             (unsigned long)contentLen);
    Debug_PrintLine(failLine);
    (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
    return 0U;
  }

  if (FLASH_If_Erase(OTA_STORED_IMAGE_ADDRESS, OTA_STORED_AREA_SIZE) != 0)
  {
    Debug_PrintLine("Flash erase failed before SIM download\r\n");
    (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
    return 0U;
  }

  while (offset < contentLen)
  {
    uint32_t req = contentLen - offset;
    uint32_t got;
    uint32_t i;
    uint8_t dataBuf[SIM7670_HTTP_CHUNK_SIZE];

    if (req > SIM7670_HTTP_CHUNK_SIZE)
    {
      req = SIM7670_HTTP_CHUNK_SIZE;
    }

    if (SIM7670_ReadHttpChunk(offset, req, dataBuf, &got) == 0U)
    {
      Debug_PrintLine("HTTPREAD failed\r\n");
      (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
      return 0U;
    }

    if (FLASH_If_Write(writeAddress, dataBuf, got) != 0)
    {
      Debug_PrintLine("Flash write failed during SIM download\r\n");
      (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
      return 0U;
    }

    for (i = 0U; i < got; i++)
    {
      runningCrc = OTA_Crc32Update(runningCrc, dataBuf[i]);
    }

    writeAddress += got;
    offset += got;

    if ((offset % (8U * 1024U)) == 0U)
    {
      char progressLine[72];
      snprintf(progressLine, sizeof(progressLine), "SIM download %lu/%lu\r\n",
               (unsigned long)offset,
               (unsigned long)contentLen);
      Debug_PrintLine(progressLine);
    }
  }

  if ((expectedSize != 0U) && (contentLen != expectedSize))
  {
    Debug_PrintLine("Manifest size mismatch\r\n");
    (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
    return 0U;
  }

  if ((expectedCrc != 0U) && ((~runningCrc) != expectedCrc))
  {
    Debug_PrintLine("Manifest CRC mismatch\r\n");
    (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
    return 0U;
  }

  {
    OtaStoredHeader_t hdr;
    hdr.magic = OTA_STORED_MAGIC;
    hdr.size = contentLen;
    hdr.crc = ~runningCrc;
    hdr.reserved = 0xFFFFFFFFU;

    if (FLASH_If_Write(OTA_STORED_IMAGE_ADDRESS, (const uint8_t *)&hdr, sizeof(hdr)) != 0)
    {
      Debug_PrintLine("Header write failed after SIM download\r\n");
      (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
      return 0U;
    }

    stored_image_size = hdr.size;
    stored_image_crc = hdr.crc;
  }

  (void)SIM7670_SendCmdExpectOk("AT+HTTPTERM", SIM7670_CMD_TIMEOUT_MS);
  Debug_PrintLine("SIM download stored successfully\r\n");
  OTA_PrintStoredImageStatus();
  return 1U;
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
