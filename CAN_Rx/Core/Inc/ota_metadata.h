#ifndef OTA_METADATA_H
#define OTA_METADATA_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define OTA_METADATA_MAGIC            0x4F54414DU  /* 'OTAM' */
#define OTA_METADATA_FORMAT_VERSION   1U

#define OTA_SLOT_A                    0U
#define OTA_SLOT_B                    1U

#define OTA_IMAGE_STATE_INVALID       0U
#define OTA_IMAGE_STATE_VALID         1U
#define OTA_IMAGE_STATE_PENDING       2U

#define OTA_METADATA_FLASH_ADDRESS    0x08080000U

#define OTA_APP_SLOT_A_ADDRESS        0x08010000U
#define OTA_APP_SLOT_B_ADDRESS        0x08040000U
#define OTA_APP_SLOT_A_SIZE_BYTES      0x00030000U
#define OTA_APP_SLOT_B_SIZE_BYTES      0x00040000U

#define OTA_UPDATE_ERROR_NONE         0U
#define OTA_UPDATE_ERROR_FLASH        1U
#define OTA_UPDATE_ERROR_CRC          2U
#define OTA_UPDATE_ERROR_SIZE         3U

/*
 * OTA metadata stored in dedicated flash region.
 * Keep this layout stable once bootloader/app both depend on it.
 */
typedef struct
{
  uint32_t magic;
  uint32_t formatVersion;
  uint32_t metadataVersion;

  uint32_t activeSlot;
  uint32_t bootSlot;
  uint32_t confirmedSlot;

  uint32_t appAStartAddress;
  uint32_t appASizeBytes;
  uint32_t appACrc32;
  uint32_t appAState;
  uint32_t appAVersion;

  uint32_t appBStartAddress;
  uint32_t appBSizeBytes;
  uint32_t appBCrc32;
  uint32_t appBState;
  uint32_t appBVersion;

  uint32_t updateInProgress;
  uint32_t bootAttemptCount;
  uint32_t maxBootAttempts;
  uint32_t lastErrorCode;

  uint32_t metadataCrc32;
  uint32_t reserved[9];
} OtaMetadata_t;

extern const OtaMetadata_t gOtaMetadataDefault;

uint32_t OtaMetadata_CalcCrc32(const uint8_t *data, uint32_t length);
uint8_t OtaMetadata_Load(OtaMetadata_t *metadata);
HAL_StatusTypeDef OtaMetadata_Save(const OtaMetadata_t *metadataWithoutCrc);
HAL_StatusTypeDef OtaMetadata_SetUpdatePending(uint32_t slot, uint32_t imageSize, uint32_t imageCrc, uint32_t newVersion);
HAL_StatusTypeDef OtaMetadata_ConfirmActiveImage(void);
HAL_StatusTypeDef OtaMetadata_RecordLastError(uint32_t errorCode);

#endif /* OTA_METADATA_H */
