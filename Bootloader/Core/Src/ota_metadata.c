#include "ota_metadata.h"

#include <string.h>

#define OTA_METADATA_FLASH_SECTOR     FLASH_SECTOR_8

static HAL_StatusTypeDef OtaMetadata_EraseStorage(void);
static HAL_StatusTypeDef OtaMetadata_WriteStorage(const OtaMetadata_t *metadataWithCrc);

const OtaMetadata_t gOtaMetadataDefault =
{
  .magic = OTA_METADATA_MAGIC,
  .formatVersion = OTA_METADATA_FORMAT_VERSION,
  .metadataVersion = 1U,

  .activeSlot = OTA_SLOT_A,
  .bootSlot = OTA_SLOT_A,
  .confirmedSlot = OTA_SLOT_A,

  .appAStartAddress = OTA_APP_SLOT_A_ADDRESS,
  .appASizeBytes = OTA_APP_SLOT_A_SIZE_BYTES,
  .appACrc32 = 0U,
  .appAState = OTA_IMAGE_STATE_VALID,
  .appAVersion = 0U,

  .appBStartAddress = OTA_APP_SLOT_B_ADDRESS,
  .appBSizeBytes = OTA_APP_SLOT_B_SIZE_BYTES,
  .appBCrc32 = 0U,
  .appBState = OTA_IMAGE_STATE_INVALID,
  .appBVersion = 0U,

  .updateInProgress = 0U,
  .bootAttemptCount = 0U,
  .maxBootAttempts = 3U,
  .lastErrorCode = OTA_UPDATE_ERROR_NONE,

  .metadataCrc32 = 0U,
  .reserved = {0U}
};

static uint32_t OtaMetadata_CalcCrc32(const uint8_t *data, uint32_t length)
{
  uint32_t crc = 0xFFFFFFFFU;
  uint32_t i;

  for (i = 0U; i < length; i++)
  {
    uint32_t byteValue = data[i];
    uint32_t bit;
    crc ^= byteValue;
    for (bit = 0U; bit < 8U; bit++)
    {
      uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }

  return ~crc;
}

uint8_t OtaMetadata_Load(OtaMetadata_t *metadata)
{
  const OtaMetadata_t *stored = (const OtaMetadata_t *)OTA_METADATA_FLASH_ADDRESS;
  OtaMetadata_t temp;
  uint32_t savedCrc;
  uint32_t calculatedCrc;

  if (metadata == NULL)
  {
    return 0U;
  }

  memcpy(&temp, stored, sizeof(OtaMetadata_t));

  if ((temp.magic != OTA_METADATA_MAGIC) || (temp.formatVersion != OTA_METADATA_FORMAT_VERSION))
  {
    return 0U;
  }

  savedCrc = temp.metadataCrc32;
  temp.metadataCrc32 = 0U;
  calculatedCrc = OtaMetadata_CalcCrc32((const uint8_t *)&temp, (uint32_t)sizeof(OtaMetadata_t));
  if (calculatedCrc != savedCrc)
  {
    return 0U;
  }

  memcpy(metadata, &temp, sizeof(OtaMetadata_t));
  return 1U;
}

HAL_StatusTypeDef OtaMetadata_Save(const OtaMetadata_t *metadataWithoutCrc)
{
  OtaMetadata_t temp;
  HAL_StatusTypeDef status;

  if (metadataWithoutCrc == NULL)
  {
    return HAL_ERROR;
  }

  memcpy(&temp, metadataWithoutCrc, sizeof(OtaMetadata_t));
  temp.magic = OTA_METADATA_MAGIC;
  temp.formatVersion = OTA_METADATA_FORMAT_VERSION;
  temp.metadataCrc32 = 0U;
  temp.metadataCrc32 = OtaMetadata_CalcCrc32((const uint8_t *)&temp, (uint32_t)sizeof(OtaMetadata_t));

  status = OtaMetadata_EraseStorage();
  if (status != HAL_OK)
  {
    return status;
  }

  return OtaMetadata_WriteStorage(&temp);
}

static HAL_StatusTypeDef OtaMetadata_EraseStorage(void)
{
  FLASH_EraseInitTypeDef eraseConfig;
  uint32_t sectorError = 0U;
  HAL_StatusTypeDef status;

  eraseConfig.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseConfig.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  eraseConfig.Sector = OTA_METADATA_FLASH_SECTOR;
  eraseConfig.NbSectors = 1U;

  HAL_FLASH_Unlock();
  status = HAL_FLASHEx_Erase(&eraseConfig, &sectorError);
  HAL_FLASH_Lock();

  return (status == HAL_OK) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef OtaMetadata_WriteStorage(const OtaMetadata_t *metadataWithCrc)
{
  uint32_t writeAddress = OTA_METADATA_FLASH_ADDRESS;
  const uint32_t *words = (const uint32_t *)metadataWithCrc;
  uint32_t i;

  HAL_FLASH_Unlock();
  for (i = 0U; i < (uint32_t)(sizeof(OtaMetadata_t) / sizeof(uint32_t)); i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, writeAddress, words[i]) != HAL_OK)
    {
      HAL_FLASH_Lock();
      return HAL_ERROR;
    }
    writeAddress += 4U;
  }
  HAL_FLASH_Lock();

  return HAL_OK;
}
