#include "flash_if.h"
#include "stm32f4xx_hal.h"

static uint32_t FLASH_If_GetSector(uint32_t address)
{
    if (address < 0x08004000U) return FLASH_SECTOR_0;
    if (address < 0x08008000U) return FLASH_SECTOR_1;
    if (address < 0x0800C000U) return FLASH_SECTOR_2;
    if (address < 0x08010000U) return FLASH_SECTOR_3;
    if (address < 0x08020000U) return FLASH_SECTOR_4;
    if (address < 0x08040000U) return FLASH_SECTOR_5;
    if (address < 0x08060000U) return FLASH_SECTOR_6;
    if (address < 0x08080000U) return FLASH_SECTOR_7;
    if (address < 0x080A0000U) return FLASH_SECTOR_8;
    if (address < 0x080C0000U) return FLASH_SECTOR_9;
    if (address < 0x080E0000U) return FLASH_SECTOR_10;
    return FLASH_SECTOR_11;
}

int FLASH_If_Erase(uint32_t addr, uint32_t size)
{
    FLASH_EraseInitTypeDef EraseInitStruct = {0};
    uint32_t PageError = 0;
    uint32_t startSector;
    uint32_t endSector;

    if ((size == 0U) || (addr < FLASH_BASE))
    {
        return -1;
    }

    startSector = FLASH_If_GetSector(addr);
    endSector = FLASH_If_GetSector(addr + size - 1U);

    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = startSector;
    EraseInitStruct.NbSectors = (endSector - startSector) + 1U;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -1;
    }

    HAL_FLASH_Lock();
    return 0;
}

int FLASH_If_Write(uint32_t addr, const uint8_t *data, uint32_t size)
{
    uint32_t p;
    uint32_t end;

    if ((data == 0) || (size == 0U))
    {
        return -1;
    }

    HAL_FLASH_Unlock();
    end = addr + size;
    p = addr;
    while (p < end)
    {
        uint32_t word = 0xFFFFFFFFU;
        uint32_t remaining = end - p;

        if (remaining >= 4U)
        {
            word = ((uint32_t)data[0] << 0U) |
                   ((uint32_t)data[1] << 8U) |
                   ((uint32_t)data[2] << 16U) |
                   ((uint32_t)data[3] << 24U);
        }
        else
        {
            uint32_t i;
            for (i = 0U; i < remaining; i++)
            {
                word &= ~(0xFFU << (i * 8U));
                word |= ((uint32_t)data[i] << (i * 8U));
            }
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, p, word) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return -1;
        }

        if (remaining >= 4U)
        {
            data += 4U;
            p += 4U;
        }
        else
        {
            data += remaining;
            p += remaining;
        }
    }
    HAL_FLASH_Lock();
    return 0;
}
