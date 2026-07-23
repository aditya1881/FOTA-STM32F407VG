#ifndef FLASH_IF_H
#define FLASH_IF_H

#include <stdint.h>

int FLASH_If_Erase(uint32_t addr, uint32_t size);
int FLASH_If_Write(uint32_t addr, const uint8_t *data, uint32_t size);

#endif
