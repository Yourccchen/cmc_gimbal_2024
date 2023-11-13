//
// Created by DELL on 2023/9/26.
//

#ifndef HERO_TEST_REMOTEIO_H
#define HERO_TEST_REMOTEIO_H
#include <cstdint>
#include "stm32f4xx_hal.h"

extern void REMOTEIO_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#endif //HERO_TEST_REMOTEIO_H
