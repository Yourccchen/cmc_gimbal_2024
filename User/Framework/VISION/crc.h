//
// Created by DELL on 2023/9/12.
//


#ifndef HERO_GIMBAL_2024_CRC_H
#define HERO_GIMBAL_2024_CRC_H

#include <cstdint>
uint16_t Get_CRC16_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);
#endif //HERO_GIMBAL_2024_CRC_H
