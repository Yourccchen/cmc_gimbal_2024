#ifndef __CAN_DRIVER_H__
#define __CAN_DRIVER_H__

#include "main.h"
#include "bsp_can.h"

void canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t canx_receive_data(CAN_HandleTypeDef *hcan, uint16_t *rec_id, uint8_t *buf);



#endif /* __CAN_DRIVER_H__ */


