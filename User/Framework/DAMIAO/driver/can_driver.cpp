#include "can_driver.h"

/**
************************************************************************
* @brief:      	canx_send_data: 用户驱动层发送函数
* @param:       hcan: CAN句柄
* @param:       id: 	CAN设备ID
* @param:       data: 发送的数据
* @param:       len:  发送的数据长度
* @retval:     	void
* @details:    	CAN总线发送数据
************************************************************************
**/
void canx_send_data(hcan_t *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	canx_bsp_send_data(hcan, id, data, len);
}
/**
************************************************************************
* @brief:      	canx_receive_data: 用户驱动层发送函数
* @param:       hcan；CAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t canx_receive_data(hcan_t *hcan, uint16_t *rec_id, uint8_t *buf)
{
	uint8_t len;
	len = canx_bsp_receive(hcan, rec_id, buf);
	return len;
}


