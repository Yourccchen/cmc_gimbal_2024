#include "can_driver.h"

/**
************************************************************************
* @brief:      	canx_send_data: �û������㷢�ͺ���
* @param:       hcan: CAN���
* @param:       id: 	CAN�豸ID
* @param:       data: ���͵�����
* @param:       len:  ���͵����ݳ���
* @retval:     	void
* @details:    	CAN���߷�������
************************************************************************
**/
void canx_send_data(hcan_t *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	canx_bsp_send_data(hcan, id, data, len);
}
/**
************************************************************************
* @brief:      	canx_receive_data: �û������㷢�ͺ���
* @param:       hcan��CAN���
* @param:       buf���������ݻ���
* @retval:     	���յ����ݳ���
* @details:    	��������
************************************************************************
**/
uint8_t canx_receive_data(hcan_t *hcan, uint16_t *rec_id, uint8_t *buf)
{
	uint8_t len;
	len = canx_bsp_receive(hcan, rec_id, buf);
	return len;
}


