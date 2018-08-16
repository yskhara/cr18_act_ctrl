/*
 * SerialClass.h
 *
 *  Created on: Aug 15, 2018
 *      Author: yusaku
 */

#ifndef ROSSERIAL_CLIENT_SRC_ROS_LIB_SERIALCLASS_H_
#define ROSSERIAL_CLIENT_SRC_ROS_LIB_SERIALCLASS_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

extern UART_HandleTypeDef huart1;
static constexpr uint16_t BUF_SIZE = 1024;

class SerialClass
{
private:
	static constexpr uint16_t buf_mask = BUF_SIZE - 1;
	uint8_t tx_buf[BUF_SIZE];
	uint8_t rx_buf[BUF_SIZE];
	volatile bool tx_cplt = true;
	uint16_t rx_tail = 0;
	uint16_t tx_head = 0;
	uint16_t tx_tail = 0;
	UART_HandleTypeDef &huart;

public:
	SerialClass(UART_HandleTypeDef &huart) : huart(huart)
	{
		//this->huart = huart;
	}

	inline UART_HandleTypeDef * const get_handle(void)
	{
		return &huart;
	}

	inline void start_dma(void)
	{
		tx_cplt = true;
		rx_tail = 0;
		HAL_UART_Receive_DMA(&huart, (uint8_t *) rx_buf, BUF_SIZE);
	}

	inline int read(void)
	{
		uint16_t rx_head = (BUF_SIZE - huart.hdmarx->Instance->CNDTR)
				& buf_mask;
		if (rx_tail == rx_head)
		{
			return -1;
		}

		int c = (int) rx_buf[rx_tail++];
		rx_tail &= buf_mask;
		return c;
	}

	inline void write(const uint8_t * const c, const int length)
	{
		if (length > BUF_SIZE || length < 1)
		{
			return;
		}

		while(!tx_cplt)
		{

		}

		for (int i = 0; i < length; i++)
		{
			tx_buf[i] = c[i];
		}

		if(tx_cplt)
		{
			tx_cplt = false;
			HAL_UART_Transmit_DMA(&huart, tx_buf, length);
		}
	}

	inline void tx_cplt_callback(void)
	{
		tx_cplt = true;
	}
};

SerialClass serial(huart1);

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	// comparing pointer
	if (huart->Instance == serial.get_handle()->Instance)
	{
		serial.tx_cplt_callback();
	}
}

#endif /* ROSSERIAL_CLIENT_SRC_ROS_LIB_SERIALCLASS_H_ */
