#include "peripherals.h"

#ifndef COM_PROTOCOL_HELPER_H_
#define COM_PROTOCOL_HELPER_H_

void busy_delay( unsigned int milliseconds );
size_t UART_receive_length( lpuart_rtos_handle_t *handle );
bool UART_send( lpuart_rtos_handle_t *handle, uint8_t *buffer, uint32_t length );
bool UART_receive( lpuart_rtos_handle_t *handle, uint8_t *buffer, uint32_t length, size_t *received );
bool I2C_send( lpi2c_rtos_handle_t * handle, lpi2c_master_transfer_t* transfer,
		uint16_t slaveAddress, uint8_t subAddress, uint8_t * masterSendBuffer, size_t tx_size );
bool I2C_request( lpi2c_rtos_handle_t * handle, lpi2c_master_transfer_t* transfer,
		uint16_t slaveAddress, uint8_t subAddress, uint8_t * rx_buffer, size_t rx_size );
bool I2C_variableRequest( lpi2c_rtos_handle_t * handle, lpi2c_master_transfer_t* transfer, uint8_t slaveAddress,
		uint32_t subAddress, size_t subSize, uint8_t * rx_buffer, size_t rx_size );
bool SPI_transfer( uint8_t * txBuffer, uint8_t * rxBuffer, size_t transferSize, uint32_t pcsPin );

#endif /* COM_PROTOCOL_HELPER_H_ */
