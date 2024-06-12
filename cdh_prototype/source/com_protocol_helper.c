#include "peripherals.h"
#include "fsl_gpio.h"
#include "com_protocol_helper.h"
#include "board.h"
#include "fsl_debug_console.h"
#include <time.h>

#define SHOW_DATA 0

void busy_delay( unsigned int milliseconds ) {
    long pause;
    clock_t now,then;
    pause = milliseconds * CLOCKS_PER_SEC / 1000;
    now = then = clock();
    while( ( now-then ) < pause )
        now = clock();
}

size_t UART_receive_length( lpuart_rtos_handle_t* handle ) {
	return LPUART_TransferGetRxRingBufferLength( handle->base, (lpuart_handle_t*)handle->t_state );
}

bool UART_send( lpuart_rtos_handle_t* handle, uint8_t *buffer, uint32_t length ) {
#if SHOW_DATA
	PRINTF( "UART will send data: " );
	int i = 0;
	for( i = 0; i < length; i++ ) {
		PRINTF( "%2x  ", buffer[i] );
	}
#endif
	status_t status = LPUART_RTOS_Send( handle, buffer, length );
	if( status == kStatus_Success ) {
#if SHOW_DATA
		PRINTF( "- successfully sent\r\n" );
#endif
		return true;
	} else {
		PRINTF( "Error in UART send: %d!\r\n", status );
		return false;
	}
}

bool UART_receive( lpuart_rtos_handle_t *handle, uint8_t *buffer, uint32_t length, size_t *received ) {
#if SHOW_DATA
	PRINTF( "UART will attempt to receive %i bytes of data, ", length );
#endif
	status_t status = LPUART_RTOS_Receive( handle, buffer, length, received );
	if( status == kStatus_Success ) {
		if( length < *received ) {
			*received = length;
		}
#if SHOW_DATA
		PRINTF( "successfully received: " );
		for( int i = 0; i < *received; i++ ) {
			PRINTF( "%2x ", buffer[i] );
		}
		PRINTF( "\r\n" );
#endif
		return true;
	} else {
		//PRINTF( "Error in UART receive: %d!\r\n", status );
		return false;
	}
}

bool I2C_send(lpi2c_rtos_handle_t * handle, lpi2c_master_transfer_t* transfer,
		uint16_t slaveAddress, uint8_t subAddress, uint8_t * masterSendBuffer, size_t tx_size) {
#if SHOW_DATA
	PRINTF( "I2C will send data to ( %x, %x ): ", slaveAddress, subAddress );
	int i = 0;
	for( i = 0; i < tx_size; i++ ) {
		PRINTF( "%2x ", masterSendBuffer[i] );
	}
#endif
	//Make modifications on lpi2c_master_transfer_t
	transfer->slaveAddress = slaveAddress;
	transfer->data = masterSendBuffer;
	transfer->subaddress = subAddress;
	transfer->subaddressSize = 1;
	transfer->dataSize = tx_size;
	transfer->direction = kLPI2C_Write;
	// Monitor status through transfer
	status_t status = LPI2C_RTOS_Transfer( handle, transfer );
	if( status == kStatus_Success ) {
#if SHOW_DATA
		PRINTF( "- successfully sent\r\n" );
#endif
		return true;
	} else {
		PRINTF( "Error in I2C send: %d!\r\n", status );
		return false;
	}
}

bool I2C_request( lpi2c_rtos_handle_t * handle, lpi2c_master_transfer_t* transfer, uint16_t slaveAddress,
		uint8_t subAddress, uint8_t * rx_buffer, size_t rx_size ) {
#if SHOW_DATA
	PRINTF( "I2C will request data from ( %x, %x ), ", slaveAddress, subAddress );
#endif
	// Make modifications on lpi2c_master_transfer_t
	transfer->slaveAddress = slaveAddress;
	transfer->subaddress = subAddress;
	transfer->subaddressSize = 1;
	transfer->data = rx_buffer;
	transfer->dataSize = rx_size;
	transfer->direction = kLPI2C_Read;
	// Monitor status through transfer
	status_t status = LPI2C_RTOS_Transfer( handle, transfer );
	if( status == kStatus_Success ) {
#if SHOW_DATA
		PRINTF( "successfully received: " );
		for( int i = 0; i < rx_size; i++ ) {
			PRINTF( "%2x ", rx_buffer[i] );
		}
		PRINTF( "\r\n" );
#endif
		return true;
	} else {
		PRINTF( "Error in I2C receive: %d!\r\n", status );
		return false;
	}
}

bool I2C_variableRequest( lpi2c_rtos_handle_t * handle, lpi2c_master_transfer_t* transfer, uint8_t slaveAddress,
		uint32_t subAddress, size_t subSize, uint8_t * rx_buffer, size_t rx_size ) {
#if SHOW_DATA
	PRINTF( "I2C will request data from ( %x, %x ), ", slaveAddress, subAddress );
#endif
	// Make modifications on lpi2c_master_transfer_t
	transfer->slaveAddress = slaveAddress;
	transfer->subaddress = subAddress;
	transfer->subaddressSize = subSize;
	transfer->data = rx_buffer;
	transfer->dataSize = rx_size;
	transfer->direction = kLPI2C_Read;
	// Monitor status through transfer
	status_t status = LPI2C_RTOS_Transfer( handle, transfer );
	if( status == kStatus_Success ) {
#if SHOW_DATA
		PRINTF( "successfully received: " );
		for( int i = 0; i < rx_size; i++ ) {
			PRINTF( "%2x ", rx_buffer[i] );
		}
		PRINTF( "\r\n" );
#endif
		return true;
	} else {
		PRINTF( "Error in I2C receive: %d!\r\n", status );
		return false;
	}
}

/*!
 * brief SPI send and receive
 *
 * SPI transfer lpspi_master_config_t config
 *
 * param txBuffer pointer to tx buffer array.
 * param rxBuffer pointer to rx buffer array.
 * param transferSize number of bytes to send and receive
 * Param pcsPin chip select pin. Use following
 * 		RWA0
 * 		RWA1
 * 		RWA2
 * 		RWA3
 * return false if receive was not successful.
 */
bool SPI_transfer( uint8_t * txBuffer, uint8_t* rxBuffer, size_t transferSize, uint32_t pcsPin )
{
	lpspi_transfer_t LPSPI1_config;
	status_t status;
	//Start master transfer
	LPSPI1_config.txData      = txBuffer;
	LPSPI1_config.rxData      = rxBuffer;
	LPSPI1_config.dataSize    = transferSize;
	LPSPI1_config.configFlags = pcsPin | kLPSPI_MasterPcsContinuous;
	status = LPSPI_RTOS_Transfer( &LPSPI1_handle, &LPSPI1_config );
#if SHOW_DATA
	PRINTF( "SPI will transfer to PCS %i, sending: ", pcsPin );
	int i = 0;
	for( i = 0; i < transferSize; i++ ) {
		PRINTF( "%2x ", txBuffer[i] );
	}
#endif
	if( status == kStatus_Success ) {
#if SHOW_DATA
	PRINTF( ", receiving: " );
	int i = 0;
	for( i = 0; i < transferSize; i++ ) {
		PRINTF( "%2x ", rxBuffer[i] );
	}
	PRINTF( "\r\n" );
#endif
		return true;
	} else {
		PRINTF( "SPI master transfer completed with error %i.\r\n", status );
		return false;
	}
}
