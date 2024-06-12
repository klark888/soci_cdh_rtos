/*
 * MCU Pinouts:
IMG:
80	GPIO_AD_B1_10	UART4_TX	CIA Board
79	GPIO_AD_B1_11	UART4_RX	CIA Board
 */
#include "img_wrap.h"
#include "peripherals.h"
#include "com_protocol_helper.h"
#include "fsl_debug_console.h"
#include <stdbool.h>

//command responses
#define SUCCESS             0x1111
#define HANDSHAKEFAILURE    0x2222
#define CMDFAILURE          0x3333
#define PARAMFAILURE        0x4444
#define SENDFAILURE         0x5555
#define RECEIVEFAILURE      0x6666
#define NOTRECEIVEFAILURE   0x7777
// Packages sent from IMG are 32 bytes or less
#define PACKAGE_SIZE 31
//Possible responses
static const uint8_t NAK = 0x00;
static const uint8_t ACK = 0x01;
//global variables
uint16_t current_image_size = 0;
uint8_t IMG_PICTURE[IMAGE_SIZE];

// To send commands to IMG
// Param command  The main command to send
// Param param    Specifies a sub command
uint32_t image_sendCommand( uint8_t command, uint8_t param ) {
	uint8_t sending[] = { command, param };
	uint32_t retVal = 0;
	for( int try = 0; try < 3; try++ ) {
		PRINTF( "Sending command to IMG System command=%i, param=%i\r\n", command, param );
		size_t clearSize = UART_receive_length( &LPUART4_rtos_handle );
		if( clearSize != 0 ) {
			uint8_t clear[clearSize];
			UART_receive( &LPUART4_rtos_handle, clear, clearSize, 0 );
		}
		if( UART_send( &LPUART4_rtos_handle, (uint8_t*)&sending, sizeof( sending ) ) ) {
			uint8_t receiving[5];
			busy_delay( 200 );
			size_t size_of_receive = UART_receive_length( &LPUART4_rtos_handle );
			PRINTF( "[IMG WRAP] Bytes in UART: %i\r\n", size_of_receive );
			if( size_of_receive >= sizeof( receiving ) ) {
				if( UART_receive( &LPUART4_rtos_handle, (uint8_t*)&receiving, sizeof( receiving ), 0 ) ) {
					if( receiving[0] != ACK ) retVal = HANDSHAKEFAILURE;
					else if( receiving[1] != command ) retVal = CMDFAILURE;
					else if( receiving[2] != param ) retVal = PARAMFAILURE;
					else retVal = SUCCESS | ( (uint32_t)receiving[3] ) << 24 | ( (uint32_t)receiving[4] ) << 16;
					break;
				}
				retVal = RECEIVEFAILURE;
			} else {
				retVal = NOTRECEIVEFAILURE;
			}
		} else {
			retVal = SENDFAILURE;
		}
	}
	return retVal;
}

// get picture
uint32_t image_getPicture( uint8_t IMG_param ) {
	// get image size in prior to know number of packages
	uint32_t response = image_sendCommand( GET_PICTURE_SIZE, IMG_param );
	if( ( response & 0xFFFF ) == SUCCESS ) {
		unsigned int pictureSize = response >> 16;
		unsigned int numPackages = pictureSize / PACKAGE_SIZE; // last byte is verification byte
		unsigned int remainingBytes = pictureSize - numPackages * PACKAGE_SIZE;
		current_image_size = pictureSize;
		PRINTF( "pictureSize = %d,\n", pictureSize );
		PRINTF( "numPackages = %d, \n", numPackages );
		PRINTF( "remainingBytes = %d.\n", remainingBytes );
		/* Checking response of valid command */
		response = image_sendCommand( GET_PICTURE, IMG_param );
		if( ( response & 0xFFFF ) != SUCCESS ) {
			return response;
		}
		PRINTF( "External ACK received, ready to receive data.\n" );
		uint8_t package_buffer[ PACKAGE_SIZE + 1 ];
		size_t receiveSize = 0;
		// Begin reading data
		for( int i = 0; i <= numPackages; i++ ) {
			size_t currentSize = i == numPackages ? remainingBytes : PACKAGE_SIZE;
			package_buffer[currentSize] = 0;
			for( int try = 0; try < 10; try++ ) {// retry until package received is correct
				// wait for package
				UART_send( &LPUART4_rtos_handle, (uint8_t *)&ACK, 1 );
				UART_receive( &LPUART4_rtos_handle, package_buffer, currentSize + 1, &receiveSize );
				if( package_buffer[currentSize] != 0xFF ) {
					PRINTF( "verification byte is not 0xFF, need to resend package.\n" );
					UART_send( &LPUART4_rtos_handle, (uint8_t *)&NAK, 1 );
				} else {
					for( int j = 0; j < currentSize; j++ ) {
						IMG_PICTURE[ i * PACKAGE_SIZE + j ] = package_buffer[j];
					}
					break;
				}
			}
		}
		return SUCCESS;
	} else {
		return response;
	}
}
