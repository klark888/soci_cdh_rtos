#include "fsl_gpio.h"
#include "fsl_debug_console.h"
#include "peripherals.h"
#include "com_protocol_helper.h"
#include "com_wrap.h"
#include "fsl_rtwdog.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//enable/disable antenna
#define ANTENNA_ENABLE 1
//enable/disable/prinout radio
#define COM_TYPE 1 // Uplink/downlink communication type: 0 for disabled, 1 for antenna, 2 for console

//commands                                     SOH   CMD   CONTROL                      CKSUM
static unsigned char set_dealer_mode_buf[] = { 0x01, 0x44, 0x01,                         0xBA };//set to user
static unsigned char set_channel[] =         { 0x01, 0x03, 0x03,                         0xF9 };//set to channel 3
static unsigned char get_channel[] =         { 0x01, 0x04,                               0xFB };//get channel
static unsigned char set_rx_freq[] =         { 0x01, 0x39, 0x03, 0x1A, 0x0D, 0xFF, 0x88, 0x15 };//set ch3 to 437.125MHz
//static unsigned char get_rx_freq[] =         { 0x01, 0x3A, 0x03,                         0xC2 };//get rx frequency
static unsigned char set_tx_freq[] =         { 0x01, 0x37, 0x03, 0x1A, 0x0D, 0xFF, 0x88, 0x17 };//set ch3 to 437.125MHz
//static unsigned char get_tx_freq[] =         { 0x01, 0x38, 0x03,                         0xC4 };//get tx frequency
static unsigned char set_bandwidth[] =       { 0x01, 0x70, 0x04, 0x01,                   0x8A };//set to 12.5kBps
static unsigned char set_current_power[] =   { 0x01, 0x71, 0x01,                         0x8D };//set to 0.5W
static unsigned char get_current_power[] =   { 0x01, 0x72,                               0x8D };//get current
static unsigned char set_modulation[] =      { 0x01, 0x2B, 0x00,                         0xD4 };//DEPRECATED
static unsigned char program_buf[] =         { 0x01, 0x1E,                               0xE1 };//set programming
static unsigned char warm_reset[] =          { 0x01, 0x1D, 0x01,                         0xE1 };//warm reset
//expected responses                               SOH   CMD   DATA                          CKSUM
static unsigned char set_dealer_response[] =     { 0x01, 0xC4, 0x00,                         0x3B };
static unsigned char set_channel_response[] =    { 0x01, 0x83, 0x00,                         0x7C };
static unsigned char get_channel_response[] =    { 0x01, 0x84, 0x03,                         0x78 };
static unsigned char set_rx_freq_response[] =    { 0x01, 0xB9, 0x00,                         0x46 };
//static unsigned char get_rx_freq_response[] =    { 0x01, 0xBA, 0x00, 0x1A, 0x0D, 0xFF, 0x88, 0x97 };
static unsigned char set_tx_freq_response[] =    { 0x01, 0xB7, 0x00,                         0x48 };
//static unsigned char get_tx_freq_response[] =    { 0x01, 0xB8, 0x00, 0x1A, 0x0D, 0xFF, 0x88, 0x99 };
static unsigned char set_bandwidth_response[] =  { 0x01, 0xF0, 0x00,                         0x0F };
static unsigned char set_power_response[] =      { 0x01, 0xF1, 0x00,                         0x0E };
static unsigned char get_power_response[] =      { 0x01, 0xF2, 0x01,                         0x0C };
static unsigned char set_modulation_response[] = { 0x01, 0xAB, 0x00,                         0x54 };
static unsigned char program_response[] =        { 0x01, 0x9E, 0x00,                         0x61 };
static unsigned char reset_response[] =          { 0x01, 0x9D, 0x00,                         0x62 };
//antenna comm
const uint8_t antenna_slave_addr = 0x33;
#if ANTENNA_ENABLE
gpio_pin_config_t burn_init_cfg = { kGPIO_DigitalOutput, 0, kGPIO_NoIntmode };
const uint8_t antenna_burn_pin1 = 23;
const uint8_t antenna_burn_pin2 = 28;
#endif

struct com_telemetry com_telem_data;

// Sends data to radio via UART, if the response is not correct it retries sending the command
static bool sendConfigCommand( uint8_t command[], uint8_t expectedResponse[], size_t sizeCommand, size_t sizeResponse ) {
    for( int try = 0; try < 1; try++ ) {
    	//clear rx prior to command transmission
        size_t rx_size = UART_receive_length( &LPUART1_rtos_handle );
        if( rx_size != 0 ) {
        	uint8_t receive[rx_size];
        	UART_receive( &LPUART1_rtos_handle, receive, rx_size, 0 );
        }
        //send command and check results
        size_t received = 0;
    	if( UART_send( &LPUART1_rtos_handle, command, sizeCommand ) ) {
    		busy_delay( 500 );
    		RTWDOG_Refresh( RTWDOG );
    		uint8_t actualResponse[sizeResponse];
    		if( sizeResponse <= UART_receive_length( &LPUART1_rtos_handle ) &&
    				UART_receive( &LPUART1_rtos_handle, actualResponse, sizeResponse, &received ) ) {
    			bool correct = true;
    			for( int i = 0; i < sizeResponse; i++ ) {
    				if( correct && actualResponse[i] != expectedResponse[i] ) {
    					correct = false;
    				}
    			}
    			if( correct ) {
    				return true;
    			}
    		}
    	}
    }
    return false;
}

//set radio to command mode
static bool enterCommandMode() {
	busy_delay( 300 ); // More than 100 milliseconds silence
	if( UART_send( &LPUART1_rtos_handle, (uint8_t*)"+++", 3 ) ) {
		busy_delay( 300 );
		RTWDOG_Refresh( RTWDOG );
		com_telem_data.is_command_mode = 1;
		return true;
	}
	return false;
}

//set radio to data mode
static bool exitCommandMode() {
	if( sendConfigCommand( warm_reset, reset_response, sizeof( warm_reset ), sizeof( reset_response ) ) ) {
		com_telem_data.is_command_mode = 0;
		return true;
	}
	return false;
}

/**
 * Configures the radio by:
 * 1. Entering command mode
 * 2. Setting dealer mode
 * 3. Setting channel
 * 4. Setting Rx Frequency
 * 5. Setting Tx frequency
 * 6. Setting bandwidth
 * 7. Setting power
 * 8. Setting modulation
 * 9. Setting programming.
 *
 * If everything checks out, radio is confirmed to be configured properly
 */
bool com_configRadio() {
	PRINTF( "Configuring radio\r\n" );
#if COM_TYPE == 1
	uint32_t flags = enterCommandMode();
	//PRINTF( "Setting dealer mode TRUE...\n" );
	flags |= sendConfigCommand( set_dealer_mode_buf, set_dealer_response,
			sizeof( set_dealer_mode_buf ), sizeof( set_dealer_response ) ) << 1;
	//PRINTF( "Setting channel to 3...\n" );
	flags |= sendConfigCommand( set_channel, set_channel_response,
			sizeof( set_channel ), sizeof( set_channel_response ) ) << 2;
	//PRINTF( "Setting Rx frequency to 437.125MHz...\n" );
	flags |= sendConfigCommand( set_rx_freq, set_rx_freq_response,
			sizeof( set_rx_freq ), sizeof( set_rx_freq_response ) ) << 3;
	//PRINTF( "Setting Tx frequency to 437.125MHz...\n" );
	flags |= sendConfigCommand( set_tx_freq, set_tx_freq_response,
			sizeof( set_tx_freq ), sizeof( set_tx_freq_response ) ) << 4;
	//PRINTF( "Setting Bandwidth to 12.5k...\n" );
	flags |= sendConfigCommand( set_bandwidth, set_bandwidth_response,
			sizeof( set_bandwidth ), sizeof( set_bandwidth_response ) ) << 5;
	//PRINTF( "Setting power 0.5W...\n" );
	flags |= sendConfigCommand( set_current_power, set_power_response,
			sizeof( set_current_power ), sizeof( set_power_response ) ) << 6;
	//PRINTF( "Setting modulation...\n" );
	flags |= sendConfigCommand( set_modulation, set_modulation_response,
			sizeof( set_modulation ), sizeof( set_modulation_response ) ) << 7;
	//PRINTF( "Setting programming...\n" );
	flags |= sendConfigCommand( program_buf, program_response,
				sizeof( program_buf ), sizeof( program_response ) ) << 8;
	//PRINTF( "Exiting...\n" );
	flags |= exitCommandMode() << 9;
#else
	uint32_t flags = 0;
#endif
	com_telem_data.radio_config_status = flags;
	PRINTF( "Radio config success: %d\r\n", flags );
	return flags == ( 1 | 2 | 4 | 8 | 16 | 32 | 64 | 128 | 256 | 512 );
}

//checks health of coms
bool com_healthcheck() {
	PRINTF( "Checking COM health\r\n" );
#if COM_TYPE == 1
	uint32_t flags = enterCommandMode();
	//PRINTF( "Getting channel...\r\n" );
	flags |= sendConfigCommand( get_channel, get_channel_response,
			sizeof( get_channel ), sizeof( get_channel_response ) ) << 1;
	//PRINTF( "Getting Rx frequency...\r\n" );
	flags |= 1/*sendConfigCommand( get_rx_freq, get_rx_freq_response,
				sizeof( get_rx_freq ), sizeof( get_rx_freq_response ) )*/ << 2;
	//PRINTF( "Getting Tx frequency...\r\n" );
	flags |= 1/*sendConfigCommand( get_tx_freq, get_tx_freq_response,
				sizeof( get_tx_freq ), sizeof( get_tx_freq_response ) )*/ << 3;
	//PRINTF( "Getting power...\r\n" );
	flags |= sendConfigCommand( get_current_power, get_power_response,
				sizeof( get_current_power ), sizeof( get_power_response ) ) << 4;
	//PRINTF( "Exiting...\r\n" );
	flags |= exitCommandMode() << 5;
	com_telem_data.com_system_health_status = flags;
#else
	uint32_t flags = 0;
#endif
	PRINTF( "COM health success: %d\r\n", flags );
	return flags == ( 1 | 2 | 4 | 8 | 16 | 32 );
}

//returns a true if doors are deployed
bool com_i2c_checkDeploy()  {
	uint8_t rcv_buffer[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
	I2C_request( &LPI2C1_masterHandle, &LPI2C1_masterTransfer,
			antenna_slave_addr, 0, rcv_buffer, sizeof( rcv_buffer ) );
	com_telem_data.com_antenna_i2c = *( (uint32_t*)rcv_buffer );
	uint8_t test = 0x8 | 0x16 | 0x32 | 0x64;
	PRINTF( "Antenna deployment: %i\r\n", test );
	return ( test & rcv_buffer[0] ) == test;
}

void com_antenna_send( uint8_t* buffer, size_t size ) {
#if COM_TYPE == 0
#elif COM_TYPE == 1
	PRINTF( "Sending packet of size %i\r\n", size );
	UART_send( &LPUART1_rtos_handle, buffer, size );
#elif COM_TYPE == 2
	PRINTF( "Sending packet:" );
	for( int i = 0; i < size; i++ ) {
		if( i % 16 == 0 ) {
			PRINTF( "\r\n" );
		}
		int val = ( ( (int)buffer[i] ) + 256 ) % 256;
		if( val < 16 ) {
			PRINTF( "0" );
		}
		PRINTF( "%x ", val );
	}
	PRINTF( "\r\n" );
#endif
}

void com_antenna_receive( uint8_t* buffer, size_t* size ) {
#if COM_TYPE == 0
	*size = 0;
#elif COM_TYPE == 1
	PRINTF( "Attempting to receive packet of size %i\r\n", *size );
	size_t real_size = UART_receive_length( &LPUART1_rtos_handle );
	if( real_size > 0 ) {
		PRINTF( "Uplink packet received from MCC, size %i\r\n", real_size );
		UART_receive( &LPUART1_rtos_handle, buffer, real_size > *size ? *size : real_size, size );
	} else {
		PRINTF( "No uplink packet detected from MCC\r\n" );
		*size = 0;
	}
#elif COM_TYPE == 2
	*size = 0;
	PRINTF( "Receiving packet:\r\n" );
	for( int i = 0; i < *size; ) {
		int next_char = GETCHAR();
		if( next_char >= '0' && next_char <='9' ) {
			next_char = next_char - '0';
		} else if( next_char >= 'A' && next_char <= 'F' ) {
			next_char = next_char - 'A' + 0xA;
		} else if( next_char >= 'a' && next_char <= 'f' ) {
			next_char = next_char - 'a' + 0xA;
		} else if( next_char == '\r' || next_char == '\n' ) {
			*size = i + 1;
			break;
		} else if( next_char == ' ' || next_char == '\t' ) {
			i++;
		} else {
			*size = 0;
			break;
		}
		buffer[i] = ( buffer[i] << 4 ) | next_char;
	}
#endif
}

void com_deployAntenna_algorithmOne() {
#if ANTENNA_ENABLE
	//using algorithm one as the default
	uint8_t algo = 0x1F;
    I2C_send( &LPI2C1_masterHandle, &LPI2C1_masterTransfer, antenna_slave_addr, 0, &algo, 1 );
#endif
}

void com_deployAntenna_algorithmTwo() {
#if ANTENNA_ENABLE
	uint8_t algo = 0x2F;
	I2C_send( &LPI2C1_masterHandle, &LPI2C1_masterTransfer, antenna_slave_addr, 0, &algo, 1 );
#endif
}

void com_set_burn_wire1() {
#if ANTENNA_ENABLE
	GPIO_PinInit( GPIO1, antenna_burn_pin1, &burn_init_cfg );
	GPIO_PinWrite( GPIO1, antenna_burn_pin1, 1 );
#endif
}

void com_set_burn_wire2() {
#if ANTENNA_ENABLE
	GPIO_PinInit( GPIO1, antenna_burn_pin2, &burn_init_cfg );
	GPIO_PinWrite( GPIO1, antenna_burn_pin2, 1 );
#endif
}
