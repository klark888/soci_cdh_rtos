#include "sun_wrap.h"
#include "fsl_debug_console.h"
#include "peripherals.h"
#include "fsL_lpuart.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "task.h"
#include "com_protocol_helper.h"
#include "eps_wrap.h"

static const uint8_t anglesComm[4] = { 0x60, 0x04, 0x01, 0x05 };
static const int angleRespLength = 17; /* number of bytes returned for angle command */
static const unsigned long timeBetweenSends = 30;//number of seconds between command resending
sun_t Sun1; // sun sensor struct
static unsigned long last_sent = 0;
static uint8_t sun_recv_buffer[17];

/* converts the response bytes into floats, but saves them as doubles */
/* since the data is stored as a double, and also checks data for errors */
/* takes a pointer to the data that will be written over (e.g. angles), */
/* a pointer to the error, and a pointer to isValid */
static void readFloats(double* data, uint8_t* error, uint8_t* isValid){
   for (int x = 0; x < 3; x++){
      uint8_t bytes[4] = {sun_recv_buffer[4*x+3], sun_recv_buffer[4*x+4], sun_recv_buffer[4*x+5], sun_recv_buffer[4*x+6]};
      memcpy((data + x), &bytes, sizeof(float));
   }
   /* next, take the total sum of all the bytes except the address byte */
   int totSum = 0;
   /* need to sum from command (2nd) byte to last data byte (2nd to last of array) */
   for(int x = 1; x < angleRespLength - 1; x++){
      totSum += (int)sun_recv_buffer[x];
   }
   /* next, need to pare totSum down to its least significant byte, which can be accomplished by */
   /* comparing it to 255 */
   totSum = (totSum & 255);
   /* then compare it to the checkSum byte, which is the last byte */
   if(totSum != (int)sun_recv_buffer[angleRespLength - 1]){
      *data = -1000.0;
      *(data + 1) = -1000.0;
      *(data + 2) = -1000.0;
      *isValid = 0;
      *error = 14;
      return;
   }
   /* lastly, if we're getting values for angles (floatsToRead is 3) then the 4th read value needs to be the error (2nd to last) byte */
   *error = sun_recv_buffer[angleRespLength - 2];
   /* check if the error is acceptable */
   if(*error == 0){
      /* if it is acceptable, then the validity flag is raised (is 1) */
      *isValid = 1;
   }else{
      /* otherwise, it is not (is 0) */
      *isValid = 0;
   }
   return;
}

static void sendSunCommand() {
	last_sent = eps_getRTCTime() + timeBetweenSends;
	//clear buffer
	size_t size_clear = UART_receive_length( &LPUART3_rtos_handle );
	size_clear = size_clear > 64 ? 64 : size_clear;
	uint8_t clear_buffer[size_clear];
	UART_receive( &LPUART3_rtos_handle, (uint8_t*)&clear_buffer, size_clear, 0 );
	//sending command
	for( int try = 0; try < 3; try++ ) {
		if( UART_send( &LPUART3_rtos_handle, (uint8_t*)&anglesComm, sizeof( anglesComm ) ) ) {
			break;
		}
	}
}

/* getSunAngles() will issue the angles commmand to the sun sensor, read in the response, perform some checks */
/* and then call readFloats to interpret and check the data */
/* it needs the pointer to the sun structure */
/* error list */
/* 0 - nominal */
/* 10 - not enough radiation detected */
/* 11 - albedo */
/* 12 - albedo + sun */
/* 13 - detected light source but out of FoV */
/* 14 - incorrect checksum */
/* 15 - incorrect command byte response */
/* 16 - UART communications failure */
void getSunAngles( sun_t * Sun ) {
	//if more than 30 seconds between commands, send another command
	if( eps_getRTCTime() + timeBetweenSends - last_sent >= timeBetweenSends ) {
		sendSunCommand();
	}
	//read command
	size_t size_of_receive = UART_receive_length( &LPUART3_rtos_handle );
	if( size_of_receive >= angleRespLength ) {
		UART_receive( &LPUART3_rtos_handle, (uint8_t*)&sun_recv_buffer, angleRespLength, 0 );
		if( anglesComm[1] == sun_recv_buffer[1] ) {
			readFloats( Sun->angles, &( Sun->error ), &( Sun->isValid ) );
		} else {
			*(Sun->angles) = -1000.0;
			*(Sun->angles + 1) = -1000.0;
			*(Sun->angles + 2) = -1000.0;
			Sun->error = 15;
			Sun->isValid = 0;
		}
		sendSunCommand();
		return;
	}
	*(Sun->angles) = -2000.0;
	*(Sun->angles + 1) = -2000.0;
	*(Sun->angles + 2) = -2000.0;
	Sun->error = 16;
	Sun->isValid = 0;
}
