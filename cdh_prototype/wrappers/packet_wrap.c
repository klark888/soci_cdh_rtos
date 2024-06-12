#include "com_wrap.h"
#include "eps_wrap.h"
#include "img_wrap.h"
#include "packet_wrap.h"
#include "FSW_Lib0.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "com_protocol_helper.h"
#include "fsl_gpio.h"
#include "hmac_sha256.h"

//DEFINITIONS-------------------------------------------------------------------------------
//disable/enable hmac
#define HMAC_ENABLE 1
//magic number
#define DOWNLINK_MAGIC_NUMBER 0x62009D01
#define UPLINK_MAGIC_NUMBER 0x69434F53
//uplink packet ids
#define NO_COMMAND_ID     0x0000
#define COMMAND_HEADER_ID 0x1111
#define COM_COMMAND_ID    0x2222
#define GNC_COMMAND_ID    0x3333
#define IMG_COMMAND_ID    0x4444
#define RAW_COMMAND_ID    0x5555
//downlink packet ids
#define HEADER_TEST_ID       0x0000
#define BEACON_ID            0x1111
#define EPS_TELEMETRY_ID     0x2222
#define COM_TELEMETRY_ID     0x3333
#define GNC_TELEMETRY_ID     0x4444
#define SEN_TELEMETRY_ID     0x5555
#define IMG_TELEMETRY_ID     0x6666
#define LOGGER_ID            0x7777
#define ACK_ID               0x8888
//packet struct
#define DOWNLINK_LEN 320
#define TLM_LEN 243
#define SZ_TLM 255
struct telemetry_packet {
	uint32_t magic_number;
	uint16_t packet_id;
	uint16_t response_id;
	uint16_t byte_location;
	uint8_t  length;
	uint8_t  checksum;
	uint8_t  data[TLM_LEN];
};
#define CMD_LEN 214
#define SZ_CMD 255
struct command_packet {
	uint32_t magic_number;
	uint16_t packet_id;
	uint16_t response_id;
	uint8_t  telem_flags;
	uint8_t  hmac[32];
	uint8_t  data[CMD_LEN];
};

//MISCELLANEOUS PACKET STRUCTURES-------------------------------------------------------------------------------
struct beacon_telemetry {
	float32_t obc_battery_temperature;
	float32_t obc_battery_voltage;
	float32_t obc_battery_current;
	float32_t obc_battery_charge;
	uint32_t obc_operating_state;
	uint64_t obc_clock_time;
	uint64_t obc_start_time;
};
#define ACK_LEN 30
struct ack_packet {
	uint8_t count;
	uint16_t command_id[ACK_LEN];
	uint16_t subcommand_id[ACK_LEN];
	uint32_t return_value[ACK_LEN];
};
struct com_command {
	uint8_t reconfig_flight_radio;
	uint8_t check_radio_health;
	uint8_t check_antenna_status;
	uint8_t echo_signal;
	uint8_t manually_enter_antenna_algo1;
	uint8_t manually_enter_antenna_algo2;
	uint8_t manually_enter_burn_wire1;
	uint8_t manually_enter_burn_wire2;
};
struct img_command {
	uint16_t send_location_offset;
	uint8_t active_flags;
	uint8_t check_status;
	uint8_t take_picture;
	uint8_t get_picture_size;
	uint8_t get_picture;
	uint8_t set_picture_contrast;
	uint8_t set_picture_brightness;
	uint8_t set_picture_exposure;
	uint8_t set_sleep_time;
};
struct raw_command {
	uint8_t method_type;
	uint8_t send_receive;
	uint8_t send_handle;
	uint32_t identifier;
	uint8_t send_length;
	uint8_t send_buffer[40];
};

//INTERNAL VARIABLES-------------------------------------------------------------------------------
//hmac key - should be 32 bytes long
static char* HMAC_KEY = "123456789012345679012";
//primary downlink buffer being written
static uint16_t num_downlink_used = 0;
static struct telemetry_packet downlink_data[DOWNLINK_LEN];
static struct command_packet   uplink_data;
//cache variable for image taking
static uint32_t image_buffer_offset = 0;
//logger information
static uint16_t log_write = 0;
static unsigned char logger_buffer[8192];
static unsigned char statement[] =
		"=====================SOC-i=====================\n"
		"NOTE: This is a test command to see if MCC can \n"
		"      parse messages correctly. Remove from    \n"
		"      final code.                              \n"
		"NAME: Satellite for Optimal Control and Imaging\n"
		"CREATED-BY: University of Washington Seattle   \n"
		"            Aeronautical and Astronautical     \n"
		"            CubeSat Team                       \n"
		"RSO-LINK: https://huskylink.washington.edu/orga\n"
		"          nization/aact                        \n"
		"GITHUB-CODE: AA-CubeSat-Team/soci_cdh_rtos     \n"
		"CODE-VERSION: https://github.com/klark888      \n"
		"UW Mechanical Engineering > UW AeroAstro       \n"
		"===============================================\0";

//UTILITY METHODS-------------------------------------------------------------------------------
static void queueDownlink( uint16_t packet_id, void* ptr, size_t size ) {
	uplink_data.response_id++;
	uint8_t* buffer = (uint8_t*)ptr;
	uint16_t packet_count = ( (uint16_t)size ) / TLM_LEN + ( size % TLM_LEN == 0 ? 0 : 1 );
	if( packet_count >= DOWNLINK_LEN || size > 32768 * TLM_LEN ) {
		return;
	} else if( packet_count >= DOWNLINK_LEN - num_downlink_used ) {
		num_downlink_used = 0;
	}
	for( uint16_t i = 0; i < packet_count; i++ ) {
		uint16_t loc = num_downlink_used + i;
		downlink_data[loc].magic_number = DOWNLINK_MAGIC_NUMBER;
		downlink_data[loc].packet_id = packet_id;
		downlink_data[loc].response_id = uplink_data.response_id;
		downlink_data[loc].byte_location = i;
		downlink_data[loc].length = (uint8_t)( i == packet_count - 1 ? size % TLM_LEN + ( SZ_TLM - TLM_LEN ) : SZ_TLM );
		for( int j = 0; j < downlink_data[loc].length; j++ ) {
			uint8_t byte = buffer[ i * TLM_LEN + j ];
			downlink_data[loc].checksum += byte;
			downlink_data[loc].data[j] = byte;
		}
	}
	num_downlink_used += packet_count;
}

static void prepareAckPacket( struct ack_packet* packet, uint16_t command_id,
		uint16_t subcommand_id, uint32_t return_value ) {
	if( packet->count < ACK_LEN ) {
		packet->command_id[packet->count] = command_id;
		packet->subcommand_id[packet->count] = subcommand_id;
		packet->return_value[packet->count] = return_value;
		packet->count++;
	}
}

//DOWNLINK METHODS-------------------------------------------------------------------------------
void packet_queue_beacon() {
	PRINTF( "BEACON packet queued\r\n" );
	struct beacon_telemetry packet;
	packet.obc_battery_temperature = eps_telem_data.BAT_TEMP;
	packet.obc_battery_voltage = eps_telem_data.PowerBusVoltageVbat;
	packet.obc_battery_current = eps_telem_data.PowerBusCurrentVbat;
	packet.obc_battery_charge = eps_telem_data.RemainingCapacity;
	packet.obc_operating_state = (uint32_t)operatingMode;
	packet.obc_clock_time = eps_getRTCTime();
	packet.obc_start_time = initialize_time;
	queueDownlink( BEACON_ID, &packet, sizeof( packet ) );
}

void packet_queue_eps() {
	PRINTF( "EPS packet queued\r\n" );
	queueDownlink( EPS_TELEMETRY_ID, &eps_telem_data, sizeof( eps_telem_data ) );
}

void packet_queue_com() {
	PRINTF( "COM packet queued\r\n" );
	queueDownlink( COM_TELEMETRY_ID, &com_telem_data, sizeof( com_telem_data ) );
}

void packet_queue_gnc() {
	PRINTF( "GNC packet queued\r\n" );
	queueDownlink( GNC_TELEMETRY_ID, (fsw_telemetry*)&rtY.fsw_telem, sizeof( rtY.fsw_telem ) );
}

void packet_queue_sen() {
	PRINTF( "SENS packet queued\r\n" );
	queueDownlink( SEN_TELEMETRY_ID, (fsw_telemetry*)&rtU.telecommands, sizeof( rtU.telecommands ) );
}

void packet_queue_image() {
	PRINTF( "IMAGE packet queued\r\n" );
	queueDownlink( IMG_TELEMETRY_ID, &IMG_PICTURE[image_buffer_offset], current_image_size );
}

void packet_queue_logger() {
	PRINTF( "LOGGER packet queued\r\n" );
	queueDownlink( LOGGER_ID, &logger_buffer, log_write );
}

void packet_queue_header() {
	PRINTF( "HEADER packet queued\r\n" );
	queueDownlink( HEADER_TEST_ID, &statement, sizeof( statement ) );
}

bool packet_send_downlink() {
	static int index;
	static long last_send;
	if( num_downlink_used > 0 ) {
		if( eps_getRTCTime() - last_send > 3 ) {
			PRINTF( "Sending downlink packet %i/%i\r\n", index, num_downlink_used );
			com_antenna_send( (uint8_t*)&downlink_data[index], downlink_data[index].length );
			last_send = eps_getRTCTime();
			index++;
			if( index >= num_downlink_used ) {
				num_downlink_used = 0;
				index = 0;
				return false;
			}
		}
		return true;
	}
	return true;
}

//LOGGER METHODS-------------------------------------------------------------------------------
#define CHAROF( number ) ( (char)( number % 10 ) - '0' )
void packet_log( char* statement ) {
	uint16_t offset = log_write;
	if( offset > sizeof( logger_buffer ) - 50 ) {\
		log_write = 0;
		offset = 0;
	}
	unsigned long time = eps_getRTCTime();
	logger_buffer[ offset + 0 ] = '[';
	logger_buffer[ offset + 1 ] = CHAROF( time / 1 );
	logger_buffer[ offset + 2 ] = CHAROF( time / 10 );
	logger_buffer[ offset + 3 ] = CHAROF( time / 100 );
	logger_buffer[ offset + 4 ] = CHAROF( time / 1000 );
	logger_buffer[ offset + 5 ] = CHAROF( time / 10000 );
	logger_buffer[ offset + 6 ] = CHAROF( time / 100000 );
	logger_buffer[ offset + 7 ] = CHAROF( time / 1000000 );
	logger_buffer[ offset + 8 ] = CHAROF( time / 10000000 );
	logger_buffer[ offset + 9 ] = ']';
	offset += 10;
	for( int i = 0; i < 32; i++ ) {
		logger_buffer[ offset++ ] = statement[i];
		if( statement[i] == '\n' ) {
			log_write = offset;
			return;
		}
	}
	PRINTF( "LOGGER DETECTED FAULT\r\n" );
}

//UPLINK METHODS-------------------------------------------------------------------------------
//detects if uplink packets have been received and verifies them. returns true if uplink process can begin
bool packet_detect_uplink() {
	size_t available = SZ_CMD;
	uint8_t* buffer = (uint8_t*)&uplink_data;
	memset( &uplink_data, 0, SZ_CMD );
	com_antenna_receive( buffer, &available );
	if( available != 0 ) {
		PRINTF( "Radio detects potential uplink data\r\n" );
		bool detect_magic = false;
		for( int i = 0; i <= available - sizeof( uint32_t ); i++ ) {
			uint32_t magic = *( (uint32_t*)&buffer[i] );
			PRINTF( "%x ", magic );
			if( magic == UPLINK_MAGIC_NUMBER ) {
				if( i != 0 ) {
					for( int j = 0; j < available; j++ ) {
						buffer[j] = buffer[ i + j ];
					}
				}
				detect_magic = true;
			}
		}
		if( detect_magic ) {
			PRINTF( "Radio detects uplink magic number, starting HMAC check\r\n" );
#if HMAC_ENABLE
			uint8_t compare[32];
			size_t size = hmac_sha256( HMAC_KEY, sizeof( HMAC_KEY ), &uplink_data.data, CMD_LEN, compare, 32 );
			for( int i = 0; i < size; i++ ) {
				if( uplink_data.hmac[i] != compare[i] ) {
					PRINTF( "Uplink packet failed HMAC check, sending NACK\r\n" );
					struct ack_packet packet;
					prepareAckPacket( &packet, COMMAND_HEADER_ID, 0, 0 );
					queueDownlink( ACK_ID, &packet, sizeof( packet ) );
					return false;
				}
			}
#endif
			PRINTF( "Uplink successful, starting packet processing\r\n" );
			return true;
		}
	}
	return false;
}

void packet_process_uplink() {
	struct ack_packet acks;
	PRINTF( "Starting packet processing\r\n" );
	switch( uplink_data.packet_id ) {
		case NO_COMMAND_ID :
			PRINTF( "Uplink packet: no command\r\n" );
			prepareAckPacket( &acks, NO_COMMAND_ID, 0, 0 );
			break;
		case COM_COMMAND_ID :
			PRINTF( "Uplink packet: com command\r\n" );
			struct com_command* com = ( struct com_command* )&uplink_data.data;
			if( com->reconfig_flight_radio == 1 ) {
				prepareAckPacket( &acks, COM_COMMAND_ID, 1, com_configRadio() );
			}
			if( com->check_radio_health == 1 ) {
				prepareAckPacket( &acks, COM_COMMAND_ID, 2, com_healthcheck() );
			}
			if( com->check_antenna_status == 1 ) {
				prepareAckPacket( &acks, COM_COMMAND_ID, 3, com_i2c_checkDeploy() );
			}
			if( com->echo_signal == 1 ) {
				uint8_t b = 0xFF;
				com_antenna_send( &b, 1 );
				prepareAckPacket( &acks, COM_COMMAND_ID, 4, 1 );
			}
			if( com->manually_enter_antenna_algo1 == 1 ) {
				com_deployAntenna_algorithmOne();
				prepareAckPacket( &acks, COM_COMMAND_ID, 5, 1 );
			}
			if( com->manually_enter_antenna_algo2 == 1 ) {
				com_deployAntenna_algorithmTwo();
				prepareAckPacket( &acks, COM_COMMAND_ID, 6, 1 );
			}
			if( com->manually_enter_burn_wire1 == 1 ) {
				com_set_burn_wire1();
				prepareAckPacket( &acks, COM_COMMAND_ID, 7, 1 );
			}
			if( com->manually_enter_burn_wire2 == 1 ) {
				com_set_burn_wire2();
				prepareAckPacket( &acks, COM_COMMAND_ID, 8, 1 );
			}
			break;
		case GNC_COMMAND_ID :
			PRINTF( "Uplink packet: gnc command\r\n" );
			rtU.telecommands = *( ( cdh_data* )&uplink_data.data );
			prepareAckPacket( &acks, GNC_COMMAND_ID, 0, 0 );
			break;
		case IMG_COMMAND_ID :
			PRINTF( "Uplink packet: img command\r\n" );
			struct img_command* img = ( struct img_command* )&uplink_data.data;
			uint32_t offset = img->send_location_offset * TLM_LEN;
			uint8_t flag = img->active_flags;
			if( offset < IMAGE_SIZE ) {
				image_buffer_offset = offset;
			}
			if( ( ( flag >> 0 ) & 1 ) == 1 ) {
				prepareAckPacket( &acks, IMG_COMMAND_ID, CHECK_STATUS,
						image_sendCommand( CHECK_STATUS, img->check_status ) );
			}
			if( ( ( flag >> 1 ) & 1 ) == 1 ) {
				prepareAckPacket( &acks, IMG_COMMAND_ID, TAKE_PICTURE,
						image_sendCommand( TAKE_PICTURE, img->take_picture ) );
			}
			if( ( ( flag >> 2 ) & 1 ) == 1 ) {
				prepareAckPacket( &acks, IMG_COMMAND_ID, GET_PICTURE_SIZE,
						image_sendCommand( GET_PICTURE_SIZE, img->get_picture_size ) );
			}
			if( ( ( flag >> 3 ) & 1 ) == 1 ) {
				prepareAckPacket( &acks, IMG_COMMAND_ID, GET_PICTURE,
						image_getPicture( img->get_picture ) );
			}
			if( ( ( flag >> 4 ) & 1 ) == 1 ) {
				prepareAckPacket( &acks, IMG_COMMAND_ID, SET_CONTRAST,
						image_sendCommand( SET_CONTRAST, img->set_picture_contrast ) );
			}
			if( ( ( flag >> 5 ) & 1 ) == 1 ) {
				prepareAckPacket( &acks, IMG_COMMAND_ID, SET_BRIGHTNESS,
						image_sendCommand( SET_BRIGHTNESS, img->set_picture_brightness ) );
			}
			if( ( ( flag >> 6 ) & 1 ) == 1 ) {
				prepareAckPacket( &acks, IMG_COMMAND_ID, SET_EXPOSURE,
						image_sendCommand( SET_EXPOSURE, img->set_picture_exposure ) );
			}
			if( ( ( flag >> 7 ) & 1 ) == 1 ) {
				prepareAckPacket( &acks, IMG_COMMAND_ID, SET_SLEEP_TIME,
						image_sendCommand( SET_SLEEP_TIME, img->set_sleep_time ) );
			}
			break;
		case RAW_COMMAND_ID :
			PRINTF( "Uplink packet: raw command\r\n" );
			struct raw_command* raw = ( struct raw_command* )&uplink_data.data;
			bool success = false;
			uint8_t sr = raw->send_receive;
			uint8_t handle_id = raw->send_handle;
			uint32_t identifier = raw->identifier;
			uint8_t length = raw->send_length;
			length = length < sizeof( raw->send_buffer ) ? length : sizeof( raw->send_buffer );
			uint8_t recv[ sizeof( raw->send_buffer ) ];
			switch( raw->method_type ) {
				case 1 : //uart
					lpuart_rtos_handle_t* handle_art = 0;
					switch( handle_id ) {
						case 1 : handle_art = &LPUART1_rtos_handle; break;
						case 2 : handle_art = &LPUART3_rtos_handle; break;
						case 4 : handle_art = &LPUART4_rtos_handle; break;
						default :
					}
					if( handle_art != 0 ) {
						if( sr ) {
							success = UART_send( handle_art, (uint8_t*)&( raw->send_buffer ), length );
						} else {
							success = UART_receive( handle_art, (uint8_t*)&recv, length, 0 );
						}
					}
					break;
				case 2 : //i2c
					lpi2c_rtos_handle_t* handle_i2c = 0;
					lpi2c_master_transfer_t* transfer = 0;
					switch( handle_id ) {
						case 1 :
							handle_i2c = &LPI2C1_masterHandle;
							transfer = &LPI2C1_masterTransfer;
							break;
						case 2 :
							handle_i2c = &LPI2C2_masterHandle;
							transfer = &LPI2C2_masterTransfer;
							break;
						case 4 :
							handle_i2c = &LPI2C3_masterHandle;
							transfer = &LPI2C3_masterTransfer;
							break;
						default :
					}
					if( handle_i2c != 0 ) {
						if( sr ) {
							success = I2C_send( handle_i2c, transfer, identifier >> 16, identifier | 0xFF,
									(uint8_t*)&( raw->send_buffer ), length );
						} else {
							success = I2C_send( handle_i2c, transfer, identifier >> 16, identifier | 0xFF,
									(uint8_t*)&recv, length );
						}
					}
					break;
				case 3 : //spi
					success = SPI_transfer( (uint8_t*)&( raw->send_buffer ), (uint8_t*)&recv, length, identifier );
					break;
				case 4 : //gpio
					GPIO_PinWrite( GPIO1, identifier, raw->send_buffer[0] );
					success = true;
					break;
				default :
			}
			prepareAckPacket( &acks, RAW_COMMAND_ID, raw->method_type, success );
			break;
		default :
			prepareAckPacket( &acks, NO_COMMAND_ID, 1, 1 );
	}
	queueDownlink( ACK_ID, &acks, sizeof( acks ) );
	for( int i = 0; i < 8; i++ ) {
		if( ( uplink_data.telem_flags >> i ) | 1 ) {
			switch( i ) {
				case 0 : packet_queue_header(); break;
				case 1 : packet_queue_beacon(); break;
				case 2 : packet_queue_eps(); break;
				case 3 : packet_queue_com(); break;
				case 4 : packet_queue_gnc(); break;
				case 5 : packet_queue_image(); break;
				case 6 : packet_queue_logger(); break;
				default :
			}
		}
	}
}


//____________________________________________________________________________________________________
void debugger_printTelemPacket( struct telemetry_packet* packet ) {
	printf( "Magic Number: %x\r\n", packet->magic_number );
	printf( "Packet ID: %x\r\n", packet->packet_id );
	printf( "Response ID: %x\r\n", packet->response_id );
	printf( "Location: %i\r\n", packet->byte_location );
	printf( "Checksum: %x\r\n", packet->checksum );
	printf( "Packet: \"" );
	printf( (char*)&( packet->data ) );
	printf( "\"\n" );
}

void debugger_printArray( char* name, real_T buffer[], int size ) {
	printf( name );
	printf( ": [" );
	for( int i = 0; i < size; i++ ) {
		printf( "%d", (int)( buffer[i] * 1000 ) );
		if( i != size - 1 ) {
			printf( ", " );
		}
	}
	printf( " ]\r\n" );
}
void debugger_printGNCPacket( fsw_telemetry* packet ) {
	printf( "gnc_mode: %i\r\n", packet->gnc_mode );
	debugger_printArray( "sc_quat", packet->sc_quat, 4 );
	debugger_printArray( "sc_body_rates_radps", packet->sc_body_rates_radps, 3 );
	debugger_printArray( "cmd_quat", packet->cmd_quat, 4 );
	debugger_printArray( "cmd_body_rates_radps", packet->cmd_body_rates_radps, 3 );
	debugger_printArray( "mekf_3sigma_rad", packet->mekf_3sigma_rad, 6 );
	debugger_printArray( "mekf_bias_radps", packet->mekf_bias_radps, 3 );
	printf( "mekf_telem: %i\r\n", packet->mekf_telem );
	debugger_printArray( "r_eci_m", packet->r_eci_m, 3 );
	printf( "ace_err: %d\r\n", (int)( packet->ace_err * 1000 ) );
	printf( "eclipse: %i\r\n", packet->eclipse );
	printf( "sgp4_flag: %i\r\n", packet->sgp4_flag );
	printf( "sc_above_gs: %i\r\n", packet->sc_above_gs );
	printf( "sc_above_targ: %i\r\n", packet->sc_above_targ );
	printf( "elev_gs_rad: %d\r\n", (int)( packet->elev_gs_rad * 1000 ) );
	printf( "elev_targ_rad: %d\r\n", (int)( packet->elev_targ_rad * 1000 ) );
	printf( "ss_valid: %i\r\n", packet->ss_valid );
}

void debugger_printCommandPacket() {
	struct command_packet packet;
	memset( &packet, 0, SZ_CMD );
	//user inputs-------------------------
	//NO_COMMAND_ID COMMAND_HEADER_ID COM_COMMAND_ID GNC_COMMAND_ID IMG_COMMAND_ID RAW_COMMAND_ID
	packet.packet_id = COM_COMMAND_ID;
	packet.response_id = 0x3300;
	// header 1, beacon 2, eps 4, com 8 gnc 16, image 32, logger 64
	packet.telem_flags = ( 2 | 16 | 64 );
	//data------------------------------
	struct com_command data_packet;
	memset( &data_packet, 0, sizeof( data_packet ) );
	data_packet.reconfig_flight_radio = 0;
	data_packet.check_radio_health = 1;
	data_packet.check_antenna_status = 1;
	data_packet.echo_signal = 1;
	data_packet.manually_enter_antenna_algo1 = 0;
	data_packet.manually_enter_antenna_algo2 = 0;
	data_packet.manually_enter_burn_wire1 = 0;
	data_packet.manually_enter_burn_wire2 = 0;
	//processing-------------------------
	packet.magic_number = UPLINK_MAGIC_NUMBER;
	memcpy( &packet.data, &data_packet, sizeof( data_packet ) );
	hmac_sha256( HMAC_KEY, sizeof( HMAC_KEY ), &packet.data, CMD_LEN, &packet.hmac, 32 );
	//print output-------------------------
	PRINTF( "UPLINK PACKET:\r\n" );
	uint8_t* ptr = (uint8_t*)&packet;
	for( int i = 0; i < SZ_CMD; i++ ) {
		PRINTF( "%x ", ptr[i] );
	}
	PRINTF( "\r\n" );
}

