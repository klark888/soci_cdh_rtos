#include <stdbool.h>
#include "eps_wrap.h"
#include "peripherals.h"
#include "com_protocol_helper.h"
#include "fsl_debug_console.h"

#define EPS_SLAVE_ADDR 0x51
#define RTC_SLAVE_ADDR 0x68
#define MTQ ( 1 << 0 )
#define RWA ( 1 << 1 )
#define IMG ( 1 << 2 )
#define COM ( 1 << 3 )
#define SEN ( 1 << 4 )
#define EPS_WRITE( subaddress, parameter ) writeReg( EPS_SLAVE_ADDR, subaddress, parameter )
#define EPS_READ( reg ) readReg( EPS_SLAVE_ADDR, reg, 2 )
#define RTC_READ( reg ) readReg( RTC_SLAVE_ADDR, reg, 1 )
#define RTC_TODEC( val ) ( ( val >> 4 ) * 10 + ( val & 0xF ) )

//flag for operating mode
volatile int operatingMode;
//timekeeping
volatile unsigned long initialize_time;
//telem
struct eps_telemetry eps_telem_data;

static void writeReg( uint8_t address, uint8_t subaddress, uint8_t parameter ) {
	I2C_send( &LPI2C1_masterHandle, &LPI2C1_masterTransfer, address, subaddress, &parameter, 1 );
}

static uint32_t readReg( uint8_t address, uint8_t subaddress, uint8_t count ) {
	uint32_t receive = 0;
	I2C_request( &LPI2C1_masterHandle, &LPI2C1_masterTransfer, address, subaddress, (uint8_t*)&receive, count );
	return receive;
}

static int twosComp( uint16_t x ) {
	return ( 1 + ~(unsigned)x ) & 0xFF; //TODO: Check this!
}

static void getTelemetryGroup( uint8_t family, uint8_t* recv_buffer, uint8_t size ) {
	I2C_variableRequest( &LPI2C1_masterHandle, &LPI2C1_masterTransfer, EPS_SLAVE_ADDR, 0x0B00 | family, 2, recv_buffer, size );
}

void eps_updateTelemtry() {
	uint8_t data[24];
	getTelemetryGroup( 0, (uint8_t*)&data, 24 );
	eps_telem_data.BCR8W_1_InputVoltage = ( float32_t )( ((data[1] << 8) | data[0]) * 0.008 );
	eps_telem_data.BCR8W_1_InputCurrent = ( float32_t )( ((data[3] << 8) | data[2]) * 2 );
	eps_telem_data.BCR8W_1_OutputVoltage = ( float32_t )( ((data[5] << 8) | data[4]) * 0.008 );
	eps_telem_data.BCR8W_1_OutputCurrent = ( float32_t )( ((data[7] << 8) | data[6]) * 2 );
	eps_telem_data.BCR8W_2_InputVoltage = ( float32_t )( ((data[9] << 8) | data[8]) * 0.008 );
	eps_telem_data.BCR8W_2_InputCurrent = ( float32_t )( ((data[11] << 8) | data[10]) * 2 );
	eps_telem_data.BCR8W_2_OutputVoltage = ( float32_t )( ((data[13] << 8) | data[12]) * 0.008 );
	eps_telem_data.BCR8W_2_OutputCurrent = ( float32_t )( ((data[15] << 8) | data[14]) * 2 );
	eps_telem_data.BCR3W_InputVoltage = ( float32_t )( ((data[17] << 8) | data[16]) * 0.008 );
	eps_telem_data.BCR3W_InputCurrent = ( float32_t )( ((data[19] << 8) | data[18]) * 5 );
	eps_telem_data.BCR3W_OutputVoltage = ( float32_t )( ((data[21] << 8) | data[20]) * 0.008 );
	eps_telem_data.BCR3W_OutputCurrent = ( float32_t )( ((data[23] << 8) | data[22]) * 2 );
	getTelemetryGroup( 1, (uint8_t*)&data, 10 );
	eps_telem_data.M_SP_TemperatureXPlus = ( float32_t )( twosComp((data[1] << 8) | data[0]) * 0.5 );
	eps_telem_data.M_SP_TemperatureXMinus = ( float32_t )( twosComp((data[3] << 8) | data[2]) * 0.5 );
	eps_telem_data.M_SP_TemperatureYPlus = ( float32_t )( twosComp((data[5] << 8) | data[4]) * 0.5 );
	eps_telem_data.M_SP_TemperatureYMinus = ( float32_t )( twosComp((data[7] << 8) | data[6]) * 0.5 );
	eps_telem_data.M_SP_TemperatureZPlus = ( float32_t )( twosComp((data[9] << 8) | data[8]) * 0.5 );
	getTelemetryGroup( 2, (uint8_t*)&data, 24 );
	eps_telem_data.OutputVoltageBCR = ( float32_t )( ((data[1] << 8) | data[0]) * 0.0030945 );
	eps_telem_data.OutputCurrentBCR = ( float32_t )( ((data[3] << 8) | data[2]) * 0.0020676 );
	eps_telem_data.InputVoltagePCM = ( float32_t )( ((data[5] << 8) | data[4]) * 0.0030945 );
	eps_telem_data.InputCurrentPCM = ( float32_t )( ((data[7] << 8) | data[6]) * 0.0020676 );
	eps_telem_data.PowerBusVoltage3V3 = ( float32_t )( ((data[9] << 8) | data[8]) * 0.0030945 );
	eps_telem_data.PowerBusCurrent3V3 = ( float32_t )( ((data[11] << 8) | data[10]) * 0.0020676 );
	eps_telem_data.PowerBusVoltage5V = ( float32_t )( ((data[13] << 8) | data[12]) * 0.0030945 );
	eps_telem_data.PowerBusCurrent5V = ( float32_t )( ((data[15] << 8) | data[14]) * 0.0020676 );
	eps_telem_data.PowerBusVoltageVbat = ( float32_t )( ((data[17] << 8) | data[16]) * 0.0030945 );
	eps_telem_data.PowerBusCurrentVbat = ( float32_t )( ((data[19] << 8) | data[18]) * 0.0020676 );
	eps_telem_data.PowerBusVoltage12V = ( float32_t )( ((data[21] << 8) | data[20]) * 0.0030945 );
	eps_telem_data.PowerBusCurrent12V = ( float32_t )( ((data[23] << 8) | data[22]) * 0.0020676 );
	getTelemetryGroup( 3, (uint8_t*)&data, 24 );
	eps_telem_data.SW1_V = ( float32_t )( ((data[1] << 8) | data[0]) * 0.0030945 );
	eps_telem_data.SW1_C = ( float32_t )( ((data[3] << 8) | data[2]) * 0.0008336 - 0.010 );
	eps_telem_data.SW2_V = ( float32_t )( ((data[5] << 8) | data[4]) * 0.0030945 );
	eps_telem_data.SW2_C = ( float32_t )( ((data[7] << 8) | data[6]) * 0.0008336 - 0.010 );
	eps_telem_data.SW3_V = ( float32_t )( ((data[9] << 8) | data[8]) * 0.0030945 );
	eps_telem_data.SW3_C = ( float32_t )( ((data[11] << 8) | data[10]) * 0.0008336 - 0.010 );
	eps_telem_data.SW4_V = ( float32_t )( ((data[13] << 8) | data[12]) * 0.0030945 );
	eps_telem_data.SW4_C = ( float32_t )( ((data[15] << 8) | data[14]) * 0.0008336 - 0.010 );
	eps_telem_data.SW5_V = ( float32_t )( ((data[17] << 8) | data[16]) * 0.0030945 );
	eps_telem_data.SW5_C = ( float32_t )( ((data[19] << 8) | data[18]) * 0.0008336 - 0.010 );
	eps_telem_data.SW6_V = ( float32_t )( ((data[21] << 8) | data[20]) * 0.0030945 );
	eps_telem_data.SW6_C = ( float32_t )( ((data[23] << 8) | data[22]) * 0.0008336 - 0.010 );
	getTelemetryGroup( 4, (uint8_t*)&data, 16 );
	eps_telem_data.VBAT_1 = ((data[1] << 8) | data[0]) * 4.883;
	eps_telem_data.VBAT_2 = ((data[3] << 8) | data[2]) * 4.883;
	eps_telem_data.IBAT = twosComp((data[5] << 8) | data[4]) * 0.26;
	eps_telem_data.BAT_TEMP = twosComp((data[7] << 8) | data[6]) * 0.1;
	eps_telem_data.PCB_TEMP = twosComp((data[9] << 8) | data[8]) * 0.125;
	eps_telem_data.AvailableCapacity = ((data[11] << 8) | data[10]) * 1.6;
	eps_telem_data.RemainingCapacity = ((data[13] << 8) | data[12]);
	eps_telem_data.AccumulatedBatteryCurrent = ((data[15] << 8) | data[14]) * 1.042;
	getTelemetryGroup( 5, (uint8_t*)&data, 12 );
	eps_telem_data.WatchdogPeriod = ((data[1] << 8) | data[0]);
	eps_telem_data.PDMsInitialState = ((data[3] << 8) | data[2]);
	eps_telem_data.PDMsState = ((data[5] << 8) | data[4]);
	eps_telem_data.HousekeepingPeriod = ((data[7] << 8) | data[6]);
	eps_telem_data.BatteryHeaterStatus = ((data[9] << 8) | data[8]);
	eps_telem_data.SafetyHazardEnvironment = ((data[11] << 8) | data[10]);
}

void eps_configEPS() {
	PRINTF( "Configuring EPS\r\n" );
	//PRINTF( "Setting EPS Watchdog to 4min...\r\n" );
	EPS_WRITE( 0x05, 0x04 );
	//PRINTF( "Set Initial PDM States ALL ACTIVE...\r\n" );
	EPS_WRITE( 0x06, 0xFF );
	//PRINTF( "Resetting PDM States...\r\n" );
	EPS_WRITE( 0x07, 0xFF );
	//PRINTF( "Setting Housekeeping 1min...\r\n" );
	EPS_WRITE( 0x09, 0x01 );
	//PRINTF( "Disabling Safety Hazard Environment...\r\n" );
	EPS_WRITE( 0x0A, 0xFF );
}

int eps_currentPowerMode() {
	uint16_t status = EPS_READ( 0x02 );
	if( ( status & ( 1 << 4 ) ) == ( 1 << 4 ) ) {
		PRINTF( "EPS Battery Status: Critical Low Power Mode\r\n" );
		return CRIT_LOW_POWER;
	}
	PRINTF( "EPS Battery Status: Nominal Mode\r\n" );
	return NOMINAL_POWER;
}

void eps_healthcheck() {
	PRINTF( "Checking EPS health\r\n" );
	uint16_t status = EPS_READ( 0x01 );
	if( ( status & 0xFF ) != 0 ) {
		PRINTF( "Power bus has errors, resetting bus.\r\n" );
		//EPS_WRITE( 0xFE, 0xFF );
	}
	if( ( status & 0xFF00 ) != 0 ) {
		PRINTF( "PDM has errors, reconfiguring EPS\r\n" );
		//eps_configEPS();
	}
	PRINTF( "Verifying ID Register...\r\n" );
	if( EPS_READ( 0x04 ) != 0x67D7 ) {
		PRINTF( "ID Register does not match, resetting\r\n" );
		//EPS_WRITE( 0xFF, 0xFF );
	}
}

void eps_pdmSwitch( bool mtq, bool rwa, bool img, bool com, bool sen ) {
	uint8_t pdms = mtq ? MTQ : 0;
	pdms |= rwa ? RWA : 0;
	pdms |= img ? IMG : 0;
	pdms |= com ? COM : 0;
	pdms |= sen ? SEN : 0;
	EPS_WRITE( 0x08, pdms );
}

unsigned long eps_getRTCTime() {
	unsigned long time = 0;
	uint8_t read = RTC_READ( 0 );//seconds
	time += RTC_TODEC( read );
	read = RTC_READ( 1 );
	time += RTC_TODEC( read ) * 60;//minutes
	read = RTC_READ( 2 );
	time += RTC_TODEC( read ) * 3600;//hours
	read = RTC_READ( 4 );
	time += RTC_TODEC( read ) * 86400;//days
	read = RTC_READ( 5 );
	uint8_t month = RTC_TODEC( read );//month
	switch( month ) {
		case 12 : time += 30 * 86400;
		case 11 : time += 31 * 86400;
		case 10 : time += 30 * 86400;
		case  9 : time += 31 * 86400;
		case  8 : time += 31 * 86400;
		case  7 : time += 30 * 86400;
		case  6 : time += 31 * 86400;
		case  5 : time += 30 * 86400;
		case  4 : time += 31 * 86400;
		case  3 : time += 28 * 86400;
		case  2 : time += 31 * 86400;
		case  1 : default :
	}
	read = RTC_READ( 6 );
	uint8_t year = RTC_TODEC( read );//year
	time += year * 365 * 86400;
	time += year / 4 * 86400;//leap year
	if( year % 4 == 0 && month < 3 ) {
		time -= 86400;
	}
	if( time == 0 ) {//assertion
		for(;;);
	}
	return time;
}
