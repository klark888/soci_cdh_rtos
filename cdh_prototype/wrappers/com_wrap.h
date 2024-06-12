#include "arm_math.h"
#include "fsl_common.h"

struct __attribute__((__packed__)) com_telemetry {
	uint32_t radio_config_status;
	uint32_t com_system_health_status;
	uint32_t com_antenna_i2c;
	uint32_t is_command_mode;
};

extern struct com_telemetry com_telem_data;

//com configuration commands
bool com_configRadio();
//com radio status check functions
bool com_healthcheck();
bool com_i2c_checkDeploy();
//com radio send and receive functions
void com_antenna_send( uint8_t* buffer, size_t size );
void com_antenna_receive( uint8_t* buffer, size_t* actual_size );
//com radio initialization functions
void com_deployAntenna_algorithmOne();
void com_deployAntenna_algorithmTwo();
void com_set_burn_wire1();
void com_set_burn_wire2();
