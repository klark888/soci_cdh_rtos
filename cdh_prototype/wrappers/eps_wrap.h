#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"

enum PowerMode {
	CRIT_LOW_POWER = 0,
	LOW_POWER = 1,
	NOMINAL_POWER = 2
};

struct eps_telemetry {
	float32_t BCR8W_1_InputVoltage;
	float32_t BCR8W_1_InputCurrent;
	float32_t BCR8W_1_OutputVoltage;
	float32_t BCR8W_1_OutputCurrent;
	float32_t BCR8W_2_InputVoltage;
	float32_t BCR8W_2_InputCurrent;
	float32_t BCR8W_2_OutputVoltage;
	float32_t BCR8W_2_OutputCurrent;
	float32_t BCR3W_InputVoltage;
	float32_t BCR3W_InputCurrent;
	float32_t BCR3W_OutputVoltage;
	float32_t BCR3W_OutputCurrent;
	float32_t M_SP_TemperatureXPlus;
	float32_t M_SP_TemperatureXMinus;
	float32_t M_SP_TemperatureYPlus;
	float32_t M_SP_TemperatureYMinus;
	float32_t M_SP_TemperatureZPlus;
	float32_t OutputVoltageBCR;
	float32_t OutputCurrentBCR;
	float32_t InputVoltagePCM;
	float32_t InputCurrentPCM;
	float32_t PowerBusVoltage3V3;
	float32_t PowerBusCurrent3V3;
	float32_t PowerBusVoltage5V;
	float32_t PowerBusCurrent5V;
	float32_t PowerBusVoltageVbat;
	float32_t PowerBusCurrentVbat;
	float32_t PowerBusVoltage12V;
	float32_t PowerBusCurrent12V;
	float32_t SW1_V;
	float32_t SW1_C;
	float32_t SW2_V;
	float32_t SW2_C;
	float32_t SW3_V;
	float32_t SW3_C;
	float32_t SW4_V;
	float32_t SW4_C;
	float32_t SW5_V;
	float32_t SW5_C;
	float32_t SW6_V;
	float32_t SW6_C;
	float32_t VBAT_1;
	float32_t VBAT_2;
	float32_t IBAT;
	float32_t BAT_TEMP;
	float32_t PCB_TEMP;
	float32_t AvailableCapacity;
	float32_t RemainingCapacity;
	float32_t AccumulatedBatteryCurrent;
	uint32_t WatchdogPeriod;
	uint32_t PDMsInitialState;
	uint32_t PDMsState;
	uint32_t HousekeepingPeriod;
	uint32_t BatteryHeaterStatus;
	uint32_t SafetyHazardEnvironment;
};

//flag for operating mode
extern volatile int operatingMode;
//timekeeping
extern volatile unsigned long initialize_time;
//telemetry eps
extern struct eps_telemetry eps_telem_data;

unsigned long eps_getRTCTime();
void eps_configEPS();
void eps_healthcheck();
void eps_pdmSwitch( bool mtq, bool rwa, bool img, bool com, bool sen );
void eps_updateTelemtry();
int eps_currentPowerMode();
