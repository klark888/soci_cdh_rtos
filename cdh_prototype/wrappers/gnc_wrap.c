#include "peripherals.h"
#include "fsl_debug_console.h"
#include "com_protocol_helper.h"
#include "FSW_Lib0.h"
#include "rtwtypes.h"
#include "zero_crossing_types.h"
#include <stddef.h>

#include "mag_wrap.h"
#include "gyro_wrap.h"
#include "phd_wrap.h"
#include "mtq_wrap.h"
#include "rwa_wrap.h"
#include "sun_wrap.h"
#include "eps_wrap.h"
#include "packet_wrap.h"

void gnc_initIO() {
	PRINTF( "Initializing PWM...\n" );
	initPWM( &mtq, &LPI2C1_masterHandle );
	PRINTF( "Initializing PHD...\n" );
	quickStartPhd( &Phd1, &LPI2C2_masterHandle );
	PRINTF( "Initializing Gyro...\n" );
	startGyro( &Gyro1, &LPI2C1_masterHandle, &LPI2C1_masterTransfer );
	startGyro( &Gyro2, &LPI2C2_masterHandle, &LPI2C2_masterTransfer );
	//*startGyro( &Gyro3, &LPI2C3_masterHandle, &LPI2C3_masterTransfer );
	PRINTF( "Initializing Mags...\n" );
	quickStartMag( &Mag1, &LPI2C1_masterHandle );
	quickStartMag( &Mag2, &LPI2C2_masterHandle );
	//*quickStartMag( &Mag3, &LPI2C3_masterHandle );
	PRINTF( "Initializing RW...\n" );
	SPI_GPIO_init();
	rw0.reqClcMode = 0;
	rw1.reqClcMode = 0;
	rw2.reqClcMode = 0;
	rw3.reqClcMode = 0;
	commandRW( 7, &rw0, RWA0 );
	commandRW( 7, &rw1, RWA1 );
	commandRW( 7, &rw2, RWA2 );
	commandRW( 7, &rw3, RWA3 );
}

void gnc_initModel() {
	//initial obc state
	rtU.telecommands.MET_utc_s = initialize_time;
	rtU.telecommands.telecom[0] = 0;
	rtU.telecommands.telecom[1] = 0;
	rtU.telecommands.telecom[2] = 0;
	rtU.telecommands.telecom[3] = 1;
	rtU.telecommands.telecom[4] = 0;
	rtU.telecommands.telecom[5] = 1;
	//initial two line orbital element
	rtU.telecommands.orbit_tle[0] = 24;
	rtU.telecommands.orbit_tle[1] = initialize_time;
	rtU.telecommands.orbit_tle[2] = 0.00017033;
	rtU.telecommands.orbit_tle[3] = 97.5508;
	rtU.telecommands.orbit_tle[4] = 251.3052;
	rtU.telecommands.orbit_tle[5] = 0.0017553;
	rtU.telecommands.orbit_tle[6] = 52.9833;
	rtU.telecommands.orbit_tle[7] = 307.2997;
	rtU.telecommands.orbit_tle[8] = 15.0695111;
	//intial command to execute
	rtU.telecommands.MET_soar_utc_s = initialize_time + 864000;
	rtU.telecommands.quat_soar_cmd[0] = 0.2008693887;
	rtU.telecommands.quat_soar_cmd[1] = -0.6036003117;
	rtU.telecommands.quat_soar_cmd[2] = -0.7407817115;
	rtU.telecommands.quat_soar_cmd[3] = -0.2157790728;
	rtU.telecommands.target_latlonalt[0] = 35.672792;
	rtU.telecommands.target_latlonalt[1] = 136.690816;
	rtU.telecommands.target_latlonalt[2] = 0;
	rtU.telecommands.triad_override = false;
	//run model init
	FSW_Lib0_initialize();
}

void gnc_cdhInput() {
	//PRINTF( "GNC reading CDH\n" );
	// update current time
	rtU.telecommands.MET_utc_s = eps_getRTCTime();
	// enforce cdh modes
	switch( operatingMode ) {
		case CRIT_LOW_POWER : //CLPM
			rtU.telecommands.telecom[1] = 1;
			rtU.telecommands.telecom[2] = 0;
			break;
		case LOW_POWER : //LPM
			rtU.telecommands.telecom[1] = 0;
			rtU.telecommands.telecom[2] = 1;
			break;
		case NOMINAL_POWER : //Nominal
			rtU.telecommands.telecom[1] = 0;
			rtU.telecommands.telecom[2] = 0;
			break;
		default :
	}
}

void gnc_sensorInput() {
	PRINTF( "GNC sun sensor\n" );
	getSunAngles( &Sun1 );
	rtU.sensor_meas.sun_meas_ss_deg[0] = Sun1.angles[0];
	rtU.sensor_meas.sun_meas_ss_deg[1] = Sun1.angles[1];
	rtU.sensor_meas.sun_meas_valid = Sun1.isValid;
	PRINTF( "GNC reading PHD\n" );
	readPhdData( &Phd1 );
	rtU.sensor_meas.photodiodes_uA[0] = Phd1.current[0];
	rtU.sensor_meas.photodiodes_uA[1] = Phd1.current[1];
	rtU.sensor_meas.photodiodes_uA[2] = Phd1.current[2];
	rtU.sensor_meas.photodiodes_uA[3] = Phd1.current[3];
	rtU.sensor_meas.photodiodes_uA[4] = Phd1.current[4];
	PRINTF( "GNC reading GYRO\n" );
	readGyroData( &Gyro1 );
	readGyroData( &Gyro2 );
	//readGyroData( &Gyro3 );
	rtU.sensor_meas.gyro_gyro_radps[0] = Gyro1.gyroXYZ[0];
	rtU.sensor_meas.gyro_gyro_radps[1] = Gyro1.gyroXYZ[1];
	rtU.sensor_meas.gyro_gyro_radps[2] = Gyro1.gyroXYZ[2];
	rtU.sensor_meas.gyro_gyro_radps[3] = Gyro2.gyroXYZ[0];
	rtU.sensor_meas.gyro_gyro_radps[4] = Gyro2.gyroXYZ[1];
	rtU.sensor_meas.gyro_gyro_radps[5] = Gyro2.gyroXYZ[2];
	rtU.sensor_meas.gyro_gyro_radps[6] = Gyro3.gyroXYZ[0];
	rtU.sensor_meas.gyro_gyro_radps[7] = Gyro3.gyroXYZ[1];
	rtU.sensor_meas.gyro_gyro_radps[8] = Gyro3.gyroXYZ[2];
	rtU.sensor_meas.gyro_meas_valid[0] = 1;
	rtU.sensor_meas.gyro_meas_valid[1] = 1;
	rtU.sensor_meas.gyro_meas_valid[2] = 0;
	PRINTF( "GNC reading MTQ\n" );
	readMagData( &Mag1 );
	readMagData( &Mag2 );
	//readMagData( &Mag3 );
	rtU.sensor_meas.mag_mag_uT[0] = Mag1.magXYZ[0];
	rtU.sensor_meas.mag_mag_uT[1] = Mag1.magXYZ[1];
	rtU.sensor_meas.mag_mag_uT[2] = Mag1.magXYZ[2];
	rtU.sensor_meas.mag_mag_uT[3] = Mag2.magXYZ[0];
	rtU.sensor_meas.mag_mag_uT[4] = Mag2.magXYZ[1];
	rtU.sensor_meas.mag_mag_uT[5] = Mag2.magXYZ[2];
	rtU.sensor_meas.mag_mag_uT[6] = Mag3.magXYZ[0];
	rtU.sensor_meas.mag_mag_uT[7] = Mag3.magXYZ[1];
	rtU.sensor_meas.mag_mag_uT[8] = Mag3.magXYZ[2];
	rtU.sensor_meas.mag_meas_valid[0] = 1;
	rtU.sensor_meas.mag_meas_valid[1] = 1;
	rtU.sensor_meas.mag_meas_valid[2] = 0;
	PRINTF( "GNC reading RWA\n" );
	commandRW( 4, &rw0, RWA0 );
	commandRW( 4, &rw1, RWA1 );
	commandRW( 4, &rw2, RWA2 );
	commandRW( 4, &rw3, RWA3 );
	rtU.actuator_meas_p.rwa_rpm[0] = rw0.currSpeed;
	rtU.actuator_meas_p.rwa_rpm[1] = rw1.currSpeed;
	rtU.actuator_meas_p.rwa_rpm[2] = rw2.currSpeed;
	rtU.actuator_meas_p.rwa_rpm[3] = rw3.currSpeed;
	rtU.actuator_meas_p.rwa_valid[0] = 1;
	rtU.actuator_meas_p.rwa_valid[1] = 1;
	rtU.actuator_meas_p.rwa_valid[2] = 1;
	rtU.actuator_meas_p.rwa_valid[3] = 1;
}

void gnc_rt_OneStep() {
  //PRINTF( "GNC stepping\r\n" );
  static boolean_T OverrunFlags[2] = { 0, 0 };
  static boolean_T eventFlags[2] = { 0, 0 };/* Model has 2 rates */
  static int_T taskCounter[2] = { 0, 0 };

  /* Disable interrupts here */
  /* Check base rate for overrun */
  if (OverrunFlags[0]) {
    rtmSetErrorStatus(rtM, "Overrun");
    return;
  }

  OverrunFlags[0] = true;
  /* Save FPU context here (if necessary) */
  /* Re-enable timer or interrupt here */
  /*
   * For a bare-board target (i.e., no operating system), the
   * following code checks whether any subrate overruns,
   * and also sets the rates that need to run this time step.
   */
  if (taskCounter[1] == 0) {
    if (eventFlags[1]) {
      OverrunFlags[0] = false;
      OverrunFlags[1] = true;

      /* Sampling too fast */
      rtmSetErrorStatus(rtM, "Overrun");
      return;
    }

    eventFlags[1] = true;
  }

  taskCounter[1]++;
  if (taskCounter[1] == 20) {
    taskCounter[1]= 0;
  }

  /* Set model inputs associated with base rate here */
  /* Step the model for base rate */
  FSW_Lib0_step0();

  /* Get model outputs here */
  /* Indicate task for base rate complete */
  OverrunFlags[0] = false;

  /* If task 1 is running, don't run any lower priority task */
  if (OverrunFlags[1]) {
    return;
  }

  /* Step the model for subrate */
  if (eventFlags[1]) {
    OverrunFlags[1] = true;
    /* Set model inputs associated with subrates here */

    /* Step the model for subrate 1 */
    FSW_Lib0_step1();

    /* Get model outputs here */

    /* Indicate task complete for subrate */
    OverrunFlags[1] = false;
    eventFlags[1] = false;
  }

  /* Disable interrupts here */
  /* Restore FPU context here (if necessary) */
  /* Enable interrupts here */
}

void gnc_sensorOutput() {
	//PRINTF( "GNC writing RWA\n" );
	rw0.reqSpeed = rtY.fsw_out_l.rwa_cmd_rpm[0];
	rw1.reqSpeed = rtY.fsw_out_l.rwa_cmd_rpm[1];
	rw2.reqSpeed = rtY.fsw_out_l.rwa_cmd_rpm[2];
	rw3.reqSpeed = rtY.fsw_out_l.rwa_cmd_rpm[3];
	commandRW( 6, &rw0, RWA0 );
	commandRW( 6, &rw1, RWA1 );
	commandRW( 6, &rw2, RWA2 );
	commandRW( 6, &rw3, RWA3 );
	//PRINTF( "GNC writing MTQ\n" );
	float x = rtY.fsw_out_l.mtq_cmd_Am2[0] - rtY.fsw_out_l.mtq_cmd_Am2[1];
	float y = rtY.fsw_out_l.mtq_cmd_Am2[2] - rtY.fsw_out_l.mtq_cmd_Am2[3];
	float z = rtY.fsw_out_l.mtq_cmd_Am2[4];
	setMoments( x, y, z, &mtq );
}

void gnc_cdhOutput() {
	//PRINTF( "GNC writing CDH\n" );
	static bool is_above_gs, is_above_target;
	static uint8_t prev_mode;
	//log data
	if( is_above_gs && !rtY.fsw_telem.sc_above_gs ) {
		packet_log( "GNC leaving GS\n" );
		PRINTF( "GNC leaving GS\r\n" );
	}
	if( !is_above_gs && rtY.fsw_telem.sc_above_gs ) {
		packet_log( "GNC entering GS\n" );
		PRINTF( "GNC entering GS\r\n" );
	}
	if( is_above_target && !rtY.fsw_telem.sc_above_targ ) {
		packet_log( "GNC leaving target\n" );
		PRINTF( "GNC leaving target\r\n" );
	}
	if( !is_above_target && rtY.fsw_telem.sc_above_targ ) {
		packet_log( "GNC entering target\n" );
		PRINTF( "GNC entering target\r\n" );
		}
	if( rtY.fsw_telem.gnc_mode != prev_mode ) {
		packet_log( "GNC mode changed\n" );
		PRINTF( "GNC mode: %i\n", rtY.fsw_telem.gnc_mode );
	}
	//update comparison flags
	prev_mode = rtY.fsw_telem.gnc_mode;
	is_above_gs = rtY.fsw_telem.sc_above_gs;
	is_above_target = rtY.fsw_telem.sc_above_targ;
}
