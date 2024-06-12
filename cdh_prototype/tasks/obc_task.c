#include "obc_task.h"
#include "com_wrap.h"
#include "eps_wrap.h"
#include "img_wrap.h"
#include "gnc_wrap.h"
#include "packet_wrap.h"
#include "fsl_rtwdog.h"
#include "fsl_semc.h"
#include "fsl_debug_console.h"
#include "lpm.h"
#include "task.h"
#include "specific.h"
#include "semc_sdram.h"
#include "peripherals.h"
#include "power_mode_switch.h"
#include "FSW_Lib0.h"
#include <stdint.h>
#include <time.h>
#include <stdbool.h>
#include <time.h>

#define IDLE_PERIOD          10 * CLOCKS_PER_SEC    //10sec
#define GNC_PERIOD                CLOCKS_PER_SEC / 4//250ms
#define DEPLOY_PERIOD        20 * CLOCKS_PER_SEC    //20sec
#define DETUMBLE_PERIOD CLOCKS_PER_SEC//45 * 60 * CLOCKS_PER_SEC    //45 minutes
#define UNIX_TO_UTC   946684800
#define IGNORE_TIMER 1721001600 - UNIX_TO_UTC
#define DEPLOY_START 1725148800 - UNIX_TO_UTC
#define DEPLOY_END   1725494400 - UNIX_TO_UTC

enum current_states { INIT, NORMAL, DEPLOY, DOWNLINK };

void obc_task( void *pvParameters ) {
	packet_log( "OBC task initialized\n" );
	PRINTF( "OBC task initialization\r\n" );
	//init states
	s_curRunMode = LPM_PowerModeOverRun;
	operatingMode = NOMINAL_POWER;
	int current_state = INIT;
	//init config
	LPM_Init( s_curRunMode );
	initialize_time = eps_getRTCTime();
	//init variables
	unsigned int attempted_deploy = 0;
	bool pic_not_taken = true;
	clock_t last_idle = 0;
	clock_t last_gnc = 0;
	clock_t last_deploy = 0;
	unsigned int current_cycle = 0xFFFFFFFF;
	//for loop
	for(;;) {
		switch( current_state ) {//Transitions
			case INIT:
				//delay 45 minutes
				packet_log( "OBC waiting for detumble\n" );
				if( initialize_time > IGNORE_TIMER ) {
					packet_log( "OBC skipping detumble\n" );
					PRINTF( "Launch date passed, ignoring timer\r\n" );
				} else {
					PRINTF( "Waiting %i seconds for detumble\r\n", DETUMBLE_PERIOD / CLOCKS_PER_SEC );
					clock_t detumble_start = clock();
					while( clock() - detumble_start < DETUMBLE_PERIOD ) {
						RTWDOG_Refresh( RTWDOG );
						busy_delay( 1000 );
						PRINTF( "Awaiting detumble at %d seconds\r\n", clock() - detumble_start );
					}
				}
				packet_log( "OBC configuring peripherals\n" );
				PRINTF( "Detumble complete\r\n" );
				eps_configEPS();
				com_configRadio();
				gnc_initIO();
				gnc_initModel();
				packet_log( "OBC entering normal routine\n" );
				current_state = NORMAL;
				break;

			case NORMAL:
				if( clock() - last_idle > IDLE_PERIOD ) {
					current_cycle++;
					last_idle = clock();
					PRINTF( "Completing nominal checks, cycle: %d, at time %d\r\n", current_cycle, last_idle );
					if( current_cycle % 12 == 0 ) {//EPS
						eps_healthcheck();
						eps_updateTelemtry();
						eps_currentPowerMode();
					}
					if( current_cycle % 24 == 0 && !com_healthcheck() ) {//COM
						packet_log( "COM health check failed\n" );
						com_configRadio();
					}
					if( rtY.fsw_telem.sc_above_targ ) {//IMAGE
						if( pic_not_taken ) {
							packet_log( "COM taking picture\n" );
							image_sendCommand( TAKE_PICTURE, 1 );
							image_getPicture( 1 );
							pic_not_taken = false;
						}
					} else {
						pic_not_taken = true;
					}
					unsigned long curr_rtc_time = eps_getRTCTime();
					if( DEPLOY_START < curr_rtc_time && curr_rtc_time < DEPLOY_END ) {//DEPLOY
						if( attempted_deploy == 0 ) {
							packet_log( "OBC initiating deploy\n" );
							current_state = DEPLOY;
							break;
						} else if( current_cycle % 100 == 0 ) {
							attempted_deploy = 0;
						}
					}
					if( packet_detect_uplink() ) {//UPLINK
						packet_log( "OBC processing uplink\n" );
						packet_process_uplink();
						current_state = DOWNLINK;
						break;
					}
					if( current_cycle % 4 == 0 || rtY.fsw_telem.sc_above_targ ) {//PACKET
						packet_queue_beacon();
						switch( current_cycle % 12 ) {
							case 0  : packet_queue_header(); break;
							case 2  : packet_queue_eps(); break;
							case 4  : packet_queue_com(); break;
							case 6  : packet_queue_gnc(); break;
							case 8  : packet_queue_sen(); break;
							case 10 : packet_queue_logger(); break;
							default :
						}
						current_state = DOWNLINK;
						break;
					}
				}
				break;

			case DEPLOY:
				if( clock() - last_deploy < DEPLOY_PERIOD ) {
					break;
				}
				last_deploy = clock();
				if( attempted_deploy == 4 || com_i2c_checkDeploy() ) {
					packet_log( "OBC deploy finished\n" );
					current_state = NORMAL;
					break;
				}
				//deploy antenna
				switch( attempted_deploy++ ) {
					case 0 :
						com_deployAntenna_algorithmOne();
						break;
					case 1 :
						com_deployAntenna_algorithmTwo();
						break;
					case 2 :
						com_set_burn_wire1();
						break;
					case 3 :
						com_set_burn_wire2();
						break;
					default :
				}
				break;

			case DOWNLINK:
				if( !packet_send_downlink() ) {
					current_state = NORMAL;
				}
				break;

			default:
				packet_log( "OBC unknown state\n" );
				current_state = INIT;
				break;
		}
		if( clock() - last_gnc > GNC_PERIOD ) {//GNC
			last_gnc = clock();
			//gnc_sensorInput();
			//gnc_cdhInput();
			//gnc_rt_OneStep();
			//gnc_sensorOutput();
			//gnc_cdhOutput();
		}
		RTWDOG_Refresh( RTWDOG );
		busy_delay( 100 );//100ms delay
	}
}
