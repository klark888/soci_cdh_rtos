/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v14.0
processor: MIMXRT1021xxxxx
package_id: MIMXRT1021DAG5A
mcu_data: ksdk2_0
processor_version: 14.0.0
board: MIMXRT1020-EVK
external_user_signals: {}
pin_labels:
- {pin_num: '52', pin_signal: WAKEUP, label: USER_BUTTON, identifier: USER_BUTTON}
power_domains: {NVCC_GPIO: '3.3'}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '79', peripheral: LPUART4, signal: RX, pin_signal: GPIO_AD_B1_11}
  - {pin_num: '80', peripheral: LPUART4, signal: TX, pin_signal: GPIO_AD_B1_10}
  - {pin_num: '93', peripheral: LPUART3, signal: RX, pin_signal: GPIO_AD_B0_15}
  - {pin_num: '94', peripheral: LPUART3, signal: TX, pin_signal: GPIO_AD_B0_14}
  - {pin_num: '75', peripheral: LPI2C1, signal: SCL, pin_signal: GPIO_AD_B1_14, slew_rate: Slow, software_input_on: Enable, open_drain: Enable, speed: MHZ_100, drive_strength: R0_6,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, pull_up_down_config: Pull_Up_22K_Ohm, hysteresis_enable: Disable}
  - {pin_num: '74', peripheral: LPI2C1, signal: SDA, pin_signal: GPIO_AD_B1_15, slew_rate: Slow, software_input_on: Enable, open_drain: Enable, speed: MHZ_100, drive_strength: R0_6,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, pull_up_down_config: Pull_Up_22K_Ohm, hysteresis_enable: Disable}
  - {pin_num: '82', peripheral: LPI2C2, signal: SCL, pin_signal: GPIO_AD_B1_08, slew_rate: Slow, software_input_on: Enable, open_drain: Enable, speed: MHZ_100, drive_strength: R0_6,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, pull_up_down_config: Pull_Up_22K_Ohm, hysteresis_enable: Disable}
  - {pin_num: '81', peripheral: LPI2C2, signal: SDA, pin_signal: GPIO_AD_B1_09, slew_rate: Slow, software_input_on: Enable, open_drain: Enable, speed: MHZ_100, drive_strength: R0_6,
    pull_keeper_select: Keeper, pull_keeper_enable: Enable, pull_up_down_config: Pull_Up_22K_Ohm, hysteresis_enable: Disable}
  - {pin_num: '83', peripheral: GPIO1, signal: 'gpio_io, 23', pin_signal: GPIO_AD_B1_07}
  - {pin_num: '52', peripheral: GPIO5, signal: 'gpio_io, 00', pin_signal: WAKEUP}
  - {pin_num: '110', peripheral: JTAG, signal: TCK, pin_signal: GPIO_AD_B0_01}
  - {pin_num: '108', peripheral: JTAG, signal: TDI, pin_signal: GPIO_AD_B0_03}
  - {pin_num: '107', peripheral: JTAG, signal: TDO, pin_signal: GPIO_AD_B0_04}
  - {pin_num: '111', peripheral: JTAG, signal: TMS, pin_signal: GPIO_AD_B0_00}
  - {pin_num: '106', peripheral: JTAG, signal: TRSTB, pin_signal: GPIO_AD_B0_05}
  - {pin_num: '98', peripheral: LPSPI1, signal: SCK, pin_signal: GPIO_AD_B0_10}
  - {pin_num: '95', peripheral: LPSPI1, signal: SDI, pin_signal: GPIO_AD_B0_13}
  - {pin_num: '96', peripheral: LPSPI1, signal: SDO, pin_signal: GPIO_AD_B0_12}
  - {pin_num: '78', peripheral: GPIO1, signal: 'gpio_io, 28', pin_signal: GPIO_AD_B1_12}
  - {pin_num: '76', peripheral: GPIO1, signal: 'gpio_io, 29', pin_signal: GPIO_AD_B1_13}
  - {pin_num: '101', peripheral: LPUART1, signal: RX, pin_signal: GPIO_AD_B0_07}
  - {pin_num: '105', peripheral: LPUART1, signal: TX, pin_signal: GPIO_AD_B0_06}
  - {pin_num: '89', peripheral: GPIO1, signal: 'gpio_io, 19', pin_signal: GPIO_AD_B1_03}
  - {pin_num: '90', peripheral: GPIO1, signal: 'gpio_io, 18', pin_signal: GPIO_AD_B1_02}
  - {pin_num: '99', peripheral: LPI2C3, signal: SDA, pin_signal: GPIO_AD_B0_09, identifier: '', software_input_on: Enable, open_drain: Enable, speed: MHZ_100, pull_up_down_config: Pull_Up_22K_Ohm}
  - {pin_num: '100', peripheral: LPI2C3, signal: SCL, pin_signal: GPIO_AD_B0_08, software_input_on: Enable, open_drain: Enable, speed: MHZ_100, pull_up_down_config: Pull_Up_22K_Ohm}
  - {pin_num: '84', peripheral: LPSPI1, signal: PCS3, pin_signal: GPIO_AD_B1_06}
  - {pin_num: '87', peripheral: LPSPI1, signal: PCS2, pin_signal: GPIO_AD_B1_05}
  - {pin_num: '88', peripheral: LPSPI1, signal: PCS1, pin_signal: GPIO_AD_B1_04}
  - {pin_num: '91', peripheral: GPIO1, signal: 'gpio_io, 17', pin_signal: GPIO_AD_B1_01}
  - {pin_num: '92', peripheral: GPIO1, signal: 'gpio_io, 16', pin_signal: GPIO_AD_B1_00}
  - {pin_num: '97', peripheral: LPSPI1, signal: PCS0, pin_signal: GPIO_AD_B0_11}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           
  CLOCK_EnableClock(kCLOCK_IomuxcSnvs);       

  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_00_JTAG_TMS, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_01_JTAG_TCK, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_JTAG_TDI, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_04_JTAG_TDO, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_05_JTAG_TRSTB, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_06_LPUART1_TX, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_07_LPUART1_RX, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_08_LPI2C3_SCL, 1U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_09_LPI2C3_SDA, 1U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_LPSPI1_SCK, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_11_LPSPI1_PCS0, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_12_LPSPI1_SDO, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_13_LPSPI1_SDI, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_14_LPUART3_TX, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_15_LPUART3_RX, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_00_GPIO1_IO16, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_01_GPIO1_IO17, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_02_GPIO1_IO18, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_03_GPIO1_IO19, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_04_LPSPI1_PCS1, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_05_LPSPI1_PCS2, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_06_LPSPI1_PCS3, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_07_GPIO1_IO23, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_08_LPI2C2_SCL, 1U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_09_LPI2C2_SDA, 1U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_10_LPUART4_TX, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_11_LPUART4_RX, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_12_GPIO1_IO28, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_13_GPIO1_IO29, 0U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_14_LPI2C1_SCL, 1U); 
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_15_LPI2C1_SDA, 1U); 
  IOMUXC_SetPinMux(IOMUXC_SNVS_WAKEUP_GPIO5_IO00, 0U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_08_LPI2C3_SCL, 0xD8B0U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_09_LPI2C3_SDA, 0xD8B0U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_08_LPI2C2_SCL, 0xD8B0U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_09_LPI2C2_SDA, 0xD8B0U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_14_LPI2C1_SCL, 0xD8B0U); 
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_15_LPI2C1_SDA, 0xD8B0U); 
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/