/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v13.0
processor: MIMXRT1021xxxxx
package_id: MIMXRT1021DAG5A
mcu_data: ksdk2_0
processor_version: 14.0.0
board: MIMXRT1020-EVK
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: 866434d8-d5d8-49f5-9bd0-90c806985966
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'uart_cmsis_common'
- type_id: 'uart_cmsis_common_9cb8e302497aa696fdbb5a4fd622c2a8'
- global_USART_CMSIS_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'gpio_adapter_common'
- type_id: 'gpio_adapter_common_57579b9ac814fe26bf95df0a384c36b6'
- global_gpio_adapter_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * GPT initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPT'
- type: 'gpt'
- mode: 'general'
- custom_name_enabled: 'true'
- type_id: 'gpt_e92a0cbd07e389b82a1d19b05eb9fdda'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPT2'
- config_sets:
  - fsl_gpt:
    - gpt_config:
      - clockSource: 'kGPT_ClockSource_LowFreq'
      - clockSourceFreq: 'BOARD_BootClockRUN'
      - oscDivider: '1'
      - divider: '1'
      - enableFreeRun: 'false'
      - enableRunInWait: 'true'
      - enableRunInStop: 'true'
      - enableRunInDoze: 'true'
      - enableRunInDbg: 'false'
      - enableMode: 'true'
    - input_capture_channels: []
    - output_compare_channels: []
    - interrupt_requests: ''
    - isInterruptEnabled: 'false'
    - interrupt:
      - IRQn: 'GPT2_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const gpt_config_t GPT_config = {
  .clockSource = kGPT_ClockSource_LowFreq,
  .divider = 1UL,
  .enableFreeRun = false,
  .enableRunInWait = true,
  .enableRunInStop = true,
  .enableRunInDoze = true,
  .enableRunInDbg = false,
  .enableMode = true
};

static void GPT_init(void) {
  /* GPT device and channels initialization */
  GPT_Init(GPT_PERIPHERAL, &GPT_config);
  GPT_SetOscClockDivider(GPT_PERIPHERAL, 1);
  /* Enable GPT interrupt sources */
  GPT_EnableInterrupts(GPT_PERIPHERAL, 0);
}

/***********************************************************************************************************************
 * NVIC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'NVIC'
- type: 'nvic'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'nvic_57b5eef3774cc60acaede6f5b8bddc67'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'NVIC'
- config_sets:
  - nvic:
    - interrupt_table:
      - 0: []
      - 1: []
      - 2: []
      - 3: []
      - 4: []
      - 5: []
    - interrupts:
      - 0:
        - channelId: 'int_0'
        - interrupt_t:
          - IRQn: 'LPI2C1_IRQn'
          - enable_interrrupt: 'enabled'
          - enable_priority: 'true'
          - priority: '3'
          - enable_custom_name: 'false'
      - 1:
        - channelId: 'int_1'
        - interrupt_t:
          - IRQn: 'LPI2C2_IRQn'
          - enable_interrrupt: 'enabled'
          - enable_priority: 'true'
          - priority: '3'
          - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/* Empty initialization function (commented out)
static void NVIC_init(void) {
} */

/***********************************************************************************************************************
 * LPI2C1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPI2C1'
- type: 'lpi2c'
- mode: 'master'
- custom_name_enabled: 'false'
- type_id: 'lpi2c_6b71962515c3208facfccd030afebc98'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'LPI2C1'
- config_sets:
  - main:
    - clockSource: 'Lpi2cClock'
    - clockSourceFreq: 'BOARD_BootClockRUN'
  - interrupt_vector: []
  - master:
    - mode: 'freertos'
    - config:
      - enableMaster: 'true'
      - enableDoze: 'true'
      - debugEnable: 'false'
      - ignoreAck: 'false'
      - pinConfig: 'kLPI2C_2PinOpenDrain'
      - baudRate_Hz: '100000'
      - busIdleTimeout_ns: '0'
      - pinLowTimeout_ns: '0'
      - sdaGlitchFilterWidth_ns: '0'
      - sclGlitchFilterWidth_ns: '0'
      - hostRequest:
        - enable: 'false'
        - source: 'kLPI2C_HostRequestExternalPin'
        - polarity: 'kLPI2C_HostRequestPinActiveHigh'
      - edmaRequestSources: ''
    - transfer:
      - enable_custom_handle: 'false'
      - flags: ''
      - slaveAddress: '0'
      - direction: 'kLPI2C_Write'
      - subaddress: '0'
      - subaddressSize: '0'
      - blocking_buffer: 'false'
      - enable_custom_buffer: 'false'
      - dataSize: '32'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lpi2c_master_config_t LPI2C1_masterConfig = {
  .enableMaster = true,
  .enableDoze = true,
  .debugEnable = false,
  .ignoreAck = false,
  .pinConfig = kLPI2C_2PinOpenDrain,
  .baudRate_Hz = 100000UL,
  .busIdleTimeout_ns = 0UL,
  .pinLowTimeout_ns = 0UL,
  .sdaGlitchFilterWidth_ns = 0U,
  .sclGlitchFilterWidth_ns = 0U,
  .hostRequest = {
    .enable = false,
    .source = kLPI2C_HostRequestExternalPin,
    .polarity = kLPI2C_HostRequestPinActiveHigh
  }
};
lpi2c_master_transfer_t LPI2C1_masterTransfer = {
  .flags = kLPI2C_TransferDefaultFlag,
  .slaveAddress = 0,
  .direction = kLPI2C_Write,
  .subaddress = 0,
  .subaddressSize = 0,
  .data = LPI2C1_masterBuffer,
  .dataSize = 32
};
lpi2c_rtos_handle_t LPI2C1_masterHandle;
uint8_t LPI2C1_masterBuffer[LPI2C1_MASTER_BUFFER_SIZE];

static void LPI2C1_init(void) {
  LPI2C_RTOS_Init(&LPI2C1_masterHandle, LPI2C1_PERIPHERAL, &LPI2C1_masterConfig, LPI2C1_CLOCK_FREQ);
}

/***********************************************************************************************************************
 * LPI2C2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPI2C2'
- type: 'lpi2c'
- mode: 'master'
- custom_name_enabled: 'false'
- type_id: 'lpi2c_6b71962515c3208facfccd030afebc98'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'LPI2C2'
- config_sets:
  - main:
    - clockSource: 'Lpi2cClock'
    - clockSourceFreq: 'BOARD_BootClockRUN'
  - interrupt_vector: []
  - master:
    - mode: 'freertos'
    - config:
      - enableMaster: 'true'
      - enableDoze: 'true'
      - debugEnable: 'false'
      - ignoreAck: 'false'
      - pinConfig: 'kLPI2C_2PinOpenDrain'
      - baudRate_Hz: '100000'
      - busIdleTimeout_ns: '0'
      - pinLowTimeout_ns: '0'
      - sdaGlitchFilterWidth_ns: '0'
      - sclGlitchFilterWidth_ns: '0'
      - hostRequest:
        - enable: 'false'
        - source: 'kLPI2C_HostRequestExternalPin'
        - polarity: 'kLPI2C_HostRequestPinActiveHigh'
      - edmaRequestSources: ''
    - transfer:
      - enable_custom_handle: 'false'
      - flags: ''
      - slaveAddress: '0x7E'
      - direction: 'kLPI2C_Write'
      - subaddress: '0'
      - subaddressSize: '0'
      - blocking_buffer: 'false'
      - enable_custom_buffer: 'false'
      - dataSize: '32'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lpi2c_master_config_t LPI2C2_masterConfig = {
  .enableMaster = true,
  .enableDoze = true,
  .debugEnable = false,
  .ignoreAck = false,
  .pinConfig = kLPI2C_2PinOpenDrain,
  .baudRate_Hz = 100000UL,
  .busIdleTimeout_ns = 0UL,
  .pinLowTimeout_ns = 0UL,
  .sdaGlitchFilterWidth_ns = 0U,
  .sclGlitchFilterWidth_ns = 0U,
  .hostRequest = {
    .enable = false,
    .source = kLPI2C_HostRequestExternalPin,
    .polarity = kLPI2C_HostRequestPinActiveHigh
  }
};
lpi2c_master_transfer_t LPI2C2_masterTransfer = {
  .flags = kLPI2C_TransferDefaultFlag,
  .slaveAddress = 0x7E,
  .direction = kLPI2C_Write,
  .subaddress = 0,
  .subaddressSize = 0,
  .data = LPI2C2_masterBuffer,
  .dataSize = 32
};
lpi2c_rtos_handle_t LPI2C2_masterHandle;
uint8_t LPI2C2_masterBuffer[LPI2C2_MASTER_BUFFER_SIZE];

static void LPI2C2_init(void) {
  LPI2C_RTOS_Init(&LPI2C2_masterHandle, LPI2C2_PERIPHERAL, &LPI2C2_masterConfig, LPI2C2_CLOCK_FREQ);
}

/***********************************************************************************************************************
 * LPUART3 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPUART3'
- type: 'lpuart'
- mode: 'freertos'
- custom_name_enabled: 'false'
- type_id: 'lpuart_54a65a580e3462acdbacefd5299e0cac'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'LPUART3'
- config_sets:
  - fsl_lpuart_freertos:
    - lpuart_rtos_configuration:
      - clockSource: 'LpuartClock'
      - srcclk: 'BOARD_BootClockRUN'
      - baudrate: '115200'
      - parity: 'kLPUART_ParityDisabled'
      - stopbits: 'kLPUART_OneStopBit'
      - buffer_size: '64'
      - enableRxRTS: 'false'
      - enableTxCTS: 'false'
      - txCtsSource: 'kLPUART_CtsSourcePin'
      - txCtsConfig: 'kLPUART_CtsSampleAtStart'
    - interrupt_rx_tx:
      - IRQn: 'LPUART3_IRQn'
      - enable_priority: 'true'
      - priority: '5'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
lpuart_rtos_handle_t LPUART3_rtos_handle;
lpuart_handle_t LPUART3_lpuart_handle;
uint8_t LPUART3_background_buffer[LPUART3_BACKGROUND_BUFFER_SIZE];
lpuart_rtos_config_t LPUART3_rtos_config = {
  .base = LPUART3_PERIPHERAL,
  .baudrate = 115200UL,
  .srcclk = 80000000UL,
  .parity = kLPUART_ParityDisabled,
  .stopbits = kLPUART_OneStopBit,
  .buffer = LPUART3_background_buffer,
  .buffer_size = LPUART3_BACKGROUND_BUFFER_SIZE,
  .enableRxRTS = false,
  .enableTxCTS = false,
  .txCtsSource = kLPUART_CtsSourcePin,
  .txCtsConfig = kLPUART_CtsSampleAtStart,
};

static void LPUART3_init(void) {
  /* LPUART rtos initialization */
  LPUART_RTOS_Init(&LPUART3_rtos_handle, &LPUART3_lpuart_handle, &LPUART3_rtos_config);
  /* Interrupt vector LPUART3_IRQn priority settings in the NVIC. */
  NVIC_SetPriority(LPUART3_IRQN, LPUART3_IRQ_PRIORITY);
}

/***********************************************************************************************************************
 * LPUART4 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPUART4'
- type: 'lpuart'
- mode: 'freertos'
- custom_name_enabled: 'false'
- type_id: 'lpuart_54a65a580e3462acdbacefd5299e0cac'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'LPUART4'
- config_sets:
  - fsl_lpuart_freertos:
    - lpuart_rtos_configuration:
      - clockSource: 'LpuartClock'
      - srcclk: 'BOARD_BootClockRUN'
      - baudrate: '115200'
      - parity: 'kLPUART_ParityDisabled'
      - stopbits: 'kLPUART_OneStopBit'
      - buffer_size: '64'
      - enableRxRTS: 'false'
      - enableTxCTS: 'false'
      - txCtsSource: 'kLPUART_CtsSourcePin'
      - txCtsConfig: 'kLPUART_CtsSampleAtStart'
    - interrupt_rx_tx:
      - IRQn: 'LPUART4_IRQn'
      - enable_priority: 'true'
      - priority: '5'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
lpuart_rtos_handle_t LPUART4_rtos_handle;
lpuart_handle_t LPUART4_lpuart_handle;
uint8_t LPUART4_background_buffer[LPUART4_BACKGROUND_BUFFER_SIZE];
lpuart_rtos_config_t LPUART4_rtos_config = {
  .base = LPUART4_PERIPHERAL,
  .baudrate = 115200UL,
  .srcclk = 80000000UL,
  .parity = kLPUART_ParityDisabled,
  .stopbits = kLPUART_OneStopBit,
  .buffer = LPUART4_background_buffer,
  .buffer_size = LPUART4_BACKGROUND_BUFFER_SIZE,
  .enableRxRTS = false,
  .enableTxCTS = false,
  .txCtsSource = kLPUART_CtsSourcePin,
  .txCtsConfig = kLPUART_CtsSampleAtStart,
};

static void LPUART4_init(void) {
  /* LPUART rtos initialization */
  LPUART_RTOS_Init(&LPUART4_rtos_handle, &LPUART4_lpuart_handle, &LPUART4_rtos_config);
  /* Interrupt vector LPUART4_IRQn priority settings in the NVIC. */
  NVIC_SetPriority(LPUART4_IRQN, LPUART4_IRQ_PRIORITY);
}

/***********************************************************************************************************************
 * LPI2C3 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPI2C3'
- type: 'lpi2c'
- mode: 'master'
- custom_name_enabled: 'false'
- type_id: 'lpi2c_6b71962515c3208facfccd030afebc98'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'LPI2C3'
- config_sets:
  - main:
    - clockSource: 'Lpi2cClock'
    - clockSourceFreq: 'BOARD_BootClockRUN'
  - interrupt_vector: []
  - master:
    - mode: 'freertos'
    - config:
      - enableMaster: 'true'
      - enableDoze: 'true'
      - debugEnable: 'false'
      - ignoreAck: 'false'
      - pinConfig: 'kLPI2C_2PinOpenDrain'
      - baudRate_Hz: '100000'
      - busIdleTimeout_ns: '0'
      - pinLowTimeout_ns: '0'
      - sdaGlitchFilterWidth_ns: '0'
      - sclGlitchFilterWidth_ns: '0'
      - hostRequest:
        - enable: 'false'
        - source: 'kLPI2C_HostRequestExternalPin'
        - polarity: 'kLPI2C_HostRequestPinActiveHigh'
      - edmaRequestSources: ''
    - transfer:
      - enable_custom_handle: 'false'
      - flags: ''
      - slaveAddress: '0'
      - direction: 'kLPI2C_Write'
      - subaddress: '0'
      - subaddressSize: '0'
      - blocking_buffer: 'false'
      - enable_custom_buffer: 'false'
      - dataSize: '32'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lpi2c_master_config_t LPI2C3_masterConfig = {
  .enableMaster = true,
  .enableDoze = true,
  .debugEnable = false,
  .ignoreAck = false,
  .pinConfig = kLPI2C_2PinOpenDrain,
  .baudRate_Hz = 100000UL,
  .busIdleTimeout_ns = 0UL,
  .pinLowTimeout_ns = 0UL,
  .sdaGlitchFilterWidth_ns = 0U,
  .sclGlitchFilterWidth_ns = 0U,
  .hostRequest = {
    .enable = false,
    .source = kLPI2C_HostRequestExternalPin,
    .polarity = kLPI2C_HostRequestPinActiveHigh
  }
};
lpi2c_master_transfer_t LPI2C3_masterTransfer = {
  .flags = kLPI2C_TransferDefaultFlag,
  .slaveAddress = 0,
  .direction = kLPI2C_Write,
  .subaddress = 0,
  .subaddressSize = 0,
  .data = LPI2C3_masterBuffer,
  .dataSize = 32
};
lpi2c_rtos_handle_t LPI2C3_masterHandle;
uint8_t LPI2C3_masterBuffer[LPI2C3_MASTER_BUFFER_SIZE];

static void LPI2C3_init(void) {
  LPI2C_RTOS_Init(&LPI2C3_masterHandle, LPI2C3_PERIPHERAL, &LPI2C3_masterConfig, LPI2C3_CLOCK_FREQ);
}

/***********************************************************************************************************************
 * LPSPI1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPSPI1'
- type: 'lpspi'
- mode: 'freertos'
- custom_name_enabled: 'false'
- type_id: 'lpspi_6e21a1e0a09f0a012d683c4f91752db8'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'LPSPI1'
- config_sets:
  - transfer:
    - config:
      - transmitBuffer:
        - init: 'true'
      - receiveBuffer:
        - init: 'true'
      - dataSize: '32'
      - enableTransferStruct: 'defined'
      - flags: ''
  - main:
    - mode: 'kLPSPI_Master'
    - clockSource: 'LpspiClock'
    - clockSourceFreq: 'BOARD_BootClockRUN'
    - master:
      - baudRate: '500000'
      - bitsPerFrame: '8'
      - cpol: 'kLPSPI_ClockPolarityActiveHigh'
      - cpha: 'kLPSPI_ClockPhaseFirstEdge'
      - direction: 'kLPSPI_MsbFirst'
      - pcsToSckDelayInNanoSec: '1000'
      - lastSckToPcsDelayInNanoSec: '1000'
      - betweenTransferDelayInNanoSec: '1000'
      - whichPcs: 'kLPSPI_Pcs0'
      - pcsActiveHighOrLow: 'kLPSPI_PcsActiveLow'
      - pinCfg: 'kLPSPI_SdiInSdoOut'
      - dataOutConfig: 'kLpspiDataOutRetained'
      - enableInputDelay: 'false'
    - allPcsPolarityEnable: 'false'
    - allPcsPolarity:
      - kLPSPI_Pcs1Active: 'kLPSPI_PcsActiveHigh'
      - kLPSPI_Pcs2Active: 'kLPSPI_PcsActiveHigh'
      - kLPSPI_Pcs3Active: 'kLPSPI_PcsActiveHigh'
    - interrupt_priority:
      - IRQn: 'LPSPI1_IRQn'
      - enable_priority: 'true'
      - priority: '5'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lpspi_master_config_t LPSPI1_config = {
  .baudRate = 500000UL,
  .bitsPerFrame = 8UL,
  .cpol = kLPSPI_ClockPolarityActiveHigh,
  .cpha = kLPSPI_ClockPhaseFirstEdge,
  .direction = kLPSPI_MsbFirst,
  .pcsToSckDelayInNanoSec = 1000UL,
  .lastSckToPcsDelayInNanoSec = 1000UL,
  .betweenTransferDelayInNanoSec = 1000UL,
  .whichPcs = kLPSPI_Pcs0,
  .pcsActiveHighOrLow = kLPSPI_PcsActiveLow,
  .pinCfg = kLPSPI_SdiInSdoOut,
  .dataOutConfig = kLpspiDataOutRetained,
#if defined(FSL_LPSPI_DRIVER_VERSION) && (FSL_LPSPI_DRIVER_VERSION >= (MAKE_VERSION(2, 3, 0)))
  .enableInputDelay = false,
#endif
};
lpspi_transfer_t LPSPI1_transfer = {
  .txData = LPSPI1_txBuffer,
  .rxData = LPSPI1_rxBuffer,
  .dataSize = 32,
  .configFlags = 0
};
lpspi_rtos_handle_t LPSPI1_handle;
uint8_t LPSPI1_txBuffer[LPSPI1_BUFFER_SIZE];
uint8_t LPSPI1_rxBuffer[LPSPI1_BUFFER_SIZE];

static void LPSPI1_init(void) {
  /* Interrupt vector LPSPI1_IRQn priority settings in the NVIC. */
  NVIC_SetPriority(LPSPI1_IRQN, LPSPI1_IRQ_PRIORITY);
  LPSPI_RTOS_Init(&LPSPI1_handle, LPSPI1_PERIPHERAL, &LPSPI1_config, LPSPI1_CLOCK_FREQ);
}

/***********************************************************************************************************************
 * LPUART1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LPUART1'
- type: 'lpuart'
- mode: 'freertos'
- custom_name_enabled: 'false'
- type_id: 'lpuart_54a65a580e3462acdbacefd5299e0cac'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'LPUART1'
- config_sets:
  - fsl_lpuart_freertos:
    - lpuart_rtos_configuration:
      - clockSource: 'LpuartClock'
      - srcclk: 'BOARD_BootClockRUN'
      - baudrate: '115200'
      - parity: 'kLPUART_ParityDisabled'
      - stopbits: 'kLPUART_OneStopBit'
      - buffer_size: '64'
      - enableRxRTS: 'false'
      - enableTxCTS: 'false'
      - txCtsSource: 'kLPUART_CtsSourcePin'
      - txCtsConfig: 'kLPUART_CtsSampleAtStart'
    - interrupt_rx_tx:
      - IRQn: 'LPUART1_IRQn'
      - enable_priority: 'true'
      - priority: '5'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
lpuart_rtos_handle_t LPUART1_rtos_handle;
lpuart_handle_t LPUART1_lpuart_handle;
uint8_t LPUART1_background_buffer[LPUART1_BACKGROUND_BUFFER_SIZE];
lpuart_rtos_config_t LPUART1_rtos_config = {
  .base = LPUART1_PERIPHERAL,
  .baudrate = 38400UL,
  .srcclk = 80000000UL,
  .parity = kLPUART_ParityDisabled,
  .stopbits = kLPUART_OneStopBit,
  .buffer = LPUART1_background_buffer,
  .buffer_size = LPUART1_BACKGROUND_BUFFER_SIZE,
  .enableRxRTS = false,
  .enableTxCTS = false,
  .txCtsSource = kLPUART_CtsSourcePin,
  .txCtsConfig = kLPUART_CtsSampleAtStart,
};

static void LPUART1_init(void) {
  /* LPUART rtos initialization */
  LPUART_RTOS_Init(&LPUART1_rtos_handle, &LPUART1_lpuart_handle, &LPUART1_rtos_config);
  /* Interrupt vector LPUART1_IRQn priority settings in the NVIC. */
  NVIC_SetPriority(LPUART1_IRQN, LPUART1_IRQ_PRIORITY);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
static void BOARD_InitPeripherals_CommonPostInit(void)
{
  /* Interrupt vector LPI2C1_IRQn priority settings in the NVIC. */
  NVIC_SetPriority(INT_0_IRQN, INT_0_IRQ_PRIORITY);
  /* Interrupt vector LPI2C2_IRQn priority settings in the NVIC. */
  NVIC_SetPriority(INT_1_IRQN, INT_1_IRQ_PRIORITY);
  /* Enable interrupt LPI2C1_IRQn request in the NVIC. */
  EnableIRQ(INT_0_IRQN);
  /* Enable interrupt LPI2C2_IRQn request in the NVIC. */
  EnableIRQ(INT_1_IRQN);
}

void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  GPT_init();
  LPI2C1_init();
  LPI2C2_init();
  LPUART3_init();
  LPUART4_init();
  LPI2C3_init();
  LPSPI1_init();
  LPUART1_init();
  /* Common post-initialization */
  BOARD_InitPeripherals_CommonPostInit();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
