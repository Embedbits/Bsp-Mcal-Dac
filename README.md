# DAC MCAL Module

The **DAC (Digital-to-Analog Converter) MCAL module** provides an abstraction layer for managing digital-to-analog conversion peripherals on STM32 microcontrollers.  
This module is part of the MCAL layer and ensures a unified API across different STM32 families.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family.

---

## Features

- Initialization and deinitialization of DAC peripherals and their channels, default configuration
- APB1 bus clock of DAC1 handled through the RCC module
- Output modes: buffered / unbuffered pin output, internal connection to on-chip peripherals (ADC, comparators, operational amplifiers)
- Software and hardware (timer TRGO / LPTIM output / EXTI line 9) conversion triggers, the list contains the triggers of the device line only
- 12-bit right / left aligned and 8-bit right aligned data formats
- Single and dual channel (simultaneous) output update, output register read-back
- Built-in waveform generation: noise (LFSR) and triangle wave
- Sample & hold mode with timing, LSI sample & hold clock (started by the module)
- Offset calibration on initialization of buffered outputs, trimming value readable / writable
- Voltage conversion helpers (output value from voltage, output voltage from output register)
- DMA buffer output through the DMA module (DMA channel per DAC channel, one shot / circular) with half transfer, transfer complete and error (DMA underrun, DMA transfer error) callbacks
- Every register write verified by read-back, HW state checked before configuration changes

---

## STM32L4 / STM32L4+ specifics

Public interface of the STM32H5 module (`Dev/STM32H5`, STM32G4 implementation as base). Differences:

| Feature                                  | STM32L4 / STM32L4+ behavior                                                                          |
|------------------------------------------|------------------------------------------------------------------------------------------------------|
| DAC peripherals                          | DAC1 only: 2 channels (PA4 / PA5), 1 channel on STM32L45x / L46x, no DAC on STM32L41x / L42x (module empty) |
| Clock source                             | `DAC_CLK_SRC_PCLK1` only (APB1 clock, no kernel clock multiplexer) - `Dac_Set_ClockSource()` only checks that no channel is enabled |
| Sample & hold clock                      | `DAC_SH_CLK_SRC_LSI` only (started through RCC if not running), `DAC_SH_CLK_SRC_LSE` refused       |
| Channel 2 of single channel DAC          | `DAC_CHANNEL_2` is not in the list `dac_ChannelId_t`, dual data / dual software trigger are refused  |
| Triggers                                 | STM32L4 (TSELx 3 bits): TIM2 / TIM4 / TIM5 / TIM6 / TIM7 / TIM8 TRGO, EXTI line 9; STM32L4+ (TSELx 4 bits): additionally TIM1 / TIM15 TRGO, LPTIM1 / LPTIM2 output; `dac_TriggerId_t` contains the triggers of the device line only (triggers of timers not present on the MCU and the STM32L4+ triggers on STM32L4 are not in the list, guards by the CMSIS instance macros and `DAC_CR_TSEL1_3`) |
| Channel ready flag                       | not available (no DACxRDY) - `Dac_Set_ChannelActive()` verifies ENx only, `DAC_FLAG_READY` refused   |
| High frequency interface mode            | not available on STM32L4                                                                             |
| Calibration in `Dac_ChannelInit()`       | buffered output modes only - the calibration runs in buffered pin mode, so it is skipped for unbuffered and internal outputs (the pin of an internal output is never driven, e.g. SMPS enable PA4 of Nucleo-64-P) |
| Sawtooth wave, signed data format        | not available on STM32L4                                                                             |
| DMA                                      | `Dma` selects the DMA channel from the list `dac_Dma_t` - one item per DAC channel, DMA peripheral and channel, named `DAC_DMA_DAC1_CHx_DMAy_CHANNELz` (e.g. `DAC_DMA_DAC1_CH1_DMA1_CHANNEL3`). STM32L4: fixed request mapping (DMA_CSELR, the request selection is part of the item) - channel 1 on DMA1 channel 3 / DMA2 channel 4, channel 2 on DMA1 channel 4 / DMA2 channel 5; the list contains these channels only. STM32L4+: DMAMUX1 (any channel), the list contains all of them. Items of another DAC channel and `DAC_DMA_UNUSED` are refused in the DMA mode. The DMA channel stays enabled after a normal mode transfer and is disabled by the module on transfer complete |
| Errors                                   | DMA underrun and DMA transfer error, GPDMA specific errors never reported                            |
| Interrupt line                           | DMA underrun on `TIM6_DAC` (shared with TIM6)                                                        |

STM32L4 errata sheets not reviewed yet.

---

## Supported Hardware

| MCU line                                   | DAC1 channels | Output pins                      | Resolution |
|--------------------------------------------|---------------|----------------------------------|------------|
| STM32L41x / L42x                           | -             | -                                | -          |
| STM32L43x / L44x                           | 2             | PA4 (DAC1_OUT1), PA5 (DAC1_OUT2) | 12-bit     |
| STM32L45x / L46x                           | 1             | PA4 (DAC1_OUT1)                  | 12-bit     |
| STM32L47x / L48x / L49x / L4Ax             | 2             | PA4 (DAC1_OUT1), PA5 (DAC1_OUT2) | 12-bit     |
| STM32L4P5 / L4Q5 / L4R / L4S (STM32L4+)    | 2             | PA4 (DAC1_OUT1), PA5 (DAC1_OUT2) | 12-bit     |

Default DMA channels (`Dac_Get_DefaultConfig()`): channel 1 - DMA1 channel 3, channel 2 - DMA1 channel 4.

Nucleo-64-P boards use PA4 - PA7 for the SMPS - DAC outputs on these pins must not be used there
(internal output mode only).

---

## Module Structure

| File          | Description                                                                 |
|---------------|-----------------------------------------------------------------------------|
| `Dac_Port.h`  | Public API                                                                  |
| `Dac_Types.h` | Public types                                                                |
| `Dac.c`       | Clock, peripheral and channel configuration, calibration, data output       |
| `Dac_Dma.c`   | DMA buffer output (DMA channel per DAC channel)                             |
| `Dac_Isr.c`   | DAC interrupt (DMA underrun of all channels, TIM6_DAC line)                 |
| `Dac.h`, `Dac_Dma.h`, `Dac_Isr.h` | Private interfaces (not exported by the CMake library)  |

---

## Public API

### Module Management
- `dac_ModuleVersion_t Dac_Get_ModuleVersion ( void );`
- `dac_RequestState_t  Dac_Init              ( dac_Config_t * const dacConfig );`
- `dac_RequestState_t  Dac_Deinit            ( dac_PeriphId_t periphId );`
- `void                Dac_Task              ( void );`
- `dac_RequestState_t  Dac_Get_DefaultConfig ( dac_Config_t * const dacConfig );`

### Clock Configuration
- `dac_RequestState_t  Dac_Set_ClockSource   ( dac_ClkSrc_t clkSource );`
- `dac_RequestState_t  Dac_Get_ClockSource   ( dac_ClkSrc_t * const clkSource );`
- `dac_RequestState_t  Dac_Set_ShClockSource ( dac_ShClkSrc_t clkSource );`
- `dac_RequestState_t  Dac_Get_ShClockSource ( dac_ShClkSrc_t * const clkSource );`

### Peripheral and Channel Configuration
- `dac_RequestState_t  Dac_PeriphInit           ( dac_PeriphConfig_t * const periphConfig );`
- `dac_RequestState_t  Dac_ChannelInit          ( dac_PeriphId_t periphId, dac_ChannelConfig_t * const channelConfig );`
- `dac_RequestState_t  Dac_Set_ChannelActive    ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
- `dac_RequestState_t  Dac_Set_ChannelInactive  ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
- `dac_RequestState_t  Dac_Get_ChannelState     ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_FunctionState_t * const channelState );`
- `Dac_Set_` / `Dac_Get_` `OutputMode`, `TriggerSrc`, `DataFormat`, `WaveMode`, `WaveAmplitude`, `SampleHoldConfig`

### Calibration
- `dac_RequestState_t  Dac_Set_Calibration ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
- `dac_RequestState_t  Dac_Set_TrimValue   ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TrimValue_t trimValue );`
- `dac_RequestState_t  Dac_Get_TrimValue   ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TrimValue_t * const trimValue );`

### Data Output
- `dac_RequestState_t  Dac_Set_Data            ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Data_t data );`
- `dac_RequestState_t  Dac_Set_DualData        ( dac_PeriphId_t periphId, dac_Data_t dataCh1, dac_Data_t dataCh2 );`
- `dac_RequestState_t  Dac_Get_OutputData      ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Data_t * const data );`
- `dac_RequestState_t  Dac_Set_SwTrigger       ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
- `dac_RequestState_t  Dac_Set_DualSwTrigger   ( dac_PeriphId_t periphId );`
- `dac_RequestState_t  Dac_Get_DataFromVoltage ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Voltage_mV_t refVoltage, dac_Voltage_mV_t outVoltage, dac_Data_t * const data );`
- `dac_RequestState_t  Dac_Get_OutputVoltage   ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Voltage_mV_t refVoltage, dac_Voltage_mV_t * const outVoltage );`
- `dac_RequestState_t  Dac_Set_Voltage         ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Voltage_mV_t refVoltage, dac_Voltage_mV_t outVoltage );`
- `dac_RequestState_t  Dac_Get_Flag            ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_FlagId_t flagId, dac_FlagState_t * const flagState );`
- `dac_RequestState_t  Dac_Clear_Flag          ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_FlagId_t flagId );`

### DMA Data Transfer
- `dac_RequestState_t  Dac_Set_DataConfig ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, const dac_DataConfig_t * const dataConfig );`
- `dac_RequestState_t  Dac_Get_DataConfig ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_DataConfig_t * const dataConfig );`
- `dac_RequestState_t  Dac_Set_DmaStart   ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
- `dac_RequestState_t  Dac_Set_DmaStop    ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`

---

## Usage Notes

- Channel configuration (output mode, trigger, wave, sample & hold, calibration, trimming) is allowed only while the channel is disabled - otherwise `DAC_REQUEST_ERROR` is returned and no register is modified.
- Calibration is executed by `Dac_ChannelInit()` for buffered outputs in buffered normal mode (the configured output mode is restored). The output pin should not be loaded during initialization.
- Without trigger (`DAC_TRIGGER_NONE`) the output is updated a few APB clock cycles after the data write, otherwise on the trigger event. DAC has no trigger edge selection (rising edge).
- Wave generation requires a trigger (wave value is added to the data holding register on every trigger).
- DMA transfer requires a hardware trigger - DAC DMA requests are not generated without trigger or on software trigger. A DMA channel owned by an initialized DAC channel is rejected for every other DAC channel (both channels of one configuration, other peripheral or separate initialization) until the owner is deinitialized or reconfigured. DMA channels shall not be shared with other modules.
- DMA underrun or DMA transfer error stops the transfer and is reported through `ErrorCallback`.
- The DAC interrupt handler is registered on the `TIM6_DAC` / `TIM7_DAC` line - TIM6 / TIM7 interrupts are not processed while a DAC DMA transfer of the line is initialized.
- `Dac_Task()` has no periodic processing, it is provided for consistency of the MCAL interface.

### Example

```c
dac_Config_t dacConfig;

(void)Dac_Get_DefaultConfig( &dacConfig );                  /* DAC1 channel 1, buffered output on PA4 */
dacConfig.PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_1 ].InitValue = 2048u;

if( DAC_REQUEST_OK == Dac_Init( &dacConfig ) )
{
    (void)Dac_Set_Voltage( DAC_PERIPH_1, DAC_CHANNEL_1, 3300u, 1200u );  /* 1.2 V at VREF+ = 3.3 V */
}
```

---

## Testing

- Unit tests (host, Unity / CMock / RegMem): `Tests/UnitTests/Test_Dac.c` - DAC registers are emulated, the calibration flag by a HW model thread. Variant expectations by CMSIS definitions: MCUs without DAC (STM32L41x / L42x) ignore all tests, single channel DAC (STM32L45x / L46x) has no `DAC_CHANNEL_2` in the list and ignores channel 2 tests.
  ```bash
  cmake --preset STM32L432xC_UnitTest && cmake --build --preset STM32L432xC_UnitTest
  ctest --test-dir Build/STM32L432xC_UnitTest -L Dac
  ```
- Integration tests (target, boards named by the MCU): `Tests/IntegrationTests/ItTest_Dac.c` - output registers, calibration, sample and hold with LSI, wave generation, DMA buffer output with TIM6 TRGO trigger and DMA underrun, output voltage measured by the ADC (internal outputs by ADC2 / ADC1 channel 17 / 18 on all boards with DAC except STM32L4R / L4S - DAC1 outputs not connected to the ADC, Bug AB#1282; buffered / unbuffered pin output PA4 by ADC1 channel 9 only on boards with free PA4 - NUCLEO-L476RG and the Nucleo-144 boards; the groups with a Nucleo-64-P board use internal outputs only, PA4 = SMPS enable). STM32L41x / L42x: all tests ignored.

---

## 🛠 CMake Integration

```cmake
target_link_libraries(User_Lib PRIVATE Dac_Lib)
```

1. Include `Dac_Lib` in your CMake library.
2. Include `Dac_Port.h` in your project.
3. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution–NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** — [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.

---

## 🌐 Useful Links

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [Azure DevOps](https://azure.microsoft.com/en-us/services/devops/)
- [Embedbits Github](https://github.com/Embedbits)
- [CC BY-NC 4.0 License](https://creativecommons.org/licenses/by-nc/4.0/)
