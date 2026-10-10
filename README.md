# DAC MCAL Module

The **DAC (Digital-to-Analog Converter) MCAL module** provides an abstraction layer for managing digital-to-analog conversion peripherals on STM32 microcontrollers.  
This module is part of the MCAL layer and ensures a unified API across different STM32 families.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family.

---

## Features

- Initialization and deinitialization of DAC peripherals and their channels, default configuration
- APB bus clock of every DAC peripheral handled through the RCC module
- Output modes: buffered / unbuffered pin output, internal connection to on-chip peripherals (comparators, operational amplifiers, ADC)
- Software and hardware (timer TRGO / LPTIM output / EXTI line 9 / HRTIM) conversion triggers
- 12-bit right / left aligned and 8-bit right aligned data formats
- Single and dual channel (simultaneous) output update, output register read-back
- Built-in waveform generation: noise (LFSR) and triangle wave
- Sample & hold mode with timing, LSI sample & hold clock (started by the module)
- Offset calibration on initialization of pin outputs, trimming value readable / writable
- Voltage conversion helpers (output value from voltage, output voltage from output register)
- DMA buffer output through the DMA module (DMA stream per DAC channel, DMAMUX1 routing, one shot / circular) with half transfer, transfer complete and error (DMA underrun, DMA transfer error) callbacks
- Every register write verified by read-back, HW state checked before configuration changes

---

## STM32H7 specifics

Public interface of the STM32H5 module (`Dev/STM32H5`), implementation based on the STM32G4 module. Differences:

| Feature                                  | STM32H7 behavior                                                                                     |
|------------------------------------------|------------------------------------------------------------------------------------------------------|
| DAC peripherals                          | DAC1 (2 channels, pins PA4 / PA5) on every device, DAC2 (1 channel, pin PA6, SmartRun domain) on STM32H7A3 / H7B0 / H7B3 |
| Clock source                             | `DAC_CLK_SRC_PCLK` only (DAC1 - PCLK1, DAC2 - PCLK4, no kernel clock multiplexer) - `Dac_Set_ClockSource()` only checks that no channel is enabled |
| Sample & hold clock                      | `DAC_SH_CLK_SRC_LSI` only (started through RCC if not running), `DAC_SH_CLK_SRC_LSE` refused       |
| Channel ready flag                       | not available - `Dac_Set_ChannelActive()` confirms ENx by read-back, `DAC_FLAG_READY` returns `DAC_REQUEST_ERROR` |
| High frequency interface mode            | not available (no HFSEL)                                                                             |
| Channel 2 of DAC2                        | refused by all channel services, dual data / dual software trigger of DAC2 refused                   |
| Triggers                                 | same set for every DAC peripheral: SW, TIM1 / 2 / 4 / 5 / 6 / 7 / 8 / 15 TRGO, LPTIM1 / LPTIM2 output, EXTI line 9; HRTIM DAC triggers 1 / 2 (devices with HRTIM1), TIM23 / TIM24 TRGO (STM32H72x / H73x), LPTIM3 output (STM32H7A3 / H7B0 / H7B3) |
| Sawtooth wave, signed data format        | not available                                                                                        |
| DMA                                      | DAC1: DMA1 / DMA2 stream selected in `dac_DataConfig_t` (`DmaPeriphId`, `DmaChannelId` = stream), request routed by DMAMUX1 (`DMA_REQ_DAC1_CHy`). DAC2: request served by BDMA2 / DMAMUX2 (not supported by the Dma module) - DMA transfer refused. Buffers in AXI SRAM / SRAM1 - SRAM4 (DMA1 / DMA2 can not access DTCM) |
| Errors                                   | DMA underrun and DMA transfer error, GPDMA specific errors never reported                            |
| Interrupt lines                          | DMA underrun of DAC1 on `TIM6_DAC` (shared with TIM6), DAC2 on `DAC2` (not used - no DMA transfer); the line is disabled by `Dac_Deinit()` only if it was enabled by the module |

---

## Supported Hardware

| MCU line                              | Peripherals | Output pins                                  | Resolution |
|---------------------------------------|-------------|----------------------------------------------|------------|
| STM32H742 / H743 / H745 / H747 / H750 / H753 / H755 / H757 | DAC1 | PA4 (DAC1_OUT1), PA5 (DAC1_OUT2)  | 12-bit     |
| STM32H723 / H725 / H730 / H733 / H735 | DAC1        | PA4 (DAC1_OUT1), PA5 (DAC1_OUT2)             | 12-bit     |
| STM32H7A3 / H7B0 / H7B3               | DAC1, DAC2  | PA4, PA5 (DAC1), PA6 (DAC2_OUT1)             | 12-bit     |
| STM32H7R3 / H7R7 / H7S3 / H7S7        | -           | no DAC - the module is empty (content under `DAC1`), unit tests only for the classic lines | -          |

Default DMA streams (`Dac_Get_DefaultConfig()`): DAC1 - DMA1 stream 5 / 6 (DAC2 - DMA1 stream 7 reported, DMA transfer refused).

Not yet tested on hardware (integration tests prepared for STM32H7 Nucleo boards).

---

## Module Structure

| File          | Description                                                                 |
|---------------|-----------------------------------------------------------------------------|
| `Dac_Port.h`  | Public API                                                                  |
| `Dac_Types.h` | Public types                                                                |
| `Dac.c`       | Clock, peripheral and channel configuration, calibration, data output       |
| `Dac_Dma.c`   | DMA buffer output (DMA stream per DAC channel, DMAMUX1 request)             |
| `Dac_Isr.c`   | DAC interrupt (DMA underrun of all channels, TIM6_DAC / DAC2 lines)         |
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
- Calibration is executed by `Dac_ChannelInit()` for pin outputs of DAC1 / DAC2 in buffered normal mode (the configured output mode is restored). The output pin should not be loaded during initialization.
- Without trigger (`DAC_TRIGGER_NONE`) the output is updated one APB clock cycle after the data write, otherwise on the trigger event. DAC has no trigger edge selection (rising edge).
- Wave generation requires a trigger (wave value is added to the data holding register on every trigger).
- DMA transfer requires a hardware trigger - DAC DMA requests are not generated without trigger or on software trigger. A DMA channel owned by an initialized DAC channel is rejected for every other DAC channel (both channels of one configuration, other peripheral or separate initialization) until the owner is deinitialized or reconfigured. DMA channels shall not be shared with other modules.
- DMA underrun or DMA transfer error stops the transfer and is reported through `ErrorCallback`.
- The DAC1 interrupt handler is registered on the `TIM6_DAC` line - TIM6 interrupts are not processed while a DAC1 DMA transfer is initialized.
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

- Unit tests (host, Unity / CMock / RegMem): `Tests/UnitTests/Test_Dac.c` - DAC registers of all peripherals are emulated, calibration flags by a HW model thread.
  ```bash
  cmake --preset STM32H743xI_UnitTest && cmake --build --preset STM32H743xI_UnitTest
  ctest --test-dir Build_STM32H743xI_UnitTest -L Dac
  ```
- Integration tests (target, STM32H7 Nucleo boards): `Tests/IntegrationTests/ItTest_Dac.c` - prepared in the integration test phase.

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
