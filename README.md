# DAC MCAL Module

The **DAC (Digital-to-Analog Converter) MCAL module** provides an abstraction layer for managing digital-to-analog conversion peripherals on STM32 microcontrollers.  
This module is part of the MCAL layer and ensures a unified API across different STM32 families.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family.

---

## Features

- Initialization and deinitialization of DAC peripheral and its channels, default configuration
- Shared ADC / DAC kernel clock handled through the RCC module (an active ADC clock is never switched)
- High frequency interface mode (HFSEL) selected automatically from the AHB clock
- Output modes: buffered / unbuffered pin output, internal connection to on-chip peripherals (ADC)
- Software and hardware (timer TRGO / EXTI / LPTIM) conversion triggers
- 12-bit right / left aligned and 8-bit right aligned data formats
- Single and dual channel (simultaneous) output update, output register read-back
- Built-in waveform generation: noise (LFSR) and triangle wave
- Sample & hold mode with timing, LSI / LSE sample & hold clock
- Offset calibration on initialization, trimming value readable / writable
- Voltage conversion helpers (output value from voltage, output voltage from output register)
- DMA buffer output through the GPDMA module (one shot / circular) with half transfer, transfer complete and error (DMA underrun) callbacks
- Every register write verified by read-back, HW state checked before configuration changes

---

## Supported Hardware

| MCU family | Peripheral | Channels | Output pins          | Resolution |
|------------|------------|----------|----------------------|------------|
| STM32H503  | DAC1       | 2        | PA4 (OUT1), PA5 (OUT2) | 12-bit     |
| STM32H563  | DAC1       | 2        | PA4 (OUT1), PA5 (OUT2) | 12-bit     |

---

## Module Structure

| File          | Description                                                                 |
|---------------|-----------------------------------------------------------------------------|
| `Dac_Port.h`  | Public API                                                                  |
| `Dac_Types.h` | Public types                                                                |
| `Dac.c`       | Clock, peripheral and channel configuration, calibration, data output       |
| `Dac_Dma.c`   | DMA buffer output (GPDMA channel per DAC channel)                           |
| `Dac_Isr.c`   | DAC interrupt (DMA underrun of both channels)                               |
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
- The kernel clock multiplexer is shared with ADC. If the ADC clock is already active, `Dac_Init()` accepts only the clock source selected by ADC.
- Calibration is executed by `Dac_ChannelInit()` in buffered normal mode (the configured output mode is restored). The output pin should not be loaded during initialization.
- Without trigger (`DAC_TRIGGER_NONE`) the output is updated one APB clock cycle after the data write, otherwise on the trigger event. DAC has no trigger edge selection (rising edge).
- Wave generation requires a trigger (wave value is added to the data holding register on every trigger).
- DMA transfer requires a hardware trigger - DAC DMA requests are not generated without trigger or on software trigger. Circular buffers use a self linked GPDMA transfer list node (`GPDMA_XFER_LIST_EXEC_CYCLIC_ALL`), the block is reloaded by GPDMA HW without SW intervention.
- DMA underrun or GPDMA error stops the transfer and is reported through `ErrorCallback`.
- Sample & hold requires a running LSI / LSE oscillator (not started by the DAC module).
- `Dac_Task()` has no periodic processing, it is provided for consistency of the MCAL interface.

### Example

```c
dac_Config_t dacConfig;

(void)Dac_Get_DefaultConfig( &dacConfig );                  /* Channel 1, buffered output on PA4 */
dacConfig.PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_1 ].InitValue = 2048u;

if( DAC_REQUEST_OK == Dac_Init( &dacConfig ) )
{
    (void)Dac_Set_Voltage( DAC_PERIPH_1, DAC_CHANNEL_1, 3300u, 1200u );  /* 1.2 V at VREF+ = 3.3 V */
}
```

---

## Testing

- Unit tests (host, Unity / CMock / RegMem): `Tests/UnitTests/Test_Dac.c`
  ```bash
  cmake --preset STM32H563xI_UnitTest && cmake --build --preset STM32H563xI_UnitTest
  ctest --test-dir Build_STM32H563xI_UnitTest -L Dac
  ```
- Integration tests (target, NUCLEO board): `Tests/IntegrationTests/ItTest_Dac.c` - DAC1_OUT1 (PA4) is measured by ADC1 channel 18 (same pin), no external wiring is needed.

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
