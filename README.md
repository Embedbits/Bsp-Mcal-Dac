# DAC MCAL Module

The **DAC (Digital-to-Analog Converter) MCAL module** provides an abstraction layer for managing digital-to-analog conversion peripherals on STM32 microcontrollers.  
This module is part of the MCAL layer and ensures a unified API across different STM32 families.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family.

---

## Features

STM32F4 family (public interface `Dac_Port.h` identical with STM32H5):

- Initialization and deinitialization of DAC peripheral and its channels, default configuration
- DAC bus clock (APB1) handled through the RCC module
- Output modes: buffered / unbuffered pin output
- Software and hardware (TIM2 / TIM4 / TIM5 / TIM6 / TIM7 / TIM8 TRGO, EXTI line 9) conversion triggers
- 12-bit right / left aligned and 8-bit right aligned data formats
- Single and dual channel (simultaneous) output update, output register read-back
- Built-in waveform generation: noise (LFSR) and triangle wave
- Voltage conversion helpers (output value from voltage, output voltage from output register)
- DMA buffer output through the DMA module (fixed DMA1 streams, one shot / circular) with half transfer, transfer complete and error (DMA underrun, DMA transfer error) callbacks
- Every register write verified by read-back, HW state checked before configuration changes

Features of the common interface not available on STM32F4 DAC accept only the inactive / standard configuration, otherwise `DAC_REQUEST_ERROR` is returned:

| Feature                                        | STM32F4 behavior                                                     |
|------------------------------------------------|----------------------------------------------------------------------|
| Kernel clock multiplexer                       | `DAC_CLK_SRC_PCLK1` only                                             |
| Internal output connection (on-chip periph.)   | `DAC_OUTPUT_PIN_INTERNAL_BUFFERED` / `DAC_OUTPUT_INTERNAL` refused   |
| Sample & hold, sample & hold clock             | only `State = DAC_FUNCTION_INACTIVE`, clock selection always refused |
| Offset calibration, trimming                   | always refused                                                       |
| Flags (calibration, busy, ready)               | only `DAC_FLAG_DMA_UNDERRUN`                                         |
| GPDMA errors (config, update, trigger overrun) | never reported                                                       |

---

## Supported Hardware

| MCU family | Peripheral | Channels                                         | Output pins            | DMA (request mapping)                  | Resolution |
|------------|------------|--------------------------------------------------|------------------------|----------------------------------------|------------|
| STM32F4    | DAC        | 2 (1 on devices without `DAC_CHANNEL2_SUPPORT`)  | PA4 (OUT1), PA5 (OUT2) | DMA1 stream 5 / stream 6, channel 7    | 12-bit     |

Tested on STM32F4DISCOVERY (STM32F407VG).

---

## Module Structure

| File          | Description                                                                 |
|---------------|-----------------------------------------------------------------------------|
| `Dac_Port.h`  | Public API                                                                  |
| `Dac_Types.h` | Public types                                                                |
| `Dac.c`       | Clock, peripheral and channel configuration, data output                    |
| `Dac_Dma.c`   | DMA buffer output (fixed DMA stream per DAC channel)                        |
| `Dac_Isr.c`   | DAC interrupt (DMA underrun of both channels, TIM6_DAC line)                |
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
- `dac_RequestState_t  Dac_Set_ShClockSource ( dac_ShClkSrc_t clkSource );` (not supported)
- `dac_RequestState_t  Dac_Get_ShClockSource ( dac_ShClkSrc_t * const clkSource );` (not supported)

### Peripheral and Channel Configuration
- `dac_RequestState_t  Dac_PeriphInit           ( dac_PeriphConfig_t * const periphConfig );`
- `dac_RequestState_t  Dac_ChannelInit          ( dac_PeriphId_t periphId, dac_ChannelConfig_t * const channelConfig );`
- `dac_RequestState_t  Dac_Set_ChannelActive    ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
- `dac_RequestState_t  Dac_Set_ChannelInactive  ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
- `dac_RequestState_t  Dac_Get_ChannelState     ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_FunctionState_t * const channelState );`
- `Dac_Set_` / `Dac_Get_` `OutputMode`, `TriggerSrc`, `DataFormat`, `WaveMode`, `WaveAmplitude`, `SampleHoldConfig`

### Calibration (not supported by STM32F4)
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

- Channel configuration (output mode, trigger, wave) is allowed only while the channel is disabled - otherwise `DAC_REQUEST_ERROR` is returned and no register is modified.
- STM32F4 DAC output is always connected to the pin - the pin is configured as analog by `Dac_ChannelInit()`. STM32F4 DAC has no ready flag, the output settles within tWAKEUP after the channel enable (not waited).
- Without trigger (`DAC_TRIGGER_NONE`) the output is updated one APB clock cycle after the data write, otherwise on the trigger event. DAC has no trigger edge selection (rising edge).
- Wave generation requires a trigger (wave value is added to the data holding register on every trigger).
- DMA transfer requires a hardware trigger - DAC DMA requests are not generated without trigger or on software trigger. DAC requests are connected to fixed streams (DAC channel 1 = DMA1 stream 5, channel 2 = DMA1 stream 6, channel selection 7), `DmaPeriphId` / `DmaChannelId` must match this mapping (`Dac_Get_DefaultConfig()` fills them). Circular buffers use the circular mode of the stream.
- On every trigger the data holding register is moved to the output and the next buffer item is requested - the last item of a one shot buffer reaches the output with the trigger following the transfer complete.
- The DMA buffer must not be placed in CCM data RAM (not accessible by DMA).
- DMA underrun or DMA transfer error stops the transfer and is reported through `ErrorCallback`.
- The DMA underrun interrupt shares the NVIC line with TIM6 (`TIM6_DAC_IRQn`) - the line handler is replaced by the DAC handler when a DMA transfer is initialized, TIM6 interrupts can not be used together with DAC DMA. The line is disabled in NVIC by `Dac_Deinit()` only if the DAC module enabled it.
- `Dac_Task()` has no periodic processing, it is provided for consistency of the MCAL interface.

### Example

```c
dac_Config_t dacConfig;

(void)Dac_Get_DefaultConfig( &dacConfig );                  /* Channel 1, buffered output on PA4 */
dacConfig.PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_1 ].InitValue = 2048u;

if( DAC_REQUEST_OK == Dac_Init( &dacConfig ) )
{
    (void)Dac_Set_Voltage( DAC_PERIPH_1, DAC_CHANNEL_1, 3000u, 1200u );  /* 1.2 V at VREF+ = 3.0 V */
}
```

---

## Testing

- Unit tests (host, Unity / CMock / RegMem): `Tests/UnitTests/Test_Dac.c`
  ```bash
  cmake --preset STM32F407xG_UnitTest && cmake --build --preset STM32F407xG_UnitTest
  ctest --test-dir Build/STM32F407xG_UnitTest -L Dac
  ```
- Integration tests (target, STM32F4DISCOVERY): `Tests/IntegrationTests/ItTest_Dac.c` - DAC_OUT1 (PA4) / DAC_OUT2 (PA5) are measured by ADC1 channels 4 / 5 (same pins), TIM6 TRGO triggers DMA transfers, no external wiring is needed.

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
