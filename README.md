# DAC MCAL Module

The **DAC (Digital-to-Analog Converter) MCAL module** provides an abstraction layer for managing digital-to-analog conversion peripherals on STM32 microcontrollers.  
This module is part of the MCAL layer and ensures a unified API across different STM32 families.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family.

---

## Features

- Initialization and deinitialization of DAC peripheral and its channels
- Support for default configuration retrieval
- 12-bit right/left aligned and 8-bit right aligned data formats
- Single and dual channel (synchronous) output update
- Software and hardware (timer / EXTI / LPTIM) conversion triggers
- Output buffer enable/disable and output connection (pin / on-chip peripherals)
- Built-in waveform generation: noise (LFSR) and triangle wave
- Sample & hold mode for low-power operation
- Offset calibration (trimming)
- Polling, interrupt and DMA based transfer modes
- DMA underrun detection with error callback
- Standardized request state return values
- Versioning support for module management

---

## Supported Hardware

| MCU family | Peripheral | Channels | Resolution |
|------------|------------|----------|------------|
| STM32H503  | DAC1       | 2        | 12-bit     |
| STM32H563  | DAC1       | 2        | 12-bit     |

---

## Public API

### Module Management
- `dac_ModuleVersion_t     Dac_Get_ModuleVersion   ( void );`
- `dac_RequestState_t      Dac_Init                ( dac_Config_t * const dacConfig );`
- `dac_RequestState_t      Dac_Deinit              ( dac_PeriphId_t periphId );`
- `void                    Dac_Task                ( void );`
- `dac_RequestState_t      Dac_Get_DefaultConfig   ( dac_Config_t * const dacConfig );`

### Peripheral Control
- `dac_RequestState_t      Dac_Set_PeriphActive    ( dac_PeriphId_t periphId );`
- `dac_RequestState_t      Dac_Set_PeriphInactive  ( dac_PeriphId_t periphId );`
- `dac_RequestState_t      Dac_Get_PeriphState     ( dac_PeriphId_t periphId, dac_FlagState_t * const reqState );`

### Channel Configuration
- `dac_RequestState_t      Dac_ChannelInit         ( dac_PeriphId_t periphId, dac_ChannelConfig_t * const channelConfig );`
- `dac_RequestState_t      Dac_Set_ChannelActive   ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
- `dac_RequestState_t      Dac_Set_ChannelInactive ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
- `dac_RequestState_t      Dac_Set_OutputBuffer    ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_OutputBuffer_t outBuffer );`
- `dac_RequestState_t      Dac_Get_OutputBuffer    ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_OutputBuffer_t * const outBuffer );`
- `dac_RequestState_t      Dac_Set_OutputConnection( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_OutputConnect_t outConnect );`
- `dac_RequestState_t      Dac_Get_OutputConnection( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_OutputConnect_t * const outConnect );`
- `dac_RequestState_t      Dac_Set_Calibration     ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
- `dac_RequestState_t      Dac_Get_TrimmingValue   ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TrimValue_t * const trimValue );`

### Trigger Configuration
- `dac_RequestState_t      Dac_Set_TriggerSrc      ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TriggerId_t triggerSrc );`
- `dac_RequestState_t      Dac_Get_TriggerSrc      ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TriggerId_t * const triggerSrc );`
- `dac_RequestState_t      Dac_Set_SwTrigger       ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`

### Waveform Generation
- `dac_RequestState_t      Dac_Set_WaveMode        ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_WaveMode_t waveMode );`
- `dac_RequestState_t      Dac_Get_WaveMode        ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_WaveMode_t * const waveMode );`
- `dac_RequestState_t      Dac_Set_WaveAmplitude   ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_WaveAmplitude_t amplitude );`
- `dac_RequestState_t      Dac_Get_WaveAmplitude   ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_WaveAmplitude_t * const amplitude );`

### Sample & Hold
- `dac_RequestState_t      Dac_Set_SampleHoldConfig( dac_PeriphId_t periphId, dac_ChannelId_t channelId, const dac_SampleHoldConfig_t * const shConfig );`
- `dac_RequestState_t      Dac_Get_SampleHoldConfig( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_SampleHoldConfig_t * const shConfig );`

### Data Output
- `dac_RequestState_t      Dac_Set_DataFormat      ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_DataFormat_t dataFormat );`
- `dac_RequestState_t      Dac_Get_DataFormat      ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_DataFormat_t * const dataFormat );`
- `dac_RequestState_t      Dac_Set_Data            ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Data_t data );`
- `dac_RequestState_t      Dac_Set_DualData        ( dac_PeriphId_t periphId, dac_Data_t dataCh1, dac_Data_t dataCh2 );`
- `dac_RequestState_t      Dac_Get_OutputData      ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Data_t * const data );`
- `dac_RequestState_t      Dac_Set_Voltage_mV      ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Voltage_mV_t voltage );`

### Flags
- `dac_RequestState_t      Dac_Get_Flag            ( dac_PeriphId_t periphId, dac_FlagId_t flagId, dac_FlagState_t * const flagState );`
- `dac_RequestState_t      Dac_Clear_Flag          ( dac_PeriphId_t periphId, dac_FlagId_t flagId );`

---

## Transfer Modes

The channel transfer mode is selected in `dac_ChannelConfig_t` and handled by a dedicated sub-module:

| Mode    | Source         | Description                                                        |
|---------|----------------|--------------------------------------------------------------------|
| Polling | `Dac_Poll.c`   | Data written directly by the application / `Dac_Task`              |
| ISR     | `Dac_Isr.c`    | DMA underrun and error handling via interrupts with callbacks      |
| DMA     | `Dac_Dma.c`    | Continuous waveform output from memory buffer (circular / normal)  |

---

## DMA Support
- `dac_RequestState_t      Dac_Set_DmaStart        ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_DataCnt_t dataCnt, const dac_Data_t * const dataBuff );`
- `dac_RequestState_t      Dac_Set_DmaStop         ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
- `dac_RequestState_t      Dac_Get_DmaRemainingCnt ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_DataCnt_t * const dataCnt );`
- `dac_RequestState_t      Dac_Set_DmaReqActive    ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
- `dac_RequestState_t      Dac_Set_DmaReqInactive  ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
- `dac_RequestState_t      Dac_Get_DmaReqState     ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_FlagState_t * const reqState );`

---

## Interrupts and Callbacks
- `dac_RequestState_t      Dac_Set_IrqPriority         ( dac_PeriphId_t periphId, dac_IrqPrio_t irqPrio );`
- `dac_RequestState_t      Dac_Get_IrqPriority         ( dac_PeriphId_t periphId, dac_IrqPrio_t * const irqPrio );`
- DMA Underrun:
  - `dac_RequestState_t      Dac_Set_UnderrunIrqActive   ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
  - `dac_RequestState_t      Dac_Set_UnderrunIrqInactive ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );`
  - `dac_RequestState_t      Dac_Get_UnderrunIrqState    ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_FlagState_t * const reqState );`
  - `dac_RequestState_t      Dac_Set_UnderrunIsrCallback ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_UnderrunIrqCallback_t * const callback );`
- Transfer (DMA):
  - `dac_RequestState_t      Dac_Set_XferHalfCallback    ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_XferIrqCallback_t * const callback );`
  - `dac_RequestState_t      Dac_Set_XferDoneCallback    ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_XferIrqCallback_t * const callback );`

---

## GPIO Configuration
- `dac_RequestState_t      Dac_InitOutGpio         ( dac_OutPin_t pinId );`

DAC outputs on STM32H5: `DAC1_OUT1` → **PA4**, `DAC1_OUT2` → **PA5** (analog mode).

---

## Usage Notes

- The DAC module is hardware dependent and must be configured per STM32 family branch.  
- The **default configuration API** helps to ensure safe initialization.  
- Calibration should be performed after power-up (with the channel disabled) for best offset accuracy.  
- When a hardware trigger is used, the output is updated only on the trigger event; with no trigger, the output is updated one APB clock cycle after the data write.  
- The `Task` function should be periodically called if polling mode or background processing is used.  
- In DMA mode, an underrun occurs when a new trigger arrives before the previous DMA request was served — enable the underrun interrupt to detect it.

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
