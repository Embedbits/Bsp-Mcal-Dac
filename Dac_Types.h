/**
 * \defgroup Dac Dac
 * \brief Dac module
 */

/**
 * \author Mr.Nobody
 * \file Dac_Types.h
 * \ingroup Dac
 * \brief Digital-to-Analog Converter (DAC) module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through Port file.
 *
 */

#ifndef DAC_DAC_TYPES_H
#define DAC_DAC_TYPES_H
/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Stm32_dac.h"                      /* DAC utilities functionality    */
#include "Dma_Types.h"                      /* DMA types definitions          */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define DAC_NULL_PTR                        ( ( void* ) 0u )

/** Maximum value of 12-bit right aligned data */
#define DAC_DATA_12BIT_MAX                  ( 0x0FFFu )

/** Maximum value of 8-bit right aligned data */
#define DAC_DATA_8BIT_MAX                   ( 0x00FFu )

/** Maximum offset trimming value (OTRIMx is 5 bits wide) */
#define DAC_TRIM_VALUE_MAX                  ( 0x1Fu )

/** Maximum sample and hold sample time (TSAMPLEx is 10 bits wide) */
#define DAC_SH_SAMPLE_TIME_MAX              ( 0x03FFu )

/** Maximum sample and hold hold time (THOLDx is 10 bits wide) */
#define DAC_SH_HOLD_TIME_MAX                ( 0x03FFu )

/** Maximum sample and hold refresh time (TREFRESHx is 8 bits wide) */
#define DAC_SH_REFRESH_TIME_MAX             ( 0x00FFu )

/* ========================== EXPORTED MACROS =============================== */

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t dac_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t dac_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t dac_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    dac_MajorVersion_t Major; /**< Major version */
    dac_MinorVersion_t Minor; /**< Minor version */
    dac_PatchVersion_t Patch; /**< Patch version */
}   dac_ModuleVersion_t;


/** Function status enumeration */
typedef enum
{
    DAC_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    DAC_FUNCTION_ACTIVE         /**< Function status is active   */
}   dac_FunctionState_t;


/** Enumeration used to signal request processing state */
typedef enum
{
    DAC_REQUEST_ERROR = 0u, /**< Processing request failed  */
    DAC_REQUEST_OK          /**< Processing request succeed */
}   dac_RequestState_t;


/** Flag states enumeration */
typedef enum
{
    DAC_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    DAC_FLAG_ACTIVE         /**< Active flag state   */
}   dac_FlagState_t;

/* -------------------------------------------------------------------------- */
/* ------------------------ Clock source configuration ---------------------- */
/* -------------------------------------------------------------------------- */

/** \brief List of available DAC clock sources
 *
 * \note STM32G4 DAC has no kernel clock multiplexer - every DAC peripheral is clocked by the
 *       AHB clock (HCLK), its clock is enabled by \ref Dac_PeriphInit. */
typedef enum
{
    DAC_CLK_SRC_HCLK = 0u, /**< AHB clock (HCLK) as clock source of DAC */
    DAC_CLK_SRC_CNT        /**< Count of DAC clock sources              */
}   dac_ClkSrc_t;


/** \brief List of sample and hold clock sources
 *
 * \note STM32G4 sample and hold mode is clocked by LSI only (started by RCC module) -
 *       \ref DAC_SH_CLK_SRC_LSE is refused. */
typedef enum
{
    DAC_SH_CLK_SRC_LSI = 0u, /**< Low Speed Internal oscillator (LSI) as sample and hold clock */
    DAC_SH_CLK_SRC_LSE,      /**< Low Speed External oscillator (LSE) - not supported          */
    DAC_SH_CLK_SRC_CNT       /**< Count of sample and hold clock sources                       */
}   dac_ShClkSrc_t;

/* -------------------------------------------------------------------------- */
/* ------------------------ Peripheral configuration ------------------------ */
/* -------------------------------------------------------------------------- */

/** \brief Digital-to-Analog Converter (DAC) peripheral enumeration
 *
 * - DAC1: 2 channels, buffered external outputs DAC1_OUT1 (PA4), DAC1_OUT2 (PA5)
 * - DAC2: 1 channel, buffered external output DAC2_OUT1 (PA6) - devices with DAC2
 * - DAC3: 2 channels, internal outputs only (OPAMP / COMP, no buffer)
 * - DAC4: 2 channels, internal outputs only (OPAMP / COMP, no buffer) - devices with DAC4
 */
typedef enum
{
    DAC_PERIPH_1 = 0u, /**< Digital-to-Analog Converter (DAC) peripheral 1            */
#if defined(DAC2)
    DAC_PERIPH_2,      /**< Digital-to-Analog Converter (DAC) peripheral 2            */
#endif /* DAC2 */
#if defined(DAC3)
    DAC_PERIPH_3,      /**< Digital-to-Analog Converter (DAC) peripheral 3            */
#endif /* DAC3 */
#if defined(DAC4)
    DAC_PERIPH_4,      /**< Digital-to-Analog Converter (DAC) peripheral 4            */
#endif /* DAC4 */
    DAC_PERIPH_CNT     /**< Count of Digital-to-Analog Converter (DAC) peripherals    */
}   dac_PeriphId_t;


/** \brief Digital-to-Analog Converter (DAC) channel enumeration
 *
 * \note DAC2 has channel 1 only - requests for its channel 2 return error.
 */
typedef enum
{
    DAC_CHANNEL_1 = 0u, /**< DAC channel 1 (DAC1_OUT1 PA4, DAC2_OUT1 PA6) */
    DAC_CHANNEL_2,      /**< DAC channel 2 (DAC1_OUT2 PA5)                */
    DAC_CHANNEL_CNT     /**< Count of DAC channels                        */
}   dac_ChannelId_t;


/** \brief List of channel output modes (buffer and output connection, MODEx[1:0])
 *
 * \note DAC3 / DAC4 outputs are connected to on-chip peripherals only (no buffer, no pin) -
 *       only \ref DAC_OUTPUT_INTERNAL is accepted.
 */
typedef enum
{
    DAC_OUTPUT_PIN_BUFFERED = 0u,      /**< Output connected to external pin, output buffer enabled                      */
    DAC_OUTPUT_PIN_INTERNAL_BUFFERED,  /**< Output connected to external pin and on-chip peripherals, buffer enabled     */
    DAC_OUTPUT_PIN_UNBUFFERED,         /**< Output connected to external pin, output buffer disabled                     */
    DAC_OUTPUT_INTERNAL,               /**< Output connected to on-chip peripherals only (e.g. ADC), buffer disabled     */
    DAC_OUTPUT_CNT                     /**< Count of output modes                                                        */
}   dac_OutputMode_t;


/** \brief List of available conversion trigger sources
 *
 * \note DAC_TRIGGER_NONE - trigger is disabled (TENx = 0), the output is updated a few bus clock
 *       cycles after the data holding register write.
 * \note DAC has no trigger edge selection - conversion is started on the rising edge of the
 *       selected trigger signal.
 * \note Trigger connection depends on the DAC peripheral (RM0440 DAC interconnection): TIM1 TRGO
 *       for DAC3 only, TIM8 TRGO for DAC1 / DAC2 / DAC4, HRTIM DAC trigger 1 for DAC1 / DAC4,
 *       2 for DAC2, 3 for DAC3. A trigger not connected to the peripheral is refused.
 * \note Sawtooth wave triggers (HRTIM step / reset) are not part of the interface.
 */
typedef enum
{
    DAC_TRIGGER_NONE = 0u,         /**< No trigger, output updated after data write */
    DAC_TRIGGER_SOFTWARE,          /**< Software trigger (Dac_Set_SwTrigger())      */
    DAC_TRIGGER_TIM1_TRGO,         /**< External trigger TIM1 TRGO (DAC3)           */
    DAC_TRIGGER_TIM2_TRGO,         /**< External trigger TIM2 TRGO                  */
    DAC_TRIGGER_TIM3_TRGO,         /**< External trigger TIM3 TRGO                  */
    DAC_TRIGGER_TIM4_TRGO,         /**< External trigger TIM4 TRGO                  */
    DAC_TRIGGER_TIM6_TRGO,         /**< External trigger TIM6 TRGO                  */
    DAC_TRIGGER_TIM7_TRGO,         /**< External trigger TIM7 TRGO                  */
    DAC_TRIGGER_TIM8_TRGO,         /**< External trigger TIM8 TRGO (DAC1/2/4)       */
    DAC_TRIGGER_TIM15_TRGO,        /**< External trigger TIM15 TRGO                 */
    DAC_TRIGGER_EXTI_LINE9,        /**< External trigger EXTI line 9                */
#if defined(HRTIM1)
    DAC_TRIGGER_HRTIM_DAC_TRG1,    /**< External trigger HRTIM DAC trigger 1 (DAC1/4) */
    DAC_TRIGGER_HRTIM_DAC_TRG2,    /**< External trigger HRTIM DAC trigger 2 (DAC2)   */
    DAC_TRIGGER_HRTIM_DAC_TRG3,    /**< External trigger HRTIM DAC trigger 3 (DAC3)   */
#endif /* HRTIM1 */
    DAC_TRIGGER_CNT                /**< Count of trigger sources                    */
}   dac_TriggerId_t;


/** \brief List of data formats (selects the data holding register used for the output value)
 *
 * \note STM32G4 signed data format (SINFORMATx) is not part of the interface - unsigned data.
 */
typedef enum
{
    DAC_DATA_FORMAT_12BIT_RIGHT = 0u, /**< 12-bit right aligned data (0 - 4095)                              */
    DAC_DATA_FORMAT_12BIT_LEFT,       /**< 12-bit left aligned data (bits 15:4 used, bits 3:0 are ignored)   */
    DAC_DATA_FORMAT_8BIT_RIGHT,       /**< 8-bit right aligned data (0 - 255)                                */
    DAC_DATA_FORMAT_CNT               /**< Count of data formats                                             */
}   dac_DataFormat_t;


/** \brief List of wave auto generation modes
 *
 * \note STM32G4 sawtooth wave generation is not part of the interface.
 */
typedef enum
{
    DAC_WAVE_NONE = 0u, /**< Wave generation disabled                                     */
    DAC_WAVE_NOISE,     /**< Pseudo noise wave (LFSR) added to the data holding register  */
    DAC_WAVE_TRIANGLE,  /**< Triangle wave added to the data holding register             */
    DAC_WAVE_CNT        /**< Count of wave generation modes                               */
}   dac_WaveMode_t;


/** \brief List of wave amplitudes (MAMPx)
 *
 * - Triangle wave: amplitude of the triangle in LSB (1 - 4095).
 * - Noise wave:    LFSR bits unmasked, amplitude 2^n - 1 unmasks LFSR bits [n-1:0].
 */
typedef enum
{
    DAC_WAVE_AMPLITUDE_1 = 0u, /**< Amplitude 1    / LFSR bit 0 unmasked       */
    DAC_WAVE_AMPLITUDE_3,      /**< Amplitude 3    / LFSR bits [1:0] unmasked  */
    DAC_WAVE_AMPLITUDE_7,      /**< Amplitude 7    / LFSR bits [2:0] unmasked  */
    DAC_WAVE_AMPLITUDE_15,     /**< Amplitude 15   / LFSR bits [3:0] unmasked  */
    DAC_WAVE_AMPLITUDE_31,     /**< Amplitude 31   / LFSR bits [4:0] unmasked  */
    DAC_WAVE_AMPLITUDE_63,     /**< Amplitude 63   / LFSR bits [5:0] unmasked  */
    DAC_WAVE_AMPLITUDE_127,    /**< Amplitude 127  / LFSR bits [6:0] unmasked  */
    DAC_WAVE_AMPLITUDE_255,    /**< Amplitude 255  / LFSR bits [7:0] unmasked  */
    DAC_WAVE_AMPLITUDE_511,    /**< Amplitude 511  / LFSR bits [8:0] unmasked  */
    DAC_WAVE_AMPLITUDE_1023,   /**< Amplitude 1023 / LFSR bits [9:0] unmasked  */
    DAC_WAVE_AMPLITUDE_2047,   /**< Amplitude 2047 / LFSR bits [10:0] unmasked */
    DAC_WAVE_AMPLITUDE_4095,   /**< Amplitude 4095 / LFSR bits [11:0] unmasked */
    DAC_WAVE_AMPLITUDE_CNT     /**< Count of wave amplitudes                   */
}   dac_WaveAmplitude_t;


/** \brief Type representing DAC data (format given by \ref dac_DataFormat_t) */
typedef uint16_t dac_Data_t;


/** \brief Type representing voltage in millivolts */
typedef uint16_t dac_Voltage_mV_t;


/** \brief Type representing offset trimming value (0 - \ref DAC_TRIM_VALUE_MAX) */
typedef uint8_t dac_TrimValue_t;


/** \brief Type representing sample and hold sample time in sample and hold clock cycles
 *         (0 - \ref DAC_SH_SAMPLE_TIME_MAX) */
typedef uint16_t dac_ShSampleTime_t;


/** \brief Type representing sample and hold hold time in sample and hold clock cycles
 *         (0 - \ref DAC_SH_HOLD_TIME_MAX) */
typedef uint16_t dac_ShHoldTime_t;


/** \brief Type representing sample and hold refresh time in sample and hold clock cycles
 *         (0 - \ref DAC_SH_REFRESH_TIME_MAX) */
typedef uint8_t dac_ShRefreshTime_t;


/** \brief List of DAC flags */
typedef enum
{
    DAC_FLAG_DMA_UNDERRUN = 0u, /**< DMA underrun (DMAUDRx), cleared by Dac_Clear_Flag()               */
    DAC_FLAG_CALIBRATION,       /**< Offset calibration status (CAL_FLAGx)                             */
    DAC_FLAG_SH_WRITE_BUSY,     /**< Sample time write ongoing (BWSTx)                                 */
    DAC_FLAG_READY,             /**< Channel ready (DACxRDY)                                           */
    DAC_FLAG_CNT                /**< Count of DAC flags                                                */
}   dac_FlagId_t;

/* -------------------------------------------------------------------------- */
/* ---------------------- Data handling configuration ----------------------- */
/* -------------------------------------------------------------------------- */

/** \brief List of channel data transfer modes */
typedef enum
{
    DAC_TRANSFER_MODE_DIRECT = 0u, /**< Output value written by Dac_Set_Data() / Dac_Set_DualData()       */
    DAC_TRANSFER_MODE_DMA,         /**< Output values moved from buffer by DMA on every hardware trigger  */
    DAC_TRANSFER_MODE_CNT          /**< Count of data transfer modes                                      */
}   dac_TransferMode_t;


/** \brief List of data buffer handling modes */
typedef enum
{
    DAC_BUFFER_MODE_ONE_SHOT = 0u, /**< Transfer stops when the whole buffer was output              */
    DAC_BUFFER_MODE_CIRCULAR,      /**< Transfer continues from the buffer start after the last item */
    DAC_BUFFER_MODE_CNT            /**< Count of buffer modes                                        */
}   dac_BufferMode_t;


/** \brief Type representing data buffer size in count of \ref dac_Data_t items */
typedef uint16_t dac_BufferSize_t;


/** \brief List of data transfer errors reported through \ref dac_ErrCallback_t
 *
 * \note STM32G4 reports DAC_ERROR_DMA_UNDERRUN and DAC_ERROR_DMA_TRANSFER only, the other
 *       errors belong to DMA controllers of other families.
 */
typedef enum
{
    DAC_ERROR_DMA_UNDERRUN = 0u,   /**< DAC DMA underrun - trigger came before the previous DMA request was served */
    DAC_ERROR_DMA_TRANSFER,        /**< DMA transfer error (bus error during transfer)                           */
    DAC_ERROR_DMA_CONFIG,          /**< DMA configuration error - not reported by STM32G4                        */
    DAC_ERROR_DMA_CONFIG_UPDATE,   /**< DMA configuration update error - not reported by STM32G4                 */
    DAC_ERROR_DMA_TRIGGER_OVERRUN, /**< DMA trigger overrun - not reported by STM32G4                            */
    DAC_ERROR_CNT                  /**< Count of data transfer errors                                            */
}   dac_ErrorId_t;


/** DMA peripherals enumeration list (DAC requests are routed to any channel by DMAMUX1) */
typedef enum
{
    DAC_DMA_PERIPH_1 = DMA_PERIPH_1, /**< DMA peripheral 1 identification */
#if defined(DMA2)
    DAC_DMA_PERIPH_2 = DMA_PERIPH_2, /**< DMA peripheral 2 identification */
#endif
    DAC_DMA_PERIPH_CNT               /**< Count of DMA peripherals        */
}   dac_DmaPeriphId_t;


/** Enumeration of available channels of DMA peripheral */
typedef enum
{
    DAC_DMA_CHANNEL_1 = DMA_CHANNEL_1, /**< DMA channel 1 */
    DAC_DMA_CHANNEL_2 = DMA_CHANNEL_2, /**< DMA channel 2 */
    DAC_DMA_CHANNEL_3 = DMA_CHANNEL_3, /**< DMA channel 3 */
    DAC_DMA_CHANNEL_4 = DMA_CHANNEL_4, /**< DMA channel 4 */
    DAC_DMA_CHANNEL_5 = DMA_CHANNEL_5, /**< DMA channel 5 */
    DAC_DMA_CHANNEL_6 = DMA_CHANNEL_6, /**< DMA channel 6 */
#if defined(DMA1_Channel7)
    DAC_DMA_CHANNEL_7 = DMA_CHANNEL_7, /**< DMA channel 7 */
#endif
#if defined(DMA1_Channel8)
    DAC_DMA_CHANNEL_8 = DMA_CHANNEL_8, /**< DMA channel 8 */
#endif
    DAC_DMA_CHANNEL_CNT                /**< Count of DMA channels */
}   dac_DmaChannelId_t;


/** DMA channel priority options enumeration */
typedef enum
{
    DAC_DMA_PRIORITY_LOW      = DMA_PRIORITY_LOW     , /**< Priority level : Low       */
    DAC_DMA_PRIORITY_MEDIUM   = DMA_PRIORITY_MEDIUM  , /**< Priority level : Medium    */
    DAC_DMA_PRIORITY_HIGH     = DMA_PRIORITY_HIGH    , /**< Priority level : High      */
    DAC_DMA_PRIORITY_VERYHIGH = DMA_PRIORITY_VERYHIGH, /**< Priority level : Very high */
}   dac_DmaPriority_t;


/** \brief Type representing interrupt priority of the DAC (DMA underrun) interrupt
 *
 * \note The DAC interrupt shares the NVIC line with TIM6 (DAC1 / DAC3, TIM6_DAC) or TIM7
 *       (DAC2 / DAC4, TIM7_DAC) - the priority of the last initialized channel is applied.
 */
typedef uint32_t dac_IrqPrio_t;


/** \brief Data transfer event callback (half transfer, transfer complete) */
typedef void ( dac_Callback_t )( void );


/** \brief Data transfer error callback, error identification is given as parameter */
typedef void ( dac_ErrCallback_t )( dac_ErrorId_t errorId );

/* -------------------------------------------------------------------------- */
/* ------------------------ Configuration structure ------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Channel data transfer configuration
 *
 * DAC_TRANSFER_MODE_DMA requires a hardware trigger (DAC DMA requests are not generated on
 * software trigger). Buffer items are written to the data holding register selected by the
 * channel data format.
 *
 * Callback events:
 * - HalfTransferCallback:     BufferSize / 2 items were output
 * - TransferCompleteCallback: BufferSize items were output (in circular mode the output
 *                             continues with DataBuffer[ 0 ])
 * - ErrorCallback:            DMA underrun or DMA error (\ref dac_ErrorId_t), the transfer is stopped
 *
 * Unused callback shall be set to DAC_NULL_PTR. All other fields are used only in
 * DAC_TRANSFER_MODE_DMA. The DMA channel shall not be used by another channel or module and
 * DataBuffer shall be accessible by DMA.
 */
typedef struct
{
    dac_TransferMode_t  TransferMode;             /**< Data transfer mode (DIRECT / DMA)                       */
    const dac_Data_t   *DataBuffer;               /**< Buffer of output values (DMA mode, must not be NULL)    */
    dac_BufferSize_t    BufferSize;               /**< Buffer size in count of dac_Data_t items (> 0)          */
    dac_BufferMode_t    BufferMode;               /**< One shot / circular buffer handling                     */

    dac_DmaPeriphId_t   DmaPeriphId;              /**< DMA peripheral                                          */
    dac_DmaChannelId_t  DmaChannelId;             /**< DMA channel                                             */
    dac_DmaPriority_t   DmaPriority;              /**< DMA channel priority                                    */

    dac_IrqPrio_t       IrqPriority;              /**< DAC interrupt priority (DMA underrun)                   */

    dac_Callback_t     *HalfTransferCallback;     /**< Half of the buffer output. DAC_NULL_PTR if not used.    */
    dac_Callback_t     *TransferCompleteCallback; /**< Whole buffer output. DAC_NULL_PTR if not used.          */
    dac_ErrCallback_t  *ErrorCallback;            /**< Transfer error. DAC_NULL_PTR if not used.               */
}   dac_DataConfig_t;


/** \brief Sample and hold configuration of a channel (times in sample and hold clock cycles) */
typedef struct
{
    dac_FunctionState_t State;       /**< Sample and hold mode active / inactive  */
    dac_ShSampleTime_t  SampleTime;  /**< Sample phase duration (TSAMPLEx)        */
    dac_ShHoldTime_t    HoldTime;    /**< Hold phase duration (THOLDx)            */
    dac_ShRefreshTime_t RefreshTime; /**< Refresh phase duration (TREFRESHx)      */
}   dac_SampleHoldConfig_t;


/** \brief Channel configuration structure */
typedef struct
{
    dac_ChannelId_t        ChannelId;     /**< Channel identification                                     */
    dac_FunctionState_t    ChannelUsed;   /**< Channel is initialized by Dac_PeriphInit() (ACTIVE) or not */
    dac_OutputMode_t       OutputMode;    /**< Output buffer and connection                               */
    dac_TriggerId_t        TriggerId;     /**< Conversion trigger source                                  */
    dac_DataFormat_t       DataFormat;    /**< Data format of the output value                            */
    dac_WaveMode_t         WaveMode;      /**< Wave auto generation mode                                  */
    dac_WaveAmplitude_t    WaveAmplitude; /**< Wave amplitude / LFSR mask (ignored for DAC_WAVE_NONE)     */
    dac_SampleHoldConfig_t SampleHold;    /**< Sample and hold configuration                              */
    dac_Data_t             InitValue;     /**< Output value written on initialization (in DataFormat)     */
    dac_DataConfig_t       DataConfig;    /**< Data transfer configuration                                */
}   dac_ChannelConfig_t;


/** \brief Peripheral configuration structure */
typedef struct
{
    dac_PeriphId_t      PeriphId;                     /**< DAC peripheral identification                                     */
    dac_ShClkSrc_t      ShClockSource;                /**< Sample and hold clock (used if any channel has sample and hold)    */
    dac_ChannelConfig_t Channels[ DAC_CHANNEL_CNT ];  /**< Channel configuration, indexed by \ref dac_ChannelId_t            */
}   dac_PeriphConfig_t;


/** \brief Module configuration representing top layer configuration structure */
typedef struct
{
    dac_ClkSrc_t       ClockSource;                    /**< DAC clock source (AHB clock)                          */
    dac_PeriphConfig_t PeriphConfig[ DAC_PERIPH_CNT ]; /**< Peripheral configuration, indexed by \ref dac_PeriphId_t */
}   dac_Config_t;


#endif /* DAC_DAC_TYPES_H */

