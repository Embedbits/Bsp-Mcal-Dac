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
 * STM32F7 DAC is the older DAC IP: no kernel clock multiplexer (APB1 bus clock),
 * output buffer enable / disable only (output always connected to the pin), no
 * sample and hold, no offset calibration, no channel ready flag. Types of these
 * features are kept for the common public interface, functions accept only the
 * inactive / standard configuration (see Dac.c).
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

/** Maximum offset trimming value (offset calibration is not supported by STM32F7) */
#define DAC_TRIM_VALUE_MAX                  ( 0x1Fu )

/** Maximum sample and hold sample time (sample and hold is not supported by STM32F7) */
#define DAC_SH_SAMPLE_TIME_MAX              ( 0x03FFu )

/** Maximum sample and hold hold time (sample and hold is not supported by STM32F7) */
#define DAC_SH_HOLD_TIME_MAX                ( 0x03FFu )

/** Maximum sample and hold refresh time (sample and hold is not supported by STM32F7) */
#define DAC_SH_REFRESH_TIME_MAX             ( 0x00FFu )

/** DAC peripheral identification bit offset in encoded DMA stream value */
#define DAC_DMA_BIT_MASK_PERIPH_BIT_OFFSET  ( 20u )

/** DAC channel identification bit offset in encoded DMA stream value */
#define DAC_DMA_BIT_MASK_CHANNEL_BIT_OFFSET ( 15u )

/** DMA peripheral identification bit offset in encoded DMA stream value */
#define DAC_DMA_BIT_MASK_DMA_BIT_OFFSET     ( 10u )

/** Stream identification bit offset in encoded DMA stream value */
#define DAC_DMA_BIT_MASK_STREAM_BIT_OFFSET  ( 5u )

/** Channel selection (CHSEL) bit offset in encoded DMA stream value */
#define DAC_DMA_BIT_MASK_CHSEL_BIT_OFFSET   ( 0u )

/** Mask of one field (5 bits) in encoded DMA stream value */
#define DAC_DMA_BIT_MASK_FIELD              ( 0x1Fu )

/* ========================== EXPORTED MACROS =============================== */

/**
 * \brief Encodes DMA stream (DAC peripheral, DAC channel, DMA peripheral, stream, channel selection) into
 *        single value of \ref dac_DmaCode_t
 *
 * The macro defines the values of the DMA stream list \ref dac_Dma_t, e.g. the DAC channel 1 request on DMA1
 * stream 5 (channel selection 7) is \ref DAC_DMA_DAC1_CH1_DMA1_STREAM5:
 * DAC_DMA_ENCODE( DAC_PERIPH_1, DAC_CHANNEL_1, DAC_DMA_PERIPH_1, DAC_DMA_CHANNEL_5, 7u )
 */
#define DAC_DMA_ENCODE( PERIPH_ID, CHANNEL_ID, DMA_ID, STREAM_ID, CHSEL )   ( (dac_DmaCode_t)( ( (uint32_t)(PERIPH_ID)  << DAC_DMA_BIT_MASK_PERIPH_BIT_OFFSET  ) | \
                                                                                               ( (uint32_t)(CHANNEL_ID) << DAC_DMA_BIT_MASK_CHANNEL_BIT_OFFSET ) | \
                                                                                               ( (uint32_t)(DMA_ID)     << DAC_DMA_BIT_MASK_DMA_BIT_OFFSET     ) | \
                                                                                               ( (uint32_t)(STREAM_ID)  << DAC_DMA_BIT_MASK_STREAM_BIT_OFFSET  ) | \
                                                                                               ( (uint32_t)(CHSEL)      << DAC_DMA_BIT_MASK_CHSEL_BIT_OFFSET   )   ) )

/** Stream is not configured by the module (value of the DAC_DMA_UNUSED item of the DMA stream list) */
#define DAC_DMA_CODE_UNUSED                 DAC_DMA_ENCODE( DAC_PERIPH_CNT, DAC_CHANNEL_CNT, DAC_DMA_PERIPH_CNT, DAC_DMA_CHANNEL_CNT, 0u )

/** Extract DAC peripheral ID from encoded DMA stream value */
#define DAC_DMA_BIT_MASK_DECODE_PERIPH( CODED_VAL )  ( ( (CODED_VAL) >> DAC_DMA_BIT_MASK_PERIPH_BIT_OFFSET ) & DAC_DMA_BIT_MASK_FIELD )

/** Extract DAC channel ID from encoded DMA stream value */
#define DAC_DMA_BIT_MASK_DECODE_CHANNEL( CODED_VAL ) ( ( (CODED_VAL) >> DAC_DMA_BIT_MASK_CHANNEL_BIT_OFFSET ) & DAC_DMA_BIT_MASK_FIELD )

/** Extract DMA peripheral ID from encoded DMA stream value */
#define DAC_DMA_BIT_MASK_DECODE_DMA( CODED_VAL )     ( ( (CODED_VAL) >> DAC_DMA_BIT_MASK_DMA_BIT_OFFSET ) & DAC_DMA_BIT_MASK_FIELD )

/** Extract stream ID from encoded DMA stream value */
#define DAC_DMA_BIT_MASK_DECODE_STREAM( CODED_VAL )  ( ( (CODED_VAL) >> DAC_DMA_BIT_MASK_STREAM_BIT_OFFSET ) & DAC_DMA_BIT_MASK_FIELD )

/** Extract channel selection (CHSEL) from encoded DMA stream value */
#define DAC_DMA_BIT_MASK_DECODE_CHSEL( CODED_VAL )   ( ( (CODED_VAL) >> DAC_DMA_BIT_MASK_CHSEL_BIT_OFFSET ) & DAC_DMA_BIT_MASK_FIELD )

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
 * \note STM32F7 DAC has no kernel clock multiplexer - it is clocked by the APB1 bus clock. */
typedef enum
{
    DAC_CLK_SRC_PCLK1 = 0u, /**< APB1 bus clock (PCLK1) as clock source of DAC */
    DAC_CLK_SRC_CNT         /**< Count of DAC clock sources                    */
}   dac_ClkSrc_t;


/** \brief List of sample and hold clock sources
 *
 * \note Sample and hold is not supported by STM32F7 - the selection is always refused. */
typedef enum
{
    DAC_SH_CLK_SRC_LSI = 0u, /**< Low Speed Internal oscillator (LSI) - not supported */
    DAC_SH_CLK_SRC_LSE,      /**< Low Speed External oscillator (LSE) - not supported */
    DAC_SH_CLK_SRC_CNT       /**< Count of sample and hold clock sources              */
}   dac_ShClkSrc_t;

/* -------------------------------------------------------------------------- */
/* ------------------------ Peripheral configuration ------------------------ */
/* -------------------------------------------------------------------------- */

/** \brief Digital-to-Analog Converter (DAC) peripheral enumeration */
typedef enum
{
    DAC_PERIPH_1 = 0u, /**< Digital-to-Analog Converter (DAC) peripheral 1            */
    DAC_PERIPH_CNT     /**< Count of Digital-to-Analog Converter (DAC) peripherals    */
}   dac_PeriphId_t;


/** \brief Digital-to-Analog Converter (DAC) channel enumeration */
typedef enum
{
    DAC_CHANNEL_1 = 0u, /**< DAC channel 1 (DAC_OUT1, PA4) */
#if defined(DAC_CR_EN2)
    DAC_CHANNEL_2,      /**< DAC channel 2 (DAC_OUT2, PA5) */
#endif /* DAC_CR_EN2 */
    DAC_CHANNEL_CNT     /**< Count of DAC channels          */
}   dac_ChannelId_t;


/** \brief List of channel output modes
 *
 * \note STM32F7 DAC output is always connected to the pin - internal connection modes are
 *       not supported (configuration request returns error).
 */
typedef enum
{
    DAC_OUTPUT_PIN_BUFFERED = 0u,      /**< Output connected to external pin, output buffer enabled                  */
    DAC_OUTPUT_PIN_INTERNAL_BUFFERED,  /**< Output connected to pin and on-chip peripherals, buffered - not supported */
    DAC_OUTPUT_PIN_UNBUFFERED,         /**< Output connected to external pin, output buffer disabled                 */
    DAC_OUTPUT_INTERNAL,               /**< Output connected to on-chip peripherals only - not supported             */
    DAC_OUTPUT_CNT                     /**< Count of output modes                                                    */
}   dac_OutputMode_t;


/** \brief List of available conversion trigger sources
 *
 * \note DAC_TRIGGER_NONE - trigger is disabled (TENx = 0), the output is updated one APB clock
 *       cycle after the data holding register write.
 * \note DAC has no trigger edge selection - conversion is started on the rising edge of the
 *       selected trigger signal.
 */
typedef enum
{
    DAC_TRIGGER_NONE = 0u,         /**< No trigger, output updated after data write */
    DAC_TRIGGER_SOFTWARE,          /**< Software trigger (Dac_Set_SwTrigger())      */
#if defined (TIM2)
    DAC_TRIGGER_TIM2_TRGO,         /**< External trigger TIM2 TRGO                  */
#endif /* TIM2 */
#if defined (TIM4)
    DAC_TRIGGER_TIM4_TRGO,         /**< External trigger TIM4 TRGO                  */
#endif /* TIM4 */
#if defined (TIM5)
    DAC_TRIGGER_TIM5_TRGO,         /**< External trigger TIM5 TRGO                  */
#endif /* TIM5 */
#if defined (TIM6)
    DAC_TRIGGER_TIM6_TRGO,         /**< External trigger TIM6 TRGO                  */
#endif /* TIM6 */
#if defined (TIM7)
    DAC_TRIGGER_TIM7_TRGO,         /**< External trigger TIM7 TRGO                  */
#endif /* TIM7 */
#if defined (TIM8)
    DAC_TRIGGER_TIM8_TRGO,         /**< External trigger TIM8 TRGO                  */
#endif /* TIM8 */
    DAC_TRIGGER_EXTI_LINE9,        /**< External trigger EXTI line 9                */
    DAC_TRIGGER_CNT                /**< Count of trigger sources                    */
}   dac_TriggerId_t;


/** \brief List of data formats (selects the data holding register used for the output value) */
typedef enum
{
    DAC_DATA_FORMAT_12BIT_RIGHT = 0u, /**< 12-bit right aligned data (0 - 4095)                              */
    DAC_DATA_FORMAT_12BIT_LEFT,       /**< 12-bit left aligned data (bits 15:4 used, bits 3:0 are ignored)   */
    DAC_DATA_FORMAT_8BIT_RIGHT,       /**< 8-bit right aligned data (0 - 255)                                */
    DAC_DATA_FORMAT_CNT               /**< Count of data formats                                             */
}   dac_DataFormat_t;


/** \brief List of wave auto generation modes */
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


/** \brief Type representing offset trimming value (offset calibration is not supported by STM32F7) */
typedef uint8_t dac_TrimValue_t;


/** \brief Type representing sample and hold sample time (sample and hold is not supported by STM32F7) */
typedef uint16_t dac_ShSampleTime_t;


/** \brief Type representing sample and hold hold time (sample and hold is not supported by STM32F7) */
typedef uint16_t dac_ShHoldTime_t;


/** \brief Type representing sample and hold refresh time (sample and hold is not supported by STM32F7) */
typedef uint8_t dac_ShRefreshTime_t;


/** \brief List of DAC flags
 *
 * \note STM32F7 DAC status register contains only the DMA underrun flags - requests for other
 *       flags return error.
 */
typedef enum
{
    DAC_FLAG_DMA_UNDERRUN = 0u, /**< DMA underrun (DMAUDRx), cleared by Dac_Clear_Flag() */
    DAC_FLAG_CALIBRATION,       /**< Offset calibration status - not supported           */
    DAC_FLAG_SH_WRITE_BUSY,     /**< Sample time write ongoing - not supported           */
    DAC_FLAG_READY,             /**< Channel ready - not supported                       */
    DAC_FLAG_CNT                /**< Count of DAC flags                                  */
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
 * \note STM32F7 reports DAC_ERROR_DMA_UNDERRUN and DAC_ERROR_DMA_TRANSFER only, the other
 *       errors belong to DMA controllers of other families.
 */
typedef enum
{
    DAC_ERROR_DMA_UNDERRUN = 0u,   /**< DAC DMA underrun - trigger came before the previous DMA request was served */
    DAC_ERROR_DMA_TRANSFER,        /**< DMA transfer error (bus error during transfer)                           */
    DAC_ERROR_DMA_CONFIG,          /**< DMA configuration error - not reported by STM32F7                        */
    DAC_ERROR_DMA_CONFIG_UPDATE,   /**< DMA configuration update error - not reported by STM32F7                 */
    DAC_ERROR_DMA_TRIGGER_OVERRUN, /**< DMA trigger overrun - not reported by STM32F7                            */
    DAC_ERROR_CNT                  /**< Count of data transfer errors                                            */
}   dac_ErrorId_t;


/** \brief Encoded DMA stream (value of \ref dac_Dma_t) */
typedef uint32_t dac_DmaCode_t;


/** DMA peripherals enumeration list
 *
 * \note DAC DMA requests are connected to DMA1 only (fixed request mapping).
 */
typedef enum
{
    DAC_DMA_PERIPH_1 = DMA_PERIPH_1,   /**< DMA peripheral 1 identification */
#if defined(DMA2)
    DAC_DMA_PERIPH_2 = DMA_PERIPH_2,   /**< DMA peripheral 2 identification */
#endif
    DAC_DMA_PERIPH_CNT                 /**< Count of DMA peripherals        */
}   dac_DmaPeriphId_t;


/** Enumeration of DMA transfer channels (DMA streams of STM32F7)
 *
 * \note DAC DMA requests are connected to fixed streams: DAC channel 1 to DMA1 stream 5,
 *       DAC channel 2 to DMA1 stream 6 (channel selection 7), see the list \ref dac_Dma_t.
 */
typedef enum
{
    DAC_DMA_CHANNEL_0 = DMA_STREAM_0, /**< DMA stream 0 */
    DAC_DMA_CHANNEL_1 = DMA_STREAM_1, /**< DMA stream 1 */
    DAC_DMA_CHANNEL_2 = DMA_STREAM_2, /**< DMA stream 2 */
    DAC_DMA_CHANNEL_3 = DMA_STREAM_3, /**< DMA stream 3 */
    DAC_DMA_CHANNEL_4 = DMA_STREAM_4, /**< DMA stream 4 */
    DAC_DMA_CHANNEL_5 = DMA_STREAM_5, /**< DMA stream 5 */
    DAC_DMA_CHANNEL_6 = DMA_STREAM_6, /**< DMA stream 6 */
    DAC_DMA_CHANNEL_7 = DMA_STREAM_7, /**< DMA stream 7 */
    DAC_DMA_CHANNEL_CNT               /**< Count of DMA streams */
}   dac_DmaChannelId_t;


/**
 * \brief List of DMA streams able to serve the DAC channel of the peripherals (STM32CubeMX database / reference
 *        manual DMA request mapping, the channel selection of the stream is part of the value, streams
 *        existing only on some STM32F7 lines are guarded by the CMSIS device line)
 */
typedef enum
{
    DAC_DMA_DAC1_CH1_DMA1_STREAM5      = DAC_DMA_ENCODE( DAC_PERIPH_1, DAC_CHANNEL_1, DAC_DMA_PERIPH_1, DAC_DMA_CHANNEL_5, 7u ), /**< DAC channel 1 request on DMA1 stream 5 (channel selection 7) */
    DAC_DMA_DAC1_CH2_DMA1_STREAM6      = DAC_DMA_ENCODE( DAC_PERIPH_1, DAC_CHANNEL_2, DAC_DMA_PERIPH_1, DAC_DMA_CHANNEL_6, 7u ), /**< DAC channel 2 request on DMA1 stream 6 (channel selection 7) */
    DAC_DMA_UNUSED                     = DAC_DMA_CODE_UNUSED  /**< DMA stream is not selected */
}   dac_Dma_t;


/** DMA channel priority options enumeration */
typedef enum
{
    DAC_DMA_PRIORITY_LOW      = DMA_PRIORITY_LOW     , /**< Priority level : Low       */
    DAC_DMA_PRIORITY_MEDIUM   = DMA_PRIORITY_MEDIUM  , /**< Priority level : Medium    */
    DAC_DMA_PRIORITY_HIGH     = DMA_PRIORITY_HIGH    , /**< Priority level : High      */
    DAC_DMA_PRIORITY_VERYHIGH = DMA_PRIORITY_VERYHIGH, /**< Priority level : Very high */
}   dac_DmaPriority_t;


/** \brief Type representing interrupt priority of the DAC (DMA underrun) interrupt */
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
 * DAC_TRANSFER_MODE_DMA.
 *
 * \note STM32F7: Dma has to be the item of the list \ref dac_Dma_t of the channel (the fixed DMA
 *       request mapping, filled by Dac_Get_DefaultConfig()). DataBuffer must not be placed in
 *       CCM data RAM (not accessible by DMA).
 */
typedef struct
{
    dac_TransferMode_t  TransferMode;             /**< Data transfer mode (DIRECT / DMA)                       */
    const dac_Data_t   *DataBuffer;               /**< Buffer of output values (DMA mode, must not be NULL)    */
    dac_BufferSize_t    BufferSize;               /**< Buffer size in count of dac_Data_t items (> 0)          */
    dac_BufferMode_t    BufferMode;               /**< One shot / circular buffer handling                     */

    dac_Dma_t           Dma;                      /**< DMA stream of the channel request                       */
    dac_DmaPriority_t   DmaPriority;              /**< DMA channel priority                                    */

    dac_IrqPrio_t       IrqPriority;              /**< DAC interrupt priority (DMA underrun)                   */

    dac_Callback_t     *HalfTransferCallback;     /**< Half of the buffer output. DAC_NULL_PTR if not used.    */
    dac_Callback_t     *TransferCompleteCallback; /**< Whole buffer output. DAC_NULL_PTR if not used.          */
    dac_ErrCallback_t  *ErrorCallback;            /**< Transfer error. DAC_NULL_PTR if not used.               */
}   dac_DataConfig_t;


/** \brief Sample and hold configuration of a channel
 *
 * \note Sample and hold is not supported by STM32F7 - only State == DAC_FUNCTION_INACTIVE is accepted.
 */
typedef struct
{
    dac_FunctionState_t State;       /**< Sample and hold mode active / inactive  */
    dac_ShSampleTime_t  SampleTime;  /**< Sample phase duration                   */
    dac_ShHoldTime_t    HoldTime;    /**< Hold phase duration                     */
    dac_ShRefreshTime_t RefreshTime; /**< Refresh phase duration                  */
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
    dac_SampleHoldConfig_t SampleHold;    /**< Sample and hold configuration (must be inactive)           */
    dac_Data_t             InitValue;     /**< Output value written on initialization (in DataFormat)     */
    dac_DataConfig_t       DataConfig;    /**< Data transfer configuration                                */
}   dac_ChannelConfig_t;


/** \brief Peripheral configuration structure */
typedef struct
{
    dac_PeriphId_t      PeriphId;                     /**< DAC peripheral identification                                     */
    dac_ShClkSrc_t      ShClockSource;                /**< Sample and hold clock (not used by STM32F7, value is checked only) */
    dac_ChannelConfig_t Channels[ DAC_CHANNEL_CNT ];  /**< Channel configuration, indexed by \ref dac_ChannelId_t            */
}   dac_PeriphConfig_t;


/** \brief Module configuration representing top layer configuration structure */
typedef struct
{
    dac_ClkSrc_t       ClockSource;                    /**< DAC clock source                                         */
    dac_PeriphConfig_t PeriphConfig[ DAC_PERIPH_CNT ]; /**< Peripheral configuration, indexed by \ref dac_PeriphId_t */
}   dac_Config_t;


#endif /* DAC_DAC_TYPES_H */

