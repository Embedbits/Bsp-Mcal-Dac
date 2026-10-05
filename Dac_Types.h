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
#include "Gpdma_Types.h"                    /* DMA types definitions          */
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

/** \brief List of available shared kernel clock sources
 *
 * \note The kernel clock source is shared by ADC and DAC peripherals (ADCDACSEL). */
typedef enum
{
    DAC_CLK_SRC_HCLK = 0u, /**< HCLK as clock source for ADC's and DAC                                      */
    DAC_CLK_SRC_SYSCLK,    /**< SysClk as clock source for ADC's and DAC                                    */
    DAC_CLK_SRC_PLL2R,     /**< PLL2 output R as clock source for ADC's and DAC                             */
    DAC_CLK_SRC_HSE,       /**< High Speed External oscillator (HSE) as clock source for ADC's and DAC       */
    DAC_CLK_SRC_HSI,       /**< 64MHz High Speed Internal oscillator (HSI) as clock source for ADC's and DAC */
    DAC_CLK_SRC_CSI,       /**< 4MHz Low Power Internal oscillator (CSI) as clock source for ADC's and DAC   */
    DAC_CLK_SRC_CNT        /**< Count of DAC clock sources                                                   */
}   dac_ClkSrc_t;


/** \brief List of available sample and hold clock sources (DACSEL, common for all channels) */
typedef enum
{
    DAC_SH_CLK_SRC_LSI = 0u, /**< Low Speed Internal oscillator (LSI) as sample and hold clock */
    DAC_SH_CLK_SRC_LSE,      /**< Low Speed External oscillator (LSE) as sample and hold clock */
    DAC_SH_CLK_SRC_CNT       /**< Count of sample and hold clock sources                       */
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
    DAC_CHANNEL_1 = 0u, /**< DAC channel 1 (DAC1_OUT1, PA4) */
    DAC_CHANNEL_2,      /**< DAC channel 2 (DAC1_OUT2, PA5) */
    DAC_CHANNEL_CNT     /**< Count of DAC channels          */
}   dac_ChannelId_t;


/** \brief List of channel output modes (buffer and output connection, MODEx[1:0]) */
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
 * \note DAC_TRIGGER_NONE - trigger is disabled (TENx = 0), the output is updated one APB clock
 *       cycle after the data holding register write.
 * \note DAC has no trigger edge selection - conversion is started on the rising edge of the
 *       selected trigger signal.
 */
typedef enum
{
    DAC_TRIGGER_NONE = 0u,         /**< No trigger, output updated after data write */
    DAC_TRIGGER_SOFTWARE,          /**< Software trigger (Dac_Set_SwTrigger())      */
    DAC_TRIGGER_TIM1_TRGO,         /**< External trigger TIM1 TRGO                  */
    DAC_TRIGGER_TIM2_TRGO,         /**< External trigger TIM2 TRGO                  */
#if defined (TIM8)
    DAC_TRIGGER_TIM4_TRGO,         /**< External trigger TIM4 TRGO                  */
    DAC_TRIGGER_TIM5_TRGO,         /**< External trigger TIM5 TRGO                  */
#else
    DAC_TRIGGER_TIM3_TRGO,         /**< External trigger TIM3 TRGO                  */
#endif /* TIM8 */
    DAC_TRIGGER_TIM6_TRGO,         /**< External trigger TIM6 TRGO                  */
    DAC_TRIGGER_TIM7_TRGO,         /**< External trigger TIM7 TRGO                  */
#if defined (TIM8)
    DAC_TRIGGER_TIM8_TRGO,         /**< External trigger TIM8 TRGO                  */
    DAC_TRIGGER_TIM15_TRGO,        /**< External trigger TIM15 TRGO                 */
#endif /* TIM8 */
    DAC_TRIGGER_LPTIM1_CH1,        /**< External trigger LPTIM1 channel 1           */
    DAC_TRIGGER_LPTIM2_CH1,        /**< External trigger LPTIM2 channel 1           */
    DAC_TRIGGER_EXTI_LINE9,        /**< External trigger EXTI line 9                */
#if defined (PLAY1)
    DAC_TRIGGER_PLAY1_OUT11,       /**< External trigger PLAY1 output 11            */
    DAC_TRIGGER_PLAY1_OUT12,       /**< External trigger PLAY1 output 12            */
#endif /* PLAY1 */
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


/** \brief List of data transfer errors reported through \ref dac_ErrCallback_t */
typedef enum
{
    DAC_ERROR_DMA_UNDERRUN = 0u,   /**< DAC DMA underrun - trigger came before the previous DMA request was served */
    DAC_ERROR_DMA_TRANSFER,        /**< DMA transfer error (bus error during transfer)                           */
    DAC_ERROR_DMA_CONFIG,          /**< DMA configuration error                                                  */
    DAC_ERROR_DMA_CONFIG_UPDATE,   /**< DMA configuration (linked list) update error                             */
    DAC_ERROR_DMA_TRIGGER_OVERRUN, /**< DMA trigger overrun                                                      */
    DAC_ERROR_CNT                  /**< Count of data transfer errors                                            */
}   dac_ErrorId_t;


/** DMA peripherals enumeration list */
typedef enum
{
#if defined(GPDMA1)
    DAC_DMA_PERIPH_1 = GPDMA_PERIPH_1, /**< DMA peripheral 1 identification */
#endif
#if defined(GPDMA2)
    DAC_DMA_PERIPH_2 = GPDMA_PERIPH_2, /**< DMA peripheral 2 identification */
#endif
    DAC_DMA_PERIPH_CNT                 /**< Count of DMA peripherals        */
}   dac_DmaPeriphId_t;


/** Enumeration of available channels for all DMA peripherals */
typedef enum
{
    DAC_DMA_CHANNEL_0 = GPDMA_CHANNEL_0, /**< DMA transfer channel 0 */
    DAC_DMA_CHANNEL_1 = GPDMA_CHANNEL_1, /**< DMA transfer channel 1 */
    DAC_DMA_CHANNEL_2 = GPDMA_CHANNEL_2, /**< DMA transfer channel 2 */
    DAC_DMA_CHANNEL_3 = GPDMA_CHANNEL_3, /**< DMA transfer channel 3 */
    DAC_DMA_CHANNEL_4 = GPDMA_CHANNEL_4, /**< DMA transfer channel 4 */
    DAC_DMA_CHANNEL_5 = GPDMA_CHANNEL_5, /**< DMA transfer channel 5 */
    DAC_DMA_CHANNEL_6 = GPDMA_CHANNEL_6, /**< DMA transfer channel 6 */
    DAC_DMA_CHANNEL_7 = GPDMA_CHANNEL_7, /**< DMA transfer channel 7 */
    DAC_DMA_CHANNEL_CNT                  /**< Count of DMA channels  */
}   dac_DmaChannelId_t;


/** DMA channel priority options enumeration */
typedef enum
{
    DAC_DMA_PRIORITY_LOW      = GPDMA_PRIORITY_LOW     , /**< Priority level : Low       */
    DAC_DMA_PRIORITY_MEDIUM   = GPDMA_PRIORITY_MEDIUM  , /**< Priority level : Medium    */
    DAC_DMA_PRIORITY_HIGH     = GPDMA_PRIORITY_HIGH    , /**< Priority level : High      */
    DAC_DMA_PRIORITY_VERYHIGH = GPDMA_PRIORITY_VERYHIGH, /**< Priority level : Very high */
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
    dac_ClkSrc_t       ClockSource;                    /**< ADC / DAC kernel clock source (shared with ADC module) */
    dac_PeriphConfig_t PeriphConfig[ DAC_PERIPH_CNT ]; /**< Peripheral configuration, indexed by \ref dac_PeriphId_t */
}   dac_Config_t;


#endif /* DAC_DAC_TYPES_H */

