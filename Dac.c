/**
 * \author Mr.Nobody
 * \file Dac.c
 * \ingroup Dac
 * \brief Dac module common functionality
 *
 * STM32H7 DAC peripherals: DAC1 (2 channels, PA4 / PA5, APB1 clock) on every device and DAC2
 * (1 channel, PA6, APB4 clock, SmartRun domain) on STM32H7A3 / H7B0 / H7B3. No kernel clock
 * multiplexer, sample and hold is clocked by LSI. DAC2 DMA request is served by BDMA2 / DMAMUX2
 * (not supported by Dma module) - DMA transfer is refused for DAC2. No high frequency interface
 * mode, no channel ready flag, no sawtooth wave and no signed data format on STM32H7.
 *
 * \note  The module is empty on MCUs without DAC (STM32H7R / STM32H7S).
 */
/* ============================== INCLUDES ================================== */
#include "Dac.h"                            /* Self include                   */
#include "Dac_Port.h"                       /* Own port file include          */
#include "Dac_Types.h"                      /* Module types definitions       */
#include "Dac_Dma.h"                        /* DMA data transfer handler      */
#include "Dac_Isr.h"                        /* DAC interrupt handler          */
#include "Rcc_Port.h"                       /* RCC Mcal layer include         */
#include "Gpio_Port.h"                      /* GPIO Mcal layer include        */
#include "Stm32.h"                          /* MCU common functionality (DAC1 instance pointer) */

#if defined(DAC1)
/* ============================== TYPEDEFS ================================== */

/** GPIO pin of a DAC channel output */
typedef struct
{
    gpio_PortId_t PortId; /**< GPIO port of the channel output */
    gpio_PinId_t  PinId;  /**< GPIO pin of the channel output  */
}   dac_GpioConfig_t;


/** DAC peripheral configuration */
typedef struct
{
    DAC_TypeDef         *PeriphReg;   /**< CMSIS DAC instance register pointer                      */
    rcc_PeriphId_t       PeriphRcc;   /**< RCC identification (APB clock enable, peripheral reset)  */
    dac_ChannelId_t      ChannelCnt;  /**< Count of channels of the peripheral                      */
    dac_FunctionState_t  ExternalOut; /**< Buffered outputs to pins (ACTIVE) or internal outputs only */
}   dac_PeriphConfigStruct_t;


/** Default DMA channel of a DAC channel (distinct for every DAC channel) */
typedef struct
{
    dac_DmaPeriphId_t  DmaPeriphId;  /**< DMA peripheral */
    dac_DmaChannelId_t DmaChannelId; /**< DMA channel    */
}   dac_DmaDefaultConfig_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static dac_RequestState_t Dac_Check_ChannelDisabled     ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );
static dac_RequestState_t Dac_Check_PeriphDisabled      ( dac_PeriphId_t periphId );
static dac_RequestState_t Dac_Check_AllPeriphsDisabled  ( void );
static dac_RequestState_t Dac_Check_ChannelConfig       ( dac_PeriphId_t periphId, const dac_ChannelConfig_t * const channelConfig );
static dac_RequestState_t Dac_Check_SampleHoldConfig    ( const dac_SampleHoldConfig_t * const shConfig );
static dac_RequestState_t Dac_Check_DataValue           ( dac_DataFormat_t dataFormat, dac_Data_t data );
static dac_RequestState_t Dac_Check_DmaTrigger          ( dac_TriggerId_t triggerSrc );
static dac_RequestState_t Dac_Check_OutputMode          ( dac_PeriphId_t periphId, dac_OutputMode_t outputMode );
static dac_RequestState_t Dac_Check_DataConfig          ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TriggerId_t triggerSrc, const dac_DataConfig_t * const dataConfig );
static dac_RequestState_t Dac_Set_PinAnalog             ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );
static dac_RequestState_t Dac_Write_TrimValue           ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TrimValue_t trimValue );
static dac_RequestState_t Dac_Write_CalibrationMode     ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, uint32_t llMode );
static dac_RequestState_t Dac_Write_ChannelMode         ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, uint32_t llBuffer, uint32_t llConnect, uint32_t llShMode );
static dac_RequestState_t Dac_Get_CalibrationFlag       ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_FlagState_t * const flagState );
static dac_RequestState_t Dac_Get_DualDataReg           ( dac_PeriphId_t periphId, dac_DataFormat_t dataFormat, volatile uint32_t ** const dataReg );
static dac_RequestState_t Dac_Set_Delay                 ( dac_TimeUs_t delayUs );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define DAC_MAJOR_VERSION           ( 1u )

/** Value of minor version of SW module */
#define DAC_MINOR_VERSION           ( 0u )

/** Value of patch version of SW module */
#define DAC_PATCH_VERSION           ( 0u )


/** Delay between two offset calibration steps in us (tOFFTRIM, same value as used by ST HAL) */
#define DAC_CAL_STEP_DELAY_US        ( 1000u )

/** Initial trimming value of the calibration binary search (middle of the OTRIM range) */
#define DAC_CAL_TRIM_START           ( 16u )

/** Initial step of the calibration binary search */
#define DAC_CAL_TRIM_STEP            ( 8u )


/** Count of microseconds in one second */
#define DAC_US_PER_S                 ( 1000000u )

/** Divider used for rounding of integer division to the nearest value */
#define DAC_ROUND_DIVIDER            ( 2u )

/** Full scale of the output data register (DORx is always 12-bit right aligned) */
#define DAC_OUTPUT_FULL_SCALE        ( (uint32_t)DAC_DATA_12BIT_MAX )

/** LUT value signaling a trigger not connected to the DAC peripheral */
#define DAC_TRIG_UNSUPPORTED         ( 0xFFFFFFFFu )

/** LUT value signaling a flag not available on STM32H7 (no SR bit) */
#define DAC_FLAG_UNSUPPORTED         ( 0u )

/* =============================== MACROS =================================== */

/** Integer division rounded up (e.g. CPU cycles per microsecond, the delay is never shorter) */
#define DAC_DIV_ROUND_UP( dividend, divisor )    ( ( (dividend) + (divisor) - 1u ) / (divisor) )

/** Integer division rounded to the nearest value */
#define DAC_DIV_ROUND( dividend, divisor )       ( ( (dividend) + ( (divisor) / DAC_ROUND_DIVIDER ) ) / (divisor) )

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** CMSIS instance lookup, indexed by dac_PeriphId_t */
static const dac_PeriphConfigStruct_t dac_PeriphConf[ ] =
{
    { .PeriphReg = DAC1, .PeriphRcc = RCC_PERIPH_DAC12, .ChannelCnt = DAC_CHANNEL_CNT, .ExternalOut = DAC_FUNCTION_ACTIVE },
#if defined(DAC2)
    { .PeriphReg = DAC2, .PeriphRcc = RCC_PERIPH_DAC2,  .ChannelCnt = DAC_CHANNEL_2,   .ExternalOut = DAC_FUNCTION_ACTIVE },
#endif /* DAC2 */
};

_Static_assert( DAC_PERIPH_CNT == ( sizeof(dac_PeriphConf) / sizeof(dac_PeriphConfigStruct_t) ), "Dac: dac_PeriphConf has incorrect size." );


/** dac_PeriphId_t x dac_ChannelId_t -> output pin (peripherals with external outputs only) */
static const dac_GpioConfig_t dac_GpioConfig[ DAC_PERIPH_CNT ][ DAC_CHANNEL_CNT ] =
{
    [DAC_PERIPH_1] =
    {
        [DAC_CHANNEL_1] = { .PortId = GPIO_PORT_A, .PinId = GPIO_PIN_ID_4 },
        [DAC_CHANNEL_2] = { .PortId = GPIO_PORT_A, .PinId = GPIO_PIN_ID_5 },
    },
#if defined(DAC2)
    [DAC_PERIPH_2] =
    {
        [DAC_CHANNEL_1] = { .PortId = GPIO_PORT_A, .PinId = GPIO_PIN_ID_6 },
    },
#endif /* DAC2 */
};


/** dac_PeriphId_t x dac_ChannelId_t -> default DMA stream (DMA1 streams 5 / 6 as STM32F4 / F7 DAC,
 *  DAC2 has no DMA transfer - stream 7 is reported only) */
static const dac_DmaDefaultConfig_t dac_DmaDefaultConfig[ DAC_PERIPH_CNT ][ DAC_CHANNEL_CNT ] =
{
    [DAC_PERIPH_1] =
    {
        [DAC_CHANNEL_1] = { .DmaPeriphId = DAC_DMA_PERIPH_1, .DmaChannelId = DAC_DMA_CHANNEL_5 },
        [DAC_CHANNEL_2] = { .DmaPeriphId = DAC_DMA_PERIPH_1, .DmaChannelId = DAC_DMA_CHANNEL_6 },
    },
#if defined(DAC2)
    [DAC_PERIPH_2] =
    {
        [DAC_CHANNEL_1] = { .DmaPeriphId = DAC_DMA_PERIPH_1, .DmaChannelId = DAC_DMA_CHANNEL_7 },
        [DAC_CHANNEL_2] = { .DmaPeriphId = DAC_DMA_PERIPH_1, .DmaChannelId = DAC_DMA_CHANNEL_7 },
    },
#endif /* DAC2 */
};


/** dac_ChannelId_t -> LL_DAC_CHANNEL_x */
static const uint32_t dac_LlChannelLut[ DAC_CHANNEL_CNT ] =
{
    [DAC_CHANNEL_1] = LL_DAC_CHANNEL_1,
    [DAC_CHANNEL_2] = LL_DAC_CHANNEL_2,
};


/** dac_PeriphId_t x dac_TriggerId_t -> LL_DAC_TRIG_* (STM32H7 DAC interconnection, TSELx - same
 *  trigger set for every DAC peripheral of the device). DAC_TRIGGER_NONE has no own source
 *  (TENx = 0). DAC_TRIG_UNSUPPORTED - trigger is not connected to the peripheral (not used on
 *  STM32H7, kept for the common checks). */
static const uint32_t dac_TriggerSrcLut[ DAC_PERIPH_CNT ][ DAC_TRIGGER_CNT ] =
{
    [DAC_PERIPH_1] =
    {
        [DAC_TRIGGER_NONE]           = LL_DAC_TRIG_SOFTWARE,
        [DAC_TRIGGER_SOFTWARE]       = LL_DAC_TRIG_SOFTWARE,
        [DAC_TRIGGER_TIM1_TRGO]      = LL_DAC_TRIG_EXT_TIM1_TRGO,
        [DAC_TRIGGER_TIM2_TRGO]      = LL_DAC_TRIG_EXT_TIM2_TRGO,
        [DAC_TRIGGER_TIM4_TRGO]      = LL_DAC_TRIG_EXT_TIM4_TRGO,
        [DAC_TRIGGER_TIM5_TRGO]      = LL_DAC_TRIG_EXT_TIM5_TRGO,
        [DAC_TRIGGER_TIM6_TRGO]      = LL_DAC_TRIG_EXT_TIM6_TRGO,
        [DAC_TRIGGER_TIM7_TRGO]      = LL_DAC_TRIG_EXT_TIM7_TRGO,
        [DAC_TRIGGER_TIM8_TRGO]      = LL_DAC_TRIG_EXT_TIM8_TRGO,
        [DAC_TRIGGER_TIM15_TRGO]     = LL_DAC_TRIG_EXT_TIM15_TRGO,
#if defined(HRTIM1)
        [DAC_TRIGGER_HRTIM_DAC_TRG1] = LL_DAC_TRIG_EXT_HRTIM_TRGO1,
        [DAC_TRIGGER_HRTIM_DAC_TRG2] = LL_DAC_TRIG_EXT_HRTIM_TRGO2,
#endif /* HRTIM1 */
        [DAC_TRIGGER_LPTIM1_OUT]     = LL_DAC_TRIG_EXT_LPTIM1_OUT,
        [DAC_TRIGGER_LPTIM2_OUT]     = LL_DAC_TRIG_EXT_LPTIM2_OUT,
        [DAC_TRIGGER_EXTI_LINE9]     = LL_DAC_TRIG_EXT_EXTI_LINE9,
#if defined(TIM23)
        [DAC_TRIGGER_TIM23_TRGO]     = LL_DAC_TRIG_EXT_TIM23_TRGO,
#endif /* TIM23 */
#if defined(TIM24)
        [DAC_TRIGGER_TIM24_TRGO]     = LL_DAC_TRIG_EXT_TIM24_TRGO,
#endif /* TIM24 */
#if defined(DAC2)
        [DAC_TRIGGER_LPTIM3_OUT]     = LL_DAC_TRIG_EXT_LPTIM3_OUT,
#endif /* DAC2 */
    },
#if defined(DAC2)
    [DAC_PERIPH_2] =
    {
        [DAC_TRIGGER_NONE]           = LL_DAC_TRIG_SOFTWARE,
        [DAC_TRIGGER_SOFTWARE]       = LL_DAC_TRIG_SOFTWARE,
        [DAC_TRIGGER_TIM1_TRGO]      = LL_DAC_TRIG_EXT_TIM1_TRGO,
        [DAC_TRIGGER_TIM2_TRGO]      = LL_DAC_TRIG_EXT_TIM2_TRGO,
        [DAC_TRIGGER_TIM4_TRGO]      = LL_DAC_TRIG_EXT_TIM4_TRGO,
        [DAC_TRIGGER_TIM5_TRGO]      = LL_DAC_TRIG_EXT_TIM5_TRGO,
        [DAC_TRIGGER_TIM6_TRGO]      = LL_DAC_TRIG_EXT_TIM6_TRGO,
        [DAC_TRIGGER_TIM7_TRGO]      = LL_DAC_TRIG_EXT_TIM7_TRGO,
        [DAC_TRIGGER_TIM8_TRGO]      = LL_DAC_TRIG_EXT_TIM8_TRGO,
        [DAC_TRIGGER_TIM15_TRGO]     = LL_DAC_TRIG_EXT_TIM15_TRGO,
#if defined(HRTIM1)
        [DAC_TRIGGER_HRTIM_DAC_TRG1] = LL_DAC_TRIG_EXT_HRTIM_TRGO1,
        [DAC_TRIGGER_HRTIM_DAC_TRG2] = LL_DAC_TRIG_EXT_HRTIM_TRGO2,
#endif /* HRTIM1 */
        [DAC_TRIGGER_LPTIM1_OUT]     = LL_DAC_TRIG_EXT_LPTIM1_OUT,
        [DAC_TRIGGER_LPTIM2_OUT]     = LL_DAC_TRIG_EXT_LPTIM2_OUT,
        [DAC_TRIGGER_EXTI_LINE9]     = LL_DAC_TRIG_EXT_EXTI_LINE9,
#if defined(TIM23)
        [DAC_TRIGGER_TIM23_TRGO]     = LL_DAC_TRIG_EXT_TIM23_TRGO,
#endif /* TIM23 */
#if defined(TIM24)
        [DAC_TRIGGER_TIM24_TRGO]     = LL_DAC_TRIG_EXT_TIM24_TRGO,
#endif /* TIM24 */
#if defined(DAC2)
        [DAC_TRIGGER_LPTIM3_OUT]     = LL_DAC_TRIG_EXT_LPTIM3_OUT,
#endif /* DAC2 */
    },
#endif /* DAC2 */
};


/** dac_OutputMode_t -> LL_DAC_OUTPUT_BUFFER_x */
static const uint32_t dac_OutputBufferLut[ DAC_OUTPUT_CNT ] =
{
    [DAC_OUTPUT_PIN_BUFFERED]          = LL_DAC_OUTPUT_BUFFER_ENABLE,
    [DAC_OUTPUT_PIN_INTERNAL_BUFFERED] = LL_DAC_OUTPUT_BUFFER_ENABLE,
    [DAC_OUTPUT_PIN_UNBUFFERED]        = LL_DAC_OUTPUT_BUFFER_DISABLE,
    [DAC_OUTPUT_INTERNAL]              = LL_DAC_OUTPUT_BUFFER_DISABLE,
};


/** dac_OutputMode_t -> LL_DAC_OUTPUT_CONNECT_x. With buffer enabled the INTERNAL connection
 *  keeps the pin connected as well (MODEx = 001), with buffer disabled the GPIO connection
 *  is the pin only mode (MODEx = 010). */
static const uint32_t dac_OutputConnectLut[ DAC_OUTPUT_CNT ] =
{
    [DAC_OUTPUT_PIN_BUFFERED]          = LL_DAC_OUTPUT_CONNECT_GPIO,
    [DAC_OUTPUT_PIN_INTERNAL_BUFFERED] = LL_DAC_OUTPUT_CONNECT_INTERNAL,
    [DAC_OUTPUT_PIN_UNBUFFERED]        = LL_DAC_OUTPUT_CONNECT_GPIO,
    [DAC_OUTPUT_INTERNAL]              = LL_DAC_OUTPUT_CONNECT_INTERNAL,
};


/** dac_OutputMode_t -> output is connected to the external pin (pin is configured as analog) */
static const dac_FunctionState_t dac_OutputPinLut[ DAC_OUTPUT_CNT ] =
{
    [DAC_OUTPUT_PIN_BUFFERED]          = DAC_FUNCTION_ACTIVE,
    [DAC_OUTPUT_PIN_INTERNAL_BUFFERED] = DAC_FUNCTION_ACTIVE,
    [DAC_OUTPUT_PIN_UNBUFFERED]        = DAC_FUNCTION_ACTIVE,
    [DAC_OUTPUT_INTERNAL]              = DAC_FUNCTION_INACTIVE,
};


/** dac_DataFormat_t -> LL_DAC_DMA_REG_DATA_x (data holding register of the format) */
static const uint32_t dac_DataRegLut[ DAC_DATA_FORMAT_CNT ] =
{
    [DAC_DATA_FORMAT_12BIT_RIGHT] = LL_DAC_DMA_REG_DATA_12BITS_RIGHT_ALIGNED,
    [DAC_DATA_FORMAT_12BIT_LEFT]  = LL_DAC_DMA_REG_DATA_12BITS_LEFT_ALIGNED,
    [DAC_DATA_FORMAT_8BIT_RIGHT]  = LL_DAC_DMA_REG_DATA_8BITS_RIGHT_ALIGNED,
};


/** dac_DataFormat_t -> bits of the data holding register used by the format (read-back mask) */
static const uint32_t dac_DataMaskLut[ DAC_DATA_FORMAT_CNT ] =
{
    [DAC_DATA_FORMAT_12BIT_RIGHT] = DAC_DHR12R1_DACC1DHR_Msk,
    [DAC_DATA_FORMAT_12BIT_LEFT]  = DAC_DHR12L1_DACC1DHR_Msk,
    [DAC_DATA_FORMAT_8BIT_RIGHT]  = DAC_DHR8R1_DACC1DHR_Msk,
};


/** dac_DataFormat_t -> maximum data value accepted by the format */
static const uint32_t dac_DataMaxLut[ DAC_DATA_FORMAT_CNT ] =
{
    [DAC_DATA_FORMAT_12BIT_RIGHT] = DAC_DATA_12BIT_MAX,
    [DAC_DATA_FORMAT_12BIT_LEFT]  = UINT16_MAX,
    [DAC_DATA_FORMAT_8BIT_RIGHT]  = DAC_DATA_8BIT_MAX,
};


/** dac_DataFormat_t -> full scale of the converted value (before alignment) */
static const uint32_t dac_FullScaleLut[ DAC_DATA_FORMAT_CNT ] =
{
    [DAC_DATA_FORMAT_12BIT_RIGHT] = DAC_DATA_12BIT_MAX,
    [DAC_DATA_FORMAT_12BIT_LEFT]  = DAC_DATA_12BIT_MAX,
    [DAC_DATA_FORMAT_8BIT_RIGHT]  = DAC_DATA_8BIT_MAX,
};


/** dac_DataFormat_t -> left shift of the converted value (alignment in the data register) */
static const uint32_t dac_DataShiftLut[ DAC_DATA_FORMAT_CNT ] =
{
    [DAC_DATA_FORMAT_12BIT_RIGHT] = 0u,
    [DAC_DATA_FORMAT_12BIT_LEFT]  = DAC_DHR12L1_DACC1DHR_Pos,
    [DAC_DATA_FORMAT_8BIT_RIGHT]  = 0u,
};


/** dac_DataFormat_t -> position of channel 2 data in the dual data holding register */
static const uint32_t dac_DualShiftLut[ DAC_DATA_FORMAT_CNT ] =
{
    [DAC_DATA_FORMAT_12BIT_RIGHT] = DAC_DHR12RD_DACC2DHR_Pos,
    [DAC_DATA_FORMAT_12BIT_LEFT]  = DAC_DHR12LD_DACC2DHR_Pos - DAC_DHR12LD_DACC1DHR_Pos,
    [DAC_DATA_FORMAT_8BIT_RIGHT]  = DAC_DHR8RD_DACC2DHR_Pos,
};


/** dac_WaveMode_t -> LL_DAC_WAVE_AUTO_GENERATION_x */
static const uint32_t dac_WaveModeLut[ DAC_WAVE_CNT ] =
{
    [DAC_WAVE_NONE]     = LL_DAC_WAVE_AUTO_GENERATION_NONE,
    [DAC_WAVE_NOISE]    = LL_DAC_WAVE_AUTO_GENERATION_NOISE,
    [DAC_WAVE_TRIANGLE] = LL_DAC_WAVE_AUTO_GENERATION_TRIANGLE,
};


/** dac_WaveAmplitude_t -> MAMPx value. LL_DAC_TRIANGLE_AMPLITUDE_x and LL_DAC_NOISE_LFSR_UNMASK_x
 *  share the same MAMPx encoding, the triangle definitions are used for both wave types. */
static const uint32_t dac_WaveAmplitudeLut[ DAC_WAVE_AMPLITUDE_CNT ] =
{
    [DAC_WAVE_AMPLITUDE_1]    = LL_DAC_TRIANGLE_AMPLITUDE_1,
    [DAC_WAVE_AMPLITUDE_3]    = LL_DAC_TRIANGLE_AMPLITUDE_3,
    [DAC_WAVE_AMPLITUDE_7]    = LL_DAC_TRIANGLE_AMPLITUDE_7,
    [DAC_WAVE_AMPLITUDE_15]   = LL_DAC_TRIANGLE_AMPLITUDE_15,
    [DAC_WAVE_AMPLITUDE_31]   = LL_DAC_TRIANGLE_AMPLITUDE_31,
    [DAC_WAVE_AMPLITUDE_63]   = LL_DAC_TRIANGLE_AMPLITUDE_63,
    [DAC_WAVE_AMPLITUDE_127]  = LL_DAC_TRIANGLE_AMPLITUDE_127,
    [DAC_WAVE_AMPLITUDE_255]  = LL_DAC_TRIANGLE_AMPLITUDE_255,
    [DAC_WAVE_AMPLITUDE_511]  = LL_DAC_TRIANGLE_AMPLITUDE_511,
    [DAC_WAVE_AMPLITUDE_1023] = LL_DAC_TRIANGLE_AMPLITUDE_1023,
    [DAC_WAVE_AMPLITUDE_2047] = LL_DAC_TRIANGLE_AMPLITUDE_2047,
    [DAC_WAVE_AMPLITUDE_4095] = LL_DAC_TRIANGLE_AMPLITUDE_4095,
};


/** dac_FlagId_t x dac_ChannelId_t -> LL_DAC_FLAG_x (DAC_SR bit), DAC_FLAG_UNSUPPORTED - no channel ready flag on STM32H7 */
static const uint32_t dac_FlagLut[ DAC_FLAG_CNT ][ DAC_CHANNEL_CNT ] =
{
    [DAC_FLAG_DMA_UNDERRUN]  = { [DAC_CHANNEL_1] = LL_DAC_FLAG_DMAUDR1, [DAC_CHANNEL_2] = LL_DAC_FLAG_DMAUDR2 },
    [DAC_FLAG_CALIBRATION]   = { [DAC_CHANNEL_1] = LL_DAC_FLAG_CAL1,    [DAC_CHANNEL_2] = LL_DAC_FLAG_CAL2    },
    [DAC_FLAG_SH_WRITE_BUSY] = { [DAC_CHANNEL_1] = LL_DAC_FLAG_BWST1,   [DAC_CHANNEL_2] = LL_DAC_FLAG_BWST2   },
    [DAC_FLAG_READY]         = { [DAC_CHANNEL_1] = DAC_FLAG_UNSUPPORTED, [DAC_CHANNEL_2] = DAC_FLAG_UNSUPPORTED },
};


/** Data format selected per peripheral / channel. The format selects which data holding
 *  register is written, it has no own HW configuration field. */
static dac_DataFormat_t dac_ShadowDataFormat[ DAC_PERIPH_CNT ][ DAC_CHANNEL_CNT ];


/** Data transfer runtime context per peripheral / channel (shared with DMA and interrupt
 *  handlers through Dac_Get_XferContext()) */
static dac_XferContext_t dac_XferContext[ DAC_PERIPH_CNT ][ DAC_CHANNEL_CNT ];

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
dac_ModuleVersion_t Dac_Get_ModuleVersion( void )
{
    dac_ModuleVersion_t retVersion;

    retVersion.Major = DAC_MAJOR_VERSION;
    retVersion.Minor = DAC_MINOR_VERSION;
    retVersion.Patch = DAC_PATCH_VERSION;

    return (retVersion);
}


/**
 * \brief Initializes module Dac
 *
 * Selects the shared ADC / DAC kernel clock source (which enables the DAC bus clock), then
 * initializes every peripheral from dacConfig->PeriphConfig[] that has at least one used
 * channel (see \ref Dac_PeriphInit()).
 *
 * \note  dacConfig->PeriphConfig[] is indexed by \ref dac_PeriphId_t. A slot without any channel
 *        with ChannelUsed == DAC_FUNCTION_ACTIVE is skipped. For a used slot
 *        PeriphConfig[ i ].PeriphId must be equal to i, otherwise \ref DAC_REQUEST_ERROR is returned.
 *
 * \note  If initialization of any peripheral fails, all peripherals touched by this call are
 *        deinitialized (see \ref Dac_Deinit()) and \ref DAC_REQUEST_ERROR is returned.
 *
 * \pre   All DAC channels must be disabled (checked by Dac_Set_ClockSource()).
 *
 * \param dacConfig [in]: Pointer to module configuration structure \ref dac_Config_t. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Init( dac_Config_t * const dacConfig )
{
    dac_RequestState_t  retState  = DAC_REQUEST_ERROR;
    dac_FunctionState_t clkConfOk = DAC_FUNCTION_INACTIVE;

    if( DAC_NULL_PTR != dacConfig )
    {
        /* ------------------------------------------------------------------ */
        /* ---------------------- Clock configuration ----------------------- */
        /* ------------------------------------------------------------------ */

        retState = Dac_Set_ClockSource( dacConfig->ClockSource );

        if( DAC_REQUEST_OK == retState )
        {
            /* All DAC channels were disabled - every peripheral touched from now on is owned by this initialization */
            clkConfOk = DAC_FUNCTION_ACTIVE;
        }
        else
        {
            /* Clock configuration failed, peripherals are not initialized */
            clkConfOk = DAC_FUNCTION_INACTIVE;
        }

        /* ------------------------------------------------------------------ */
        /* -------------------- Peripherals configuration ------------------- */
        /* ------------------------------------------------------------------ */

        for( dac_PeriphId_t periphIdx = DAC_PERIPH_1; ( DAC_PERIPH_CNT > periphIdx ) &&
                                                      ( DAC_REQUEST_OK == retState ); periphIdx ++ )
        {
            dac_PeriphConfig_t * const periphCfg  = &dacConfig->PeriphConfig[ periphIdx ];
            dac_FunctionState_t        periphUsed = DAC_FUNCTION_INACTIVE;

            for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; DAC_CHANNEL_CNT > channelIdx; channelIdx ++ )
            {
                if( DAC_FUNCTION_ACTIVE == periphCfg->Channels[ channelIdx ].ChannelUsed )
                {
                    periphUsed = DAC_FUNCTION_ACTIVE;
                }
                else
                {
                    /* Channel is not used */
                }
            }

            if( DAC_FUNCTION_INACTIVE == periphUsed )
            {
                /* Peripheral is not used - nothing to configure */
                retState = DAC_REQUEST_OK;
            }
            else if( periphIdx != periphCfg->PeriphId )
            {
                /* PeriphConfig[] slot does not match its peripheral identification */
                retState = DAC_REQUEST_ERROR;
            }
            else
            {
                retState = Dac_PeriphInit( periphCfg );
            }
        }

        /* ------------------------------------------------------------------ */
        /* --------------------- Initialization result ---------------------- */
        /* ------------------------------------------------------------------ */

        if( ( DAC_REQUEST_OK      != retState  ) &&
            ( DAC_FUNCTION_ACTIVE == clkConfOk )    )
        {
            /* Initialization failed - all peripherals are returned to the disabled state */
            for( dac_PeriphId_t periphIdx = DAC_PERIPH_1; DAC_PERIPH_CNT > periphIdx; periphIdx ++ )
            {
                /* Result is intentionally not evaluated, initialization is already reported as failed */
                (void)Dac_Deinit( periphIdx );
            }
        }
        else
        {
            /* Initialization succeeded or no peripheral was touched */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Deinitializes DAC peripheral
 *
 * Stops running DMA transfers and releases their resources (DMA channel, interrupts),
 * disables the channels, disables the DAC interrupt and the APB clock of the peripheral.
 *
 * \param periphId [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Deinit( dac_PeriphId_t periphId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( DAC_PERIPH_CNT > periphId )
    {
        retState = DAC_REQUEST_OK;

        for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; ( dac_PeriphConf[ periphId ].ChannelCnt > channelIdx ) &&
                                                         ( DAC_REQUEST_OK == retState )                        ; channelIdx ++ )
        {
            dac_XferContext_t * const xferCtx = &dac_XferContext[ periphId ][ channelIdx ];

            /* --- Release DMA transfer resources (DMA channel, interrupts) --- */
            if( DAC_FUNCTION_ACTIVE == xferCtx->InitState )
            {
                retState = Dac_Dma_Deinit( periphId, channelIdx );

                if( DAC_REQUEST_OK == retState )
                {
                    xferCtx->XferState = DAC_FUNCTION_INACTIVE;
                    xferCtx->InitState = DAC_FUNCTION_INACTIVE;
                }
                else
                {
                    /* DMA resources could not be released */
                }
            }
            else
            {
                /* DMA transfer is not initialized */
            }

            /* --- Disable channel --- */
            if( DAC_REQUEST_OK == retState )
            {
                retState = Dac_Set_ChannelInactive( periphId, channelIdx );
            }
            else
            {
                /* Previous step failed */
            }
        }

        /* --- Disable DAC interrupt --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Isr_Deinit( periphId );
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Disable APB clock of the peripheral --- */
        if( DAC_REQUEST_OK == retState )
        {
            const rcc_RequestState_t rccState = Rcc_Set_PeriphInactive( dac_PeriphConf[ periphId ].PeriphRcc );

            if( RCC_REQUEST_OK == rccState )
            {
                retState = DAC_REQUEST_OK;
            }
            else
            {
                retState = DAC_REQUEST_ERROR;
            }
        }
        else
        {
            /* Previous step failed */
        }

        if( DAC_REQUEST_OK == retState )
        {
            for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; DAC_CHANNEL_CNT > channelIdx; channelIdx ++ )
            {
                dac_ShadowDataFormat[ periphId ][ channelIdx ]              = DAC_DATA_FORMAT_12BIT_RIGHT;
                dac_XferContext[ periphId ][ channelIdx ].Config.TransferMode = DAC_TRANSFER_MODE_DIRECT;
            }
        }
        else
        {
            /* Previous step failed */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Main task of module Dac
 *
 * This function shall be called in the main loop of the application or the task
 * scheduler. It shall be called periodically, depending on the module's
 * requirements.
 *
 * \note  The DAC module has no periodic processing - direct output values are written
 *        synchronously and DMA transfers are serviced from interrupts. The task is
 *        provided for consistency of the MCAL interface.
 */
void Dac_Task( void )
{
    /* No periodic processing required */
    return;
}


/**
 * \brief Returns default module configuration
 *
 * Default configuration: clock PCLK, sample and hold clock LSI, DAC1 channel 1 used with
 * buffered pin output, no trigger, 12-bit right aligned data, no wave generation, sample and
 * hold disabled, initial value 0 and direct data transfer. Other channels and peripherals are
 * not used, default DMA streams: DAC1 channel 1 / 2 - DMA1 stream 5 / 6 (DAC2 - DMA1 stream 7,
 * DMA transfer not available).
 *
 * \param dacConfig [out]: Pointer to module configuration structure to be filled. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_DefaultConfig( dac_Config_t * const dacConfig )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( DAC_NULL_PTR != dacConfig )
    {
        dacConfig->ClockSource = DAC_CLK_SRC_PCLK;

        for( dac_PeriphId_t periphIdx = DAC_PERIPH_1; DAC_PERIPH_CNT > periphIdx; periphIdx ++ )
        {
            dac_PeriphConfig_t * const periphCfg = &dacConfig->PeriphConfig[ periphIdx ];

            periphCfg->PeriphId      = periphIdx;
            periphCfg->ShClockSource = DAC_SH_CLK_SRC_LSI;

            for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; DAC_CHANNEL_CNT > channelIdx; channelIdx ++ )
            {
                dac_ChannelConfig_t * const channelCfg = &periphCfg->Channels[ channelIdx ];

                channelCfg->ChannelId                           = channelIdx;
                channelCfg->ChannelUsed                         = DAC_FUNCTION_INACTIVE;
                channelCfg->OutputMode                          = ( DAC_FUNCTION_ACTIVE == dac_PeriphConf[ periphIdx ].ExternalOut ) ? DAC_OUTPUT_PIN_BUFFERED : DAC_OUTPUT_INTERNAL;
                channelCfg->TriggerId                           = DAC_TRIGGER_NONE;
                channelCfg->DataFormat                          = DAC_DATA_FORMAT_12BIT_RIGHT;
                channelCfg->WaveMode                            = DAC_WAVE_NONE;
                channelCfg->WaveAmplitude                       = DAC_WAVE_AMPLITUDE_1;
                channelCfg->SampleHold.State                    = DAC_FUNCTION_INACTIVE;
                channelCfg->SampleHold.SampleTime               = 0u;
                channelCfg->SampleHold.HoldTime                 = 0u;
                channelCfg->SampleHold.RefreshTime              = 0u;
                channelCfg->InitValue                           = 0u;
                channelCfg->DataConfig.TransferMode             = DAC_TRANSFER_MODE_DIRECT;
                channelCfg->DataConfig.DataBuffer               = DAC_NULL_PTR;
                channelCfg->DataConfig.BufferSize               = 0u;
                channelCfg->DataConfig.BufferMode               = DAC_BUFFER_MODE_ONE_SHOT;
                channelCfg->DataConfig.DmaPeriphId              = dac_DmaDefaultConfig[ periphIdx ][ channelIdx ].DmaPeriphId;
                channelCfg->DataConfig.DmaChannelId             = dac_DmaDefaultConfig[ periphIdx ][ channelIdx ].DmaChannelId;
                channelCfg->DataConfig.DmaPriority              = DAC_DMA_PRIORITY_LOW;
                channelCfg->DataConfig.IrqPriority              = 0u;
                channelCfg->DataConfig.HalfTransferCallback     = DAC_NULL_PTR;
                channelCfg->DataConfig.TransferCompleteCallback = DAC_NULL_PTR;
                channelCfg->DataConfig.ErrorCallback            = DAC_NULL_PTR;
            }
        }

        dacConfig->PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_ACTIVE;

        retState = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* -------------------------- Clock configuration --------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Selects the DAC clock source
 *
 * \note  STM32H7 DAC has no kernel clock multiplexer - DAC1 is clocked by PCLK1, DAC2 by PCLK4
 *        (\ref DAC_CLK_SRC_PCLK). The clock of a peripheral is enabled by \ref Dac_PeriphInit and
 *        disabled by \ref Dac_Deinit, no register is written here.
 *
 * \pre   All DAC channels must be disabled. Otherwise \ref DAC_REQUEST_ERROR is returned.
 *
 * \param clkSource [in]: Required DAC clock source, value from \ref dac_ClkSrc_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_ClockSource( dac_ClkSrc_t clkSource )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( DAC_CLK_SRC_CNT > clkSource )
    {
        retState = Dac_Check_AllPeriphsDisabled( );
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the DAC clock source (always the APB clock on STM32H7)
 *
 * \param clkSource [out]: Pointer to store the clock source (\ref dac_ClkSrc_t). Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_ClockSource( dac_ClkSrc_t * const clkSource )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( DAC_NULL_PTR != clkSource )
    {
        *clkSource = DAC_CLK_SRC_PCLK;
        retState   = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects the sample and hold clock source (common for all channels)
 *
 * \note  STM32H7 sample and hold mode is clocked by LSI only - LSI is started by RCC module if it
 *        is not running, \ref DAC_SH_CLK_SRC_LSE is refused.
 *
 * \pre   All DAC channels must be disabled. Otherwise \ref DAC_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param clkSource [in]: Required sample and hold clock source, value from \ref dac_ShClkSrc_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_ShClockSource( dac_ShClkSrc_t clkSource )
{
    dac_RequestState_t  retState = DAC_REQUEST_ERROR;
    rcc_FunctionState_t lsiState = RCC_FUNCTION_INACTIVE;

    if( DAC_SH_CLK_SRC_LSI == clkSource )
    {
        retState = Dac_Check_AllPeriphsDisabled( );

        if( DAC_REQUEST_OK == retState )
        {
            rcc_RequestState_t rccState = Rcc_Get_OscState( RCC_OSC_LSI, &lsiState );

            if( ( RCC_REQUEST_OK        == rccState ) &&
                ( RCC_FUNCTION_INACTIVE == lsiState )    )
            {
                rccState = Rcc_Set_OscActive( RCC_OSC_LSI );
            }
            else
            {
                /* LSI is running or its state is not available */
            }

            if( RCC_REQUEST_OK == rccState )
            {
                retState = DAC_REQUEST_OK;
            }
            else
            {
                retState = DAC_REQUEST_ERROR;
            }
        }
        else
        {
            /* At least one DAC channel is enabled, configuration change is not allowed */
        }
    }
    else
    {
        /* Invalid source or LSE (not available on STM32H7) */
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the sample and hold clock source (always LSI on STM32H7)
 *
 * \param clkSource [out]: Pointer to store the clock source (\ref dac_ShClkSrc_t). Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_ShClockSource( dac_ShClkSrc_t * const clkSource )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( DAC_NULL_PTR != clkSource )
    {
        *clkSource = DAC_SH_CLK_SRC_LSI;
        retState   = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ------------------- Peripheral and channel configuration ----------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Initializes a DAC peripheral and all its used channels
 *
 * - APB clock of the peripheral is enabled (RCC).
 * - Sample and hold clock is selected if any used channel has sample and hold active.
 * - Every channel with ChannelUsed == DAC_FUNCTION_ACTIVE is initialized by Dac_ChannelInit().
 *
 * \note  Channels[] is indexed by \ref dac_ChannelId_t, Channels[ i ].ChannelId must be equal to i
 *        for a used channel.
 *
 * \pre   All channels of the peripheral must be disabled. Otherwise \ref DAC_REQUEST_ERROR is
 *        returned and no register is modified.
 *
 * \param periphConfig [in]: Pointer to peripheral configuration structure \ref dac_PeriphConfig_t.
 *                           Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_PeriphInit( dac_PeriphConfig_t * const periphConfig )
{
    dac_RequestState_t  retState = DAC_REQUEST_ERROR;
    dac_FunctionState_t shUsed   = DAC_FUNCTION_INACTIVE;

    if( ( DAC_NULL_PTR       != periphConfig                ) &&
        ( DAC_PERIPH_CNT      > periphConfig->PeriphId      ) &&
        ( DAC_SH_CLK_SRC_CNT  > periphConfig->ShClockSource )    )
    {
        const dac_PeriphId_t periphId = periphConfig->PeriphId;

        /* --- Whole configuration is checked before any register is modified --- */
        retState = Dac_Check_PeriphDisabled( periphId );

        for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; ( DAC_CHANNEL_CNT > channelIdx ) &&
                                                         ( DAC_REQUEST_OK == retState )  ; channelIdx ++ )
        {
            const dac_ChannelConfig_t * const channelCfg = &periphConfig->Channels[ channelIdx ];

            if( DAC_FUNCTION_ACTIVE != channelCfg->ChannelUsed )
            {
                /* Channel is not used */
            }
            else if( channelIdx != channelCfg->ChannelId )
            {
                /* Channels[] slot does not match its channel identification */
                retState = DAC_REQUEST_ERROR;
            }
            else
            {
                retState = Dac_Check_ChannelConfig( periphId, channelCfg );

                if( DAC_FUNCTION_ACTIVE == channelCfg->SampleHold.State )
                {
                    shUsed = DAC_FUNCTION_ACTIVE;
                }
                else
                {
                    /* Sample and hold is not used by the channel */
                }
            }
        }

        /* --- Channels with DMA transfer shall use different DMA channels --- */
        if( DAC_REQUEST_OK == retState )
        {
            const dac_ChannelConfig_t * const channel1Cfg = &periphConfig->Channels[ DAC_CHANNEL_1 ];
            const dac_ChannelConfig_t * const channel2Cfg = &periphConfig->Channels[ DAC_CHANNEL_2 ];

            if( ( DAC_FUNCTION_ACTIVE                  == channel1Cfg->ChannelUsed             ) &&
                ( DAC_FUNCTION_ACTIVE                  == channel2Cfg->ChannelUsed             ) &&
                ( DAC_TRANSFER_MODE_DMA                == channel1Cfg->DataConfig.TransferMode ) &&
                ( DAC_TRANSFER_MODE_DMA                == channel2Cfg->DataConfig.TransferMode ) &&
                ( channel1Cfg->DataConfig.DmaPeriphId  == channel2Cfg->DataConfig.DmaPeriphId  ) &&
                ( channel1Cfg->DataConfig.DmaChannelId == channel2Cfg->DataConfig.DmaChannelId )    )
            {
                /* Both DAC channels request the same DMA channel */
                retState = DAC_REQUEST_ERROR;
            }
            else
            {
                /* DMA channels do not collide */
            }
        }
        else
        {
            /* Previous step failed */
        }

        /* --- APB clock of the peripheral --- */
        if( DAC_REQUEST_OK == retState )
        {
            const rcc_RequestState_t rccState = Rcc_Set_PeriphActive( dac_PeriphConf[ periphId ].PeriphRcc );

            if( RCC_REQUEST_OK == rccState )
            {
                retState = DAC_REQUEST_OK;
            }
            else
            {
                retState = DAC_REQUEST_ERROR;
            }
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Sample and hold clock --- */
        if( ( DAC_REQUEST_OK      == retState ) &&
            ( DAC_FUNCTION_ACTIVE == shUsed   )    )
        {
            retState = Dac_Set_ShClockSource( periphConfig->ShClockSource );
        }
        else
        {
            /* Initialization failed or sample and hold clock is not needed */
        }

        /* --- Channels --- */
        for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; ( DAC_CHANNEL_CNT > channelIdx ) &&
                                                         ( DAC_REQUEST_OK == retState )  ; channelIdx ++ )
        {
            if( DAC_FUNCTION_ACTIVE == periphConfig->Channels[ channelIdx ].ChannelUsed )
            {
                retState = Dac_ChannelInit( periphId, &periphConfig->Channels[ channelIdx ] );
            }
            else
            {
                /* Channel is not used */
            }
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes and enables a single DAC channel
 *
 * Sequence: output pin (analog, only for pin output modes), offset calibration (peripherals with
 * buffered external outputs), output mode,
 * sample and hold, trigger, wave generation, data format, initial output value, data transfer
 * configuration (DMA resources) and channel enable.
 *
 * \note  The whole configuration is checked before any register is modified.
 *
 * \note  In DAC_TRANSFER_MODE_DMA the DMA transfer is not started, call Dac_Set_DmaStart().
 *
 * \pre   The channel must be disabled. Otherwise \ref DAC_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId      [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelConfig [in]: Pointer to channel configuration structure \ref dac_ChannelConfig_t.
 *                            Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_ChannelInit( dac_PeriphId_t periphId, dac_ChannelConfig_t * const channelConfig )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT  > periphId      ) &&
        ( DAC_NULL_PTR   != channelConfig )    )
    {
        const dac_ChannelId_t channelId = channelConfig->ChannelId;

        retState = Dac_Check_ChannelConfig( periphId, channelConfig );

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Check_ChannelDisabled( periphId, channelId );
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Output pin --- */
        if( ( DAC_REQUEST_OK      == retState                                      ) &&
            ( DAC_FUNCTION_ACTIVE == dac_OutputPinLut[ channelConfig->OutputMode ] )    )
        {
            retState = Dac_Set_PinAnalog( periphId, channelId );
        }
        else
        {
            /* Initialization failed or output is not connected to the pin */
        }

        /* --- Offset calibration (output mode is restored by the calibration) --- */
        if( ( DAC_REQUEST_OK      == retState                               ) &&
            ( DAC_FUNCTION_ACTIVE == dac_PeriphConf[ periphId ].ExternalOut )    )
        {
            retState = Dac_Set_Calibration( periphId, channelId );
        }
        else
        {
            /* Initialization failed or peripheral without buffered outputs */
        }

        /* --- Output and sample and hold mode --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_OutputMode( periphId, channelId, channelConfig->OutputMode );
        }
        else
        {
            /* Previous step failed */
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_SampleHoldConfig( periphId, channelId, &channelConfig->SampleHold );
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Trigger, wave generation and data format --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_TriggerSrc( periphId, channelId, channelConfig->TriggerId );
        }
        else
        {
            /* Previous step failed */
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_WaveAmplitude( periphId, channelId, channelConfig->WaveAmplitude );
        }
        else
        {
            /* Previous step failed */
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_WaveMode( periphId, channelId, channelConfig->WaveMode );
        }
        else
        {
            /* Previous step failed */
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_DataFormat( periphId, channelId, channelConfig->DataFormat );
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Initial output value (written before the data transfer configuration) --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_Data( periphId, channelId, channelConfig->InitValue );
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Data transfer resources --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_DataConfig( periphId, channelId, &channelConfig->DataConfig );
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Channel enable --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_ChannelActive( periphId, channelId );
        }
        else
        {
            /* Previous step failed */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables a DAC channel (ENx read back)
 *
 * \note  STM32H7 DAC has no channel ready flag. The output settles within the wakeup time
 *        (tWAKEUP) after the enable (refer to device datasheet), this time is not waited.
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_ChannelActive( dac_PeriphId_t periphId, dac_ChannelId_t channelId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                        > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt > channelId )    )
    {
        DAC_TypeDef * const periphReg = dac_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      llChannel = dac_LlChannelLut[ channelId ];

        LL_DAC_Enable( periphReg, llChannel );

        for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t enabled = LL_DAC_IsEnabled( periphReg, llChannel );

            if( 0u != enabled )
            {
                retState = DAC_REQUEST_OK;
                break;
            }
            else
            {
                /* Channel enable has not yet been applied, keep return state as error */
                retState = DAC_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disables a DAC channel
 *
 * \note  A running DMA transfer of the channel is stopped first (Dac_Set_DmaStop()).
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_ChannelInactive( dac_PeriphId_t periphId, dac_ChannelId_t channelId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                        > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt > channelId )    )
    {
        DAC_TypeDef * const periphReg = dac_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      llChannel = dac_LlChannelLut[ channelId ];

        retState = DAC_REQUEST_OK;

        if( DAC_FUNCTION_ACTIVE == dac_XferContext[ periphId ][ channelId ].XferState )
        {
            retState = Dac_Set_DmaStop( periphId, channelId );
        }
        else
        {
            /* No DMA transfer is running */
        }

        if( DAC_REQUEST_OK == retState )
        {
            LL_DAC_Disable( periphReg, llChannel );

            for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t enabled = LL_DAC_IsEnabled( periphReg, llChannel );

                if( 0u == enabled )
                {
                    retState = DAC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Channel disable has not yet been applied, keep return state as error */
                    retState = DAC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* DMA transfer could not be stopped, channel is kept enabled */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns enable state of a DAC channel
 *
 * \param periphId      [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId     [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param channelState [out]: Pointer to store the channel state. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_ChannelState( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_FunctionState_t * const channelState )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId     ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId    ) &&
        ( DAC_NULL_PTR                          != channelState )    )
    {
        const uint32_t enabled = LL_DAC_IsEnabled( dac_PeriphConf[ periphId ].PeriphReg, dac_LlChannelLut[ channelId ] );

        if( 0u != enabled )
        {
            *channelState = DAC_FUNCTION_ACTIVE;
        }
        else
        {
            *channelState = DAC_FUNCTION_INACTIVE;
        }

        retState = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects the channel output mode (output buffer and output connection)
 *
 * \note  The output pin is not configured by this function (see Dac_ChannelInit()).
 *
 * \pre   The channel must be disabled (MODEx is writable only while ENx = 0). Otherwise
 *        \ref DAC_REQUEST_ERROR is returned and no register is modified.
 *
 * \param periphId   [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId  [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param outputMode [in]: Required output mode, value from \ref dac_OutputMode_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_OutputMode( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_OutputMode_t outputMode )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                        > periphId   ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt > channelId  ) &&
        ( DAC_OUTPUT_CNT                        > outputMode )    )
    {
        retState = Dac_Check_OutputMode( periphId, outputMode );

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Check_ChannelDisabled( periphId, channelId );
        }
        else
        {
            /* Output mode is not available on the peripheral */
        }

        if( DAC_REQUEST_OK == retState )
        {
            /* Sample and hold mode bit is kept */
            const uint32_t llShMode = LL_DAC_GetOutputMode( dac_PeriphConf[ periphId ].PeriphReg, dac_LlChannelLut[ channelId ] );

            retState = Dac_Write_ChannelMode( periphId, channelId, dac_OutputBufferLut[ outputMode ], dac_OutputConnectLut[ outputMode ], llShMode );
        }
        else
        {
            /* Channel is enabled, configuration change is not allowed */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the channel output mode (output buffer and output connection)
 *
 * \param periphId    [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId   [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param outputMode [out]: Pointer to store the output mode. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_OutputMode( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_OutputMode_t * const outputMode )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId   ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId  ) &&
        ( DAC_NULL_PTR                          != outputMode )    )
    {
        DAC_TypeDef * const periphReg = dac_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      llBuffer  = LL_DAC_GetOutputBuffer( periphReg, dac_LlChannelLut[ channelId ] );
        const uint32_t      llConnect = LL_DAC_GetOutputConnection( periphReg, dac_LlChannelLut[ channelId ] );

        for( dac_OutputMode_t modeIdx = DAC_OUTPUT_PIN_BUFFERED; DAC_OUTPUT_CNT > modeIdx; modeIdx ++ )
        {
            if( ( dac_OutputBufferLut[ modeIdx ]  == llBuffer  ) &&
                ( dac_OutputConnectLut[ modeIdx ] == llConnect )    )
            {
                *outputMode = modeIdx;
                retState    = DAC_REQUEST_OK;
            }
            else
            {
                /* Output mode does not match */
            }
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects the channel conversion trigger (TENx, TSELx)
 *
 * \pre   The channel must be disabled and the trigger connected to the peripheral (see
 *        \ref dac_TriggerId_t). Otherwise \ref DAC_REQUEST_ERROR is returned and no register is
 *        modified.
 *
 * \pre   A channel with initialized DMA transfer requires a hardware trigger (DAC DMA requests
 *        are not generated without trigger or on software trigger). Otherwise
 *        \ref DAC_REQUEST_ERROR is returned and no register is modified.
 *
 * \param periphId   [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId  [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param triggerSrc [in]: Required trigger, value from \ref dac_TriggerId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_TriggerSrc( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TriggerId_t triggerSrc )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId                                    ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId                                   ) &&
        ( DAC_TRIGGER_CNT                        > triggerSrc                                  ) &&
        ( DAC_TRIG_UNSUPPORTED                  != dac_TriggerSrcLut[ periphId ][ triggerSrc ] )    )
    {
        const dac_XferContext_t * const xferCtx = &dac_XferContext[ periphId ][ channelId ];

        retState = Dac_Check_ChannelDisabled( periphId, channelId );

        if( ( DAC_REQUEST_OK        == retState                     ) &&
            ( DAC_FUNCTION_ACTIVE   == xferCtx->InitState           ) &&
            ( DAC_TRANSFER_MODE_DMA == xferCtx->Config.TransferMode )    )
        {
            retState = Dac_Check_DmaTrigger( triggerSrc );
        }
        else
        {
            /* Channel is enabled or DMA transfer is not initialized */
        }

        if( DAC_REQUEST_OK == retState )
        {
            DAC_TypeDef * const periphReg = dac_PeriphConf[ periphId ].PeriphReg;
            const uint32_t      llChannel = dac_LlChannelLut[ channelId ];
            const uint32_t      llTrigger = dac_TriggerSrcLut[ periphId ][ triggerSrc ];
            uint32_t            llTrigEn  = 0u;

            LL_DAC_SetTriggerSource( periphReg, llChannel, llTrigger );

            if( DAC_TRIGGER_NONE == triggerSrc )
            {
                LL_DAC_DisableTrigger( periphReg, llChannel );
                llTrigEn = 0u;
            }
            else
            {
                LL_DAC_EnableTrigger( periphReg, llChannel );
                llTrigEn = 1u;
            }

            for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t trigReg   = LL_DAC_GetTriggerSource( periphReg, llChannel );
                const uint32_t trigEnReg = LL_DAC_IsTriggerEnabled( periphReg, llChannel );

                if( ( llTrigger == trigReg   ) &&
                    ( llTrigEn  == trigEnReg )    )
                {
                    retState = DAC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Trigger has not yet been applied, keep return state as error */
                    retState = DAC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Channel is enabled or trigger is not allowed for DMA transfer */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the channel conversion trigger
 *
 * \param periphId    [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId   [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param triggerSrc [out]: Pointer to store the trigger. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_TriggerSrc( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TriggerId_t * const triggerSrc )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId   ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId  ) &&
        ( DAC_NULL_PTR                          != triggerSrc )    )
    {
        DAC_TypeDef * const periphReg = dac_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      llChannel = dac_LlChannelLut[ channelId ];
        const uint32_t      trigEnReg = LL_DAC_IsTriggerEnabled( periphReg, llChannel );
        const uint32_t      trigReg   = LL_DAC_GetTriggerSource( periphReg, llChannel );

        if( 0u == trigEnReg )
        {
            *triggerSrc = DAC_TRIGGER_NONE;
            retState    = DAC_REQUEST_OK;
        }
        else
        {
            for( dac_TriggerId_t trigIdx = DAC_TRIGGER_SOFTWARE; ( DAC_TRIGGER_CNT > trigIdx )  &&
                                                                 ( DAC_REQUEST_OK != retState ); trigIdx ++ )
            {
                if( dac_TriggerSrcLut[ periphId ][ trigIdx ] == trigReg )
                {
                    *triggerSrc = trigIdx;
                    retState    = DAC_REQUEST_OK;
                }
                else
                {
                    /* Trigger does not match */
                }
            }
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects the channel data format (data holding register used for the output value)
 *
 * \note  The format has no own HW configuration field - it is stored in the module and
 *        used by Dac_Set_Data(), Dac_Set_DualData() and the DMA transfer.
 *
 * \pre   No DMA transfer may be running on the channel. Otherwise \ref DAC_REQUEST_ERROR is
 *        returned.
 *
 * \param periphId   [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId  [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param dataFormat [in]: Required data format, value from \ref dac_DataFormat_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_DataFormat( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_DataFormat_t dataFormat )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId                                           ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId                                          ) &&
        ( DAC_DATA_FORMAT_CNT                    > dataFormat                                         ) &&
        ( DAC_FUNCTION_ACTIVE                   != dac_XferContext[ periphId ][ channelId ].XferState )    )
    {
        dac_ShadowDataFormat[ periphId ][ channelId ] = dataFormat;
        retState                                      = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the channel data format
 *
 * \param periphId    [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId   [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param dataFormat [out]: Pointer to store the data format. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_DataFormat( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_DataFormat_t * const dataFormat )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId   ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId  ) &&
        ( DAC_NULL_PTR                          != dataFormat )    )
    {
        *dataFormat = dac_ShadowDataFormat[ periphId ][ channelId ];
        retState    = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects the channel wave auto generation mode (WAVEx)
 *
 * \note  Wave generation requires a trigger (TENx = 1), the wave value is added to the data
 *        holding register on every trigger.
 *
 * \pre   The channel must be disabled. Otherwise \ref DAC_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param waveMode  [in]: Required wave mode, value from \ref dac_WaveMode_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_WaveMode( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_WaveMode_t waveMode )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                        > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt > channelId ) &&
        ( DAC_WAVE_CNT                          > waveMode  )    )
    {
        retState = Dac_Check_ChannelDisabled( periphId, channelId );

        if( DAC_REQUEST_OK == retState )
        {
            DAC_TypeDef * const periphReg = dac_PeriphConf[ periphId ].PeriphReg;
            const uint32_t      llChannel = dac_LlChannelLut[ channelId ];

            LL_DAC_SetWaveAutoGeneration( periphReg, llChannel, dac_WaveModeLut[ waveMode ] );

            for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t waveReg = LL_DAC_GetWaveAutoGeneration( periphReg, llChannel );

                if( dac_WaveModeLut[ waveMode ] == waveReg )
                {
                    retState = DAC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Wave mode has not yet been applied, keep return state as error */
                    retState = DAC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Channel is enabled, configuration change is not allowed */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the channel wave auto generation mode
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param waveMode [out]: Pointer to store the wave mode. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_WaveMode( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_WaveMode_t * const waveMode )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId ) &&
        ( DAC_NULL_PTR                          != waveMode  )    )
    {
        const uint32_t waveReg = LL_DAC_GetWaveAutoGeneration( dac_PeriphConf[ periphId ].PeriphReg, dac_LlChannelLut[ channelId ] );

        for( dac_WaveMode_t waveIdx = DAC_WAVE_NONE; DAC_WAVE_CNT > waveIdx; waveIdx ++ )
        {
            if( dac_WaveModeLut[ waveIdx ] == waveReg )
            {
                *waveMode = waveIdx;
                retState  = DAC_REQUEST_OK;
            }
            else
            {
                /* Wave mode does not match */
            }
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects the channel wave amplitude (triangle) / LFSR mask (noise) (MAMPx)
 *
 * \pre   The channel must be disabled. Otherwise \ref DAC_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param amplitude [in]: Required amplitude, value from \ref dac_WaveAmplitude_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_WaveAmplitude( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_WaveAmplitude_t amplitude )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                        > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt > channelId ) &&
        ( DAC_WAVE_AMPLITUDE_CNT                > amplitude )    )
    {
        retState = Dac_Check_ChannelDisabled( periphId, channelId );

        if( DAC_REQUEST_OK == retState )
        {
            DAC_TypeDef * const periphReg = dac_PeriphConf[ periphId ].PeriphReg;
            const uint32_t      llChannel = dac_LlChannelLut[ channelId ];

            LL_DAC_SetWaveTriangleAmplitude( periphReg, llChannel, dac_WaveAmplitudeLut[ amplitude ] );

            for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t ampReg = LL_DAC_GetWaveTriangleAmplitude( periphReg, llChannel );

                if( dac_WaveAmplitudeLut[ amplitude ] == ampReg )
                {
                    retState = DAC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Amplitude has not yet been applied, keep return state as error */
                    retState = DAC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Channel is enabled, configuration change is not allowed */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the channel wave amplitude (triangle) / LFSR mask (noise)
 *
 * \param periphId   [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId  [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param amplitude [out]: Pointer to store the amplitude. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_WaveAmplitude( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_WaveAmplitude_t * const amplitude )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId ) &&
        ( DAC_NULL_PTR                          != amplitude )    )
    {
        const uint32_t ampReg = LL_DAC_GetWaveTriangleAmplitude( dac_PeriphConf[ periphId ].PeriphReg, dac_LlChannelLut[ channelId ] );

        for( dac_WaveAmplitude_t ampIdx = DAC_WAVE_AMPLITUDE_1; DAC_WAVE_AMPLITUDE_CNT > ampIdx; ampIdx ++ )
        {
            if( dac_WaveAmplitudeLut[ ampIdx ] == ampReg )
            {
                *amplitude = ampIdx;
                retState   = DAC_REQUEST_OK;
            }
            else
            {
                /* Amplitude does not match */
            }
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures channel sample and hold mode and its timing
 *
 * \note  The sample and hold clock (Dac_Set_ShClockSource()) must be running while the mode is
 *        active. Timing registers are written also with State == DAC_FUNCTION_INACTIVE.
 *
 * \note  After the sample time write the busy flag BWSTx is waited (sample time is transferred
 *        to the sample and hold clock domain).
 *
 * \pre   The channel must be disabled (MODEx, SHHR, SHRR are writable only while ENx = 0).
 *        Otherwise \ref DAC_REQUEST_ERROR is returned and no register is modified.
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param shConfig  [in]: Pointer to sample and hold configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_SampleHoldConfig( dac_PeriphId_t periphId, dac_ChannelId_t channelId, const dac_SampleHoldConfig_t * const shConfig )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId ) &&
        ( DAC_NULL_PTR                          != shConfig  )    )
    {
        DAC_TypeDef * const periphReg = dac_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      llChannel = dac_LlChannelLut[ channelId ];

        retState = Dac_Check_SampleHoldConfig( shConfig );

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Check_ChannelDisabled( periphId, channelId );
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Sample time (transfer to the sample and hold clock domain signaled by BWSTx) --- */
        if( DAC_REQUEST_OK == retState )
        {
            LL_DAC_SetSampleAndHoldSampleTime( periphReg, llChannel, (uint32_t)shConfig->SampleTime );

            for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t busyReg   = READ_BIT( periphReg->SR, dac_FlagLut[ DAC_FLAG_SH_WRITE_BUSY ][ channelId ] );
                const uint32_t sampleReg = LL_DAC_GetSampleAndHoldSampleTime( periphReg, llChannel );

                if( ( 0u                             == busyReg   ) &&
                    ( (uint32_t)shConfig->SampleTime == sampleReg )    )
                {
                    retState = DAC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Sample time has not yet been applied, keep return state as error */
                    retState = DAC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Hold and refresh time --- */
        if( DAC_REQUEST_OK == retState )
        {
            LL_DAC_SetSampleAndHoldHoldTime( periphReg, llChannel, (uint32_t)shConfig->HoldTime );
            LL_DAC_SetSampleAndHoldRefreshTime( periphReg, llChannel, (uint32_t)shConfig->RefreshTime );

            for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t holdReg    = LL_DAC_GetSampleAndHoldHoldTime( periphReg, llChannel );
                const uint32_t refreshReg = LL_DAC_GetSampleAndHoldRefreshTime( periphReg, llChannel );

                if( ( (uint32_t)shConfig->HoldTime    == holdReg    ) &&
                    ( (uint32_t)shConfig->RefreshTime == refreshReg )    )
                {
                    retState = DAC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Hold / refresh time has not yet been applied, keep return state as error */
                    retState = DAC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Sample and hold mode (output buffer and connection are kept) --- */
        if( DAC_REQUEST_OK == retState )
        {
            const uint32_t llBuffer  = LL_DAC_GetOutputBuffer( periphReg, llChannel );
            const uint32_t llConnect = LL_DAC_GetOutputConnection( periphReg, llChannel );
            uint32_t       llShMode  = LL_DAC_OUTPUT_MODE_NORMAL;

            if( DAC_FUNCTION_ACTIVE == shConfig->State )
            {
                llShMode = LL_DAC_OUTPUT_MODE_SAMPLE_AND_HOLD;
            }
            else
            {
                llShMode = LL_DAC_OUTPUT_MODE_NORMAL;
            }

            retState = Dac_Write_ChannelMode( periphId, channelId, llBuffer, llConnect, llShMode );
        }
        else
        {
            /* Previous step failed */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns channel sample and hold mode and its timing
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param shConfig [out]: Pointer to store the sample and hold configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_SampleHoldConfig( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_SampleHoldConfig_t * const shConfig )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId ) &&
        ( DAC_NULL_PTR                          != shConfig  )    )
    {
        DAC_TypeDef * const periphReg = dac_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      llChannel = dac_LlChannelLut[ channelId ];
        const uint32_t      llShMode  = LL_DAC_GetOutputMode( periphReg, llChannel );

        if( LL_DAC_OUTPUT_MODE_SAMPLE_AND_HOLD == llShMode )
        {
            shConfig->State = DAC_FUNCTION_ACTIVE;
        }
        else
        {
            shConfig->State = DAC_FUNCTION_INACTIVE;
        }

        shConfig->SampleTime  = (dac_ShSampleTime_t)LL_DAC_GetSampleAndHoldSampleTime( periphReg, llChannel );
        shConfig->HoldTime    = (dac_ShHoldTime_t)LL_DAC_GetSampleAndHoldHoldTime( periphReg, llChannel );
        shConfig->RefreshTime = (dac_ShRefreshTime_t)LL_DAC_GetSampleAndHoldRefreshTime( periphReg, llChannel );

        retState = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ------------------------------ Calibration ------------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Executes channel offset calibration and stores the found trimming value (OTRIMx)
 *
 * Calibration is executed in normal mode with output buffer enabled (the output mode is
 * restored afterwards). The trimming value is searched by binary search: the result is the
 * lowest value for which the calibration flag CAL_FLAGx is set (same procedure as ST HAL).
 * DAC_CAL_STEP_DELAY_US is waited after every trimming value change.
 *
 * \note  Calibration is relevant for buffered output modes. The output pin must not be loaded
 *        during the calibration.
 *
 * \pre   The channel must be disabled. Otherwise \ref DAC_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_Calibration( dac_PeriphId_t periphId, dac_ChannelId_t channelId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId                               ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId                              ) &&
        ( DAC_FUNCTION_ACTIVE                   == dac_PeriphConf[ periphId ].ExternalOut )    )
    {
        DAC_TypeDef * const periphReg  = dac_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      llChannel  = dac_LlChannelLut[ channelId ];
        const uint32_t      oldBuffer  = LL_DAC_GetOutputBuffer( periphReg, llChannel );
        const uint32_t      oldConnect = LL_DAC_GetOutputConnection( periphReg, llChannel );
        const uint32_t      oldShMode  = LL_DAC_GetOutputMode( periphReg, llChannel );
        dac_TrimValue_t     trimValue  = DAC_CAL_TRIM_START;
        dac_FlagState_t     calFlag    = DAC_FLAG_INACTIVE;

        retState = Dac_Check_ChannelDisabled( periphId, channelId );

        /* --- Normal mode, buffer enabled, calibration mode (CENx) --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Write_ChannelMode( periphId, channelId, LL_DAC_OUTPUT_BUFFER_ENABLE, LL_DAC_OUTPUT_CONNECT_GPIO, LL_DAC_OUTPUT_MODE_NORMAL );
        }
        else
        {
            /* Previous step failed */
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Write_CalibrationMode( periphId, channelId, LL_DAC_MODE_CALIBRATION );
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Binary search of the trimming value --- */
        for( dac_TrimValue_t trimStep = DAC_CAL_TRIM_STEP; ( 0u != trimStep )             &&
                                                           ( DAC_REQUEST_OK == retState ); trimStep >>= 1u )
        {
            retState = Dac_Write_TrimValue( periphId, channelId, trimValue );

            if( DAC_REQUEST_OK == retState )
            {
                retState = Dac_Set_Delay( DAC_CAL_STEP_DELAY_US );
            }
            else
            {
                /* Previous step failed */
            }

            if( DAC_REQUEST_OK == retState )
            {
                retState = Dac_Get_CalibrationFlag( periphId, channelId, &calFlag );
            }
            else
            {
                /* Previous step failed */
            }

            if( ( DAC_REQUEST_OK  == retState ) &&
                ( DAC_FLAG_ACTIVE == calFlag  )    )
            {
                /* Offset is compensated - lower trimming value is tried */
                trimValue -= trimStep;
            }
            else if( DAC_REQUEST_OK == retState )
            {
                /* Offset is not compensated - higher trimming value is tried */
                trimValue += trimStep;
            }
            else
            {
                /* Calibration step failed */
            }
        }

        /* --- Final value: the found value or one step above --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Write_TrimValue( periphId, channelId, trimValue );
        }
        else
        {
            /* Previous step failed */
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_Delay( DAC_CAL_STEP_DELAY_US );
        }
        else
        {
            /* Previous step failed */
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Get_CalibrationFlag( periphId, channelId, &calFlag );
        }
        else
        {
            /* Previous step failed */
        }

        if( ( DAC_REQUEST_OK     == retState  ) &&
            ( DAC_FLAG_INACTIVE  == calFlag   ) &&
            ( DAC_TRIM_VALUE_MAX  > trimValue )    )
        {
            trimValue ++;
            retState = Dac_Write_TrimValue( periphId, channelId, trimValue );
        }
        else
        {
            /* Calibration failed or the found value is final */
        }

        /* --- Normal operation and original output mode are restored --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Write_CalibrationMode( periphId, channelId, LL_DAC_MODE_NORMAL_OPERATION );
        }
        else
        {
            /* Previous step failed */
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Write_ChannelMode( periphId, channelId, oldBuffer, oldConnect, oldShMode );
        }
        else
        {
            /* Previous step failed */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the channel offset trimming value (e.g. value stored from a previous calibration)
 *
 * \pre   The channel must be disabled. Otherwise \ref DAC_REQUEST_ERROR is returned and no
 *        register is modified.
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param trimValue [in]: Trimming value, 0 - \ref DAC_TRIM_VALUE_MAX
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_TrimValue( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TrimValue_t trimValue )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId ) &&
        ( DAC_TRIM_VALUE_MAX                    >= trimValue )    )
    {
        retState = Dac_Check_ChannelDisabled( periphId, channelId );

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Write_TrimValue( periphId, channelId, trimValue );
        }
        else
        {
            /* Channel is enabled, configuration change is not allowed */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the channel offset trimming value
 *
 * \param periphId   [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId  [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param trimValue [out]: Pointer to store the trimming value. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_TrimValue( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TrimValue_t * const trimValue )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId ) &&
        ( DAC_NULL_PTR                          != trimValue )    )
    {
        *trimValue = (dac_TrimValue_t)LL_DAC_GetTrimmingValue( dac_PeriphConf[ periphId ].PeriphReg, dac_LlChannelLut[ channelId ] );
        retState   = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ------------------------------ Data output ------------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Writes the channel output value to the data holding register of the channel format
 *
 * The value is moved to the output (DORx) one APB clock cycle after the write if the trigger
 * is disabled, otherwise on the next trigger.
 *
 * \note  Read-back verifies the data holding register (bits used by the format), not the output
 *        register - the output is updated by HW later when a trigger is used.
 *
 * \pre   No DMA transfer may be running on the channel and data must be within the range of
 *        the channel data format. Otherwise \ref DAC_REQUEST_ERROR is returned and no register
 *        is modified.
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param data      [in]: Output value in the channel data format (\ref dac_DataFormat_t)
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_Data( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Data_t data )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;
    uint32_t           regAddr  = 0u;

    if( ( DAC_PERIPH_CNT                         > periphId                                           ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId                                          ) &&
        ( DAC_FUNCTION_ACTIVE                   != dac_XferContext[ periphId ][ channelId ].XferState )    )
    {
        const dac_DataFormat_t dataFormat = dac_ShadowDataFormat[ periphId ][ channelId ];

        retState = Dac_Check_DataValue( dataFormat, data );

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Get_DataRegAddr( periphId, channelId, &regAddr );
        }
        else
        {
            /* Previous step failed */
        }

        if( DAC_REQUEST_OK == retState )
        {
            volatile uint32_t * const dataReg  = (volatile uint32_t *)(uintptr_t)regAddr;
            const uint32_t            dataMask = dac_DataMaskLut[ dataFormat ];

            WRITE_REG( *dataReg, (uint32_t)data );

            for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t dataRegValue = READ_BIT( *dataReg, dataMask );

                if( ( (uint32_t)data & dataMask ) == dataRegValue )
                {
                    retState = DAC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Data has not yet been applied, keep return state as error */
                    retState = DAC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Previous step failed */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes output values of both channels at once (dual data holding register)
 *
 * \note  Both outputs are updated simultaneously (no trigger, or the same trigger on both
 *        channels, see Dac_Set_DualSwTrigger()).
 *
 * \pre   The peripheral must have two channels (not DAC2), both channels must use the same data
 *        format, no DMA transfer may be running on any channel and data must be within the range
 *        of the format. Otherwise
 *        \ref DAC_REQUEST_ERROR is returned and no register is modified.
 *
 * \param periphId [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param dataCh1  [in]: Output value of channel 1 in the channel data format
 * \param dataCh2  [in]: Output value of channel 2 in the channel data format
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_DualData( dac_PeriphId_t periphId, dac_Data_t dataCh1, dac_Data_t dataCh2 )
{
    dac_RequestState_t  retState = DAC_REQUEST_ERROR;
    volatile uint32_t * dataReg  = DAC_NULL_PTR;

    if( ( DAC_PERIPH_CNT   > periphId                              ) &&
        ( DAC_CHANNEL_CNT == dac_PeriphConf[ periphId ].ChannelCnt )    )
    {
        const dac_DataFormat_t dataFormat = dac_ShadowDataFormat[ periphId ][ DAC_CHANNEL_1 ];
        const dac_RequestState_t ch1State = Dac_Check_DataValue( dataFormat, dataCh1 );
        const dac_RequestState_t ch2State = Dac_Check_DataValue( dataFormat, dataCh2 );

        if( ( dataFormat          == dac_ShadowDataFormat[ periphId ][ DAC_CHANNEL_2 ]      ) &&
            ( DAC_REQUEST_OK      == ch1State                                               ) &&
            ( DAC_REQUEST_OK      == ch2State                                               ) &&
            ( DAC_FUNCTION_ACTIVE != dac_XferContext[ periphId ][ DAC_CHANNEL_1 ].XferState ) &&
            ( DAC_FUNCTION_ACTIVE != dac_XferContext[ periphId ][ DAC_CHANNEL_2 ].XferState )    )
        {
            retState = Dac_Get_DualDataReg( periphId, dataFormat, &dataReg );
        }
        else
        {
            /* Different formats, data out of range or DMA transfer running */
            retState = DAC_REQUEST_ERROR;
        }

        if( DAC_REQUEST_OK == retState )
        {
            const uint32_t dualShift = dac_DualShiftLut[ dataFormat ];
            const uint32_t dataMask  = dac_DataMaskLut[ dataFormat ] | ( dac_DataMaskLut[ dataFormat ] << dualShift );
            const uint32_t dualData  = (uint32_t)dataCh1 | ( (uint32_t)dataCh2 << dualShift );

            WRITE_REG( *dataReg, dualData );

            for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t dataRegValue = READ_BIT( *dataReg, dataMask );

                if( ( dualData & dataMask ) == dataRegValue )
                {
                    retState = DAC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Data has not yet been applied, keep return state as error */
                    retState = DAC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Previous step failed */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the actual channel output value (data output register DORx, 12-bit right aligned)
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param data     [out]: Pointer to store the output value (0 - \ref DAC_DATA_12BIT_MAX). Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_OutputData( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Data_t * const data )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId ) &&
        ( DAC_NULL_PTR                          != data      )    )
    {
        *data    = (dac_Data_t)LL_DAC_RetrieveOutputData( dac_PeriphConf[ periphId ].PeriphReg, dac_LlChannelLut[ channelId ] );
        retState = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Generates software trigger of a channel (SWTRIGx)
 *
 * \note  SWTRIGx is cleared by HW when the data holding register is moved to the output, so the
 *        write is not verified by read-back.
 *
 * \pre   The channel must be enabled with trigger DAC_TRIGGER_SOFTWARE. Otherwise
 *        \ref DAC_REQUEST_ERROR is returned and no register is modified.
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_SwTrigger( dac_PeriphId_t periphId, dac_ChannelId_t channelId )
{
    dac_RequestState_t  retState     = DAC_REQUEST_ERROR;
    dac_TriggerId_t     triggerSrc   = DAC_TRIGGER_CNT;
    dac_FunctionState_t channelState = DAC_FUNCTION_INACTIVE;

    if( ( DAC_PERIPH_CNT                        > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt > channelId )    )
    {
        retState = Dac_Get_TriggerSrc( periphId, channelId, &triggerSrc );

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Get_ChannelState( periphId, channelId, &channelState );
        }
        else
        {
            /* Previous step failed */
        }

        if( ( DAC_REQUEST_OK       == retState     ) &&
            ( DAC_TRIGGER_SOFTWARE == triggerSrc   ) &&
            ( DAC_FUNCTION_ACTIVE  == channelState )    )
        {
            LL_DAC_TrigSWConversion( dac_PeriphConf[ periphId ].PeriphReg, dac_LlChannelLut[ channelId ] );
            retState = DAC_REQUEST_OK;
        }
        else
        {
            /* Channel is disabled or software trigger is not selected */
            retState = DAC_REQUEST_ERROR;
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Generates software trigger of both channels at once (simultaneous output update)
 *
 * \note  SWTRIGx bits are cleared by HW, the write is not verified by read-back.
 *
 * \pre   The peripheral must have two channels (not DAC2), both channels must be enabled with
 *        trigger DAC_TRIGGER_SOFTWARE. Otherwise
 *        \ref DAC_REQUEST_ERROR is returned and no register is modified.
 *
 * \param periphId [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_DualSwTrigger( dac_PeriphId_t periphId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT   > periphId                              ) &&
        ( DAC_CHANNEL_CNT == dac_PeriphConf[ periphId ].ChannelCnt )    )
    {
        retState = DAC_REQUEST_OK;

        for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; ( DAC_CHANNEL_CNT > channelIdx ) &&
                                                         ( DAC_REQUEST_OK == retState )  ; channelIdx ++ )
        {
            dac_TriggerId_t     triggerSrc   = DAC_TRIGGER_CNT;
            dac_FunctionState_t channelState = DAC_FUNCTION_INACTIVE;

            retState = Dac_Get_TriggerSrc( periphId, channelIdx, &triggerSrc );

            if( DAC_REQUEST_OK == retState )
            {
                retState = Dac_Get_ChannelState( periphId, channelIdx, &channelState );
            }
            else
            {
                /* Previous step failed */
            }

            if( ( DAC_REQUEST_OK       != retState     ) ||
                ( DAC_TRIGGER_SOFTWARE != triggerSrc   ) ||
                ( DAC_FUNCTION_ACTIVE  != channelState )    )
            {
                /* Channel is disabled or software trigger is not selected */
                retState = DAC_REQUEST_ERROR;
            }
            else
            {
                /* Channel is ready for software trigger */
            }
        }

        if( DAC_REQUEST_OK == retState )
        {
            LL_DAC_TrigSWConversion( dac_PeriphConf[ periphId ].PeriphReg, dac_LlChannelLut[ DAC_CHANNEL_1 ] | dac_LlChannelLut[ DAC_CHANNEL_2 ] );
        }
        else
        {
            /* Previous step failed */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Converts a voltage to the output value in the channel data format
 *
 * data = outVoltage * full scale / refVoltage, rounded to the nearest value, aligned by the
 * format (12-bit left aligned value is shifted to bits 15:4).
 *
 * \param periphId   [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId  [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param refVoltage [in]: DAC reference voltage (VREF+) in mV, must be > 0
 * \param outVoltage [in]: Required output voltage in mV, must be <= refVoltage
 * \param data      [out]: Pointer to store the output value. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_DataFromVoltage( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Voltage_mV_t refVoltage, dac_Voltage_mV_t outVoltage, dac_Data_t * const data )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId   ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId  ) &&
        ( 0u                                     < refVoltage ) &&
        ( refVoltage                            >= outVoltage ) &&
        ( DAC_NULL_PTR                          != data       )    )
    {
        const dac_DataFormat_t dataFormat = dac_ShadowDataFormat[ periphId ][ channelId ];
        const uint32_t         rawValue   = DAC_DIV_ROUND( (uint32_t)outVoltage * dac_FullScaleLut[ dataFormat ], (uint32_t)refVoltage );

        *data    = (dac_Data_t)( rawValue << dac_DataShiftLut[ dataFormat ] );
        retState = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the actual channel output voltage calculated from the output register DORx
 *
 * outVoltage = DORx * refVoltage / 4095, rounded to the nearest value.
 *
 * \param periphId    [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId   [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param refVoltage  [in]: DAC reference voltage (VREF+) in mV, must be > 0
 * \param outVoltage [out]: Pointer to store the output voltage in mV. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_OutputVoltage( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Voltage_mV_t refVoltage, dac_Voltage_mV_t * const outVoltage )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;
    dac_Data_t         outData  = 0u;

    if( ( 0u            < refVoltage ) &&
        ( DAC_NULL_PTR != outVoltage )    )
    {
        retState = Dac_Get_OutputData( periphId, channelId, &outData );

        if( DAC_REQUEST_OK == retState )
        {
            *outVoltage = (dac_Voltage_mV_t)DAC_DIV_ROUND( (uint32_t)outData * (uint32_t)refVoltage, DAC_OUTPUT_FULL_SCALE );
        }
        else
        {
            /* Output value is not available */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Sets the channel output voltage (Dac_Get_DataFromVoltage() and Dac_Set_Data())
 *
 * \param periphId   [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId  [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param refVoltage [in]: DAC reference voltage (VREF+) in mV, must be > 0
 * \param outVoltage [in]: Required output voltage in mV, must be <= refVoltage
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_Voltage( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Voltage_mV_t refVoltage, dac_Voltage_mV_t outVoltage )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;
    dac_Data_t         outData  = 0u;

    retState = Dac_Get_DataFromVoltage( periphId, channelId, refVoltage, outVoltage, &outData );

    if( DAC_REQUEST_OK == retState )
    {
        retState = Dac_Set_Data( periphId, channelId, outData );
    }
    else
    {
        /* Voltage can not be converted */
    }

    return ( retState );
}


/**
 * \brief Returns state of a DAC channel flag
 *
 * \note  STM32H7 DAC has no channel ready flag - \ref DAC_FLAG_READY returns \ref DAC_REQUEST_ERROR.
 *
 * \param periphId   [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId  [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param flagId     [in]: Flag identification, value from \ref dac_FlagId_t
 * \param flagState [out]: Pointer to store the flag state. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_Flag( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_FlagId_t flagId, dac_FlagState_t * const flagState )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId                         ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId                        ) &&
        ( DAC_FLAG_CNT                           > flagId                           ) &&
        ( DAC_FLAG_UNSUPPORTED                  != dac_FlagLut[ flagId ][ channelId ] ) &&
        ( DAC_NULL_PTR                          != flagState                        )    )
    {
        const uint32_t flagReg = READ_BIT( dac_PeriphConf[ periphId ].PeriphReg->SR, dac_FlagLut[ flagId ][ channelId ] );

        if( 0u != flagReg )
        {
            *flagState = DAC_FLAG_ACTIVE;
        }
        else
        {
            *flagState = DAC_FLAG_INACTIVE;
        }

        retState = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Clears a DAC channel flag
 *
 * \note  Only DAC_FLAG_DMA_UNDERRUN can be cleared (other flags are HW status flags). The
 *        underrun flag can be set by HW at any time, so the clear is not verified by read-back.
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param flagId    [in]: Flag identification, value from \ref dac_FlagId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Clear_Flag( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_FlagId_t flagId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId ) &&
        ( DAC_FLAG_DMA_UNDERRUN                 == flagId    )    )
    {
        /* DMAUDRx is cleared by writing 1, other SR bits are read only */
        WRITE_REG( dac_PeriphConf[ periphId ].PeriphReg->SR, dac_FlagLut[ flagId ][ channelId ] );
        retState = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* --------------------------- DMA data transfer ---------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Configures the channel data transfer (direct output or DMA buffer output)
 *
 * Resources of a previous DMA configuration are released, for DAC_TRANSFER_MODE_DMA the DMA
 * channel and the DAC interrupt are initialized (the transfer is started by Dac_Set_DmaStart()).
 *
 * \pre   No DMA transfer may be running on the channel. DAC_TRANSFER_MODE_DMA requires a hardware
 *        trigger selected for the channel (Dac_Set_TriggerSrc()). Otherwise \ref DAC_REQUEST_ERROR
 *        is returned and no register is modified.
 *
 * \param periphId   [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId  [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param dataConfig [in]: Pointer to data transfer configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_DataConfig( dac_PeriphId_t periphId, dac_ChannelId_t channelId, const dac_DataConfig_t * const dataConfig )
{
    dac_RequestState_t retState   = DAC_REQUEST_ERROR;
    dac_TriggerId_t    triggerSrc = DAC_TRIGGER_CNT;

    if( ( DAC_PERIPH_CNT                         > periphId   ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId  ) &&
        ( DAC_NULL_PTR                          != dataConfig )    )
    {
        dac_XferContext_t * const xferCtx = &dac_XferContext[ periphId ][ channelId ];

        retState = Dac_Get_TriggerSrc( periphId, channelId, &triggerSrc );

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Check_DataConfig( periphId, channelId, triggerSrc, dataConfig );
        }
        else
        {
            /* Previous step failed */
        }

        if( ( DAC_REQUEST_OK      == retState           ) &&
            ( DAC_FUNCTION_ACTIVE == xferCtx->XferState )    )
        {
            /* DMA transfer is running, configuration change is not allowed */
            retState = DAC_REQUEST_ERROR;
        }
        else
        {
            /* Configuration is valid and no transfer is running */
        }

        /* --- Resources of the previous configuration are released --- */
        if( ( DAC_REQUEST_OK      == retState           ) &&
            ( DAC_FUNCTION_ACTIVE == xferCtx->InitState )    )
        {
            retState = Dac_Dma_Deinit( periphId, channelId );

            if( DAC_REQUEST_OK == retState )
            {
                xferCtx->InitState = DAC_FUNCTION_INACTIVE;
            }
            else
            {
                /* DMA resources could not be released */
            }
        }
        else
        {
            /* Configuration is invalid or DMA transfer is not initialized */
        }

        /* --- New configuration --- */
        if( DAC_REQUEST_OK == retState )
        {
            xferCtx->Config = *dataConfig;

            if( DAC_TRANSFER_MODE_DMA == dataConfig->TransferMode )
            {
                retState = Dac_Dma_Init( periphId, channelId );

                if( DAC_REQUEST_OK == retState )
                {
                    xferCtx->InitState = DAC_FUNCTION_ACTIVE;
                }
                else
                {
                    /* DMA initialization failed - channel falls back to direct transfer */
                    xferCtx->Config.TransferMode = DAC_TRANSFER_MODE_DIRECT;
                }
            }
            else
            {
                /* Direct transfer needs no resources */
            }
        }
        else
        {
            /* Previous step failed */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the channel data transfer configuration
 *
 * \param periphId    [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId   [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param dataConfig [out]: Pointer to store the data transfer configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_DataConfig( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_DataConfig_t * const dataConfig )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId   ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId  ) &&
        ( DAC_NULL_PTR                          != dataConfig )    )
    {
        *dataConfig = dac_XferContext[ periphId ][ channelId ].Config;
        retState    = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts the DMA buffer output of a channel
 *
 * The DMA channel is armed for the whole buffer, DAC DMA requests and the DMA underrun
 * interrupt are enabled. Buffer items are output on every hardware trigger.
 *
 * \pre   The channel must be enabled, DMA transfer must be initialized (DAC_TRANSFER_MODE_DMA)
 *        and not running. Otherwise \ref DAC_REQUEST_ERROR is returned and no register is modified.
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_DmaStart( dac_PeriphId_t periphId, dac_ChannelId_t channelId )
{
    dac_RequestState_t  retState     = DAC_REQUEST_ERROR;
    dac_FunctionState_t channelState = DAC_FUNCTION_INACTIVE;

    if( ( DAC_PERIPH_CNT                        > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt > channelId )    )
    {
        dac_XferContext_t * const xferCtx = &dac_XferContext[ periphId ][ channelId ];

        retState = Dac_Get_ChannelState( periphId, channelId, &channelState );

        if( ( DAC_REQUEST_OK        == retState                     ) &&
            ( DAC_FUNCTION_ACTIVE   == channelState                 ) &&
            ( DAC_FUNCTION_ACTIVE   == xferCtx->InitState           ) &&
            ( DAC_FUNCTION_INACTIVE == xferCtx->XferState           ) &&
            ( DAC_TRANSFER_MODE_DMA == xferCtx->Config.TransferMode )    )
        {
            /* State is set before the start - first DMA event may come immediately */
            xferCtx->XferState = DAC_FUNCTION_ACTIVE;

            retState = Dac_Dma_Start( periphId, channelId );

            if( DAC_REQUEST_OK != retState )
            {
                /* Start failed, partially started transfer is stopped */
                (void)Dac_Dma_Stop( periphId, channelId );
                xferCtx->XferState = DAC_FUNCTION_INACTIVE;
            }
            else
            {
                /* Transfer is running */
            }
        }
        else
        {
            /* Channel is disabled, DMA transfer is not initialized or already running */
            retState = DAC_REQUEST_ERROR;
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stops the DMA buffer output of a channel
 *
 * DAC DMA requests, the DMA underrun interrupt and the DMA channel are disabled. The last
 * output value is kept.
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems (also if no transfer was running). Otherwise returns
 *         \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_DmaStop( dac_PeriphId_t periphId, dac_ChannelId_t channelId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                        > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt > channelId )    )
    {
        dac_XferContext_t * const xferCtx = &dac_XferContext[ periphId ][ channelId ];

        if( DAC_FUNCTION_ACTIVE == xferCtx->InitState )
        {
            retState = Dac_Dma_Stop( periphId, channelId );
        }
        else
        {
            /* DMA transfer is not initialized - nothing to stop */
            retState = DAC_REQUEST_OK;
        }

        if( DAC_REQUEST_OK == retState )
        {
            xferCtx->XferState = DAC_FUNCTION_INACTIVE;
        }
        else
        {
            /* Transfer could not be stopped, state is kept */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ------------------ Private interface (see Dac.h) ------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Returns CMSIS register pointer of a DAC peripheral (for DMA / interrupt handlers)
 *
 * \param periphId   [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param periphReg [out]: Pointer to store the register pointer. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_PeriphReg( dac_PeriphId_t periphId, DAC_TypeDef ** const periphReg )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_NULL_PTR   != periphReg )    )
    {
        *periphReg = dac_PeriphConf[ periphId ].PeriphReg;
        retState   = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns LL channel identification of a DAC channel (for DMA / interrupt handlers)
 *
 * \param channelId  [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param llChannel [out]: Pointer to store LL_DAC_CHANNEL_x. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_LlChannel( dac_ChannelId_t channelId, uint32_t * const llChannel )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_CHANNEL_CNT  > channelId ) &&
        ( DAC_NULL_PTR    != llChannel )    )
    {
        *llChannel = dac_LlChannelLut[ channelId ];
        retState   = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns data transfer context of a DAC channel (for DMA / interrupt handlers)
 *
 * \param periphId     [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId    [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param xferContext [out]: Pointer to store the context pointer. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_XferContext( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_XferContext_t ** const xferContext )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId    ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId   ) &&
        ( DAC_NULL_PTR                          != xferContext )    )
    {
        *xferContext = &dac_XferContext[ periphId ][ channelId ];
        retState     = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns address of the data holding register selected by the channel data format
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param regAddr  [out]: Pointer to store the register address. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_DataRegAddr( dac_PeriphId_t periphId, dac_ChannelId_t channelId, uint32_t * const regAddr )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId ) &&
        ( DAC_NULL_PTR                          != regAddr   )    )
    {
        const dac_DataFormat_t dataFormat = dac_ShadowDataFormat[ periphId ][ channelId ];

        *regAddr = LL_DAC_DMA_GetRegAddr( dac_PeriphConf[ periphId ].PeriphReg, dac_LlChannelLut[ channelId ], dac_DataRegLut[ dataFormat ] );
        retState = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reports half of the buffer output (HalfTransferCallback)
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_XferHalf( dac_PeriphId_t periphId, dac_ChannelId_t channelId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                        > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt > channelId )    )
    {
        dac_Callback_t * const callback = dac_XferContext[ periphId ][ channelId ].Config.HalfTransferCallback;

        if( DAC_NULL_PTR != callback )
        {
            callback();
        }
        else
        {
            /* Event is not reported */
        }

        retState = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Handles the end of the buffer: circular output continues (DMA channel in circular
 *        mode, see Dac_Dma.c), one shot output is stopped (Dac_Set_DmaStop()); then
 *        TransferCompleteCallback is called (the transfer can be restarted from the callback)
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_XferDone( dac_PeriphId_t periphId, dac_ChannelId_t channelId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                        > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt > channelId )    )
    {
        const dac_XferContext_t * const xferCtx  = &dac_XferContext[ periphId ][ channelId ];
        dac_Callback_t * const          callback = xferCtx->Config.TransferCompleteCallback;

        if( DAC_BUFFER_MODE_CIRCULAR == xferCtx->Config.BufferMode )
        {
            /* Output continues from the buffer start */
            retState = DAC_REQUEST_OK;
        }
        else
        {
            /* One shot - DMA requests and transfer are stopped */
            retState = Dac_Set_DmaStop( periphId, channelId );
        }

        if( DAC_NULL_PTR != callback )
        {
            callback();
        }
        else
        {
            /* Event is not reported */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stops the channel DMA transfer and reports a data transfer error (ErrorCallback)
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param errorId   [in]: Error identification, value from \ref dac_ErrorId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_XferError( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_ErrorId_t errorId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                        > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt > channelId ) &&
        ( DAC_ERROR_CNT                         > errorId   )    )
    {
        dac_ErrCallback_t * const callback = dac_XferContext[ periphId ][ channelId ].Config.ErrorCallback;

        /* Transfer is stopped - DMA underrun stops DAC DMA requests, DMA transfer error disables the channel */
        retState = Dac_Set_DmaStop( periphId, channelId );

        if( DAC_NULL_PTR != callback )
        {
            callback( errorId );
        }
        else
        {
            /* Error is not reported */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Checks that a DAC channel is disabled (ENx = 0)
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Returns \ref DAC_REQUEST_OK if the channel is disabled. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Check_ChannelDisabled( dac_PeriphId_t periphId, dac_ChannelId_t channelId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                        > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt > channelId )    )
    {
        const uint32_t enabled = LL_DAC_IsEnabled( dac_PeriphConf[ periphId ].PeriphReg, dac_LlChannelLut[ channelId ] );

        if( 0u == enabled )
        {
            retState = DAC_REQUEST_OK;
        }
        else
        {
            /* Channel is enabled */
            retState = DAC_REQUEST_ERROR;
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that both channels of a DAC peripheral are disabled
 *
 * \param periphId [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 *
 * \return Returns \ref DAC_REQUEST_OK if all channels are disabled. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Check_PeriphDisabled( dac_PeriphId_t periphId )
{
    dac_RequestState_t retState = DAC_REQUEST_OK;

    for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; ( dac_PeriphConf[ periphId ].ChannelCnt > channelIdx ) &&
                                                     ( DAC_REQUEST_OK == retState )                        ; channelIdx ++ )
    {
        retState = Dac_Check_ChannelDisabled( periphId, channelIdx );
    }

    return ( retState );
}


/**
 * \brief Checks that all channels of all DAC peripherals are disabled
 *
 * \return Returns \ref DAC_REQUEST_OK if all channels are disabled. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Check_AllPeriphsDisabled( void )
{
    dac_RequestState_t retState = DAC_REQUEST_OK;

    for( dac_PeriphId_t periphIdx = DAC_PERIPH_1; ( DAC_PERIPH_CNT > periphIdx ) &&
                                                  ( DAC_REQUEST_OK == retState ); periphIdx ++ )
    {
        retState = Dac_Check_PeriphDisabled( periphIdx );
    }

    return ( retState );
}


/**
 * \brief Checks channel configuration (all enumerations, value ranges, data transfer)
 *
 * \param periphId      [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelConfig [in]: Pointer to channel configuration. Must not be NULL.
 *
 * \return Returns \ref DAC_REQUEST_OK if the configuration is valid. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Check_ChannelConfig( dac_PeriphId_t periphId, const dac_ChannelConfig_t * const channelConfig )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId                                                  ) &&
        ( DAC_NULL_PTR                          != channelConfig                                             ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelConfig->ChannelId                                  ) &&
        ( DAC_OUTPUT_CNT                         > channelConfig->OutputMode                                 ) &&
        ( DAC_TRIGGER_CNT                        > channelConfig->TriggerId                                  ) &&
        ( DAC_TRIG_UNSUPPORTED                  != dac_TriggerSrcLut[ periphId ][ channelConfig->TriggerId ] ) &&
        ( DAC_DATA_FORMAT_CNT                    > channelConfig->DataFormat                                 ) &&
        ( DAC_WAVE_CNT                           > channelConfig->WaveMode                                   ) &&
        ( DAC_WAVE_AMPLITUDE_CNT                 > channelConfig->WaveAmplitude                              )    )
    {
        retState = Dac_Check_OutputMode( periphId, channelConfig->OutputMode );

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Check_SampleHoldConfig( &channelConfig->SampleHold );
        }
        else
        {
            /* Previous step failed */
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Check_DataValue( channelConfig->DataFormat, channelConfig->InitValue );
        }
        else
        {
            /* Previous step failed */
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Check_DataConfig( periphId, channelConfig->ChannelId, channelConfig->TriggerId, &channelConfig->DataConfig );
        }
        else
        {
            /* Previous step failed */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks sample and hold configuration (state and timing ranges)
 *
 * \param shConfig [in]: Pointer to sample and hold configuration. Must not be NULL.
 *
 * \return Returns \ref DAC_REQUEST_OK if the configuration is valid. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Check_SampleHoldConfig( const dac_SampleHoldConfig_t * const shConfig )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_NULL_PTR            != shConfig                                ) &&
        ( ( DAC_FUNCTION_ACTIVE   == shConfig->State ) ||
          ( DAC_FUNCTION_INACTIVE == shConfig->State )                       ) &&
        ( DAC_SH_SAMPLE_TIME_MAX  >= shConfig->SampleTime                    ) &&
        ( DAC_SH_HOLD_TIME_MAX    >= shConfig->HoldTime                      )    )
    {
        /* RefreshTime covers the whole TREFRESHx range by its type (8 bits) */
        retState = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that a data value is within the range of a data format
 *
 * \param dataFormat [in]: Data format, value from \ref dac_DataFormat_t
 * \param data       [in]: Data value
 *
 * \return Returns \ref DAC_REQUEST_OK if the value is valid. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Check_DataValue( dac_DataFormat_t dataFormat, dac_Data_t data )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_DATA_FORMAT_CNT           > dataFormat     ) &&
        ( dac_DataMaxLut[ dataFormat ] >= (uint32_t)data )    )
    {
        retState = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that a trigger generates DAC DMA requests (hardware trigger)
 *
 * \param triggerSrc [in]: Trigger, value from \ref dac_TriggerId_t
 *
 * \return Returns \ref DAC_REQUEST_OK for hardware triggers. Otherwise (no trigger, software
 *         trigger) returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Check_DmaTrigger( dac_TriggerId_t triggerSrc )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_TRIGGER_CNT       > triggerSrc ) &&
        ( DAC_TRIGGER_NONE     != triggerSrc ) &&
        ( DAC_TRIGGER_SOFTWARE != triggerSrc )    )
    {
        retState = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that an output mode is available on a DAC peripheral
 *
 * \param periphId   [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param outputMode [in]: Output mode, value from \ref dac_OutputMode_t
 *
 * \return Returns \ref DAC_REQUEST_OK for any output mode of peripherals with buffered external
 *         outputs (DAC1 / DAC2 of STM32H7) and for \ref DAC_OUTPUT_INTERNAL of peripherals with
 *         internal outputs only. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Check_OutputMode( dac_PeriphId_t periphId, dac_OutputMode_t outputMode )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT > periphId   ) &&
        ( DAC_OUTPUT_CNT > outputMode )    )
    {
        if( DAC_FUNCTION_ACTIVE == dac_PeriphConf[ periphId ].ExternalOut )
        {
            retState = DAC_REQUEST_OK;
        }
        else if( DAC_OUTPUT_INTERNAL == outputMode )
        {
            retState = DAC_REQUEST_OK;
        }
        else
        {
            /* Internal outputs only - no pin and no buffer */
            retState = DAC_REQUEST_ERROR;
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks channel data transfer configuration
 *
 * \param periphId   [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId  [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param triggerSrc [in]: Trigger used by the channel, value from \ref dac_TriggerId_t
 * \param dataConfig [in]: Pointer to data transfer configuration. Must not be NULL.
 *
 * \return Returns \ref DAC_REQUEST_OK if the configuration is valid. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Check_DataConfig( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TriggerId_t triggerSrc, const dac_DataConfig_t * const dataConfig )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_NULL_PTR          != dataConfig               ) &&
        ( DAC_TRANSFER_MODE_CNT  > dataConfig->TransferMode )    )
    {
        if( DAC_TRANSFER_MODE_DMA == dataConfig->TransferMode )
        {
            retState = Dac_Check_DmaTrigger( triggerSrc );

            if( DAC_REQUEST_OK == retState )
            {
                retState = Dac_Dma_Check_Config( periphId, channelId, dataConfig );
            }
            else
            {
                /* DMA requests are not generated without hardware trigger */
            }
        }
        else
        {
            /* Direct transfer has no further parameters */
            retState = DAC_REQUEST_OK;
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures the channel output pin as analog without pull (GPIO port clock is enabled by Gpio_Init())
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise (also for peripherals with internal outputs
 *         only) returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Set_PinAnalog( dac_PeriphId_t periphId, dac_ChannelId_t channelId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId                               ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId                              ) &&
        ( DAC_FUNCTION_ACTIVE                   == dac_PeriphConf[ periphId ].ExternalOut )    )
    {
        gpio_Config_t gpioConfig;

        gpioConfig.PortId         = dac_GpioConfig[ periphId ][ channelId ].PortId;
        gpioConfig.PinId          = dac_GpioConfig[ periphId ][ channelId ].PinId;
        gpioConfig.PinMode        = GPIO_PIN_MODE_ANALOG;
        gpioConfig.PinPull        = GPIO_PIN_PULL_NONE;
        gpioConfig.PinSpeed       = GPIO_PIN_SPEED_LOW;
        gpioConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
        gpioConfig.PinAltFunction = GPIO_ALT_FUNC_0;
        gpioConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

        /* Port clock activation, pin configuration and read-back verification is done by Gpio_Init() */
        const gpio_RequestState_t gpioState = Gpio_Init( &gpioConfig );

        if( GPIO_REQUEST_OK == gpioState )
        {
            retState = DAC_REQUEST_OK;
        }
        else
        {
            retState = DAC_REQUEST_ERROR;
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the channel offset trimming value (OTRIMx) with read-back verification
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param trimValue [in]: Trimming value, 0 - \ref DAC_TRIM_VALUE_MAX
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Write_TrimValue( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TrimValue_t trimValue )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                         > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt  > channelId ) &&
        ( DAC_TRIM_VALUE_MAX                    >= trimValue )    )
    {
        DAC_TypeDef * const periphReg = dac_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      llChannel = dac_LlChannelLut[ channelId ];

        LL_DAC_SetTrimmingValue( periphReg, llChannel, (uint32_t)trimValue );

        for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t trimReg = LL_DAC_GetTrimmingValue( periphReg, llChannel );

            if( (uint32_t)trimValue == trimReg )
            {
                retState = DAC_REQUEST_OK;
                break;
            }
            else
            {
                /* Trimming value has not yet been applied, keep return state as error */
                retState = DAC_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Switches the channel between normal operation and calibration mode (CENx)
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param llMode    [in]: LL_DAC_MODE_NORMAL_OPERATION or LL_DAC_MODE_CALIBRATION
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Write_CalibrationMode( dac_PeriphId_t periphId, dac_ChannelId_t channelId, uint32_t llMode )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                        > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt > channelId )    )
    {
        DAC_TypeDef * const periphReg = dac_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      llChannel = dac_LlChannelLut[ channelId ];

        LL_DAC_SetMode( periphReg, llChannel, llMode );

        for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t modeReg = LL_DAC_GetMode( periphReg, llChannel );

            if( llMode == modeReg )
            {
                retState = DAC_REQUEST_OK;
                break;
            }
            else
            {
                /* Mode has not yet been applied, keep return state as error */
                retState = DAC_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the channel mode (MODEx: output buffer, output connection, sample and hold) with
 *        read-back verification
 *
 * \pre   The channel is disabled (checked by the caller).
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param llBuffer  [in]: LL_DAC_OUTPUT_BUFFER_x
 * \param llConnect [in]: LL_DAC_OUTPUT_CONNECT_x
 * \param llShMode  [in]: LL_DAC_OUTPUT_MODE_x
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Write_ChannelMode( dac_PeriphId_t periphId, dac_ChannelId_t channelId, uint32_t llBuffer, uint32_t llConnect, uint32_t llShMode )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT                        > periphId  ) &&
        ( dac_PeriphConf[ periphId ].ChannelCnt > channelId )    )
    {
        DAC_TypeDef * const periphReg = dac_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      llChannel = dac_LlChannelLut[ channelId ];

        LL_DAC_SetOutputBuffer( periphReg, llChannel, llBuffer );
        LL_DAC_SetOutputConnection( periphReg, llChannel, llConnect );
        LL_DAC_SetOutputMode( periphReg, llChannel, llShMode );

        for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t bufferReg  = LL_DAC_GetOutputBuffer( periphReg, llChannel );
            const uint32_t connectReg = LL_DAC_GetOutputConnection( periphReg, llChannel );
            const uint32_t shModeReg  = LL_DAC_GetOutputMode( periphReg, llChannel );

            if( ( llBuffer  == bufferReg  ) &&
                ( llConnect == connectReg ) &&
                ( llShMode  == shModeReg  )    )
            {
                retState = DAC_REQUEST_OK;
                break;
            }
            else
            {
                /* Mode has not yet been applied, keep return state as error */
                retState = DAC_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the channel offset calibration flag (CAL_FLAGx)
 *
 * \param periphId   [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId  [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param flagState [out]: Pointer to store the flag state. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Get_CalibrationFlag( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_FlagState_t * const flagState )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    retState = Dac_Get_Flag( periphId, channelId, DAC_FLAG_CALIBRATION, flagState );

    return ( retState );
}


/**
 * \brief Returns the dual data holding register of a data format
 *
 * \param periphId    [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param dataFormat  [in]: Data format, value from \ref dac_DataFormat_t
 * \param dataReg    [out]: Pointer to store the register pointer. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Get_DualDataReg( dac_PeriphId_t periphId, dac_DataFormat_t dataFormat, volatile uint32_t ** const dataReg )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT       > periphId   ) &&
        ( DAC_DATA_FORMAT_CNT  > dataFormat ) &&
        ( DAC_NULL_PTR        != dataReg    )    )
    {
        DAC_TypeDef * const periphReg = dac_PeriphConf[ periphId ].PeriphReg;

        if( DAC_DATA_FORMAT_12BIT_RIGHT == dataFormat )
        {
            *dataReg = &periphReg->DHR12RD;
        }
        else if( DAC_DATA_FORMAT_12BIT_LEFT == dataFormat )
        {
            *dataReg = &periphReg->DHR12LD;
        }
        else
        {
            *dataReg = &periphReg->DHR8RD;
        }

        retState = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Busy-wait delay in microseconds derived from the actual CPU (HCLK) frequency
 *
 * \note  One loop iteration takes at least one CPU clock cycle, so the delay is never shorter
 *        than required (it is longer, depending on the compiler optimization).
 *
 * \param delayUs [in]: Required delay in microseconds
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise (HCLK frequency not available) returns
 *         \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Set_Delay( dac_TimeUs_t delayUs )
{
    dac_RequestState_t       retState = DAC_REQUEST_ERROR;
    rcc_FreqHz_t             hclkFreq = 0u;
    /* CPU clock == HCLK, provided by Rcc as clock of the system timer */
    const rcc_RequestState_t rccState = Rcc_Get_PeriphClk( RCC_PERIPH_SYSTICK, &hclkFreq );

    if( ( RCC_REQUEST_OK == rccState ) &&
        ( 0u             != hclkFreq )    )
    {
        /* CPU clock cycles per microsecond, rounded up */
        const dac_TimeoutCnt_t cyclesPerUs = (dac_TimeoutCnt_t)DAC_DIV_ROUND_UP( hclkFreq, DAC_US_PER_S );
        const dac_TimeoutCnt_t loopCnt     = (dac_TimeoutCnt_t)delayUs * cyclesPerUs;

        for( volatile dac_TimeoutCnt_t delayCnt = 0u; loopCnt > delayCnt; delayCnt ++ )
        {
            /* Busy-wait: intentionally empty */
        }

        retState = DAC_REQUEST_OK;
    }
    else
    {
        /* CPU clock frequency is not available, delay can not be derived */
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */

#endif /* DAC1 */
