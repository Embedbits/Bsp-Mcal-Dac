/**
 * \author Mr.Nobody
 * \file Dac.c
 * \ingroup Dac
 * \brief Dac module common functionality
 *
 * STM32F4 DAC: DAC1 with two channels (DAC_OUT1 = PA4, DAC_OUT2 = PA5, channel 2 only on
 * devices with DAC_CHANNEL2_SUPPORT) clocked by the APB1 bus clock. The output is always
 * connected to the pin, only the output buffer can be disabled (BOFFx). Features of the
 * public interface not available on STM32F4 (kernel clock multiplexer, internal output
 * connection, sample and hold, offset calibration / trimming, channel ready flag) accept
 * only the inactive / standard configuration, otherwise return error.
 *
 * \note  The module is empty on MCUs without DAC peripheral (e.g. STM32F401,
 *        STM32F411).
 *
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

#if defined(DAC)
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
    DAC_TypeDef *PeriphReg; /**< CMSIS DAC instance register pointer */
}   dac_PeriphConfigStruct_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static dac_RequestState_t Dac_Check_ChannelDisabled     ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );
static dac_RequestState_t Dac_Check_PeriphDisabled      ( dac_PeriphId_t periphId );
static dac_RequestState_t Dac_Check_AllPeriphsDisabled  ( void );
static dac_RequestState_t Dac_Check_ChannelConfig       ( dac_PeriphId_t periphId, const dac_ChannelConfig_t * const channelConfig );
static dac_RequestState_t Dac_Check_SampleHoldConfig    ( const dac_SampleHoldConfig_t * const shConfig );
static dac_RequestState_t Dac_Check_DataValue           ( dac_DataFormat_t dataFormat, dac_Data_t data );
static dac_RequestState_t Dac_Check_DmaTrigger          ( dac_TriggerId_t triggerSrc );
static dac_RequestState_t Dac_Check_DataConfig          ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TriggerId_t triggerSrc, const dac_DataConfig_t * const dataConfig );
static dac_RequestState_t Dac_Set_PinAnalog             ( dac_ChannelId_t channelId );
static dac_RequestState_t Dac_Set_DmaRequestRelease     ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );
#if defined(DAC_CHANNEL2_SUPPORT)
static dac_RequestState_t Dac_Get_DualDataReg           ( dac_PeriphId_t periphId, dac_DataFormat_t dataFormat, volatile uint32_t ** const dataReg );
#endif /* DAC_CHANNEL2_SUPPORT */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define DAC_MAJOR_VERSION           ( 1u )

/** Value of minor version of SW module */
#define DAC_MINOR_VERSION           ( 0u )

/** Value of patch version of SW module */
#define DAC_PATCH_VERSION           ( 0u )


/** Divider used for rounding of integer division to the nearest value */
#define DAC_ROUND_DIVIDER            ( 2u )

/** Full scale of the output data register (DORx is always 12-bit right aligned) */
#define DAC_OUTPUT_FULL_SCALE        ( (uint32_t)DAC_DATA_12BIT_MAX )

/* =============================== MACROS =================================== */

/** Integer division rounded to the nearest value */
#define DAC_DIV_ROUND( dividend, divisor )       ( ( (dividend) + ( (divisor) / DAC_ROUND_DIVIDER ) ) / (divisor) )

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** CMSIS instance lookup, indexed by dac_PeriphId_t */
static const dac_PeriphConfigStruct_t dac_PeriphConf[ ] =
{
    { .PeriphReg = DAC1 },
};

_Static_assert( DAC_PERIPH_CNT == ( sizeof(dac_PeriphConf) / sizeof(dac_PeriphConfigStruct_t) ), "Dac: dac_PeriphConf has incorrect size." );


/** dac_ChannelId_t -> output pin (DAC_OUT1 = PA4, DAC_OUT2 = PA5 on all STM32F4 devices) */
static const dac_GpioConfig_t dac_GpioConfig[ DAC_CHANNEL_CNT ] =
{
    [DAC_CHANNEL_1] = { .PortId = GPIO_PORT_A, .PinId = GPIO_PIN_ID_4 },
#if defined(DAC_CHANNEL2_SUPPORT)
    [DAC_CHANNEL_2] = { .PortId = GPIO_PORT_A, .PinId = GPIO_PIN_ID_5 },
#endif /* DAC_CHANNEL2_SUPPORT */
};


/** dac_ChannelId_t -> LL_DAC_CHANNEL_x */
static const uint32_t dac_LlChannelLut[ DAC_CHANNEL_CNT ] =
{
    [DAC_CHANNEL_1] = LL_DAC_CHANNEL_1,
#if defined(DAC_CHANNEL2_SUPPORT)
    [DAC_CHANNEL_2] = LL_DAC_CHANNEL_2,
#endif /* DAC_CHANNEL2_SUPPORT */
};


/** dac_ClkSrc_t -> rcc_PeriphId_t (APB1 bus clock enable of the DAC) */
static const rcc_PeriphId_t dac_ClkSrcRccLut[ DAC_CLK_SRC_CNT ] =
{
    [DAC_CLK_SRC_PCLK1] = RCC_PERIPH_DAC,
};


/** dac_TriggerId_t -> LL_DAC_TRIG_*. Entries follow the exact order and \#if guarding of
 *  dac_TriggerId_t in Dac_Types.h. DAC_TRIGGER_NONE has no own source (TENx = 0), TSELx is
 *  returned to its reset value. */
static const uint32_t dac_TriggerSrcLut[ ] =
{
    [DAC_TRIGGER_NONE]        = 0u,
    [DAC_TRIGGER_SOFTWARE]    = LL_DAC_TRIG_SOFTWARE,
#if defined (TIM2)
    [DAC_TRIGGER_TIM2_TRGO]   = LL_DAC_TRIG_EXT_TIM2_TRGO,
#endif /* TIM2 */
#if defined (TIM4)
    [DAC_TRIGGER_TIM4_TRGO]   = LL_DAC_TRIG_EXT_TIM4_TRGO,
#endif /* TIM4 */
#if defined (TIM5)
    [DAC_TRIGGER_TIM5_TRGO]   = LL_DAC_TRIG_EXT_TIM5_TRGO,
#endif /* TIM5 */
#if defined (TIM6)
    [DAC_TRIGGER_TIM6_TRGO]   = LL_DAC_TRIG_EXT_TIM6_TRGO,
#endif /* TIM6 */
#if defined (TIM7)
    [DAC_TRIGGER_TIM7_TRGO]   = LL_DAC_TRIG_EXT_TIM7_TRGO,
#endif /* TIM7 */
#if defined (TIM8)
    [DAC_TRIGGER_TIM8_TRGO]   = LL_DAC_TRIG_EXT_TIM8_TRGO,
#endif /* TIM8 */
    [DAC_TRIGGER_EXTI_LINE9]  = LL_DAC_TRIG_EXT_EXTI_LINE9,
};

_Static_assert( DAC_TRIGGER_CNT == ( sizeof(dac_TriggerSrcLut) / sizeof(uint32_t) ), "Dac: dac_TriggerSrcLut has incorrect size." );


/** dac_OutputMode_t -> output mode is available on STM32F4 (output always connected to the pin) */
static const dac_FunctionState_t dac_OutputSupportLut[ DAC_OUTPUT_CNT ] =
{
    [DAC_OUTPUT_PIN_BUFFERED]          = DAC_FUNCTION_ACTIVE,
    [DAC_OUTPUT_PIN_INTERNAL_BUFFERED] = DAC_FUNCTION_INACTIVE,
    [DAC_OUTPUT_PIN_UNBUFFERED]        = DAC_FUNCTION_ACTIVE,
    [DAC_OUTPUT_INTERNAL]              = DAC_FUNCTION_INACTIVE,
};


/** dac_OutputMode_t -> LL_DAC_OUTPUT_BUFFER_x (unsupported modes are never written) */
static const uint32_t dac_OutputBufferLut[ DAC_OUTPUT_CNT ] =
{
    [DAC_OUTPUT_PIN_BUFFERED]          = LL_DAC_OUTPUT_BUFFER_ENABLE,
    [DAC_OUTPUT_PIN_INTERNAL_BUFFERED] = LL_DAC_OUTPUT_BUFFER_ENABLE,
    [DAC_OUTPUT_PIN_UNBUFFERED]        = LL_DAC_OUTPUT_BUFFER_DISABLE,
    [DAC_OUTPUT_INTERNAL]              = LL_DAC_OUTPUT_BUFFER_DISABLE,
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


#if defined(DAC_CHANNEL2_SUPPORT)
/** dac_DataFormat_t -> position of channel 2 data in the dual data holding register */
static const uint32_t dac_DualShiftLut[ DAC_DATA_FORMAT_CNT ] =
{
    [DAC_DATA_FORMAT_12BIT_RIGHT] = DAC_DHR12RD_DACC2DHR_Pos,
    [DAC_DATA_FORMAT_12BIT_LEFT]  = DAC_DHR12LD_DACC2DHR_Pos - DAC_DHR12LD_DACC1DHR_Pos,
    [DAC_DATA_FORMAT_8BIT_RIGHT]  = DAC_DHR8RD_DACC2DHR_Pos,
};
#endif /* DAC_CHANNEL2_SUPPORT */


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


/** dac_ChannelId_t -> DMA underrun flag (DAC_SR, the only flags of STM32F4 DAC) */
static const uint32_t dac_UnderrunFlagLut[ DAC_CHANNEL_CNT ] =
{
    [DAC_CHANNEL_1] = LL_DAC_FLAG_DMAUDR1,
#if defined(DAC_CHANNEL2_SUPPORT)
    [DAC_CHANNEL_2] = LL_DAC_FLAG_DMAUDR2,
#endif /* DAC_CHANNEL2_SUPPORT */
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
 * Enables the DAC bus clock (see \ref Dac_Set_ClockSource()), then initializes every peripheral
 * from dacConfig->PeriphConfig[] that has at least one used channel (see \ref Dac_PeriphInit()).
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

        for( dac_PeriphId_t periphIdx = DAC_PERIPH_1; ( DAC_PERIPH_CNT > periphIdx ) && ( DAC_REQUEST_OK == retState ); periphIdx ++ )
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

        if( ( DAC_REQUEST_OK != retState ) && ( DAC_FUNCTION_ACTIVE == clkConfOk ) )
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
 * Stops running DMA transfers and releases their resources (DMA stream, interrupts),
 * disables all channels, the DAC (DMA underrun) interrupt and the DAC bus clock.
 *
 * \note  The DAC interrupt line (TIM6_DAC) is disabled in NVIC only if it was enabled by
 *        this module (see Dac_Isr.c) - the line is shared with TIM6.
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

        for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; ( DAC_CHANNEL_CNT > channelIdx ) && ( DAC_REQUEST_OK == retState ); channelIdx ++ )
        {
            dac_XferContext_t * const xferCtx = &dac_XferContext[ periphId ][ channelIdx ];

            /* --- Release DMA transfer resources (DMA stream, interrupts) --- */
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
        }

        /* --- Disable DAC interrupt --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Isr_Deinit( periphId );
        }

        /* --- Disable DAC bus clock --- */
        if( DAC_REQUEST_OK == retState )
        {
            const rcc_RequestState_t rccState = Rcc_Set_PeriphInactive( dac_ClkSrcRccLut[ DAC_CLK_SRC_PCLK1 ] );

            if( RCC_REQUEST_OK == rccState )
            {
                retState = DAC_REQUEST_OK;
            }
            else
            {
                retState = DAC_REQUEST_ERROR;
            }
        }

        if( DAC_REQUEST_OK == retState )
        {
            for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; DAC_CHANNEL_CNT > channelIdx; channelIdx ++ )
            {
                dac_ShadowDataFormat[ periphId ][ channelIdx ]              = DAC_DATA_FORMAT_12BIT_RIGHT;
                dac_XferContext[ periphId ][ channelIdx ].Config.TransferMode = DAC_TRANSFER_MODE_DIRECT;
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
 * Default configuration: APB1 bus clock, channel 1 used with buffered pin output, no trigger,
 * 12-bit right aligned data, no wave generation, sample and hold disabled, initial value 0 and
 * direct data transfer. DMA fields are filled with the fixed DMA request mapping of the channel
 * (DMA1 stream 5 / 6). Channel 2 is not used.
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
        dacConfig->ClockSource = DAC_CLK_SRC_PCLK1;
        retState               = DAC_REQUEST_OK;

        for( dac_PeriphId_t periphIdx = DAC_PERIPH_1; ( DAC_PERIPH_CNT > periphIdx ) && ( DAC_REQUEST_OK == retState ); periphIdx ++ )
        {
            dac_PeriphConfig_t * const periphCfg = &dacConfig->PeriphConfig[ periphIdx ];

            periphCfg->PeriphId      = periphIdx;
            periphCfg->ShClockSource = DAC_SH_CLK_SRC_LSI;

            for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; ( DAC_CHANNEL_CNT > channelIdx ) && ( DAC_REQUEST_OK == retState ); channelIdx ++ )
            {
                dac_ChannelConfig_t * const channelCfg = &periphCfg->Channels[ channelIdx ];

                channelCfg->ChannelId                           = channelIdx;
                channelCfg->ChannelUsed                         = DAC_FUNCTION_INACTIVE;
                channelCfg->OutputMode                          = DAC_OUTPUT_PIN_BUFFERED;
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
                channelCfg->DataConfig.DmaPriority              = DAC_DMA_PRIORITY_LOW;
                channelCfg->DataConfig.IrqPriority              = 0u;
                channelCfg->DataConfig.HalfTransferCallback     = DAC_NULL_PTR;
                channelCfg->DataConfig.TransferCompleteCallback = DAC_NULL_PTR;
                channelCfg->DataConfig.ErrorCallback            = DAC_NULL_PTR;

                /* DMA stream is given by the DMA request mapping of the channel */
                retState = Dac_Dma_Get_Stream( periphIdx, channelIdx, &channelCfg->DataConfig.Dma );
            }

            periphCfg->Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
        }
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
 * \brief Enables the DAC bus clock (APB1)
 *
 * \note  STM32F4 DAC has no kernel clock multiplexer, DAC_CLK_SRC_PCLK1 is the only source.
 *
 * \pre   All DAC channels must be disabled. Otherwise \ref DAC_REQUEST_ERROR is returned and no
 *        register is modified.
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

        /* --- DAC bus clock enable (verified by Rcc module) --- */
        if( DAC_REQUEST_OK == retState )
        {
            const rcc_RequestState_t rccState = Rcc_Set_PeriphActive( dac_ClkSrcRccLut[ clkSource ] );

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
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the DAC clock source
 *
 * \note  STM32F4 DAC is always clocked by the APB1 bus clock (DAC_CLK_SRC_PCLK1).
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
        *clkSource = DAC_CLK_SRC_PCLK1;
        retState   = DAC_REQUEST_OK;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects the sample and hold clock source
 *
 * \note  Sample and hold is not supported by STM32F4.
 *
 * \param clkSource [in]: Required sample and hold clock source, value from \ref dac_ShClkSrc_t
 *
 * \return Returns always \ref DAC_REQUEST_ERROR - feature is not supported.
 */
dac_RequestState_t Dac_Set_ShClockSource( dac_ShClkSrc_t clkSource )
{
    (void)clkSource;

    return ( DAC_REQUEST_ERROR );
}


/**
 * \brief Returns the sample and hold clock source
 *
 * \note  Sample and hold is not supported by STM32F4.
 *
 * \param clkSource [out]: Pointer to store the clock source (\ref dac_ShClkSrc_t).
 *
 * \return Returns always \ref DAC_REQUEST_ERROR - feature is not supported.
 */
dac_RequestState_t Dac_Get_ShClockSource( dac_ShClkSrc_t * const clkSource )
{
    (void)clkSource;

    return ( DAC_REQUEST_ERROR );
}


/* -------------------------------------------------------------------------- */
/* ------------------- Peripheral and channel configuration ----------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Initializes a DAC peripheral and all its used channels
 *
 * Every channel with ChannelUsed == DAC_FUNCTION_ACTIVE is initialized by Dac_ChannelInit().
 *
 * \note  Channels[] is indexed by \ref dac_ChannelId_t, Channels[ i ].ChannelId must be equal to i
 *        for a used channel.
 *
 * \note  ShClockSource is checked only (sample and hold is not supported by STM32F4, every used
 *        channel must have sample and hold inactive).
 *
 * \pre   DAC bus clock must be enabled (Dac_Set_ClockSource()) and all channels of the peripheral
 *        must be disabled. Otherwise \ref DAC_REQUEST_ERROR is returned and no register is modified.
 *
 * \param periphConfig [in]: Pointer to peripheral configuration structure \ref dac_PeriphConfig_t.
 *                           Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_PeriphInit( dac_PeriphConfig_t * const periphConfig )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_NULL_PTR       != periphConfig                ) &&
        ( DAC_PERIPH_CNT      > periphConfig->PeriphId      ) &&
        ( DAC_SH_CLK_SRC_CNT  > periphConfig->ShClockSource )    )
    {
        const dac_PeriphId_t periphId = periphConfig->PeriphId;

        /* --- Whole configuration is checked before any register is modified --- */
        retState = Dac_Check_PeriphDisabled( periphId );

        for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; ( DAC_CHANNEL_CNT > channelIdx ) && ( DAC_REQUEST_OK == retState ); channelIdx ++ )
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
            }
        }

        /* --- Channels --- */
        for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; ( DAC_CHANNEL_CNT > channelIdx ) && ( DAC_REQUEST_OK == retState ); channelIdx ++ )
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
 * Sequence: output pin (analog), output buffer, trigger, wave generation, data format, initial
 * output value, data transfer configuration (DMA resources) and channel enable.
 *
 * \note  The whole configuration is checked before any register is modified (sample and hold
 *        must be inactive, output mode must be a pin output mode).
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

    if( ( DAC_PERIPH_CNT > periphId      ) &&
        ( DAC_NULL_PTR  != channelConfig )    )
    {
        const dac_ChannelId_t channelId = channelConfig->ChannelId;

        retState = Dac_Check_ChannelConfig( periphId, channelConfig );

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Check_ChannelDisabled( periphId, channelId );
        }

        /* --- Output pin (STM32F4 output is always connected to the pin) --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_PinAnalog( channelId );
        }

        /* --- Output buffer --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_OutputMode( periphId, channelId, channelConfig->OutputMode );
        }

        /* --- Trigger, wave generation and data format --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_TriggerSrc( periphId, channelId, channelConfig->TriggerId );
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_WaveAmplitude( periphId, channelId, channelConfig->WaveAmplitude );
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_WaveMode( periphId, channelId, channelConfig->WaveMode );
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_DataFormat( periphId, channelId, channelConfig->DataFormat );
        }

        /* --- Initial output value (written before the data transfer configuration) --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_Data( periphId, channelId, channelConfig->InitValue );
        }

        /* --- Data transfer resources --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_DataConfig( periphId, channelId, &channelConfig->DataConfig );
        }

        /* --- Channel enable --- */
        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Set_ChannelActive( periphId, channelId );
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables a DAC channel
 *
 * \note  STM32F4 DAC has no channel ready flag - the enable bit is verified by read-back. The
 *        channel is operational after tWAKEUP and the output voltage settles within
 *        LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US (refer to device datasheet), these times are
 *        not waited.
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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId )    )
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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId )    )
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

    if( ( DAC_PERIPH_CNT  > periphId     ) &&
        ( DAC_CHANNEL_CNT > channelId    ) &&
        ( DAC_NULL_PTR   != channelState )    )
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
 * \brief Selects the channel output mode (output buffer, BOFFx)
 *
 * \note  STM32F4 DAC output is always connected to the pin - only DAC_OUTPUT_PIN_BUFFERED and
 *        DAC_OUTPUT_PIN_UNBUFFERED are supported, other modes return \ref DAC_REQUEST_ERROR.
 *
 * \note  The output pin is not configured by this function (see Dac_ChannelInit()).
 *
 * \pre   The channel must be disabled. Otherwise \ref DAC_REQUEST_ERROR is returned and no
 *        register is modified.
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

    if( ( DAC_PERIPH_CNT      > periphId                           ) &&
        ( DAC_CHANNEL_CNT     > channelId                          ) &&
        ( DAC_OUTPUT_CNT      > outputMode                         ) &&
        ( DAC_FUNCTION_ACTIVE == dac_OutputSupportLut[ outputMode ] )    )
    {
        retState = Dac_Check_ChannelDisabled( periphId, channelId );

        if( DAC_REQUEST_OK == retState )
        {
            DAC_TypeDef * const periphReg = dac_PeriphConf[ periphId ].PeriphReg;
            const uint32_t      llChannel = dac_LlChannelLut[ channelId ];
            const uint32_t      llBuffer  = dac_OutputBufferLut[ outputMode ];

            LL_DAC_SetOutputBuffer( periphReg, llChannel, llBuffer );

            for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t bufferReg = LL_DAC_GetOutputBuffer( periphReg, llChannel );

                if( llBuffer == bufferReg )
                {
                    retState = DAC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Output buffer has not yet been applied, keep return state as error */
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
 * \brief Returns the channel output mode (output buffer)
 *
 * \param periphId    [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId   [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param outputMode [out]: Pointer to store the output mode (DAC_OUTPUT_PIN_BUFFERED or
 *                          DAC_OUTPUT_PIN_UNBUFFERED). Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Get_OutputMode( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_OutputMode_t * const outputMode )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT  > periphId   ) &&
        ( DAC_CHANNEL_CNT > channelId  ) &&
        ( DAC_NULL_PTR   != outputMode )    )
    {
        const uint32_t llBuffer = LL_DAC_GetOutputBuffer( dac_PeriphConf[ periphId ].PeriphReg, dac_LlChannelLut[ channelId ] );

        if( LL_DAC_OUTPUT_BUFFER_ENABLE == llBuffer )
        {
            *outputMode = DAC_OUTPUT_PIN_BUFFERED;
        }
        else
        {
            *outputMode = DAC_OUTPUT_PIN_UNBUFFERED;
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
 * \brief Selects the channel conversion trigger (TENx, TSELx)
 *
 * \pre   The channel must be disabled. Otherwise \ref DAC_REQUEST_ERROR is returned and no
 *        register is modified.
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

    if( ( DAC_PERIPH_CNT  > periphId   ) &&
        ( DAC_CHANNEL_CNT > channelId  ) &&
        ( DAC_TRIGGER_CNT > triggerSrc )    )
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
            const uint32_t      llTrigger = dac_TriggerSrcLut[ triggerSrc ];
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

                if( ( llTrigger == trigReg ) && ( llTrigEn == trigEnReg ) )
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

    if( ( DAC_PERIPH_CNT  > periphId   ) &&
        ( DAC_CHANNEL_CNT > channelId  ) &&
        ( DAC_NULL_PTR   != triggerSrc )    )
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
            for( dac_TriggerId_t trigIdx = DAC_TRIGGER_SOFTWARE; ( DAC_TRIGGER_CNT > trigIdx ) && ( DAC_REQUEST_OK != retState ); trigIdx ++ )
            {
                if( dac_TriggerSrcLut[ trigIdx ] == trigReg )
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

    if( ( DAC_PERIPH_CNT      > periphId                                       ) &&
        ( DAC_CHANNEL_CNT     > channelId                                      ) &&
        ( DAC_DATA_FORMAT_CNT > dataFormat                                     ) &&
        ( DAC_FUNCTION_ACTIVE != dac_XferContext[ periphId ][ channelId ].XferState ) )
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

    if( ( DAC_PERIPH_CNT  > periphId   ) &&
        ( DAC_CHANNEL_CNT > channelId  ) &&
        ( DAC_NULL_PTR   != dataFormat )    )
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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId ) &&
        ( DAC_WAVE_CNT    > waveMode  )    )
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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId ) &&
        ( DAC_NULL_PTR   != waveMode  )    )
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

    if( ( DAC_PERIPH_CNT         > periphId  ) &&
        ( DAC_CHANNEL_CNT        > channelId ) &&
        ( DAC_WAVE_AMPLITUDE_CNT > amplitude )    )
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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId ) &&
        ( DAC_NULL_PTR   != amplitude )    )
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
 * \brief Configures channel sample and hold mode
 *
 * \note  Sample and hold is not supported by STM32F4 - only a valid configuration with
 *        State == DAC_FUNCTION_INACTIVE is accepted (nothing is written).
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param shConfig  [in]: Pointer to sample and hold configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK for inactive sample and hold.
 *         Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Set_SampleHoldConfig( dac_PeriphId_t periphId, dac_ChannelId_t channelId, const dac_SampleHoldConfig_t * const shConfig )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId )    )
    {
        retState = Dac_Check_SampleHoldConfig( shConfig );
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns channel sample and hold configuration
 *
 * \note  Sample and hold is not supported by STM32F4 - inactive state with zero timing is returned.
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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId ) &&
        ( DAC_NULL_PTR   != shConfig  )    )
    {
        shConfig->State       = DAC_FUNCTION_INACTIVE;
        shConfig->SampleTime  = 0u;
        shConfig->HoldTime    = 0u;
        shConfig->RefreshTime = 0u;

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
 * \brief Executes channel offset calibration
 *
 * \note  Offset calibration is not supported by STM32F4.
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Returns always \ref DAC_REQUEST_ERROR - feature is not supported.
 */
dac_RequestState_t Dac_Set_Calibration( dac_PeriphId_t periphId, dac_ChannelId_t channelId )
{
    (void)periphId;
    (void)channelId;

    return ( DAC_REQUEST_ERROR );
}


/**
 * \brief Writes the channel offset trimming value
 *
 * \note  Offset trimming is not supported by STM32F4.
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param trimValue [in]: Trimming value
 *
 * \return Returns always \ref DAC_REQUEST_ERROR - feature is not supported.
 */
dac_RequestState_t Dac_Set_TrimValue( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TrimValue_t trimValue )
{
    (void)periphId;
    (void)channelId;
    (void)trimValue;

    return ( DAC_REQUEST_ERROR );
}


/**
 * \brief Returns the channel offset trimming value
 *
 * \note  Offset trimming is not supported by STM32F4.
 *
 * \param periphId   [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId  [in]: DAC channel identification, value from \ref dac_ChannelId_t
 * \param trimValue [out]: Pointer to store the trimming value.
 *
 * \return Returns always \ref DAC_REQUEST_ERROR - feature is not supported.
 */
dac_RequestState_t Dac_Get_TrimValue( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TrimValue_t * const trimValue )
{
    (void)periphId;
    (void)channelId;
    (void)trimValue;

    return ( DAC_REQUEST_ERROR );
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

    if( ( DAC_PERIPH_CNT       > periphId                                        ) &&
        ( DAC_CHANNEL_CNT      > channelId                                       ) &&
        ( DAC_FUNCTION_ACTIVE != dac_XferContext[ periphId ][ channelId ].XferState ) )
    {
        const dac_DataFormat_t dataFormat = dac_ShadowDataFormat[ periphId ][ channelId ];

        retState = Dac_Check_DataValue( dataFormat, data );

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Get_DataRegAddr( periphId, channelId, &regAddr );
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
 * \note  Devices without DAC channel 2 return \ref DAC_REQUEST_ERROR.
 *
 * \pre   Both channels must use the same data format, no DMA transfer may be running on any
 *        channel and data must be within the range of the format. Otherwise
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
#if defined(DAC_CHANNEL2_SUPPORT)
    volatile uint32_t * dataReg  = DAC_NULL_PTR;

    if( DAC_PERIPH_CNT > periphId )
    {
        const dac_DataFormat_t dataFormat = dac_ShadowDataFormat[ periphId ][ DAC_CHANNEL_1 ];
        const dac_RequestState_t ch1State = Dac_Check_DataValue( dataFormat, dataCh1 );
        const dac_RequestState_t ch2State = Dac_Check_DataValue( dataFormat, dataCh2 );

        if( ( dataFormat          == dac_ShadowDataFormat[ periphId ][ DAC_CHANNEL_2 ]   ) &&
            ( DAC_REQUEST_OK      == ch1State                                            ) &&
            ( DAC_REQUEST_OK      == ch2State                                            ) &&
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
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }
#else
    /* Device has single DAC channel */
    (void)periphId;
    (void)dataCh1;
    (void)dataCh2;
#endif /* DAC_CHANNEL2_SUPPORT */

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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId ) &&
        ( DAC_NULL_PTR   != data      )    )
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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId )    )
    {
        retState = Dac_Get_TriggerSrc( periphId, channelId, &triggerSrc );

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Get_ChannelState( periphId, channelId, &channelState );
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
 * \note  Devices without DAC channel 2 return \ref DAC_REQUEST_ERROR.
 *
 * \pre   Both channels must be enabled with trigger DAC_TRIGGER_SOFTWARE. Otherwise
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

#if defined(DAC_CHANNEL2_SUPPORT)
    if( DAC_PERIPH_CNT > periphId )
    {
        retState = DAC_REQUEST_OK;

        for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; ( DAC_CHANNEL_CNT > channelIdx ) && ( DAC_REQUEST_OK == retState ); channelIdx ++ )
        {
            dac_TriggerId_t     triggerSrc   = DAC_TRIGGER_CNT;
            dac_FunctionState_t channelState = DAC_FUNCTION_INACTIVE;

            retState = Dac_Get_TriggerSrc( periphId, channelIdx, &triggerSrc );

            if( DAC_REQUEST_OK == retState )
            {
                retState = Dac_Get_ChannelState( periphId, channelIdx, &channelState );
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
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }
#else
    /* Device has single DAC channel */
    (void)periphId;
#endif /* DAC_CHANNEL2_SUPPORT */

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

    if( ( DAC_PERIPH_CNT  > periphId   ) &&
        ( DAC_CHANNEL_CNT > channelId  ) &&
        ( 0u              < refVoltage ) &&
        ( refVoltage     >= outVoltage ) &&
        ( DAC_NULL_PTR   != data       )    )
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

    if( ( 0u           < refVoltage ) &&
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
 * \note  STM32F4 DAC has only the DMA underrun flags - other flags return \ref DAC_REQUEST_ERROR.
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

    if( ( DAC_PERIPH_CNT        > periphId  ) &&
        ( DAC_CHANNEL_CNT       > channelId ) &&
        ( DAC_FLAG_DMA_UNDERRUN == flagId   ) &&
        ( DAC_NULL_PTR         != flagState )    )
    {
        const uint32_t flagReg = READ_BIT( dac_PeriphConf[ periphId ].PeriphReg->SR, dac_UnderrunFlagLut[ channelId ] );

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
 * \note  Only DAC_FLAG_DMA_UNDERRUN exists on STM32F4. The underrun flag can be set by HW at
 *        any time, so the clear is not verified by read-back.
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

    if( ( DAC_PERIPH_CNT        > periphId  ) &&
        ( DAC_CHANNEL_CNT       > channelId ) &&
        ( DAC_FLAG_DMA_UNDERRUN == flagId   )    )
    {
        /* DMAUDRx is cleared by writing 1, other SR bits are reserved */
        WRITE_REG( dac_PeriphConf[ periphId ].PeriphReg->SR, dac_UnderrunFlagLut[ channelId ] );
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
 * stream and the DAC interrupt are initialized (the transfer is started by Dac_Set_DmaStart()).
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

    if( ( DAC_PERIPH_CNT  > periphId   ) &&
        ( DAC_CHANNEL_CNT > channelId  ) &&
        ( DAC_NULL_PTR   != dataConfig )    )
    {
        dac_XferContext_t * const xferCtx = &dac_XferContext[ periphId ][ channelId ];

        retState = Dac_Get_TriggerSrc( periphId, channelId, &triggerSrc );

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Check_DataConfig( periphId, channelId, triggerSrc, dataConfig );
        }

        if( ( DAC_REQUEST_OK == retState ) && ( DAC_FUNCTION_ACTIVE == xferCtx->XferState ) )
        {
            /* DMA transfer is running, configuration change is not allowed */
            retState = DAC_REQUEST_ERROR;
        }
        else
        {
            /* Configuration is valid and no transfer is running */
        }

        /* --- Resources of the previous configuration are released --- */
        if( ( DAC_REQUEST_OK == retState ) && ( DAC_FUNCTION_ACTIVE == xferCtx->InitState ) )
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

    if( ( DAC_PERIPH_CNT  > periphId   ) &&
        ( DAC_CHANNEL_CNT > channelId  ) &&
        ( DAC_NULL_PTR   != dataConfig )    )
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
 * The DMA stream is armed for the whole buffer, DAC DMA requests and the DMA underrun
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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId )    )
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
 * DAC DMA requests, the DMA underrun interrupt and the DMA stream are disabled. The last
 * output value is kept.
 *
 * \note  Device errata ("DMA request not automatically cleared by clearing DMAEN"): a DAC DMA
 *        request pending at the stop would be serviced immediately by the next start. Errata
 *        sequence is applied after DMAEN is cleared: the underrun flag is checked and cleared,
 *        the DAC bus clock is disabled and enabled again (registers keep the configuration, the
 *        DMA stream and the trigger are configured again by the next start). Triggers of the
 *        other channel are not processed while the DAC clock is disabled.
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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId )    )
    {
        dac_XferContext_t * const xferCtx = &dac_XferContext[ periphId ][ channelId ];

        if( DAC_FUNCTION_ACTIVE == xferCtx->InitState )
        {
            retState = Dac_Dma_Stop( periphId, channelId );

            if( DAC_REQUEST_OK == retState )
            {
                retState = Dac_Set_DmaRequestRelease( periphId, channelId );
            }
            else
            {
                /* DMA requests could not be stopped */
            }
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

    if( ( DAC_PERIPH_CNT > periphId  ) &&
        ( DAC_NULL_PTR  != periphReg )    )
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

    if( ( DAC_CHANNEL_CNT > channelId ) &&
        ( DAC_NULL_PTR   != llChannel )    )
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

    if( ( DAC_PERIPH_CNT  > periphId    ) &&
        ( DAC_CHANNEL_CNT > channelId   ) &&
        ( DAC_NULL_PTR   != xferContext )    )
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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId ) &&
        ( DAC_NULL_PTR   != regAddr   )    )
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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId )    )
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
 * \brief Handles the end of the buffer: circular output continues (DMA stream in circular
 *        mode), one shot output is stopped (Dac_Set_DmaStop()); then TransferCompleteCallback
 *        is called (the transfer can be restarted from the callback)
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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId )    )
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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId ) &&
        ( DAC_ERROR_CNT   > errorId   )    )
    {
        dac_ErrCallback_t * const callback = dac_XferContext[ periphId ][ channelId ].Config.ErrorCallback;

        /* Transfer is stopped - DMA underrun stops DAC DMA requests, DMA errors disable the stream */
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

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId )    )
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
 * \brief Checks that all channels of a DAC peripheral are disabled
 *
 * \param periphId [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 *
 * \return Returns \ref DAC_REQUEST_OK if all channels are disabled. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Check_PeriphDisabled( dac_PeriphId_t periphId )
{
    dac_RequestState_t retState = DAC_REQUEST_OK;

    for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; ( DAC_CHANNEL_CNT > channelIdx ) && ( DAC_REQUEST_OK == retState ); channelIdx ++ )
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

    for( dac_PeriphId_t periphIdx = DAC_PERIPH_1; ( DAC_PERIPH_CNT > periphIdx ) && ( DAC_REQUEST_OK == retState ); periphIdx ++ )
    {
        retState = Dac_Check_PeriphDisabled( periphIdx );
    }

    return ( retState );
}


/**
 * \brief Checks channel configuration (all enumerations, value ranges, features supported by
 *        STM32F4, data transfer)
 *
 * \param periphId      [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelConfig [in]: Pointer to channel configuration. Must not be NULL.
 *
 * \return Returns \ref DAC_REQUEST_OK if the configuration is valid. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Check_ChannelConfig( dac_PeriphId_t periphId, const dac_ChannelConfig_t * const channelConfig )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT          > periphId                     ) &&
        ( DAC_NULL_PTR           != channelConfig                ) &&
        ( DAC_CHANNEL_CNT         > channelConfig->ChannelId     ) &&
        ( DAC_OUTPUT_CNT          > channelConfig->OutputMode    ) &&
        ( DAC_TRIGGER_CNT         > channelConfig->TriggerId     ) &&
        ( DAC_DATA_FORMAT_CNT     > channelConfig->DataFormat    ) &&
        ( DAC_WAVE_CNT            > channelConfig->WaveMode      ) &&
        ( DAC_WAVE_AMPLITUDE_CNT  > channelConfig->WaveAmplitude )    )
    {
        if( DAC_FUNCTION_ACTIVE == dac_OutputSupportLut[ channelConfig->OutputMode ] )
        {
            retState = Dac_Check_SampleHoldConfig( &channelConfig->SampleHold );
        }
        else
        {
            /* Output mode is not supported by STM32F4 */
            retState = DAC_REQUEST_ERROR;
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Check_DataValue( channelConfig->DataFormat, channelConfig->InitValue );
        }

        if( DAC_REQUEST_OK == retState )
        {
            retState = Dac_Check_DataConfig( periphId, channelConfig->ChannelId, channelConfig->TriggerId, &channelConfig->DataConfig );
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks sample and hold configuration
 *
 * \note  Sample and hold is not supported by STM32F4 - only the inactive state is valid, the
 *        timing values are checked against the common interface ranges.
 *
 * \param shConfig [in]: Pointer to sample and hold configuration. Must not be NULL.
 *
 * \return Returns \ref DAC_REQUEST_OK if the configuration is valid. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Check_SampleHoldConfig( const dac_SampleHoldConfig_t * const shConfig )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_NULL_PTR            != shConfig             ) &&
        ( DAC_FUNCTION_INACTIVE   == shConfig->State      ) &&
        ( DAC_SH_SAMPLE_TIME_MAX  >= shConfig->SampleTime ) &&
        ( DAC_SH_HOLD_TIME_MAX    >= shConfig->HoldTime   )    )
    {
        /* RefreshTime covers the whole range by its type (8 bits) */
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

    if( ( DAC_DATA_FORMAT_CNT          > dataFormat     ) &&
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

    if( ( DAC_TRIGGER_CNT      > triggerSrc ) &&
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
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Set_PinAnalog( dac_ChannelId_t channelId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( DAC_CHANNEL_CNT > channelId )
    {
        gpio_Config_t gpioConfig;

        gpioConfig.PortId         = dac_GpioConfig[ channelId ].PortId;
        gpioConfig.PinId          = dac_GpioConfig[ channelId ].PinId;
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
 * \brief Releases a DAC DMA request pending after the DMA requests of a channel were stopped
 *
 * Device errata "DMA request not automatically cleared by clearing DMAEN": the pending request
 * is not cleared by DMAEN = 0, it would be serviced as soon as the DMA transfer is started again
 * (spurious transfer not aligned to the trigger). Errata sequence after DMAEN was cleared: the
 * underrun flag (DMAUDRx) is checked and cleared, the DAC bus clock is disabled and enabled
 * again. Registers keep the configuration (output value, trigger, other channel), the DMA stream
 * and the trigger are configured again by the next start.
 *
 * \pre   DMAEN of the channel is cleared and its DMA stream is disabled (Dac_Dma_Stop()).
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Set_DmaRequestRelease( dac_PeriphId_t periphId, dac_ChannelId_t channelId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( ( DAC_PERIPH_CNT  > periphId  ) &&
        ( DAC_CHANNEL_CNT > channelId )    )
    {
        DAC_TypeDef * const  periphReg = dac_PeriphConf[ periphId ].PeriphReg;
        const rcc_PeriphId_t rccId     = dac_ClkSrcRccLut[ DAC_CLK_SRC_PCLK1 ];

        if( 0u != READ_BIT( periphReg->SR, dac_UnderrunFlagLut[ channelId ] ) )
        {
            /* DMAUDRx is cleared by writing 1, other SR bits are reserved */
            WRITE_REG( periphReg->SR, dac_UnderrunFlagLut[ channelId ] );
        }
        else
        {
            /* No underrun of the stopped transfer */
        }

        rcc_RequestState_t rccState = Rcc_Set_PeriphInactive( rccId );

        if( RCC_REQUEST_OK == rccState )
        {
            rccState = Rcc_Set_PeriphActive( rccId );
        }
        else
        {
            /* DAC clock could not be disabled */
        }

        retState = ( RCC_REQUEST_OK == rccState ) ? DAC_REQUEST_OK : DAC_REQUEST_ERROR;
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


#if defined(DAC_CHANNEL2_SUPPORT)
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

    if( ( DAC_PERIPH_CNT      > periphId   ) &&
        ( DAC_DATA_FORMAT_CNT > dataFormat ) &&
        ( DAC_NULL_PTR       != dataReg    )    )
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
#endif /* DAC_CHANNEL2_SUPPORT */

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */

#endif /* DAC */
