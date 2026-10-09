/**
 * \author Mr.Nobody
 * \file Dac_Isr.c
 * \ingroup Dac
 * \brief Dac module DAC interrupt handler
 *
 * The DAC interrupt is shared by both channels of the peripheral and reports the DMA underrun
 * (DMAUDRx) of a running DMA transfer. The transfer is stopped and the error is reported
 * through Dac_Set_XferError() (Dac.c).
 *
 * STM32L4: the DAC1 underrun interrupt shares the NVIC line with TIM6 (TIM6_DAC_IRQn). The
 * handler of the line is replaced by the DAC handler when a DMA transfer is initialized, so TIM6
 * interrupts can not be used together with DAC DMA transfer. The line is disabled in NVIC only
 * if it was enabled by this module.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Dac_Isr.h"                        /* Self include                   */
#include "Dac.h"                            /* Module private interface       */
#include "Nvic_Port.h"                      /* NVIC Mcal layer include        */

#if defined(DAC1)
/* ============================== TYPEDEFS ================================== */

/** DAC interrupt connection to NVIC */
typedef struct
{
    nvic_PeriphIrqList_t IrqId;   /**< NVIC interrupt identification      */
    nvic_IsrCallback_t   Handler; /**< Interrupt handler of the NVIC line  */
}   dac_IsrIrqConfig_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static void               Dac_Isr_Tim6DacHandler ( void );
static dac_RequestState_t Dac_Isr_Handler        ( dac_PeriphId_t periphId );
static dac_FunctionState_t Dac_Isr_Get_LineUsed  ( dac_PeriphId_t periphId );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** dac_PeriphId_t -> DAC interrupt line (shared with TIM6) */
static const dac_IsrIrqConfig_t dac_IsrIrqConfig[ ] =
{
    { .IrqId = NVIC_PERIPH_IRQ_TIM6_DAC, .Handler = Dac_Isr_Tim6DacHandler },
};

_Static_assert( DAC_PERIPH_CNT == ( sizeof(dac_IsrIrqConfig) / sizeof(dac_IsrIrqConfig_t) ), "Dac: dac_IsrIrqConfig has incorrect size." );


/** dac_ChannelId_t -> DMA underrun interrupt enable bit (DAC_CR) */
static const uint32_t dac_IsrUnderrunItLut[ DAC_CHANNEL_CNT ] =
{
    [DAC_CHANNEL_1] = LL_DAC_IT_DMAUDRIE1,
#if defined(DAC_CHANNEL2_SUPPORT)
    [DAC_CHANNEL_2] = LL_DAC_IT_DMAUDRIE2,
#endif /* DAC_CHANNEL2_SUPPORT */
};


/** dac_ChannelId_t -> DMA underrun flag (DAC_SR) */
static const uint32_t dac_IsrUnderrunFlagLut[ DAC_CHANNEL_CNT ] =
{
    [DAC_CHANNEL_1] = LL_DAC_FLAG_DMAUDR1,
#if defined(DAC_CHANNEL2_SUPPORT)
    [DAC_CHANNEL_2] = LL_DAC_FLAG_DMAUDR2,
#endif /* DAC_CHANNEL2_SUPPORT */
};


/** The shared interrupt line was enabled in NVIC for the peripheral by this module */
static dac_FunctionState_t dac_IsrLineOwned[ DAC_PERIPH_CNT ];

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Connects the DAC interrupt to NVIC (handler, priority, enable)
 *
 * \note  The interrupt line is shared by both channels and with TIM6 - the priority of the last
 *        initialized channel is applied.
 *
 * \param periphId [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param irqPrio  [in]: Interrupt priority
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Isr_Init( dac_PeriphId_t periphId, dac_IrqPrio_t irqPrio )
{
    dac_RequestState_t  retState  = DAC_REQUEST_ERROR;
    nvic_RequestState_t nvicState = NVIC_REQUEST_ERROR;

    if( DAC_PERIPH_CNT > periphId )
    {
        const dac_IsrIrqConfig_t * const irqConfig = &dac_IsrIrqConfig[ periphId ];

        nvicState = Nvic_Set_PeriphIrq_Handler( irqConfig->IrqId, irqConfig->Handler );

        if( NVIC_REQUEST_OK == nvicState )
        {
            nvicState = Nvic_Set_PeriphIrq_Prio( irqConfig->IrqId, (nvic_IrqPrio_t)irqPrio );
        }
        else
        {
            /* Handler registration failed, priority is not configured */
        }

        if( NVIC_REQUEST_OK == nvicState )
        {
            nvicState = Nvic_Set_PeriphIrq_Active( irqConfig->IrqId );
        }
        else
        {
            /* Priority configuration failed, interrupt is not enabled */
        }

        if( NVIC_REQUEST_OK == nvicState )
        {
            dac_IsrLineOwned[ periphId ] = DAC_FUNCTION_ACTIVE;
            retState                     = DAC_REQUEST_OK;
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
 * \brief Disables DMA underrun interrupts of all channels and the DAC interrupt in NVIC
 *
 * \note  The interrupt line is shared with TIM6 - it is disabled in NVIC only if it was enabled
 *        by \ref Dac_Isr_Init() for this peripheral and no other DAC peripheral of the line uses it.
 *
 * \param periphId [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Isr_Deinit( dac_PeriphId_t periphId )
{
    dac_RequestState_t retState = DAC_REQUEST_ERROR;

    if( DAC_PERIPH_CNT > periphId )
    {
        retState = DAC_REQUEST_OK;

        for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1;
             ( DAC_CHANNEL_CNT > channelIdx ) &&
             ( DAC_REQUEST_OK == retState );
             channelIdx ++ )
        {
            retState = Dac_Isr_Set_UnderrunIrqInactive( periphId, channelIdx );
        }

        if( ( DAC_REQUEST_OK      == retState                     ) &&
            ( DAC_FUNCTION_ACTIVE == dac_IsrLineOwned[ periphId ] )    )
        {
            dac_IsrLineOwned[ periphId ] = DAC_FUNCTION_INACTIVE;

            const dac_FunctionState_t lineUsed = Dac_Isr_Get_LineUsed( periphId );

            if( DAC_FUNCTION_INACTIVE == lineUsed )
            {
                const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Inactive( dac_IsrIrqConfig[ periphId ].IrqId );

                if( NVIC_REQUEST_OK == nvicState )
                {
                    retState = DAC_REQUEST_OK;
                }
                else
                {
                    dac_IsrLineOwned[ periphId ] = DAC_FUNCTION_ACTIVE;
                    retState                     = DAC_REQUEST_ERROR;
                }
            }
            else
            {
                /* Other DAC peripheral of the line still uses the interrupt */
            }
        }
        else
        {
            /* Interrupts could not be disabled or the line is not owned by the peripheral */
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Clears a pending DMA underrun flag and enables the DMA underrun interrupt of a channel
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Isr_Set_UnderrunIrqActive( dac_PeriphId_t periphId, dac_ChannelId_t channelId )
{
    dac_RequestState_t retState  = DAC_REQUEST_ERROR;
    DAC_TypeDef *      periphReg = DAC_NULL_PTR;

    retState = Dac_Get_PeriphReg( periphId, &periphReg );

    if( ( DAC_REQUEST_OK  == retState  ) &&
        ( DAC_CHANNEL_CNT  > channelId )    )
    {
        const uint32_t itMask = dac_IsrUnderrunItLut[ channelId ];

        /* Stale underrun is cleared (write 1), other SR bits are read only */
        WRITE_REG( periphReg->SR, dac_IsrUnderrunFlagLut[ channelId ] );
        SET_BIT( periphReg->CR, itMask );

        for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t crReg = READ_BIT( periphReg->CR, itMask );

            if( itMask == crReg )
            {
                retState = DAC_REQUEST_OK;
                break;
            }
            else
            {
                /* Interrupt enable has not yet been applied, keep return state as error */
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
 * \brief Disables the DMA underrun interrupt of a channel
 *
 * \param periphId  [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 * \param channelId [in]: DAC channel identification, value from \ref dac_ChannelId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
dac_RequestState_t Dac_Isr_Set_UnderrunIrqInactive( dac_PeriphId_t periphId, dac_ChannelId_t channelId )
{
    dac_RequestState_t retState  = DAC_REQUEST_ERROR;
    DAC_TypeDef *      periphReg = DAC_NULL_PTR;

    retState = Dac_Get_PeriphReg( periphId, &periphReg );

    if( ( DAC_REQUEST_OK  == retState  ) &&
        ( DAC_CHANNEL_CNT  > channelId )    )
    {
        const uint32_t itMask = dac_IsrUnderrunItLut[ channelId ];

        CLEAR_BIT( periphReg->CR, itMask );

        for( dac_TimeoutCnt_t iterationCnt = 0u; DAC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t crReg = READ_BIT( periphReg->CR, itMask );

            if( 0u == crReg )
            {
                retState = DAC_REQUEST_OK;
                break;
            }
            else
            {
                /* Interrupt disable has not yet been applied, keep return state as error */
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

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Returns whether another DAC peripheral of the same NVIC line owns the line
 *
 * \param periphId [in]: DAC peripheral identification, must be valid
 *
 * \return \ref DAC_FUNCTION_ACTIVE if another peripheral of the line enabled the line,
 *         otherwise \ref DAC_FUNCTION_INACTIVE.
 */
static dac_FunctionState_t Dac_Isr_Get_LineUsed( dac_PeriphId_t periphId )
{
    dac_FunctionState_t lineUsed = DAC_FUNCTION_INACTIVE;

    for( dac_PeriphId_t periphIdx = DAC_PERIPH_1; DAC_PERIPH_CNT > periphIdx; periphIdx ++ )
    {
        if( ( periphIdx                           != periphId                           ) &&
            ( dac_IsrIrqConfig[ periphIdx ].IrqId == dac_IsrIrqConfig[ periphId ].IrqId ) &&
            ( DAC_FUNCTION_ACTIVE                 == dac_IsrLineOwned[ periphIdx ]      )    )
        {
            lineUsed = DAC_FUNCTION_ACTIVE;
        }
        else
        {
            /* Peripheral does not use the line */
        }
    }

    return ( lineUsed );
}


/**
 * \brief Common DAC interrupt handler - processes enabled and pending DMA underrun of all channels
 *
 * \param periphId [in]: DAC peripheral identification, value from \ref dac_PeriphId_t
 *
 * \return Function processing state. Returns \ref DAC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref DAC_REQUEST_ERROR.
 */
static dac_RequestState_t Dac_Isr_Handler( dac_PeriphId_t periphId )
{
    dac_RequestState_t retState  = DAC_REQUEST_ERROR;
    DAC_TypeDef *      periphReg = DAC_NULL_PTR;

    retState = Dac_Get_PeriphReg( periphId, &periphReg );

    if( DAC_REQUEST_OK == retState )
    {
        const uint32_t srReg = READ_REG( periphReg->SR );
        const uint32_t crReg = READ_REG( periphReg->CR );

        for( dac_ChannelId_t channelIdx = DAC_CHANNEL_1; DAC_CHANNEL_CNT > channelIdx; channelIdx ++ )
        {
            /* Channel 2 of single channel DAC has empty masks - never pending */
            if( ( 0u != ( srReg & dac_IsrUnderrunFlagLut[ channelIdx ] ) ) &&
                ( 0u != ( crReg & dac_IsrUnderrunItLut[ channelIdx ]   ) )    )
            {
                WRITE_REG( periphReg->SR, dac_IsrUnderrunFlagLut[ channelIdx ] );
                retState = Dac_Set_XferError( periphId, channelIdx, DAC_ERROR_DMA_UNDERRUN );
            }
            else
            {
                /* No underrun pending on the channel */
            }
        }
    }
    else
    {
        retState = DAC_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/** \brief TIM6_DAC line handler - DAC1 underrun (registered in NVIC by Dac_Isr_Init()) */
static void Dac_Isr_Tim6DacHandler( void )
{
    (void)Dac_Isr_Handler( DAC_PERIPH_1 );
    __DSB();    /* Cortex-M4 r0p1 erratum 838869: stores completed before the exception return */
}

/* ================================ TASKS =================================== */

#endif /* DAC1 */
