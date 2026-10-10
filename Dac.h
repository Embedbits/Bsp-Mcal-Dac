/**
 * \author Mr.Nobody
 * \file Dac.h
 * \ingroup Dac
 * \brief Dac module private interface shared between Dac.c and DMA / interrupt handlers
 *
 * This file is private to the Dac library (not exported through public headers). It
 * connects the module root (Dac.c) with the DMA data transfer handler (Dac_Dma.c) and
 * the DAC interrupt handler (Dac_Isr.c). Transfer state and user callbacks are handled
 * once in Dac.c, the handlers only move the data and report events.
 *
 */

#ifndef DAC_DAC_H
#define DAC_DAC_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Dac_Types.h"                      /* Module types definitions       */

#if defined(DAC1)
/* ============================= TYPEDEFS =================================== */

/** \brief Type representing iteration count of busy-wait loops (register read-back, HW flags) */
typedef uint32_t dac_TimeoutCnt_t;


/** \brief Type representing frequency in Hz */
typedef uint32_t dac_FreqHz_t;


/** \brief Type representing time in microseconds */
typedef uint32_t dac_TimeUs_t;


/** \brief Runtime context of channel data transfer (one per DAC channel) */
typedef struct
{
    dac_DataConfig_t             Config;    /**< Copy of user data transfer configuration           */
    volatile dac_FunctionState_t XferState; /**< DMA transfer is running (between start and stop)    */
    dac_FunctionState_t          InitState; /**< DMA transfer resources are initialized              */
}   dac_XferContext_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Busy-wait iteration budget used while polling a HW flag or register read-back (common for
 *  all Dac module files). Same order of magnitude and role as ADC_TIMEOUT_RAW of Adc module. */
#define DAC_TIMEOUT_RAW              ( (dac_TimeoutCnt_t)0x84FCBu )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* Implemented in Dac.c - common services for DMA / interrupt handlers */
dac_RequestState_t Dac_Get_PeriphReg   ( dac_PeriphId_t periphId, DAC_TypeDef ** const periphReg );
dac_RequestState_t Dac_Get_LlChannel   ( dac_ChannelId_t channelId, uint32_t * const llChannel );
dac_RequestState_t Dac_Get_XferContext ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_XferContext_t ** const xferContext );
dac_RequestState_t Dac_Get_DataRegAddr ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, uint32_t * const regAddr );

dac_RequestState_t Dac_Set_XferHalf    ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );
dac_RequestState_t Dac_Set_XferDone    ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );
dac_RequestState_t Dac_Set_XferError   ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_ErrorId_t errorId );

#endif /* DAC1 */

#ifdef __cplusplus
}
#endif

#endif /* DAC_DAC_H */
