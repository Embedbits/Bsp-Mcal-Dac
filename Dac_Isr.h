/**
 * \author Mr.Nobody
 * \file Dac_Isr.h
 * \ingroup Dac
 * \brief Dac module DAC interrupt handler (private)
 *
 * DAC interrupt (DMA underrun of both channels) used by DAC_TRANSFER_MODE_DMA. Private to
 * the Dac library.
 *
 */

#ifndef DAC_DAC_ISR_H
#define DAC_DAC_ISR_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Dac_Types.h"                      /* Module types definitions       */

#if defined(DAC1)
/* ============================= TYPEDEFS =================================== */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

dac_RequestState_t Dac_Isr_Init                  ( dac_PeriphId_t periphId, dac_IrqPrio_t irqPrio );
dac_RequestState_t Dac_Isr_Deinit                ( dac_PeriphId_t periphId );

dac_RequestState_t Dac_Isr_Set_UnderrunIrqActive   ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );
dac_RequestState_t Dac_Isr_Set_UnderrunIrqInactive ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );

#endif /* DAC1 */

#ifdef __cplusplus
}
#endif

#endif /* DAC_DAC_ISR_H */
