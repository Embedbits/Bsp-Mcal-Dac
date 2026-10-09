/**
 * \author Mr.Nobody
 * \file Dac_Dma.h
 * \ingroup Dac
 * \brief Dac module DMA data transfer handler (private)
 *
 * DAC_TRANSFER_MODE_DMA - output values are moved from the user buffer to the channel data
 * holding register by a DMA channel on every DAC DMA request. Private to the Dac library.
 *
 */

#ifndef DAC_DAC_DMA_H
#define DAC_DAC_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Dac_Types.h"                      /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

dac_RequestState_t Dac_Dma_Check_Config ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, const dac_DataConfig_t * const dataConfig );

dac_RequestState_t Dac_Dma_Init         ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );
dac_RequestState_t Dac_Dma_Deinit       ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );
dac_RequestState_t Dac_Dma_Start        ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );
dac_RequestState_t Dac_Dma_Stop         ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );

#ifdef __cplusplus
}
#endif

#endif /* DAC_DAC_DMA_H */
