/**
 * \author Mr.Nobody
 * \file Dac_Port.h
 * \ingroup Dac
 * \brief Digital-to-Analog Converter (DAC) module public functionality
 *
 * This file contains all available public functionality, any other files shall
 * not used outside of the module.
 *
 */

#ifndef DAC_DAC_PORT_H
#define DAC_DAC_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "Dac_Types.h"                      /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

dac_ModuleVersion_t         Dac_Get_ModuleVersion           ( void );

dac_RequestState_t          Dac_Init                        ( dac_Config_t * const dacConfig );
dac_RequestState_t          Dac_Deinit                      ( dac_PeriphId_t periphId );
void                        Dac_Task                        ( void );

dac_RequestState_t          Dac_Get_DefaultConfig           ( dac_Config_t * const dacConfig );

/* -------------------------------------------------------------------------- */
/* -------------------------- Clock configuration --------------------------- */
/* -------------------------------------------------------------------------- */

dac_RequestState_t          Dac_Set_ClockSource             ( dac_ClkSrc_t clkSource );
dac_RequestState_t          Dac_Get_ClockSource             ( dac_ClkSrc_t * const clkSource );

dac_RequestState_t          Dac_Set_ShClockSource           ( dac_ShClkSrc_t clkSource );
dac_RequestState_t          Dac_Get_ShClockSource           ( dac_ShClkSrc_t * const clkSource );

/* -------------------------------------------------------------------------- */
/* ------------------- Peripheral and channel configuration ----------------- */
/* -------------------------------------------------------------------------- */

dac_RequestState_t          Dac_PeriphInit                  ( dac_PeriphConfig_t * const periphConfig );
dac_RequestState_t          Dac_ChannelInit                 ( dac_PeriphId_t periphId, dac_ChannelConfig_t * const channelConfig );

dac_RequestState_t          Dac_Set_ChannelActive           ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );
dac_RequestState_t          Dac_Set_ChannelInactive         ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );
dac_RequestState_t          Dac_Get_ChannelState            ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_FunctionState_t * const channelState );

dac_RequestState_t          Dac_Set_OutputMode              ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_OutputMode_t outputMode );
dac_RequestState_t          Dac_Get_OutputMode              ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_OutputMode_t * const outputMode );

dac_RequestState_t          Dac_Set_TriggerSrc              ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TriggerId_t triggerSrc );
dac_RequestState_t          Dac_Get_TriggerSrc              ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TriggerId_t * const triggerSrc );

dac_RequestState_t          Dac_Set_DataFormat              ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_DataFormat_t dataFormat );
dac_RequestState_t          Dac_Get_DataFormat              ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_DataFormat_t * const dataFormat );

dac_RequestState_t          Dac_Set_WaveMode                ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_WaveMode_t waveMode );
dac_RequestState_t          Dac_Get_WaveMode                ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_WaveMode_t * const waveMode );

dac_RequestState_t          Dac_Set_WaveAmplitude           ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_WaveAmplitude_t amplitude );
dac_RequestState_t          Dac_Get_WaveAmplitude           ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_WaveAmplitude_t * const amplitude );

dac_RequestState_t          Dac_Set_SampleHoldConfig        ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, const dac_SampleHoldConfig_t * const shConfig );
dac_RequestState_t          Dac_Get_SampleHoldConfig        ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_SampleHoldConfig_t * const shConfig );

/* -------------------------------------------------------------------------- */
/* ------------------------------ Calibration ------------------------------- */
/* -------------------------------------------------------------------------- */

dac_RequestState_t          Dac_Set_Calibration             ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );

dac_RequestState_t          Dac_Set_TrimValue               ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TrimValue_t trimValue );
dac_RequestState_t          Dac_Get_TrimValue               ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_TrimValue_t * const trimValue );

/* -------------------------------------------------------------------------- */
/* ------------------------------ Data output ------------------------------- */
/* -------------------------------------------------------------------------- */

dac_RequestState_t          Dac_Set_Data                    ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Data_t data );
dac_RequestState_t          Dac_Set_DualData                ( dac_PeriphId_t periphId, dac_Data_t dataCh1, dac_Data_t dataCh2 );
dac_RequestState_t          Dac_Get_OutputData              ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Data_t * const data );

dac_RequestState_t          Dac_Set_SwTrigger               ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );
dac_RequestState_t          Dac_Set_DualSwTrigger           ( dac_PeriphId_t periphId );

dac_RequestState_t          Dac_Get_DataFromVoltage         ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Voltage_mV_t refVoltage, dac_Voltage_mV_t outVoltage, dac_Data_t * const data );
dac_RequestState_t          Dac_Get_OutputVoltage           ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Voltage_mV_t refVoltage, dac_Voltage_mV_t * const outVoltage );
dac_RequestState_t          Dac_Set_Voltage                 ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Voltage_mV_t refVoltage, dac_Voltage_mV_t outVoltage );

dac_RequestState_t          Dac_Get_Flag                    ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_FlagId_t flagId, dac_FlagState_t * const flagState );
dac_RequestState_t          Dac_Clear_Flag                  ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_FlagId_t flagId );

/* -------------------------------------------------------------------------- */
/* --------------------------- DMA data transfer ---------------------------- */
/* -------------------------------------------------------------------------- */

dac_RequestState_t          Dac_Set_DataConfig              ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, const dac_DataConfig_t * const dataConfig );
dac_RequestState_t          Dac_Get_DataConfig              ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_DataConfig_t * const dataConfig );

dac_RequestState_t          Dac_Set_DmaStart                ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );
dac_RequestState_t          Dac_Set_DmaStop                 ( dac_PeriphId_t periphId, dac_ChannelId_t channelId );

#ifdef __cplusplus
}
#endif

#endif /* DAC_DAC_PORT_H */

