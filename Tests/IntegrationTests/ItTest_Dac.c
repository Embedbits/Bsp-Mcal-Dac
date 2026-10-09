/**
 * \author Mr.Nobody
 * \file ItTest_Dac.c
 * \ingroup Dac
 * \brief Integration tests of Digital-to-Analog Converter (DAC) module on target (STM32L4 / STM32L4+).
 *
 * Dac module runs on the MCU together with real RCC, NVIC, GPIO, DMA and ADC modules and
 * hardware. Tests verify behavior which cannot be verified by unit tests (emulated
 * registers): channel enable / output register update, offset calibration, trigger
 * handling, wave generation, sample and hold with LSI, output voltage measured by the ADC
 * and DMA buffer output driven by a hardware trigger.
 *
 * No external wiring is used:
 * - Output values are verified by the output registers (DORx).
 * - The output voltage is measured by the ADC (Adc module, both converters use VREF+ as
 *   reference - the DAC value and the ADC result are compared directly):
 *   - internal output (DAC_OUTPUT_INTERNAL): ADC2 channel 17 / 18, ADC1 channel 17 / 18 on
 *     single ADC MCUs (shared with the temperature sensor / VBAT),
 *   - pin output of channel 1 (PA4, boards with free PA4): ADC1 channel 9 (same pin).
 * - TIM6 TRGO (update event) is the hardware trigger of DMA transfers. TIM6 is driven
 *   directly through LL (TRGO configuration is not provided by Tim module).
 *
 * Used resources: DAC1 (channel 2 where available), ADC1 / ADC2, TIM6, DMA channels of the
 * default DAC configuration (DMA1 channel 3 / 4).
 *
 * \note Boards are named by the MCU detected by integration testing (DEV_ID and flash size -
 *       the first MCU of the same identification in the Ral presets, other MCUs of the
 *       identification are listed too). Board facts from the STM32CubeMX board files:
 *       Nucleo-64-P boards use PA4 as SMPS enable (ADP5301 VEN) - the DAC pins are never
 *       driven on the groups with a Nucleo-64-P board (internal output only, no calibration).
 *       PA5 (DAC1_OUT2) drives the user LED of Nucleo-64 boards and is linked to PB7 on
 *       Nucleo-32 boards - the pin of channel 2 is never driven. STM32L41x / L42x have no DAC -
 *       all tests ignored.
 * \note STM32L4 errata sheets not reviewed yet.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Dac_Port.h"                       /* Module under test              */
#include "Adc_Port.h"                       /* ADC used for output measurement */
#include "Rcc_Port.h"                       /* Clock of TIM6, DAC clock state */
#include "Dma_Port.h"                       /* DMA channel of the DAC (underrun test) */
#include "Stm32_tim.h"                      /* TIM6 used as hardware trigger  */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

#if defined(DAC1)
static void         It_Dac_Get_Config       ( dac_Config_t * const config );
static void         It_Dac_Set_DmaConfig    ( dac_ChannelConfig_t * const channelConfig, dac_BufferMode_t bufferMode );
static void         It_Dac_Adc_Init         ( dac_ChannelId_t dacChannel, dac_OutputMode_t outputMode );
static uint32_t     It_Dac_Adc_Measure      ( void );
static uint32_t     It_Dac_Get_Diff         ( uint32_t valueA, uint32_t valueB );
static void         It_Dac_Tim6_Start       ( void );
static void         It_Dac_Tim6_Stop        ( void );
static uint32_t     It_Dac_Wait             ( volatile uint32_t * const counter, uint32_t expectedCnt );
static dac_Data_t   It_Dac_Wait_OutputData  ( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Data_t expectedData );

static void         It_Dac_HalfCallback     ( void );
static void         It_Dac_CompleteCallback ( void );
static void         It_Dac_ErrorCallback    ( dac_ErrorId_t errorId );
#endif /* DAC1 */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/*----------------------------- Board configuration --------------------------*/
#if defined(IT_BOARD_STM32L412xB) || \
    defined(IT_BOARD_STM32L422xB)

    /* NUCLEO-L412KB, NUCLEO-L412RB-P: MCU without DAC - all tests ignored */

#elif defined(IT_BOARD_STM32L431xC) || \
      defined(IT_BOARD_STM32L432xC) || \
      defined(IT_BOARD_STM32L433xC) || \
      defined(IT_BOARD_STM32L442xC) || \
      defined(IT_BOARD_STM32L443xC) || \
      defined(IT_BOARD_STM32L451xE) || \
      defined(IT_BOARD_STM32L452xE) || \
      defined(IT_BOARD_STM32L462xE)

    /** NUCLEO-L432KC / L433RC-P, NUCLEO-L452RE / L452RE-P - PA4 is the SMPS enable of the
     *  Nucleo-64-P board of the group: output connected to on-chip peripherals only */
    #define IT_DAC_OUTPUT_MODE              ( DAC_OUTPUT_INTERNAL )

#elif defined(IT_BOARD_STM32L471xG) || \
      defined(IT_BOARD_STM32L475xG) || \
      defined(IT_BOARD_STM32L476xG) || \
      defined(IT_BOARD_STM32L485xG) || \
      defined(IT_BOARD_STM32L486xG) || \
      defined(IT_BOARD_STM32L496xG) || \
      defined(IT_BOARD_STM32L4A6xG) || \
      defined(IT_BOARD_STM32L4P5xG) || \
      defined(IT_BOARD_STM32L4Q5xG) || \
      defined(IT_BOARD_STM32L4R5xI) || \
      defined(IT_BOARD_STM32L4R7xI) || \
      defined(IT_BOARD_STM32L4R9xI) || \
      defined(IT_BOARD_STM32L4S5xI) || \
      defined(IT_BOARD_STM32L4S7xI) || \
      defined(IT_BOARD_STM32L4S9xI)

    /** NUCLEO-L476RG (PA4 Arduino A2) and the Nucleo-144 boards (PA4 on the connectors
     *  only) - buffered pin output of channel 1, measured by ADC1 channel 9 (same pin) */
    #define IT_DAC_OUTPUT_MODE              ( DAC_OUTPUT_PIN_BUFFERED )
    #define IT_DAC_PIN_OUTPUT

#else
    #error "Board of Dac integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/*------------------------------ Resources -----------------------------------*/

/** DAC peripheral and channel under test */
#define IT_DAC_PERIPH                       ( DAC_PERIPH_1 )
#define IT_DAC_CHANNEL                      ( DAC_CHANNEL_1 )

/** DAC channel of DMA transfers in circular mode (channel 2 where available) */
#if defined(DAC_CHANNEL2_SUPPORT)
#define IT_DAC_CIRC_CHANNEL                 ( DAC_CHANNEL_2 )
#else
#define IT_DAC_CIRC_CHANNEL                 ( DAC_CHANNEL_1 )
#endif /* DAC_CHANNEL2_SUPPORT */

/** ADC measuring the internal DAC outputs (ADC2 channel 17 / 18, ADC1 on single ADC MCUs) */
#if defined(ADC2)
#define IT_DAC_ADC_INT_PERIPH               ( ADC_PERIPH_2 )
#else
#define IT_DAC_ADC_INT_PERIPH               ( ADC_PERIPH_1 )
#endif /* ADC2 */

/** ADC measuring the pin output of channel 1 (PA4 = ADC1 channel 9) */
#define IT_DAC_ADC_PIN_PERIPH               ( ADC_PERIPH_1 )
#define IT_DAC_ADC_PIN_CHANNEL              ( ADC_CH_ADC1_IN9_PA4 )

/** Output values used by the tests (buffered output linear range) */
#define IT_DAC_VALUE_LOW                    ( 600u )
#define IT_DAC_VALUE_MID                    ( 2048u )
#define IT_DAC_VALUE_HIGH                   ( 3500u )

/** Maximal difference between DAC value and ADC result [LSB, 12-bit] */
#define IT_DAC_TOLERANCE_LSB                ( 60u )

/** Analog supply / reference voltage of the Nucleo boards [mV] */
#define IT_DAC_VREF_MV                      ( 3300u )

/** Size of the DMA buffer */
#define IT_DAC_BUF_SIZE                     ( 16u )

/** TIM6 prescaler and auto-reload - trigger period of 0.08 ms - 0.13 ms (timer clock 80 - 120 MHz) */
#define IT_DAC_TIM6_PRESCALER               ( 999u )
#define IT_DAC_TIM6_AUTORELOAD              ( 9u )

/** Maximal count of wait loop iterations (DMA events) */
#define IT_DAC_WAIT_LOOPS                   ( 20000000u )

/** Interrupt priority of the DAC interrupt */
#define IT_DAC_IRQ_PRIO                     ( 5u )

/** Sample and hold timing [LSI cycles] */
#define IT_DAC_SH_SAMPLE_TIME               ( 20u )
#define IT_DAC_SH_HOLD_TIME                 ( 50u )
#define IT_DAC_SH_REFRESH_TIME              ( 10u )

/** Ignore message of the tests on MCUs without DAC */
#define IT_DAC_NO_DAC_MSG                   "MCU without DAC"

/** DAC1 outputs not connected to the ADC - STM32L4R / L4S (Bug AB#1282) */
#if defined(STM32L4R5xx) || \
    defined(STM32L4R7xx) || \
    defined(STM32L4R9xx) || \
    defined(STM32L4S5xx) || \
    defined(STM32L4S7xx) || \
    defined(STM32L4S9xx)
#define IT_DAC_ADC_NOT_CONNECTED
#endif /* STM32L4R / L4S */

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

#if defined(DAC1)
/** DMA output buffer */
static dac_Data_t               itDac_DmaBuffer[ IT_DAC_BUF_SIZE ];

/** Counts of callback calls and the last error */
static volatile uint32_t        itDac_HalfCnt;
static volatile uint32_t        itDac_CompleteCnt;
static volatile uint32_t        itDac_ErrorCnt;
static volatile dac_ErrorId_t   itDac_LastError;

/** ADC peripheral used by the last It_Dac_Adc_Init() */
static adc_PeriphId_t           itDac_AdcPeriph;

#if !defined(IT_DAC_ADC_NOT_CONNECTED)
/** ADC channel of the internal output of a DAC channel (item of the list adc_Channel_t), indexed by dac_ChannelId_t */
static const adc_Channel_t      itDac_AdcIntChannelLut[ DAC_CHANNEL_CNT ] =
{
#if defined(ADC2)
    [DAC_CHANNEL_1] = ADC_CH_ADC2_DAC1,
#if defined(DAC_CHANNEL2_SUPPORT)
    [DAC_CHANNEL_2] = ADC_CH_ADC2_DAC2,
#endif /* DAC_CHANNEL2_SUPPORT */
#else
    [DAC_CHANNEL_1] = ADC_CH_ADC1_DAC1,
#if defined(DAC_CHANNEL2_SUPPORT)
    [DAC_CHANNEL_2] = ADC_CH_ADC1_DAC2,
#endif /* DAC_CHANNEL2_SUPPORT */
#endif /* ADC2 */
};
#endif /* IT_DAC_ADC_NOT_CONNECTED */
#endif /* DAC1 */

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
#if defined(DAC1)
    itDac_HalfCnt     = 0u;
    itDac_CompleteCnt = 0u;
    itDac_ErrorCnt    = 0u;
    itDac_LastError   = DAC_ERROR_CNT;
    itDac_AdcPeriph   = ADC_PERIPH_CNT;

    for( uint32_t dataIdx = 0u; IT_DAC_BUF_SIZE > dataIdx; dataIdx++ )
    {
        itDac_DmaBuffer[ dataIdx ] = (dac_Data_t)( ( dataIdx + 1u ) * 200u );
    }
#endif /* DAC1 */
}


void tearDown( void )
{
    /* Every test case runs after system reset */
}

/* =============================== TESTS ==================================== */

/*------------------------------ Configuration -------------------------------*/

/**
 * \brief   Dac_Init() on target enables the clock and the channel.
 *
 * \details Initializes the module with the test configuration (DAC1 channel 1, output mode of
 *          the board, no trigger) and reads the configuration back.
 *
 * \par Expected results
 * - Dac_Init() returns DAC_REQUEST_OK, APB1 clock of DAC1 is enabled in RCC.
 * - Clock source PCLK1, sample and hold clock LSI.
 * - Channel enabled, output mode of the board, no trigger, no DMA underrun.
 * - Ready flag refused (no DACxRDY on STM32L4).
 * - Trimming value (found by calibration of a buffered output) is within 0 - DAC_TRIM_VALUE_MAX.
 */
void It_Dac_Init_DefaultConfig_ReadBack( void )
{
#if defined(DAC1)
    dac_Config_t        config;
    dac_ClkSrc_t        clkSrc       = DAC_CLK_SRC_CNT;
    dac_ShClkSrc_t      shClkSrc     = DAC_SH_CLK_SRC_CNT;
    dac_FunctionState_t channelState = DAC_FUNCTION_INACTIVE;
    dac_OutputMode_t    outputMode   = DAC_OUTPUT_CNT;
    dac_TriggerId_t     triggerSrc   = DAC_TRIGGER_CNT;
    dac_FlagState_t     readyFlag    = DAC_FLAG_INACTIVE;
    dac_FlagState_t     underrunFlag = DAC_FLAG_ACTIVE;
    dac_TrimValue_t     trimValue    = 0xFFu;
    rcc_FunctionState_t rccState     = RCC_FUNCTION_INACTIVE;

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphState( RCC_PERIPH_DAC1, &rccState ) );
    TEST_ASSERT_EQUAL( RCC_FUNCTION_ACTIVE, rccState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( DAC_CLK_SRC_PCLK1, clkSrc );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ShClockSource( &shClkSrc ) );
    TEST_ASSERT_EQUAL( DAC_SH_CLK_SRC_LSI, shClkSrc );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ChannelState( IT_DAC_PERIPH, IT_DAC_CHANNEL, &channelState ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE, channelState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_Flag( IT_DAC_PERIPH, IT_DAC_CHANNEL, DAC_FLAG_READY, &readyFlag ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputMode( IT_DAC_PERIPH, IT_DAC_CHANNEL, &outputMode ) );
    TEST_ASSERT_EQUAL( IT_DAC_OUTPUT_MODE, outputMode );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TriggerSrc( IT_DAC_PERIPH, IT_DAC_CHANNEL, &triggerSrc ) );
    TEST_ASSERT_EQUAL( DAC_TRIGGER_NONE, triggerSrc );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_Flag( IT_DAC_PERIPH, IT_DAC_CHANNEL, DAC_FLAG_DMA_UNDERRUN, &underrunFlag ) );
    TEST_ASSERT_EQUAL( DAC_FLAG_INACTIVE, underrunFlag );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TrimValue( IT_DAC_PERIPH, IT_DAC_CHANNEL, &trimValue ) );
    TEST_ASSERT_LESS_OR_EQUAL( DAC_TRIM_VALUE_MAX, trimValue );
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Init() on target rejects invalid configurations and second initialization.
 *
 * \details
 * 1. NULL configuration.
 * 2. Initial value above 12-bit range.
 * 3. DMA transfer without hardware trigger.
 * 4. Invalid trigger DAC_TRIGGER_CNT.
 * 5. Channel 2 of the single channel DAC (STM32L45x / L46x).
 * 6. Valid initialization, then second initialization while the channel is enabled.
 *
 * \par Expected results
 * 1. - 5. DAC_REQUEST_ERROR, channel stays disabled.
 * 6. DAC_REQUEST_OK, then DAC_REQUEST_ERROR.
 */
void It_Dac_Init_InvalidConfig_ReturnsError( void )
{
#if defined(DAC1)
    dac_Config_t        config;
    dac_FunctionState_t channelState = DAC_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( DAC_NULL_PTR ) );

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].InitValue = DAC_DATA_12BIT_MAX + 1u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    It_Dac_Get_Config( &config );
    It_Dac_Set_DmaConfig( &config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ], DAC_BUFFER_MODE_ONE_SHOT );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].TriggerId = DAC_TRIGGER_NONE;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].TriggerId = DAC_TRIGGER_CNT;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ChannelState( IT_DAC_PERIPH, IT_DAC_CHANNEL, &channelState ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_INACTIVE, channelState );

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Deinit() on target disables the channel and the clock of DAC1, new
 *          initialization possible.
 *
 * \par Expected results
 * - APB1 clock of DAC1 is disabled after Dac_Deinit(), second Dac_Init() returns
 *   DAC_REQUEST_OK and the channel is enabled again.
 */
void It_Dac_Deinit_InitializedPeripheral_InitializationPossibleAgain( void )
{
#if defined(DAC1)
    dac_Config_t        config;
    dac_FunctionState_t channelState = DAC_FUNCTION_INACTIVE;
    rcc_FunctionState_t rccState     = RCC_FUNCTION_ACTIVE;

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Deinit( IT_DAC_PERIPH ) );

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphState( RCC_PERIPH_DAC1, &rccState ) );
    TEST_ASSERT_EQUAL( RCC_FUNCTION_INACTIVE, rccState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ChannelState( IT_DAC_PERIPH, IT_DAC_CHANNEL, &channelState ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE, channelState );
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* DAC1 */
}


/**
 * \brief   Offset calibration on target finds the trimming value (CAL_FLAG toggles).
 *
 * \details Channel 1 with buffered pin output initialized (calibrated by Dac_Init()), disabled,
 *          calibrated again by Dac_Set_Calibration() and enabled. The calibration drives the
 *          pin (buffered pin mode) - boards without free PA4: test ignored.
 *
 * \par Expected results
 * - Calibration of the enabled channel is refused.
 * - Dac_Set_Calibration() of the disabled channel returns DAC_REQUEST_OK, trimming value is
 *   within 0 - DAC_TRIM_VALUE_MAX, the channel can be enabled again.
 */
void It_Dac_Set_Calibration_DisabledChannel_TrimValueFound( void )
{
#if defined(IT_DAC_PIN_OUTPUT)
    dac_Config_t        config;
    dac_TrimValue_t     trimValue    = 0xFFu;
    dac_FunctionState_t channelState = DAC_FUNCTION_INACTIVE;

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_Calibration( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_ChannelInactive( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Calibration( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TrimValue( IT_DAC_PERIPH, IT_DAC_CHANNEL, &trimValue ) );
    TEST_ASSERT_LESS_OR_EQUAL( DAC_TRIM_VALUE_MAX, trimValue );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_ChannelActive( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ChannelState( IT_DAC_PERIPH, IT_DAC_CHANNEL, &channelState ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE, channelState );
#else
    TEST_IGNORE_MESSAGE( "Board without free DAC1_OUT1 pin (or MCU without DAC)" );
#endif /* IT_DAC_PIN_OUTPUT */
}


/**
 * \brief   Sample and hold mode on target starts LSI and updates the output.
 *
 * \details Channel 1 (output mode of the board) with sample and hold (LSI clock, sample 20,
 *          hold 50, refresh 10 cycles).
 *
 * \par Expected results
 * - Dac_Init() returns DAC_REQUEST_OK, LSI is running.
 * - Sample and hold configuration is read back, LSE is refused as sample and hold clock.
 * - Written value appears in the output register.
 */
void It_Dac_Init_SampleHold_LsiStartedOutputUpdated( void )
{
#if defined(DAC1)
    dac_Config_t           config;
    dac_SampleHoldConfig_t shConfig = { .State = DAC_FUNCTION_INACTIVE };
    rcc_FunctionState_t    lsiState = RCC_FUNCTION_INACTIVE;

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].ShClockSource                                     = DAC_SH_CLK_SRC_LSI;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].SampleHold.State       = DAC_FUNCTION_ACTIVE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].SampleHold.SampleTime  = IT_DAC_SH_SAMPLE_TIME;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].SampleHold.HoldTime    = IT_DAC_SH_HOLD_TIME;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].SampleHold.RefreshTime = IT_DAC_SH_REFRESH_TIME;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_OscState( RCC_OSC_LSI, &lsiState ) );
    TEST_ASSERT_EQUAL( RCC_FUNCTION_ACTIVE, lsiState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_SampleHoldConfig( IT_DAC_PERIPH, IT_DAC_CHANNEL, &shConfig ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE, shConfig.State );
    TEST_ASSERT_EQUAL( IT_DAC_SH_SAMPLE_TIME, shConfig.SampleTime );
    TEST_ASSERT_EQUAL( IT_DAC_SH_HOLD_TIME, shConfig.HoldTime );
    TEST_ASSERT_EQUAL( IT_DAC_SH_REFRESH_TIME, shConfig.RefreshTime );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ShClockSource( DAC_SH_CLK_SRC_LSE ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, IT_DAC_VALUE_MID ) );
    TEST_ASSERT_EQUAL( IT_DAC_VALUE_MID, It_Dac_Wait_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, IT_DAC_VALUE_MID ) );
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* DAC1 */
}


/**
 * \brief   Channel 2 is usable only on MCUs with two DAC channels.
 *
 * \details MCUs with channel 2: channel 2 with internal output initialized, value written.
 *          STM32L45x / L46x (one channel): channel 2 data / dual data / dual software trigger
 *          requests with initialized channel 1.
 *
 * \par Expected results
 * - Two channels: Dac_Init() returns DAC_REQUEST_OK, channel 2 enabled, written value appears
 *   in the output register of channel 2.
 * - One channel: all channel 2 requests return DAC_REQUEST_ERROR.
 */
void It_Dac_Channel2_Availability_UsableOrRefused( void )
{
#if defined(DAC1)
    dac_Config_t        config;
    dac_Data_t          outData      = 0u;
    dac_FunctionState_t channelState = DAC_FUNCTION_INACTIVE;

    It_Dac_Get_Config( &config );
#if defined(DAC_CHANNEL2_SUPPORT)
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_INACTIVE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].OutputMode  = DAC_OUTPUT_INTERNAL;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ChannelState( IT_DAC_PERIPH, DAC_CHANNEL_2, &channelState ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE, channelState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, DAC_CHANNEL_2, 0x0456u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, DAC_CHANNEL_2, &outData ) );
    TEST_ASSERT_EQUAL_HEX16( 0x0456u, outData );
#else
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    /* Single channel DAC - DAC_CHANNEL_2 is not in the list, the channel out of the list and the dual services are refused */
    TEST_ASSERT_EQUAL_UINT32( 1u, (uint32_t)DAC_CHANNEL_CNT );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_ChannelState( IT_DAC_PERIPH, DAC_CHANNEL_CNT, &channelState ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_Data( IT_DAC_PERIPH, DAC_CHANNEL_CNT, 0x0456u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_OutputData( IT_DAC_PERIPH, DAC_CHANNEL_CNT, &outData ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DualData( IT_DAC_PERIPH, 0u, 0u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DualSwTrigger( IT_DAC_PERIPH ) );
#endif /* DAC_CHANNEL2_SUPPORT */
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* DAC1 */
}

/*-------------------------------- Data output -------------------------------*/

/**
 * \brief   Dac_Set_Data() without trigger updates the output register immediately.
 *
 * \par Expected results
 * - Dac_Get_OutputData() returns the written value for 12-bit right, 12-bit left
 *   and 8-bit right format (converted to 12-bit right aligned).
 */
void It_Dac_Set_Data_NoTrigger_OutputRegisterUpdated( void )
{
#if defined(DAC1)
    dac_Config_t config;
    dac_Data_t   outData = 0u;

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, 0x0ABCu ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, &outData ) );
    TEST_ASSERT_EQUAL_HEX16( 0x0ABCu, outData );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( IT_DAC_PERIPH, IT_DAC_CHANNEL, DAC_DATA_FORMAT_12BIT_LEFT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, 0x5670u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, &outData ) );
    TEST_ASSERT_EQUAL_HEX16( 0x0567u, outData );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( IT_DAC_PERIPH, IT_DAC_CHANNEL, DAC_DATA_FORMAT_8BIT_RIGHT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, 0x80u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, &outData ) );
    TEST_ASSERT_EQUAL_HEX16( 0x0800u, outData );
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* DAC1 */
}


/**
 * \brief   Software trigger moves the data holding register to the output.
 *
 * \par Expected results
 * - Output register keeps the initial value after Dac_Set_Data().
 * - Output register contains the new value after Dac_Set_SwTrigger().
 */
void It_Dac_Set_SwTrigger_SoftwareTrigger_OutputUpdatedOnTrigger( void )
{
#if defined(DAC1)
    dac_Config_t config;
    dac_Data_t   outData = 0xFFFFu;

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].TriggerId = DAC_TRIGGER_SOFTWARE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].InitValue = 0u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, IT_DAC_VALUE_MID ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, &outData ) );
    TEST_ASSERT_EQUAL( 0u, outData );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_SwTrigger( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    TEST_ASSERT_EQUAL( IT_DAC_VALUE_MID, It_Dac_Wait_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, IT_DAC_VALUE_MID ) );
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Set_DualData() and dual software trigger update outputs of both channels.
 *
 * \details Both channels of DAC1 with software trigger, channel 2 with internal output.
 *          MCUs with one DAC channel: test ignored.
 *
 * \par Expected results
 * - Output register of channel 2 keeps 0 after Dac_Set_DualData(), both output registers
 *   contain the written values after Dac_Set_DualSwTrigger().
 */
void It_Dac_Set_DualData_BothChannels_OutputsUpdated( void )
{
#if defined(DAC_CHANNEL2_SUPPORT)
    dac_Config_t config;
    dac_Data_t   outData = 0xFFFFu;

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].TriggerId   = DAC_TRIGGER_SOFTWARE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].OutputMode  = DAC_OUTPUT_INTERNAL;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].TriggerId   = DAC_TRIGGER_SOFTWARE;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DualData( IT_DAC_PERIPH, 0x0123u, 0x0FEDu ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, DAC_CHANNEL_2, &outData ) );
    TEST_ASSERT_EQUAL_HEX16( 0u, outData );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DualSwTrigger( IT_DAC_PERIPH ) );

    TEST_ASSERT_EQUAL_HEX16( 0x0123u, It_Dac_Wait_OutputData( IT_DAC_PERIPH, DAC_CHANNEL_1, 0x0123u ) );
    TEST_ASSERT_EQUAL_HEX16( 0x0FEDu, It_Dac_Wait_OutputData( IT_DAC_PERIPH, DAC_CHANNEL_2, 0x0FEDu ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC channel 2" );
#endif /* DAC_CHANNEL2_SUPPORT */
}


/**
 * \brief   Triangle wave generation changes the output on every software trigger.
 *
 * \details Channel with software trigger, triangle wave amplitude 4095 and data
 *          holding register 0 is triggered several times.
 *
 * \note    STM32G4 Bug AB#1167: the output of the first trigger is the data holding register (the
 *          triangle counter is incremented after the trigger).
 *
 * \par Expected results
 * - The first trigger outputs the data holding register value (0).
 * - Output value increments by 1 on every next trigger (rising part of the triangle).
 */
void It_Dac_Set_WaveMode_Triangle_OutputFollowsTriangle( void )
{
#if defined(DAC1)
    dac_Config_t config;
    dac_Data_t   lastData = 0u;

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].TriggerId     = DAC_TRIGGER_SOFTWARE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].WaveMode      = DAC_WAVE_TRIANGLE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].WaveAmplitude = DAC_WAVE_AMPLITUDE_4095;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].InitValue     = 0u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    /* The first trigger outputs DHR + triangle counter (0), the counter is incremented after
     * the trigger - the output follows from the next trigger */
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_SwTrigger( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    lastData = It_Dac_Wait_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, 0u );
    TEST_ASSERT_EQUAL_HEX16( 0u, lastData );

    for( uint32_t triggerIdx = 0u; 8u > triggerIdx; triggerIdx++ )
    {
        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_SwTrigger( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
        TEST_ASSERT_EQUAL( lastData + 1u, It_Dac_Wait_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, lastData + 1u ) );
        lastData++;
    }
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Set_Voltage() sets the output, Dac_Get_OutputVoltage() reads it back.
 *
 * \par Expected results
 * - Output register contains half of the range for half of the reference voltage.
 * - Output voltage read back equals the required voltage within 1 mV.
 */
void It_Dac_Set_Voltage_HalfReference_OutputVoltageMatches( void )
{
#if defined(DAC1)
    dac_Config_t     config;
    dac_Data_t       outData    = 0u;
    dac_Voltage_mV_t outVoltage = 0u;

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Voltage( IT_DAC_PERIPH, IT_DAC_CHANNEL, IT_DAC_VREF_MV, IT_DAC_VREF_MV / 2u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, &outData ) );
    TEST_ASSERT_UINT_WITHIN( 1u, IT_DAC_VALUE_MID, outData );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputVoltage( IT_DAC_PERIPH, IT_DAC_CHANNEL, IT_DAC_VREF_MV, &outVoltage ) );
    TEST_ASSERT_UINT_WITHIN( 1u, IT_DAC_VREF_MV / 2u, outVoltage );
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* DAC1 */
}

/*------------------------------ Output voltage ------------------------------*/

/**
 * \brief   Output voltage of buffered pin output measured by ADC matches the DAC value.
 *
 * \details DAC1_OUT1 (PA4) is converted by ADC1 channel 9 (same pin) for low, middle and high
 *          output value. Boards without free PA4: test ignored.
 *
 * \par Expected results
 * - ADC result differs from the DAC value by less than IT_DAC_TOLERANCE_LSB.
 */
void It_Dac_Set_Data_PinBufferedOutput_AdcMeasuresOutput( void )
{
#if defined(IT_DAC_PIN_OUTPUT)
    const dac_Data_t values[ ] = { IT_DAC_VALUE_LOW, IT_DAC_VALUE_MID, IT_DAC_VALUE_HIGH };
    dac_Config_t     config;

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].OutputMode = DAC_OUTPUT_PIN_BUFFERED;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    It_Dac_Adc_Init( IT_DAC_CHANNEL, DAC_OUTPUT_PIN_BUFFERED );

    for( uint32_t valueIdx = 0u; ( sizeof( values ) / sizeof( values[ 0u ] ) ) > valueIdx; valueIdx++ )
    {
        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, values[ valueIdx ] ) );

        const uint32_t adcResult = It_Dac_Adc_Measure();

        TEST_ASSERT_LESS_THAN( IT_DAC_TOLERANCE_LSB, It_Dac_Get_Diff( values[ valueIdx ], adcResult ) );
    }
#else
    TEST_IGNORE_MESSAGE( "Board without free DAC1_OUT1 pin (or MCU without DAC)" );
#endif /* IT_DAC_PIN_OUTPUT */
}


/**
 * \brief   Output voltage of unbuffered pin output measured by ADC matches the DAC value.
 *
 * \details DAC1_OUT1 (PA4) with output buffer disabled is converted by ADC1 channel 9
 *          (long sampling time). Boards without free PA4: test ignored.
 *
 * \par Expected results
 * - ADC result differs from the DAC value by less than IT_DAC_TOLERANCE_LSB.
 */
void It_Dac_Set_Data_PinUnbufferedOutput_AdcMeasuresOutput( void )
{
#if defined(IT_DAC_PIN_OUTPUT)
    dac_Config_t config;

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].OutputMode = DAC_OUTPUT_PIN_UNBUFFERED;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].InitValue  = IT_DAC_VALUE_MID;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    It_Dac_Adc_Init( IT_DAC_CHANNEL, DAC_OUTPUT_PIN_UNBUFFERED );

    const uint32_t adcResult = It_Dac_Adc_Measure();

    TEST_ASSERT_LESS_THAN( IT_DAC_TOLERANCE_LSB, It_Dac_Get_Diff( IT_DAC_VALUE_MID, adcResult ) );
#else
    TEST_IGNORE_MESSAGE( "Board without free DAC1_OUT1 pin (or MCU without DAC)" );
#endif /* IT_DAC_PIN_OUTPUT */
}


/**
 * \brief   Internal outputs (no pin) measured by the ADC channels of the DAC outputs match the
 *          DAC values.
 *
 * \details Channel 1 (and channel 2 where available) with output connected to on-chip
 *          peripherals only, converted by ADC2 channel 17 / 18 (ADC1 channel 17 / 18 on single
 *          ADC MCUs) for low, middle and high output value - every channel measured by its own
 *          ADC initialization.
 * \note    Bug AB#1282: the DAC1 outputs of STM32L4R / L4S are not connected to the ADC - test
 *          ignored (the pin output is measured by the pin output tests).
 *
 * \par Expected results
 * - ADC result differs from the DAC value by less than IT_DAC_TOLERANCE_LSB.
 */
void It_Dac_Set_Data_InternalOutput_AdcMeasuresOutput( void )
{
#if defined(IT_DAC_ADC_NOT_CONNECTED)
    TEST_IGNORE_MESSAGE( "DAC1 outputs not connected to the ADC (STM32L4R / L4S)" );
#elif defined(DAC1)
    const dac_Data_t      values[ ]   = { IT_DAC_VALUE_LOW, IT_DAC_VALUE_MID, IT_DAC_VALUE_HIGH };
    const dac_ChannelId_t channels[ ] =
    {
        DAC_CHANNEL_1,
#if defined(DAC_CHANNEL2_SUPPORT)
        DAC_CHANNEL_2,
#endif /* DAC_CHANNEL2_SUPPORT */
    };
    dac_Config_t     config;

    It_Dac_Get_Config( &config );
    for( uint32_t channelIdx = 0u; ( sizeof( channels ) / sizeof( channels[ 0u ] ) ) > channelIdx; channelIdx++ )
    {
        config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ channels[ channelIdx ] ].ChannelUsed = DAC_FUNCTION_ACTIVE;
        config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ channels[ channelIdx ] ].OutputMode  = DAC_OUTPUT_INTERNAL;
    }
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    for( uint32_t channelIdx = 0u; ( sizeof( channels ) / sizeof( channels[ 0u ] ) ) > channelIdx; channelIdx++ )
    {
        It_Dac_Adc_Init( channels[ channelIdx ], DAC_OUTPUT_INTERNAL );

        for( uint32_t valueIdx = 0u; ( sizeof( values ) / sizeof( values[ 0u ] ) ) > valueIdx; valueIdx++ )
        {
            TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, channels[ channelIdx ], values[ valueIdx ] ) );

            const uint32_t adcResult = It_Dac_Adc_Measure();

            TEST_ASSERT_LESS_THAN( IT_DAC_TOLERANCE_LSB, It_Dac_Get_Diff( values[ valueIdx ], adcResult ) );
        }

        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( itDac_AdcPeriph ) );
    }
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* IT_DAC_ADC_NOT_CONNECTED / DAC1 */
}


/**
 * \brief   Dac_Set_Voltage() with half of the reference voltage is measured by ADC.
 *
 * \details Channel 1 with output mode of the board (pin: ADC1 channel 9, internal: ADC channel
 *          of the DAC output).
 *
 * \par Expected results
 * - ADC result corresponds to half of the reference within IT_DAC_TOLERANCE_LSB.
 */
void It_Dac_Set_Voltage_HalfReference_AdcMeasuresOutput( void )
{
#if defined(DAC1)
    dac_Config_t config;

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    It_Dac_Adc_Init( IT_DAC_CHANNEL, IT_DAC_OUTPUT_MODE );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Voltage( IT_DAC_PERIPH, IT_DAC_CHANNEL, IT_DAC_VREF_MV, IT_DAC_VREF_MV / 2u ) );

    const uint32_t adcResult = It_Dac_Adc_Measure();

    TEST_ASSERT_LESS_THAN( IT_DAC_TOLERANCE_LSB, It_Dac_Get_Diff( IT_DAC_VALUE_MID, adcResult ) );
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* DAC1 */
}

/*------------------------------- DMA transfer -------------------------------*/

/**
 * \brief   DMA transfer with hardware trigger can be started and stopped.
 *
 * \details Channel with TIM6 TRGO trigger and DMA circular buffer. TIM6 is not
 *          running - no DMA request is generated.
 *
 * \par Expected results
 * - Dac_Set_DmaStart() returns DAC_REQUEST_OK, second start DAC_REQUEST_ERROR,
 *   direct data write refused while running.
 * - Dac_Set_DmaStop() returns DAC_REQUEST_OK, no error is reported, direct write possible.
 */
void It_Dac_Set_DmaStart_HwTrigger_StartStopHandled( void )
{
#if defined(DAC1)
    dac_Config_t config;

    It_Dac_Get_Config( &config );
    It_Dac_Set_DmaConfig( &config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ], DAC_BUFFER_MODE_CIRCULAR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DmaStart( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, 0u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStop( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, 0u ) );

    TEST_ASSERT_EQUAL( 0u, itDac_ErrorCnt );
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* DAC1 */
}


/**
 * \brief   One shot DMA buffer output driven by TIM6 TRGO.
 *
 * \details DAC1 channel 1 (default DMA1 channel 3 - STM32L4 request mapping, DMAMUX1 on
 *          STM32L4+), 16 items, TIM6 update event as trigger.
 *
 * \par Expected results
 * - Half transfer and transfer complete callbacks are called once, no error.
 * - Output register contains the last buffer item, the transfer is stopped (direct write
 *   accepted, new start possible - DMA channel disabled after the normal mode transfer).
 */
void It_Dac_DmaOneShot_Tim6Trigger_BufferOutput( void )
{
#if defined(DAC1)
    dac_Config_t config;
    dac_Data_t   outData = 0u;

    It_Dac_Get_Config( &config );
    It_Dac_Set_DmaConfig( &config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ], DAC_BUFFER_MODE_ONE_SHOT );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    It_Dac_Tim6_Start();

    TEST_ASSERT_EQUAL_UINT32( 1u, It_Dac_Wait( &itDac_CompleteCnt, 1u ) );

    /* The last item is moved from the data holding register to the output by the next trigger */
    outData = It_Dac_Wait_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, itDac_DmaBuffer[ IT_DAC_BUF_SIZE - 1u ] );

    It_Dac_Tim6_Stop();

    TEST_ASSERT_EQUAL( itDac_DmaBuffer[ IT_DAC_BUF_SIZE - 1u ], outData );
    TEST_ASSERT_EQUAL_UINT32( 1u, itDac_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itDac_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itDac_ErrorCnt );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, 0u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStop( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* DAC1 */
}


/**
 * \brief   Circular DMA buffer output driven by TIM6 TRGO.
 *
 * \details DAC1 channel 2 (default DMA1 channel 4; channel 1 / DMA1 channel 3 on MCUs with one
 *          DAC channel) with internal output, 16 items in circular mode, TIM6 update event as
 *          trigger. Transfer is stopped after the third transfer complete.
 *
 * \par Expected results
 * - At least 3 transfer complete and 3 half transfer callbacks, no error.
 * - Dac_Set_DmaStop() returns DAC_REQUEST_OK.
 */
void It_Dac_DmaCircular_Tim6Trigger_BufferRepeated( void )
{
#if defined(DAC1)
    dac_Config_t config;

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed       = DAC_FUNCTION_INACTIVE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CIRC_CHANNEL ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CIRC_CHANNEL ].OutputMode  = DAC_OUTPUT_INTERNAL;
    It_Dac_Set_DmaConfig( &config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CIRC_CHANNEL ], DAC_BUFFER_MODE_CIRCULAR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( IT_DAC_PERIPH, IT_DAC_CIRC_CHANNEL ) );
    It_Dac_Tim6_Start();

    TEST_ASSERT_EQUAL_UINT32( 3u, It_Dac_Wait( &itDac_CompleteCnt, 3u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStop( IT_DAC_PERIPH, IT_DAC_CIRC_CHANNEL ) );
    It_Dac_Tim6_Stop();

    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( 3u, itDac_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itDac_ErrorCnt );
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* DAC1 */
}


/**
 * \brief   DMA underrun is detected and reported by the DAC interrupt (TIM6_DAC line).
 *
 * \details Circular transfer of DAC1 channel 1 is running, the DMA channel is disabled behind
 *          the module (Dma_Set_TransferInactive()) - the next triggers are not served by DMA.
 *
 * \par Expected results
 * - ErrorCallback is called with DAC_ERROR_DMA_UNDERRUN, the transfer is stopped (direct
 *   write accepted, underrun flag cleared).
 */
void It_Dac_DmaUnderrun_DmaChannelDisabled_ErrorReported( void )
{
#if defined(DAC1)
    dac_Config_t     config;
    dac_DataConfig_t dataConfig;
    dac_FlagState_t  underrunFlag = DAC_FLAG_ACTIVE;

    It_Dac_Get_Config( &config );
    It_Dac_Set_DmaConfig( &config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ], DAC_BUFFER_MODE_CIRCULAR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DataConfig( IT_DAC_PERIPH, IT_DAC_CHANNEL, &dataConfig ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    It_Dac_Tim6_Start();
    TEST_ASSERT_EQUAL_UINT32( 1u, It_Dac_Wait( &itDac_CompleteCnt, 1u ) );

    TEST_ASSERT_EQUAL( DMA_REQUEST_OK, Dma_Set_TransferInactive( (dma_PeriphId_t)DAC_DMA_BIT_MASK_DECODE_DMA( (dac_DmaCode_t)dataConfig.Dma ),
                                                                 (dma_ChannelId_t)DAC_DMA_BIT_MASK_DECODE_DMA_CHANNEL( (dac_DmaCode_t)dataConfig.Dma ) ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, It_Dac_Wait( &itDac_ErrorCnt, 1u ) );
    It_Dac_Tim6_Stop();

    TEST_ASSERT_EQUAL( DAC_ERROR_DMA_UNDERRUN, itDac_LastError );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_Flag( IT_DAC_PERIPH, IT_DAC_CHANNEL, DAC_FLAG_DMA_UNDERRUN, &underrunFlag ) );
    TEST_ASSERT_EQUAL( DAC_FLAG_INACTIVE, underrunFlag );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, 0u ) );
#else
    TEST_IGNORE_MESSAGE( IT_DAC_NO_DAC_MSG );
#endif /* DAC1 */
}

/* ========================== LOCAL FUNCTIONS =============================== */

#if defined(DAC1)
/**
 * \brief Fills module configuration: default configuration (DAC1 channel 1, no trigger, 12-bit
 *        right aligned, direct transfer) with the output mode of the board.
 *
 * \param config [out]: Module configuration
 */
static void It_Dac_Get_Config( dac_Config_t * const config )
{
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DefaultConfig( config ) );

    config->PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].OutputMode = IT_DAC_OUTPUT_MODE;
}


/**
 * \brief Configures DMA transfer of a channel with TIM6 trigger, test buffer and all callbacks
 *        (DMA channel of the DAC channel is kept from the default configuration).
 *
 * \param channelConfig [in,out]: Channel configuration
 * \param bufferMode        [in]: One shot / circular
 */
static void It_Dac_Set_DmaConfig( dac_ChannelConfig_t * const channelConfig, dac_BufferMode_t bufferMode )
{
    channelConfig->TriggerId                           = DAC_TRIGGER_TIM6_TRGO;
    channelConfig->DataConfig.TransferMode             = DAC_TRANSFER_MODE_DMA;
    channelConfig->DataConfig.DataBuffer               = itDac_DmaBuffer;
    channelConfig->DataConfig.BufferSize               = IT_DAC_BUF_SIZE;
    channelConfig->DataConfig.BufferMode               = bufferMode;
    channelConfig->DataConfig.DmaPriority              = DAC_DMA_PRIORITY_HIGH;
    channelConfig->DataConfig.IrqPriority              = IT_DAC_IRQ_PRIO;
    channelConfig->DataConfig.HalfTransferCallback     = It_Dac_HalfCallback;
    channelConfig->DataConfig.TransferCompleteCallback = It_Dac_CompleteCallback;
    channelConfig->DataConfig.ErrorCallback            = It_Dac_ErrorCallback;
}


/**
 * \brief Initializes the ADC measuring a DAC output by Adc module: 12-bit, single conversion,
 *        software start, polling without buffer, ADC clock HCLK / 4, the longest sampling time.
 *        Pin output modes: ADC1 channel 9 (PA4, channel 1 only). Internal output modes: ADC
 *        channel of the internal DAC output (ADC2 / ADC1 channel 17 / 18).
 *
 * \param dacChannel [in]: Measured DAC channel
 * \param outputMode [in]: Output mode of the DAC channel
 */
static void It_Dac_Adc_Init( dac_ChannelId_t dacChannel, dac_OutputMode_t outputMode )
{
    adc_Config_t       config       = { 0 };
    adc_Channel_t      adcChannel   = IT_DAC_ADC_PIN_CHANNEL;

    if( ( DAC_OUTPUT_PIN_BUFFERED   == outputMode ) ||
        ( DAC_OUTPUT_PIN_UNBUFFERED == outputMode )    )
    {
        itDac_AdcPeriph = IT_DAC_ADC_PIN_PERIPH;
        adcChannel      = IT_DAC_ADC_PIN_CHANNEL;
    }
    else
    {
        itDac_AdcPeriph = IT_DAC_ADC_INT_PERIPH;
#if !defined(IT_DAC_ADC_NOT_CONNECTED)
        adcChannel      = itDac_AdcIntChannelLut[ dacChannel ];
#else
        /* The DAC outputs are not connected to the ADC (STM32L4R / L4S) - the internal outputs are not measured */
        (void)dacChannel;
        adcChannel      = IT_DAC_ADC_PIN_CHANNEL;
#endif /* IT_DAC_ADC_NOT_CONNECTED */
    }

    for( adc_PeriphId_t periphIdx = ADC_PERIPH_1; ADC_PERIPH_CNT > periphIdx; periphIdx++ )
    {
        config.PeriphConfig[ periphIdx ].PeriphId = periphIdx;
    }

    adc_PeriphConfig_t * const periphCfg = &config.PeriphConfig[ itDac_AdcPeriph ];

    periphCfg->Resolution              = ADC_RESOLUTION_12BIT;
    periphCfg->RegTriggerMode          = ADC_REG_TRIGGER_MODE_SINGLE;
    periphCfg->RegTriggerEdge          = ADC_TRIGGER_EDGE_RISING;
    periphCfg->RegTriggerId            = ADC_REG_TRIGGER_SOFTWARE;
    periphCfg->InjTriggerMode          = ADC_INJ_TRIGGER_MODE_CONTINUOUS;
    periphCfg->InjTriggerId            = ADC_INJ_TRIGGER_SOFTWARE;
    periphCfg->InjTriggerEdge          = ADC_TRIGGER_EDGE_RISING;
    periphCfg->InjChannelsCnt          = 0u;
    periphCfg->DataConfig.TransferMode = ADC_TRANSFER_MODE_POLL;
    periphCfg->DataConfig.DataBuffer   = ADC_NULL_PTR;
    periphCfg->DataConfig.BufferSize   = 0u;
    periphCfg->DataConfig.BufferMode   = ADC_BUFFER_MODE_ONE_SHOT;

    periphCfg->RegChannelsCnt                    = 1u;
    periphCfg->RegChannels[ 0u ].Channel         = adcChannel;
    periphCfg->RegChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_640_5_CYCLES;

    config.ClockSource  = ADC_CLK_SRC_HCLK;
    config.ClockDivider = ADC_CLK_DIV_4;

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );
}


/**
 * \brief Converts the ADC channel of the last It_Dac_Adc_Init() (polling, the first conversion
 *        after initialization is discarded - STM32G4 device errata ES0430 2.7.9, conversion done
 *        late after calibration; STM32L4 errata sheets not reviewed yet).
 *
 * \return ADC result (12-bit)
 */
static uint32_t It_Dac_Adc_Measure( void )
{
    adc_Data_t adcResult = 0u;

    for( uint32_t convIdx = 0u; 2u > convIdx; convIdx++ )
    {
        adc_FlagState_t eocFlag = ADC_FLAG_INACTIVE;

        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( itDac_AdcPeriph ) );

        for( uint32_t loopIdx = 0u; IT_DAC_WAIT_LOOPS > loopIdx; loopIdx++ )
        {
            TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( itDac_AdcPeriph, ADC_FLAG_REG_EOC, &eocFlag ) );

            if( ADC_FLAG_ACTIVE == eocFlag )
            {
                break;
            }
            else
            {
                /* Conversion is running */
            }
        }

        TEST_ASSERT_EQUAL( ADC_FLAG_ACTIVE, eocFlag );

        /* EOC is cleared by the data register read */
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_RegData( itDac_AdcPeriph, &adcResult ) );
    }

    return ( (uint32_t)adcResult );
}


/**
 * \brief Returns absolute difference of two values.
 *
 * \param valueA [in]: First value
 * \param valueB [in]: Second value
 *
 * \return |valueA - valueB|
 */
static uint32_t It_Dac_Get_Diff( uint32_t valueA, uint32_t valueB )
{
    uint32_t diff = 0u;

    if( valueA > valueB )
    {
        diff = valueA - valueB;
    }
    else
    {
        diff = valueB - valueA;
    }

    return ( diff );
}


/**
 * \brief Starts TIM6 with update event as TRGO (hardware trigger of the DAC).
 */
static void It_Dac_Tim6_Start( void )
{
    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Set_PeriphActive( RCC_PERIPH_TIM6 ) );

    LL_TIM_SetPrescaler( TIM6, IT_DAC_TIM6_PRESCALER );
    LL_TIM_SetAutoReload( TIM6, IT_DAC_TIM6_AUTORELOAD );
    LL_TIM_SetTriggerOutput( TIM6, LL_TIM_TRGO_UPDATE );
    LL_TIM_GenerateEvent_UPDATE( TIM6 );
    LL_TIM_EnableCounter( TIM6 );
}


/**
 * \brief Stops TIM6 (no further DAC triggers).
 */
static void It_Dac_Tim6_Stop( void )
{
    LL_TIM_DisableCounter( TIM6 );
}


/**
 * \brief Waits until a callback counter reaches the expected value (or timeout).
 *
 * \param counter     [in]: Counter incremented by a callback
 * \param expectedCnt [in]: Expected value
 *
 * \return Counter value at the end of the wait
 */
static uint32_t It_Dac_Wait( volatile uint32_t * const counter, uint32_t expectedCnt )
{
    for( uint32_t loopIdx = 0u; IT_DAC_WAIT_LOOPS > loopIdx; loopIdx++ )
    {
        if( expectedCnt <= *counter )
        {
            break;
        }
        else
        {
            /* Event has not been reported yet */
        }
    }

    return ( *counter );
}


/**
 * \brief Waits until the output register of a channel contains the expected value (or
 *        timeout) - the output is updated a few clock cycles after the trigger.
 *
 * \param periphId     [in]: DAC peripheral identification
 * \param channelId    [in]: DAC channel identification
 * \param expectedData [in]: Expected output value (12-bit right aligned)
 *
 * \return Output value at the end of the wait
 */
static dac_Data_t It_Dac_Wait_OutputData( dac_PeriphId_t periphId, dac_ChannelId_t channelId, dac_Data_t expectedData )
{
    dac_Data_t outData = 0xFFFFu;

    for( uint32_t loopIdx = 0u; IT_DAC_WAIT_LOOPS > loopIdx; loopIdx++ )
    {
        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( periphId, channelId, &outData ) );

        if( expectedData == outData )
        {
            break;
        }
        else
        {
            /* Output has not been updated yet */
        }
    }

    return ( outData );
}


/** \brief DAC half transfer callback */
static void It_Dac_HalfCallback( void )
{
    itDac_HalfCnt++;
}


/** \brief DAC transfer complete callback */
static void It_Dac_CompleteCallback( void )
{
    itDac_CompleteCnt++;
}


/**
 * \brief DAC data transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void It_Dac_ErrorCallback( dac_ErrorId_t errorId )
{
    itDac_LastError = errorId;
    itDac_ErrorCnt++;
}
#endif /* DAC1 */
