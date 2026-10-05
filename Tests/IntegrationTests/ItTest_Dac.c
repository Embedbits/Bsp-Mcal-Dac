/**
 * \author Mr.Nobody
 * \file ItTest_Dac.c
 * \ingroup Dac
 * \brief Integration tests of Digital-to-Analog Converter (DAC) module on target (STM32F4).
 *
 * Dac module runs on the MCU together with real RCC, NVIC, GPIO and DMA modules and
 * hardware. Tests verify behavior which cannot be verified by unit tests (emulated
 * registers): output register update, trigger handling, wave generation, output voltage
 * and DMA buffer output driven by a hardware trigger.
 *
 * No external wiring is used:
 * - DAC_OUT1 (PA4) and DAC_OUT2 (PA5) are ADC1 input channels 4 and 5 - the output is
 *   measured by ADC1 (both converters use VREF+ as reference, so the DAC value and the ADC
 *   result are compared directly). ADC1 is driven through Adc module (polling mode).
 * - TIM6 TRGO (update event) is the hardware trigger of DMA transfers. TIM6 is driven
 *   directly through LL (TRGO configuration is not provided by Tim module).
 *
 * \note STM32F4DISCOVERY: PA4 is connected to the LRCK input of the audio codec (kept in
 *       reset), PA5 to the SPI clock input of the accelerometer - both are high impedance
 *       inputs, the measurement is not influenced.
 *
 * Used resources: DAC (both channels), ADC1 (channels 4, 5), TIM6, DMA1 streams 5 and 6.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Dac_Port.h"                       /* Module under test              */
#include "Rcc_Port.h"                       /* Clock of TIM6, DAC clock state */
#include "Dma_Port.h"                       /* DMA stream of the DAC (underrun test) */
#include "Adc_Port.h"                       /* ADC1 used for output measurement */
#include "Stm32_tim.h"                      /* TIM6 used as hardware trigger  */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void         It_Dac_Get_Config       ( dac_Config_t * const config );
static void         It_Dac_Set_DmaConfig    ( dac_ChannelConfig_t * const channelConfig, dac_BufferMode_t bufferMode );
static void         It_Dac_Adc_Init         ( adc_ChannelId_t adcChannel );
static uint32_t     It_Dac_Adc_Measure      ( void );
static void         It_Dac_Tim6_Start       ( void );
static void         It_Dac_Tim6_Stop        ( void );
static uint32_t     It_Dac_Wait             ( volatile uint32_t * const counter, uint32_t expectedCnt );
static uint32_t     It_Dac_Get_Diff         ( uint32_t valueA, uint32_t valueB );

static void         It_Dac_HalfCallback     ( void );
static void         It_Dac_CompleteCallback ( void );
static void         It_Dac_ErrorCallback    ( dac_ErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** DAC peripheral and channel under test */
#define IT_DAC_PERIPH                       ( DAC_PERIPH_1 )
#define IT_DAC_CHANNEL                      ( DAC_CHANNEL_1 )

/** ADC1 channels connected to DAC_OUT1 (PA4) and DAC_OUT2 (PA5) */
#define IT_DAC_ADC_CHANNEL_OUT1             ( ADC_CHANNEL_4 )
#define IT_DAC_ADC_CHANNEL_OUT2             ( ADC_CHANNEL_5 )

/** Maximal difference between DAC value and ADC result [LSB, 12-bit] */
#define IT_DAC_TOLERANCE_LSB                ( 60u )

/** Output values used for the measurement (buffered output linear range) */
#define IT_DAC_VALUE_LOW                    ( 600u )
#define IT_DAC_VALUE_MID                    ( 2048u )
#define IT_DAC_VALUE_HIGH                   ( 3500u )

/** Analog supply / reference voltage of STM32F4DISCOVERY [mV] */
#define IT_DAC_VREF_MV                      ( 3000u )

/** Size of the DMA buffer */
#define IT_DAC_BUF_SIZE                     ( 16u )

/** TIM6 prescaler and auto-reload - trigger period of ~0.1 ms - 1 ms (APB1 timer clock 16 - 84 MHz) */
#define IT_DAC_TIM6_PRESCALER               ( 999u )
#define IT_DAC_TIM6_AUTORELOAD              ( 9u )

/** Maximal count of wait loop iterations (ADC conversion, DMA events) */
#define IT_DAC_WAIT_LOOPS                   ( 20000000u )

/** Interrupt priority of the DAC interrupt */
#define IT_DAC_IRQ_PRIO                     ( 5u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** DMA output buffer */
static dac_Data_t               itDac_DmaBuffer[ IT_DAC_BUF_SIZE ];

/** Counts of callback calls and the last error */
static volatile uint32_t        itDac_HalfCnt;
static volatile uint32_t        itDac_CompleteCnt;
static volatile uint32_t        itDac_ErrorCnt;
static volatile dac_ErrorId_t   itDac_LastError;

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    itDac_HalfCnt     = 0u;
    itDac_CompleteCnt = 0u;
    itDac_ErrorCnt    = 0u;
    itDac_LastError   = DAC_ERROR_CNT;

    for( uint32_t dataIdx = 0u; IT_DAC_BUF_SIZE > dataIdx; dataIdx++ )
    {
        itDac_DmaBuffer[ dataIdx ] = (dac_Data_t)( ( dataIdx + 1u ) * 200u );
    }
}


void tearDown( void )
{
    /* Every test case runs after system reset */
}

/* =============================== TESTS ==================================== */

/*------------------------------ Configuration -------------------------------*/

/**
 * \brief   Dac_Init() on target configures clock and channel.
 *
 * \details Initializes the module with the default configuration (channel 1,
 *          buffered pin output, no trigger) and reads the configuration back.
 *
 * \par Expected results
 * - Dac_Init() returns DAC_REQUEST_OK, DAC bus clock is enabled in RCC.
 * - Clock source PCLK1, channel enabled, buffered pin output, no trigger, sample and hold
 *   inactive, no DMA underrun.
 */
void It_Dac_Init_DefaultConfig_ReadBack( void )
{
    dac_Config_t           config;
    dac_ClkSrc_t           clkSrc       = DAC_CLK_SRC_CNT;
    dac_FunctionState_t    channelState = DAC_FUNCTION_INACTIVE;
    dac_OutputMode_t       outputMode   = DAC_OUTPUT_CNT;
    dac_TriggerId_t        triggerSrc   = DAC_TRIGGER_CNT;
    dac_FlagState_t        underrunFlag = DAC_FLAG_ACTIVE;
    dac_SampleHoldConfig_t shConfig     = { .State = DAC_FUNCTION_ACTIVE };
    rcc_FunctionState_t    rccState     = RCC_FUNCTION_INACTIVE;

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphState( RCC_PERIPH_DAC, &rccState ) );
    TEST_ASSERT_EQUAL( RCC_FUNCTION_ACTIVE, rccState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( DAC_CLK_SRC_PCLK1, clkSrc );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ChannelState( IT_DAC_PERIPH, IT_DAC_CHANNEL, &channelState ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE, channelState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputMode( IT_DAC_PERIPH, IT_DAC_CHANNEL, &outputMode ) );
    TEST_ASSERT_EQUAL( DAC_OUTPUT_PIN_BUFFERED, outputMode );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TriggerSrc( IT_DAC_PERIPH, IT_DAC_CHANNEL, &triggerSrc ) );
    TEST_ASSERT_EQUAL( DAC_TRIGGER_NONE, triggerSrc );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_SampleHoldConfig( IT_DAC_PERIPH, IT_DAC_CHANNEL, &shConfig ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_INACTIVE, shConfig.State );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_Flag( IT_DAC_PERIPH, IT_DAC_CHANNEL, DAC_FLAG_DMA_UNDERRUN, &underrunFlag ) );
    TEST_ASSERT_EQUAL( DAC_FLAG_INACTIVE, underrunFlag );
}


/**
 * \brief   Dac_Init() on target rejects invalid configurations and second initialization.
 *
 * \details
 * 1. NULL configuration.
 * 2. Initial value above 12-bit range.
 * 3. DMA transfer without hardware trigger.
 * 4. Output connected to on-chip peripherals (not supported by STM32F4).
 * 5. Valid initialization, then second initialization while the channel is enabled.
 *
 * \par Expected results
 * 1. - 4. DAC_REQUEST_ERROR, channel stays disabled.
 * 5. DAC_REQUEST_OK, then DAC_REQUEST_ERROR.
 */
void It_Dac_Init_InvalidConfig_ReturnsError( void )
{
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
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].OutputMode = DAC_OUTPUT_INTERNAL;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ChannelState( IT_DAC_PERIPH, IT_DAC_CHANNEL, &channelState ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_INACTIVE, channelState );

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );
}


/**
 * \brief   Dac_Deinit() on target disables the channel and the DAC clock, new initialization possible.
 *
 * \par Expected results
 * - DAC bus clock is disabled after Dac_Deinit(), second Dac_Init() returns DAC_REQUEST_OK and
 *   the channel is enabled again.
 */
void It_Dac_Deinit_InitializedPeripheral_InitializationPossibleAgain( void )
{
    dac_Config_t        config;
    dac_FunctionState_t channelState = DAC_FUNCTION_INACTIVE;
    rcc_FunctionState_t rccState     = RCC_FUNCTION_ACTIVE;

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Deinit( IT_DAC_PERIPH ) );

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphState( RCC_PERIPH_DAC, &rccState ) );
    TEST_ASSERT_EQUAL( RCC_FUNCTION_INACTIVE, rccState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ChannelState( IT_DAC_PERIPH, IT_DAC_CHANNEL, &channelState ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE, channelState );
}


/**
 * \brief   Features not available on STM32F4 are refused on target.
 *
 * \par Expected results
 * - Calibration, trimming, sample and hold clock, active sample and hold, internal output
 *   and flags other than DMA underrun return DAC_REQUEST_ERROR, the channel keeps working.
 */
void It_Dac_UnsupportedFeatures_ReturnError( void )
{
    dac_Config_t                 config;
    dac_TrimValue_t              trimValue    = 0u;
    dac_FlagState_t              flagState    = DAC_FLAG_INACTIVE;
    dac_Data_t                   outData      = 0u;
    const dac_SampleHoldConfig_t shConfig     = { .State = DAC_FUNCTION_ACTIVE, .SampleTime = 1u, .HoldTime = 1u, .RefreshTime = 1u };

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_Calibration( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_TrimValue( IT_DAC_PERIPH, IT_DAC_CHANNEL, 1u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_TrimValue( IT_DAC_PERIPH, IT_DAC_CHANNEL, &trimValue ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ShClockSource( DAC_SH_CLK_SRC_LSI ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SampleHoldConfig( IT_DAC_PERIPH, IT_DAC_CHANNEL, &shConfig ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_Flag( IT_DAC_PERIPH, IT_DAC_CHANNEL, DAC_FLAG_READY, &flagState ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, 0x0321u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, &outData ) );
    TEST_ASSERT_EQUAL_HEX16( 0x0321u, outData );
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
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, &outData ) );
    TEST_ASSERT_EQUAL( IT_DAC_VALUE_MID, outData );
}


/**
 * \brief   Dac_Set_DualData() and dual software trigger update outputs of both channels.
 *
 * \details Both channels with software trigger.
 *
 * \par Expected results
 * - Output registers keep 0 after Dac_Set_DualData(), contain the written values after
 *   Dac_Set_DualSwTrigger().
 */
void It_Dac_Set_DualData_BothChannels_OutputsUpdated( void )
{
    dac_Config_t config;
    dac_Data_t   outData = 0xFFFFu;

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].TriggerId   = DAC_TRIGGER_SOFTWARE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].TriggerId   = DAC_TRIGGER_SOFTWARE;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DualData( IT_DAC_PERIPH, 0x0123u, 0x0FEDu ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, DAC_CHANNEL_2, &outData ) );
    TEST_ASSERT_EQUAL_HEX16( 0u, outData );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DualSwTrigger( IT_DAC_PERIPH ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, DAC_CHANNEL_1, &outData ) );
    TEST_ASSERT_EQUAL_HEX16( 0x0123u, outData );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, DAC_CHANNEL_2, &outData ) );
    TEST_ASSERT_EQUAL_HEX16( 0x0FEDu, outData );
}


/**
 * \brief   Triangle wave generation changes the output on every software trigger.
 *
 * \details Channel with software trigger, triangle wave amplitude 4095 and data
 *          holding register 0 is triggered several times.
 *
 * \par Expected results
 * - Output value increments by 1 on every trigger (rising part of the triangle).
 */
void It_Dac_Set_WaveMode_Triangle_OutputFollowsTriangle( void )
{
    dac_Config_t config;
    dac_Data_t   outData  = 0u;
    dac_Data_t   lastData = 0u;

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].TriggerId     = DAC_TRIGGER_SOFTWARE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].WaveMode      = DAC_WAVE_TRIANGLE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].WaveAmplitude = DAC_WAVE_AMPLITUDE_4095;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].InitValue     = 0u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_SwTrigger( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, &lastData ) );

    for( uint32_t triggerIdx = 0u; 8u > triggerIdx; triggerIdx++ )
    {
        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_SwTrigger( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, &outData ) );
        TEST_ASSERT_EQUAL( lastData + 1u, outData );
        lastData = outData;
    }
}

/*------------------------------ Output voltage ------------------------------*/

/**
 * \brief   Output voltage of buffered pin output measured by ADC matches the DAC value.
 *
 * \details DAC_OUT1 (PA4) is converted by ADC1 channel 4 for low, middle and high output value.
 *
 * \par Expected results
 * - ADC result differs from the DAC value by less than IT_DAC_TOLERANCE_LSB.
 */
void It_Dac_Set_Data_BufferedOutput_AdcMeasuresOutput( void )
{
    const dac_Data_t testValues[ ] = { IT_DAC_VALUE_LOW, IT_DAC_VALUE_MID, IT_DAC_VALUE_HIGH };
    dac_Config_t     config;

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    It_Dac_Adc_Init( IT_DAC_ADC_CHANNEL_OUT1 );

    for( uint32_t valueIdx = 0u; ( sizeof( testValues ) / sizeof( dac_Data_t ) ) > valueIdx; valueIdx++ )
    {
        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, testValues[ valueIdx ] ) );

        const uint32_t adcResult = It_Dac_Adc_Measure();

        TEST_ASSERT_LESS_THAN( IT_DAC_TOLERANCE_LSB, It_Dac_Get_Diff( testValues[ valueIdx ], adcResult ) );
    }
}


/**
 * \brief   Output voltage of unbuffered channel 2 measured by ADC matches the DAC value.
 *
 * \details DAC_OUT2 (PA5) with output buffer disabled is converted by ADC1 channel 5
 *          (longest sampling time - output impedance of the unbuffered output).
 *
 * \par Expected results
 * - Dac_Get_OutputMode() returns DAC_OUTPUT_PIN_UNBUFFERED.
 * - ADC result differs from the DAC value by less than IT_DAC_TOLERANCE_LSB.
 */
void It_Dac_Set_Data_UnbufferedOutput_AdcMeasuresOutput( void )
{
    dac_Config_t     config;
    dac_OutputMode_t outputMode = DAC_OUTPUT_CNT;

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_INACTIVE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].OutputMode  = DAC_OUTPUT_PIN_UNBUFFERED;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].InitValue   = IT_DAC_VALUE_MID;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    It_Dac_Adc_Init( IT_DAC_ADC_CHANNEL_OUT2 );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputMode( IT_DAC_PERIPH, DAC_CHANNEL_2, &outputMode ) );
    TEST_ASSERT_EQUAL( DAC_OUTPUT_PIN_UNBUFFERED, outputMode );

    const uint32_t adcResult = It_Dac_Adc_Measure();

    TEST_ASSERT_LESS_THAN( IT_DAC_TOLERANCE_LSB, It_Dac_Get_Diff( IT_DAC_VALUE_MID, adcResult ) );
}


/**
 * \brief   Dac_Set_Voltage() sets the output voltage, Dac_Get_OutputVoltage() reads it back.
 *
 * \par Expected results
 * - Output voltage read back equals the required voltage within 1 mV.
 * - ADC result corresponds to half of the reference within IT_DAC_TOLERANCE_LSB.
 */
void It_Dac_Set_Voltage_HalfReference_OutputVoltageMatches( void )
{
    dac_Config_t     config;
    dac_Voltage_mV_t outVoltage = 0u;

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    It_Dac_Adc_Init( IT_DAC_ADC_CHANNEL_OUT1 );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Voltage( IT_DAC_PERIPH, IT_DAC_CHANNEL, IT_DAC_VREF_MV, IT_DAC_VREF_MV / 2u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputVoltage( IT_DAC_PERIPH, IT_DAC_CHANNEL, IT_DAC_VREF_MV, &outVoltage ) );
    TEST_ASSERT_UINT_WITHIN( 1u, IT_DAC_VREF_MV / 2u, outVoltage );

    const uint32_t adcResult = It_Dac_Adc_Measure();

    TEST_ASSERT_LESS_THAN( IT_DAC_TOLERANCE_LSB, It_Dac_Get_Diff( IT_DAC_VALUE_MID, adcResult ) );
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
}


/**
 * \brief   One shot DMA buffer output driven by TIM6 TRGO.
 *
 * \details Channel 1, 16 items, TIM6 update event as trigger.
 *
 * \par Expected results
 * - Half transfer and transfer complete callbacks are called once, no error.
 * - Output register contains the last buffer item, the transfer is stopped (direct write
 *   accepted, new start possible).
 */
void It_Dac_DmaOneShot_Tim6Trigger_BufferOutput( void )
{
    dac_Config_t config;
    dac_Data_t   outData = 0u;

    It_Dac_Get_Config( &config );
    It_Dac_Set_DmaConfig( &config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ], DAC_BUFFER_MODE_ONE_SHOT );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    It_Dac_Tim6_Start();

    TEST_ASSERT_EQUAL_UINT32( 1u, It_Dac_Wait( &itDac_CompleteCnt, 1u ) );

    /* The last item is moved from the data holding register to the output by the next trigger */
    for( uint32_t loopIdx = 0u; ( IT_DAC_WAIT_LOOPS > loopIdx ) && ( itDac_DmaBuffer[ IT_DAC_BUF_SIZE - 1u ] != outData ); loopIdx++ )
    {
        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( IT_DAC_PERIPH, IT_DAC_CHANNEL, &outData ) );
    }

    It_Dac_Tim6_Stop();

    TEST_ASSERT_EQUAL( itDac_DmaBuffer[ IT_DAC_BUF_SIZE - 1u ], outData );
    TEST_ASSERT_EQUAL_UINT32( 1u, itDac_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itDac_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itDac_ErrorCnt );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStop( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
}


/**
 * \brief   Circular DMA buffer output of channel 2 driven by TIM6 TRGO.
 *
 * \details Channel 2, 16 items in circular mode, TIM6 update event as trigger. Transfer is
 *          stopped after the third transfer complete.
 *
 * \par Expected results
 * - At least 3 transfer complete and 3 half transfer callbacks, no error.
 * - Dac_Set_DmaStop() returns DAC_REQUEST_OK.
 */
void It_Dac_DmaCircular_Tim6Trigger_BufferRepeated( void )
{
    dac_Config_t config;

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_INACTIVE;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    It_Dac_Set_DmaConfig( &config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ], DAC_BUFFER_MODE_CIRCULAR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( IT_DAC_PERIPH, DAC_CHANNEL_2 ) );
    It_Dac_Tim6_Start();

    TEST_ASSERT_EQUAL_UINT32( 3u, It_Dac_Wait( &itDac_CompleteCnt, 3u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStop( IT_DAC_PERIPH, DAC_CHANNEL_2 ) );
    It_Dac_Tim6_Stop();

    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( 3u, itDac_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itDac_ErrorCnt );
}


/**
 * \brief   DMA underrun is detected and reported by the DAC interrupt.
 *
 * \details Circular transfer of channel 1 is running, the DMA stream is disabled behind the
 *          module (Dma_Set_TransferInactive()) - the next triggers are not served by DMA.
 *
 * \par Expected results
 * - ErrorCallback is called with DAC_ERROR_DMA_UNDERRUN, the transfer is stopped (direct
 *   write accepted, underrun flag cleared).
 */
void It_Dac_DmaUnderrun_StreamDisabled_ErrorReported( void )
{
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

    TEST_ASSERT_EQUAL( DMA_REQUEST_OK, Dma_Set_TransferInactive( (dma_PeriphId_t)dataConfig.DmaPeriphId, (dma_ChannelId_t)dataConfig.DmaChannelId ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, It_Dac_Wait( &itDac_ErrorCnt, 1u ) );
    It_Dac_Tim6_Stop();

    TEST_ASSERT_EQUAL( DAC_ERROR_DMA_UNDERRUN, itDac_LastError );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_Flag( IT_DAC_PERIPH, IT_DAC_CHANNEL, DAC_FLAG_DMA_UNDERRUN, &underrunFlag ) );
    TEST_ASSERT_EQUAL( DAC_FLAG_INACTIVE, underrunFlag );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, 0u ) );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Fills module configuration: default configuration (channel 1 buffered pin
 *        output, no trigger, 12-bit right aligned, direct transfer).
 *
 * \param config [out]: Module configuration
 */
static void It_Dac_Get_Config( dac_Config_t * const config )
{
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DefaultConfig( config ) );
}


/**
 * \brief Configures DMA transfer of a channel with TIM6 trigger, test buffer and all callbacks
 *        (DMA stream of the channel is kept from the default configuration).
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
 * \brief Initializes ADC1 by Adc module: 12-bit, single conversion, software start, polling
 *        without buffer, ADC clock PCLK2 / 4, one regular channel with the longest sampling
 *        time (pin of the DAC output is configured as analog by Dac and Adc).
 *
 * \param adcChannel [in]: ADC1 channel connected to the measured DAC output
 */
static void It_Dac_Adc_Init( adc_ChannelId_t adcChannel )
{
    adc_Config_t config = { 0 };

    for( adc_PeriphId_t periphIdx = ADC_PERIPH_1; ADC_PERIPH_CNT > periphIdx; periphIdx++ )
    {
        config.PeriphConfig[ periphIdx ].PeriphId = periphIdx;
    }

    adc_PeriphConfig_t * const periphCfg = &config.PeriphConfig[ ADC_PERIPH_1 ];

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
    periphCfg->RegChannels[ 0u ].ChannelId       = adcChannel;
    periphCfg->RegChannels[ 0u ].ChannelInput    = ADC_CHANNEL_INPUT_PIN_SINGLE;
    periphCfg->RegChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_480_CYCLES;

    config.ClockSource  = ADC_CLK_SRC_PCLK2;
    config.ClockDivider = ADC_CLK_DIV_4;

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );
}


/**
 * \brief Converts the ADC1 channel (polling, the first conversion after initialization is
 *        discarded).
 *
 * \return ADC result (12-bit)
 */
static uint32_t It_Dac_Adc_Measure( void )
{
    adc_Data_t adcResult = 0u;

    for( uint32_t convIdx = 0u; 2u > convIdx; convIdx++ )
    {
        adc_FlagState_t eocFlag = ADC_FLAG_INACTIVE;
        uint32_t        loopIdx = 0u;

        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( ADC_PERIPH_1 ) );

        while( ( IT_DAC_WAIT_LOOPS > loopIdx ) && ( ADC_FLAG_INACTIVE == eocFlag ) )
        {
            TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( ADC_PERIPH_1, ADC_FLAG_REG_EOC, &eocFlag ) );
            loopIdx++;
        }

        TEST_ASSERT_EQUAL( ADC_FLAG_ACTIVE, eocFlag );

        /* EOC is cleared by the data register read */
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_RegData( ADC_PERIPH_1, &adcResult ) );
    }

    return ( ( uint32_t ) adcResult );
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
    for( uint32_t loopIdx = 0u; ( IT_DAC_WAIT_LOOPS > loopIdx ) && ( expectedCnt > *counter ); loopIdx++ )
    {
        /* Busy wait for the interrupt */
    }

    return ( *counter );
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
