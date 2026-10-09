/**
 * \author Mr.Nobody
 * \file ItTest_Dac.c
 * \ingroup Dac
 * \brief Integration tests of Digital-to-Analog Converter (DAC) module on target.
 *
 * Dac module runs on the MCU together with real RCC, NVIC, GPIO, GPDMA and ADC
 * modules and hardware. Tests verify behavior which cannot be verified by unit
 * tests (emulated registers): channel ready / output register update, offset
 * calibration, trigger handling, wave generation and output voltage.
 *
 * No external wiring is used - DAC1_OUT1 (PA4) is the same pin as ADC1 input
 * channel 9, the output is measured by ADC1 (both converters use VREF+ as
 * reference, so the DAC value and the ADC result are compared directly).
 *
 * \note DMA buffer output needs a hardware trigger (timer TRGO). TRGO
 *       configuration is not provided by Tim module yet (TIM-IRQ-02), so only
 *       DMA start / stop handling is verified.
 *
 * Used resources: DAC1, ADC1 (channel 9), GPDMA1 channel 6 (DMA mode).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Dac_Port.h"                       /* Module under test              */
#include "Adc_Port.h"                       /* Output measurement             */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void         It_Dac_Get_Config       ( dac_Config_t * const config );
static void         It_Dac_Adc_Init         ( void );
static adc_Data_t   It_Dac_Adc_Measure      ( void );
static uint32_t     It_Dac_Get_Diff         ( uint32_t valueA, uint32_t valueB );

static void         It_Dac_AdcCompleteCallback ( void );
static void         It_Dac_AdcErrorCallback    ( adc_ErrorId_t errorId );
static void         It_Dac_ErrorCallback       ( dac_ErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** DAC peripheral and channel under test */
#define IT_DAC_PERIPH                       ( DAC_PERIPH_1 )
#define IT_DAC_CHANNEL                      ( DAC_CHANNEL_1 )

/** ADC peripheral and channel connected to DAC1_OUT1 (PA4) */
#define IT_DAC_ADC_PERIPH                   ( ADC_PERIPH_1 )
#define IT_DAC_ADC_CHANNEL                  ( ADC_CHANNEL_9 )

/** Maximal difference between DAC value and ADC result [LSB, 12-bit] */
#define IT_DAC_TOLERANCE_LSB                ( 60u )

/** Output values used for the measurement (buffered output linear range) */
#define IT_DAC_VALUE_LOW                    ( 600u )
#define IT_DAC_VALUE_MID                    ( 2048u )
#define IT_DAC_VALUE_HIGH                   ( 3500u )

/** Analog supply / reference voltage of NUCLEO boards [mV] */
#define IT_DAC_VREF_MV                      ( 3300u )

/** Size of the DMA buffer */
#define IT_DAC_BUF_SIZE                     ( 16u )

/** Maximal count of wait loop iterations */
#define IT_DAC_WAIT_LOOPS                   ( 1000000u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** ADC conversion result */
static adc_Data_t               itDac_AdcBuffer[ 1u ];

/** DMA output buffer */
static dac_Data_t               itDac_DmaBuffer[ IT_DAC_BUF_SIZE ];

/** Counts of callback calls */
static volatile uint32_t        itDac_AdcCompleteCnt;
static volatile uint32_t        itDac_AdcErrorCnt;
static volatile uint32_t        itDac_ErrorCnt;

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    itDac_AdcCompleteCnt = 0u;
    itDac_AdcErrorCnt    = 0u;
    itDac_ErrorCnt       = 0u;

    for( uint32_t dataIdx = 0u; IT_DAC_BUF_SIZE > dataIdx; dataIdx++ )
    {
        itDac_DmaBuffer[ dataIdx ] = (dac_Data_t)( dataIdx * 256u );
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
 * - Dac_Init() returns DAC_REQUEST_OK.
 * - Clock source HCLK, channel enabled and ready, buffered pin output, no trigger.
 * - Trimming value found by calibration is within 0 - DAC_TRIM_VALUE_MAX.
 */
void It_Dac_Init_DefaultConfig_ReadBack( void )
{
    dac_Config_t        config;
    dac_ClkSrc_t        clkSrc       = DAC_CLK_SRC_CNT;
    dac_FunctionState_t channelState = DAC_FUNCTION_INACTIVE;
    dac_OutputMode_t    outputMode   = DAC_OUTPUT_CNT;
    dac_TriggerId_t     triggerSrc   = DAC_TRIGGER_CNT;
    dac_FlagState_t     readyFlag    = DAC_FLAG_INACTIVE;
    dac_TrimValue_t     trimValue    = 0xFFu;

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( DAC_CLK_SRC_HCLK, clkSrc );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ChannelState( IT_DAC_PERIPH, IT_DAC_CHANNEL, &channelState ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE, channelState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_Flag( IT_DAC_PERIPH, IT_DAC_CHANNEL, DAC_FLAG_READY, &readyFlag ) );
    TEST_ASSERT_EQUAL( DAC_FLAG_ACTIVE, readyFlag );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputMode( IT_DAC_PERIPH, IT_DAC_CHANNEL, &outputMode ) );
    TEST_ASSERT_EQUAL( DAC_OUTPUT_PIN_BUFFERED, outputMode );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TriggerSrc( IT_DAC_PERIPH, IT_DAC_CHANNEL, &triggerSrc ) );
    TEST_ASSERT_EQUAL( DAC_TRIGGER_NONE, triggerSrc );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TrimValue( IT_DAC_PERIPH, IT_DAC_CHANNEL, &trimValue ) );
    TEST_ASSERT_LESS_OR_EQUAL( DAC_TRIM_VALUE_MAX, trimValue );
}


/**
 * \brief   Dac_Init() on target rejects invalid configurations and second initialization.
 *
 * \details
 * 1. NULL configuration.
 * 2. Initial value above 12-bit range.
 * 3. DMA transfer without hardware trigger.
 * 4. Valid initialization, then second initialization while the channel is enabled.
 *
 * \par Expected results
 * 1. - 3. DAC_REQUEST_ERROR.
 * 4. DAC_REQUEST_OK, then DAC_REQUEST_ERROR.
 */
void It_Dac_Init_InvalidConfig_ReturnsError( void )
{
    dac_Config_t config;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( DAC_NULL_PTR ) );

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].InitValue = DAC_DATA_12BIT_MAX + 1u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].DataConfig.TransferMode = DAC_TRANSFER_MODE_DMA;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].DataConfig.DataBuffer   = itDac_DmaBuffer;
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ].DataConfig.BufferSize   = IT_DAC_BUF_SIZE;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );
}


/**
 * \brief   Dac_Deinit() on target disables the channel and allows new initialization.
 *
 * \par Expected results
 * - Channel is disabled after Dac_Deinit(), second Dac_Init() returns DAC_REQUEST_OK.
 */
void It_Dac_Deinit_InitializedPeripheral_InitializationPossibleAgain( void )
{
    dac_Config_t        config;
    dac_FunctionState_t channelState = DAC_FUNCTION_ACTIVE;

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Deinit( IT_DAC_PERIPH ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ChannelState( IT_DAC_PERIPH, IT_DAC_CHANNEL, &channelState ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_INACTIVE, channelState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
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
    dac_Data_t   outData = 0u;

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
 * \brief   Dac_Set_DualData() updates outputs of both channels.
 *
 * \par Expected results
 * - Output registers of both channels contain the written values.
 */
void It_Dac_Set_DualData_BothChannels_OutputsUpdated( void )
{
    dac_Config_t config;
    dac_Data_t   outData = 0u;

    It_Dac_Get_Config( &config );
    config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DualData( IT_DAC_PERIPH, 0x0123u, 0x0FEDu ) );

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
 * \details DAC1_OUT1 (PA4) is converted by ADC1 channel 9 for low, middle and
 *          high output value.
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
    It_Dac_Adc_Init();

    for( uint32_t valueIdx = 0u; ( sizeof( testValues ) / sizeof( dac_Data_t ) ) > valueIdx; valueIdx++ )
    {
        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, testValues[ valueIdx ] ) );

        const adc_Data_t adcResult = It_Dac_Adc_Measure();

        TEST_ASSERT_LESS_THAN( IT_DAC_TOLERANCE_LSB, It_Dac_Get_Diff( testValues[ valueIdx ], adcResult ) );
    }

    TEST_ASSERT_EQUAL( 0u, itDac_AdcErrorCnt );
}


/**
 * \brief   Dac_Set_Voltage() sets the output voltage, Dac_Get_OutputVoltage() reads it back.
 *
 * \par Expected results
 * - Output voltage read back equals the required voltage within 1 mV.
 * - ADC result corresponds to the required voltage within IT_DAC_TOLERANCE_LSB.
 */
void It_Dac_Set_Voltage_1650mV_OutputVoltageMatches( void )
{
    dac_Config_t     config;
    dac_Voltage_mV_t outVoltage = 0u;

    It_Dac_Get_Config( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    It_Dac_Adc_Init();

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Voltage( IT_DAC_PERIPH, IT_DAC_CHANNEL, IT_DAC_VREF_MV, 1650u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputVoltage( IT_DAC_PERIPH, IT_DAC_CHANNEL, IT_DAC_VREF_MV, &outVoltage ) );
    TEST_ASSERT_UINT_WITHIN( 1u, 1650u, outVoltage );

    const adc_Data_t adcResult = It_Dac_Adc_Measure();

    TEST_ASSERT_LESS_THAN( IT_DAC_TOLERANCE_LSB, It_Dac_Get_Diff( 2048u, adcResult ) );
}

/*------------------------------- DMA transfer -------------------------------*/

/**
 * \brief   DMA transfer with hardware trigger can be started and stopped.
 *
 * \details Channel with TIM6 TRGO trigger and DMA circular buffer. TIM6 is not
 *          running - no DMA request is generated.
 *
 * \par Expected results
 * - Dac_Set_DmaStart() returns DAC_REQUEST_OK, second start DAC_REQUEST_ERROR.
 * - Dac_Set_DmaStop() returns DAC_REQUEST_OK, no error is reported.
 */
void It_Dac_Set_DmaStart_HwTrigger_StartStopHandled( void )
{
    dac_Config_t config;

    It_Dac_Get_Config( &config );
    dac_ChannelConfig_t * const channelCfg = &config.PeriphConfig[ IT_DAC_PERIPH ].Channels[ IT_DAC_CHANNEL ];

    channelCfg->TriggerId               = DAC_TRIGGER_TIM6_TRGO;
    channelCfg->DataConfig.TransferMode = DAC_TRANSFER_MODE_DMA;
    channelCfg->DataConfig.DataBuffer   = itDac_DmaBuffer;
    channelCfg->DataConfig.BufferSize   = IT_DAC_BUF_SIZE;
    channelCfg->DataConfig.BufferMode   = DAC_BUFFER_MODE_CIRCULAR;
    channelCfg->DataConfig.DmaPeriphId  = DAC_DMA_PERIPH_1;
    channelCfg->DataConfig.DmaChannelId = DAC_DMA_CHANNEL_6;
    channelCfg->DataConfig.DmaPriority  = DAC_DMA_PRIORITY_HIGH;
    channelCfg->DataConfig.IrqPriority  = 5u;
    channelCfg->DataConfig.ErrorCallback = It_Dac_ErrorCallback;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DmaStart( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_Data( IT_DAC_PERIPH, IT_DAC_CHANNEL, 0u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStop( IT_DAC_PERIPH, IT_DAC_CHANNEL ) );

    TEST_ASSERT_EQUAL( 0u, itDac_ErrorCnt );
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
 * \brief Initializes ADC1 with channel 9 (PA4) in polling mode, one result per start.
 */
static void It_Dac_Adc_Init( void )
{
    adc_Config_t config = { 0u };

    for( adc_PeriphId_t periphIdx = ADC_PERIPH_1; ADC_PERIPH_CNT > periphIdx; periphIdx++ )
    {
        config.PeriphConfig[ periphIdx ].PeriphId       = periphIdx;
        config.PeriphConfig[ periphIdx ].RegChannelsCnt = 0u;
        config.PeriphConfig[ periphIdx ].InjChannelsCnt = 0u;
    }

    adc_PeriphConfig_t * const periphCfg = &config.PeriphConfig[ IT_DAC_ADC_PERIPH ];

    periphCfg->Resolution     = ADC_RESOLUTION_12BIT;
    periphCfg->RegTriggerMode = ADC_REG_TRIGGER_MODE_SINGLE;
    periphCfg->RegTriggerEdge = ADC_TRIGGER_EDGE_RISING;
    periphCfg->RegTriggerId   = ADC_REG_TRIGGER_SOFTWARE;
    periphCfg->InjTriggerMode = ADC_INJ_TRIGGER_MODE_CONTINUOUS;
    periphCfg->InjTriggerId   = ADC_INJ_TRIGGER_SOFTWARE;
    periphCfg->InjTriggerEdge = ADC_TRIGGER_EDGE_RISING;

    periphCfg->RegChannelsCnt                    = 1u;
    periphCfg->RegChannels[ 0u ].ChannelId       = IT_DAC_ADC_CHANNEL;
    periphCfg->RegChannels[ 0u ].ChannelInput    = ADC_CHANNEL_INPUT_PIN_SINGLE;
    periphCfg->RegChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_814_CYCLES;

    periphCfg->DataConfig.TransferMode             = ADC_TRANSFER_MODE_POLL;
    periphCfg->DataConfig.DataBuffer               = itDac_AdcBuffer;
    periphCfg->DataConfig.BufferSize               = 1u;
    periphCfg->DataConfig.BufferMode               = ADC_BUFFER_MODE_ONE_SHOT;
    periphCfg->DataConfig.DmaPeriphId              = ADC_DMA_PERIPH_1;
    periphCfg->DataConfig.DmaChannelId             = ADC_DMA_CHANNEL_7;
    periphCfg->DataConfig.DmaPriority              = ADC_DMA_PRIORITY_LOW;
    periphCfg->DataConfig.IrqPriority              = 5u;
    periphCfg->DataConfig.HalfTransferCallback     = ADC_NULL_PTR;
    periphCfg->DataConfig.TransferCompleteCallback = It_Dac_AdcCompleteCallback;
    periphCfg->DataConfig.ErrorCallback            = It_Dac_AdcErrorCallback;
    periphCfg->DataConfig.InjCompleteCallback      = ADC_NULL_PTR;

    /* Same kernel clock as DAC (shared multiplexer), HCLK 160 MHz / 4 = 40 MHz */
    config.ClockSource  = ADC_CLK_SRC_HCLK;
    config.ClockDivider = ADC_CLK_DIV_4;

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );
}


/**
 * \brief Converts ADC1 channel 9 once (polling mode).
 *
 * \return ADC result (12-bit)
 */
static adc_Data_t It_Dac_Adc_Measure( void )
{
    const uint32_t expectedCnt = itDac_AdcCompleteCnt + 1u;

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_DAC_ADC_PERIPH ) );

    for( uint32_t loopIdx = 0u; ( IT_DAC_WAIT_LOOPS > loopIdx ) && ( expectedCnt > itDac_AdcCompleteCnt ); loopIdx++ )
    {
        Adc_Task();
    }

    TEST_ASSERT_EQUAL( expectedCnt, itDac_AdcCompleteCnt );

    return ( itDac_AdcBuffer[ 0u ] );
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


/** \brief ADC conversion complete callback */
static void It_Dac_AdcCompleteCallback( void )
{
    itDac_AdcCompleteCnt++;
}


/**
 * \brief ADC error callback.
 *
 * \param errorId [in]: Error identification
 */
static void It_Dac_AdcErrorCallback( adc_ErrorId_t errorId )
{
    (void)errorId;
    itDac_AdcErrorCnt++;
}


/**
 * \brief DAC data transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void It_Dac_ErrorCallback( dac_ErrorId_t errorId )
{
    (void)errorId;
    itDac_ErrorCnt++;
}
