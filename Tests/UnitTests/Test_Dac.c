/**
 * \author Mr.Nobody
 * \file Test_Dac.c
 * \ingroup Dac
 * \brief Unit tests of Digital-to-Analog Converter (DAC) module.
 *
 * Dac.c, Dac_Dma.c and Dac_Isr.c are compiled unchanged with real LL drivers.
 * DAC registers are emulated by RegMem, RCC, NVIC, GPIO and GPDMA modules are
 * mocked by CMock. Handlers registered in NVIC / GPDMA are captured by stubs and
 * called directly to test interrupt handling.
 *
 * Channel ready flags (DACxRDY follows ENx) and the offset calibration flag
 * (CAL_FLAGx set while CENx = 1 and OTRIMx >= utDac_CalThreshold) are emulated by
 * HW model running in background thread (Ut_Dac_HwModel).
 *
 * \note Emulated registers are plain memory - data holding registers are not
 *       moved to the output registers, write 1 to clear flags stays set.
 *
 * \note Data format, transfer context and GPDMA channel ownership are kept in
 *       static module context - every test is executed as separate process
 *       (CTest), setUp() additionally releases the context (Dac_Deinit()).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "UtCommon.h"                       /* Common test helpers            */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Dac_Port.h"                       /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockGpdma_Port.h"                 /* GPDMA module mock              */
#include "Stm32_dac.h"                      /* DAC registers definition       */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void                 Ut_Dac_HwModel              ( void );
static void                 Ut_Dac_Set_ModelActive      ( void );
static rcc_RequestState_t   Ut_Dac_RccGetClkStub        ( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt );
static rcc_RequestState_t   Ut_Dac_RccGetStateStub      ( rcc_PeriphId_t periphId, rcc_FunctionState_t * const funcState, int callCnt );
static rcc_RequestState_t   Ut_Dac_RccGetClkSrcStub     ( rcc_PeriphId_t periphId, rcc_PeriphId_t * const periphClkSrc, int callCnt );
static rcc_RequestState_t   Ut_Dac_RccSetActiveStub     ( rcc_PeriphId_t periphId, int callCnt );
static gpio_RequestState_t  Ut_Dac_GpioInitStub         ( gpio_Config_t *gpioConfig, int callCnt );
static nvic_RequestState_t  Ut_Dac_NvicSetHandlerStub   ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static gpdma_RequestState_t Ut_Dac_GpdmaInitStub        ( gpdma_ConfigStruct_t * const configStruct, int callCnt );
static gpdma_RequestState_t Ut_Dac_GpdmaBlockSizeStub   ( gpdma_PeriphId_t periphId, gpdma_ChannelId_t channelId, gpdma_BlockSize_t blockSize, int callCnt );
static gpdma_RequestState_t Ut_Dac_GpdmaSrcAddrStub     ( gpdma_PeriphId_t periphId, gpdma_ChannelId_t channelId, gpdma_SrcAddr_t sourceAddr, int callCnt );
static gpdma_RequestState_t Ut_Dac_GpdmaDstAddrStub     ( gpdma_PeriphId_t periphId, gpdma_ChannelId_t channelId, gpdma_DstAddr_t destAddr, int callCnt );
static gpdma_RequestState_t Ut_Dac_GpdmaChActiveStub    ( gpdma_PeriphId_t periphId, gpdma_ChannelId_t channelId, int callCnt );
static gpdma_RequestState_t Ut_Dac_GpdmaChInactiveStub  ( gpdma_PeriphId_t periphId, gpdma_ChannelId_t channelId, int callCnt );
static gpdma_RequestState_t Ut_Dac_GpdmaListConfigStub  ( gpdma_TransferConfig_t * const transferConfig, gpdma_TransfersCount_t transferCount,
                                                          gpdma_ChannelType_t channelType, volatile gpdma_XferList_t * const transferList, int callCnt );
static void                 Ut_Dac_Ignore_PeriphMocks   ( void );
static void                 Ut_Dac_Reset_Mocks          ( void );
static void                 Ut_Dac_Release              ( void );
static void                 Ut_Dac_Get_Config           ( dac_Config_t * const config );
static void                 Ut_Dac_Set_DmaConfig        ( dac_ChannelConfig_t * const channelConfig, dac_BufferMode_t bufferMode );
static void                 Ut_Dac_Init                 ( dac_Config_t * const config );

static void                 Ut_Dac_HalfCallback         ( void );
static void                 Ut_Dac_CompleteCallback     ( void );
static void                 Ut_Dac_ErrorCallback        ( dac_ErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** DAC peripheral used by tests */
#define UT_DAC_PERIPH                       ( DAC_PERIPH_1 )
#define UT_DAC_REG                          ( DAC1 )
#define UT_DAC_NVIC                         ( NVIC_PERIPH_IRQ_DAC1 )

/** HCLK frequency returned by RCC stub by default [Hz] */
#define UT_DAC_HCLK_HZ                      ( 64000000u )

/** Default trimming value threshold of the calibration model */
#define UT_DAC_CAL_THRESHOLD                ( 11u )

/** Interrupt priority of test configurations */
#define UT_DAC_PRIO                         ( 6u )

/** Size of the test data buffer */
#define UT_DAC_BUF_SIZE                     ( 8u )

/** MODE bits of channel 1 / 2 in MCR */
#define UT_DAC_MCR_MODE1                    ( DAC_MCR_MODE1 )
#define UT_DAC_MCR_MODE2                    ( DAC_MCR_MODE2 )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** HCLK frequency returned by RCC stub */
static rcc_FreqHz_t             utDac_HclkHz;

/** ADC clock state and selected kernel clock returned by RCC stubs */
static rcc_FunctionState_t      utDac_AdcClkState;
static rcc_PeriphId_t           utDac_RccClkSrc;

/** Last peripheral activated in RCC and count of activations */
static rcc_PeriphId_t           utDac_RccLastActive;
static uint32_t                 utDac_RccActiveCnt;

/** Trimming value from which the calibration model sets CAL_FLAGx */
static volatile uint32_t        utDac_CalThreshold;

/** GPIO configuration of the last Gpio_Init call and count of calls */
static gpio_Config_t            utDac_GpioConfig;
static uint32_t                 utDac_GpioInitCnt;

/** ISR registered in NVIC for the DAC */
static nvic_IsrCallback_t       utDac_Isr;

/** GPDMA configuration captured by Gpdma_Init stub */
static gpdma_ConfigStruct_t     utDac_DmaConfig;
static gpdma_TransferConfig_t   utDac_DmaXferConfig;
static uint32_t                 utDac_DmaInitCnt;

/** Values of the last GPDMA channel arming calls */
static gpdma_BlockSize_t        utDac_DmaBlockSize;
static gpdma_SrcAddr_t          utDac_DmaSrcAddr;
static gpdma_DstAddr_t          utDac_DmaDstAddr;
static uint32_t                 utDac_DmaActiveCnt;
static uint32_t                 utDac_DmaInactiveCnt;

/** Transfer configuration and channel type of the last transfer list node regeneration (start) */
static gpdma_TransferConfig_t   utDac_DmaNodeConfig;
static gpdma_ChannelType_t      utDac_DmaNodeChType;
static uint32_t                 utDac_DmaNodeCnt;

/** Output data buffer */
static dac_Data_t               utDac_Buffer[ UT_DAC_BUF_SIZE ];

/** Counts of callback calls and the last error */
static uint32_t                 utDac_HalfCnt;
static uint32_t                 utDac_CompleteCnt;
static uint32_t                 utDac_ErrorCnt;
static dac_ErrorId_t            utDac_LastError;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    /* Stops HW model of previous test and clears registers */
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utDac_HclkHz        = UT_DAC_HCLK_HZ;
    utDac_AdcClkState   = RCC_FUNCTION_INACTIVE;
    utDac_RccClkSrc     = RCC_PERIPH_DAC_HCLK;
    utDac_CalThreshold  = UT_DAC_CAL_THRESHOLD;

    Ut_Dac_Release();

    utDac_RccLastActive  = RCC_PERIPH_ID_CNT;
    utDac_RccActiveCnt   = 0u;
    utDac_GpioInitCnt    = 0u;
    utDac_Isr            = NULL;
    utDac_DmaInitCnt     = 0u;
    utDac_DmaBlockSize   = 0u;
    utDac_DmaSrcAddr     = 0u;
    utDac_DmaDstAddr     = 0u;
    utDac_DmaActiveCnt   = 0u;
    utDac_DmaInactiveCnt = 0u;
    utDac_DmaNodeCnt     = 0u;
    utDac_DmaNodeChType  = GPDMA_CHANNEL_OPTION_CNT;
    utDac_HalfCnt        = 0u;
    utDac_CompleteCnt    = 0u;
    utDac_ErrorCnt       = 0u;
    utDac_LastError      = DAC_ERROR_CNT;

    for( uint32_t idx = 0u; UT_DAC_BUF_SIZE > idx; idx++ )
    {
        utDac_Buffer[ idx ] = (dac_Data_t)( idx * 100u );
    }
}


void tearDown( void )
{
    (void)RegMem_Set_ModelInactive();
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Dac_Get_ModuleVersion() returns version of the module.
 *
 * \par Expected results
 * - Version is 1.0.0.
 */
void Ut_Dac_Get_ModuleVersion_ReturnsVersion( void )
{
    const dac_ModuleVersion_t version = Dac_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}

/* ============================ INITIALIZATION ============================== */

/**
 * \brief   Dac_Init() rejects NULL configuration and invalid clock source.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR is returned.
 * - No other module is called (strict mocks without expectations).
 */
void Ut_Dac_Init_InvalidArguments_ReturnsError( void )
{
    dac_Config_t config;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( NULL ) );

    Ut_Dac_Get_Config( &config );
    config.ClockSource = DAC_CLK_SRC_CNT;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );
}


/**
 * \brief   Dac_Get_DefaultConfig() fills the default configuration.
 *
 * \par Expected results
 * - HCLK kernel clock, channel 1 used (buffered pin output, no trigger, 12-bit right,
 *   direct transfer), channel 2 not used. NULL pointer is rejected.
 */
void Ut_Dac_Get_DefaultConfig_DefaultValues( void )
{
    dac_Config_t config;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_DefaultConfig( NULL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( DAC_CLK_SRC_HCLK, config.ClockSource );
    TEST_ASSERT_EQUAL( UT_DAC_PERIPH, config.PeriphConfig[ UT_DAC_PERIPH ].PeriphId );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE,   config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_INACTIVE, config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed );
    TEST_ASSERT_EQUAL( DAC_OUTPUT_PIN_BUFFERED,     config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].OutputMode );
    TEST_ASSERT_EQUAL( DAC_TRIGGER_NONE,            config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].TriggerId );
    TEST_ASSERT_EQUAL( DAC_DATA_FORMAT_12BIT_RIGHT, config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataFormat );
    TEST_ASSERT_EQUAL( DAC_TRANSFER_MODE_DIRECT,    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataConfig.TransferMode );

    /* Bug AB#578: default GPDMA channels of the DAC channels do not collide */
    TEST_ASSERT_EQUAL( DAC_DMA_CHANNEL_0, config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataConfig.DmaChannelId );
    TEST_ASSERT_EQUAL( DAC_DMA_CHANNEL_1, config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].DataConfig.DmaChannelId );
}


/**
 * \brief   Dac_Init() with default configuration initializes channel 1.
 *
 * \details Default configuration with initial value 0x123.
 *
 * \par Expected results
 * - DAC_REQUEST_OK, RCC_PERIPH_DAC_HCLK activated in RCC (once).
 * - PA4 configured as analog without pull.
 * - Channel 1 enabled, trigger disabled, buffered pin output (MODE1 = 0), normal mode (CEN1 = 0).
 * - DHR12R1 contains the initial value, channel 2 not touched.
 * - Trimming value found by calibration (model threshold) stored in OTRIM1.
 */
void Ut_Dac_Init_DefaultConfig_Channel1Initialized( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].InitValue = 0x123u;
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( RCC_PERIPH_DAC_HCLK, utDac_RccLastActive );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_RccActiveCnt );

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_GpioInitCnt );
    TEST_ASSERT_EQUAL( GPIO_PORT_A, utDac_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_4, utDac_GpioConfig.PinId );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ANALOG, utDac_GpioConfig.PinMode );
    TEST_ASSERT_EQUAL( GPIO_PIN_PULL_NONE, utDac_GpioConfig.PinPull );

    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN1, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->MCR & ( UT_DAC_MCR_MODE1 | UT_DAC_MCR_MODE2 ) );
    TEST_ASSERT_EQUAL_HEX32( 0x123u, UT_DAC_REG->DHR12R1 );
    TEST_ASSERT_EQUAL_HEX32( UT_DAC_CAL_THRESHOLD, UT_DAC_REG->CCR & DAC_CCR_OTRIM1 );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CCR & DAC_CCR_OTRIM2 );
}


/**
 * \brief   Dac_Init() refuses to switch the shared kernel clock used by ADC.
 *
 * \details ADC bus clock is active and the multiplexer selects SYSCLK, DAC requests HCLK.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR, no clock is activated in RCC, DAC registers are not modified.
 */
void Ut_Dac_Init_AdcRunningOnOtherClock_ReturnsError( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Ignore_PeriphMocks();
    utDac_AdcClkState = RCC_FUNCTION_ACTIVE;
    utDac_RccClkSrc   = RCC_PERIPH_ADC_SYSCLK;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_RccActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );
}


/**
 * \brief   Dac_Init() accepts the kernel clock already selected by ADC.
 *
 * \par Expected results
 * - DAC_REQUEST_OK, RCC_PERIPH_DAC_HCLK activated.
 */
void Ut_Dac_Init_AdcRunningOnSameClock_ClockSelected( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    utDac_AdcClkState = RCC_FUNCTION_ACTIVE;
    utDac_RccClkSrc   = RCC_PERIPH_ADC_HCLK;

    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( RCC_PERIPH_DAC_HCLK, utDac_RccLastActive );
}


/**
 * \brief   Dac_Init() is refused while a channel is enabled.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR, no clock activated, CR not modified.
 */
void Ut_Dac_Init_ChannelEnabled_ReturnsError( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Ignore_PeriphMocks();
    UT_DAC_REG->CR = DAC_CR_EN2;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_RccActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN2, UT_DAC_REG->CR );
}


/**
 * \brief   Dac_Init() rejects invalid channel configurations before register writes.
 *
 * \details Initial value out of format range, invalid output mode, channel slot
 *          mismatch, sample time out of range, peripheral slot mismatch.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR in all cases, channel registers (CR, MCR, DHR12R1) stay 0.
 */
void Ut_Dac_Init_InvalidChannelConfig_ReturnsErrorWithoutRegisterWrite( void )
{
    dac_Config_t config;

    Ut_Dac_Ignore_PeriphMocks();

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].InitValue = DAC_DATA_12BIT_MAX + 1u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataFormat = DAC_DATA_FORMAT_8BIT_RIGHT;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].InitValue  = DAC_DATA_8BIT_MAX + 1u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].OutputMode = DAC_OUTPUT_CNT;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelId = DAC_CHANNEL_2;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].SampleHold.SampleTime = DAC_SH_SAMPLE_TIME_MAX + 1u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].PeriphId = DAC_PERIPH_CNT;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->MCR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->DHR12R1 );
}


/**
 * \brief   Dac_PeriphInit() selects the high frequency interface mode from HCLK.
 *
 * \details HCLK 64 MHz, 100 MHz and 250 MHz (RCC stub).
 *
 * \par Expected results
 * - HFSEL = disabled / ABOVE_80MHZ / ABOVE_160MHZ.
 */
void Ut_Dac_PeriphInit_HclkFrequency_HighFreqModeSelected( void )
{
    const rcc_FreqHz_t hclkHz[ ]   = { 64000000u, 100000000u, 250000000u };
    const uint32_t     hfMode[ ]   = { LL_DAC_HIGH_FREQ_MODE_DISABLE, LL_DAC_HIGH_FREQ_MODE_ABOVE_80MHZ, LL_DAC_HIGH_FREQ_MODE_ABOVE_160MHZ };
    dac_Config_t       config;

    Ut_Dac_Ignore_PeriphMocks();
    Ut_Dac_Set_ModelActive();

    for( uint32_t idx = 0u; 3u > idx; idx++ )
    {
        Ut_Dac_Get_Config( &config );
        utDac_HclkHz = hclkHz[ idx ];

        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_PeriphInit( &config.PeriphConfig[ UT_DAC_PERIPH ] ) );
        TEST_ASSERT_EQUAL_HEX32( hfMode[ idx ], UT_DAC_REG->MCR & DAC_MCR_HFSEL );

        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Deinit( UT_DAC_PERIPH ) );
    }
}


/**
 * \brief   Dac_Init() with sample and hold channel selects the sample and hold clock and timing.
 *
 * \details Channel 1 with sample and hold (sample 100, hold 200, refresh 30), clock LSE.
 *
 * \par Expected results
 * - RCC_PERIPH_DAC_SAH_LSE activated after the kernel clock.
 * - SHSR1 / SHHR / SHRR contain the timing, MODE1 sample and hold bit set.
 * - Dac_Get_SampleHoldConfig() returns the configuration.
 */
void Ut_Dac_Init_SampleHold_ShClockAndTimingConfigured( void )
{
    dac_Config_t           config;
    dac_SampleHoldConfig_t shConfig = { 0 };

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].ShClockSource = DAC_SH_CLK_SRC_LSE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].SampleHold.State       = DAC_FUNCTION_ACTIVE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].SampleHold.SampleTime  = 100u;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].SampleHold.HoldTime    = 200u;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].SampleHold.RefreshTime = 30u;
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( RCC_PERIPH_DAC_SAH_LSE, utDac_RccLastActive );
    TEST_ASSERT_EQUAL_UINT32( 2u, utDac_RccActiveCnt );

    TEST_ASSERT_EQUAL_HEX32( 100u, UT_DAC_REG->SHSR1 );
    TEST_ASSERT_EQUAL_HEX32( 200u, UT_DAC_REG->SHHR & DAC_SHHR_THOLD1 );
    TEST_ASSERT_EQUAL_HEX32( 30u,  UT_DAC_REG->SHRR & DAC_SHRR_TREFRESH1 );
    TEST_ASSERT_EQUAL_HEX32( DAC_MCR_MODE1_2, UT_DAC_REG->MCR & UT_DAC_MCR_MODE1 );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &shConfig ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE, shConfig.State );
    TEST_ASSERT_EQUAL_UINT16( 100u, shConfig.SampleTime );
    TEST_ASSERT_EQUAL_UINT16( 200u, shConfig.HoldTime );
    TEST_ASSERT_EQUAL_UINT8( 30u, shConfig.RefreshTime );
}


/**
 * \brief   Dac_Init() with internal output does not configure the pin.
 *
 * \details Channel 2 used with DAC_OUTPUT_INTERNAL, channel 1 not used.
 *
 * \par Expected results
 * - No Gpio_Init call, MODE2 = 011 (buffer disabled, internal connection), EN2 set.
 */
void Ut_Dac_Init_InternalOutputChannel2_NoPinConfigured( void )
{
    dac_Config_t     config;
    dac_OutputMode_t outputMode = DAC_OUTPUT_CNT;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_INACTIVE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].OutputMode  = DAC_OUTPUT_INTERNAL;
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_GpioInitCnt );
    TEST_ASSERT_EQUAL_HEX32( DAC_MCR_MODE2_0 | DAC_MCR_MODE2_1, UT_DAC_REG->MCR & UT_DAC_MCR_MODE2 );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN2, UT_DAC_REG->CR );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_2, &outputMode ) );
    TEST_ASSERT_EQUAL( DAC_OUTPUT_INTERNAL, outputMode );
}


/**
 * \brief   Dac_Deinit() disables channels, interrupt and the DAC bus clock.
 *
 * \par Expected results
 * - DAC_REQUEST_OK, CR = 0, Nvic_Set_PeriphIrq_Inactive( DAC1 ) and
 *   Rcc_Set_PeriphInactive( RCC_PERIPH_DAC_HCLK ) called.
 * - Invalid peripheral is rejected.
 */
void Ut_Dac_Deinit_InitializedPeripheral_ChannelsAndClockDisabled( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    Ut_Dac_Init( &config );
    Ut_Dac_Reset_Mocks();

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_DAC_NVIC, NVIC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_DAC_HCLK, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Deinit( UT_DAC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Deinit( DAC_PERIPH_CNT ) );
}

/* ============================ CLOCK SOURCES =============================== */

/**
 * \brief   Dac_Get_ClockSource() / Dac_Get_ShClockSource() map the RCC selection.
 *
 * \par Expected results
 * - RCC_PERIPH_DAC_CSI -> DAC_CLK_SRC_CSI, RCC_PERIPH_DAC_SAH_LSE -> DAC_SH_CLK_SRC_LSE.
 * - Unknown RCC value and NULL pointer return DAC_REQUEST_ERROR.
 */
void Ut_Dac_Get_ClockSource_RccSelection_Mapped( void )
{
    dac_ClkSrc_t   clkSrc   = DAC_CLK_SRC_CNT;
    dac_ShClkSrc_t shClkSrc = DAC_SH_CLK_SRC_CNT;

    Ut_Dac_Ignore_PeriphMocks();

    utDac_RccClkSrc = RCC_PERIPH_DAC_CSI;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( DAC_CLK_SRC_CSI, clkSrc );

    utDac_RccClkSrc = RCC_PERIPH_DAC_SAH_LSE;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ShClockSource( &shClkSrc ) );
    TEST_ASSERT_EQUAL( DAC_SH_CLK_SRC_LSE, shClkSrc );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_ClockSource( NULL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_ShClockSource( NULL ) );
}


/**
 * \brief   Dac_Set_ShClockSource() is refused while a channel is enabled.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR, no clock activated in RCC. Invalid source rejected.
 */
void Ut_Dac_Set_ShClockSource_ChannelEnabled_ReturnsError( void )
{
    UT_DAC_REG->CR = DAC_CR_EN1;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ShClockSource( DAC_SH_CLK_SRC_LSI ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ShClockSource( DAC_SH_CLK_SRC_CNT ) );
}

/* ============================== CALIBRATION =============================== */

/**
 * \brief   Dac_Set_Calibration() finds the lowest trimming value setting CAL_FLAG.
 *
 * \details Calibration model sets CAL_FLAG2 for OTRIM2 >= threshold. Channel 2
 *          has unbuffered pin output configured before the calibration.
 *
 * \par Expected results
 * - Thresholds 1, 11, 12, 31 are found exactly.
 * - CEN2 is cleared and the unbuffered output mode is restored.
 */
void Ut_Dac_Set_Calibration_ModelThreshold_TrimValueFound( void )
{
    const uint32_t  thresholds[ ] = { 1u, 11u, 12u, 31u };
    dac_TrimValue_t trimValue     = 0u;

    Ut_Dac_Ignore_PeriphMocks();
    Ut_Dac_Set_ModelActive();
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_OUTPUT_PIN_UNBUFFERED ) );

    for( uint32_t idx = 0u; ( sizeof( thresholds ) / sizeof( uint32_t ) ) > idx; idx++ )
    {
        utDac_CalThreshold = thresholds[ idx ];

        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Calibration( UT_DAC_PERIPH, DAC_CHANNEL_2 ) );
        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_2, &trimValue ) );
        TEST_ASSERT_EQUAL_UINT8( thresholds[ idx ], trimValue );
    }

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & DAC_CR_CEN2 );
    TEST_ASSERT_EQUAL_HEX32( DAC_MCR_MODE2_1, UT_DAC_REG->MCR & UT_DAC_MCR_MODE2 );
}


/**
 * \brief   Dac_Set_Calibration() / Dac_Set_TrimValue() are refused for enabled channel.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR, CCR and CR not modified.
 */
void Ut_Dac_Set_Calibration_ChannelEnabled_ReturnsError( void )
{
    UT_DAC_REG->CR = DAC_CR_EN1;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_Calibration( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_1, 5u ) );

    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN1, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CCR );
}


/**
 * \brief   Dac_Set_TrimValue() writes OTRIMx, values above maximum are rejected.
 *
 * \par Expected results
 * - OTRIM1 = 7, OTRIM2 = 31, value 32 returns DAC_REQUEST_ERROR.
 */
void Ut_Dac_Set_TrimValue_ValidValues_WrittenToCcr( void )
{
    dac_TrimValue_t trimValue = 0u;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_1, 7u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_TRIM_VALUE_MAX ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_TRIM_VALUE_MAX + 1u ) );

    TEST_ASSERT_EQUAL_HEX32( 7u | ( DAC_TRIM_VALUE_MAX << DAC_CCR_OTRIM2_Pos ), UT_DAC_REG->CCR );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_2, &trimValue ) );
    TEST_ASSERT_EQUAL_UINT8( DAC_TRIM_VALUE_MAX, trimValue );
}

/* ========================= CHANNEL CONFIGURATION ========================== */

/**
 * \brief   Dac_Set_OutputMode() writes every output mode and reads it back.
 *
 * \par Expected results
 * - MODE1 bits 1:0 = 00 / 01 / 10 / 11, Dac_Get_OutputMode() returns the mode.
 * - Sample and hold bit is kept.
 */
void Ut_Dac_Set_OutputMode_AllModes_ReadBack( void )
{
    const uint32_t   modeBits[ DAC_OUTPUT_CNT ] = { 0u, DAC_MCR_MODE1_0, DAC_MCR_MODE1_1, DAC_MCR_MODE1_1 | DAC_MCR_MODE1_0 };
    dac_OutputMode_t outputMode = DAC_OUTPUT_CNT;

    UT_DAC_REG->MCR = DAC_MCR_MODE1_2;

    for( dac_OutputMode_t modeIdx = DAC_OUTPUT_PIN_BUFFERED; DAC_OUTPUT_CNT > modeIdx; modeIdx++ )
    {
        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_1, modeIdx ) );
        TEST_ASSERT_EQUAL_HEX32( modeBits[ modeIdx ] | DAC_MCR_MODE1_2, UT_DAC_REG->MCR & UT_DAC_MCR_MODE1 );

        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_1, &outputMode ) );
        TEST_ASSERT_EQUAL( modeIdx, outputMode );
    }

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_OUTPUT_CNT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_CNT, DAC_OUTPUT_PIN_BUFFERED ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_1, NULL ) );
}


/**
 * \brief   Channel configuration setters are refused for enabled channel.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR for output mode, trigger, wave mode, amplitude and sample and
 *   hold, MCR and CR are not modified.
 */
void Ut_Dac_Set_ChannelConfig_ChannelEnabled_ReturnsErrorWithoutWrite( void )
{
    const dac_SampleHoldConfig_t shConfig = { .State = DAC_FUNCTION_ACTIVE, .SampleTime = 1u, .HoldTime = 1u, .RefreshTime = 1u };

    UT_DAC_REG->CR = DAC_CR_EN2;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_OUTPUT_INTERNAL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_TRIGGER_TIM6_TRGO ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_WaveMode( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_WAVE_NOISE ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_WaveAmplitude( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_WAVE_AMPLITUDE_255 ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_2, &shConfig ) );

    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN2, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->MCR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->SHSR2 );
}


/**
 * \brief   Dac_Set_TriggerSrc() writes every trigger and reads it back.
 *
 * \par Expected results
 * - DAC_TRIGGER_NONE: TEN2 = 0. Other triggers: TEN2 = 1, TSEL2 = LL value.
 * - Dac_Get_TriggerSrc() returns the trigger for every value.
 */
void Ut_Dac_Set_TriggerSrc_AllTriggers_ReadBack( void )
{
    dac_TriggerId_t triggerSrc = DAC_TRIGGER_CNT;

    for( dac_TriggerId_t trigIdx = DAC_TRIGGER_NONE; DAC_TRIGGER_CNT > trigIdx; trigIdx++ )
    {
        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, trigIdx ) );

        if( DAC_TRIGGER_NONE == trigIdx )
        {
            TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & DAC_CR_TEN2 );
        }
        else
        {
            TEST_ASSERT_EQUAL_HEX32( DAC_CR_TEN2, UT_DAC_REG->CR & DAC_CR_TEN2 );
        }

        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, &triggerSrc ) );
        TEST_ASSERT_EQUAL( trigIdx, triggerSrc );
    }

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_TRIGGER_TIM6_TRGO ) );
    TEST_ASSERT_EQUAL_HEX32( LL_DAC_TRIG_EXT_TIM6_TRGO << DAC_CR_EN2_Pos, UT_DAC_REG->CR & DAC_CR_TSEL2 );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & ( DAC_CR_TEN1 | DAC_CR_TSEL1 ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_TRIGGER_CNT ) );
}


/**
 * \brief   Wave mode and amplitude setters write WAVEx / MAMPx and read them back.
 *
 * \par Expected results
 * - Triangle with amplitude 1023 on channel 1, noise with amplitude 4095 on channel 2.
 */
void Ut_Dac_Set_WaveModeAmplitude_ReadBack( void )
{
    dac_WaveMode_t      waveMode  = DAC_WAVE_CNT;
    dac_WaveAmplitude_t amplitude = DAC_WAVE_AMPLITUDE_CNT;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_WaveMode( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_WAVE_TRIANGLE ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_WaveAmplitude( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_WAVE_AMPLITUDE_1023 ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_WaveMode( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_WAVE_NOISE ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_WaveAmplitude( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_WAVE_AMPLITUDE_4095 ) );

    TEST_ASSERT_EQUAL_HEX32( LL_DAC_WAVE_AUTO_GENERATION_TRIANGLE | LL_DAC_TRIANGLE_AMPLITUDE_1023 |
                             ( ( LL_DAC_WAVE_AUTO_GENERATION_NOISE | LL_DAC_NOISE_LFSR_UNMASK_BITS11_0 ) << DAC_CR_EN2_Pos ),
                             UT_DAC_REG->CR );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_WaveMode( UT_DAC_PERIPH, DAC_CHANNEL_2, &waveMode ) );
    TEST_ASSERT_EQUAL( DAC_WAVE_NOISE, waveMode );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_WaveAmplitude( UT_DAC_PERIPH, DAC_CHANNEL_1, &amplitude ) );
    TEST_ASSERT_EQUAL( DAC_WAVE_AMPLITUDE_1023, amplitude );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_WaveMode( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_WAVE_CNT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_WaveAmplitude( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_WAVE_AMPLITUDE_CNT ) );
}


/**
 * \brief   Dac_Set_SampleHoldConfig() rejects invalid configuration and stuck busy flag.
 *
 * \details Hold time above maximum, invalid state, NULL pointer, BWST1 permanently set.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR in all cases. Sample and hold mode bit is not set.
 */
void Ut_Dac_Set_SampleHoldConfig_InvalidOrBusy_ReturnsError( void )
{
    dac_SampleHoldConfig_t shConfig = { .State = DAC_FUNCTION_ACTIVE, .SampleTime = 10u, .HoldTime = DAC_SH_HOLD_TIME_MAX + 1u, .RefreshTime = 1u };

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &shConfig ) );

    shConfig.HoldTime = 10u;
    shConfig.State    = (dac_FunctionState_t)2u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &shConfig ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, NULL ) );

    shConfig.State = DAC_FUNCTION_ACTIVE;
    UT_DAC_REG->SR = DAC_SR_BWST1;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &shConfig ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->MCR & DAC_MCR_MODE1_2 );
}

/* =============================== DATA OUTPUT ============================== */

/**
 * \brief   Dac_Set_Data() writes the data holding register of the channel format.
 *
 * \par Expected results
 * - 12-bit right -> DHR12R2, 12-bit left -> DHR12L2, 8-bit right -> DHR8R2.
 * - Values out of format range are rejected.
 */
void Ut_Dac_Set_Data_DataFormats_HoldingRegisterWritten( void )
{
    dac_DataFormat_t dataFormat = DAC_DATA_FORMAT_CNT;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( UT_DAC_PERIPH, DAC_CHANNEL_2, 0x0FFFu ) );
    TEST_ASSERT_EQUAL_HEX32( 0x0FFFu, UT_DAC_REG->DHR12R2 );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_Data( UT_DAC_PERIPH, DAC_CHANNEL_2, 0x1000u ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_DATA_FORMAT_12BIT_LEFT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( UT_DAC_PERIPH, DAC_CHANNEL_2, 0xABC0u ) );
    TEST_ASSERT_EQUAL_HEX32( 0xABC0u, UT_DAC_REG->DHR12L2 );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_DATA_FORMAT_8BIT_RIGHT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( UT_DAC_PERIPH, DAC_CHANNEL_2, 0x5Au ) );
    TEST_ASSERT_EQUAL_HEX32( 0x5Au, UT_DAC_REG->DHR8R2 );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_Data( UT_DAC_PERIPH, DAC_CHANNEL_2, 0x100u ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_2, &dataFormat ) );
    TEST_ASSERT_EQUAL( DAC_DATA_FORMAT_8BIT_RIGHT, dataFormat );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_DATA_FORMAT_CNT ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->DHR12R1 );
}


/**
 * \brief   Dac_Set_DualData() writes the dual data holding register.
 *
 * \par Expected results
 * - 12-bit right: DHR12RD = ch1 | ch2 << 16. 8-bit right: DHR8RD = ch1 | ch2 << 8.
 * - Different formats of the channels are rejected.
 */
void Ut_Dac_Set_DualData_SameFormat_DualRegisterWritten( void )
{
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DualData( UT_DAC_PERIPH, 0x0123u, 0x0456u ) );
    TEST_ASSERT_EQUAL_HEX32( 0x04560123u, UT_DAC_REG->DHR12RD );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_12BIT_LEFT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DualData( UT_DAC_PERIPH, 0x0010u, 0x0020u ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_8BIT_RIGHT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_DATA_FORMAT_8BIT_RIGHT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DualData( UT_DAC_PERIPH, 0x12u, 0x34u ) );
    TEST_ASSERT_EQUAL_HEX32( 0x3412u, UT_DAC_REG->DHR8RD );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DualData( UT_DAC_PERIPH, 0x12u, 0x134u ) );
}


/**
 * \brief   Dac_Get_OutputData() and Dac_Get_OutputVoltage() read the output register.
 *
 * \par Expected results
 * - DOR1 value returned, 4095 at 3300 mV reference -> 3300 mV, 2048 -> 1650 mV.
 * - Reference 0 and NULL pointers rejected.
 */
void Ut_Dac_Get_OutputData_OutputRegister_ValueAndVoltage( void )
{
    dac_Data_t       outData    = 0u;
    dac_Voltage_mV_t outVoltage = 0u;

    UT_DAC_REG->DOR1 = 0x0FFFu;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( UT_DAC_PERIPH, DAC_CHANNEL_1, &outData ) );
    TEST_ASSERT_EQUAL_HEX16( 0x0FFFu, outData );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputVoltage( UT_DAC_PERIPH, DAC_CHANNEL_1, 3300u, &outVoltage ) );
    TEST_ASSERT_EQUAL_UINT16( 3300u, outVoltage );

    UT_DAC_REG->DOR2 = 2048u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputVoltage( UT_DAC_PERIPH, DAC_CHANNEL_2, 3300u, &outVoltage ) );
    TEST_ASSERT_EQUAL_UINT16( 1650u, outVoltage );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_OutputVoltage( UT_DAC_PERIPH, DAC_CHANNEL_2, 0u, &outVoltage ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_OutputVoltage( UT_DAC_PERIPH, DAC_CHANNEL_2, 3300u, NULL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_OutputData( UT_DAC_PERIPH, DAC_CHANNEL_CNT, &outData ) );
}


/**
 * \brief   Dac_Get_DataFromVoltage() converts voltage according to the data format.
 *
 * \par Expected results
 * - 1650 mV of 3300 mV: 12-bit right 2048, 12-bit left 0x8000, 8-bit right 128.
 * - 3300 mV -> 4095. Voltage above reference and reference 0 rejected.
 * - Dac_Set_Voltage() writes the converted value.
 */
void Ut_Dac_Get_DataFromVoltage_DataFormats_Converted( void )
{
    dac_Data_t data = 0u;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DataFromVoltage( UT_DAC_PERIPH, DAC_CHANNEL_1, 3300u, 1650u, &data ) );
    TEST_ASSERT_EQUAL_HEX16( 2048u, data );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DataFromVoltage( UT_DAC_PERIPH, DAC_CHANNEL_1, 3300u, 3300u, &data ) );
    TEST_ASSERT_EQUAL_HEX16( 4095u, data );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_12BIT_LEFT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DataFromVoltage( UT_DAC_PERIPH, DAC_CHANNEL_1, 3300u, 1650u, &data ) );
    TEST_ASSERT_EQUAL_HEX16( 0x8000u, data );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_8BIT_RIGHT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DataFromVoltage( UT_DAC_PERIPH, DAC_CHANNEL_1, 3300u, 1650u, &data ) );
    TEST_ASSERT_EQUAL_HEX16( 128u, data );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_DataFromVoltage( UT_DAC_PERIPH, DAC_CHANNEL_1, 3300u, 3301u, &data ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_DataFromVoltage( UT_DAC_PERIPH, DAC_CHANNEL_1, 0u, 0u, &data ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_DataFromVoltage( UT_DAC_PERIPH, DAC_CHANNEL_1, 3300u, 0u, NULL ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Voltage( UT_DAC_PERIPH, DAC_CHANNEL_2, 3300u, 1000u ) );
    TEST_ASSERT_EQUAL_HEX32( 1241u, UT_DAC_REG->DHR12R2 );
}


/**
 * \brief   Software trigger is generated only for enabled channel with software trigger.
 *
 * \par Expected results
 * - Trigger disabled or channel disabled: DAC_REQUEST_ERROR, SWTRIGR = 0.
 * - Single trigger: SWTRIG1 set. Dual trigger: SWTRIG1 and SWTRIG2 set.
 */
void Ut_Dac_Set_SwTrigger_SoftwareTriggerSelected_SwtrigSet( void )
{
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SwTrigger( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_TRIGGER_SOFTWARE ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_TRIGGER_SOFTWARE ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SwTrigger( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->SWTRIGR );

    UT_DAC_REG->CR |= DAC_CR_EN1;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DualSwTrigger( UT_DAC_PERIPH ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_SwTrigger( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_SWTRIGR_SWTRIG1, UT_DAC_REG->SWTRIGR );

    UT_DAC_REG->CR |= DAC_CR_EN2;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DualSwTrigger( UT_DAC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_SWTRIGR_SWTRIG1 | DAC_SWTRIGR_SWTRIG2, UT_DAC_REG->SWTRIGR );
}


/**
 * \brief   Dac_Get_Flag() reads status flags, Dac_Clear_Flag() clears only DMA underrun.
 *
 * \par Expected results
 * - Ready / calibration / busy / underrun flags of channel 2 are reported from SR.
 * - Clear of underrun writes DMAUDR2, clear of other flags is rejected.
 */
void Ut_Dac_Get_Flag_StatusRegister_FlagsReported( void )
{
    dac_FlagState_t flagState = DAC_FLAG_INACTIVE;

    UT_DAC_REG->SR = DAC_SR_DAC2RDY | DAC_SR_CAL_FLAG2;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_FLAG_READY, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_FLAG_ACTIVE, flagState );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_FLAG_CALIBRATION, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_FLAG_ACTIVE, flagState );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_FLAG_SH_WRITE_BUSY, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_FLAG_INACTIVE, flagState );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_READY, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_FLAG_INACTIVE, flagState );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_CNT, &flagState ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Clear_Flag( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_FLAG_READY ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_SR_DAC2RDY | DAC_SR_CAL_FLAG2, UT_DAC_REG->SR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Clear_Flag( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_FLAG_DMA_UNDERRUN ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_SR_DMAUDR2, UT_DAC_REG->SR );
}


/**
 * \brief   Dac_Set_ChannelActive() fails when the channel does not become ready.
 *
 * \details No HW model - DAC1RDY stays 0.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR (timeout), EN1 is set.
 */
void Ut_Dac_Set_ChannelActive_NotReady_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ChannelActive( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN1, UT_DAC_REG->CR );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_ChannelInactive( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );
}

/* ============================== DMA TRANSFER ============================== */

/**
 * \brief   DMA transfer requires a hardware trigger.
 *
 * \details DMA configuration with no trigger and with software trigger.
 *
 * \par Expected results
 * - Dac_Init() and Dac_Set_DataConfig() return DAC_REQUEST_ERROR, GPDMA is not initialized.
 */
void Ut_Dac_Set_DataConfig_DmaWithoutHwTrigger_ReturnsError( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].TriggerId = DAC_TRIGGER_SOFTWARE;

    Ut_Dac_Ignore_PeriphMocks();
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataConfig ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_DmaInitCnt );
}


/**
 * \brief   DMA transfer configuration is checked.
 *
 * \details TIM6 trigger with NULL buffer, size 0, invalid DMA channel.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR in all cases, GPDMA is not initialized.
 */
void Ut_Dac_Set_DataConfig_InvalidDmaConfig_ReturnsError( void )
{
    dac_ChannelConfig_t channelCfg;
    dac_Config_t        config;

    Ut_Dac_Get_Config( &config );
    channelCfg = config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ];
    Ut_Dac_Set_DmaConfig( &channelCfg, DAC_BUFFER_MODE_ONE_SHOT );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_TRIGGER_TIM6_TRGO ) );

    channelCfg.DataConfig.DataBuffer = NULL;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &channelCfg.DataConfig ) );

    channelCfg.DataConfig.DataBuffer = utDac_Buffer;
    channelCfg.DataConfig.BufferSize = 0u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &channelCfg.DataConfig ) );

    channelCfg.DataConfig.BufferSize   = UT_DAC_BUF_SIZE;
    channelCfg.DataConfig.DmaChannelId = DAC_DMA_CHANNEL_CNT;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &channelCfg.DataConfig ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, NULL ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_DmaInitCnt );
}


/**
 * \brief   Dac_Init() in DMA mode initializes the GPDMA channel and DAC interrupt.
 *
 * \details Channel 2, TIM6 trigger, DMA buffer of 8 items, 12-bit left aligned format.
 *
 * \par Expected results
 * - Gpdma_Init() called once: memory to peripheral, request DAC1_CH2, block 16 bytes,
 *   source = buffer (increment, 16-bit), destination = DHR12L2 (static, 16-bit),
 *   transfer complete and error handlers registered.
 * - DAC interrupt registered in NVIC, DMA requests not enabled yet (DMAEN2 = 0).
 */
void Ut_Dac_Init_DmaMode_GpdmaChannelInitialized( void )
{
    dac_Config_t     config;
    dac_DataConfig_t dataConfig;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_INACTIVE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].DataFormat  = DAC_DATA_FORMAT_12BIT_LEFT;
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInitCnt );
    TEST_ASSERT_EQUAL( GPDMA_DIR_MEMORY_TO_PERIPH, utDac_DmaXferConfig.Direction );
    TEST_ASSERT_EQUAL( GPDMA_XFER_LIST_EXEC_ONCE, utDac_DmaXferConfig.XferListExecMode );
    TEST_ASSERT_EQUAL( GPDMA_REQ_DAC1_CH2, utDac_DmaXferConfig.RequestSource );
    TEST_ASSERT_EQUAL_UINT16( UT_DAC_BUF_SIZE * sizeof( dac_Data_t ), utDac_DmaXferConfig.BlockSize );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utDac_Buffer, utDac_DmaXferConfig.SourceAddr );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_INCREMENT, utDac_DmaXferConfig.SourceAddrMode );
    TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_16BITS, utDac_DmaXferConfig.SourceDataSize );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_DAC_REG->DHR12L2, utDac_DmaXferConfig.DestinationAddr );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_STATIC, utDac_DmaXferConfig.DestinationAddrMode );
    TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_16BITS, utDac_DmaXferConfig.DestinationDataSize );
    TEST_ASSERT_NOT_NULL( utDac_DmaConfig.TransferCompleteIsr );
    TEST_ASSERT_NOT_NULL( utDac_DmaConfig.HalfTransferIsr );
    TEST_ASSERT_NOT_NULL( utDac_DmaConfig.ErrorIsr );

    TEST_ASSERT_NOT_NULL( utDac_Isr );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & DAC_CR_DMAEN2 );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_2, &dataConfig ) );
    TEST_ASSERT_EQUAL( DAC_TRANSFER_MODE_DMA, dataConfig.TransferMode );
    TEST_ASSERT_EQUAL_PTR( utDac_Buffer, dataConfig.DataBuffer );
}


/**
 * \brief   Dac_Init() rejects the same GPDMA channel for both DAC channels.
 *
 * \details Both channels in DMA mode with GPDMA1 channel 6, then channel 2 with
 *          GPDMA1 channel 7.
 * \note    Bug AB#578: the second channel reconfigured the GPDMA channel of the first one.
 *
 * \par Expected results
 * - Same GPDMA channel: DAC_REQUEST_ERROR, Gpdma_Init() not called.
 * - Different GPDMA channels: DAC_REQUEST_OK, Gpdma_Init() called twice.
 */
void Ut_Dac_Init_BothChannelsSameGpdmaChannel_ReturnsError( void )
{
    dac_Config_t config;

    Ut_Dac_Ignore_PeriphMocks();
    Ut_Dac_Set_ModelActive();

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ], DAC_BUFFER_MODE_ONE_SHOT );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_DmaInitCnt );

    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].DataConfig.DmaChannelId = DAC_DMA_CHANNEL_7;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utDac_DmaInitCnt );
}


/**
 * \brief   Dac_ChannelInit() rejects GPDMA channel owned by the other DAC channel.
 *
 * \details Channel 1 initialized in DMA mode with GPDMA1 channel 6, channel 2 is
 *          initialized separately with the same GPDMA channel.
 * \note    Bug AB#578: ownership of the GPDMA channel was checked per DAC channel only.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR, Gpdma_Init() called only for channel 1, channel 2 not enabled.
 */
void Ut_Dac_ChannelInit_GpdmaChannelOwnedByOtherChannel_ReturnsError( void )
{
    dac_Config_t        config;
    dac_ChannelConfig_t channel2Cfg;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInitCnt );

    channel2Cfg             = config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ];
    channel2Cfg.ChannelUsed = DAC_FUNCTION_ACTIVE;
    Ut_Dac_Set_DmaConfig( &channel2Cfg, DAC_BUFFER_MODE_ONE_SHOT );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_ChannelInit( UT_DAC_PERIPH, &channel2Cfg ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInitCnt );
    TEST_ASSERT_BITS_LOW( DAC_CR_EN2, UT_DAC_REG->CR );
}


/**
 * \brief   Dac_Set_DmaStart() arms GPDMA and enables DAC DMA requests.
 *
 * \par Expected results
 * - GPDMA block size, source, destination set and channel enabled.
 * - DMAEN1 and DMAUDRIE1 set, second start refused, direct data write refused.
 * - Dac_Set_DmaStop() clears DMAEN1 / DMAUDRIE1 and disables the GPDMA channel.
 */
void Ut_Dac_Set_DmaStart_InitializedDma_RequestsEnabled( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    TEST_ASSERT_EQUAL_UINT16( UT_DAC_BUF_SIZE * sizeof( dac_Data_t ), utDac_DmaBlockSize );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utDac_Buffer, utDac_DmaSrcAddr );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_DAC_REG->DHR12R1, utDac_DmaDstAddr );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_DMAEN1 | DAC_CR_DMAUDRIE1, UT_DAC_REG->CR & ( DAC_CR_DMAEN1 | DAC_CR_DMAUDRIE1 ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_Data( UT_DAC_PERIPH, DAC_CHANNEL_1, 0u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_8BIT_RIGHT ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStop( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & ( DAC_CR_DMAEN1 | DAC_CR_DMAUDRIE1 ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInactiveCnt );
}


/**
 * \brief   Dac_Set_DmaStart() is refused for disabled channel and direct transfer.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR, GPDMA channel not enabled, DMAEN1 = 0.
 */
void Ut_Dac_Set_DmaStart_NotReady_ReturnsError( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Ignore_PeriphMocks();

    /* Direct transfer (nothing configured) */
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    /* DMA configured, channel disabled */
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_TRIGGER_TIM6_TRGO ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataConfig ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_DmaActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & DAC_CR_DMAEN1 );

    /* Hardware trigger can not be removed while DMA transfer is configured */
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_TRIGGER_NONE ) );
}


/**
 * \brief   One shot transfer complete stops the transfer and calls the callback.
 *
 * \par Expected results
 * - Half transfer handler calls HalfTransferCallback.
 * - Transfer complete: DMAEN1 cleared, GPDMA channel disabled, TransferCompleteCallback
 *   called once, new start possible.
 */
void Ut_Dac_DmaComplete_OneShot_TransferStopped( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    utDac_DmaConfig.HalfTransferIsr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_HalfCnt );

    utDac_DmaConfig.TransferCompleteIsr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_CompleteCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & DAC_CR_DMAEN1 );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaActiveCnt );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
}


/**
 * \brief   Circular transfer uses the cyclic GPDMA transfer list, transfer complete keeps it running.
 *
 * \par Expected results
 * - Gpdma_Init() and the node regenerated on start use GPDMA_XFER_LIST_EXEC_CYCLIC_ALL.
 * - Transfer complete: TransferCompleteCallback called, GPDMA channel neither re-armed nor
 *   disabled by SW, DMAEN1 kept.
 */
void Ut_Dac_DmaComplete_Circular_CyclicListKeptRunning( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_CIRCULAR );
    Ut_Dac_Init( &config );
    TEST_ASSERT_EQUAL( GPDMA_XFER_LIST_EXEC_CYCLIC_ALL, utDac_DmaXferConfig.XferListExecMode );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaNodeCnt );
    TEST_ASSERT_EQUAL( GPDMA_XFER_LIST_EXEC_CYCLIC_ALL, utDac_DmaNodeConfig.XferListExecMode );

    utDac_DmaConfig.TransferCompleteIsr();
    utDac_DmaConfig.TransferCompleteIsr();

    TEST_ASSERT_EQUAL_UINT32( 2u, utDac_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaActiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_DmaInactiveCnt );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_DMAEN1, UT_DAC_REG->CR & DAC_CR_DMAEN1 );
}


/**
 * \brief   Transfer list node is regenerated on start from the actual configuration.
 *
 * \details Channel 1 on GPDMA channel 6 (2D capable), one shot transfer started and completed,
 *          data format changed to 8-bit right aligned, transfer started again.
 *
 * \par Expected results
 * - Node of the second start: GPDMA_XFER_LIST_EXEC_ONCE, destination DHR8R1, block 16 bytes,
 *   channel type GPDMA_CHANNEL_LINEAR_2D. Channel registers get the same block.
 */
void Ut_Dac_Set_DmaStart_FormatChanged_NodeRegenerated( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    utDac_DmaConfig.TransferCompleteIsr();

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_8BIT_RIGHT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utDac_DmaNodeCnt );
    TEST_ASSERT_EQUAL( GPDMA_CHANNEL_LINEAR_2D, utDac_DmaNodeChType );
    TEST_ASSERT_EQUAL( GPDMA_XFER_LIST_EXEC_ONCE, utDac_DmaNodeConfig.XferListExecMode );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_DAC_REG->DHR8R1, utDac_DmaNodeConfig.DestinationAddr );
    TEST_ASSERT_EQUAL_UINT16( UT_DAC_BUF_SIZE * sizeof( dac_Data_t ), utDac_DmaNodeConfig.BlockSize );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_DAC_REG->DHR8R1, utDac_DmaDstAddr );
}


/**
 * \brief   GPDMA error stops the transfer and reports the error.
 *
 * \par Expected results
 * - ErrorCallback called with DAC_ERROR_DMA_TRANSFER, DMAEN1 cleared.
 */
void Ut_Dac_DmaError_TransferError_ReportedAndStopped( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_CIRCULAR );
    Ut_Dac_Init( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    utDac_DmaConfig.ErrorIsr( GPDMA_ERROR_TRANSFER );

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_ErrorCnt );
    TEST_ASSERT_EQUAL( DAC_ERROR_DMA_TRANSFER, utDac_LastError );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & DAC_CR_DMAEN1 );
}


/**
 * \brief   DAC interrupt reports DMA underrun of the channel.
 *
 * \details DMA transfer on channel 2 running, DMAUDR2 set in SR, DAC ISR called.
 *          Second call without pending flag.
 *
 * \par Expected results
 * - ErrorCallback called once with DAC_ERROR_DMA_UNDERRUN.
 * - DMAEN2 and DMAUDRIE2 cleared, GPDMA channel disabled.
 */
void Ut_Dac_Isr_DmaUnderrun_ReportedAndStopped( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_INACTIVE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ], DAC_BUFFER_MODE_CIRCULAR );
    Ut_Dac_Init( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_2 ) );
    TEST_ASSERT_NOT_NULL( utDac_Isr );

    (void)RegMem_Set_ModelInactive();
    UT_DAC_REG->SR = DAC_SR_DMAUDR2;
    utDac_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_ErrorCnt );
    TEST_ASSERT_EQUAL( DAC_ERROR_DMA_UNDERRUN, utDac_LastError );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & ( DAC_CR_DMAEN2 | DAC_CR_DMAUDRIE2 ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInactiveCnt );

    utDac_Isr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_ErrorCnt );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief HW model of DAC1 - runs in background thread.
 *
 * DACxRDY follows ENx, CAL_FLAGx is set while CENx = 1 and OTRIMx >= utDac_CalThreshold.
 */
static void Ut_Dac_HwModel( void )
{
    const uint32_t cr       = UT_DAC_REG->CR;
    const uint32_t ccr      = UT_DAC_REG->CCR;
    const uint32_t trim1    = ( ccr & DAC_CCR_OTRIM1 ) >> DAC_CCR_OTRIM1_Pos;
    const uint32_t trim2    = ( ccr & DAC_CCR_OTRIM2 ) >> DAC_CCR_OTRIM2_Pos;
    uint32_t       setMask  = 0u;
    uint32_t       clrMask  = 0u;

    if( 0u != ( cr & DAC_CR_EN1 ) ) { setMask |= DAC_SR_DAC1RDY; } else { clrMask |= DAC_SR_DAC1RDY; }
    if( 0u != ( cr & DAC_CR_EN2 ) ) { setMask |= DAC_SR_DAC2RDY; } else { clrMask |= DAC_SR_DAC2RDY; }

    if( ( 0u != ( cr & DAC_CR_CEN1 ) ) && ( utDac_CalThreshold <= trim1 ) ) { setMask |= DAC_SR_CAL_FLAG1; } else { clrMask |= DAC_SR_CAL_FLAG1; }
    if( ( 0u != ( cr & DAC_CR_CEN2 ) ) && ( utDac_CalThreshold <= trim2 ) ) { setMask |= DAC_SR_CAL_FLAG2; } else { clrMask |= DAC_SR_CAL_FLAG2; }

    (void)__atomic_or_fetch( &UT_DAC_REG->SR, setMask, __ATOMIC_SEQ_CST );
    (void)__atomic_and_fetch( &UT_DAC_REG->SR, ~clrMask, __ATOMIC_SEQ_CST );
}


/**
 * \brief Starts the DAC HW model.
 */
static void Ut_Dac_Set_ModelActive( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( Ut_Dac_HwModel ) );
}


/** \brief RCC clock stub - every clock runs at utDac_HclkHz */
static rcc_RequestState_t Ut_Dac_RccGetClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)periphId;
    (void)callCnt;

    /* Dac_Set_Delay() asks for the clock after every register change the HW model reacts to
     * (OTRIM -> CAL_FLAG). The model is let to finish whole cycles with the new register state
     * before the module reads the flags back, the result does not depend on host load. */
    (void)RegMem_Wait_ModelCycles( 1u );

    *periphClk = utDac_HclkHz;

    return ( RCC_REQUEST_OK );
}


/** \brief RCC state stub - ADC bus clock state utDac_AdcClkState */
static rcc_RequestState_t Ut_Dac_RccGetStateStub( rcc_PeriphId_t periphId, rcc_FunctionState_t * const funcState, int callCnt )
{
    (void)periphId;
    (void)callCnt;

    *funcState = utDac_AdcClkState;

    return ( RCC_REQUEST_OK );
}


/** \brief RCC clock source stub - selected source utDac_RccClkSrc */
static rcc_RequestState_t Ut_Dac_RccGetClkSrcStub( rcc_PeriphId_t periphId, rcc_PeriphId_t * const periphClkSrc, int callCnt )
{
    (void)periphId;
    (void)callCnt;

    *periphClkSrc = utDac_RccClkSrc;

    return ( RCC_REQUEST_OK );
}


/** \brief RCC activation stub - stores the last activated peripheral */
static rcc_RequestState_t Ut_Dac_RccSetActiveStub( rcc_PeriphId_t periphId, int callCnt )
{
    (void)callCnt;

    utDac_RccLastActive = periphId;
    utDac_RccActiveCnt++;

    return ( RCC_REQUEST_OK );
}


/** \brief GPIO initialization stub - stores the pin configuration */
static gpio_RequestState_t Ut_Dac_GpioInitStub( gpio_Config_t *gpioConfig, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( gpioConfig );

    utDac_GpioConfig = *gpioConfig;
    utDac_GpioInitCnt++;

    return ( GPIO_REQUEST_OK );
}


/** \brief NVIC handler registration stub - stores ISR of the DAC */
static nvic_RequestState_t Ut_Dac_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( UT_DAC_NVIC, irqId );
    TEST_ASSERT_NOT_NULL( irqHandler );

    utDac_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/** \brief GPDMA initialization stub - stores channel and transfer configuration */
static gpdma_RequestState_t Ut_Dac_GpdmaInitStub( gpdma_ConfigStruct_t * const configStruct, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( configStruct );
    TEST_ASSERT_NOT_NULL( configStruct->TransferConfig );

    utDac_DmaConfig     = *configStruct;
    utDac_DmaXferConfig = *configStruct->TransferConfig;
    utDac_DmaInitCnt++;

    return ( GPDMA_REQUEST_OK );
}


/** \brief GPDMA block size stub */
static gpdma_RequestState_t Ut_Dac_GpdmaBlockSizeStub( gpdma_PeriphId_t periphId, gpdma_ChannelId_t channelId, gpdma_BlockSize_t blockSize, int callCnt )
{
    (void)periphId;
    (void)channelId;
    (void)callCnt;

    utDac_DmaBlockSize = blockSize;

    return ( GPDMA_REQUEST_OK );
}


/** \brief GPDMA source address stub */
static gpdma_RequestState_t Ut_Dac_GpdmaSrcAddrStub( gpdma_PeriphId_t periphId, gpdma_ChannelId_t channelId, gpdma_SrcAddr_t sourceAddr, int callCnt )
{
    (void)periphId;
    (void)channelId;
    (void)callCnt;

    utDac_DmaSrcAddr = sourceAddr;

    return ( GPDMA_REQUEST_OK );
}


/** \brief GPDMA destination address stub */
static gpdma_RequestState_t Ut_Dac_GpdmaDstAddrStub( gpdma_PeriphId_t periphId, gpdma_ChannelId_t channelId, gpdma_DstAddr_t destAddr, int callCnt )
{
    (void)periphId;
    (void)channelId;
    (void)callCnt;

    utDac_DmaDstAddr = destAddr;

    return ( GPDMA_REQUEST_OK );
}


/** \brief GPDMA channel enable stub */
static gpdma_RequestState_t Ut_Dac_GpdmaChActiveStub( gpdma_PeriphId_t periphId, gpdma_ChannelId_t channelId, int callCnt )
{
    (void)periphId;
    (void)channelId;
    (void)callCnt;

    utDac_DmaActiveCnt++;

    return ( GPDMA_REQUEST_OK );
}


/** \brief GPDMA channel disable stub */
static gpdma_RequestState_t Ut_Dac_GpdmaChInactiveStub( gpdma_PeriphId_t periphId, gpdma_ChannelId_t channelId, int callCnt )
{
    (void)periphId;
    (void)channelId;
    (void)callCnt;

    utDac_DmaInactiveCnt++;

    return ( GPDMA_REQUEST_OK );
}


/** \brief GPDMA transfer list generation stub - stores the node configuration */
static gpdma_RequestState_t Ut_Dac_GpdmaListConfigStub( gpdma_TransferConfig_t * const transferConfig, gpdma_TransfersCount_t transferCount,
                                                        gpdma_ChannelType_t channelType, volatile gpdma_XferList_t * const transferList, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( transferConfig );
    TEST_ASSERT_NOT_NULL( transferList );
    TEST_ASSERT_EQUAL_UINT32( 1u, transferCount );

    utDac_DmaNodeConfig = *transferConfig;
    utDac_DmaNodeChType = channelType;
    utDac_DmaNodeCnt++;

    return ( GPDMA_REQUEST_OK );
}


/**
 * \brief Ignores / stubs all calls of RCC / NVIC / GPIO / GPDMA functions used by the module.
 */
static void Ut_Dac_Ignore_PeriphMocks( void )
{
    Rcc_Set_PeriphActive_StubWithCallback( Ut_Dac_RccSetActiveStub );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphState_StubWithCallback( Ut_Dac_RccGetStateStub );
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_Dac_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Dac_RccGetClkStub );

    Gpio_Init_StubWithCallback( Ut_Dac_GpioInitStub );

    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Dac_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_IgnoreAndReturn( NVIC_REQUEST_OK );

    Gpdma_Get_DefaultConfig_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Init_StubWithCallback( Ut_Dac_GpdmaInitStub );
    Gpdma_Set_Priority_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Set_HalfTransferIsrHandler_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Set_HalfTransferIrqActive_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Set_HalfTransferIrqInactive_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Set_InterruptActive_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Set_InterruptInactive_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Set_BlockSize_StubWithCallback( Ut_Dac_GpdmaBlockSizeStub );
    Gpdma_Set_SourceAddr_StubWithCallback( Ut_Dac_GpdmaSrcAddrStub );
    Gpdma_Set_DestinationAddr_StubWithCallback( Ut_Dac_GpdmaDstAddrStub );
    Gpdma_Set_ChannelActive_StubWithCallback( Ut_Dac_GpdmaChActiveStub );
    Gpdma_Set_ChannelInactive_StubWithCallback( Ut_Dac_GpdmaChInactiveStub );
    Gpdma_Get_XferListConfig_StubWithCallback( Ut_Dac_GpdmaListConfigStub );
    Gpdma_Get_XferList_LinkReg_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Set_XferListLink_IgnoreAndReturn( GPDMA_REQUEST_OK );
}


/**
 * \brief Removes all expectations and ignores of the mocks.
 */
static void Ut_Dac_Reset_Mocks( void )
{
    MockRcc_Port_Destroy();
    MockNvic_Port_Destroy();
    MockGpio_Port_Destroy();
    MockGpdma_Port_Destroy();
    MockRcc_Port_Init();
    MockNvic_Port_Init();
    MockGpio_Port_Init();
    MockGpdma_Port_Init();
}


/**
 * \brief Releases static module context of the previous test and re-initializes the mocks.
 */
static void Ut_Dac_Release( void )
{
    Ut_Dac_Ignore_PeriphMocks();

    (void)Dac_Deinit( UT_DAC_PERIPH );

    Ut_Dac_Reset_Mocks();
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );
}


/**
 * \brief Returns default module configuration (channel 1 used).
 *
 * \param config [out]: Module configuration
 */
static void Ut_Dac_Get_Config( dac_Config_t * const config )
{
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DefaultConfig( config ) );
}


/**
 * \brief Configures DMA transfer of a channel with TIM6 trigger and test buffer.
 *
 * \param channelConfig [in,out]: Channel configuration
 * \param bufferMode        [in]: One shot / circular
 */
static void Ut_Dac_Set_DmaConfig( dac_ChannelConfig_t * const channelConfig, dac_BufferMode_t bufferMode )
{
    channelConfig->TriggerId                           = DAC_TRIGGER_TIM6_TRGO;
    channelConfig->DataConfig.TransferMode             = DAC_TRANSFER_MODE_DMA;
    channelConfig->DataConfig.DataBuffer               = utDac_Buffer;
    channelConfig->DataConfig.BufferSize               = UT_DAC_BUF_SIZE;
    channelConfig->DataConfig.BufferMode               = bufferMode;
    channelConfig->DataConfig.DmaPeriphId              = DAC_DMA_PERIPH_1;
    channelConfig->DataConfig.DmaChannelId             = DAC_DMA_CHANNEL_6;
    channelConfig->DataConfig.DmaPriority              = DAC_DMA_PRIORITY_HIGH;
    channelConfig->DataConfig.IrqPriority              = UT_DAC_PRIO;
    channelConfig->DataConfig.HalfTransferCallback     = Ut_Dac_HalfCallback;
    channelConfig->DataConfig.TransferCompleteCallback = Ut_Dac_CompleteCallback;
    channelConfig->DataConfig.ErrorCallback            = Ut_Dac_ErrorCallback;
}


/**
 * \brief Initializes the module with HW model and stubbed peripheral modules.
 *
 * \param config [in]: Module configuration
 */
static void Ut_Dac_Init( dac_Config_t * const config )
{
    Ut_Dac_Ignore_PeriphMocks();
    Ut_Dac_Set_ModelActive();

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( config ) );
}


/** \brief Half transfer callback */
static void Ut_Dac_HalfCallback( void )
{
    utDac_HalfCnt++;
}


/** \brief Transfer complete callback */
static void Ut_Dac_CompleteCallback( void )
{
    utDac_CompleteCnt++;
}


/**
 * \brief Transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void Ut_Dac_ErrorCallback( dac_ErrorId_t errorId )
{
    utDac_LastError = errorId;
    utDac_ErrorCnt++;
}
