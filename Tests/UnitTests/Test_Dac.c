/**
 * \author Mr.Nobody
 * \file Test_Dac.c
 * \ingroup Dac
 * \brief Unit tests of Digital-to-Analog Converter (DAC) module (STM32F7).
 *
 * Dac.c, Dac_Dma.c and Dac_Isr.c are compiled unchanged with real LL drivers.
 * DAC registers are emulated by RegMem, RCC, NVIC, GPIO and DMA modules are
 * mocked by CMock. Handlers registered in NVIC / DMA are captured by stubs and
 * called directly to test interrupt handling.
 *
 * \note Emulated registers are plain memory - data holding registers are not
 *       moved to the output registers, write 1 to clear flags stays set.
 *
 * \note Data format and transfer context are kept in static module context -
 *       every test is executed as separate process (CTest), setUp() additionally
 *       releases the context (Dac_Deinit()).
 */

/* ============================= INCLUDES =================================== */
#include <string.h>                         /* memset                         */
#include "unity.h"                         /* Unity testing framework        */
#include "UtCommon.h"                       /* Common test helpers            */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Dac_Port.h"                       /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockDma_Port.h"                   /* DMA module mock                */
#include "Stm32_dac.h"                      /* DAC registers definition       */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static rcc_RequestState_t   Ut_Dac_RccSetActiveStub     ( rcc_PeriphId_t periphId, int callCnt );
static rcc_RequestState_t   Ut_Dac_RccSetInactiveStub   ( rcc_PeriphId_t periphId, int callCnt );
static gpio_RequestState_t  Ut_Dac_GpioInitStub        ( gpio_Config_t *gpioConfig, int callCnt );
static nvic_RequestState_t  Ut_Dac_NvicSetHandlerStub   ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static dma_RequestState_t   Ut_Dac_DmaInitStub          ( dma_ConfigStruct_t * const dmaConfig, int callCnt );
static dma_RequestState_t   Ut_Dac_DmaModeStub          ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_TransferMode_t transferMode, int callCnt );
static dma_RequestState_t   Ut_Dac_DmaPeriphAddrStub    ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_PeriphAddr_t periphAddr, int callCnt );
static dma_RequestState_t   Ut_Dac_DmaMemoryAddrStub    ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_MemoryAddr_t memoryAddr, int callCnt );
static dma_RequestState_t   Ut_Dac_DmaDataCountStub     ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t dataCount, int callCnt );
static dma_RequestState_t   Ut_Dac_DmaActiveStub        ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Dac_DmaInactiveStub      ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Dac_DmaHtIrqActiveStub   ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Dac_DmaHtIrqInactiveStub ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
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
#define UT_DAC_NVIC                         ( NVIC_PERIPH_IRQ_TIM6_DAC )

/** DMA streams of the DAC channels (RM0385 DMA1 request mapping, channel selection 7) */
#define UT_DAC_DMA_PERIPH                   ( DMA_PERIPH_1 )
#define UT_DAC_DMA_STREAM_CH1               ( DMA_STREAM_5 )
#define UT_DAC_DMA_STREAM_CH2               ( DMA_STREAM_6 )

/** Encoded DMA stream from the DAC peripheral index, DAC channel index, DMA peripheral index, stream number and
 *  channel selection number (bit-fields written independently of DAC_DMA_ENCODE) */
#define UT_DAC_DMA_CODE( PERIPH, CHANNEL, DMA, STREAM, CHSEL )  ( ( (PERIPH) << 20u ) | ( (CHANNEL) << 15u ) | ( (DMA) << 10u ) | ( (STREAM) << 5u ) | (CHSEL) )

/** Interrupt priority of test configurations */
#define UT_DAC_PRIO                         ( 6u )

/** Size of the test data buffer */
#define UT_DAC_BUF_SIZE                     ( 8u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** Last peripheral activated in RCC and count of activations */
static rcc_PeriphId_t           utDac_RccLastActive;
static uint32_t                 utDac_RccActiveCnt;

/** Count of DAC clock deactivations (stub), DMAEN1 and count of disabled DMA streams at the
 *  last deactivation */
static uint32_t                 utDac_RccInactiveCnt;
static uint32_t                 utDac_RccInactiveDmaEn;
static uint32_t                 utDac_RccInactiveDmaStops;

/** GPIO configuration of the last Gpio_Init call and count of calls */
static gpio_Config_t            utDac_GpioConfig;
static uint32_t                 utDac_GpioInitCnt;

/** ISR registered in NVIC for the DAC */
static nvic_IsrCallback_t       utDac_Isr;

/** DMA configuration captured by Dma_Init stub */
static dma_ConfigStruct_t       utDac_DmaConfig;
static uint32_t                 utDac_DmaInitCnt;

/** Values of the last DMA stream arming calls */
static dma_ChannelId_t          utDac_DmaStream;
static dma_TransferMode_t       utDac_DmaMode;
static dma_PeriphAddr_t         utDac_DmaPeriphAddr;
static dma_MemoryAddr_t         utDac_DmaMemoryAddr;
static dma_DataCount_t          utDac_DmaDataCount;
static uint32_t                 utDac_DmaActiveCnt;
static uint32_t                 utDac_DmaInactiveCnt;
static dma_RequestState_t       utDac_DmaActiveResult;

/** Half transfer interrupt state of the DMA stream (1 - enabled, 0 - disabled) */
static uint32_t                 utDac_DmaHtIrqState;

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
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    Ut_Dac_Release();

    utDac_RccLastActive   = RCC_PERIPH_ID_CNT;
    utDac_RccActiveCnt    = 0u;
    utDac_RccInactiveCnt      = 0u;
    utDac_RccInactiveDmaEn    = 0u;
    utDac_RccInactiveDmaStops = 0u;
    utDac_GpioInitCnt     = 0u;
    utDac_Isr             = NULL;
    utDac_DmaInitCnt      = 0u;
    utDac_DmaStream       = DMA_STREAM_CNT;
    utDac_DmaMode         = DMA_TRANSFER_MODE_NORMAL;
    utDac_DmaPeriphAddr   = 0u;
    utDac_DmaMemoryAddr   = 0u;
    utDac_DmaDataCount    = 0u;
    utDac_DmaActiveCnt    = 0u;
    utDac_DmaInactiveCnt  = 0u;
    utDac_DmaActiveResult = DMA_REQUEST_OK;
    utDac_DmaHtIrqState   = 0xFFu;
    utDac_HalfCnt         = 0u;
    utDac_CompleteCnt     = 0u;
    utDac_ErrorCnt        = 0u;
    utDac_LastError       = DAC_ERROR_CNT;

    (void)memset( &utDac_DmaConfig, 0, sizeof( utDac_DmaConfig ) );

    for( uint32_t idx = 0u; UT_DAC_BUF_SIZE > idx; idx++ )
    {
        utDac_Buffer[ idx ] = (dac_Data_t)( idx * 100u );
    }
}


void tearDown( void )
{
    /* Mocks are verified by generated runner */
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
 * - APB1 clock, channel 1 used (buffered pin output, no trigger, 12-bit right, direct
 *   transfer), channel 2 not used.
 * - DMA fields contain the fixed request mapping: DMA1 stream 5 (channel 1) / stream 6 (channel 2).
 * - NULL pointer is rejected.
 */
void Ut_Dac_Get_DefaultConfig_DefaultValues( void )
{
    dac_Config_t config;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_DefaultConfig( NULL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DefaultConfig( &config ) );

    const dac_ChannelConfig_t * const ch1Cfg = &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ];
    const dac_ChannelConfig_t * const ch2Cfg = &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ];

    TEST_ASSERT_EQUAL( DAC_CLK_SRC_PCLK1, config.ClockSource );
    TEST_ASSERT_EQUAL( UT_DAC_PERIPH, config.PeriphConfig[ UT_DAC_PERIPH ].PeriphId );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE,         ch1Cfg->ChannelUsed );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_INACTIVE,       ch2Cfg->ChannelUsed );
    TEST_ASSERT_EQUAL( DAC_OUTPUT_PIN_BUFFERED,     ch1Cfg->OutputMode );
    TEST_ASSERT_EQUAL( DAC_TRIGGER_NONE,            ch1Cfg->TriggerId );
    TEST_ASSERT_EQUAL( DAC_DATA_FORMAT_12BIT_RIGHT, ch1Cfg->DataFormat );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_INACTIVE,       ch1Cfg->SampleHold.State );
    TEST_ASSERT_EQUAL( DAC_TRANSFER_MODE_DIRECT,    ch1Cfg->DataConfig.TransferMode );
    TEST_ASSERT_EQUAL( DAC_DMA_DAC1_CH1_DMA1_STREAM5, ch1Cfg->DataConfig.Dma );
    TEST_ASSERT_EQUAL( DAC_DMA_DAC1_CH2_DMA1_STREAM6, ch2Cfg->DataConfig.Dma );
}


/**
 * \brief   Dac_Init() with default configuration initializes channel 1.
 *
 * \details Default configuration with initial value 0x123.
 *
 * \par Expected results
 * - DAC_REQUEST_OK, RCC_PERIPH_DAC activated in RCC (once).
 * - PA4 configured as analog without pull.
 * - Channel 1 enabled, trigger disabled, output buffer enabled (CR = EN1).
 * - DHR12R1 contains the initial value, channel 2 not touched.
 * - DAC interrupt is not registered (direct transfer).
 */
void Ut_Dac_Init_DefaultConfig_Channel1Initialized( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].InitValue = 0x123u;
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( RCC_PERIPH_DAC, utDac_RccLastActive );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_RccActiveCnt );

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_GpioInitCnt );
    TEST_ASSERT_EQUAL( GPIO_PORT_A, utDac_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_4, utDac_GpioConfig.PinId );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ANALOG, utDac_GpioConfig.PinMode );
    TEST_ASSERT_EQUAL( GPIO_PIN_PULL_NONE, utDac_GpioConfig.PinPull );

    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN1, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL_HEX32( 0x123u, UT_DAC_REG->DHR12R1 );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->DHR12R2 );
    TEST_ASSERT_NULL( utDac_Isr );
}


/**
 * \brief   Dac_Init() initializes channel 2 with unbuffered output.
 *
 * \details Channel 1 not used, channel 2 with DAC_OUTPUT_PIN_UNBUFFERED.
 *
 * \par Expected results
 * - PA5 configured as analog, CR = EN2 | BOFF2.
 * - Dac_Get_OutputMode() returns DAC_OUTPUT_PIN_UNBUFFERED.
 */
void Ut_Dac_Init_UnbufferedChannel2_PinAndBufferConfigured( void )
{
    dac_Config_t     config;
    dac_OutputMode_t outputMode = DAC_OUTPUT_CNT;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_INACTIVE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].OutputMode  = DAC_OUTPUT_PIN_UNBUFFERED;
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_GpioInitCnt );
    TEST_ASSERT_EQUAL( GPIO_PORT_A, utDac_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_5, utDac_GpioConfig.PinId );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ANALOG, utDac_GpioConfig.PinMode );

    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN2 | DAC_CR_BOFF2, UT_DAC_REG->CR );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_2, &outputMode ) );
    TEST_ASSERT_EQUAL( DAC_OUTPUT_PIN_UNBUFFERED, outputMode );
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
 * - DAC_REQUEST_ERROR in all cases, channel registers (CR, DHR12R1) stay 0, no pin configured.
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

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].ShClockSource = DAC_SH_CLK_SRC_CNT;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->DHR12R1 );
    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_GpioInitCnt );
}


/**
 * \brief   Dac_Init() rejects features not supported by STM32F7.
 *
 * \details Output connected to on-chip peripherals (with and without pin), active sample
 *          and hold, DMA stream not matching the DMA request mapping of the channel.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR in all cases, CR stays 0, no pin configured, DMA not initialized.
 */
void Ut_Dac_Init_UnsupportedFeature_ReturnsErrorWithoutRegisterWrite( void )
{
    dac_Config_t config;

    Ut_Dac_Ignore_PeriphMocks();

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].OutputMode = DAC_OUTPUT_PIN_INTERNAL_BUFFERED;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].OutputMode = DAC_OUTPUT_INTERNAL;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].SampleHold.State = DAC_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataConfig.Dma = DAC_DMA_DAC1_CH2_DMA1_STREAM6;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_GpioInitCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_DmaInitCnt );
}


/**
 * \brief   Dac_Deinit() of directly driven channels disables channels and the DAC bus clock.
 *
 * \details The shared TIM6_DAC interrupt line was not enabled by the module.
 *
 * \par Expected results
 * - DAC_REQUEST_OK, CR = 0, Rcc_Set_PeriphInactive( RCC_PERIPH_DAC ) called.
 * - Nvic_Set_PeriphIrq_Inactive() is not called (TIM6 interrupt is kept).
 * - Invalid peripheral is rejected.
 */
void Ut_Dac_Deinit_DirectChannels_ChannelsAndClockDisabled( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    Ut_Dac_Init( &config );
    Ut_Dac_Reset_Mocks();

    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_DAC, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Deinit( UT_DAC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Deinit( DAC_PERIPH_CNT ) );
}


/**
 * \brief   Dac_Deinit() releases DMA resources and the DAC interrupt line.
 *
 * \details Channel 2 initialized for DMA transfer.
 *
 * \par Expected results
 * - DMA1 stream 6 stopped and its interrupt disabled in NVIC, channels, DMA requests and
 *   DMA underrun interrupt disabled.
 * - TIM6_DAC interrupt line disabled in NVIC, DAC bus clock disabled.
 */
void Ut_Dac_Deinit_DmaChannel_DmaAndInterruptReleased( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_INACTIVE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );
    Ut_Dac_Reset_Mocks();

    Dma_Set_TransferInactive_ExpectAndReturn( UT_DAC_DMA_PERIPH, UT_DAC_DMA_STREAM_CH2, DMA_REQUEST_OK );
    Dma_Set_InterruptInactive_ExpectAndReturn( UT_DAC_DMA_PERIPH, UT_DAC_DMA_STREAM_CH2, DMA_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_DAC_NVIC, NVIC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_DAC, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Deinit( UT_DAC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & ( DAC_CR_EN1 | DAC_CR_EN2 | DAC_CR_DMAEN2 | DAC_CR_DMAUDRIE2 ) );
}

/* ============================ CLOCK SOURCES =============================== */

/**
 * \brief   Clock source functions of STM32F7 DAC.
 *
 * \par Expected results
 * - Dac_Get_ClockSource() returns DAC_CLK_SRC_PCLK1, NULL pointer rejected.
 * - Dac_Set_ClockSource() is refused while a channel is enabled and for invalid source.
 * - Sample and hold clock functions always return DAC_REQUEST_ERROR (not supported).
 */
void Ut_Dac_ClockSource_ApbClockOnly( void )
{
    dac_ClkSrc_t   clkSrc   = DAC_CLK_SRC_CNT;
    dac_ShClkSrc_t shClkSrc = DAC_SH_CLK_SRC_CNT;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( DAC_CLK_SRC_PCLK1, clkSrc );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_ClockSource( NULL ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ClockSource( DAC_CLK_SRC_CNT ) );
    UT_DAC_REG->CR = DAC_CR_EN1;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ClockSource( DAC_CLK_SRC_PCLK1 ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ShClockSource( DAC_SH_CLK_SRC_LSI ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_ShClockSource( &shClkSrc ) );
}

/* ====================== FEATURES NOT SUPPORTED BY F7 ====================== */

/**
 * \brief   Offset calibration and trimming are not supported by STM32F7.
 *
 * \par Expected results
 * - Dac_Set_Calibration(), Dac_Set_TrimValue(), Dac_Get_TrimValue() return DAC_REQUEST_ERROR,
 *   CR is not modified.
 */
void Ut_Dac_Calibration_NotSupported_ReturnsError( void )
{
    dac_TrimValue_t trimValue = 0u;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_Calibration( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_1, 5u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_1, &trimValue ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );
}


/**
 * \brief   Sample and hold accepts only the inactive configuration.
 *
 * \par Expected results
 * - Inactive configuration: DAC_REQUEST_OK, no register written.
 * - Active state, invalid state, hold time above maximum, NULL pointer and invalid channel:
 *   DAC_REQUEST_ERROR.
 * - Dac_Get_SampleHoldConfig() returns inactive state with zero timing.
 */
void Ut_Dac_SampleHold_InactiveOnly( void )
{
    dac_SampleHoldConfig_t shConfig = { .State = DAC_FUNCTION_INACTIVE, .SampleTime = 10u, .HoldTime = 10u, .RefreshTime = 1u };

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &shConfig ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_CNT, &shConfig ) );

    shConfig.State = DAC_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &shConfig ) );

    shConfig.State = (dac_FunctionState_t)2u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &shConfig ) );

    shConfig.State    = DAC_FUNCTION_INACTIVE;
    shConfig.HoldTime = DAC_SH_HOLD_TIME_MAX + 1u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &shConfig ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, NULL ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_2, &shConfig ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_INACTIVE, shConfig.State );
    TEST_ASSERT_EQUAL_UINT16( 0u, shConfig.SampleTime );
    TEST_ASSERT_EQUAL_UINT16( 0u, shConfig.HoldTime );
    TEST_ASSERT_EQUAL_UINT8( 0u, shConfig.RefreshTime );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_2, NULL ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );
}

/* ========================= CHANNEL CONFIGURATION ========================== */

/**
 * \brief   Dac_Set_OutputMode() writes the output buffer bit and reads it back.
 *
 * \par Expected results
 * - Unbuffered: BOFF1 set, buffered: BOFF1 cleared, Dac_Get_OutputMode() returns the mode.
 * - Modes with internal connection, invalid mode / channel and NULL pointer are rejected,
 *   BOFF1 is not modified.
 */
void Ut_Dac_Set_OutputMode_PinModes_ReadBack( void )
{
    dac_OutputMode_t outputMode = DAC_OUTPUT_CNT;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_OUTPUT_PIN_UNBUFFERED ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_BOFF1, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_1, &outputMode ) );
    TEST_ASSERT_EQUAL( DAC_OUTPUT_PIN_UNBUFFERED, outputMode );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_OUTPUT_PIN_BUFFERED ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_1, &outputMode ) );
    TEST_ASSERT_EQUAL( DAC_OUTPUT_PIN_BUFFERED, outputMode );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_OUTPUT_INTERNAL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_OUTPUT_PIN_INTERNAL_BUFFERED ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_OUTPUT_CNT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_CNT, DAC_OUTPUT_PIN_BUFFERED ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_1, NULL ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );
}


/**
 * \brief   Channel configuration setters are refused for enabled channel.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR for output mode, trigger, wave mode and amplitude, CR is not modified.
 */
void Ut_Dac_Set_ChannelConfig_ChannelEnabled_ReturnsErrorWithoutWrite( void )
{
    UT_DAC_REG->CR = DAC_CR_EN2;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_OUTPUT_PIN_UNBUFFERED ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_TRIGGER_TIM6_TRGO ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_WaveMode( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_WAVE_NOISE ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_WaveAmplitude( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_WAVE_AMPLITUDE_255 ) );

    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN2, UT_DAC_REG->CR );
}


/**
 * \brief   Dac_Set_TriggerSrc() writes every trigger and reads it back.
 *
 * \par Expected results
 * - DAC_TRIGGER_NONE: TEN2 = 0. Other triggers: TEN2 = 1.
 * - Dac_Get_TriggerSrc() returns the trigger for every value.
 * - TSEL2 encoding of TIM8 / software / EXTI9 trigger (RM0385), channel 1 not touched.
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

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_TRIGGER_TIM8_TRGO ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_TSEL2_0, UT_DAC_REG->CR & DAC_CR_TSEL2 );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_TRIGGER_EXTI_LINE9 ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_TSEL2_2 | DAC_CR_TSEL2_1, UT_DAC_REG->CR & DAC_CR_TSEL2 );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_TRIGGER_SOFTWARE ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_TSEL2, UT_DAC_REG->CR & DAC_CR_TSEL2 );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & ( DAC_CR_TEN1 | DAC_CR_TSEL1 ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_TRIGGER_CNT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, NULL ) );
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
 * \brief   Dac_Set_ChannelActive() / Dac_Set_ChannelInactive() set and clear ENx.
 *
 * \par Expected results
 * - EN1 set, Dac_Get_ChannelState() returns active, EN1 cleared, state inactive.
 * - Invalid channel and NULL pointer rejected.
 */
void Ut_Dac_Set_ChannelActive_EnableBit_ReadBack( void )
{
    dac_FunctionState_t channelState = DAC_FUNCTION_INACTIVE;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_ChannelActive( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN1, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ChannelState( UT_DAC_PERIPH, DAC_CHANNEL_1, &channelState ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE, channelState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_ChannelInactive( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ChannelState( UT_DAC_PERIPH, DAC_CHANNEL_1, &channelState ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_INACTIVE, channelState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ChannelActive( UT_DAC_PERIPH, DAC_CHANNEL_CNT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ChannelInactive( DAC_PERIPH_CNT, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_ChannelState( UT_DAC_PERIPH, DAC_CHANNEL_1, NULL ) );
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
 * - 12-bit right: DHR12RD = ch1 | ch2 << 16. 12-bit left: DHR12LD = ch1 | ch2 << 16.
 *   8-bit right: DHR8RD = ch1 | ch2 << 8.
 * - Different formats of the channels and data out of range are rejected.
 */
void Ut_Dac_Set_DualData_SameFormat_DualRegisterWritten( void )
{
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DualData( UT_DAC_PERIPH, 0x0123u, 0x0456u ) );
    TEST_ASSERT_EQUAL_HEX32( 0x04560123u, UT_DAC_REG->DHR12RD );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_12BIT_LEFT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DualData( UT_DAC_PERIPH, 0x0010u, 0x0020u ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_DATA_FORMAT_12BIT_LEFT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DualData( UT_DAC_PERIPH, 0x1230u, 0xABC0u ) );
    TEST_ASSERT_EQUAL_HEX32( 0xABC01230u, UT_DAC_REG->DHR12LD );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_8BIT_RIGHT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_DATA_FORMAT_8BIT_RIGHT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DualData( UT_DAC_PERIPH, 0x12u, 0x34u ) );
    TEST_ASSERT_EQUAL_HEX32( 0x3412u, UT_DAC_REG->DHR8RD );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DualData( UT_DAC_PERIPH, 0x12u, 0x134u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DualData( DAC_PERIPH_CNT, 0x12u, 0x34u ) );
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

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DualSwTrigger( DAC_PERIPH_CNT ) );
}


/**
 * \brief   Dac_Get_Flag() reads the DMA underrun flag, Dac_Clear_Flag() clears it.
 *
 * \par Expected results
 * - Underrun flag of channel 2 is reported from SR, channel 1 inactive.
 * - Flags not available on STM32F7 (calibration, busy, ready) return DAC_REQUEST_ERROR.
 * - Clear of underrun writes DMAUDR2, clear of other flags is rejected.
 */
void Ut_Dac_Get_Flag_UnderrunOnly( void )
{
    dac_FlagState_t flagState = DAC_FLAG_INACTIVE;

    UT_DAC_REG->SR = DAC_SR_DMAUDR2;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_FLAG_DMA_UNDERRUN, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_FLAG_ACTIVE, flagState );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_DMA_UNDERRUN, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_FLAG_INACTIVE, flagState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_READY, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_CALIBRATION, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_SH_WRITE_BUSY, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_CNT, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_DMA_UNDERRUN, NULL ) );

    UT_DAC_REG->SR = 0u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Clear_Flag( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_FLAG_READY ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->SR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Clear_Flag( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_FLAG_DMA_UNDERRUN ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_SR_DMAUDR2, UT_DAC_REG->SR );
}

/* ============================== DMA TRANSFER ============================== */

/**
 * \brief   DMA transfer requires a hardware trigger.
 *
 * \details DMA configuration with software trigger.
 *
 * \par Expected results
 * - Dac_Init() and Dac_Set_DataConfig() return DAC_REQUEST_ERROR, DMA is not initialized.
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
 * \details TIM6 trigger with NULL buffer, size 0, buffer in CCM data RAM (MCUs with CCM data
 *          RAM only), DMA stream and peripheral not matching the request mapping, invalid
 *          priority and buffer mode.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR in all cases, DMA is not initialized.
 */
void Ut_Dac_Set_DataConfig_InvalidDmaConfig_ReturnsError( void )
{
    dac_ChannelConfig_t channelCfg;
    dac_DataConfig_t    dataCfg;
    dac_Config_t        config;

    Ut_Dac_Get_Config( &config );
    channelCfg = config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ];
    Ut_Dac_Set_DmaConfig( &channelCfg, DAC_BUFFER_MODE_ONE_SHOT );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_TRIGGER_TIM6_TRGO ) );

    dataCfg            = channelCfg.DataConfig;
    dataCfg.DataBuffer = NULL;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataCfg ) );

    dataCfg            = channelCfg.DataConfig;
    dataCfg.BufferSize = 0u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataCfg ) );

#if defined(CCMDATARAM_BASE)
    dataCfg            = channelCfg.DataConfig;
    dataCfg.DataBuffer = (const dac_Data_t *)(uintptr_t)CCMDATARAM_BASE;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataCfg ) );
#endif /* CCMDATARAM_BASE */

    dataCfg     = channelCfg.DataConfig;
    dataCfg.Dma = DAC_DMA_DAC1_CH2_DMA1_STREAM6;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataCfg ) );

    dataCfg     = channelCfg.DataConfig;
    dataCfg.Dma = DAC_DMA_UNUSED;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataCfg ) );

    dataCfg     = channelCfg.DataConfig;
    dataCfg.Dma = (dac_Dma_t)DAC_DMA_ENCODE( DAC_PERIPH_1, DAC_CHANNEL_1, DAC_DMA_PERIPH_2, DAC_DMA_CHANNEL_5, 7u );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataCfg ) );

    dataCfg             = channelCfg.DataConfig;
    dataCfg.DmaPriority = (dac_DmaPriority_t)DMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataCfg ) );

    dataCfg            = channelCfg.DataConfig;
    dataCfg.BufferMode = DAC_BUFFER_MODE_CNT;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataCfg ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, NULL ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_DmaInitCnt );
}


/**
 * \brief   Dac_Init() in DMA mode initializes the DMA stream and the DAC interrupt.
 *
 * \details Channel 2, TIM6 trigger, DMA buffer of 8 items, 12-bit left aligned format,
 *          half transfer callback set.
 *
 * \par Expected results
 * - Dma_Init() called once: DMA1 stream 6, channel selection 7, memory to peripheral, normal
 *   mode, peripheral = DHR12L2 (static, 16-bit), memory = buffer (increment, 16-bit),
 *   8 items, high priority, transfer complete / half transfer / error handlers registered.
 * - Half transfer interrupt of the stream enabled.
 * - DAC interrupt registered on TIM6_DAC line, DMA requests not enabled yet (DMAEN2 = 0).
 */
void Ut_Dac_Init_DmaMode_StreamInitialized( void )
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
    TEST_ASSERT_EQUAL( UT_DAC_DMA_PERIPH, utDac_DmaConfig.DmaPeriphId );
    TEST_ASSERT_EQUAL( UT_DAC_DMA_STREAM_CH2, utDac_DmaConfig.DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_CHANNEL_7, utDac_DmaConfig.PeripheralReqId );
    TEST_ASSERT_EQUAL( DMA_DIR_MEMORY_TO_PERIPH, utDac_DmaConfig.Direction );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_NORMAL, utDac_DmaConfig.TransferMode );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_DAC_REG->DHR12L2, utDac_DmaConfig.PeriphAddress );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utDac_Buffer, utDac_DmaConfig.MemoryAddress );
    TEST_ASSERT_EQUAL( DMA_PERIPH_ADDR_STATIC, utDac_DmaConfig.PeriphAddrIncrement );
    TEST_ASSERT_EQUAL( DMA_MEMORY_ADDR_INCREMENT, utDac_DmaConfig.MemoryAddrIncrement );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_16BIT, utDac_DmaConfig.PeriphTransferSize );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_16BIT, utDac_DmaConfig.MemoryTransferSize );
    TEST_ASSERT_EQUAL_UINT32( UT_DAC_BUF_SIZE, utDac_DmaConfig.DataCount );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_HIGH, utDac_DmaConfig.Priority );
    TEST_ASSERT_NOT_NULL( utDac_DmaConfig.TransferCompleteCallback );
    TEST_ASSERT_NOT_NULL( utDac_DmaConfig.HalfTransferCallback );
    TEST_ASSERT_NOT_NULL( utDac_DmaConfig.TransferErrorCallback );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaHtIrqState );

    TEST_ASSERT_NOT_NULL( utDac_Isr );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & DAC_CR_DMAEN2 );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_2, &dataConfig ) );
    TEST_ASSERT_EQUAL( DAC_TRANSFER_MODE_DMA, dataConfig.TransferMode );
    TEST_ASSERT_EQUAL_PTR( utDac_Buffer, dataConfig.DataBuffer );

    /* Channel selection of the list item equals the one used by the request map */
    TEST_ASSERT_EQUAL_UINT32( (uint32_t)utDac_DmaConfig.PeripheralReqId >> DMA_SxCR_CHSEL_Pos, DAC_DMA_BIT_MASK_DECODE_CHSEL( DAC_DMA_DAC1_CH2_DMA1_STREAM6 ) );
}


/**
 * \brief   Dac_Init() in DMA mode of channel 1 initializes the stream of its DMA request.
 *
 * \details Channel 1, TIM6 trigger, DMA buffer of 8 items, 12-bit right aligned format.
 *
 * \par Expected results
 * - Dma_Init() called once: DMA1 stream 5, channel selection 7, memory to peripheral,
 *   peripheral = DHR12R1.
 * - The channel selection stored in the list item equals the request map.
 */
void Ut_Dac_Init_DmaModeChannel1_StreamInitialized( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInitCnt );
    TEST_ASSERT_EQUAL( UT_DAC_DMA_PERIPH, utDac_DmaConfig.DmaPeriphId );
    TEST_ASSERT_EQUAL( UT_DAC_DMA_STREAM_CH1, utDac_DmaConfig.DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_CHANNEL_7, utDac_DmaConfig.PeripheralReqId );
    TEST_ASSERT_EQUAL( DMA_DIR_MEMORY_TO_PERIPH, utDac_DmaConfig.Direction );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_DAC_REG->DHR12R1, utDac_DmaConfig.PeriphAddress );

    TEST_ASSERT_EQUAL_UINT32( (uint32_t)utDac_DmaConfig.PeripheralReqId >> DMA_SxCR_CHSEL_Pos, DAC_DMA_BIT_MASK_DECODE_CHSEL( DAC_DMA_DAC1_CH1_DMA1_STREAM5 ) );
}


/**
 * \brief   Items of the DMA stream list carry DAC peripheral, DAC channel, DMA peripheral, stream and
 *          channel selection of the stream.
 *
 * \details Expected values are written as (DAC peripheral index, DAC channel index, DMA peripheral
 *          index, stream number, channel selection number) taken from the DMA1 request mapping of
 *          the STM32F7 reference manual, independently of the encoding macro.
 *
 * \par Expected results
 * - Both items carry the expected bit-fields.
 * - The unused item equals DAC_DMA_CODE_UNUSED, the decoding macros return the fields.
 */
void Ut_Dac_DmaList_Items_EncodePeriphChannelDmaStreamAndChannelSelection( void )
{
    /* DMA1 (index 0), channel selection 7 */
    TEST_ASSERT_EQUAL_HEX32( UT_DAC_DMA_CODE( 0u, 0u, 0u, 5u, 7u ), DAC_DMA_DAC1_CH1_DMA1_STREAM5 );
    TEST_ASSERT_EQUAL_HEX32( UT_DAC_DMA_CODE( 0u, 1u, 0u, 6u, 7u ), DAC_DMA_DAC1_CH2_DMA1_STREAM6 );

    /* Decoding of the fields */
    TEST_ASSERT_EQUAL_UINT32( DAC_PERIPH_1,       DAC_DMA_BIT_MASK_DECODE_PERIPH( DAC_DMA_DAC1_CH2_DMA1_STREAM6 ) );
    TEST_ASSERT_EQUAL_UINT32( DAC_CHANNEL_2,      DAC_DMA_BIT_MASK_DECODE_CHANNEL( DAC_DMA_DAC1_CH2_DMA1_STREAM6 ) );
    TEST_ASSERT_EQUAL_UINT32( DAC_DMA_PERIPH_1,   DAC_DMA_BIT_MASK_DECODE_DMA( DAC_DMA_DAC1_CH2_DMA1_STREAM6 ) );
    TEST_ASSERT_EQUAL_UINT32( DAC_DMA_CHANNEL_6,  DAC_DMA_BIT_MASK_DECODE_STREAM( DAC_DMA_DAC1_CH2_DMA1_STREAM6 ) );
    TEST_ASSERT_EQUAL_UINT32( 7u,                 DAC_DMA_BIT_MASK_DECODE_CHSEL( DAC_DMA_DAC1_CH2_DMA1_STREAM6 ) );

    /* Unused stream */
    TEST_ASSERT_EQUAL_HEX32( DAC_DMA_CODE_UNUSED, DAC_DMA_UNUSED );
    TEST_ASSERT_EQUAL_UINT32( DAC_PERIPH_CNT,      DAC_DMA_BIT_MASK_DECODE_PERIPH( DAC_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( DAC_CHANNEL_CNT,     DAC_DMA_BIT_MASK_DECODE_CHANNEL( DAC_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( DAC_DMA_PERIPH_CNT,  DAC_DMA_BIT_MASK_DECODE_DMA( DAC_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( DAC_DMA_CHANNEL_CNT, DAC_DMA_BIT_MASK_DECODE_STREAM( DAC_DMA_UNUSED ) );
}


/**
 * \brief   DMA transfer without half transfer callback does not use the half transfer interrupt.
 *
 * \par Expected results
 * - Dma_Init() without half transfer callback, half transfer interrupt of the stream disabled.
 */
void Ut_Dac_Init_DmaModeWithoutHalfCallback_HalfTransferIrqDisabled( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_CIRCULAR );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataConfig.HalfTransferCallback = DAC_NULL_PTR;
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( UT_DAC_DMA_STREAM_CH1, utDac_DmaConfig.DmaChannel );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_CIRCULAR, utDac_DmaConfig.TransferMode );
    TEST_ASSERT_NULL( utDac_DmaConfig.HalfTransferCallback );
    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_DmaHtIrqState );
}


/**
 * \brief   Dac_Set_DmaStart() arms the DMA stream and enables DAC DMA requests.
 *
 * \par Expected results
 * - DMA1 stream 5: normal mode, peripheral DHR12R1, memory buffer, 8 items, stream enabled.
 * - DMAEN1 and DMAUDRIE1 set, second start refused, direct data write and format change refused.
 * - Dac_Set_DmaStop() clears DMAEN1 / DMAUDRIE1 and disables the DMA stream.
 */
void Ut_Dac_Set_DmaStart_InitializedDma_RequestsEnabled( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    TEST_ASSERT_EQUAL( UT_DAC_DMA_STREAM_CH1, utDac_DmaStream );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_NORMAL, utDac_DmaMode );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_DAC_REG->DHR12R1, utDac_DmaPeriphAddr );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utDac_Buffer, utDac_DmaMemoryAddr );
    TEST_ASSERT_EQUAL_UINT32( UT_DAC_BUF_SIZE, utDac_DmaDataCount );
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
 * \brief   Dac_Set_DmaStop() releases a DAC DMA request pending at the stop.
 *
 * \details Circular DMA output of channel 1 is started and stopped with the underrun flag set.
 *
 * \note    Device errata "DMA request not automatically cleared by clearing DMAEN" (STM32F7
 *          ES0182 2.6.1, bug AB#662 - workaround kept on STM32F7, confirmed by the errata review
 *          AB#953): a DAC DMA request pending at the stop is not cleared by DMAEN = 0
 *          and would be serviced immediately by the next start (spurious transfer not aligned
 *          to the trigger). Errata sequence: DMAEN cleared, underrun flag checked and cleared,
 *          DAC clock disabled and enabled again, DMA stream configured again by the next start.
 *
 * \par Expected results
 * - DAC clock (RCC_PERIPH_DAC) disabled once with DMAEN1 already cleared and the DMA stream
 *   already disabled, then enabled again.
 * - DMAUDR1 written (cleared), channel configuration (CR without DMAEN1 / DMAUDRIE1) kept.
 * - Next start arms the DMA stream again.
 */
void Ut_Dac_Set_DmaStop_PendingRequest_DacClockRestarted( void )
{
    dac_Config_t config;
    uint32_t     crConfig = 0u;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_CIRCULAR );
    Ut_Dac_Init( &config );

    Rcc_Set_PeriphInactive_StubWithCallback( Ut_Dac_RccSetInactiveStub );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    crConfig           = UT_DAC_REG->CR & ~( DAC_CR_DMAEN1 | DAC_CR_DMAUDRIE1 );
    utDac_RccActiveCnt = 0u;

    /* Request is pending, underrun of the stopped transfer */
    UT_DAC_REG->SR = DAC_SR_DMAUDR1;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStop( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_RccInactiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, utDac_RccInactiveDmaEn );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_RccInactiveDmaStops );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_RccActiveCnt );
    TEST_ASSERT_EQUAL( RCC_PERIPH_DAC, utDac_RccLastActive );
    TEST_ASSERT_EQUAL_HEX32( DAC_SR_DMAUDR1, UT_DAC_REG->SR );
    TEST_ASSERT_EQUAL_HEX32( crConfig, UT_DAC_REG->CR );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utDac_DmaActiveCnt );
}


/**
 * \brief   Dac_Set_DmaStart() is refused for disabled channel and direct transfer.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR, DMA stream not enabled, DMAEN1 = 0.
 * - Hardware trigger can not be removed while DMA transfer is configured.
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
 * \brief   Failed DMA stream start leaves the channel stopped.
 *
 * \details Dma_Set_TransferActive() returns error.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR, DMAEN1 / DMAUDRIE1 not set, DMA stream stopped, new start possible.
 */
void Ut_Dac_Set_DmaStart_StreamError_TransferStopped( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );

    utDac_DmaActiveResult = DMA_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & ( DAC_CR_DMAEN1 | DAC_CR_DMAUDRIE1 ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInactiveCnt );

    utDac_DmaActiveResult = DMA_REQUEST_OK;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
}


/**
 * \brief   One shot transfer complete stops the transfer and calls the callback.
 *
 * \par Expected results
 * - Half transfer handler calls HalfTransferCallback.
 * - Transfer complete: DMAEN1 cleared, DMA stream disabled, TransferCompleteCallback
 *   called once, new start possible.
 */
void Ut_Dac_DmaComplete_OneShot_TransferStopped( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    utDac_DmaConfig.HalfTransferCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_HalfCnt );

    utDac_DmaConfig.TransferCompleteCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_CompleteCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & DAC_CR_DMAEN1 );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaActiveCnt );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utDac_DmaActiveCnt );
}


/**
 * \brief   Circular transfer keeps running after transfer complete.
 *
 * \par Expected results
 * - Stream started in circular mode.
 * - Transfer complete: TransferCompleteCallback called, DMA stream neither re-armed nor
 *   disabled by SW, DMAEN1 kept.
 */
void Ut_Dac_DmaComplete_Circular_StreamKeptRunning( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_CIRCULAR );
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_CIRCULAR, utDac_DmaMode );

    utDac_DmaConfig.TransferCompleteCallback();
    utDac_DmaConfig.TransferCompleteCallback();

    TEST_ASSERT_EQUAL_UINT32( 2u, utDac_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaActiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_DmaInactiveCnt );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_DMAEN1, UT_DAC_REG->CR & DAC_CR_DMAEN1 );
}


/**
 * \brief   DMA stream is reprogrammed on start from the actual configuration.
 *
 * \details One shot transfer started and completed, data format changed to 8-bit right
 *          aligned, transfer started again.
 *
 * \par Expected results
 * - Peripheral address of the second start is DHR8R1, 8 items.
 */
void Ut_Dac_Set_DmaStart_FormatChanged_StreamReprogrammed( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    utDac_DmaConfig.TransferCompleteCallback();

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_8BIT_RIGHT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utDac_DmaActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_DAC_REG->DHR8R1, utDac_DmaPeriphAddr );
    TEST_ASSERT_EQUAL_UINT32( UT_DAC_BUF_SIZE, utDac_DmaDataCount );
}


/**
 * \brief   DMA transfer error stops the transfer and reports the error.
 *
 * \par Expected results
 * - ErrorCallback called with DAC_ERROR_DMA_TRANSFER, DMAEN1 cleared, stream disabled.
 */
void Ut_Dac_DmaError_TransferError_ReportedAndStopped( void )
{
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_CIRCULAR );
    Ut_Dac_Init( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    utDac_DmaConfig.TransferErrorCallback();

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_ErrorCnt );
    TEST_ASSERT_EQUAL( DAC_ERROR_DMA_TRANSFER, utDac_LastError );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & DAC_CR_DMAEN1 );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInactiveCnt );
}


/**
 * \brief   DAC interrupt reports DMA underrun of the channel.
 *
 * \details DMA transfer on channel 2 running, DMAUDR2 set in SR, DAC ISR called.
 *          Second call without pending flag.
 *
 * \par Expected results
 * - ErrorCallback called once with DAC_ERROR_DMA_UNDERRUN.
 * - DMAEN2 and DMAUDRIE2 cleared, DMA stream disabled.
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

    UT_DAC_REG->SR = DAC_SR_DMAUDR2;
    utDac_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_ErrorCnt );
    TEST_ASSERT_EQUAL( DAC_ERROR_DMA_UNDERRUN, utDac_LastError );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & ( DAC_CR_DMAEN2 | DAC_CR_DMAUDRIE2 ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInactiveCnt );

    UT_DAC_REG->SR = 0u;
    utDac_Isr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_ErrorCnt );
}


/**
 * \brief   DMA handlers of channel 2 report the events of channel 2.
 *
 * \details Channel 2 in DMA mode (DMA1 stream 6), one shot transfer started, handlers captured
 *          from Dma_Init() are called: half transfer, transfer complete; transfer started again
 *          and transfer error handler called. Dac_Task() is called at the end. MCUs without
 *          DAC channel 2: test ignored.
 *
 * \par Expected results
 * - Half transfer: HalfTransferCallback called once.
 * - Transfer complete: TransferCompleteCallback called once, DMAEN2 cleared, stream disabled.
 * - Transfer error: ErrorCallback with DAC_ERROR_DMA_TRANSFER, DMAEN2 cleared.
 * - Dac_Task() does not change the DAC registers.
 */
void Ut_Dac_DmaChannel2_HandlersReportChannel2( void )
{
#if defined(DAC_CR_EN2)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_INACTIVE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );
    TEST_ASSERT_EQUAL( UT_DAC_DMA_STREAM_CH2, utDac_DmaConfig.DmaChannel );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_2 ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_DMAEN2, UT_DAC_REG->CR & DAC_CR_DMAEN2 );

    utDac_DmaConfig.HalfTransferCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_HalfCnt );

    utDac_DmaConfig.TransferCompleteCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_CompleteCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & DAC_CR_DMAEN2 );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInactiveCnt );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_2 ) );
    utDac_DmaConfig.TransferErrorCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_ErrorCnt );
    TEST_ASSERT_EQUAL( DAC_ERROR_DMA_TRANSFER, utDac_LastError );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & DAC_CR_DMAEN2 );

    const uint32_t crValue = UT_DAC_REG->CR;

    Dac_Task();

    TEST_ASSERT_EQUAL_HEX32( crValue, UT_DAC_REG->CR );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC channel 2" );
#endif /* DAC_CR_EN2 */
}

/* ========================== LOCAL FUNCTIONS =============================== */

/** \brief RCC activation stub - stores the last activated peripheral */
static rcc_RequestState_t Ut_Dac_RccSetActiveStub( rcc_PeriphId_t periphId, int callCnt )
{
    (void)callCnt;

    utDac_RccLastActive = periphId;
    utDac_RccActiveCnt++;

    return ( RCC_REQUEST_OK );
}


/** \brief RCC deactivation stub of the DAC clock - stores DMAEN1 and count of disabled DMA
 *         streams at the call (errata sequence order) */
static rcc_RequestState_t Ut_Dac_RccSetInactiveStub( rcc_PeriphId_t periphId, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( RCC_PERIPH_DAC, periphId );

    utDac_RccInactiveDmaEn    = UT_DAC_REG->CR & DAC_CR_DMAEN1;
    utDac_RccInactiveDmaStops = utDac_DmaInactiveCnt;
    utDac_RccInactiveCnt++;

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


/** \brief DMA initialization stub - stores the stream configuration */
static dma_RequestState_t Ut_Dac_DmaInitStub( dma_ConfigStruct_t * const dmaConfig, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( dmaConfig );

    utDac_DmaConfig = *dmaConfig;
    utDac_DmaInitCnt++;

    return ( DMA_REQUEST_OK );
}


/** \brief DMA transfer mode stub */
static dma_RequestState_t Ut_Dac_DmaModeStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_TransferMode_t transferMode, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( UT_DAC_DMA_PERIPH, dmaBus );

    utDac_DmaStream = dmaChannel;
    utDac_DmaMode   = transferMode;

    return ( DMA_REQUEST_OK );
}


/** \brief DMA peripheral address stub */
static dma_RequestState_t Ut_Dac_DmaPeriphAddrStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_PeriphAddr_t periphAddr, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    utDac_DmaPeriphAddr = periphAddr;

    return ( DMA_REQUEST_OK );
}


/** \brief DMA memory address stub */
static dma_RequestState_t Ut_Dac_DmaMemoryAddrStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_MemoryAddr_t memoryAddr, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    utDac_DmaMemoryAddr = memoryAddr;

    return ( DMA_REQUEST_OK );
}


/** \brief DMA item count stub */
static dma_RequestState_t Ut_Dac_DmaDataCountStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t dataCount, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    utDac_DmaDataCount = dataCount;

    return ( DMA_REQUEST_OK );
}


/** \brief DMA stream enable stub - result given by utDac_DmaActiveResult */
static dma_RequestState_t Ut_Dac_DmaActiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    utDac_DmaActiveCnt++;

    return ( utDac_DmaActiveResult );
}


/** \brief DMA stream disable stub */
static dma_RequestState_t Ut_Dac_DmaInactiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    utDac_DmaInactiveCnt++;

    return ( DMA_REQUEST_OK );
}


/** \brief DMA half transfer interrupt enable stub */
static dma_RequestState_t Ut_Dac_DmaHtIrqActiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    utDac_DmaHtIrqState = 1u;

    return ( DMA_REQUEST_OK );
}


/** \brief DMA half transfer interrupt disable stub */
static dma_RequestState_t Ut_Dac_DmaHtIrqInactiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    utDac_DmaHtIrqState = 0u;

    return ( DMA_REQUEST_OK );
}


/**
 * \brief Ignores / stubs all calls of RCC / NVIC / GPIO / DMA functions used by the module.
 */
static void Ut_Dac_Ignore_PeriphMocks( void )
{
    Rcc_Set_PeriphActive_StubWithCallback( Ut_Dac_RccSetActiveStub );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );

    Gpio_Init_StubWithCallback( Ut_Dac_GpioInitStub );

    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Dac_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_IgnoreAndReturn( NVIC_REQUEST_OK );

    Dma_Get_DefaultConfig_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Init_StubWithCallback( Ut_Dac_DmaInitStub );
    Dma_Set_TransferCompleteIrqActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferErrorIrqActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_HalfTransferIrqActive_StubWithCallback( Ut_Dac_DmaHtIrqActiveStub );
    Dma_Set_HalfTransferIrqInactive_StubWithCallback( Ut_Dac_DmaHtIrqInactiveStub );
    Dma_Set_InterruptActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_InterruptInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferMode_StubWithCallback( Ut_Dac_DmaModeStub );
    Dma_Set_PeriphAddr_StubWithCallback( Ut_Dac_DmaPeriphAddrStub );
    Dma_Set_MemoryAddr_StubWithCallback( Ut_Dac_DmaMemoryAddrStub );
    Dma_Set_DataCount_StubWithCallback( Ut_Dac_DmaDataCountStub );
    Dma_Set_TransferActive_StubWithCallback( Ut_Dac_DmaActiveStub );
    Dma_Set_TransferInactive_StubWithCallback( Ut_Dac_DmaInactiveStub );
}


/**
 * \brief Removes all expectations and ignores of the mocks.
 */
static void Ut_Dac_Reset_Mocks( void )
{
    MockRcc_Port_Destroy();
    MockNvic_Port_Destroy();
    MockGpio_Port_Destroy();
    MockDma_Port_Destroy();
    MockRcc_Port_Init();
    MockNvic_Port_Init();
    MockGpio_Port_Init();
    MockDma_Port_Init();
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
 * \brief Configures DMA transfer of a channel with TIM6 trigger and test buffer (DMA stream
 *        of the channel is kept from the default configuration).
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
    channelConfig->DataConfig.DmaPriority              = DAC_DMA_PRIORITY_HIGH;
    channelConfig->DataConfig.IrqPriority              = UT_DAC_PRIO;
    channelConfig->DataConfig.HalfTransferCallback     = Ut_Dac_HalfCallback;
    channelConfig->DataConfig.TransferCompleteCallback = Ut_Dac_CompleteCallback;
    channelConfig->DataConfig.ErrorCallback            = Ut_Dac_ErrorCallback;
}


/**
 * \brief Initializes the module with stubbed peripheral modules.
 *
 * \param config [in]: Module configuration
 */
static void Ut_Dac_Init( dac_Config_t * const config )
{
    Ut_Dac_Ignore_PeriphMocks();

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
