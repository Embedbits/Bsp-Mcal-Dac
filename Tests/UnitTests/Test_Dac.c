/**
 * \author Mr.Nobody
 * \file Test_Dac.c
 * \ingroup Dac
 * \brief Unit tests of Digital-to-Analog Converter (DAC) module (STM32L4 / STM32L4+).
 *
 * Dac.c, Dac_Dma.c and Dac_Isr.c are compiled unchanged with real LL drivers.
 * DAC registers are emulated by RegMem, RCC, NVIC, GPIO and DMA modules are
 * mocked by CMock. Handlers registered in NVIC / DMA are captured by stubs and
 * called directly to test interrupt handling.
 *
 * The offset calibration flag (CAL_FLAGx set while CENx = 1 and OTRIMx >=
 * utDac_CalThreshold) is emulated by HW model running in background thread
 * (Ut_Dac_HwModel). STM32L4 DAC has no channel ready flag.
 *
 * Variants of the presets: STM32L41x / L42x have no DAC (all tests ignored),
 * STM32L45x / L46x have a single channel DAC (tests of channel 2 ignored, single
 * channel test active), STM32L4+ has 4-bit trigger selection (TIM1 / TIM15 / LPTIM).
 *
 * \note Emulated registers are plain memory - data holding registers are not
 *       moved to the output registers, write 1 to clear flags stays set.
 *
 * \note Data format and transfer context are kept in static module context -
 *       every test is executed as separate process (CTest), setUp() additionally
 *       releases the context of the peripheral (Dac_Deinit()).
 */

/* ============================= INCLUDES =================================== */
#include <string.h>                         /* memset                         */
#include "unity.h"                          /* Unity testing framework        */
#include "UtCommon.h"                       /* Common test helpers            */
#include "RegMem.h"                         /* Register memory emulation      */
#include "CmsisHost.h"                      /* Core intrinsics emulation      */
#include "Dac_Port.h"                       /* Module under test              */
#include "Dac_Dma.h"                        /* DMA handler (configuration check) */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockDma_Port.h"                   /* DMA module mock                */
#include "Stm32_dac.h"                      /* DAC registers definition       */
/* ============================= TYPEDEFS =================================== */

/** \brief Row of the expected DMA channel lists (STM32CubeMX DMA database - RM0351 / RM0394 / RM0432 request mapping) */
typedef struct
{
    uint32_t            Item;      /**< Item of the list                                  */
    uint32_t            Direction; /**< Direction (UT_DAC_DMA_DIR_TX / UT_DAC_DMA_DIR_RX) */
    dac_ChannelId_t     PeriphId;  /**< DAC channel                                       */
    uint32_t            DmaId;     /**< DMA peripheral index (0 - DMA1, 1 - DMA2)         */
    uint32_t            Channel;   /**< DMA channel index (0 - channel 1)                 */
    uint32_t            Selection; /**< DMA_CSELR request selection (0 with DMAMUX1)      */
    dma_PeriphReqId_t   Request;   /**< DMA request of the peripheral and direction       */
}   utDac_DmaRow_t;

#if defined(DAC1)
/* ======================= FORWARD DECLARATIONS ============================= */

static void                 Ut_Dac_HwModel              ( void );
static void                 Ut_Dac_Set_ModelActive      ( void );
static rcc_RequestState_t   Ut_Dac_RccGetClkStub        ( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt );
static rcc_RequestState_t   Ut_Dac_RccSetActiveStub     ( rcc_PeriphId_t periphId, int callCnt );
static rcc_RequestState_t   Ut_Dac_RccSetInactiveStub   ( rcc_PeriphId_t periphId, int callCnt );
static rcc_RequestState_t   Ut_Dac_RccGetOscStateStub   ( rcc_OscId_t oscId, rcc_FunctionState_t * const retState, int callCnt );
static rcc_RequestState_t   Ut_Dac_RccSetOscActiveStub  ( rcc_OscId_t oscId, int callCnt );
static gpio_RequestState_t  Ut_Dac_GpioInitStub         ( gpio_Config_t *gpioConfig, int callCnt );
static nvic_RequestState_t  Ut_Dac_NvicSetHandlerStub   ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static nvic_RequestState_t  Ut_Dac_NvicSetInactiveStub  ( nvic_PeriphIrqList_t irqId, int callCnt );
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

/** Default DMA channels of DAC1 channels (STM32L4 request mapping: DMA1 channel 3 / 4) */
#define UT_DAC_DMA_PERIPH                   ( DMA_PERIPH_1 )
#define UT_DAC_DMA_CHANNEL_CH1              ( DMA_CHANNEL_3 )
#define UT_DAC_DMA_CHANNEL_CH2              ( DMA_CHANNEL_4 )

/** HCLK frequency returned by RCC stub by default [Hz] */
#define UT_DAC_HCLK_HZ                      ( 80000000u )

/** Default trimming value threshold of the calibration model */
#define UT_DAC_CAL_THRESHOLD                ( 11u )

/** Interrupt priority of test configurations */
#define UT_DAC_PRIO                         ( 6u )

/** Size of the test data buffer */
#define UT_DAC_BUF_SIZE                     ( 8u )

/** MODE bits of channel 1 in MCR */
#define UT_DAC_MCR_MODE1                    ( DAC_MCR_MODE1 )

#if defined(DAC_CHANNEL2_SUPPORT)
/** MODE bits of channel 2 in MCR */
#define UT_DAC_MCR_MODE2                    ( DAC_MCR_MODE2 )

/** MODE bits of all channels in MCR */
#define UT_DAC_MCR_MODES                    ( DAC_MCR_MODE1 | DAC_MCR_MODE2 )
#else
/** MODE bits of all channels in MCR (single channel DAC) */
#define UT_DAC_MCR_MODES                    ( DAC_MCR_MODE1 )
#endif /* DAC_CHANNEL2_SUPPORT */

/* ============================== MACROS ==================================== */

/** Encoded DMA channel from the DAC peripheral, DAC channel index (0 - channel 1), DMA peripheral index (0 - DMA1),
 *  DMA channel index (0 - channel 1) and request selection (bit-fields written independently of DAC_DMA_ENCODE) */
#define UT_DAC_DMA_CODE( PERIPH, DACCH, DMA, CHANNEL, SEL )    ( ( (PERIPH) << 20u ) | ( (DACCH) << 15u ) | ( (DMA) << 10u ) | ( (CHANNEL) << 5u ) | (SEL) )

/** Directions of the rows of the expected DMA channel lists */
#define UT_DAC_DMA_DIR_TX                    ( 0u )
#define UT_DAC_DMA_DIR_RX                    ( 1u )

/** Row of the expected DMA channel lists */
#define UT_DAC_DMA_ROW( ITEM, DIR, PERIPH, DMA, CHANNEL, SEL, REQ )   { (uint32_t)(ITEM), (DIR), (PERIPH), (DMA), (CHANNEL), (SEL), (REQ) }

/** Count of the rows of the expected DMA channel lists */
#define UT_DAC_DMA_ROW_CNT                   ( sizeof( utDac_DmaRows ) / sizeof( utDac_DmaRows[ 0u ] ) )

/* ========================== LOCAL VARIABLES =============================== */

#if defined(DAC1)
/** Expected DMA channel lists (rows follow the lists of the module header; DMA peripheral and channel indexes are 0 based) */
static const utDac_DmaRow_t utDac_DmaRows[] =
{
#if !defined(DMAMUX1)
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA1_CHANNEL3      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 0u, 2u, 6u, DMA_REQ_DAC1_CH1 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA2_CHANNEL4      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 1u, 3u, 3u, DMA_REQ_DAC1_CH1 ),
#if defined(DAC_CHANNEL2_SUPPORT)
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA1_CHANNEL4      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 0u, 3u, 5u, DMA_REQ_DAC1_CH2 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA2_CHANNEL5      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 1u, 4u, 3u, DMA_REQ_DAC1_CH2 ),
#endif /* DAC_CHANNEL2_SUPPORT */
#else
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA1_CHANNEL1      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 0u, 0u, 0u, DMA_REQ_DAC1_CH1 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA1_CHANNEL2      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 0u, 1u, 0u, DMA_REQ_DAC1_CH1 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA1_CHANNEL3      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 0u, 2u, 0u, DMA_REQ_DAC1_CH1 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA1_CHANNEL4      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 0u, 3u, 0u, DMA_REQ_DAC1_CH1 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA1_CHANNEL5      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 0u, 4u, 0u, DMA_REQ_DAC1_CH1 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA1_CHANNEL6      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 0u, 5u, 0u, DMA_REQ_DAC1_CH1 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA1_CHANNEL7      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 0u, 6u, 0u, DMA_REQ_DAC1_CH1 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA2_CHANNEL1      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 1u, 0u, 0u, DMA_REQ_DAC1_CH1 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA2_CHANNEL2      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 1u, 1u, 0u, DMA_REQ_DAC1_CH1 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA2_CHANNEL3      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 1u, 2u, 0u, DMA_REQ_DAC1_CH1 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA2_CHANNEL4      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 1u, 3u, 0u, DMA_REQ_DAC1_CH1 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA2_CHANNEL5      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 1u, 4u, 0u, DMA_REQ_DAC1_CH1 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA2_CHANNEL6      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 1u, 5u, 0u, DMA_REQ_DAC1_CH1 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH1_DMA2_CHANNEL7      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_1, 1u, 6u, 0u, DMA_REQ_DAC1_CH1 ),
#if defined(DAC_CHANNEL2_SUPPORT)
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA1_CHANNEL1      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 0u, 0u, 0u, DMA_REQ_DAC1_CH2 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA1_CHANNEL2      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 0u, 1u, 0u, DMA_REQ_DAC1_CH2 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA1_CHANNEL3      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 0u, 2u, 0u, DMA_REQ_DAC1_CH2 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA1_CHANNEL4      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 0u, 3u, 0u, DMA_REQ_DAC1_CH2 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA1_CHANNEL5      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 0u, 4u, 0u, DMA_REQ_DAC1_CH2 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA1_CHANNEL6      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 0u, 5u, 0u, DMA_REQ_DAC1_CH2 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA1_CHANNEL7      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 0u, 6u, 0u, DMA_REQ_DAC1_CH2 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA2_CHANNEL1      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 1u, 0u, 0u, DMA_REQ_DAC1_CH2 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA2_CHANNEL2      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 1u, 1u, 0u, DMA_REQ_DAC1_CH2 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA2_CHANNEL3      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 1u, 2u, 0u, DMA_REQ_DAC1_CH2 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA2_CHANNEL4      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 1u, 3u, 0u, DMA_REQ_DAC1_CH2 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA2_CHANNEL5      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 1u, 4u, 0u, DMA_REQ_DAC1_CH2 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA2_CHANNEL6      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 1u, 5u, 0u, DMA_REQ_DAC1_CH2 ),
    UT_DAC_DMA_ROW( DAC_DMA_DAC1_CH2_DMA2_CHANNEL7      , UT_DAC_DMA_DIR_TX, DAC_CHANNEL_2, 1u, 6u, 0u, DMA_REQ_DAC1_CH2 ),
#endif /* DAC_CHANNEL2_SUPPORT */
#endif /* DMAMUX1 */
};
#endif /* DAC1 */

/** HCLK frequency returned by RCC stub */
static rcc_FreqHz_t             utDac_HclkHz;

/** Last peripheral activated / deactivated in RCC and count of activations */
static rcc_PeriphId_t           utDac_RccLastActive;
static uint32_t                 utDac_RccActiveCnt;
static rcc_PeriphId_t           utDac_RccLastInactive;

/** LSI state returned by RCC stub and count of LSI starts */
static rcc_FunctionState_t      utDac_LsiState;
static uint32_t                 utDac_LsiStartCnt;

/** Trimming value from which the calibration model sets CAL_FLAGx */
static volatile uint32_t        utDac_CalThreshold;

/** GPIO configuration of the last Gpio_Init call and count of calls */
static gpio_Config_t            utDac_GpioConfig;
static uint32_t                 utDac_GpioInitCnt;

/** ISR registered in NVIC for the DAC, its line and count of line deactivations */
static nvic_IsrCallback_t       utDac_Isr;
static nvic_PeriphIrqList_t     utDac_IsrIrqId;
static uint32_t                 utDac_NvicInactiveCnt;

/** DMA configuration captured by Dma_Init stub */
static dma_ConfigStruct_t       utDac_DmaConfig;
static uint32_t                 utDac_DmaInitCnt;

/** Values of the last DMA channel arming calls */
static dma_PeriphId_t           utDac_DmaBus;
static dma_ChannelId_t          utDac_DmaChannel;
static dma_TransferMode_t       utDac_DmaMode;
static dma_PeriphAddr_t         utDac_DmaPeriphAddr;
static dma_MemoryAddr_t         utDac_DmaMemoryAddr;
static dma_DataCount_t          utDac_DmaDataCount;
static uint32_t                 utDac_DmaActiveCnt;
static uint32_t                 utDac_DmaInactiveCnt;
static dma_ChannelId_t          utDac_DmaInactiveChannel;
static dma_RequestState_t       utDac_DmaActiveResult;

/** Half transfer interrupt state of the DMA channel (1 - enabled, 0 - disabled) */
static uint32_t                 utDac_DmaHtIrqState;

/** Output data buffer */
static dac_Data_t               utDac_Buffer[ UT_DAC_BUF_SIZE ];

/** Counts of callback calls and the last error */
static uint32_t                 utDac_HalfCnt;
static uint32_t                 utDac_CompleteCnt;
static uint32_t                 utDac_ErrorCnt;
static dac_ErrorId_t            utDac_LastError;
#endif /* DAC1 */

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
#if defined(DAC1)
    /* Stops HW model of previous test and clears registers */
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utDac_HclkHz        = UT_DAC_HCLK_HZ;
    utDac_CalThreshold  = UT_DAC_CAL_THRESHOLD;
    utDac_LsiState      = RCC_FUNCTION_INACTIVE;

    Ut_Dac_Release();

    utDac_RccLastActive      = RCC_PERIPH_ID_CNT;
    utDac_RccLastInactive    = RCC_PERIPH_ID_CNT;
    utDac_RccActiveCnt       = 0u;
    utDac_LsiStartCnt        = 0u;
    utDac_GpioInitCnt        = 0u;
    utDac_Isr                = NULL;
    utDac_IsrIrqId           = NVIC_PERIPH_IRQ_SIZE;
    utDac_NvicInactiveCnt    = 0u;
    utDac_DmaInitCnt         = 0u;
    utDac_DmaBus             = DMA_PERIPH_CNT;
    utDac_DmaChannel         = DMA_CHANNEL_CNT;
    utDac_DmaMode            = DMA_TRANSFER_MODE_NORMAL;
    utDac_DmaPeriphAddr      = 0u;
    utDac_DmaMemoryAddr      = 0u;
    utDac_DmaDataCount       = 0u;
    utDac_DmaActiveCnt       = 0u;
    utDac_DmaInactiveCnt     = 0u;
    utDac_DmaInactiveChannel = DMA_CHANNEL_CNT;
    utDac_DmaActiveResult    = DMA_REQUEST_OK;
    utDac_DmaHtIrqState      = 0xFFu;
    utDac_HalfCnt            = 0u;
    utDac_CompleteCnt        = 0u;
    utDac_ErrorCnt           = 0u;
    utDac_LastError          = DAC_ERROR_CNT;

    (void)memset( &utDac_DmaConfig, 0, sizeof( utDac_DmaConfig ) );

    for( uint32_t idx = 0u; UT_DAC_BUF_SIZE > idx; idx++ )
    {
        utDac_Buffer[ idx ] = (dac_Data_t)( idx * 100u );
    }
#endif /* DAC1 */
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
#if defined(DAC1)
    const dac_ModuleVersion_t version = Dac_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
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
#if defined(DAC1)
    dac_Config_t config;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( NULL ) );

    Ut_Dac_Get_Config( &config );
    config.ClockSource = DAC_CLK_SRC_CNT;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Get_DefaultConfig() fills the default configuration.
 *
 * \par Expected results
 * - PCLK1 clock, DAC1 channel 1 used (buffered pin output, no trigger, 12-bit right,
 *   direct transfer), channel 2 not used, sample and hold clock LSI.
 * - Default DMA channels follow the STM32L4 request mapping (DMA1 channel 3 / 4).
 * - NULL pointer is rejected.
 */
void Ut_Dac_Get_DefaultConfig_DefaultValues( void )
{
#if defined(DAC1)
    dac_Config_t config;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_DefaultConfig( NULL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( DAC_CLK_SRC_PCLK1, config.ClockSource );
    TEST_ASSERT_EQUAL( UT_DAC_PERIPH, config.PeriphConfig[ UT_DAC_PERIPH ].PeriphId );
    TEST_ASSERT_EQUAL( DAC_SH_CLK_SRC_LSI,    config.PeriphConfig[ UT_DAC_PERIPH ].ShClockSource );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE,   config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed );
#if defined(DAC_CHANNEL2_SUPPORT)
    TEST_ASSERT_EQUAL( DAC_FUNCTION_INACTIVE, config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed );
#endif /* DAC_CHANNEL2_SUPPORT */
    TEST_ASSERT_EQUAL( DAC_OUTPUT_PIN_BUFFERED,     config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].OutputMode );
#if defined(DAC_CHANNEL2_SUPPORT)
    TEST_ASSERT_EQUAL( DAC_OUTPUT_PIN_BUFFERED,     config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].OutputMode );
#endif /* DAC_CHANNEL2_SUPPORT */
    TEST_ASSERT_EQUAL( DAC_TRIGGER_NONE,            config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].TriggerId );
    TEST_ASSERT_EQUAL( DAC_DATA_FORMAT_12BIT_RIGHT, config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataFormat );
    TEST_ASSERT_EQUAL( DAC_TRANSFER_MODE_DIRECT,    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataConfig.TransferMode );

    TEST_ASSERT_EQUAL( DAC_DMA_DAC1_CH1_DMA1_CHANNEL3, config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataConfig.Dma );
#if defined(DAC_CHANNEL2_SUPPORT)
    TEST_ASSERT_EQUAL( DAC_DMA_DAC1_CH2_DMA1_CHANNEL4, config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].DataConfig.Dma );
#endif /* DAC_CHANNEL2_SUPPORT */
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Init() with default configuration initializes channel 1.
 *
 * \details Default configuration with initial value 0x123.
 *
 * \par Expected results
 * - DAC_REQUEST_OK, APB1 clock of DAC1 (RCC_PERIPH_DAC1) activated in RCC (once).
 * - PA4 configured as analog without pull.
 * - Channel 1 enabled, trigger disabled, buffered pin output (MODE1 = 0), normal mode (CEN1 = 0).
 * - DHR12R1 contains the initial value, channel 2 not touched.
 * - Trimming value found by calibration (model threshold) stored in OTRIM1.
 */
void Ut_Dac_Init_DefaultConfig_Channel1Initialized( void )
{
#if defined(DAC1)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].InitValue = 0x123u;
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( RCC_PERIPH_DAC1, utDac_RccLastActive );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_RccActiveCnt );

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_GpioInitCnt );
    TEST_ASSERT_EQUAL( GPIO_PORT_A, utDac_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_4, utDac_GpioConfig.PinId );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ANALOG, utDac_GpioConfig.PinMode );
    TEST_ASSERT_EQUAL( GPIO_PIN_PULL_NONE, utDac_GpioConfig.PinPull );

    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN1, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->MCR & UT_DAC_MCR_MODES );
    TEST_ASSERT_EQUAL_HEX32( 0x123u, UT_DAC_REG->DHR12R1 );
    TEST_ASSERT_EQUAL_HEX32( UT_DAC_CAL_THRESHOLD, UT_DAC_REG->CCR & DAC_CCR_OTRIM1 );
#if defined(DAC_CHANNEL2_SUPPORT)
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CCR & DAC_CCR_OTRIM2 );
#endif /* DAC_CHANNEL2_SUPPORT */
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Init() of unbuffered outputs skips the calibration, internal output has no pin.
 *
 * \details Channel 1 with internal output (initial value 0x111), then with unbuffered pin output.
 *
 * \par Expected results
 * - Internal output: DAC_REQUEST_OK, no Gpio_Init call, MODE1 = 011, EN1 set, OTRIM1 = 0
 *   (no calibration - the pin is never connected to the buffer).
 * - Unbuffered pin output: PA4 analog, MODE1 = 010, OTRIM1 = 0.
 */
void Ut_Dac_Init_UnbufferedOutputs_NoCalibration( void )
{
#if defined(DAC1)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].OutputMode = DAC_OUTPUT_INTERNAL;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].InitValue  = 0x111u;
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_GpioInitCnt );
    TEST_ASSERT_EQUAL_HEX32( DAC_MCR_MODE1_0 | DAC_MCR_MODE1_1, UT_DAC_REG->MCR & UT_DAC_MCR_MODE1 );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN1, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CCR );
    TEST_ASSERT_EQUAL_HEX32( 0x111u, UT_DAC_REG->DHR12R1 );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Deinit( UT_DAC_PERIPH ) );

    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].OutputMode = DAC_OUTPUT_PIN_UNBUFFERED;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_GpioInitCnt );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_4, utDac_GpioConfig.PinId );
    TEST_ASSERT_EQUAL_HEX32( DAC_MCR_MODE1_1, UT_DAC_REG->MCR & UT_DAC_MCR_MODE1 );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CCR );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Single channel DAC (STM32L45x / L46x) has no channel 2 in the list.
 *
 * \details Channel list with one item, requests for the channel out of the list, dual services.
 *          MCUs with two channels or without DAC: test ignored.
 *
 * \par Expected results
 * - DAC_CHANNEL_CNT == 1, the channel services refuse the channel out of the list (DAC_REQUEST_ERROR, CR = 0).
 * - Dual data and dual software trigger: DAC_REQUEST_ERROR.
 */
void Ut_Dac_SingleChannel_Channel2Refused( void )
{
#if defined(DAC1) && \
    !defined(DAC_CHANNEL2_SUPPORT)
    dac_Config_t        config;
    dac_FunctionState_t chState = DAC_FUNCTION_ACTIVE;
    dac_FlagState_t     flagState = DAC_FLAG_INACTIVE;

    Ut_Dac_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL_UINT32( 1u, (uint32_t)DAC_CHANNEL_CNT );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ChannelActive( UT_DAC_PERIPH, DAC_CHANNEL_CNT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_ChannelState( UT_DAC_PERIPH, DAC_CHANNEL_CNT, &chState ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_Data( UT_DAC_PERIPH, DAC_CHANNEL_CNT, 0u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DualData( UT_DAC_PERIPH, 0u, 0u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DualSwTrigger( UT_DAC_PERIPH ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_CNT, DAC_TRIGGER_TIM6_TRGO ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_CNT, DAC_FLAG_CALIBRATION, &flagState ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );
#else
    TEST_IGNORE_MESSAGE( "MCU without single channel DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Set_ClockSource() / Dac_Get_ClockSource() - APB1 clock only.
 *
 * \par Expected results
 * - PCLK1 accepted without RCC access, invalid source refused, getter returns PCLK1.
 * - Clock source change refused while a channel is enabled.
 * - NULL pointers rejected.
 */
void Ut_Dac_ClockSource_Pclk1ClockOnly( void )
{
#if defined(DAC1)
    dac_ClkSrc_t   clkSrc   = DAC_CLK_SRC_CNT;
    dac_ShClkSrc_t shClkSrc = DAC_SH_CLK_SRC_CNT;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_ClockSource( DAC_CLK_SRC_PCLK1 ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ClockSource( DAC_CLK_SRC_CNT ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( DAC_CLK_SRC_PCLK1, clkSrc );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ShClockSource( &shClkSrc ) );
    TEST_ASSERT_EQUAL( DAC_SH_CLK_SRC_LSI, shClkSrc );

    UT_DAC_REG->CR = DAC_CR_EN1;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ClockSource( DAC_CLK_SRC_PCLK1 ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_ClockSource( NULL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_ShClockSource( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Init() is refused while a channel is enabled.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR, no clock activated, CR not modified.
 */
void Ut_Dac_Init_ChannelEnabled_ReturnsError( void )
{
#if defined(DAC1)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Ignore_PeriphMocks();
    UT_DAC_REG->CR = DAC_CR_EN1;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_RccActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN1, UT_DAC_REG->CR );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Init() rejects invalid channel configurations before register writes.
 *
 * \details Initial value out of format range, invalid output mode, channel slot
 *          mismatch, sample time out of range, peripheral slot mismatch, invalid trigger,
 *          STM32L4 (TSELx 3 bits): trigger not connected to the DAC (TIM1 TRGO).
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR in all cases, channel registers (CR, MCR, DHR12R1) stay 0.
 */
void Ut_Dac_Init_InvalidChannelConfig_ReturnsErrorWithoutRegisterWrite( void )
{
#if defined(DAC1)
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
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelId = DAC_CHANNEL_CNT;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].SampleHold.SampleTime = DAC_SH_SAMPLE_TIME_MAX + 1u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].PeriphId = DAC_PERIPH_CNT;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].TriggerId = DAC_TRIGGER_CNT;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->MCR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->DHR12R1 );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Init() with sample and hold channel selects the sample and hold clock and timing.
 *
 * \details Channel 1 with sample and hold (sample 100, hold 200, refresh 30), clock LSI
 *          (LSI not running).
 *
 * \par Expected results
 * - LSI started by RCC once, APB1 clock of DAC1 activated (no sample and hold clock multiplexer).
 * - SHSR1 / SHHR / SHRR contain the timing, MODE1 sample and hold bit set.
 * - Dac_Get_SampleHoldConfig() returns the configuration.
 */
void Ut_Dac_Init_SampleHold_ShClockAndTimingConfigured( void )
{
#if defined(DAC1)
    dac_Config_t           config;
    dac_SampleHoldConfig_t shConfig = { 0 };

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].ShClockSource = DAC_SH_CLK_SRC_LSI;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].SampleHold.State       = DAC_FUNCTION_ACTIVE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].SampleHold.SampleTime  = 100u;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].SampleHold.HoldTime    = 200u;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].SampleHold.RefreshTime = 30u;
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_LsiStartCnt );
    TEST_ASSERT_EQUAL( RCC_PERIPH_DAC1, utDac_RccLastActive );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_RccActiveCnt );

    TEST_ASSERT_EQUAL_HEX32( 100u, UT_DAC_REG->SHSR1 );
    TEST_ASSERT_EQUAL_HEX32( 200u, UT_DAC_REG->SHHR & DAC_SHHR_THOLD1 );
    TEST_ASSERT_EQUAL_HEX32( 30u,  UT_DAC_REG->SHRR & DAC_SHRR_TREFRESH1 );
    TEST_ASSERT_EQUAL_HEX32( DAC_MCR_MODE1_2, UT_DAC_REG->MCR & UT_DAC_MCR_MODE1 );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &shConfig ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE, shConfig.State );
    TEST_ASSERT_EQUAL_UINT16( 100u, shConfig.SampleTime );
    TEST_ASSERT_EQUAL_UINT16( 200u, shConfig.HoldTime );
    TEST_ASSERT_EQUAL_UINT8( 30u, shConfig.RefreshTime );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Init() with internal output of channel 2 does not configure the pin.
 *
 * \details Channel 2 used with DAC_OUTPUT_INTERNAL, channel 1 not used. MCUs without
 *          channel 2: test ignored.
 *
 * \par Expected results
 * - No Gpio_Init call, MODE2 = 011 (buffer disabled, internal connection), EN2 set, OTRIM2 = 0.
 */
void Ut_Dac_Init_InternalOutputChannel2_NoPinConfigured( void )
{
#if defined(DAC_CHANNEL2_SUPPORT)
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
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CCR & DAC_CCR_OTRIM2 );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_2, &outputMode ) );
    TEST_ASSERT_EQUAL( DAC_OUTPUT_INTERNAL, outputMode );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC channel 2" );
#endif /* DAC_CHANNEL2_SUPPORT */
}


/**
 * \brief   Dac_Deinit() of direct channels disables channels and the APB1 clock of DAC1.
 *
 * \details Channel 1 and (MCUs with channel 2) channel 2 used in direct mode.
 *
 * \par Expected results
 * - DAC_REQUEST_OK, CR = 0, Rcc_Set_PeriphInactive( RCC_PERIPH_DAC1 ) called.
 * - The shared NVIC line is not disabled (not enabled by the module).
 * - Invalid peripheral is rejected.
 */
void Ut_Dac_Deinit_DirectChannels_ChannelsAndClockDisabled( void )
{
#if defined(DAC1)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
#if defined(DAC_CHANNEL2_SUPPORT)
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
#endif /* DAC_CHANNEL2_SUPPORT */
    Ut_Dac_Init( &config );
    Ut_Dac_Reset_Mocks();

    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_DAC1, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Deinit( UT_DAC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Deinit( DAC_PERIPH_CNT ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Items of the DMA channel list carry DAC peripheral, DAC channel, DMA peripheral, DMA channel
 *          and request selection of the channel.
 *
 * \details Expected values are the rows of the table utDac_DmaRows (STM32CubeMX DMA database -
 *          channels of the DMA_CSELR request mapping, every channel of DMA1 / DMA2 on devices with
 *          DMAMUX1) written as (DAC peripheral, DAC channel index, DMA peripheral index, DMA channel
 *          index, request selection), independently of the encoding macro.
 *
 * \par Expected results
 * - Every row of the table: the item carries the expected bit-fields and the decoding macros
 *   return the fields.
 * - Unused item equals DAC_DMA_CODE_UNUSED.
 * - Ignored on MCUs without DAC.
 */
void Ut_Dac_DmaList_Items_EncodePeriphChannelDmaAndSelection( void )
{
#if defined(DAC1)
    TEST_ASSERT_TRUE( 0u < UT_DAC_DMA_ROW_CNT );

    for( uint32_t rowIdx = 0u; UT_DAC_DMA_ROW_CNT > rowIdx; rowIdx++ )
    {
        const utDac_DmaRow_t * const row = &utDac_DmaRows[ rowIdx ];

        TEST_ASSERT_EQUAL_HEX32( UT_DAC_DMA_CODE( (uint32_t)UT_DAC_PERIPH, (uint32_t)row->PeriphId, row->DmaId, row->Channel, row->Selection ), row->Item );
        TEST_ASSERT_EQUAL_UINT32( UT_DAC_PERIPH,  DAC_DMA_BIT_MASK_DECODE_PERIPH( row->Item ) );
        TEST_ASSERT_EQUAL_UINT32( row->PeriphId,  DAC_DMA_BIT_MASK_DECODE_CHANNEL( row->Item ) );
        TEST_ASSERT_EQUAL_UINT32( row->DmaId,     DAC_DMA_BIT_MASK_DECODE_DMA( row->Item ) );
        TEST_ASSERT_EQUAL_UINT32( row->Channel,   DAC_DMA_BIT_MASK_DECODE_DMA_CHANNEL( row->Item ) );
        TEST_ASSERT_EQUAL_UINT32( row->Selection, DAC_DMA_BIT_MASK_DECODE_CSELR( row->Item ) );
    }

    /* Unused channel */
    TEST_ASSERT_EQUAL_HEX32( DAC_DMA_CODE_UNUSED, DAC_DMA_UNUSED );
    TEST_ASSERT_EQUAL_UINT32( DAC_PERIPH_CNT,      DAC_DMA_BIT_MASK_DECODE_PERIPH( DAC_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( DAC_CHANNEL_CNT,     DAC_DMA_BIT_MASK_DECODE_CHANNEL( DAC_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( DAC_DMA_PERIPH_CNT,  DAC_DMA_BIT_MASK_DECODE_DMA( DAC_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( DAC_DMA_CHANNEL_CNT, DAC_DMA_BIT_MASK_DECODE_DMA_CHANNEL( DAC_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( 0u,                  DAC_DMA_BIT_MASK_DECODE_CSELR( DAC_DMA_UNUSED ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Every item of the DMA channel list is accepted for its DAC channel only.
 *
 * \details Every row of the table utDac_DmaRows is checked by Dac_Dma_Check_Config() as the DMA
 *          channel of every DAC channel of the MCU. The decoding of the item by the initialization
 *          is checked by the DMA initialization tests of both channels.
 *
 * \par Expected results
 * - DAC_REQUEST_OK for the DAC channel of the item, DAC_REQUEST_ERROR for the other DAC channel.
 * - Ignored on MCUs without DAC.
 */
void Ut_Dac_DmaList_Items_AcceptedForOwnChannelOnly( void )
{
#if defined(DAC1)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );

    for( uint32_t rowIdx = 0u; UT_DAC_DMA_ROW_CNT > rowIdx; rowIdx++ )
    {
        const utDac_DmaRow_t * const row        = &utDac_DmaRows[ rowIdx ];
        dac_DataConfig_t             dataConfig = config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataConfig;

        dataConfig.Dma = (dac_Dma_t)row->Item;

        for( dac_ChannelId_t channelId = DAC_CHANNEL_1; DAC_CHANNEL_CNT > channelId; channelId++ )
        {
            if( row->PeriphId == channelId )
            {
                TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Dma_Check_Config( UT_DAC_PERIPH, channelId, &dataConfig ) );
            }
            else
            {
                TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Dma_Check_Config( UT_DAC_PERIPH, channelId, &dataConfig ) );
            }
        }
    }
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/* ============================ CLOCK SOURCES =============================== */

/**
 * \brief   Dac_Set_ShClockSource() - LSI only, started by RCC, refused while a channel is enabled.
 *
 * \par Expected results
 * - LSI not running: started once. LSI running: not started again.
 * - LSE and invalid source: DAC_REQUEST_ERROR without RCC access.
 * - Channel enabled: DAC_REQUEST_ERROR.
 */
void Ut_Dac_Set_ShClockSource_LsiOnly( void )
{
#if defined(DAC1)
    Ut_Dac_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_ShClockSource( DAC_SH_CLK_SRC_LSI ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_LsiStartCnt );

    utDac_LsiState = RCC_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_ShClockSource( DAC_SH_CLK_SRC_LSI ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_LsiStartCnt );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ShClockSource( DAC_SH_CLK_SRC_LSE ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ShClockSource( DAC_SH_CLK_SRC_CNT ) );

    UT_DAC_REG->CR = DAC_CR_EN1;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ShClockSource( DAC_SH_CLK_SRC_LSI ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_LsiStartCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/* ============================== CALIBRATION =============================== */

/**
 * \brief   Dac_Set_Calibration() finds the lowest trimming value setting CAL_FLAG.
 *
 * \details Calibration model sets CAL_FLAG1 for OTRIM1 >= threshold. Channel 1
 *          has unbuffered pin output configured before the calibration.
 *
 * \par Expected results
 * - Thresholds 1, 11, 12, 31 are found exactly.
 * - CEN1 is cleared and the unbuffered output mode is restored.
 */
void Ut_Dac_Set_Calibration_ModelThreshold_TrimValueFound( void )
{
#if defined(DAC1)
    const uint32_t  thresholds[ ] = { 1u, 11u, 12u, 31u };
    dac_TrimValue_t trimValue     = 0u;

    Ut_Dac_Ignore_PeriphMocks();
    Ut_Dac_Set_ModelActive();
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_OUTPUT_PIN_UNBUFFERED ) );

    for( uint32_t idx = 0u; ( sizeof( thresholds ) / sizeof( uint32_t ) ) > idx; idx++ )
    {
        utDac_CalThreshold = thresholds[ idx ];

        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Calibration( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_1, &trimValue ) );
        TEST_ASSERT_EQUAL_UINT8( thresholds[ idx ], trimValue );
    }

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & DAC_CR_CEN1 );
    TEST_ASSERT_EQUAL_HEX32( DAC_MCR_MODE1_1, UT_DAC_REG->MCR & UT_DAC_MCR_MODE1 );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Set_Calibration() / Dac_Set_TrimValue() are refused for enabled channel.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR, CCR and CR not modified.
 */
void Ut_Dac_Set_Calibration_ChannelEnabled_ReturnsError( void )
{
#if defined(DAC1)
    UT_DAC_REG->CR = DAC_CR_EN1;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_Calibration( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_1, 5u ) );

    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN1, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CCR );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Set_TrimValue() writes OTRIMx, values above maximum are rejected.
 *
 * \par Expected results
 * - OTRIM1 = 7 (OTRIM2 = 31 on MCUs with channel 2), value 32 returns DAC_REQUEST_ERROR.
 */
void Ut_Dac_Set_TrimValue_ValidValues_WrittenToCcr( void )
{
#if defined(DAC1)
    dac_TrimValue_t trimValue = 0u;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_1, 7u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_TRIM_VALUE_MAX + 1u ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_1, &trimValue ) );
    TEST_ASSERT_EQUAL_UINT8( 7u, trimValue );

#if defined(DAC_CHANNEL2_SUPPORT)
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_TRIM_VALUE_MAX ) );
    TEST_ASSERT_EQUAL_HEX32( 7u | ( DAC_TRIM_VALUE_MAX << DAC_CCR_OTRIM2_Pos ), UT_DAC_REG->CCR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TrimValue( UT_DAC_PERIPH, DAC_CHANNEL_2, &trimValue ) );
    TEST_ASSERT_EQUAL_UINT8( DAC_TRIM_VALUE_MAX, trimValue );
#else
    TEST_ASSERT_EQUAL_HEX32( 7u, UT_DAC_REG->CCR );
#endif /* DAC_CHANNEL2_SUPPORT */
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
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
#if defined(DAC1)
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
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Channel configuration setters are refused for enabled channel.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR for output mode, trigger, wave mode, amplitude and sample and
 *   hold, MCR, CR and SHSR1 are not modified.
 */
void Ut_Dac_Set_ChannelConfig_ChannelEnabled_ReturnsErrorWithoutWrite( void )
{
#if defined(DAC1)
    const dac_SampleHoldConfig_t shConfig = { .State = DAC_FUNCTION_ACTIVE, .SampleTime = 1u, .HoldTime = 1u, .RefreshTime = 1u };

    UT_DAC_REG->CR = DAC_CR_EN1;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_OutputMode( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_OUTPUT_INTERNAL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_TRIGGER_TIM6_TRGO ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_WaveMode( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_WAVE_NOISE ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_WaveAmplitude( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_WAVE_AMPLITUDE_255 ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SampleHoldConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &shConfig ) );

    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN1, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->MCR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->SHSR1 );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Set_TriggerSrc() writes every trigger and reads it back.
 *
 * \par Expected results
 * - Every trigger of the list of the device line is accepted (the triggers of timers missing on
 *   the MCU and the triggers of STM32L4+ on STM32L4 are not in the list).
 * - DAC_TRIGGER_NONE: TEN1 = 0. Other triggers: TEN1 = 1, TSEL1 = LL value.
 * - Dac_Get_TriggerSrc() returns the trigger for every value of the list.
 * - TIM6 TRGO is written to TSEL1 with the LL encoding of the device.
 */
void Ut_Dac_Set_TriggerSrc_AllTriggers_ReadBack( void )
{
#if defined(DAC1)
    dac_TriggerId_t triggerSrc = DAC_TRIGGER_CNT;

    for( dac_TriggerId_t trigIdx = DAC_TRIGGER_NONE; DAC_TRIGGER_CNT > trigIdx; trigIdx++ )
    {
        TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, trigIdx ) );

        if( DAC_TRIGGER_NONE == trigIdx )
        {
            TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & DAC_CR_TEN1 );
            TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, &triggerSrc ) );
            TEST_ASSERT_EQUAL( trigIdx, triggerSrc );
        }
        else
        {
            TEST_ASSERT_EQUAL_HEX32( DAC_CR_TEN1, UT_DAC_REG->CR & DAC_CR_TEN1 );
            TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, &triggerSrc ) );
            TEST_ASSERT_EQUAL( trigIdx, triggerSrc );
        }
    }

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_TRIGGER_TIM6_TRGO ) );
    TEST_ASSERT_EQUAL_HEX32( LL_DAC_TRIG_EXT_TIM6_TRGO | DAC_CR_TEN1, UT_DAC_REG->CR & ( DAC_CR_TSEL1 | DAC_CR_TEN1 ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_TRIGGER_CNT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   The list of the conversion triggers contains the triggers of the device line only.
 *
 * \details Counts the values of \ref dac_TriggerId_t of the build: no trigger, software trigger,
 *          TIM2 TRGO, TIM6 TRGO and EXTI line 9 on every line; TIM4 / TIM5 / TIM7 / TIM8 TRGO
 *          only where the timer exists (CMSIS instance macro); TIM1 / TIM15 TRGO and LPTIM1 /
 *          LPTIM2 output only on STM32L4+ (TSELx 4 bits).
 *
 * \par Expected results
 * - DAC_TRIGGER_CNT is the count of the triggers of the device line.
 */
void Ut_Dac_TriggerList_MatchesDeviceLine( void )
{
    uint32_t expectedTriggers = 5u;

#if defined(TIM4)
    expectedTriggers++;
#endif /* TIM4 */
#if defined(TIM5)
    expectedTriggers++;
#endif /* TIM5 */
#if defined(TIM7)
    expectedTriggers++;
#endif /* TIM7 */
#if defined(TIM8)
    expectedTriggers++;
#endif /* TIM8 */
#if defined(DAC_CR_TSEL1_3)
    expectedTriggers += 4u;
#endif /* DAC_CR_TSEL1_3 */

    TEST_ASSERT_EQUAL_UINT32( expectedTriggers, (uint32_t)DAC_TRIGGER_CNT );
}


/**
 * \brief   Trigger of channel 2 is independent of channel 1.
 *
 * \details Channel 1 with EXTI line 9, channel 2 with TIM6 TRGO. MCUs without channel 2:
 *          test ignored.
 *
 * \par Expected results
 * - TSEL2 / TEN2 hold TIM6 TRGO (LL value shifted to channel 2), channel 1 keeps EXTI line 9.
 */
void Ut_Dac_Set_TriggerSrc_Channel2_IndependentOfChannel1( void )
{
#if defined(DAC_CHANNEL2_SUPPORT)
    dac_TriggerId_t triggerSrc = DAC_TRIGGER_CNT;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_TRIGGER_EXTI_LINE9 ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_TRIGGER_TIM6_TRGO ) );

    TEST_ASSERT_EQUAL_HEX32( ( LL_DAC_TRIG_EXT_TIM6_TRGO | DAC_CR_TEN1 ) << DAC_CR_EN2_Pos, UT_DAC_REG->CR & ( DAC_CR_TSEL2 | DAC_CR_TEN2 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_DAC_TRIG_EXT_EXTI_LINE9 | DAC_CR_TEN1, UT_DAC_REG->CR & ( DAC_CR_TSEL1 | DAC_CR_TEN1 ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, &triggerSrc ) );
    TEST_ASSERT_EQUAL( DAC_TRIGGER_TIM6_TRGO, triggerSrc );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, &triggerSrc ) );
    TEST_ASSERT_EQUAL( DAC_TRIGGER_EXTI_LINE9, triggerSrc );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC channel 2" );
#endif /* DAC_CHANNEL2_SUPPORT */
}


/**
 * \brief   Wave mode and amplitude setters write WAVEx / MAMPx and read them back.
 *
 * \par Expected results
 * - Triangle with amplitude 1023 on channel 1, read back.
 * - Noise with amplitude 4095 on channel 1 (channel 2 on MCUs with channel 2), read back.
 */
void Ut_Dac_Set_WaveModeAmplitude_ReadBack( void )
{
#if defined(DAC1)
    dac_WaveMode_t      waveMode  = DAC_WAVE_CNT;
    dac_WaveAmplitude_t amplitude = DAC_WAVE_AMPLITUDE_CNT;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_WaveMode( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_WAVE_TRIANGLE ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_WaveAmplitude( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_WAVE_AMPLITUDE_1023 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_DAC_WAVE_AUTO_GENERATION_TRIANGLE | LL_DAC_TRIANGLE_AMPLITUDE_1023, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_WaveAmplitude( UT_DAC_PERIPH, DAC_CHANNEL_1, &amplitude ) );
    TEST_ASSERT_EQUAL( DAC_WAVE_AMPLITUDE_1023, amplitude );

#if defined(DAC_CHANNEL2_SUPPORT)
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_WaveMode( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_WAVE_NOISE ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_WaveAmplitude( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_WAVE_AMPLITUDE_4095 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_DAC_WAVE_AUTO_GENERATION_TRIANGLE | LL_DAC_TRIANGLE_AMPLITUDE_1023 |
                             ( ( LL_DAC_WAVE_AUTO_GENERATION_NOISE | LL_DAC_NOISE_LFSR_UNMASK_BITS11_0 ) << DAC_CR_EN2_Pos ),
                             UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_WaveMode( UT_DAC_PERIPH, DAC_CHANNEL_2, &waveMode ) );
    TEST_ASSERT_EQUAL( DAC_WAVE_NOISE, waveMode );
#else
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_WaveMode( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_WAVE_NOISE ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_WaveAmplitude( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_WAVE_AMPLITUDE_4095 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_DAC_WAVE_AUTO_GENERATION_NOISE | LL_DAC_NOISE_LFSR_UNMASK_BITS11_0, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_WaveMode( UT_DAC_PERIPH, DAC_CHANNEL_1, &waveMode ) );
    TEST_ASSERT_EQUAL( DAC_WAVE_NOISE, waveMode );
#endif /* DAC_CHANNEL2_SUPPORT */

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_WaveMode( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_WAVE_CNT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_WaveAmplitude( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_WAVE_AMPLITUDE_CNT ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
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
#if defined(DAC1)
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
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/* =============================== DATA OUTPUT ============================== */

/**
 * \brief   Dac_Set_Data() writes the data holding register of the channel format.
 *
 * \par Expected results
 * - 12-bit right -> DHR12R1, 12-bit left -> DHR12L1, 8-bit right -> DHR8R1.
 * - Values out of format range are rejected.
 */
void Ut_Dac_Set_Data_DataFormats_HoldingRegisterWritten( void )
{
#if defined(DAC1)
    dac_DataFormat_t dataFormat = DAC_DATA_FORMAT_CNT;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( UT_DAC_PERIPH, DAC_CHANNEL_1, 0x0FFFu ) );
    TEST_ASSERT_EQUAL_HEX32( 0x0FFFu, UT_DAC_REG->DHR12R1 );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_Data( UT_DAC_PERIPH, DAC_CHANNEL_1, 0x1000u ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_12BIT_LEFT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( UT_DAC_PERIPH, DAC_CHANNEL_1, 0xABC0u ) );
    TEST_ASSERT_EQUAL_HEX32( 0xABC0u, UT_DAC_REG->DHR12L1 );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_8BIT_RIGHT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( UT_DAC_PERIPH, DAC_CHANNEL_1, 0x5Au ) );
    TEST_ASSERT_EQUAL_HEX32( 0x5Au, UT_DAC_REG->DHR8R1 );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_Data( UT_DAC_PERIPH, DAC_CHANNEL_1, 0x100u ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataFormat ) );
    TEST_ASSERT_EQUAL( DAC_DATA_FORMAT_8BIT_RIGHT, dataFormat );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_CNT ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->DHR12R2 );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Set_DualData() writes the dual data holding register.
 *
 * \details MCUs without channel 2: test ignored.
 *
 * \par Expected results
 * - 12-bit right: DHR12RD = ch1 | ch2 << 16. 8-bit right: DHR8RD = ch1 | ch2 << 8.
 * - Different formats of the channels are rejected.
 */
void Ut_Dac_Set_DualData_SameFormat_DualRegisterWritten( void )
{
#if defined(DAC_CHANNEL2_SUPPORT)
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DualData( UT_DAC_PERIPH, 0x0123u, 0x0456u ) );
    TEST_ASSERT_EQUAL_HEX32( 0x04560123u, UT_DAC_REG->DHR12RD );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_12BIT_LEFT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DualData( UT_DAC_PERIPH, 0x0010u, 0x0020u ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_8BIT_RIGHT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_DATA_FORMAT_8BIT_RIGHT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DualData( UT_DAC_PERIPH, 0x12u, 0x34u ) );
    TEST_ASSERT_EQUAL_HEX32( 0x3412u, UT_DAC_REG->DHR8RD );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DualData( UT_DAC_PERIPH, 0x12u, 0x134u ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC channel 2" );
#endif /* DAC_CHANNEL2_SUPPORT */
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
#if defined(DAC1)
    dac_Data_t       outData    = 0u;
    dac_Voltage_mV_t outVoltage = 0u;

    UT_DAC_REG->DOR1 = 0x0FFFu;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputData( UT_DAC_PERIPH, DAC_CHANNEL_1, &outData ) );
    TEST_ASSERT_EQUAL_HEX16( 0x0FFFu, outData );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputVoltage( UT_DAC_PERIPH, DAC_CHANNEL_1, 3300u, &outVoltage ) );
    TEST_ASSERT_EQUAL_UINT16( 3300u, outVoltage );

    UT_DAC_REG->DOR1 = 2048u;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_OutputVoltage( UT_DAC_PERIPH, DAC_CHANNEL_1, 3300u, &outVoltage ) );
    TEST_ASSERT_EQUAL_UINT16( 1650u, outVoltage );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_OutputVoltage( UT_DAC_PERIPH, DAC_CHANNEL_1, 0u, &outVoltage ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_OutputVoltage( UT_DAC_PERIPH, DAC_CHANNEL_1, 3300u, NULL ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_OutputData( UT_DAC_PERIPH, DAC_CHANNEL_CNT, &outData ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Get_DataFromVoltage() converts voltage according to the data format.
 *
 * \par Expected results
 * - 1650 mV of 3300 mV: 12-bit right 2048, 12-bit left 0x8000, 8-bit right 128.
 * - 3300 mV -> 4095. Voltage above reference and reference 0 rejected.
 * - Dac_Set_Voltage() writes the converted value (12-bit right).
 */
void Ut_Dac_Get_DataFromVoltage_DataFormats_Converted( void )
{
#if defined(DAC1)
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

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataFormat( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DATA_FORMAT_12BIT_RIGHT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Voltage( UT_DAC_PERIPH, DAC_CHANNEL_1, 3300u, 1000u ) );
    TEST_ASSERT_EQUAL_HEX32( 1241u, UT_DAC_REG->DHR12R1 );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Software trigger is generated only for enabled channel with software trigger.
 *
 * \par Expected results
 * - Trigger disabled or channel disabled: DAC_REQUEST_ERROR, SWTRIGR = 0.
 * - Single trigger: SWTRIG1 set.
 * - MCUs with channel 2: dual trigger refused until both channels are enabled, then SWTRIG1
 *   and SWTRIG2 set. Single channel DAC: dual trigger refused.
 */
void Ut_Dac_Set_SwTrigger_SoftwareTriggerSelected_SwtrigSet( void )
{
#if defined(DAC1)
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SwTrigger( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_TRIGGER_SOFTWARE ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_SwTrigger( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->SWTRIGR );

    UT_DAC_REG->CR |= DAC_CR_EN1;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DualSwTrigger( UT_DAC_PERIPH ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_SwTrigger( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_SWTRIGR_SWTRIG1, UT_DAC_REG->SWTRIGR );

#if defined(DAC_CHANNEL2_SUPPORT)
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_TriggerSrc( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_TRIGGER_SOFTWARE ) );
    UT_DAC_REG->CR |= DAC_CR_EN2;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DualSwTrigger( UT_DAC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_SWTRIGR_SWTRIG1 | DAC_SWTRIGR_SWTRIG2, UT_DAC_REG->SWTRIGR );
#endif /* DAC_CHANNEL2_SUPPORT */
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Get_Flag() reads status flags, Dac_Clear_Flag() clears only DMA underrun.
 *
 * \par Expected results
 * - Calibration / busy / underrun flags of channel 1 are reported from SR.
 * - Ready flag is refused (STM32L4 has no DACxRDY).
 * - Clear of underrun writes DMAUDR1, clear of other flags is rejected.
 */
void Ut_Dac_Get_Flag_StatusRegister_FlagsReported( void )
{
#if defined(DAC1)
    dac_FlagState_t flagState = DAC_FLAG_INACTIVE;

    UT_DAC_REG->SR = DAC_SR_CAL_FLAG1;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_CALIBRATION, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_FLAG_ACTIVE, flagState );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_SH_WRITE_BUSY, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_FLAG_INACTIVE, flagState );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_DMA_UNDERRUN, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_FLAG_INACTIVE, flagState );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_READY, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_CNT, &flagState ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_CALIBRATION, NULL ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Clear_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_CALIBRATION ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_SR_CAL_FLAG1, UT_DAC_REG->SR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Clear_Flag( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_FLAG_DMA_UNDERRUN ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_SR_DMAUDR1, UT_DAC_REG->SR );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Set_ChannelActive() enables the channel without ready flag.
 *
 * \details No HW model - STM32L4 has no channel ready flag.
 *
 * \par Expected results
 * - DAC_REQUEST_OK, EN1 set, state active.
 * - Dac_Set_ChannelInactive(): EN1 cleared, state inactive. Invalid channel refused.
 */
void Ut_Dac_Set_ChannelActive_NoReadyFlag_Enabled( void )
{
#if defined(DAC1)
    dac_FunctionState_t chState = DAC_FUNCTION_INACTIVE;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_ChannelActive( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_HEX32( DAC_CR_EN1, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ChannelState( UT_DAC_PERIPH, DAC_CHANNEL_1, &chState ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_ACTIVE, chState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_ChannelInactive( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_ChannelState( UT_DAC_PERIPH, DAC_CHANNEL_1, &chState ) );
    TEST_ASSERT_EQUAL( DAC_FUNCTION_INACTIVE, chState );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_ChannelActive( UT_DAC_PERIPH, DAC_CHANNEL_CNT ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Get_ChannelState( UT_DAC_PERIPH, DAC_CHANNEL_1, NULL ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
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
#if defined(DAC1)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].TriggerId = DAC_TRIGGER_SOFTWARE;

    Ut_Dac_Ignore_PeriphMocks();
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataConfig ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_DmaInitCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   DMA transfer configuration is checked.
 *
 * \details TIM6 trigger with NULL buffer, size 0, invalid DMA peripheral, channel, priority
 *          and buffer mode.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR in all cases, DMA is not initialized.
 */
void Ut_Dac_Set_DataConfig_InvalidDmaConfig_ReturnsError( void )
{
#if defined(DAC1)
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

    dataCfg     = channelCfg.DataConfig;
    dataCfg.Dma = (dac_Dma_t)DAC_DMA_ENCODE( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DMA_PERIPH_1, DAC_DMA_CHANNEL_CNT, 6u );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataCfg ) );

    dataCfg     = channelCfg.DataConfig;
    dataCfg.Dma = (dac_Dma_t)DAC_DMA_ENCODE( UT_DAC_PERIPH, DAC_CHANNEL_1, DAC_DMA_PERIPH_CNT, DAC_DMA_CHANNEL_3, 6u );
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataCfg ) );

    dataCfg     = channelCfg.DataConfig;
    dataCfg.Dma = DAC_DMA_UNUSED;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataCfg ) );

#if defined(DAC_CHANNEL2_SUPPORT)
    dataCfg     = channelCfg.DataConfig;
    dataCfg.Dma = DAC_DMA_DAC1_CH2_DMA1_CHANNEL4;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataCfg ) );
#endif /* DAC_CHANNEL2_SUPPORT */

    dataCfg             = channelCfg.DataConfig;
    dataCfg.DmaPriority = (dac_DmaPriority_t)DMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataCfg ) );

    dataCfg            = channelCfg.DataConfig;
    dataCfg.BufferMode = DAC_BUFFER_MODE_CNT;
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataCfg ) );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, NULL ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_DmaInitCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Init() in DMA mode initializes the DMA channel and the DAC interrupt.
 *
 * \details Channel 1, TIM6 trigger, DMA buffer of 8 items, 12-bit left aligned format,
 *          half transfer callback set.
 *
 * \par Expected results
 * - Dma_Init() called once: DMA1 channel 3 (default of DAC1 channel 1), request DAC1_CH1,
 *   memory to peripheral, normal mode, peripheral = DHR12L1 (static, 32-bit), memory = buffer
 *   (increment, 16-bit), 8 items, high priority, transfer complete / half transfer / error
 *   handlers registered.
 * - Half transfer interrupt of the channel enabled.
 * - DAC interrupt registered on TIM6_DAC line, DMA requests not enabled yet (DMAEN1 = 0).
 */
void Ut_Dac_Init_DmaMode_ChannelInitialized( void )
{
#if defined(DAC1)
    dac_Config_t     config;
    dac_DataConfig_t dataConfig;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataFormat = DAC_DATA_FORMAT_12BIT_LEFT;
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInitCnt );
    TEST_ASSERT_EQUAL( UT_DAC_DMA_PERIPH, utDac_DmaConfig.DmaPeriphId );
    TEST_ASSERT_EQUAL( UT_DAC_DMA_CHANNEL_CH1, utDac_DmaConfig.DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_DAC1_CH1, utDac_DmaConfig.PeripheralReqId );
    TEST_ASSERT_EQUAL( DMA_DIR_MEMORY_TO_PERIPH, utDac_DmaConfig.Direction );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_NORMAL, utDac_DmaConfig.TransferMode );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_DAC_REG->DHR12L1, utDac_DmaConfig.PeriphAddress );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utDac_Buffer, utDac_DmaConfig.MemoryAddress );
    TEST_ASSERT_EQUAL( DMA_PERIPH_ADDR_STATIC, utDac_DmaConfig.PeriphAddrIncrement );
    TEST_ASSERT_EQUAL( DMA_MEMORY_ADDR_INCREMENT, utDac_DmaConfig.MemoryAddrIncrement );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_32BIT, utDac_DmaConfig.PeriphTransferSize );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_16BIT, utDac_DmaConfig.MemoryTransferSize );
    TEST_ASSERT_EQUAL_UINT32( UT_DAC_BUF_SIZE, utDac_DmaConfig.DataCount );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_HIGH, utDac_DmaConfig.Priority );
    TEST_ASSERT_NOT_NULL( utDac_DmaConfig.TransferCompleteCallback );
    TEST_ASSERT_NOT_NULL( utDac_DmaConfig.HalfTransferCallback );
    TEST_ASSERT_NOT_NULL( utDac_DmaConfig.TransferErrorCallback );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaHtIrqState );

    TEST_ASSERT_NOT_NULL( utDac_Isr );
    TEST_ASSERT_EQUAL( UT_DAC_NVIC, utDac_IsrIrqId );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & DAC_CR_DMAEN1 );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_1, &dataConfig ) );
    TEST_ASSERT_EQUAL( DAC_TRANSFER_MODE_DMA, dataConfig.TransferMode );
    TEST_ASSERT_EQUAL_PTR( utDac_Buffer, dataConfig.DataBuffer );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   DMA transfer without half transfer callback does not use the half transfer interrupt.
 *
 * \par Expected results
 * - Dma_Init() without half transfer callback, half transfer interrupt of the channel disabled.
 */
void Ut_Dac_Init_DmaModeWithoutHalfCallback_HalfTransferIrqDisabled( void )
{
#if defined(DAC1)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_CIRCULAR );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataConfig.HalfTransferCallback = DAC_NULL_PTR;
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( UT_DAC_DMA_CHANNEL_CH1, utDac_DmaConfig.DmaChannel );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_CIRCULAR, utDac_DmaConfig.TransferMode );
    TEST_ASSERT_NULL( utDac_DmaConfig.HalfTransferCallback );
    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_DmaHtIrqState );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Dac_Set_DmaStart() arms the DMA channel and enables DAC DMA requests.
 *
 * \par Expected results
 * - DMA1 channel 3: normal mode, peripheral DHR12R1, memory buffer, 8 items, channel enabled.
 * - DMAEN1 and DMAUDRIE1 set, second start refused, direct data write and format change refused.
 * - Dac_Set_DmaStop() clears DMAEN1 / DMAUDRIE1 and disables the DMA channel.
 */
void Ut_Dac_Set_DmaStart_InitializedDma_RequestsEnabled( void )
{
#if defined(DAC1)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );

    TEST_ASSERT_EQUAL( UT_DAC_DMA_PERIPH, utDac_DmaBus );
    TEST_ASSERT_EQUAL( UT_DAC_DMA_CHANNEL_CH1, utDac_DmaChannel );
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
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   DMA transfer uses the DMA channel of the configuration.
 *
 * \details DAC1 channel 1 with DMA2 channel 4 (alternative channel of the STM32L4 request
 *          mapping), transfer started and stopped.
 *
 * \par Expected results
 * - Dma_Init() with DMA2 channel 4 and request DAC1_CH1.
 * - Start arms and stop disables DMA2 channel 4.
 */
void Ut_Dac_Set_DmaStart_ConfiguredDmaChannel_Used( void )
{
#if defined(DAC1)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].DataConfig.Dma = DAC_DMA_DAC1_CH1_DMA2_CHANNEL4;
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( DMA_PERIPH_2, utDac_DmaConfig.DmaPeriphId );
    TEST_ASSERT_EQUAL( DMA_CHANNEL_4, utDac_DmaConfig.DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_DAC1_CH1, utDac_DmaConfig.PeripheralReqId );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL( DMA_PERIPH_2, utDac_DmaBus );
    TEST_ASSERT_EQUAL( DMA_CHANNEL_4, utDac_DmaChannel );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStop( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_EQUAL( DMA_CHANNEL_4, utDac_DmaInactiveChannel );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Both channels of a peripheral can not use the same DMA channel.
 *
 * \details Channel 2 selects the DMA channel of channel 1 (DMA1 channel 3). The item is built by
 *          the encoding macro - the list of channel 2 contains it on STM32L4+ (DMAMUX1) only, the
 *          collision check does not depend on the request mapping.
 *          MCUs without channel 2: test ignored.
 *
 * \par Expected results
 * - Dac_Init() returns DAC_REQUEST_ERROR, DMA is not initialized, CR not modified.
 */
void Ut_Dac_Init_BothChannelsSameDmaChannel_ReturnsError( void )
{
#if defined(DAC_CHANNEL2_SUPPORT)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ], DAC_BUFFER_MODE_ONE_SHOT );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].DataConfig.Dma = (dac_Dma_t)DAC_DMA_ENCODE( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_DMA_PERIPH_1, DAC_DMA_CHANNEL_3, 0u );

    Ut_Dac_Ignore_PeriphMocks();
    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, utDac_DmaInitCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC channel 2" );
#endif /* DAC_CHANNEL2_SUPPORT */
}


/**
 * \brief   DMA channel owned by an initialized DAC channel is rejected for the other channel.
 *
 * \details Channel 1 initialized in DMA mode with DMA1 channel 3, then channel 2
 *          (Dac_ChannelInit) with the same DMA channel, finally channel 2 with DMA1 channel 4.
 *          MCUs without channel 2: test ignored.
 * \note    STM32G4 Bug AB#1122: the DMA channel of the running owner was reconfigured.
 *
 * \par Expected results
 * - Same DMA channel: DAC_REQUEST_ERROR, Dma_Init() not called again, channel not enabled.
 * - Other DMA channel: DAC_REQUEST_OK, Dma_Init() called.
 */
void Ut_Dac_ChannelInit_DmaChannelOwnedByOtherChannel_ReturnsError( void )
{
#if defined(DAC_CHANNEL2_SUPPORT)
    dac_Config_t        config;
    dac_ChannelConfig_t channel2Cfg;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInitCnt );

    channel2Cfg             = config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ];
    channel2Cfg.ChannelUsed = DAC_FUNCTION_ACTIVE;
    Ut_Dac_Set_DmaConfig( &channel2Cfg, DAC_BUFFER_MODE_ONE_SHOT );
    channel2Cfg.DataConfig.Dma = (dac_Dma_t)DAC_DMA_ENCODE( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_DMA_PERIPH_1, DAC_DMA_CHANNEL_3, 0u );

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_ChannelInit( UT_DAC_PERIPH, &channel2Cfg ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInitCnt );
    TEST_ASSERT_BITS_LOW( DAC_CR_EN2, UT_DAC_REG->CR );

    channel2Cfg.DataConfig.Dma = DAC_DMA_DAC1_CH2_DMA1_CHANNEL4;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_ChannelInit( UT_DAC_PERIPH, &channel2Cfg ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utDac_DmaInitCnt );
    TEST_ASSERT_EQUAL( DMA_CHANNEL_4, utDac_DmaConfig.DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_DAC1_CH2, utDac_DmaConfig.PeripheralReqId );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC channel 2" );
#endif /* DAC_CHANNEL2_SUPPORT */
}


/**
 * \brief   DMA channel is released by the deinitialization / reconfiguration of the owner.
 *
 * \details Channel 1 initialized in DMA mode with DMA1 channel 3 and deinitialized, then
 *          channel 2 is initialized with the same DMA channel. Channel 1 with DMA1 channel 3
 *          (Dac_PeriphInit) before and after channel 2 is reconfigured to DMA1 channel 4.
 *          MCUs without channel 2: test ignored.
 * \note    STM32G4 Bug AB#1122: ownership of the DMA channel is released by the
 *          deinitialization of the owner.
 *
 * \par Expected results
 * - Channel 2 with the released DMA channel: DAC_REQUEST_OK, Dma_Init() called.
 * - Channel 1 with DMA1 channel 3 owned by channel 2: DAC_REQUEST_ERROR, after the
 *   reconfiguration of channel 2 (previous DMA channel released): DAC_REQUEST_OK.
 */
void Ut_Dac_Deinit_DmaChannelReleased_OtherChannelAccepted( void )
{
#if defined(DAC_CHANNEL2_SUPPORT)
    dac_Config_t     config;
    dac_DataConfig_t dataConfig;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Deinit( UT_DAC_PERIPH ) );

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_INACTIVE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ], DAC_BUFFER_MODE_ONE_SHOT );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].DataConfig.Dma = (dac_Dma_t)DAC_DMA_ENCODE( UT_DAC_PERIPH, DAC_CHANNEL_2, DAC_DMA_PERIPH_1, DAC_DMA_CHANNEL_3, 0u );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utDac_DmaInitCnt );
    TEST_ASSERT_EQUAL( DMA_CHANNEL_3, utDac_DmaConfig.DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_DAC1_CH2, utDac_DmaConfig.PeripheralReqId );

    /* Channel 1 with DMA1 channel 3 now owned by channel 2 */
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_INACTIVE;

    TEST_ASSERT_EQUAL( DAC_REQUEST_ERROR, Dac_ChannelInit( UT_DAC_PERIPH, &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ] ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utDac_DmaInitCnt );

    UT_DAC_REG->CR &= ~DAC_CR_EN2;
    dataConfig     = config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].DataConfig;
    dataConfig.Dma = DAC_DMA_DAC1_CH2_DMA1_CHANNEL4;
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DataConfig( UT_DAC_PERIPH, DAC_CHANNEL_2, &dataConfig ) );
    TEST_ASSERT_EQUAL( DMA_CHANNEL_4, utDac_DmaConfig.DmaChannel );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_ChannelInit( UT_DAC_PERIPH, &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ] ) );
    TEST_ASSERT_EQUAL( DMA_CHANNEL_3, utDac_DmaConfig.DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_DAC1_CH1, utDac_DmaConfig.PeripheralReqId );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC channel 2" );
#endif /* DAC_CHANNEL2_SUPPORT */
}


/**
 * \brief   Dac_Set_DmaStart() is refused for disabled channel and direct transfer.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR, DMA channel not enabled, DMAEN1 = 0.
 * - Hardware trigger can not be removed while DMA transfer is configured.
 */
void Ut_Dac_Set_DmaStart_NotReady_ReturnsError( void )
{
#if defined(DAC1)
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
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Failed DMA channel start leaves the DAC channel stopped.
 *
 * \details Dma_Set_TransferActive() returns error.
 *
 * \par Expected results
 * - DAC_REQUEST_ERROR, DMAEN1 / DMAUDRIE1 not set, DMA channel stopped, new start possible.
 */
void Ut_Dac_Set_DmaStart_DmaError_TransferStopped( void )
{
#if defined(DAC1)
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
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   One shot transfer complete stops the transfer and calls the callback.
 *
 * \par Expected results
 * - Half transfer handler calls HalfTransferCallback.
 * - Transfer complete: DMAEN1 cleared, DMA channel disabled (stays enabled after a normal
 *   mode transfer), TransferCompleteCallback called once, new start possible.
 */
void Ut_Dac_DmaComplete_OneShot_TransferStopped( void )
{
#if defined(DAC1)
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
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Circular transfer keeps running after transfer complete.
 *
 * \par Expected results
 * - Channel started in circular mode.
 * - Transfer complete: TransferCompleteCallback called, DMA channel neither re-armed nor
 *   disabled by SW, DMAEN1 kept.
 */
void Ut_Dac_DmaComplete_Circular_ChannelKeptRunning( void )
{
#if defined(DAC1)
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
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   DMA channel is reprogrammed on start from the actual configuration.
 *
 * \details One shot transfer started and completed, data format changed to 8-bit right
 *          aligned, transfer started again.
 *
 * \par Expected results
 * - Peripheral address of the second start is DHR8R1, 8 items.
 */
void Ut_Dac_Set_DmaStart_FormatChanged_ChannelReprogrammed( void )
{
#if defined(DAC1)
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
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   DMA transfer error stops the transfer and reports the error.
 *
 * \par Expected results
 * - ErrorCallback called with DAC_ERROR_DMA_TRANSFER, DMAEN1 cleared, DMA channel disabled.
 */
void Ut_Dac_DmaError_TransferError_ReportedAndStopped( void )
{
#if defined(DAC1)
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
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   DAC interrupt reports DMA underrun of the channel.
 *
 * \details DMA transfer on channel 1 running, DMAUDR1 set in SR, DAC ISR called.
 *          Second call without pending flag.
 *
 * \par Expected results
 * - ErrorCallback called once with DAC_ERROR_DMA_UNDERRUN.
 * - DMAEN1 and DMAUDRIE1 cleared, DMA channel disabled.
 */
void Ut_Dac_Isr_DmaUnderrun_ReportedAndStopped( void )
{
#if defined(DAC1)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_CIRCULAR );
    Ut_Dac_Init( &config );
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_1 ) );
    TEST_ASSERT_NOT_NULL( utDac_Isr );

    UT_DAC_REG->SR = DAC_SR_DMAUDR1;
    utDac_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_ErrorCnt );
    TEST_ASSERT_EQUAL( DAC_ERROR_DMA_UNDERRUN, utDac_LastError );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & ( DAC_CR_DMAEN1 | DAC_CR_DMAUDRIE1 ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_DmaInactiveCnt );

    UT_DAC_REG->SR = 0u;
    utDac_Isr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_ErrorCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   DMA handlers and DMA underrun of channel 2 report the events of channel 2.
 *
 * \details Channel 2 in DMA mode (DMA1 channel 4), one shot transfer started, handlers captured
 *          from Dma_Init() are called: half transfer, transfer complete; transfer started again
 *          and transfer error handler called; transfer started again and DMAUDR2 reported by ISR.
 *          Dac_Task() is called at the end. MCUs without channel 2: test ignored.
 *
 * \par Expected results
 * - Dma_Init() with DMA1 channel 4 and request DAC1_CH2.
 * - Half transfer: HalfTransferCallback called once.
 * - Transfer complete: TransferCompleteCallback called once, DMAEN2 cleared, DMA channel disabled.
 * - Transfer error: ErrorCallback with DAC_ERROR_DMA_TRANSFER, DMAEN2 cleared.
 * - Underrun: ErrorCallback with DAC_ERROR_DMA_UNDERRUN, DMAEN2 / DMAUDRIE2 cleared.
 * - Dac_Task() does not change the DAC registers.
 */
void Ut_Dac_DmaChannel2_HandlersReportChannel2( void )
{
#if defined(DAC_CHANNEL2_SUPPORT)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_INACTIVE;
    config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_2 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );
    TEST_ASSERT_EQUAL( UT_DAC_DMA_CHANNEL_CH2, utDac_DmaConfig.DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_DAC1_CH2, utDac_DmaConfig.PeripheralReqId );

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

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_DmaStart( UT_DAC_PERIPH, DAC_CHANNEL_2 ) );
    UT_DAC_REG->SR = DAC_SR_DMAUDR2;
    utDac_Isr();
    TEST_ASSERT_EQUAL_UINT32( 2u, utDac_ErrorCnt );
    TEST_ASSERT_EQUAL( DAC_ERROR_DMA_UNDERRUN, utDac_LastError );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & ( DAC_CR_DMAEN2 | DAC_CR_DMAUDRIE2 ) );

    const uint32_t crValue = UT_DAC_REG->CR;

    Dac_Task();

    TEST_ASSERT_EQUAL_HEX32( crValue, UT_DAC_REG->CR );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC channel 2" );
#endif /* DAC_CHANNEL2_SUPPORT */
}


/**
 * \brief   Dac_Deinit() releases DMA resources and the DAC interrupt line.
 *
 * \details Channel 1 initialized for DMA transfer.
 *
 * \par Expected results
 * - DMA1 channel 3 stopped and its interrupt disabled in NVIC, channel, DMA requests and
 *   DMA underrun interrupt disabled.
 * - TIM6_DAC interrupt line disabled in NVIC, APB1 clock of DAC1 disabled.
 */
void Ut_Dac_Deinit_DmaChannel_DmaAndInterruptReleased( void )
{
#if defined(DAC1)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );
    Ut_Dac_Reset_Mocks();

    Dma_Set_TransferInactive_ExpectAndReturn( UT_DAC_DMA_PERIPH, UT_DAC_DMA_CHANNEL_CH1, DMA_REQUEST_OK );
    Dma_Set_InterruptInactive_ExpectAndReturn( UT_DAC_DMA_PERIPH, UT_DAC_DMA_CHANNEL_CH1, DMA_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_DAC_NVIC, NVIC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_DAC1, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Deinit( UT_DAC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_DAC_REG->CR & ( DAC_CR_EN1 | DAC_CR_DMAEN1 | DAC_CR_DMAUDRIE1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   TIM6_DAC line is disabled once by the deinitialization of the line owner.
 *
 * \details Channel 1 in DMA mode, DAC1 deinitialized twice.
 *
 * \par Expected results
 * - First deinit: NVIC line TIM6_DAC disabled once, APB1 clock of DAC1 disabled.
 * - Second deinit: line not owned any more, NVIC not called again.
 */
void Ut_Dac_Isr_LineDeinit_DisabledOnce( void )
{
#if defined(DAC1)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Deinit( UT_DAC_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_NvicInactiveCnt );
    TEST_ASSERT_EQUAL( RCC_PERIPH_DAC1, utDac_RccLastInactive );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Deinit( UT_DAC_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utDac_NvicInactiveCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}


/**
 * \brief   Interrupt service routine of the DAC line ends with DSB.
 *
 * \details DAC1 channel 1 (line TIM6_DAC) in one shot DMA mode, captured ISR called once
 *          without flags.
 *
 * \note    Cortex-M4 r0p1 erratum 838869 "Store immediate overlapping exception return
 *          operation might vector to incorrect interrupt" (STM32G4 Bug AB#1104, STM32L4 errata
 *          sheets not reviewed yet): a buffered store with immediate offset still pending at the
 *          exception return may vector to an incorrect interrupt. Workaround - DSB before the
 *          exception return of every handler.
 *
 * \par Expected results
 * - The ISR executes exactly one DSB.
 */
void Ut_Dac_Isr_EndsWithDsb( void )
{
#if defined(DAC1)
    dac_Config_t config;

    Ut_Dac_Get_Config( &config );
    Ut_Dac_Set_DmaConfig( &config.PeriphConfig[ UT_DAC_PERIPH ].Channels[ DAC_CHANNEL_1 ], DAC_BUFFER_MODE_ONE_SHOT );
    Ut_Dac_Init( &config );
    TEST_ASSERT_NOT_NULL( utDac_Isr );

    UT_DAC_REG->SR = 0u;

    const uint32_t dsbCnt = CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB );

    utDac_Isr();

    TEST_ASSERT_EQUAL_UINT32( dsbCnt + 1u, CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* DAC1 */
}

/* ========================== LOCAL FUNCTIONS =============================== */

#if defined(DAC1)
/**
 * \brief HW model of the DAC - runs in background thread.
 *
 * CAL_FLAGx is set while CENx = 1 and OTRIMx >= utDac_CalThreshold.
 */
static void Ut_Dac_HwModel( void )
{
    const uint32_t cr      = UT_DAC_REG->CR;
    const uint32_t ccr     = UT_DAC_REG->CCR;
    const uint32_t trim1   = ( ccr & DAC_CCR_OTRIM1 ) >> DAC_CCR_OTRIM1_Pos;
    uint32_t       setMask = 0u;
    uint32_t       clrMask = 0u;

    if( ( 0u                 != ( cr & DAC_CR_CEN1 ) ) &&
        ( utDac_CalThreshold <= trim1                )    )
    {
        setMask |= DAC_SR_CAL_FLAG1;
    }
    else
    {
        clrMask |= DAC_SR_CAL_FLAG1;
    }

#if defined(DAC_CHANNEL2_SUPPORT)
    const uint32_t trim2 = ( ccr & DAC_CCR_OTRIM2 ) >> DAC_CCR_OTRIM2_Pos;

    if( ( 0u                 != ( cr & DAC_CR_CEN2 ) ) &&
        ( utDac_CalThreshold <= trim2                )    )
    {
        setMask |= DAC_SR_CAL_FLAG2;
    }
    else
    {
        clrMask |= DAC_SR_CAL_FLAG2;
    }
#endif /* DAC_CHANNEL2_SUPPORT */

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


/** \brief RCC activation stub - stores the last activated peripheral */
static rcc_RequestState_t Ut_Dac_RccSetActiveStub( rcc_PeriphId_t periphId, int callCnt )
{
    (void)callCnt;

    utDac_RccLastActive = periphId;
    utDac_RccActiveCnt++;

    return ( RCC_REQUEST_OK );
}


/** \brief RCC deactivation stub - stores the last deactivated peripheral */
static rcc_RequestState_t Ut_Dac_RccSetInactiveStub( rcc_PeriphId_t periphId, int callCnt )
{
    (void)callCnt;

    utDac_RccLastInactive = periphId;

    return ( RCC_REQUEST_OK );
}


/** \brief RCC oscillator state stub - LSI state utDac_LsiState */
static rcc_RequestState_t Ut_Dac_RccGetOscStateStub( rcc_OscId_t oscId, rcc_FunctionState_t * const retState, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( RCC_OSC_LSI, oscId );

    *retState = utDac_LsiState;

    return ( RCC_REQUEST_OK );
}


/** \brief RCC oscillator start stub - counts LSI starts */
static rcc_RequestState_t Ut_Dac_RccSetOscActiveStub( rcc_OscId_t oscId, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( RCC_OSC_LSI, oscId );

    utDac_LsiStartCnt++;

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


/** \brief NVIC handler registration stub - stores ISR of the DAC and its line */
static nvic_RequestState_t Ut_Dac_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( irqHandler );

    utDac_Isr      = irqHandler;
    utDac_IsrIrqId = irqId;

    return ( NVIC_REQUEST_OK );
}


/** \brief NVIC line deactivation stub - counts deactivations of the DAC line */
static nvic_RequestState_t Ut_Dac_NvicSetInactiveStub( nvic_PeriphIrqList_t irqId, int callCnt )
{
    (void)irqId;
    (void)callCnt;

    utDac_NvicInactiveCnt++;

    return ( NVIC_REQUEST_OK );
}


/** \brief DMA initialization stub - stores the channel configuration */
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

    utDac_DmaBus     = dmaBus;
    utDac_DmaChannel = dmaChannel;
    utDac_DmaMode    = transferMode;

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


/** \brief DMA channel enable stub - result given by utDac_DmaActiveResult */
static dma_RequestState_t Ut_Dac_DmaActiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    utDac_DmaActiveCnt++;

    return ( utDac_DmaActiveResult );
}


/** \brief DMA channel disable stub */
static dma_RequestState_t Ut_Dac_DmaInactiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)dmaBus;
    (void)callCnt;

    utDac_DmaInactiveChannel = dmaChannel;
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
    Rcc_Set_PeriphInactive_StubWithCallback( Ut_Dac_RccSetInactiveStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Dac_RccGetClkStub );
    Rcc_Get_OscState_StubWithCallback( Ut_Dac_RccGetOscStateStub );
    Rcc_Set_OscActive_StubWithCallback( Ut_Dac_RccSetOscActiveStub );

    Gpio_Init_StubWithCallback( Ut_Dac_GpioInitStub );

    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Dac_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_StubWithCallback( Ut_Dac_NvicSetInactiveStub );

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

    for( dac_PeriphId_t periphIdx = DAC_PERIPH_1; DAC_PERIPH_CNT > periphIdx; periphIdx++ )
    {
        (void)Dac_Deinit( periphIdx );
    }

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
 * \brief Configures DMA transfer of a channel with TIM6 trigger and test buffer (DMA channel
 *        of the DAC channel is kept from the default configuration).
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
#endif /* DAC1 */
