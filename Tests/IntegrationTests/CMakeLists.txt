#===================== Integration tests configuration ========================#
# Integration tests of Dac module on target (STM32F4DISCOVERY). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# DAC_OUT1 (PA4) / DAC_OUT2 (PA5) are measured by ADC1 channels 4 / 5 (same pins)
# through Adc module. TIM6 TRGO is the hardware trigger of DMA transfers, driven
# through LL directly by the test. No wiring is needed.
#
# The test set is independent - target interface (entry point, initialization,
# fault handlers, system reset) is implemented by ItTarget_Dac.c and
# all libraries of the firmware are listed here.

# MCUs without DAC peripheral (e.g. STM32F401, STM32F411, STM32F412 - CMSIS device
# header defines no DAC) have empty Dac_Lib - the test set is not created.
get_target_property(DAC_IT_RAL_DEFINES  Ral_Lib INTERFACE_COMPILE_DEFINITIONS)
get_target_property(DAC_IT_RAL_INCLUDES Ral_Lib INTERFACE_INCLUDE_DIRECTORIES)
list(FILTER DAC_IT_RAL_DEFINES INCLUDE REGEX "^STM32")
string(TOLOWER "${DAC_IT_RAL_DEFINES}.h" DAC_IT_DEVICE_HEADER)
find_file(DAC_IT_DEVICE_HEADER_PATH "${DAC_IT_DEVICE_HEADER}"
    PATHS ${DAC_IT_RAL_INCLUDES} NO_DEFAULT_PATH NO_CACHE REQUIRED)
file(STRINGS "${DAC_IT_DEVICE_HEADER_PATH}" DAC_IT_DAC_DEFINE REGEX "^#define[ \t]+DAC[ \t]")

if(NOT DAC_IT_DAC_DEFINE)
    message(STATUS "Dac integration tests skipped - ${DAC_IT_RAL_DEFINES} has no DAC peripheral")
    return()
endif()

# StartUp calls BspMain() - the entry point of the test firmware is provided by
# the test set (BspMain.h, ItTarget_Dac.c)
if(NOT TARGET BspMain_Lib)
    add_library(BspMain_Lib INTERFACE)
    target_include_directories(BspMain_Lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

IntegrationTesting_Add_Test(
    NAME            Dac
    TEST_SOURCE     ItTest_Dac.c
    SOURCES         ItTarget_Dac.c      # Target interface of the test set
    LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                    Nvic_Lib            # Fault handlers, system reset (ItTarget)
                    Dac_Lib             # Module under test
                    Adc_Lib             # ADC1 measurement of the DAC outputs
                    Rcc_Lib             # Clock of TIM6, DAC clock state verification
                    Dma_Lib             # DMA streams of the DAC (underrun test)
    TIMEOUT         60
)
