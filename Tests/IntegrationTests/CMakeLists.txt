#===================== Integration tests configuration ========================#
# Integration tests of Dac module on target (NUCLEO-G474RE). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# Outputs are verified by the output registers, DAC1_OUT1 (PA4) is measured by
# ADC2 channel 17 (same pin) through Adc module. TIM6 TRGO is the hardware trigger
# of DMA transfers, driven through LL directly by the test. No wiring is needed.
#
# The test set is independent - target interface (entry point, initialization,
# fault handlers, system reset) is implemented by ItTarget_Dac.c and
# all libraries of the firmware are listed here.

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
                    Adc_Lib             # ADC2 measurement of DAC1_OUT1
                    Rcc_Lib             # Clock of TIM6, DAC clock / LSI state verification
                    Dma_Lib             # DMA channels of the DAC (underrun tests)
    TIMEOUT         60
)
