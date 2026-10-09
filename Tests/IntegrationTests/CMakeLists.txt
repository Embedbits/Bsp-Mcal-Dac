#===================== Integration tests configuration ========================#
# Integration tests of Dac module on target (Nucleo board). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# Outputs are verified by the output registers and measured by the ADC through Adc
# module (internal outputs by the ADC channels of the DAC outputs, PA4 by ADC1
# channel 9 on boards with free PA4). TIM6 TRGO is the hardware trigger of DMA
# transfers, driven through LL directly by the test. No wiring is needed.
# STM32L41x / L42x have no DAC - all tests ignored (Dac_Lib empty).
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
                    Adc_Lib             # ADC measurement of the DAC outputs
                    Rcc_Lib             # Clock of TIM6, DAC clock / LSI state verification
                    Dma_Lib             # DMA channels of the DAC (underrun tests)
    TIMEOUT         60
)
