#===================== Integration tests configuration ========================#
# Integration tests of Dac module on target (Nucleo-144 boards of STM32H7). Registered in
# module CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# Outputs are verified by the output registers, the outputs of DAC1 are measured by
# ADC2 channel 16 (internal connection) through Adc module. TIM6 TRGO is the hardware
# trigger of DMA transfers, driven through LL directly by the test. No wiring is needed.
#
# The test set is independent - target interface (entry point, initialization,
# fault handlers, system reset) is implemented by ItTarget_Dac.c and
# all libraries of the firmware are listed here.

# STM32H7R / STM32H7S have no DAC (module is empty) - the test set is not created.
if(NOT MCU_FAMILY_ID STREQUAL "STM32H7RSxx")

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
                        Adc_Lib             # ADC2 measurement of the DAC1 outputs
                        Rcc_Lib             # Clock of TIM6, DAC clock / LSI state verification
                        Dma_Lib             # DMA streams of the DAC (underrun test)
        TIMEOUT         60
    )

endif()
