#======================== Unit tests configuration ============================#
# Unit tests of Dac module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(DAC_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${DAC_DIR}/.." ABSOLUTE)

# MCUs without DAC peripheral (none of STM32F7 - CMSIS device
# header defines no DAC) have empty Dac_Lib - the test set is not created.
get_target_property(DAC_UT_RAL_DEFINES  Ral_Lib INTERFACE_COMPILE_DEFINITIONS)
get_target_property(DAC_UT_RAL_INCLUDES Ral_Lib INTERFACE_INCLUDE_DIRECTORIES)
list(FILTER DAC_UT_RAL_DEFINES INCLUDE REGEX "^STM32")
string(TOLOWER "${DAC_UT_RAL_DEFINES}.h" DAC_UT_DEVICE_HEADER)
find_file(DAC_UT_DEVICE_HEADER_PATH "${DAC_UT_DEVICE_HEADER}"
    PATHS ${DAC_UT_RAL_INCLUDES} NO_DEFAULT_PATH NO_CACHE REQUIRED)
file(STRINGS "${DAC_UT_DEVICE_HEADER_PATH}" DAC_UT_DAC_DEFINE REGEX "^#define[ \t]+DAC[ \t]")

if(NOT DAC_UT_DAC_DEFINE)
    message(STATUS "Dac unit tests skipped - ${DAC_UT_RAL_DEFINES} has no DAC peripheral")
    return()
endif()

UnitTesting_Add_Test(
    NAME            Dac
    TEST_SOURCE     Test_Dac.c
    UUT_SOURCES     ${DAC_DIR}/Dac.c
                    ${DAC_DIR}/Dac_Dma.c
                    ${DAC_DIR}/Dac_Isr.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
                    ${MCAL_DIR}/Gpio/Gpio_Port.h
                    ${MCAL_DIR}/Dma/Dma_Port.h
    INCLUDE_DIRS    ${DAC_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
                    ${MCAL_DIR}/Gpio
                    ${MCAL_DIR}/Dma
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
)
