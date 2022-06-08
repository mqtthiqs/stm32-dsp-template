add_library(hal
    stm32f4xx_hal.c
    stm32f4xx_hal_cortex.c
    stm32f4xx_hal_gpio.c
    stm32f4xx_hal_rcc.c
    stm32f4xx_hal_rcc_ex.c
    stm32f4xx_hal_dma.c
    stm32f4xx_hal_i2c.c
    stm32f4xx_hal_i2s.c
    stm32f4xx_hal_i2s_ex.c
    stm32f4xx_hal_spi.c
    stm32f4xx_hal_dac.c
)

target_link_libraries(hal PUBLIC cmsis)
target_compile_options(hal PRIVATE "-Wno-attributes")

target_include_directories(hal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})