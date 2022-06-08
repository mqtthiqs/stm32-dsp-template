enable_language(ASM)

add_library(cmsis
  startup_stm32f407xx.s
  system_stm32f4xx.c
)

target_compile_definitions(cmsis PUBLIC
  STM32F407xx
)

target_compile_options(cmsis PUBLIC
  "-mcpu=cortex-m4"
  "-mthumb"
  "-mfloat-abi=hard"
  "-mfpu=fpv4-sp-d16"
  "-mthumb-interwork"
  "-O2"
)

target_link_options(cmsis PUBLIC
  "-mcpu=cortex-m4"
  "-mthumb"
  "-mfloat-abi=hard"
  "-mfpu=fpv4-sp-d16"
  "-mthumb-interwork"
  "-T${CMAKE_CURRENT_SOURCE_DIR}/STM32F407VGTx_FLASH.ld"
)

target_include_directories(cmsis PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})