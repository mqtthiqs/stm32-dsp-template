add_executable(main main.cc)

target_link_libraries(main PRIVATE grm hal m)

target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})


target_compile_options(main PUBLIC
  "-fdata-sections"
  "-ffunction-sections"
  "-fno-rtti"
  "-fno-exceptions"
)

target_link_options(main PUBLIC
  "-Wl,--gc-sections"
  "-nostartfiles"
  "-nostdlib"
)

add_custom_command(
    TARGET main
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ARGS -O binary main main.bin
)

add_custom_target(flash
  DEPENDS main
  COMMAND sh ${PROJECT_SOURCE_DIR}/flash.sh main.bin
)
