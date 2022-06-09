add_library(grm
  maths.cpp
)

target_link_libraries(grm cmsis)

target_compile_options(grm PUBLIC "-Wno-psabi")

target_compile_features(grm PUBLIC cxx_std_17)

target_include_directories(grm INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
