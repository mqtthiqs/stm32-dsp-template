add_library(grm
  maths.cpp
)

target_compile_features(grm PRIVATE cxx_std_17)

target_include_directories(grm INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
