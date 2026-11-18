cmake_minimum_required(VERSION 3.16)
project(smartlink_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(smartlink
  src/main.cpp
  src/inference.cpp
  src/transport.cpp
  src/fm.cpp
)

target_include_directories(smartlink PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/common
)

if(WIN32)
  target_link_libraries(smartlink PRIVATE ws2_32 winmm)
  target_compile_definitions(smartlink PRIVATE _CRT_SECURE_NO_WARNINGS)
else()
  target_link_libraries(smartlink PRIVATE pthread m)
endif()
