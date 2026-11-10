cmake_minimum_required(VERSION 3.16)
project(AutoCore VERSION 1.6.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/utf-8)
endif()

option(AUTOCORE_ENABLE_SANITIZERS "Enable address and undefined-behavior sanitizers" OFF)

function(autocore_enable_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
endfunction()

add_library(autocore
    src/can_bus.cpp
    src/can_controller.cpp
    src/ecus.cpp
    src/diagnostics.cpp
    src/vehicle_network.cpp
    src/isotp.cpp
    src/frame_integrity.cpp
    src/can_message_database.cpp
)
target_include_directories(autocore PUBLIC include)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_sources(autocore PRIVATE src/socketcan.cpp)
    target_compile_definitions(autocore PUBLIC AUTOCORE_HAS_SOCKETCAN=1)
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(autocore_socketcan_demo src/socketcan_demo.cpp)
    target_link_libraries(autocore_socketcan_demo PRIVATE autocore)
    autocore_enable_warnings(autocore_socketcan_demo)
endif()
autocore_enable_warnings(autocore)

if(AUTOCORE_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(autocore PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(autocore PUBLIC -fsanitize=address,undefined)
endif()

add_executable(autocore_demo src/main.cpp)
target_link_libraries(autocore_demo PRIVATE autocore)
autocore_enable_warnings(autocore_demo)

# Persistent JSON-lines engine used by the localhost interactive dashboard.
add_executable(autocore_engine src/engine.cpp)
target_link_libraries(autocore_engine PRIVATE autocore)
autocore_enable_warnings(autocore_engine)

include(CTest)
if(BUILD_TESTING)
    add_executable(autocore_tests tests/test_autocore.cpp)
    target_link_libraries(autocore_tests PRIVATE autocore)
    target_compile_definitions(autocore_tests PRIVATE AUTOCORE_SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")
    autocore_enable_warnings(autocore_tests)
    add_test(NAME autocore_tests COMMAND autocore_tests)
    find_package(Python3 3.10 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        add_test(NAME autocore_dashboard_integration
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_dashboard_integration.py
                    $<TARGET_FILE:autocore_engine>)
        set_tests_properties(autocore_dashboard_integration PROPERTIES TIMEOUT 90)
    endif()
endif()
