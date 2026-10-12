cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# disable cxa_atexit to avoid global/static destructor registration
# suitable for bare-metal targets without process exit
add_compile_options(
        $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)


# Define the build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()

# Set the project name
set(CMAKE_PROJECT_NAME CoreV2.1_FullTest)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# CubeMX hard-codes the EthIf stack to 350 bytes; CMSIS-RTOS V2 needs 1024 here.
# Patch during configuration and track the generated source so the next build
# reapplies the patch after CubeMX regeneration. Only write when the value changes.
set(ETHERNETIF_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/LWIP/Target/ethernetif.c")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${ETHERNETIF_SOURCE}")
file(READ "${ETHERNETIF_SOURCE}" ETHERNETIF_CONTENT)
set(ETHERNETIF_STACK_PATTERN
        "(^|\n)([ \t]*#[ \t]*define[ \t]+INTERFACE_THREAD_STACK_SIZE)[ \t]+[^\r\n]*")
string(REGEX MATCHALL "${ETHERNETIF_STACK_PATTERN}" ETHERNETIF_STACK_DEFINITIONS "${ETHERNETIF_CONTENT}")
list(LENGTH ETHERNETIF_STACK_DEFINITIONS ETHERNETIF_STACK_DEFINITION_COUNT)
if (NOT ETHERNETIF_STACK_DEFINITION_COUNT EQUAL 1)
    message(FATAL_ERROR "Expected exactly one INTERFACE_THREAD_STACK_SIZE definition in ${ETHERNETIF_SOURCE}")
endif ()
string(REGEX REPLACE "${ETHERNETIF_STACK_PATTERN}" "\\1\\2 ( 1024 )"
        ETHERNETIF_PATCHED_CONTENT "${ETHERNETIF_CONTENT}")
if (NOT "${ETHERNETIF_CONTENT}" STREQUAL "${ETHERNETIF_PATCHED_CONTENT}")
    file(WRITE "${ETHERNETIF_SOURCE}" "${ETHERNETIF_PATCHED_CONTENT}")
    message(STATUS "Patched EthIf thread stack size to 1024 bytes")
endif ()

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# Add sources to executable
file(GLOB_RECURSE USER_CODE_SOURCES "UserCode/*.*")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
        ${USER_CODE_SOURCES}
)

# Add include paths
include_directories(UserCode)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

include(cmake/wtr_modules.cmake)

wtr_link_packages(${CMAKE_PROJECT_NAME})

# Add driver module dependencies here
# ===================== DEPENDENCIES =====================
# e.g.
# add_subdirectory(Modules/YourDriver)
# target_link_libraries(${CMAKE_PROJECT_NAME} YourDriver)

# =======================================================

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx

        # Add user defined libraries
)
