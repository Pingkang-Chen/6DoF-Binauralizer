cmake_minimum_required(VERSION 3.22)
project(BINAURAL_PROCESSOR VERSION 0.0.1)

add_subdirectory(JUCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find zlib dependency
find_package(ZLIB REQUIRED)
if(NOT ZLIB_FOUND)
    message(FATAL_ERROR "zlib not found. Please install zlib development package")
endif()

# Set up libmysofa from local directory
set(MYSOFA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libmysofa")
set(MYSOFA_INCLUDE_DIR "${MYSOFA_DIR}/src/hrtf")

# Check if header exists
if(NOT EXISTS "${MYSOFA_INCLUDE_DIR}/mysofa.h")
    message(FATAL_ERROR "mysofa.h not found in ${MYSOFA_INCLUDE_DIR}. Please check the path.")
endif()

# Copy the mysofa_export.h.in file to mysofa_export.h (in the same directory)
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/libmysofa/src/hrtf/mysofa_export.h.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/libmysofa/src/hrtf/mysofa_export.h"
    COPYONLY)

# Add the include directories for libmysofa
include_directories(${MYSOFA_INCLUDE_DIR})
include_directories(${MYSOFA_DIR}/src)

# Update path to the existing libmysofa.a or libmysofa.dylib
set(MYSOFA_LIBRARY "${MYSOFA_DIR}/install/lib/libmysofa.a" CACHE FILEPATH "Path to libmysofa library")

# Check if the library exists at the new location
if(NOT EXISTS ${MYSOFA_LIBRARY})
    message(WARNING "libmysofa not found at ${MYSOFA_LIBRARY}.")
    
    # Try looking for libmysofa.dylib instead of libmysofa.a
    if(EXISTS "${MYSOFA_DIR}/install/lib/libmysofa.dylib")
        set(MYSOFA_LIBRARY "${MYSOFA_DIR}/install/lib/libmysofa.dylib" CACHE FILEPATH "Path to libmysofa library" FORCE)
        message(STATUS "Found libmysofa at ${MYSOFA_LIBRARY}")
    else()
        message(FATAL_ERROR "Could not find libmysofa library. Please ensure it exists at ${MYSOFA_LIBRARY}")
    endif()
endif()

juce_add_plugin(BinauralProcessor
    IS_SYNTH FALSE
    NEEDS_MIDI_INPUT FALSE
    PLUGIN_MANUFACTURER_CODE Juce
    PLUGIN_CODE Bnrl
    FORMATS AU VST3 Standalone
    PRODUCT_NAME "Binaural Processor")

target_sources(BinauralProcessor
    PRIVATE
        PluginEditor.cpp
        PluginProcessor.cpp
        BRIRProcessor.cpp)  # Add the new BRIR source file

juce_generate_juce_header(BinauralProcessor)

target_compile_definitions(BinauralProcessor
    PUBLIC
        JUCE_WEB_BROWSER=0
        JUCE_USE_CURL=0
        JUCE_VST3_CAN_REPLACE_VST2=0)

target_include_directories(BinauralProcessor 
    PRIVATE 
        ${MYSOFA_INCLUDE_DIR}
        ${MYSOFA_DIR}/src
        ${MYSOFA_DIR}/install/include
        ${ZLIB_INCLUDE_DIRS})

target_link_libraries(BinauralProcessor
    PRIVATE
        juce::juce_audio_utils
        juce::juce_dsp
        ${MYSOFA_LIBRARY}
        ${ZLIB_LIBRARIES}
        juce::juce_osc
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags)