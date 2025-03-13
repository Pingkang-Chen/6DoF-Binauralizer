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
    # VERSION ...                               # Set this if the plugin version is different to the project version
    # ICON_BIG ...                              # ICON_* arguments specify a path to an image file to use as an icon for the Standalone
    # ICON_SMALL ...
    # COMPANY_NAME ...                          # Specify the name of the plugin's author
    # IS_SYNTH TRUE/FALSE                       # Is this a synth or an effect?
    IS_SYNTH FALSE  # Changed from TRUE to FALSE since this is now an effect
    # NEEDS_MIDI_INPUT TRUE/FALSE               # Does the plugin need midi input?
    NEEDS_MIDI_INPUT FALSE  # Changed from TRUE to FALSE since we don't need MIDI
    # NEEDS_MIDI_OUTPUT TRUE/FALSE              # Does the plugin need midi output?
    # IS_MIDI_EFFECT TRUE/FALSE                 # Is this plugin a MIDI effect?
    # EDITOR_WANTS_KEYBOARD_FOCUS TRUE/FALSE    # Does the editor need keyboard focus?
    # COPY_PLUGIN_AFTER_BUILD TRUE/FALSE        # Should the plugin be installed to a default location after building?
    PLUGIN_MANUFACTURER_CODE Juce               # A four-character manufacturer id with at least one upper-case character
    PLUGIN_CODE Bnrl                            # A unique four-character plugin id with exactly one upper-case character
                                                # GarageBand 10.3 requires the first letter to be upper-case, and the remaining letters to be lower-case
    FORMATS AU VST3 Standalone                  # The formats to build. Other valid formats are: AAX Unity VST AU AUv3
    PRODUCT_NAME "Binaural Processor")        # The name of the final executable

target_sources(BinauralProcessor
    PRIVATE
        PluginEditor.cpp
        PluginProcessor.cpp)

juce_generate_juce_header(BinauralProcessor)

target_compile_definitions(BinauralProcessor
    PUBLIC
        # JUCE_WEB_BROWSER and JUCE_USE_CURL would be on by default, but you might not need them.
        JUCE_WEB_BROWSER=0  # If you remove this, add `NEEDS_WEB_BROWSER TRUE` to the `juce_add_plugin` call
        JUCE_USE_CURL=0     # If you remove this, add `NEEDS_CURL TRUE` to the `juce_add_plugin` call
        JUCE_VST3_CAN_REPLACE_VST2=0)

target_include_directories(BinauralProcessor 
    PRIVATE 
        ${MYSOFA_INCLUDE_DIR}
        ${MYSOFA_DIR}/src
        ${MYSOFA_DIR}/install/include
        ${ZLIB_INCLUDE_DIRS})  # Add zlib include directories

target_link_libraries(BinauralProcessor
    PRIVATE
        # AudioPluginData           # If we'd created a binary data target, we'd link to it here
        juce::juce_audio_utils
        juce::juce_dsp
        ${MYSOFA_LIBRARY}
        ${ZLIB_LIBRARIES}  # Link against zlib
        juce::juce_osc  # Add OSC module
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags)