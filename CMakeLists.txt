cmake_minimum_required(VERSION 3.22)
project(ESSAIM VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# JUCE épinglé (8.0.15)
include(FetchContent)
FetchContent_Declare(JUCE
  GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
  GIT_TAG        8.0.15
  GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(JUCE)

# ASIO (Windows) : -DESSAIM_ASIO=ON + variable d'env ASIOSDK_DIR pointant sur le SDK Steinberg
option(ESSAIM_ASIO "Compiler le standalone avec le support ASIO (Windows)" OFF)

juce_add_plugin(Essaim
  COMPANY_NAME "Paye Ta Ruche"
  BUNDLE_ID com.payetaruche.essaim
  PLUGIN_MANUFACTURER_CODE Ptrh
  PLUGIN_CODE Essm
  FORMATS VST3 Standalone
  PRODUCT_NAME "L'ESSAIM"
  MICROPHONE_PERMISSION_ENABLED TRUE
  MICROPHONE_PERMISSION_TEXT "L'ESSAIM capture l'entrée audio pour enregistrer les boucles."
  NEEDS_WEB_BROWSER TRUE
  NEEDS_WEBVIEW2 TRUE
  VST3_CATEGORIES Fx Tools)

juce_add_binary_data(EssaimUI
  HEADER_NAME BinaryData.h
  NAMESPACE BinaryData
  SOURCES UI/essaim_ui.html)

target_sources(Essaim PRIVATE
  Source/Engine/Engine.cpp
  Source/Plugin/PluginProcessor.cpp
  Source/Plugin/PluginEditor.cpp)

target_include_directories(Essaim PRIVATE Source)

target_compile_definitions(Essaim PUBLIC
  JUCE_WEB_BROWSER=1
  JUCE_USE_CURL=0
  JUCE_USE_WIN_WEBVIEW2_WITH_STATIC_LINKING=1
  JUCE_VST3_CAN_REPLACE_VST2=0
  JUCE_DISPLAY_SPLASH_SCREEN=0)

if(ESSAIM_ASIO AND DEFINED ENV{ASIOSDK_DIR})
  message(STATUS "ASIO activé : $ENV{ASIOSDK_DIR}")
  target_compile_definitions(Essaim PUBLIC JUCE_ASIO=1)
  target_include_directories(Essaim PRIVATE "$ENV{ASIOSDK_DIR}/common")
endif()

target_link_libraries(Essaim
  PRIVATE
    EssaimUI
    juce::juce_audio_utils
    juce::juce_dsp
    juce::juce_gui_extra
  PUBLIC
    juce::juce_recommended_config_flags
    juce::juce_recommended_lto_flags
    juce::juce_recommended_warning_flags)
