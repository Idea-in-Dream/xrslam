cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-superbuild)

# if(NOT IOS)
#   set(XRPrimer_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../xrprimer/install/lib/cmake")
# else()
#   set(XRPrimer_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../xrprimer/install/ios/lib/cmake")
# endif()
# find_package(XRPrimer REQUIRED)

include(SuperBuildDepends)

option(XRSLAM_DEBUG "..." OFF)
option(XRSLAM_ENABLE_DEBUG_INSPECTION "..." ON)
option(XRSLAM_ENABLE_THREADING "..." OFF)
option(XRSLAM_PC_HEADLESS_ONLY "..." OFF)
option(XRSLAM_IOS "..." ON)
option(XRSLAM_TEST "..." OFF)


if(NOT IOS)
  set(XRSLAM_PC ON)
endif()

add_subdirectory(xrslam)
add_subdirectory(xrslam-extra)
add_subdirectory(xrslam-interface)
add_subdirectory(xrslam-localization)

if(XRSLAM_PC)
  add_subdirectory(xrslam-pc)
endif()
