cmake_minimum_required (VERSION 3.19)
project (skils)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake//")

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  find_library(CORE_GRAPHICS CoreGraphics)
  find_library(COCOA COCOA)
  find_library(IOKIT IOKit)
  find_library(CORE_FOUNDATION CoreFoundation)
  find_library(OPENGL opengl)
endif()

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(GDAL REQUIRED)
find_package(CapnProto REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
  message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

set (CMAKE_CXX_STANDARD 20)
include_directories(/opt/local/include)
link_directories(/opt/local/lib)
include_directories(${PROJECT_SOURCE_DIR}/colormap/include)

add_definitions(-Wno-c++11-narrowing)
add_definitions(-Wno-deprecated-enum-compare-conditional)

add_executable(skils skils.cc)
target_include_directories(skils
                          PRIVATE
                          ${JSONCPP_INCLUDE_DIRS}
                          ${EIGEN3_INCLUDE_DIR})

target_link_libraries(skils
                      PRIVATE
                      raylib
                      ${JSONCPP_LINK_LIBRARIES}
                      ${GDAL_LIBRARIES}
                      ${CERES_LIBRARIES})

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(skils
                        PRIVATE
                        ${CORE_GRAPHICS}
                        ${CORE_FOUNDATION}
                        ${IOKIT}
                        ${COCOA}
                        ${OPENGL})
endif()
