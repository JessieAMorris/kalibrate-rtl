cmake_minimum_required(VERSION 2.8)
project(kalibrate C CXX)

# Set the version information here
set(MAJOR_VERSION 0)
set(API_COMPAT    0)
set(MINOR_VERSION 1)
set(MAINT_VERSION git)

set(LIBVER  "${MAJOR_VERSION}.${API_COMPAT}.${MINOR_VERSION}")

add_definitions(-Wall -g)

#CONFIGURE_FILE(
#    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake.in
#    ${CMAKE_CURRENT_BINARY_DIR}/config.h
#@ONLY)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(FFTW)
find_package(XTRX)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_subdirectory(src)





