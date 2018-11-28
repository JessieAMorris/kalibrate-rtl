cmake_minimum_required(VERSION 2.8)
project(kalibrate)

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

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(FFTW)
find_package(XTRX)

find_package(SoapySDR)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_subdirectory(src)

########################################################################
# package generator
########################################################################
if(NOT CPACK_GENERATOR)
	set(CPACK_GENERATOR DEB)
endif()
set(CPACK_PACKAGE_NAME                "libxtrx")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "XTRX API library")
set(CPACK_PACKAGE_VENDOR              "Fairwaves, Inc.")
set(CPACK_PACKAGE_CONTACT             "http://fairwaves.co/wp/contact-us/")
set(CPACK_PACKAGE_VERSION             ${LIBVER}-1)
if(ENABLE_SOAPY)
	set(CPACK_DEBIAN_PACKAGE_DEPENDS      "${CPACK_DEBIAN_PACKAGE_DEPENDS}, soapysdr (>= 0.5.2)")
endif(ENABLE_SOAPY)

set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_SET_DESTDIR               "")
set(CPACK_PACKAGING_INSTALL_PREFIX  "/usr")
set(CPACK_PACKAGE_CONTACT           "Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>")

include(CPack) #include last
