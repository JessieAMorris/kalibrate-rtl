include_directories(".")

set(kalibrate_files
	arfcn_freq.cc
	c0_detect.cc
	circular_buffer.cc
	fcch_detector.cc
	kal.cc
	offset.cc
	util.cc
	xtrx_source.cc)

add_definitions(-DXTRX_DEV)

set(DEVICE_LIBS   ${XTRX_LIBRARIES})
set(DEVICE_INC    ${XTRX_INCLUDES})

include_directories(${DEVICE_INC} ${FFTW_INCLUDES})

add_executable(kalibrate-xtrx ${kalibrate_files})
target_link_libraries(kalibrate-xtrx ${DEVICE_LIBS} ${FFTW_LIB} pthread dl)


