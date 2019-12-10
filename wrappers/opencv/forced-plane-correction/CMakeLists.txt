#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealSenseImShowExample)

add_executable(forced-plane-correction forced-plane-correction.cpp)
set_property(TARGET forced-plane-correction PROPERTY CXX_STANDARD 11)
target_link_libraries(forced-plane-correction ${DEPENDENCIES})
set_target_properties (forced-plane-correction PROPERTIES
	FOLDER "Examples/OpenCV"
)

install(
	TARGETS

	forced-plane-correction

	RUNTIME DESTINATION
	${CMAKE_INSTALL_PREFIX}/bin
)
