# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
target_sources(${LRS_TARGET}
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/cuda-align.h"
        "${CMAKE_CURRENT_LIST_DIR}/cuda-align.cu"
        "${CMAKE_CURRENT_LIST_DIR}/cuda-align.cuh"
	"${CMAKE_CURRENT_LIST_DIR}/cuda-hole-filter-with-color.h"
	"${CMAKE_CURRENT_LIST_DIR}/cuda-hole-filter-with-color.cu"
	"${CMAKE_CURRENT_LIST_DIR}/cuda-hole-filter-with-color.cuh"
	"${CMAKE_CURRENT_LIST_DIR}/cuda-force-flattening-filter.h"
	"${CMAKE_CURRENT_LIST_DIR}/cuda-force-flattening-filter.cu"
	"${CMAKE_CURRENT_LIST_DIR}/cuda-force-flattening-filter.cuh"
    "${CMAKE_CURRENT_LIST_DIR}/cuda-pointcloud.h"
        "${CMAKE_CURRENT_LIST_DIR}/cuda-pointcloud.cpp"

)
