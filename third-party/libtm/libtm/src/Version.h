// INTEL CORPORATION PROPRIETARY INFORMATION
// This software is supplied under the terms of a license agreement or nondisclosure
// agreement with Intel Corporation and may not be copied or disclosed except in
// accordance with the terms of that agreement
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

// Version.h file is an auto-generated file made by CMake from file Version.h.in. 
// Changes to Version.h are likely to be overwritten and lost

#pragma once
#define LIBTM_VERSION_MAJOR 0
#define LIBTM_VERSION_MINOR 19
#define LIBTM_VERSION_PATCH 3
#define LIBTM_VERSION_BUILD 1711

#define LIBTM_API_VERSION_MAJOR 10
#define LIBTM_API_VERSION_MINOR 0

#define LIBTM_VERSION_BUILD_POS 0
#define LIBTM_VERSION_BUILD_MSK 0xFFFFFFFF
#define LIBTM_VERSION_PATCH_POS 32
#define LIBTM_VERSION_PATCH_MSK 0xFFFF
#define LIBTM_VERSION_MINOR_POS 48
#define LIBTM_VERSION_MINOR_MSK 0xFF
#define LIBTM_VERSION_MAJOR_POS 56
#define LIBTM_VERSION_MAJOR_MSK 0xFF

#define LIBTM_VERSION (((uint64_t)(LIBTM_VERSION_MAJOR & LIBTM_VERSION_MAJOR_MSK) << LIBTM_VERSION_MAJOR_POS) | \
                       ((uint64_t)(LIBTM_VERSION_MINOR & LIBTM_VERSION_MINOR_MSK) << LIBTM_VERSION_MINOR_POS) | \
                       ((uint64_t)(LIBTM_VERSION_PATCH & LIBTM_VERSION_PATCH_MSK) << LIBTM_VERSION_PATCH_POS) | \
                       ((uint64_t)(LIBTM_VERSION_BUILD & LIBTM_VERSION_BUILD_MSK) << LIBTM_VERSION_BUILD_POS))

#define LIBTM_BRANCH feature/hole_filter_with_color
