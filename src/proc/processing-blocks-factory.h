// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once

#include "align.h"
#include "hole-filter-with-color.h"
#include "force-flattening-filter.h"

namespace librealsense
{
    std::shared_ptr<librealsense::align> create_align(rs2_stream align_to);

	std::shared_ptr<librealsense::hole_filter_with_color> create_hole_filter_with_color();

	std::shared_ptr<librealsense::force_flattening_filter> create_force_flattening_filter();
}
