#pragma once
#ifdef RS2_USE_CUDA

#include "proc/force-flattening-filter.h"
#include "cuda-force-flattening-filter.cuh"
#include "cuda-align.h"
#include <memory>
#include <stdint.h>

namespace librealsense
{
class force_flattening_filter_cuda : public force_flattening_filter
{
public:
	force_flattening_filter_cuda() : force_flattening_filter() {}

protected:
	void reset_cache() override
	{
		_helper = force_flattening_filter_cuda_helper();
	}

	void convert_rgb8_to_lab( const uint8_t* rgb, float* lab, const uint16_t width, const uint16_t height ) override
	{
		_helper.convert_rgb8_to_lab( rgb, lab, width, height );
	}

	//void hole_filter_process(
	//	uint16_t* new_depth_image,
	//	const uint16_t* depth_image,
	//	const float* lab_image,
	//	const int kernel_w,
	//	const int width,
	//	const int height
	//) override
	//{
	//	_helper.hole_filter_process( new_depth_image, depth_image, lab_image, kernel_w, width, height );
	//}

private:
	force_flattening_filter_cuda_helper _helper;

};



}

#endif