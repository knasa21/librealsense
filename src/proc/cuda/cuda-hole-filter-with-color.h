#pragma once
#ifdef RS2_USE_CUDA

#include "proc/hole-filter-with-color.h"
#include "cuda-hole-filter-with-color.cuh"
#include "cuda-align.h"
#include <memory>
#include <stdint.h>

namespace librealsense
{
class hole_filter_with_color_cuda : public hole_filter_with_color
{
public:
	hole_filter_with_color_cuda() : hole_filter_with_color() {}

protected:
	void reset_cache() override
	{
		_helper = hole_filter_with_color_cuda_helper();
	}

	void convert_rgb8_to_lab( const uint8_t* rgb, float* lab, const uint16_t width, const uint16_t height )
	{
		//const int num = 848 * 424 * 3;
		//static float test[num];
		//memset( test, 0, num * sizeof( float ) );
		_helper.convert_rgb8_to_lab( rgb, lab, width, height );
	/*	for ( int i = 0; i < num; ++i )
		{
			test[i] = lab[i];
		}

		hole_filter_with_color::convert_rgb8_to_lab( rgb, lab, width, height );
		for ( int i = 0; i < num; ++i )
		{
			if ( std::fabsf( test[i] - lab[i] ) > 0.01f )
			{
				printf( "%d : error %f != %f\n", i, test[i], lab[i] );
			}
		}*/


	}

private:
	hole_filter_with_color_cuda_helper _helper;

};



}

#endif