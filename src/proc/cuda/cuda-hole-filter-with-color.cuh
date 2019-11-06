#pragma once
#ifdef RS2_USE_CUDA

#include "../../../include/librealsense2/rs.h"
#include <memory>
#include <stdint.h>

namespace librealsense
{
/** hole_filter_with_colorでCUDAを使用するためのヘルパクラス
*   align_cuda_helperを参考に作成
*/
class hole_filter_with_color_cuda_helper
{
public:
	hole_filter_with_color_cuda_helper() :
		_d_depth_in(nullptr),
		_d_rgb_in(nullptr),
		_d_lab_out(nullptr)	{}

	void convert_rgb8_to_lab( const uint8_t* h_rgb_in, float* h_lab_out, const uint16_t width, const uint16_t height );

private:
	std::shared_ptr<uint16_t>		_d_depth_in;
	std::shared_ptr<uint8_t>		_d_rgb_in;
	std::shared_ptr<float>			_d_lab_out;
};


}

#endif // RS2_USE_CUDA