#pragma once
#ifdef RS2_USE_CUDA

#include "../../../include/librealsense2/rs.h"
#include <memory>
#include <stdint.h>

namespace librealsense
{
/** hole_filter_with_color��CUDA���g�p���邽�߂̃w���p�N���X
*   align_cuda_helper���Q�l�ɍ쐬
*/
class hole_filter_with_color_cuda_helper
{
public:
	hole_filter_with_color_cuda_helper() :
		_d_depth_in( nullptr ),
		_d_depth_out( nullptr ),
		_d_rgb_in( nullptr ),
		_d_lab_in( nullptr ),
		_d_lab_out( nullptr )
	{
		for ( int i = 0; i < 11*11; ++i )
		{
			_h_coef_P5[i] = 0;
		}
	}

	void convert_rgb8_to_lab( const uint8_t* h_rgb_in, float* h_lab_out, const uint16_t width, const uint16_t height );

	void hole_filter_process(
		uint16_t* h_depth_out,
		const uint16_t* h_depth_in,
		const float* h_lab_in,
		const int kernel_w,
		const int width,
		const int height
	);


private:
	std::shared_ptr<uint16_t>		_d_depth_in;
	std::shared_ptr<uint16_t>		_d_depth_out;
	std::shared_ptr<uint8_t>		_d_rgb_in;
	std::shared_ptr<float>			_d_lab_in;
	std::shared_ptr<float>			_d_lab_out;
	std::shared_ptr<float>			_d_p_vec_in;
	float _h_coef_P5[11 * 11];

	void initializeCoef();

};


}

#endif // RS2_USE_CUDA