#ifdef RS2_USE_CUDA

#include "cuda-hole-filter-with-color.cuh"
#include "../../../include/librealsense2/rsutil.h"
#include "../../cuda/rscuda_utils.cuh"
#include <cuda_runtime.h>
#include "device_launch_parameters.h"

using namespace librealsense;
using namespace rscuda;

__device__ float gamma_expanded( const float u )
{
	return u > 0.04045 ? pow( ( u+0.055 )/1.055, 2.4 ) : ( u / 12.92 );
}


__device__ void convert_rgb8_to_xyz( const uint8_t* rgb, float& x, float& y, float& z )
{
	// RGBの値を0.0~1.0に
	float r = rgb[0] / 255.0f;
	float g = rgb[1] / 255.0f;
	float b = rgb[2] / 255.0f;

	// ガンマ補正の除去
	r = gamma_expanded( r );
	g = gamma_expanded( g );
	b = gamma_expanded( b );

	// XYZに変換
	x = 0.4124 * r + 0.3576 * g + 0.1805 * b;
	y = 0.2126 * r + 0.7152 * g + 0.0722 * b;
	z = 0.0193 * r + 0.1192 * g + 0.9505 * b;

	x *= 100;
	y *= 100;
	z *= 100;
}

__global__ void kernel_convert_rgb8_to_lab( const uint8_t* rgb_in, float* lab_out, const uint16_t width, const uint16_t height )
{
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;

	// 式温度6,500Kの昼光色を想定した標準光源D65のXYZ値
	float xn = 95.05f, yn = 100.0f, zn = 108.91f;

	// xyzの値の補正　pow( 24/116, 3 ) = 0.008856, 1 / ( 3*pow(24/116) ) = 7.787037
	auto f = []( float val )
	{
		return val > 0.008856 ? std::powf( val, 1.0f / 3.0f ) : (7.787037 * val) + 0.137931;
	};

	int idx = y * width + x;
	
	if( idx < width * height )
	{
		// RGB,L*a*bそれぞれの編集位置のポインタ
		const uint8_t* p_rgb = rgb_in + idx * 3;
		float* p_lab = lab_out + idx * 3;

		// sRGBからCIE XYZへ変換
		float x, y, z;
		convert_rgb8_to_xyz( p_rgb, x, y, z );

		float fx = f( x / xn );
		float fy = f( y / yn );
		float fz = f( z / zn );

		p_lab[0] = 116 * fy - 16;
		p_lab[1] = 500 * ( fx - fy );
		p_lab[2] = 200 * ( fy - fz );

	}

}

void hole_filter_with_color_cuda_helper::convert_rgb8_to_lab( const uint8_t* h_rgb_in, float* h_lab_out, const uint16_t width, const uint16_t height )
{
	int size = width * height;
	if ( !_d_rgb_in ) { _d_rgb_in = alloc_dev<uint8_t>( size * 3 ); }
	cudaMemcpy( _d_rgb_in.get(), h_rgb_in, sizeof( uint8_t ) * size * 3, cudaMemcpyHostToDevice );

	if ( !_d_lab_out ) { _d_lab_out = alloc_dev<float>( size * 3 ); }
	cudaMemset( _d_lab_out.get(), 0, sizeof( float ) * size * 3 );

	const int threadSize = 32;
	dim3 threads( threadSize, threadSize );
	dim3 blocks( (width + threadSize - 1) / threadSize, (height + threadSize - 1) / threadSize );
	
	kernel_convert_rgb8_to_lab << <blocks, threads >> > (_d_rgb_in.get(), _d_lab_out.get(), width, height);

	cudaDeviceSynchronize();

	cudaMemcpy( h_lab_out, _d_lab_out.get(), size * 3 * sizeof(float), cudaMemcpyDeviceToHost );
}


#endif // RS2_USE_CUDA
