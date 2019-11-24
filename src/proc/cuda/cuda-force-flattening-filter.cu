#ifdef RS2_USE_CUDA

#include "cuda-force-flattening-filter.cuh"
#include "../../../include/librealsense2/rsutil.h"
#include "../../cuda/rscuda_utils.cuh"
#include <cuda_runtime.h>
#include "device_launch_parameters.h"
#include <vector>
#include "thrust/device_vector.h"

using namespace librealsense;
using namespace rscuda;

#define CUDA_THREAD_SIZE 32;



// CUDA関数エラーチェックマクロ
#define CHECK(call)															\
{																			\
	const cudaError_t error = call;											\
	if ( error != cudaSuccess )												\
	{																		\
		printf("Error: %s:%d, ", __FILE__, __LINE__ );						\
		printf("code:%d, reason: %s\n", error, cudaGetErrorString(error));	\
		exit(1);															\
	}																		\
}	

__constant__ float coef_P1[3][3] = {
	{0.105f, 0.324f, 0.105f, },
	{0.324f, 1.f, 0.324f, },
	{0.105f, 0.324f, 0.105f, },
};

__constant__ float coef_P5[11 * 11];

// カーネルサイズごとの距離の分散(標準偏差σの二乗)
__constant__ float _sqr_space_sigma_array[5] = { 0.444f, 1.12f, 2.811f, 6.829f, 15.122 };

// 標準偏差の計算
__device__ float calc_dispersion( const float* vals, const int size );
//{
//	float sum = 0;
//	// 非0値の数
//	float count = 0;
//
//	for ( int i = 0; i < size; ++i )
//	{
//		float val = vals[i];
//		sum += val;
//		if ( val != 0 )
//		{
//			++count;
//		}
//	}
//
//	// 0しかないとき0を返す 
//	if ( count == 0 )
//	{
//		return 0;
//	}
//
//	float mean = sum / count;
//	sum = 0;
//
//	for ( int i = 0; i < size; ++i )
//	{
//		float val = vals[i];
//		if ( val != 0 )
//		{
//			sum += std::powf( mean - val, 2 );
//		}
//	}
//
//	return sum / count;
//}


// lab値の距離
__device__ float lab_distance( const float* r_lab, const float* l_lab );
//{
//	return
//		std::powf( r_lab[0] - l_lab[0], 2 )
//		+ std::powf( r_lab[1] - l_lab[1], 2 )
//		+ std::powf( r_lab[2] - l_lab[2], 2 );
//}


__device__ float gamma_expanded( const float u );
//{
//	return u > 0.04045 ? pow( (u + 0.055) / 1.055, 2.4 ) : (u / 12.92);
//}


__device__ void convert_rgb8_to_xyz( const uint8_t* rgb, float& x, float& y, float& z );
//{
//	// RGBの値を0.0~1.0に
//	float r = rgb[0] / 255.0f;
//	float g = rgb[1] / 255.0f;
//	float b = rgb[2] / 255.0f;
//
//	// ガンマ補正の除去
//	r = gamma_expanded( r );
//	g = gamma_expanded( g );
//	b = gamma_expanded( b );
//
//	// XYZに変換
//	x = 0.4124 * r + 0.3576 * g + 0.1805 * b;
//	y = 0.2126 * r + 0.7152 * g + 0.0722 * b;
//	z = 0.0193 * r + 0.1192 * g + 0.9505 * b;
//
//	x *= 100;
//	y *= 100;
//	z *= 100;
//}

__global__ void kernel_convert_rgb8_to_lab( const uint8_t* rgb_in, float* lab_out, const uint16_t width, const uint16_t height );
//{
//	int x = blockIdx.x * blockDim.x + threadIdx.x;
//	int y = blockIdx.y * blockDim.y + threadIdx.y;
//
//	// 式温度6,500Kの昼光色を想定した標準光源D65のXYZ値
//	float xn = 95.05f, yn = 100.0f, zn = 108.91f;
//
//	// xyzの値の補正　pow( 24/116, 3 ) = 0.008856, 1 / ( 3*pow(24/116) ) = 7.787037
//	auto f = []( float val )
//	{
//		return val > 0.008856 ? std::powf( val, 1.0f / 3.0f ) : (7.787037 * val) + 0.137931;
//	};
//
//	int idx = y * width + x;
//
//	if ( x < width && y < height )
//	{
//		// RGB,L*a*bそれぞれの編集位置のポインタ
//		const uint8_t* p_rgb = rgb_in + idx * 3;
//		float* p_lab = lab_out + idx * 3;
//
//		// sRGBからCIE XYZへ変換
//		float x, y, z;
//		convert_rgb8_to_xyz( p_rgb, x, y, z );
//
//		float fx = f( x / xn );
//		float fy = f( y / yn );
//		float fz = f( z / zn );
//
//		p_lab[0] = 116 * fy - 16;
//		p_lab[1] = 500 * (fx - fy);
//		p_lab[2] = 200 * (fy - fz);
//
//	}
//
//}

void __global__ kernel_hole_filter_with_color( uint16_t* depth_out, const uint16_t* depth_in, const float* lab_in, const int kernel_w, const int width, const int height );
//{
//	const int x = blockIdx.x * blockDim.x + threadIdx.x;
//	const int y = blockIdx.y * blockDim.y + threadIdx.y;
//	const int idx = y * width + x;
//
//	if ( x < width - kernel_w && y < height - kernel_w && x > kernel_w && y > kernel_w )
//	{
//		// 処理範囲
//		const int image_kernel_size = kernel_w * 2 + 1;
//
//		int size = kernel_w * 2 + 1;
//		//float* lab_dists = new float[size*size];
//		//float* sqrt_lab_dists = new float[size*size];
//
//		// 先に色距離を配列に代入 
//		/*for ( int n = -kernel_w; n <= kernel_w; ++n )
//		{
//			for ( int m = -kernel_w; m <= kernel_w; ++m )
//			{
//				int source = (y + n) * width + (x + m);
//				float lab_dist = lab_distance( &lab_in[idx * 3], &lab_in[source * 3] );
//
//				int um = m + kernel_w;
//				int un = n + kernel_w;
//				lab_dists[un*size + um] = lab_dist;
//				sqrt_lab_dists[un*size + um] = lab_dist * lab_dist * 3;
//			}
//		}*/
//
//		// 色距離のシグマ値は標準偏差
//		const float sqr_color_sigma = 5;// calc_dispersion( lab_dists, size * size );
//
//		//delete[] lab_dists;
//		//delete[] sqrt_lab_dists;
//
//		// 標準偏差0は打ち切り
//		if ( sqr_color_sigma == 0 )
//		{
//			return;
//		}
//
//		float denominator = 0;
//		float numerator = 0;
//
//		int black_counter = 0;
//		// 値計算
//		for ( int n = -kernel_w; n <= kernel_w; ++n )
//		{
//			for ( int m = -kernel_w; m <= kernel_w; ++m )
//			{
//				const int source = (y + n) * width + (x + m);
//				const int k = (n + kernel_w) * size + (m + kernel_w);
//
//				// 自分は含めない
//				if ( n == 0 && m == 0 )
//				{
//					continue;
//				}
//
//				// 深度0は無視する
//				if ( depth_in[source] == 0 )
//				{
//					continue;
//				}
//
//				float lab_dist = lab_distance( &lab_in[idx * 3], &lab_in[source * 3] );
//				if ( lab_dist > 5 ) continue;
//				const float P = coef_P5[k];
//				const float N = std::exp( -lab_dist / (2 * sqr_color_sigma) );
//
//				numerator += depth_in[source] * P * N;
//				denominator += P * N;
//			}
//		}
//
//		if ( numerator != 0 && denominator != 0 )
//		{
//			depth_out[idx] = (uint16_t)(numerator / denominator);
//		}
//
//
//		/*depth_out[idx] =
//			coef_P[0][0] * depth_in[(y - 1)*width + x - 1] +
//			coef_P[0][1] * depth_in[(y - 1)*width + x] +
//			coef_P[0][2] * depth_in[(y - 1)*width + x + 1] +
//			coef_P[1][0] * depth_in[(y)*width + x - 1] +
//			coef_P[1][1] * depth_in[(y)*width + x] +
//			coef_P[1][2] * depth_in[(y)*width + x + 1] +
//			coef_P[2][0] * depth_in[(y + 1)*width + x - 1] +
//			coef_P[2][1] * depth_in[(y + 1)*width + x] +
//			coef_P[2][2] * depth_in[(y + 1)*width + x + 1];*/
//
//	}
//}

void force_flattening_filter_cuda_helper::convert_rgb8_to_lab( const uint8_t* h_rgb_in, float* h_lab_out, const uint16_t width, const uint16_t height )
{
	// 配列の要素数とバイト数
	const int size = width * height * 3;
	const int labBytes = sizeof( float ) * size;
	const int rgbBytes = sizeof( uint8_t ) * size;

	// デバイスメモリの確保、ホストからの転送
	if ( !_d_rgb_in ) { _d_rgb_in = alloc_dev<uint8_t>( size ); }
	cudaMemcpy( _d_rgb_in.get(), h_rgb_in, rgbBytes, cudaMemcpyHostToDevice );

	// デバイスメモリの確保、初期化
	if ( !_d_lab_out ) { _d_lab_out = alloc_dev<float>( size ); }
	cudaMemset( _d_lab_out.get(), 0, labBytes );

	// カーネルのスレッド数設定
	const int threadSize = CUDA_THREAD_SIZE;
	dim3 threads( threadSize, threadSize );
	dim3 blocks( (width + threadSize - 1) / threadSize, (height + threadSize - 1) / threadSize );

	// カーネル呼び出し
	kernel_convert_rgb8_to_lab << <blocks, threads >> > (_d_rgb_in.get(), _d_lab_out.get(), width, height);

	// CUDA同期待ち
	cudaDeviceSynchronize();

	// デバイスメモリからホストメモリへ転送
	cudaMemcpy( h_lab_out, _d_lab_out.get(), labBytes, cudaMemcpyDeviceToHost );
}

void librealsense::force_flattening_filter_cuda_helper::hole_filter_process( uint16_t* h_depth_out, const uint16_t* h_depth_in, const float* h_lab_in, const int kernel_w, const int width, const int height )
{
	// 配列の要素数とバイト数
	const int depthSize = width * height;
	const int depthBytes = sizeof( uint16_t ) * depthSize;
	const int labSize = width * height * 3;
	const int labBytes = sizeof( float ) * labSize;

	// デバイスメモリの確保、ホストからの転送
	if ( !_d_depth_in ) { _d_depth_in = alloc_dev<uint16_t>( depthSize ); }
	cudaMemcpy( _d_depth_in.get(), h_depth_in, depthBytes, cudaMemcpyHostToDevice );

	// デバイスメモリの確保、ホストからの転送
	if ( !_d_lab_in ) { _d_lab_in = alloc_dev<float>( labSize ); }
	cudaMemcpy( _d_lab_in.get(), h_lab_in, labBytes, cudaMemcpyHostToDevice );

	// デバイスメモリの確保、初期化
	if ( !_d_depth_out ) { _d_depth_out = alloc_dev<uint16_t>( depthSize ); }
	//cudaMemset( _d_depth_out.get(), 0, depthBytes );
	cudaMemcpy( _d_depth_out.get(), h_depth_in, depthBytes, cudaMemcpyHostToDevice );

	// P値の配列を作成
	if ( _h_coef_P5[0] == 0 ) { initializeCoef(); }
	int kernel_size = kernel_w * 2 + 1;

	// カーネルのスレッド数設定
	const int threadSize = CUDA_THREAD_SIZE;
	dim3 threads( threadSize, threadSize );
	dim3 blocks( (width + threadSize - 1) / threadSize, (height + threadSize - 1) / threadSize );

	const int roop_time = 6;
	for ( int i = 0; i < roop_time; ++i )
	{
		kernel_hole_filter_with_color << <blocks, threads >> > (_d_depth_out.get(), _d_depth_in.get(), _d_lab_in.get(), kernel_w, width, height);

		// CUDA同期待ち
		cudaDeviceSynchronize();

		// 入力データの更新
		cudaMemcpy( _d_depth_in.get(), _d_depth_out.get(), depthBytes, cudaMemcpyDeviceToDevice );
	}

	// カーネルのエラーチェック
	CHECK( cudaGetLastError() );

	// デバイスメモリからホストメモリへ転送
	cudaMemcpy( h_depth_out, _d_depth_out.get(), depthBytes, cudaMemcpyDeviceToHost );
}

// P値の係数(空間距離)の初期化
void librealsense::force_flattening_filter_cuda_helper::initializeCoef()
{
	const int kernel_w = 5;
	const float sqr_space_sigma = 10;
	int size = kernel_w * 2 + 1;
	for ( int n = -kernel_w; n <= kernel_w; ++n )
	{
		for ( int m = -kernel_w; m <= kernel_w; ++m )
		{
			float space_distance = n * n + m * m;
			float P = std::exp( -space_distance / (2.0f * sqr_space_sigma) );
			int um = m + kernel_w;
			int un = n + kernel_w;
			_h_coef_P5[un * size + um] = P;
		}
	}

	// コンスタントメモリへのアップロード
	for ( int i = 0; i < 11; ++i )
	{
		cudaMemcpyToSymbol( coef_P5, _h_coef_P5, 11 * 11 * sizeof( float ), cudaMemcpyHostToDevice );
	}
}


#endif // RS2_USE_CUDA
