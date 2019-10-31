#include "hole-filter-with-color.h"

#include "api.h"

// ==============================
// rs.cpp
// ==============================

// ==============================
// hole-filter-with-color.cpp 
// ==============================

namespace librealsense
{

/**
* フィルターが適用できるか
*/
bool hole_filter_with_color::should_process( const rs2::frame& frame )
{
	if ( !frame )
	{
		return false;
	}

	auto set = frame.as<rs2::frameset>();
	if ( !set )
	{
		return false;
	}

	auto profile = frame.get_profile();
	rs2_stream stream = profile.stream_type();
	rs2_format format = profile.format();
	int index = profile.stream_index();

	// 深度とカラー両方のデータが存在するときに処理を行う
	bool has_color = false, has_depth = false;
	set.foreach( [&has_color]( const rs2::frame& frame )
		{
			if ( frame.get_profile().stream_type() == RS2_STREAM_COLOR )
			{
				has_color = true;
			}
		} );
	set.foreach( [&has_depth]( const rs2::frame& frame )
		{
			if ( frame.get_profile().stream_type() == RS2_STREAM_DEPTH
				&& frame.get_profile().format() == RS2_FORMAT_Z16 )
			{
				has_depth = true;
			}
		} );

	if ( !has_color || !has_depth )
	{
		return false;
	}

	return true;
}

rs2::frame hole_filter_with_color::process_frame( const rs2::frame_source& source, const rs2::frame& f )
{
	std::vector<rs2::frame> output_frames;
	std::vector<rs2::frame> other_frames;

	auto frames = f.as<rs2::frameset>();
	auto src_depth = frames.first_or_default( RS2_STREAM_DEPTH, RS2_FORMAT_Z16 ).as<rs2::depth_frame>();
	auto src_color = frames.first_or_default( RS2_STREAM_COLOR, RS2_FORMAT_RGB8 ).as<rs2::video_frame>();


	// ここに処理を書く
	int depth_w = src_depth.get_width();
	int depth_h = src_depth.get_height();

	int color_w = src_color.get_width();
	int color_h = src_color.get_height();

	// サイズが同じ前提なので異なる場合は処理をしない
	if ( depth_w != color_w || depth_h != color_h )
	{
		return f;
	}

	int width = depth_w;
	int height = depth_h;
	int size = width * height;


	auto tgt_depth = source.allocate_video_frame(
		src_depth.get_profile(),
		src_depth,
		src_depth.get_bytes_per_pixel(),
		width,
		height,
		src_depth.get_stride_in_bytes(),
		RS2_EXTENSION_DEPTH_FRAME
	);

	if ( tgt_depth )
	{
		auto tgt_depth_ptr = dynamic_cast<librealsense::depth_frame*>( ( librealsense::frame_interface* )tgt_depth.get() );
		auto orig_depth_ptr = dynamic_cast<librealsense::depth_frame*>( ( librealsense::frame_interface* )src_depth.get() );
		auto orig_color_ptr = dynamic_cast<librealsense::video_frame*>( ( librealsense::frame_interface* )src_color.get() );

		auto depth_data = (uint16_t*)orig_depth_ptr->get_frame_data();
		auto new_data = (uint16_t*)tgt_depth_ptr->get_frame_data();

		memset( new_data, 0, size * sizeof( uint16_t ) );

		tgt_depth_ptr->set_sensor( orig_depth_ptr->get_sensor() );
		auto du = orig_depth_ptr->get_units();

		auto rgb_data = (uint8_t*)orig_color_ptr->get_frame_data();
		float* lab_data = new float[size*3];

		convert_rgb8_to_lab( rgb_data, lab_data, size );

		const int kernel_w = 3;
		const int kernel_size = 2 * kernel_w + 1;
		float kernel_lab_l[kernel_size * kernel_size] ={ 0 };
		float kernel_lab_a[kernel_size * kernel_size] ={ 0 };
		float kernel_lab_b[kernel_size * kernel_size] ={ 0 };
		uint16_t kernel_depth[kernel_size * kernel_size] ={ 0 };

		const float space_sigma = 3.3f;
		const float color_sigma = 90.0f;

		//for ( int y = 180; y < 300; ++y )
		//{
			//for ( int x = 380; x < 470; ++x )
			//{

		const int repeat_time = 5;
		//for ( int lap = 0; lap < repeat_time; ++lap )
		//{

			for ( int y = 1; y < height - 1; ++y )
			{
				for ( int x = 1; x < width - 1; ++x )
				{
					int i = y * width + x;
					if ( y < 180 || y > 400 || x < 380 || x > 470 )
					{
						new_data[i] = depth_data[i];
						continue;
					}

					uint8_t* rgb;
					rgb = rgb_data + ( y * width + x ) * 3;
					float* lab;
					lab = lab_data + ( y * width + x ) * 3;

					float non_zero_counter = 0;
					for ( int ky = -kernel_w; ky <= kernel_w; ++ky )
					{
						for ( int kx = -kernel_w; kx <= kernel_w; ++kx )
						{
							int k = ( y + ky ) * width + ( x + kx );
							//auto dist = du * depth_data[k];
							auto dist = depth_data[k];

							//int mirror = ( width - x ) + width * y;
							if ( dist != 0 )
							{
								int kernel_i = ( ky + 1 ) * kernel_size + ( kx + 1 );
								kernel_depth[kernel_i] = dist;
								kernel_lab_l[kernel_i] = lab_data[k*3];
								kernel_lab_a[kernel_i] = lab_data[k*3+1];
								kernel_lab_b[kernel_i] = lab_data[k*3+2];
								++non_zero_counter;
							}
						}
					}

					if ( non_zero_counter > 5 )
					{
						// 分母
						float sum_deno = 0;
						// 分子
						float sum_nume = 0;
						for ( int k = 0; k < kernel_size * kernel_size; ++k )
						{
							if ( kernel_depth[k] == 0 )
							{
								continue;
							}
							int x = k % kernel_size;
							int y = k / kernel_size;

							float P = std::exp( -( x * x + y * y ) / ( 2 * space_sigma * space_sigma ) );
							float N =
								std::exp(
									-std::sqrtf(
										std::powf( lab[0] - kernel_lab_l[k], 2.0f ) +
										std::powf( lab[1] - kernel_lab_a[k], 2.0f ) +
										std::powf( lab[2] - kernel_lab_b[k], 2.0f )
									) /
									( 2 * color_sigma * color_sigma )
								);

							sum_deno += kernel_depth[k] * P * N;
							sum_nume += P * N;
						}
						uint16_t n = sum_deno / sum_nume;
						new_data[i] = n;
					}

					memset( kernel_depth, 0, sizeof( uint16_t ) * kernel_size * kernel_size );
					memset( kernel_lab_l, 0, sizeof( kernel_lab_l ) );
					memset( kernel_lab_a, 0, sizeof( kernel_lab_a ) );
					memset( kernel_lab_b, 0, sizeof( kernel_lab_b ) );
					//new_data[i] = sum / ( kernel_size * kernel_size );
				}
			}
		//}


		delete[] lab_data;

		output_frames.push_back( tgt_depth );
		output_frames.push_back( src_color );

		// フレームをまとめて、返す
		auto new_composite = source.allocate_composite_frame( std::move( output_frames ) );
		return new_composite;
	}

	return f;
}

rs2::frame hole_filter_with_color::prepare_target_frame( const rs2::frame& f, const rs2::frame_source& source, rs2_extension tgt_type )
{
	auto vf = f.as<rs2::video_frame>();
	//auto ref = source.allocate_video_frame( )

	return vf;
}

void hole_filter_with_color::convert_rgb8_to_lab( const uint8_t* rgb, float* lab, const uint32_t size )
{
	// 式温度6,500Kの昼光色を想定した標準光源D65のXYZ値
	float xn = 95.05f, yn = 100.0f, zn = 108.91f;

	// xyzの値の補正　pow( 24/116, 3 ) = 0.008856, 1 / ( 3*pow(24/116) ) = 7.787037
	auto f = []( float val )
	{
		return val > 0.008856 ? std::powf( val, 1.0f / 3.0f ) : ( 7.787037 * val ) + 0.137931;
	};

	for ( int i = 0; i < size; ++i )
	{
		// RGB,L*a*bそれぞれの編集位置のポインタ
		const uint8_t* p_rgb = rgb +i * 3;
		float* p_lab = lab + i * 3;

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

void hole_filter_with_color::convert_rgb8_to_xyz( const uint8_t* rgb, float& x, float& y, float& z )
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

float hole_filter_with_color::gamma_expanded( const float u )
{
	return u > 0.04045 ? pow( ( u+0.055 )/1.055, 2.4 ) : ( u / 12.92 );
}

float hole_filter_with_color::lab_distance( const float* r_lab, const float* l_lab )
{
	return std::powf( r_lab[0] - l_lab[0], 2 )
		+  std::powf( r_lab[1] - l_lab[1], 2 )
		+  std::powf( r_lab[2] - l_lab[2], 2 );
}

}