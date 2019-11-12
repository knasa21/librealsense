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
	//timer_start();
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

	if ( !_tgt_depth )
	{
		_tgt_depth = make_shared<rs2::frame>( source.allocate_video_frame(
			src_depth.get_profile(),
			src_depth,
			src_depth.get_bytes_per_pixel(),
			width,
			height,
			src_depth.get_stride_in_bytes(),
			RS2_EXTENSION_DEPTH_FRAME
		) );

		reset_cache();
	}

	/*auto tgt_depth = source.allocate_video_frame(
		src_depth.get_profile(),
		src_depth,
		src_depth.get_bytes_per_pixel(),
		width,
		height,
		src_depth.get_stride_in_bytes(),
		RS2_EXTENSION_DEPTH_FRAME
	);*/

	//timer_end("init");

	if ( _tgt_depth )
	{
		//timer_start();
		auto tgt_depth_ptr = dynamic_cast<librealsense::depth_frame*>( ( librealsense::frame_interface* )_tgt_depth->get() );
		auto orig_depth_ptr = dynamic_cast<librealsense::depth_frame*>( ( librealsense::frame_interface* )src_depth.get() );
		auto orig_color_ptr = dynamic_cast<librealsense::video_frame*>( ( librealsense::frame_interface* )src_color.get() );

		auto depth_data = (uint16_t*)orig_depth_ptr->get_frame_data();
		auto new_data = (uint16_t*)tgt_depth_ptr->get_frame_data();

		memset( new_data, 0, size * sizeof( uint16_t ) );

		tgt_depth_ptr->set_sensor( orig_depth_ptr->get_sensor() );
		auto du = orig_depth_ptr->get_units();

		auto rgb_data = (uint8_t*)orig_color_ptr->get_frame_data();
		//float* lab_data = new float[size*3];
		if ( !_lab_data ) { _lab_data.reset( new float[size * 3] ); }

		timer_start();
		convert_rgb8_to_lab( rgb_data, _lab_data.get(), width, height );
		timer_end( "convert_rgb8_to_lab" );

		const int kernel_w = 5;
		const int kernel_size = 2 * kernel_w + 1;
		//float kernel_lab_l[kernel_size * kernel_size] ={ 0 };
		//float kernel_lab_a[kernel_size * kernel_size] ={ 0 };
		//float kernel_lab_b[kernel_size * kernel_size] ={ 0 };
		uint16_t kernel_depth[kernel_size * kernel_size] ={ 0 };

		//timer_end("init 2");

		timer_start();
		hole_filter_process( new_data, depth_data, _lab_data.get(), kernel_w, width, height );
		/*hole_filter_process( new_data, new_data, _lab_data.get(), kernel_w, width, height );
		hole_filter_process( new_data, new_data, _lab_data.get(), kernel_w, width, height );
		hole_filter_process( new_data, new_data, _lab_data.get(), kernel_w, width, height );
		hole_filter_process( new_data, new_data, _lab_data.get(), kernel_w, width, height );
		hole_filter_process( new_data, new_data, _lab_data.get(), kernel_w, width, height );*/
		timer_end("filter roop 1");

		//delete[] lab_data;

		output_frames.push_back( *_tgt_depth );
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

void hole_filter_with_color::convert_rgb8_to_lab( const uint8_t* rgb, float* lab, const uint16_t width, const uint16_t height )
{
	int size = width * height;
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
	return
		std::powf( r_lab[0] - l_lab[0], 2 )
		+	std::powf( r_lab[1] - l_lab[1], 2 )
		+	std::powf( r_lab[2] - l_lab[2], 2 );
}

void hole_filter_with_color::hole_filter_process( uint16_t * new_depth_image, const uint16_t * depth_image, const float * lab_image, const int kernel_w, const int width, const int height )
{
		for ( int y = kernel_w; y < height - kernel_w; ++y )
		{
			for ( int x = kernel_w; x < width - kernel_w; ++x )
			{
				int i = y * width + x;
				new_depth_image[i] = depth_image[i];
				//if ( 380 < x && x < 470 && 180 < y && y < 300 )
				{
					kernel_process( new_depth_image[i], depth_image, lab_image, kernel_w, x, y );
				}
			}
		}
}

void hole_filter_with_color::kernel_process( uint16_t& new_depth, const uint16_t* depth_image, const float* lab_image, const int kernel_w, const int x, const int y )
{
	int size = kernel_w * 2 + 1;
	const float& sqr_space_sigma = _sqr_space_sigma_array[kernel_w-1];

	int target = y * 848 + x;

	static std::vector<float> P_vec( size*size );

	// P値の配列作成
	// P値はサイズによって固定
	if ( P_vec[0] == 0 )
	{
		for ( int n = -kernel_w; n <= kernel_w; ++n )
		{
			for ( int m = -kernel_w; m <= kernel_w; ++m )
			{
				float space_distance = n * n + m * m;
				float P = std::exp( -space_distance / ( 2.0f * sqr_space_sigma ) );
				int um = m + kernel_w;
				int un = n + kernel_w;
				P_vec[un*size + um] = P;
			}
		}
	}


	//std::vector<float> lab_dists( size*size );
	//std::vector<float> sqrt_lab_dists( size*size );

	//// 先に色距離を配列に代入 
	//for ( int n = -kernel_w; n <= kernel_w; ++n )
	//{
	//	for ( int m = -kernel_w; m <= kernel_w; ++m )
	//	{
	//		int source = ( y + n ) * 848 + ( x + m );
	//		float lab_dist = lab_distance( &lab_image[target * 3], &lab_image[source * 3] );

	//		int um = m + kernel_w;
	//		int un = n + kernel_w;
	//		lab_dists[un*size + um] = lab_dist;
	//		sqrt_lab_dists[un*size + um] =  lab_dist * lab_dist * 3;
	//	}
	//}

	// 色距離のシグマ値は標準偏差
	const float sqr_color_sigma = 30; // calc_dispersion( lab_dists );

	// 標準偏差0は打ち切り
	if ( sqr_color_sigma == 0 )
	{
		return;
	}

	float denominator = 0;
	float numerator = 0;

	int black_counter = 0;
	// 値計算
	for ( int n = -kernel_w; n <= kernel_w; ++n )
	{
		for ( int m = -kernel_w; m <= kernel_w; ++m )
		{
			const int source = ( y + n ) * 848 + ( x + m );
			const int k = ( n + kernel_w ) * size + ( m + kernel_w );

			// 自分は含めない
			if ( n == 0 && m == 0 )
			{
				continue;
			}

			// 深度0は無視する
			if ( depth_image[source] == 0 )
			{
				continue;
			}

			const float& P = P_vec[k];
			float lab_dist = lab_distance( &lab_image[target * 3], &lab_image[source * 3] );
			if ( lab_dist > 5 ) continue;
			//const float N = std::exp( -sqrt_lab_dists[k] / (2 * sqr_color_sigma) );
			const float N = std::exp( - lab_dist / (2 * sqr_color_sigma) );

			numerator   += depth_image[source] * P * N;
			denominator += P * N;
		}
	}

	if ( numerator != 0 && denominator != 0 )
	{
		new_depth = numerator / denominator;
	}
}

float hole_filter_with_color::calc_dispersion( const std::vector<float>& vals )
{
	float sum = 0;
	// 非0値の数
	float count = 0;


	for ( const auto& val : vals )
	{
		sum += val;
		if ( val != 0 )
		{
			++count;
		}
	}

	// 0しかないとき0を返す 
	if ( count == 0 )
	{
		return 0;
	}

	float mean = sum / count;
	sum = 0;

	for ( const auto& val : vals )
	{
		if ( val != 0 )
		{
			sum += std::powf( mean - val, 2 );
		}
	}


	return sum / count;
}

void hole_filter_with_color::timer_start()
{
	_chrono_start = chrono::system_clock::now();
}

void hole_filter_with_color::timer_end( const string& text )
{
	_chrono_end = chrono::system_clock::now();

	double time = static_cast<double>( chrono::duration_cast<chrono::microseconds>
		( _chrono_end - _chrono_start ).count() / 1000.0 );

	cout << "[ " << text << " } : " << time << " ms " << endl;
}



}