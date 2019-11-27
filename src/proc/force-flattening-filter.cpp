#include "force-flattening-filter.h"

#include "api.h"
#include <random>

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
bool force_flattening_filter::should_process( const rs2::frame& frame )
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

rs2::frame force_flattening_filter::process_frame( const rs2::frame_source& source, const rs2::frame& f )
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

	_width = depth_w;
	_height = depth_h;
	int size = _width * _height;

	if ( !_tgt_depth )
	{
		_tgt_depth = std::make_shared<rs2::frame>( source.allocate_video_frame(
			src_depth.get_profile(),
			src_depth,
			src_depth.get_bytes_per_pixel(),
			_width,
			_height,
			src_depth.get_stride_in_bytes(),
			RS2_EXTENSION_DEPTH_FRAME
		) );

		reset_cache();
	}

	if ( !_labels )
	{
		// ラベルの初期化
		_labels = std::make_unique<uint32_t[]>( _width * _height );
	}
	if ( !_lab_distances )
	{
		// 色距離配列の初期化
		_lab_distances = std::make_unique<float[]>( _width * _height * 2 );
	}

	// ラベル配列の0埋め
	memset( _labels.get(), 0, sizeof( uint32_t ) * _width * _height );

	//timer_end("init");

	if ( _tgt_depth )
	{
		//timer_start();
		auto tgt_depth_ptr = dynamic_cast<librealsense::depth_frame*>((librealsense::frame_interface*)_tgt_depth->get());
		auto orig_depth_ptr = dynamic_cast<librealsense::depth_frame*>((librealsense::frame_interface*)src_depth.get());
		auto orig_color_ptr = dynamic_cast<librealsense::video_frame*>((librealsense::frame_interface*)src_color.get());

		auto depth_data = (uint16_t*)orig_depth_ptr->get_frame_data();
		auto new_data = (uint16_t*)tgt_depth_ptr->get_frame_data();

		memset( new_data, 0, size * sizeof( uint16_t ) );

		tgt_depth_ptr->set_sensor( orig_depth_ptr->get_sensor() );
		auto du = orig_depth_ptr->get_units();

		auto rgb_data = (uint8_t*)orig_color_ptr->get_frame_data();
		//float* lab_data = new float[size*3];
		if ( !_lab_data ) { _lab_data.reset( new float[size * 3] ); }

		timer_start();
		convert_rgb8_to_lab( rgb_data, _lab_data.get(), _width, _height );
		timer_end( "convert_rgb8_to_lab" );

		//timer_start();
		// 並列化すれば意味があるはず
		//make_lab_distances( _lab_data.get(), _lab_distances.get(), _width, _height );
		//timer_end( "make_lab_distance" );

		const int kernel_w = 5;
		const int kernel_size = 2 * kernel_w + 1;
		uint16_t kernel_depth[kernel_size * kernel_size] = { 0 };

		//timer_end("init 2");

		// クラスタリストの初期化
		_clusters.clear();

		timer_start();
		hole_filter_process( new_data, depth_data, _lab_data.get(), kernel_w, _width, _height );
		timer_end( "filter roop 1" );

		timer_start();
		for ( const auto& cluster : _clusters )
		{
			int r = rand_range( 0, 255 );
			int g = rand_range( 0, 255 );
			int b = rand_range( 0, 255 );
			for ( const auto& index : cluster )
			{
				rgb_data[index * 3 + 0] = r;
				rgb_data[index * 3 + 1] = g;
				rgb_data[index * 3 + 2] = b;
			}
		}
		timer_end( "color setting" );

		flattening( depth_data, new_data, _clusters );

		output_frames.push_back( *_tgt_depth );
		output_frames.push_back( src_color );

		// フレームをまとめて、返す
		auto new_composite = source.allocate_composite_frame( std::move( output_frames ) );
		return new_composite;
	}

	return f;
}

rs2::frame force_flattening_filter::prepare_target_frame( const rs2::frame& f, const rs2::frame_source& source, rs2_extension tgt_type )
{
	auto vf = f.as<rs2::video_frame>();
	//auto ref = source.allocate_video_frame( )

	return vf;
}

void force_flattening_filter::convert_rgb8_to_lab( const uint8_t* rgb, float* lab, const uint16_t width, const uint16_t height )
{
	int size = width * height;
	// 式温度6,500Kの昼光色を想定した標準光源D65のXYZ値
	float xn = 95.05f, yn = 100.0f, zn = 108.91f;

	// xyzの値の補正　pow( 24/116, 3 ) = 0.008856, 1 / ( 3*pow(24/116) ) = 7.787037
	auto f = []( float val )
	{
		return val > 0.008856 ? std::powf( val, 1.0f / 3.0f ) : (7.787037 * val) + 0.137931;
	};

	for ( int i = 0; i < size; ++i )
	{
		// RGB,L*a*bそれぞれの編集位置のポインタ
		const uint8_t* p_rgb = rgb + i * 3;
		float* p_lab = lab + i * 3;

		// sRGBからCIE XYZへ変換
		float x, y, z;
		convert_rgb8_to_xyz( p_rgb, x, y, z );

		float fx = f( x / xn );
		float fy = f( y / yn );
		float fz = f( z / zn );

		p_lab[0] = 116 * fy - 16;
		p_lab[1] = 500 * (fx - fy);
		p_lab[2] = 200 * (fy - fz);

	}
}

void force_flattening_filter::convert_rgb8_to_xyz( const uint8_t* rgb, float& x, float& y, float& z )
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

float force_flattening_filter::gamma_expanded( const float u )
{
	return u > 0.04045 ? pow( (u + 0.055) / 1.055, 2.4 ) : (u / 12.92);
}

float force_flattening_filter::lab_distance( const float* r_lab, const float* l_lab )
{
	return
		std::powf( r_lab[0] - l_lab[0], 2 )
		+ std::powf( r_lab[1] - l_lab[1], 2 )
		+ std::powf( r_lab[2] - l_lab[2], 2 );
}

void force_flattening_filter::hole_filter_process( uint16_t * new_depth_image, const uint16_t * depth_image, const float * lab_image, const int kernel_w, const int width, const int height )
{
	for ( int y = 0; y < _height; ++y )
	{
		for ( int x = 0; x < _width; ++x )
		{
			int index = y * _width + x;
			uint16_t depth = depth_image[index];
			if ( depth > 1500 )
			{
				_labels[index] = -1;
			}
		}
	}

	int label_id = 1;
	int size = _width * _height;
	for ( int i = 0; i < size; ++i )
	{
		if ( _labels[i] == 0 )
		{
			std::vector<uint32_t> labeled_indices;
			int cluster_size = labeling_process( lab_image, labeled_indices, label_id, i, 10.f );
			if ( cluster_size > 300 )
			{
				_clusters.push_back( labeled_indices );
			}
			else {
				labeled_indices.clear();
				_clusters.push_back( labeled_indices );
			}
			++label_id;
		}
	}
}

int force_flattening_filter::labeling_process( const float* lab_image, std::vector<uint32_t>& labeled_indices, const uint32_t label_id, const uint32_t start_index, const float threshold )
{
	// 探索リスト
	std::queue<uint32_t> search_q;
	// 最初のインデックス
	search_q.push( start_index );
	// ラベル付け
	_labels[start_index] = label_id;
	// リストに追加
	labeled_indices.push_back( start_index );

	// リストが空になるまで探索
	while ( !search_q.empty() )
	{
		const auto& index = search_q.front();
		search_q.pop();

		// 上
		int up = index - _width;
		// 外に出ていない、ラベルなし
		if ( index >= _width &&
			_labels[up] == 0 )
		{
			// 距離が閾値以内
			if ( lab_distance( &lab_image[index * 3], &lab_image[up * 3] ) < threshold )
				//if ( _lab_distances[up * 2 + 1] < threshold )
			{
				// ラベルを付ける
				_labels[up] = label_id;
				// リストに追加
				labeled_indices.push_back( up );
				// 探索リストに追加
				search_q.push( up );
			}

		}


		// 右
		int right = index + 1;
		// 外に出ていない、ラベルなし
		if ( right % _width != 0 &&
			_labels[right] == 0 )
		{
			// 距離が閾値以内
			if ( lab_distance( &lab_image[index * 3], &lab_image[right * 3] ) < threshold )
				//if ( _lab_distances[index * 2] < threshold )
			{
				// ラベルを付ける
				_labels[right] = label_id;
				// リストに追加
				labeled_indices.push_back( right );
				// 探索リストに追加
				search_q.push( right );
			}

		}

		// 下
		int down = index + _width;
		// 外に出ていない、ラベルなし
		if ( down < (_width * _height) &&
			_labels[down] == 0 )
		{
			// 距離が閾値以内
			if ( lab_distance( &lab_image[index * 3], &lab_image[down * 3] ) < threshold )
				//if ( _lab_distances[index * 2 + 1] < threshold )
			{
				// ラベルを付ける
				_labels[down] = label_id;
				// リストに追加
				labeled_indices.push_back( down );
				// 探索リストに追加
				search_q.push( down );
			}
		}

		// 左
		int left = index - 1;
		// 外に出ていない、ラベルなし
		if ( index % _width != 0 &&
			_labels[left] == 0 )
		{
			// 距離が閾値以内
			if ( lab_distance( &lab_image[index * 3], &lab_image[left * 3] ) < threshold )
				//if ( _lab_distances[left * 2] < threshold )
			{
				// ラベルを付ける
				_labels[left] = label_id;
				// リストに追加
				labeled_indices.push_back( left );
				// 探索リストに追加
				search_q.push( left );
			}
		}

	}

	return labeled_indices.size();
}

void force_flattening_filter::flattening( const uint16_t* depth_image, uint16_t* flat_depth_image, std::vector<std::vector<uint32_t>>& clusters )
{
	for ( int y = 0; y < _height; ++y )
	{
		for ( int x = 0; x < _width; ++x )
		{
			int index = y * _width + x;
			flat_depth_image[index] = depth_image[index];
		}
	}

	for ( const auto& indices : clusters )
	{
		if ( indices.size() == 0 ) continue;
		// 係数
		std::vector<float> coefficients;
		ransac_plane( depth_image, indices, coefficients );

		if ( coefficients.size() == 4 )
		{
			/*std::cout << "coef : " <<
				coefficients[0] << ", " <<
				coefficients[1] << ", " <<
				coefficients[2] << ", " <<
				coefficients[3] << std::endl;*/

			// 平面化
			for ( const auto& index : indices )
			{
				int x = index % _width;
				int y = index / _width;

				flat_depth_image[index] = (coefficients[0] * x + coefficients[1] * y + coefficients[3]) / -coefficients[2];
			}
		}
	}
}

void force_flattening_filter::ransac_plane( const uint16_t* depth_image, const std::vector<uint32_t>& indices, std::vector<float>& coefficients )
{
	// 試行回数
	const int times = 50;
	// 一致とみなす閾値(mm)
	const float threshold = 5;

	// 深度データを持つインデックス配列
	std::vector<const uint32_t*> available_indices;

	// 有効な値を持つ点を保存
	for ( const uint32_t& index : indices )
	{
		if ( depth_image[index] > 100 )
		{
			available_indices.push_back( &index );
		}
	}

	int available_size = available_indices.size();

	// 有効な点が三点以上ない場合は終了
	if ( available_size < 3 ) {
		return;
	}

	// 最大の一致数
	float max_match_count = 0;
	// 一致数が最も多い平面の係数
	float max_a, max_b, max_c, max_d;

	for ( int i = 0; i < times; ++i )
	{
		auto ai = 0;
		auto bi = 0;
		auto ci = 0;

		// 有効な値を持つ点のインデックス
		uint32_t ava_index_a = 0;
		uint32_t ava_index_b = 0;
		uint32_t ava_index_c = 0;

		// 被らないようにランダムで決める
		while ( ava_index_a == ava_index_b ||
				ava_index_b == ava_index_c || 
				ava_index_c == ava_index_a )
		{
			ava_index_a = rand_range( 0, available_size - 1 );
			ava_index_b = rand_range( 0, available_size - 1 );
			ava_index_c = rand_range( 0, available_size - 1 );
		}

		// 使用する点のインデックス
		ai = *available_indices[ava_index_a];
		bi = *available_indices[ava_index_b];
		ci = *available_indices[ava_index_c];

		// 三点の三次元座標( x, y は画素位置)
		float ax = ai % _width; float ay = ai / _width; float az = depth_image[ai];
		float bx = bi % _width; float by = bi / _width; float bz = depth_image[bi];
		float cx = ci % _width; float cy = ci / _width; float cz = depth_image[ci];

		//ABベクトル
		float abx = bx - ax; float aby = by - ay; float abz = bz - az;
		//ACベクトル
		float acx = cx - ax; float acy = cy - ay; float acz = cz - az;

		// ABxAC
		float cross_x = aby * acz - abz * acy;
		float cross_y = abz * acx - abx * acz;
		float cross_z = abx * acy - aby * acx;

		// 平面の係数Dを求める
		float ad = -cross_x * ax - cross_y * ay - cross_z * az;

		// 分母部分は共通なので先に作っておく
		float pow_a = cross_x * cross_x;
		float pow_b = cross_y * cross_y;
		float pow_c = cross_z * cross_z;
		float sqrt_pow = std::sqrt( pow_a + pow_b + pow_c );

		if ( sqrt_pow == 0 )
		{
			// 0除算を避ける
			continue;
		}

		// 一致数カウンタ
		int match_count = 0;
		// 点と平面の距離
		float dist = 0;

		// 各点で回す
		for ( const auto& index : available_indices )
		{
			uint16_t px = *index % _width;
			uint16_t py = *index / _width;
			uint16_t pz = depth_image[*index];
			dist = std::abs( cross_x * px + 
							 cross_y * py +	
							 cross_z * pz + ad ) / 
				   std::sqrt( sqrt_pow );

			// 閾値以内の距離か
			if ( dist < threshold )
			{
				++match_count;
			}

		}

		// これまでの合計より一致数が大きければ更新 
		if ( match_count > max_match_count )
		{
			max_match_count = match_count;
			max_a = cross_x;
			max_b = cross_y;
			max_c = cross_z;
			max_d = ad;
		}
	}
	coefficients.clear();
	coefficients.push_back( max_a );
	coefficients.push_back( max_b );
	coefficients.push_back( max_c );
	coefficients.push_back( max_d );
}

void force_flattening_filter::kernel_process( uint16_t& new_depth, const uint16_t* depth_image, const float* lab_image, const int kernel_w, const int x, const int y )
{
	int size = kernel_w * 2 + 1;
	const float& sqr_space_sigma = _sqr_space_sigma_array[kernel_w - 1];

	int target = y * _width + x;

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
				float P = std::exp( -space_distance / (2.0f * sqr_space_sigma) );
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
			const int source = (y + n) * 848 + (x + m);
			const int k = (n + kernel_w) * size + (m + kernel_w);

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
			const float N = std::exp( -lab_dist / (2 * sqr_color_sigma) );

			numerator += depth_image[source] * P * N;
			denominator += P * N;
		}
	}

	if ( numerator != 0 && denominator != 0 )
	{
		new_depth = numerator / denominator;
	}
}

float force_flattening_filter::calc_dispersion( const std::vector<float>& vals )
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

void force_flattening_filter::make_lab_distances( const float* lab_image, float* distances, const int width, const int height )
{
	for ( int y = 0; y < height; ++y )
	{
		for ( int x = 0; x < width; ++x )
		{
			const int i = y * width + x;
			const int right = i + 1;
			const int down = i + width;

			// 右
			if ( right != width )
			{
				distances[i * 2 + 0] = lab_distance( &lab_image[i * 3], &lab_image[right * 3] );
			}

			// 下
			if ( y != height - 1 )
			{
				distances[i * 2 + 1] = lab_distance( &lab_image[i * 3], &lab_image[down * 3] );
			}
		}
	}
}

void force_flattening_filter::timer_start()
{
	_chrono_start = std::chrono::system_clock::now();
}

void force_flattening_filter::timer_end( const std::string& text )
{
	_chrono_end = std::chrono::system_clock::now();

	double time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>
		(_chrono_end - _chrono_start).count() / 1000.0);

	std::cout << "[ " << text << " } : " << time << " ms " << std::endl;
}

uint64_t force_flattening_filter::rand_range( const int& min, const int& max )
{
	// 乱数生成器
	static std::mt19937_64 mt64( 0 );

	// [min, max] の一様分布整数 (int) の分布生成器
	std::uniform_int_distribution<uint64_t> get_rand_uni_int( min, max );

	// 乱数を生成
	return get_rand_uni_int( mt64 );
}

}
