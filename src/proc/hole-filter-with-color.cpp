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

	auto tgt_depth = source.allocate_video_frame(
		src_depth.get_profile(),
		src_depth,
		src_depth.get_bytes_per_pixel(),
		depth_w,
		depth_h,
		src_depth.get_stride_in_bytes(),
		RS2_EXTENSION_DEPTH_FRAME
	);

	if ( tgt_depth )
	{
		auto ptr = dynamic_cast<librealsense::depth_frame*>( ( librealsense::frame_interface* )tgt_depth.get() );
		auto orig = dynamic_cast<librealsense::depth_frame*>( ( librealsense::frame_interface* )src_depth.get() );

		auto depth_data = (uint16_t*)orig->get_frame_data();
		auto new_data = (uint16_t*)ptr->get_frame_data();

		ptr->set_sensor( orig->get_sensor() );
		auto du = orig->get_units();

		memset( new_data, 0, depth_w * depth_h * sizeof( uint16_t ) );
		for ( int i = 0; i < depth_w * depth_h; ++i )
		{
			auto dist = du * depth_data[i];
			if ( dist >= 0 && dist <= 1.0 )
			{
				new_data[i] = depth_data[i];
			}
		}

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

}