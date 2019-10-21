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
	auto depth = frames.first_or_default( RS2_STREAM_DEPTH, RS2_FORMAT_Z16 ).as<rs2::depth_frame>();
	auto color = frames.first_or_default( RS2_STREAM_COLOR, RS2_FORMAT_RGB8 ).as<rs2::video_frame>();

	// ここに処理を書く
	output_frames.push_back( depth );
	output_frames.push_back( color );

	// フレームをまとめて、返す
	auto new_composite = source.allocate_composite_frame( std::move( output_frames ) );
	return new_composite;
}

}