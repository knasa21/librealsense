#pragma once

//#include "../include/librealsense2/hpp/rs_frame.hpp"
//#include "../include/librealsense2/hpp/rs_types.hpp"
//#include "../include/librealsense2/hpp/rs_options.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"

// ==============================
// rs_processing.h 
// ==============================

#include "../include/librealsense2/hpp/rs_types.hpp"
#include "../include/librealsense2/hpp/rs_sensor.hpp"
#include "../include/librealsense2/hpp/rs_options.hpp"

#ifdef __cplusplus
extern "C" {
#endif


	/**
	* Create HoleFilterWithColor processing block.
	* \param[out] error  if non-null, receives any error that occurs during this call, otherwise, errors are ignored
	*/
	rs2_processing_block* rs2_create_hole_filter_with_color_block( rs2_error** error );

#ifdef __cplusplus
}
#endif


// ==============================
// rs_processing.hpp 
// ==============================
namespace rs2
{

/**
カラー画像を使って深度画像のエラー値を修正する穴埋めフィルタ
*/
class hole_filter_with_color : public filter
{
public:
	hole_filter_with_color() : filter( init(), 1 ) {}

	frameset process( frameset frames )
	{
		return filter::process( frames );
	}

protected:
	hole_filter_with_color( std::shared_ptr<rs2_processing_block> block ) : filter( block, 1 ) {}

private:
	friend class context;
	std::shared_ptr<rs2_processing_block> init()
	{
		rs2_error* e = nullptr;

		auto block = std::shared_ptr<rs2_processing_block>(
			rs2_create_hole_filter_with_color_block( &e ),
			rs2_delete_processing_block
			);
		error::handle( e );

		return block;
	}
};

}

// ==============================
// hole-filter-with-color.h
// ==============================
#include "proc/synthetic-stream.h"

namespace librealsense
{

class hole_filter_with_color : public generic_processing_block
{
public:
	hole_filter_with_color() : generic_processing_block( "HoleFilterWithColor" )
	{
	}

protected:
	bool should_process( const rs2::frame& frame ) override;
	rs2::frame process_frame( const rs2::frame_source& soruce, const rs2::frame& f ) override;
	
};

}