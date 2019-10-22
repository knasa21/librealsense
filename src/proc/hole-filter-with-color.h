#pragma once

//#include "../include/librealsense2/hpp/rs_frame.hpp"
//#include "../include/librealsense2/hpp/rs_types.hpp"
//#include "../include/librealsense2/hpp/rs_options.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"

// ==============================
// rs_processing.h 
// ==============================

// ==============================
// rs_processing.hpp 
// ==============================

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

	rs2::frame prepare_target_frame( const rs2::frame& f, const rs2::frame_source& source, rs2_extension tgt_type );
	
};

}