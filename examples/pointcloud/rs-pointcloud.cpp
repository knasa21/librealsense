// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max
#include <iostream>
#include <STImageproc.h>

// Helper functions
void register_glfw_callbacks( window& app, glfw_state& app_state );

int main( int argc, char * argv[] ) try
{
	// Create a simple OpenGL window for rendering:
	window app( 1280, 720, "RealSense Pointcloud Example" );
	// Construct an object to manage view state
	glfw_state app_state;
	// register callbacks to allow manipulation of the pointcloud
	register_glfw_callbacks( app, app_state );

	StackTown::STImageproc improc;
	improc.Open();

	//// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	//// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	//rs2::align align_to_color( RS2_STREAM_COLOR );
	//rs2::decimation_filter dec_filter( 2.0f );
	//rs2::disparity_transform depth_to_disparity( true );
	//rs2::temporal_filter temp_filter;
	//rs2::disparity_transform disparity_to_depth( false );

	//rs2::hole_filter_with_color hole_filter;
	//rs2::force_flattening_filter flat_filter;

	//// Declare RealSense pipeline, encapsulating the actual device and sensors
	//rs2::pipeline pipe;
	//rs2::config cfg;
	//cfg.enable_stream( RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16 );
	//cfg.enable_stream( RS2_STREAM_COLOR, 424*2, 240*2, RS2_FORMAT_RGB8 );
	//// Start streaming with default recommended configuration
	//pipe.start( cfg );

	//rs2::frame_queue q;
	//rs2::processing_block pb( [&]( rs2::frame f, const rs2::frame_source& source )
	//{
	//	// source can be used to allocate new frames and send them out
	//	std::vector<rs2::frame> output_frames;
	//	auto frames = f.as<rs2::frameset>();
	//	auto src_depth = frames.first_or_default( RS2_STREAM_DEPTH, RS2_FORMAT_Z16 ).as<rs2::depth_frame>();
	//	auto src_color = frames.first_or_default( RS2_STREAM_COLOR, RS2_FORMAT_RGB8 ).as<rs2::video_frame>();

	//	auto dst_depth = dec_filter.process( src_depth );
	//	dst_depth = depth_to_disparity.process( dst_depth );
	//	dst_depth = temp_filter.process( dst_depth );
	//	dst_depth = disparity_to_depth.process( dst_depth );

	//	output_frames.push_back( dst_depth );
	//	output_frames.push_back( src_color );

	//	q.enqueue( source.allocate_composite_frame( output_frames ) );
	//	
	//} );

	int count = 0;

	while ( app ) // Application still alive?
	{
		// Wait for the next set of frames from the camera
		//auto frames = pipe.wait_for_frames();
		////if ( ++count < 100 )
		//{
		//	pb.start( q ); // results from the block will be enqueued into q
		//	pb.invoke( frames ); // invoke the lambda above
		//	rs2::frame output_frame = q.wait_for_frame(); // fetch the result
		//	frames = align_to_color.process( output_frame );
		//	frames = hole_filter.process( frames );
		//	frames = flat_filter.process( frames );
		//}
		//else if ( count > 200 ) {
			//count = 0;
		//}
		//else
		//{
			//frames = align_to_color.process( frames );
			//frames = flat_filter.process( frames );
		//}
		//std::cout << count << std::endl;
		auto frames = improc.WaitForOriginalData().as<rs2::frameset>();
		auto filteredFrames = improc.WaitForFilteredData().as<rs2::frameset>();


		auto color = filteredFrames.get_color_frame();

		std::cout << color.get_width() << std::endl;

		// For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
		if ( !color )
			color = frames.get_infrared_frame();

		// Tell pointcloud object to map to this color frame
		//pc.map_to( color );

		auto depth = frames.get_depth_frame();

		// Generate the pointcloud and texture mappings
		//points = pc.calculate( depth );
		points = improc.GetPoints();

		// Upload the color frame to OpenGL
		app_state.tex.upload( color );

		// Draw the pointcloud
		draw_pointcloud( app.width(), app.height(), app_state, points );
	}
	improc.Close();

	return EXIT_SUCCESS;
}
catch ( const rs2::error & e )
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch ( const std::exception & e )
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
