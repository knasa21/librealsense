// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include "../example.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <atomic>

using namespace std;

/*
 This example introduces the concept of spatial stream alignment.
 For example usecase of alignment, please check out align-advanced and measure demos.
 The need for spatial alignment (from here "align") arises from the fact
 that not all camera streams are captured from a single viewport.
 Align process lets the user translate images from one viewport to another. 
 That said, results of align are synthetic streams, and suffer from several artifacts:
 1. Sampling - mapping stream to a different viewport will modify the resolution of the frame 
               to match the resolution of target viewport. This will either cause downsampling or
               upsampling via interpolation. The interpolation used needs to be of type
               Nearest Neighbor to avoid introducing non-existing values.
 2. Occlussion - Some pixels in the resulting image correspond to 3D coordinates that the original
               sensor did not see, because these 3D points were occluded in the original viewport.
               Such pixels may hold invalid texture values.
*/

// This example assumes camera with depth and color
// streams, and direction lets you define the target stream
enum class direction
{
    to_depth,
    to_color
};

// Forward definition of UI rendering, implemented below
void render_slider(rect location, float* alpha, direction* dir, bool* useHF);

// デバッグ用四角の描画
void draw_square( double x1, double y1, double x2, double y2 );

void timer_start( chrono::system_clock::time_point& time_point );
void timer_end( const chrono::system_clock::time_point& time_point, const std::string& text );

int main(int argc, char * argv[]) try
{
	chrono::system_clock::time_point start;

    // Create and initialize GUI related objects
    window app(848, 640, "RealSense Align Example"); // Simple window handling
    ImGui_ImplGlfw_Init(app, false);      // ImGui library intializition
    rs2::colorizer c;                     // Helper to colorize depth images
    texture depth_image, color_image;     // Helpers for renderig images

    // Create a pipeline to easily configure and start the camera
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16);
    cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8);
    pipe.start(cfg);

    // Define two align objects. One will be used to align
    // to depth viewport and the other to color.
    // Creating align object is an expensive operation
    // that should not be performed in the main loop
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    rs2::align align_to_color(RS2_STREAM_COLOR);
	rs2::hole_filter_with_color hole;
	rs2::force_flattening_filter flattening;
    rs2::rates_printer printer;

	rs2::decimation_filter dec;
	dec.set_option( RS2_OPTION_FILTER_MAGNITUDE, 2 );
	rs2::disparity_transform depth2disparity;
	rs2::disparity_transform disparity2depth( false );
	rs2::spatial_filter spat;
	rs2::temporal_filter temp;
	std::atomic_bool alive = true;

	rs2::decimation_filter dec_color(2.0, rs2_stream::RS2_STREAM_COLOR, rs2_format::RS2_FORMAT_RGB8);
	//dec_color.set_stream_filter( rs2_stream::RS2_STREAM_COLOR, rs2_format::RS2_FORMAT_RGB8 );
	//dec_color.set_option( RS2_OPTION_FILTER_MAGNITUDE, 2 );

	rs2::frame_queue postprocessed_frames;

    float       alpha = 1.0f;               // Transparancy coefficient 
    direction   dir = direction::to_color;  // Alignment direction

	bool useHF = false;

	std::thread video_processing_thread( [&]() {
		// In order to generate new composite frames, we have to wrap the processing
		// code in a lambda
		rs2::processing_block frame_processor(
			[&]( rs2::frameset data, // Input frameset (from the pipeline)
				rs2::frame_source& source ) // Frame pool that can allocate new frames
		{
			// First make the frames spatially aligned

			// Next, apply depth post-processing
			rs2::frame depth = data.get_depth_frame();
			// Decimation will reduce the resultion of the depth image,
			// closing small holes and speeding-up the algorithm
			depth = dec.process( depth );
			// To make sure far-away objects are filtered proportionally
			// we try to switch to disparity domain
			depth = depth2disparity.process( depth );
			// Apply spatial filtering
			depth = spat.process( depth );
			// Apply temporal filtering
			depth = temp.process( depth );
			// If we are in disparity domain, switch back to depth
			depth = disparity2depth.process( depth );

			//checking the size before align process, due to decmiation set to 2, it will redcue the size as 1/2
			float width = depth.as<rs2::video_frame>().get_width();
			float height = depth.as<rs2::video_frame>().get_height();
			std::cout << "before decimate depth:" << width << ", " << height << std::endl;

			auto color = data.get_color_frame();
			std::cout << "before decimate color:" << color.get_width() << ", " << color.get_height() << std::endl;
			color = dec_color.process( color );
			std::cout << "after decimate color:" << color.get_width() << ", " << color.get_height() << std::endl;
			rs2::frameset combined = source.allocate_composite_frame( { depth, color } );
			combined = align_to_color.process( combined );

			//checking the size after align process, the size align to color 640*480
			width = combined.get_color_frame().as<rs2::video_frame>().get_width();
			height = combined.get_color_frame().as<rs2::video_frame>().get_height();

			std::cout << "after align color:" << width << ", " << height << std::endl;

			source.frame_ready( combined );
		} );
		// Indicate that we want the results of frame_processor
		// to be pushed into postprocessed_frames queue
		frame_processor >> postprocessed_frames;

		while ( alive )
		{
			// Fetch frames from the pipeline and send them for processing
			rs2::frameset fs = pipe.wait_for_frames();
			if ( fs.size() != 0 ) frame_processor.invoke( fs );
		}
	} );

    while (app) // Application still alive?
    {
        // Using the align object, we block the application until a frameset is available
		rs2::frameset frameset;
		//frameset = pipe.wait_for_frames();

		frameset = postprocessed_frames.wait_for_frame();

		timer_start( start );
        if (dir == direction::to_depth)
        {
            // Align all frames to depth viewport
            //frameset = align_to_depth.process(frameset);
        }
        else
        {
            // Align all frames to color viewport
			auto color_b = frameset.get_color_frame();
			auto depth_b = frameset.get_color_frame();
			//std::cout << "before :" << color_b.get_width() << ", " << depth_b.get_width() << std::endl;
            //frameset = align_to_color.process(frameset);
			//auto color_a = frameset.get_color_frame();
			//auto depth_a = frameset.get_color_frame();
			//std::cout << "after :" << color_a.get_width() << ", " << depth_a.get_width() << std::endl;
        }
		timer_end( start, "align" );

		if ( useHF )
		{
			timer_start( start );
			frameset = hole.process( frameset );
			timer_end( start, "hole process" );
		}
		else {
			timer_start( start );
			frameset = flattening.process( frameset );
			timer_end( start, "flatten process" );

		}

        // With the aligned frameset we proceed as usual
		auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();
        auto colorized_depth = c.colorize(depth);

        glEnable(GL_BLEND);
        // Use the Alpha channel for blending
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        if (dir == direction::to_depth)
        {
            // When aligning to depth, first render depth image
            // and then overlay color on top with transparancy
            depth_image.render(colorized_depth, { 0, 0, app.width(), app.height() });
            color_image.render(color, { 0, 0, app.width(), app.height() }, alpha);
        }
        else
        {
            // When aligning to color, first render color image
            // and then overlay depth image on top
            color_image.render(color, { 0, 0, app.width(), app.height() });
            depth_image.render(colorized_depth, { 0, 0, app.width(), app.height() }, 1 - alpha);
        }

        glColor4f(1.f, 1.f, 1.f, 1.f);
        glDisable(GL_BLEND);

        // Render the UI:
        ImGui_ImplGlfw_NewFrame(1);
        render_slider({ 15.f, app.height() - 60, app.width() - 30, app.height() }, &alpha, &dir, &useHF);
        ImGui::Render();
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void render_slider(rect location, float* alpha, direction* dir, bool* useHF)
{
    static const int flags = ImGuiWindowFlags_NoCollapse
        | ImGuiWindowFlags_NoScrollbar
        | ImGuiWindowFlags_NoSavedSettings
        | ImGuiWindowFlags_NoTitleBar
        | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoMove;

    ImGui::SetNextWindowPos({ location.x, location.y });
    ImGui::SetNextWindowSize({ location.w, location.h });

    // Render transparency slider:
    ImGui::Begin("slider", nullptr, flags);
    ImGui::PushItemWidth(-1);
    ImGui::SliderFloat("##Slider", alpha, 0.f, 1.f);
    ImGui::PopItemWidth();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Texture Transparancy: %.3f", *alpha);

    // Render direction checkboxes:
    bool to_depth = (*dir == direction::to_depth);
    bool to_color = (*dir == direction::to_color);

    if (ImGui::Checkbox("Align To Depth", &to_depth))
    {
        *dir = to_depth ? direction::to_depth : direction::to_color;
    }
    ImGui::SameLine();
    ImGui::SetCursorPosX(location.w - 140);
    if (ImGui::Checkbox("Align To Color", &to_color))
    {
        *dir = to_color ? direction::to_color : direction::to_depth;
    }

	ImGui::SameLine();
	ImGui::SetCursorPosX( location.w / 2);
	ImGui::Checkbox( "Hole Filter", useHF );

    ImGui::End();
}

void draw_square( double x1, double y1, double x2, double y2 )
{
	glColor3d( 1.f, 0, 0 );
	glBegin(GL_LINE_LOOP);

	glVertex2d( x1, y1 );
	glVertex2d( x1, y2 );
	glVertex2d( x2, y2 );
	glVertex2d( x2, y1 );

	glEnd();
}

void timer_start( chrono::system_clock::time_point& time_point )
{
	time_point = chrono::system_clock::now();
}

void timer_end( const chrono::system_clock::time_point& time_point, const std::string& text )
{
	auto end = chrono::system_clock::now();

	double time = static_cast<double>( chrono::duration_cast<chrono::microseconds>
		( end - time_point ).count() / 1000.0 );

	cout << "[ " << text << " } : " << time << " ms " << endl;
}
