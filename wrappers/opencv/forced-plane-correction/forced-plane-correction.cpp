// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/ximgproc.hpp>

void button_cb( int state, void* userdata )
{
	std::cout << "button" << std::endl;

	bool* b = (bool*)userdata;
	*b = !*b;
}

int main( int argc, char * argv[] ) try
{
	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;
	rs2::align align_to_color( RS2_STREAM_COLOR );

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	pipe.start();

	using namespace cv;
	const auto window_name = "Display Image";
	namedWindow( window_name, WINDOW_AUTOSIZE );

	int d = 10;
	int sigma_color_int = 385;
	int sigma_space_int = 226;

	int canny_th1 = 8;
	int canny_th2 = 27;

	while ( waitKey( 1 ) < 0 && getWindowProperty( window_name, WND_PROP_AUTOSIZE ) >= 0 )
	{
		rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
		data = align_to_color.process( data );

		rs2::frame depth = data.get_depth_frame().apply_filter( color_map );
		rs2::frame color = data.get_color_frame();

		// Query frame size (width and height)
		const int w = depth.as<rs2::video_frame>().get_width();
		const int h = depth.as<rs2::video_frame>().get_height();

		const int cw = color.as<rs2::video_frame>().get_width();
		const int ch = color.as<rs2::video_frame>().get_height();

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		Mat image( Size( w, h ), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP );
		Mat color_image( Size( cw, ch ), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP );
		cvtColor( color_image, color_image, COLOR_RGB2BGR );
		Mat bilateral_color;
		bilateralFilter( color_image, bilateral_color, d, sigma_color_int / 10.0, sigma_space_int / 10.0 );

		Mat gray;
		cvtColor( bilateral_color, gray, COLOR_BGR2GRAY );
		Mat edge;
		Canny( gray, edge, canny_th1, canny_th2 );

		Mat dilation;
		dilate( edge, dilation, Mat(), Point( -1, -1 ), 1 );
		erode( dilation, dilation, Mat(), Point( -1, -1 ), 1 );
		bitwise_not( dilation, dilation );

		Mat over_dilation;
		dilate( edge, over_dilation, Mat(), Point( -1, -1 ), 2 );
		bitwise_not( over_dilation, over_dilation );

		Mat label_image( dilation.size(), CV_32S );
		int nlabels = connectedComponents( dilation, label_image, 4 );

		// ラベリング結果の描画色
		std::vector<cv::Vec3b> colors( nlabels );
		colors[0] = cv::Vec3b( 0, 0, 0 );
		for ( int label = 1; label < nlabels; ++label )
		{
			colors[label] = cv::Vec3b( (rand() & 255), (rand() & 255), (rand() & 255) );
		}
		// ラベリング結果の描画
		cv::Mat labeled( dilation.size(), CV_8UC3 );
		for ( int y = 0; y < labeled.rows; ++y )
		{
			for ( int x = 0; x < labeled.cols; ++x )
			{
				int label = label_image.at<int>( y, x );
				cv::Vec3b &pixel = labeled.at<cv::Vec3b>( y, x );
				pixel = colors[label];
			}
		}

		Mat add_line;
		image.copyTo( add_line, over_dilation );
		Mat add_line_color;
		color_image.copyTo( add_line_color, dilation );
		Mat add_line_labeled;
		labeled.copyTo( add_line_labeled, over_dilation );

		//ximgproc::thinning( dilation, dilation, ximgproc::ThinningTypes::THINNING_GUOHALL );

		// Update the window with new data
		imshow( window_name, gray );
		imshow( "color", color_image );
		imshow( "color_line", add_line_color );
		imshow( "bilateral_color", bilateral_color );
		createTrackbar( "d", "bilateral_color", &d, 100 );
		createTrackbar( "sigma_color x 10", "bilateral_color", &sigma_color_int, 1000 );
		createTrackbar( "sigma_space x 10", "bilateral_color", &sigma_space_int, 1000 );
		imshow( "canny", edge );
		createTrackbar( "threshold1", "canny", &canny_th1, 255 );
		createTrackbar( "threshold2", "canny", &canny_th2, 255 );

		imshow( "dilation", dilation );
		imshow( "labeld", add_line_labeled );
		imshow( "depth", image );
		imshow( "add_line", add_line );
		imshow( "over_dilation", over_dilation );
	}

	return EXIT_SUCCESS;
}
catch ( const rs2::error & e )
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch ( const std::exception& e )
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}



