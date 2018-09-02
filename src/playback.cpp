#include "street.hpp"

#include <librealsense2/rs.hpp>
#include "/home/nvidia/librealsense/examples/example.hpp"

#include <iostream>
#include <cstdlib>

// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    // Create a simple OpenGL window for rendering:
	window app(1280, 720, "RealSense Capture Example");
    // Declare two textures on the GPU, one for color and one for depth
    texture depth_image, color_image;

	auto bagfile = "/media/ssd/20180819_091914.bag";
    //rs2::context ctx;
    //rs2::playback device = ctx.load_device("");

	rs2::config cfg;    
	cfg.enable_device_from_file(bagfile);

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start(cfg);

    while(app) // Application still alive?
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        rs2::frame depth = color_map(data.get_depth_frame()); // Find and colorize the depth data
        rs2::frame color = data.get_color_frame();            // Find the color data

        // For cameras that don't have RGB sensor, we'll render infrared frames instead of color
        if (!color)
            color = data.get_infrared_frame();

        // Render depth on to the first half of the screen and color on to the second
        depth_image.render(depth, { 0,               0, app.width() / 2, app.height() });
        color_image.render(color, { app.width() / 2, 0, app.width() / 2, app.height() });
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
