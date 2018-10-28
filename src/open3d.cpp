#include <Core/Core.h>
#include <Cuda/Geometry/ImageCuda.h>
#include <Cuda/Geometry/ImagePyramidCuda.h>
#include <Cuda/Geometry/VectorCuda.h>

#include <opencv2/opencv.hpp>

#include <librealsense2/rs.hpp> 
#include "utils.h" 


using namespace open3d;
using namespace cv;

int main(int argc, char * argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    rs2::pipeline_profile selection = pipe.start();
    auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH)
                                 .as<rs2::video_stream_profile>();
    auto resolution = std::make_pair(depth_stream.width(), depth_stream.height());
    auto i = depth_stream.get_intrinsics();
    auto principal_point = std::make_pair(i.ppx, i.ppy);
    auto focal_length = std::make_pair(i.fx, i.fy);



    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    ImageCuda<Vector3b> image_cuda;

    Timer timer;
    while (waitKey(1) < 0 && cvGetWindowHandle(window_name))
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();


        timer.Start(); 
        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
        timer.Stop();
        PrintInfo("Conversion took %.3f ms\n", timer.GetDuration());

        timer.Start(); 
        image_cuda.Upload(image);
        timer.Stop(); 
        PrintInfo("Upload took %.3f ms\n", timer.GetDuration());

        timer.Start(); 
        Mat downloaded_image = image_cuda.Download();
        timer.Stop(); 
        PrintInfo("Download took %.3f ms\n", timer.GetDuration());

        // Update the window with new data
        imshow(window_name, downloaded_image);
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
