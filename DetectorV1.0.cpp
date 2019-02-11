/**************************************************************************************************************************
            MAIN BODY OF SURVEILLANCE CAMERA LIBRARY Version 1.0
			         --- ONLY INCLUDES THE DETECTOR ---
			
This code provides an object detector using a depth sensing Intel Realsense Camera (D435). A virtual plane is defined.
Objects appearing in front of the plane are shown (detected), those behind the plane not.

The virtual plane is selected by first selecting a quandrangular in a flat plane (floor, wall, ...) in the image. This plane
is used as a helper plane. Using the depth capability, the plane is specified in world coordinates by its normal and the distance 
to the origin (ax + by + cz + d =0). Next,a plane perpendicular to this helper plane is defined by selecting a line in the 
first quandrangular: the vector of this line in  world coordinates together with the normal of the of the helper plane define 
the virtual plane of the detector.

Copyright (C), Februari 2019, Jan de Nijs

***************************************************************************************************************************/

#include "survcamlibV1.0.h"

int main() {
	
	//Arrangements rs2 camera (Intel Realsense D435) and video and depth streams
	rs2::pipeline pipe;
	rs2::config cfg;
		
	cfg.enable_stream(RS2_STREAM_COLOR, RES_HORIZONTAL, RES_VERTICAL, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, RES_HORIZONTAL, RES_VERTICAL, RS2_FORMAT_Z16, 30);
	
	rs2::pipeline_profile selection = pipe.start(cfg);
	auto sensor = selection.get_device().first<rs2::depth_sensor>();
	
	//Select VISUAL_PRESET_HIGH_DENSITY to gather many (less accurate/reliable) depth points 
	sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_DENSITY);
	
	//Declare filter for alignment
	rs2::align align_to(RS2_STREAM_COLOR);

	//Start stream
	for (int ii=0;ii<15;ii++) {
		rs2::frameset frames = pipe.wait_for_frames();
		rs2::frame color_frame = frames.get_color_frame();
	}
	
	//Get frame to select virtual plane
	rs2::frameset frames = pipe.wait_for_frames();
	frames = frames.apply_filter(align_to);
	rs2::frame color_frame = frames.get_color_frame();
	rs2::depth_frame depth_frame = frames.get_depth_frame();
	
	//Select first quadrangular (helperPlane) and define all parameters
	cv::Mat colorShow(cv::Size(RES_HORIZONTAL, RES_VERTICAL), CV_8UC3, (void*)color_frame.get_data(),   cv::Mat::AUTO_STEP);
	Quad helperPlane = DefineQuadrangular(colorShow, depth_frame);
	
	//Select virtual detector plane: objects appearing behind the plane are not shown; objects in front are shown.
	Quad virtualPlane = DefinePerpendicularQuadrangular(colorShow, helperPlane, depth_frame);	
	
	cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
	
	//Get reference image to "repair" virtualPlane (Virtual detector plane) 	
	frames = pipe.wait_for_frames();
	frames = frames.apply_filter(align_to);
	color_frame = frames.get_color_frame();
	depth_frame = frames.get_depth_frame();
	
	cv::Mat colorShowRef(cv::Size(RES_HORIZONTAL, RES_VERTICAL), CV_8UC3, (void*)color_frame.get_data(),   cv::Mat::AUTO_STEP);
	
	//Get  camera intrinsics for transistions between pixel coordinates and world coordinates
	rs2_intrinsics intr = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
	
	//Initialize VideoWriter to record video
	cv::VideoWriter writer;
	int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
	double fps(30.0);
	std::string filename("./live.avi");
	writer.open(filename, codec, fps, cv::Size(RES_HORIZONTAL, RES_VERTICAL), true);
	if (!writer.isOpened()) {
		std::cout <<"Could not open VideoWriter" << std::endl;
		return -1;
	}

	//Display filtered image and record video
	for (int ii=0;ii<600;ii++) {
		rs2::frameset frames = pipe.wait_for_frames();
		frames = frames.apply_filter(align_to);
				
		rs2::frame color_frame = frames.get_color_frame();
		rs2::depth_frame depth_frame = frames.get_depth_frame();
		
		cv::Mat colorShow(cv::Size(RES_HORIZONTAL, RES_VERTICAL), CV_8UC3, (void*)color_frame.get_data(),   cv::Mat::AUTO_STEP);
		
		//Detect objects in front of virtual plane and show these in the video
		QuadAreaDetector(colorShow, colorShowRef, depth_frame, &intr, virtualPlane);
			
		cv::imshow("Display Image", colorShow);
		writer.write(colorShow);
		cv::waitKey(1);
	}
	
return 0;
}


				  
	