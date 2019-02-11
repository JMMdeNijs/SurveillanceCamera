/**************************************************************************************************************************
            DECLARATIONS OF FUNCTIONS of SURVEILLANCE CAMERA LIBRARY Version 1.0
			
Copyright (C), Februari 2019, Jan de Nijs

***************************************************************************************************************************/

#ifndef __SURVCAMLIB__
#define __SURVCAMLIB__


#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

//D435 supports 1280 x 720, 848 x 480, 640 x 480 and 640 x 400;
#define RES_HORIZONTAL	 1280
#define RES_VERTICAL	 720

struct Quad{
	std::vector<cv::Point2i> corners;
	std::vector<cv::Point2i> border;
	cv::Rect2i	outerRect;
	cv::Vec4f	plane;	//[a, b, c, d] of the (normalized) plane equation x*a + y*b + z*c +d =0 with ||[a, b, c]|| = 1
	int	topRow;	//y pixel index of the top corner
	int topCorner;	//index (0, 1 ..3) of the top Corner;
	int bottomCorner;//index of the bottom Corner
};  

void ReadCursorPosition( int e, int x, int y, int d, void *ptr );

cv::Vec4f PlaneFromPoints(const std::vector<cv::Point3f> points);
cv::Point2i Point2Pixel(const struct rs2_intrinsics * intrin, const cv::Point3f point);
cv::Point3f Pixel2Point(const struct rs2_intrinsics * intrin, const cv::Point2i pix, const float depth);
cv::Point3f IntersectionRayPlane(const struct rs2_intrinsics *intrin, const cv::Point2i pix, cv::Vec4f plane);

std::vector<cv::Point2i> SelectLine(const cv::Mat& image, const cv::String& winname);
std::vector<cv::Point2i> SelectQuadCorners(const cv::Mat& image);
std::vector<cv::Point2i> BorderPixels(const std::vector<cv::Point2i> corners, const int topCorner, const int bottomCorner);
std::vector<cv::Point2i> ArrangeClockWise(std::vector<cv::Point2i> corners);

Quad DefineQuadrangular(const cv::Mat& image, const rs2::depth_frame& depth_frame);
Quad DefinePerpendicularQuadrangular(cv::Mat& image, const Quad quadFirst, const rs2::depth_frame& depth_frame);

void AddTopBottomIndices(Quad& quad);

void DisplayPoint(const cv::Mat& image, const cv::Point2i point, const cv::Scalar color);
void DisplayQuadCornes(cv::Mat& image, const Quad quad);
void DisplayQuadCornesRGBB(cv::Mat& image, const Quad quad);
void DisplayQuadArea(cv::Mat& image, const Quad quad, const cv::Scalar color);
void DisplayQuad_Z(cv::Mat& image, const struct rs2_intrinsics *intr, const Quad quad);

void QuadAreaDetector(cv::Mat& image, const cv::Mat& imageRef, const rs2::depth_frame& depth_frame, const struct rs2_intrinsics *intr, const Quad quad);


#endif