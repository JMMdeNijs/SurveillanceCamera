/**************************************************************************************************************************
            DEFINITIONS OF FUNCTIONS of SURVEILLANCE CAMERA LIBRARY Version 1.0
			
Copyright (C), February 2019, Jan de Nijs

***************************************************************************************************************************/

#include "survcamlibV1.0.h"



void ReadCursorPosition( int event, int x, int y, int d, void *ptr ) {
    cv::Point2i *p = (cv::Point2i*)ptr;
    if (event == cv::EVENT_LBUTTONDOWN) { 
		p->x = x;
    	p->y = y;
	}
}

//PlaneFromPoints calculates the best plane defined by a set of 3D points
//Based on: http://www.ilikebigbits.com/blog/2015/3/2/plane-from-points
cv::Vec4f PlaneFromPoints(const std::vector<cv::Point3f> points) {
	
	if (points.size() < 3) throw std::runtime_error("Not enough points to calculate plane");

	cv::Point3f sum(0., 0., 0.);
	for (auto point : points) sum = sum + point;

	cv::Point3f centroid = sum / float(points.size());

	double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
	for (auto point : points) {
		cv::Point3f temp = point - centroid;
		xx += temp.x * temp.x;
		xy += temp.x * temp.y;
		xz += temp.x * temp.z;
		yy += temp.y * temp.y;
		yz += temp.y * temp.z;
		zz += temp.z * temp.z;
	}

	double det_x = yy*zz - yz*yz;
	double det_y = xx*zz - xz*xz;
	double det_z = xx*yy - xy*xy;

	double det_max = std::max({ det_x, det_y, det_z });
	if (det_max <= 0) return{ 0, 0, 0, 0 };

	cv::Point3f dir(0., 0., 0.), normal;
	if (det_max == det_x)
	{
		float a = static_cast<float>((xz*yz - xy*zz) / det_x);
		float b = static_cast<float>((xy*yz - xz*yy) / det_x);
		dir = { 1, a, b };
	}
	else if (det_max == det_y)
	{
		float a = static_cast<float>((yz*xz - xy*zz) / det_y);
		float b = static_cast<float>((xy*xz - yz*xx) / det_y);
		dir = { a, 1, b };
	}
	else
	{
		float a = static_cast<float>((yz*xy - xz*yy) / det_z);
		float b = static_cast<float>((xz*xy - yz*xx) / det_z);
		dir = { a, b, 1 };
	}
	
	normal = dir/norm(dir);
	if (centroid.dot(normal) < 0) normal *= -1.0;

	return {-normal.x, -normal.y, -normal.z, centroid.dot(normal) }; //cv_plane_from_point_and_normal(centroid, dir/norm(dir));
}


//Given a point in 3D space, Point2Pixel computes the corresponding pixel coordinates in an image 
//with no distortion or forward distortion coefficients produced by the same camera 
cv::Point2i Point2Pixel(const struct rs2_intrinsics * intrin, const cv::Point3f point) {
	
	float x(point.x / point.z), y(point.y / point.z);
	cv::Point2i pixel;

    if(intrin->model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY)
    {
        float r2  = x*x + y*y;
        float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        x *= f;
        y *= f;
        float dx = x + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        float dy = y + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = dx;
        y = dy;
    }
    if (intrin->model == RS2_DISTORTION_FTHETA)
    {
        float r = sqrtf(x*x + y*y);
        float rd = (float)(1.0f / intrin->coeffs[0] * atan(2 * r* tan(intrin->coeffs[0] / 2.0f)));
        x *= rd / r;
        y *= rd / r;
    }

    pixel = { (int)(x * intrin->fx + intrin->ppx), (int)(y * intrin->fy + intrin->ppy)};
	return pixel;
}
	

//Given coordinates and depth in an image with no distortion or inverse distortion coefficients,
//Pixel2Point computes the corresponding point in 3D space in the coordinate system of the camera
cv::Point3f Pixel2Point(const struct rs2_intrinsics * intrin, const cv::Point2i pix, const float depth) {
	
	cv::Point3f point;
    assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
    assert(intrin->model != RS2_DISTORTION_FTHETA); // Cannot deproject to an ftheta image

    float x = ((float)pix.x - intrin->ppx) / intrin->fx;
    float y = ((float)pix.y - intrin->ppy) / intrin->fy;
    if(intrin->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
    {	std::cout << "RS2_DISTORTION_INVERSE_BROWN_CONRADY" << std::endl;
        float r2  = x*x + y*y;
        float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        float ux = x*f + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        float uy = y*f + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = ux;
        y = uy;
    }
    point = {depth * x, depth * y, depth};
	return point;
}


//Calculates the intersection point in World Coordinates of a ray from a pixel through the origin and a plane defined by [a, b, c, d]
cv::Point3f IntersectionRayPlane(const struct rs2_intrinsics *intrin, const cv::Point2i pix, cv::Vec4f plane){
	cv::Point3f point;
	float xi = ((float)pix.x - intrin->ppx) / intrin->fx;
    float yj = ((float)pix.y - intrin->ppy) / intrin->fy;
	float z = -plane[3]/(plane[0] * xi + plane[1] * yj + plane[2]);
	
	point = { z * xi, z * yj, z};
	return point;
}

// SelectQuadCorners shows image to select a line defined by 2 pixels. Corners are selecte using the cursor, mouse-click and "Enter" key   //
std::vector<cv::Point2i> SelectLine(const cv::Mat& image, const cv::String& windowName){
	
	std::vector<cv::Point2i> line;
	cv::Rect2i marker = cv::Rect2i(1, 1, 6, 6);
	cv::Point2i cursorCoordinates;
	
	cv::imshow(windowName, image); 
	cv::setMouseCallback(windowName, ReadCursorPosition, &cursorCoordinates);
	
	for (int ii = 0; ii<2 ; ii++) {
		cv::waitKey(0);
		line.push_back(cursorCoordinates);
		marker.x = cursorCoordinates.x - 3;
		marker.y = cursorCoordinates.y - 3;
		cv::rectangle(image, marker.tl(), marker.br(), {0, 0, 0}, 4, cv::LINE_8);
		cv::imshow(windowName, image);
	}
	
	return line;
}

						
// SelectQuadCorners shows image to select the quadrangular. Corners are selecte using the cursor, mouse-click and "Enter" key   //
std::vector<cv::Point2i> SelectQuadCorners(const cv::Mat& image){
	
	cv::Rect2i marker = cv::Rect2i(1, 1, 7, 7);
	cv::Point2i cursorCoordinates, markerSize(7 ,7);
	std::vector<cv::Point2i> corners;
	
	cv::namedWindow("Select quadrangular", cv::WINDOW_AUTOSIZE);
	cv::imshow("Select quadrangular", image);
	cv::setMouseCallback("Select quadrangular", ReadCursorPosition, &cursorCoordinates);
	
	for (int ii = 0; ii<4 ; ii++) {
		cv::waitKey(0);
		marker.x = cursorCoordinates.x - markerSize.x/2;
		marker.y = cursorCoordinates.y - markerSize.y/2;
		corners.push_back(cursorCoordinates);
		cv::rectangle(image, marker.tl(), marker.br(), {0, 0, 0}, 4, cv::LINE_8);
		cv::imshow("Select quadrangular", image);
	}
		
	cv::destroyWindow("Select quadrangular");
	return corners;
}


//find the left and right-side border pixels of the quandrangular
std::vector<cv::Point2i> BorderPixels(const std::vector<cv::Point2i> corners, const int topCorner, const int bottomCorner){
	
	if (corners.size() != 4) throw std::runtime_error("Missing corner of quadrangular ");
	
	int rowOffset = corners[topCorner].y;
	std::vector<cv::Point2i> border(corners[bottomCorner].y - corners[topCorner].y +1 );
	
	for (int ii = 0; ii<4; ii++) {
		float increment((float)(corners[(ii+1)%4].x- corners[ii].x) / (corners[(ii+1)%4].y - corners[ii].y));
		 
		if (corners[(ii+1)%4].y > corners[ii].y)
			for (int rowNo = corners[ii].y; rowNo < corners[(ii+1)%4].y +1; rowNo++)
				 border[rowNo - rowOffset].y = ceil(corners[ii].x + increment * (rowNo - corners[ii].y));
		else if (corners[(ii+1)%4].y < corners[ii].y)
			for (int rowNo = corners[(ii+1)%4].y; rowNo < corners[ii].y +1; rowNo++)
				 border[rowNo - rowOffset].x = floor(corners[ii].x + increment * (rowNo - corners[ii].y));
		else	{
			if (corners[(ii+1)%4].x > corners[ii].x) {
				border[corners[ii].x - rowOffset].x = corners[ii].x;
				border[corners[ii].y - rowOffset].x = corners[(ii+1)%4].x;
			}
			else	{
				border[corners[ii].x - rowOffset].x = corners[(ii+1)%4].x;
				border[corners[ii].y - rowOffset].x = corners[ii].x;
			}
		}
	}
	return border;
}


//Arrange the corners of a quadrangular in clock-wise order
std::vector<cv::Point2i> ArrangeClockWise(std::vector<cv::Point2i> corners){
	
	if (corners.size() != 4) throw std::runtime_error("Missing corner of quadrangular ");
	
	std::vector<cv::Point2i> cornersClockWise;
	float angles[4], anglesSorted[4];
	int mapping[4];
	cv::Point2f centre = {0.f, 0.f}, vector2centre;
	
	//Calculate centre of the quandrangular
	for (auto point : corners) centre += (cv::Point2f) point;
	centre /= 4.0;
	
	//Calculate and order the polar angles of the corners with reference to the centre
	for (int ii=0; ii<4; ii++){
		vector2centre = (cv::Point2f)corners[ii] - centre;
		angles[ii] = atan2f(vector2centre.y, vector2centre.x); // + M_PI;
		anglesSorted[ii] = angles[ii];
	}
	
	std::sort(anglesSorted, anglesSorted + 4);
	
	for (int ii=0; ii<4; ii++)
		for (int jj=0; jj<4; jj++)
			if (angles[jj] > anglesSorted[ii] - 10e-9 && angles[jj] < anglesSorted[ii] + 10e-9) cornersClockWise.push_back(corners[jj]);
	
	return cornersClockWise;		
}


//Gather all parameters that specify a quadrangular in an image
//The corners are freely selected using the cursor and the "Enter" key.
//Every corner must ly outside the triangle defined by the 3 other corners of the quadrangular
Quad DefineQuadrangular(const cv::Mat& image, const rs2::depth_frame& depth_frame){
	
	Quad quadrangular;
	quadrangular.corners = SelectQuadCorners(image);
	quadrangular.corners = ArrangeClockWise(quadrangular.corners);
	
	//TO DO: a check is needed that the quad hasn't a form of a "V" or mirrored "V"; 
	//all corners have to ly outside the triangle defined by the 3 other corners
	
	//Find topRow (minimum y value) and bottom (maximum y-value) corners
	quadrangular.topCorner = 0;
	quadrangular.bottomCorner = 0;
	
	for (int ii=1; ii<4; ii++) {
		quadrangular.topCorner = (quadrangular.corners[ii].y < quadrangular.corners[quadrangular.topCorner].y ) ? ii :  quadrangular.topCorner;
		quadrangular.bottomCorner = (quadrangular.corners[ii].y > quadrangular.corners[quadrangular.bottomCorner].y ) ? ii :  quadrangular.bottomCorner;
	}
	quadrangular.topRow = quadrangular.corners[ quadrangular.topCorner ].y;
	quadrangular.border = BorderPixels(quadrangular.corners, quadrangular.topCorner, quadrangular.bottomCorner);
	
	//Gather the World Coordinates of the pixels inside the quadrangular
	rs2_intrinsics intr = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Camera intrinsics
    
	std::vector<cv::Point3f> pointsPlane;
	cv::Point2i cursorPosition;
	cv::Point3f point;
	float zCoordinate;
	
	for (int rowNo = quadrangular.topRow; rowNo < quadrangular.corners[quadrangular.bottomCorner].y +1; rowNo++)
		for (int colNo = quadrangular.border[rowNo - quadrangular.topRow].x; colNo < quadrangular.border[rowNo - quadrangular.topRow].y + 1; colNo++) {
			zCoordinate = depth_frame.get_distance(colNo, rowNo);
			if (zCoordinate > 0.20) {
				cursorPosition.x = colNo;
				cursorPosition.y = rowNo;
				point = Pixel2Point( &intr, cursorPosition, zCoordinate); 
				pointsPlane.push_back(point);
			}
		}
	
	//Define plane in 3D-space (World Coordinates)
	quadrangular.plane = PlaneFromPoints(pointsPlane);

	//tbd outerRect not defines yet!!!
	
	return quadrangular;
}


//Defines a 2nd quadrangular perpendicular to a 1st quadrangular (quadRef)
//Control by key commands 'f' (flip), '=' (increase height), '-' (decrease hight) and 'a' (accept).
Quad DefinePerpendicularQuadrangular(cv::Mat& image, const Quad quadRef, const rs2::depth_frame& depth_frame){
	
	Quad quad2nd;
	
	cv::Mat copyImage = image.clone();
	std::vector<cv::Point2i> cornersPixelCoordinates; //corners in pixel coordinates
	std::vector<cv::Vec3f>   cornersWorldCoordinates; //cornersWorldCoordinates
		
	const cv::String windowName("Select base line");
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
	cv::imshow(windowName, image);
	
	//Select two first corners that define the base line that lies in the plane of quadRef
	cornersPixelCoordinates = SelectLine(image, windowName);
	cv::waitKey(0);
	
	rs2_intrinsics intr = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
    
	cv::Vec3f normalRef = {quadRef.plane[0], quadRef.plane[1], quadRef.plane[2]};
	cv::Vec3f normal2ndQuad;
	cv::Vec3f inplaneRef, dir;
	
	//The world coordinates of the firt and second corners are calculated from intersection of the line passing the pixel and the origin and the plane 
	//defined by the first quadrangular (quadRef)
	for (int ii=0; ii<2; ii++)
		cornersWorldCoordinates.push_back(IntersectionRayPlane(&intr, cornersPixelCoordinates[ii], quadRef.plane));
		
	//Calculation of the normal vector of the 2nd quadrangular using the outer product
	dir = cornersWorldCoordinates[1] - cornersWorldCoordinates[0];	
	inplaneRef  = dir/norm(dir);
	normal2ndQuad = inplaneRef.cross(normalRef);
	
	//World coordinates of the third and second corners are calculated
	for (int ii=0; ii<2; ii++)
		cornersWorldCoordinates.push_back(cornersWorldCoordinates[ii] - normalRef/2); 
	
	cornersPixelCoordinates.push_back(Point2Pixel(&intr, cornersWorldCoordinates[2]));
	cornersPixelCoordinates.push_back(Point2Pixel(&intr, cornersWorldCoordinates[3]));
	
	//Check corner arrangement and show 2nd quadrangular in the image
	quad2nd.corners = ArrangeClockWise(cornersPixelCoordinates);
	DisplayQuadCornesRGBB(image, quad2nd);
	cv::imshow(windowName, image);
	
	AddTopBottomIndices(quad2nd);
	quad2nd.topRow = quad2nd.corners[ quad2nd.topCorner ].y;
	quad2nd.border = BorderPixels(quad2nd.corners, quad2nd.topCorner, quad2nd.bottomCorner);
	
	DisplayQuadArea(image, quad2nd, {255, 0 ,0});
	cv::imshow(windowName, image);
	
	//Adapt height and baseline of second quadrangular
	bool accept(false);
	int key;
	float height(0.25);
	
	while (!accept) {
		
		key = cv::waitKey(0);
		if 		((char)key == 65 || (char)key == 97) accept = true;
		else if ((char)key == 61) height *= 1.1;
		else if ((char)key == 45) height /= 1.1;
		else if ((char)key == 70 || (char)key == 102) height *= -1.0;
		
		for (int ii=0; ii<2; ii++)
			cornersWorldCoordinates[ii+2] = cornersWorldCoordinates[ii] - normalRef * height; 

		cornersPixelCoordinates[2] = Point2Pixel(&intr, cornersWorldCoordinates[2]);
		cornersPixelCoordinates[3] = Point2Pixel(&intr, cornersWorldCoordinates[3]);

		quad2nd.corners = ArrangeClockWise(cornersPixelCoordinates);
		AddTopBottomIndices(quad2nd);
		quad2nd.topRow = quad2nd.corners[ quad2nd.topCorner ].y;
		quad2nd.border = BorderPixels(quad2nd.corners, quad2nd.topCorner, quad2nd.bottomCorner);
		
		image = copyImage.clone();
		DisplayQuadArea(image, quad2nd, {255, 0 ,0});
		cv::imshow(windowName, image);
	}
	
	//Calculate parameters [a, b, c, d] defining the 2nd plane in world Coordinates 
	float d;
	d = -cornersWorldCoordinates[0].dot(normal2ndQuad);
	quad2nd.plane = {normal2ndQuad[0], normal2ndQuad[1], normal2ndQuad[2], d};
	
	
	DisplayQuad_Z(image, &intr, quad2nd);
	cv::imshow(windowName, image);
	
	cv::waitKey(0);
	cv::destroyWindow(windowName);
	
	return quad2nd;
}


void AddTopBottomIndices(Quad& quad){
	
	int yValues[4];
	for (int ii=0; ii<4; ii++) 		yValues[ii] = quad.corners[ii].y;
		
	std::sort(yValues, yValues + 4 );
	for (int ii=0; ii<4; ii++) {
		if (quad.corners[ii].y == yValues[0]) quad.topCorner = ii;
		if (quad.corners[ii].y == yValues[3]) quad.bottomCorner = ii;
	}
}		


void DisplayPoint(cv::Mat& image, const cv::Point2i point, const cv::Scalar color){
	
	cv::Rect2i imageMarker = cv::Rect2i(1, 1, 6, 6);
	
	imageMarker.x = point.x - 3;
	imageMarker.y = point.y - 3;
	cv::rectangle(image, imageMarker.tl(), imageMarker.br(), color, 3, cv::LINE_4);
}


void DisplayQuadCornes(cv::Mat& image, const Quad quad){
	
	cv::Rect2i imageMarker = cv::Rect2i(1, 1, 7, 7);
	cv::Point2i markerSize(7 ,7);
	
	for (int jj = 0; jj<4 ; jj++) {
		imageMarker.x = quad.corners[jj].x - markerSize.x/2;
		imageMarker.y = quad.corners[jj].y - markerSize.y/2;
		if (jj==quad.topCorner)
			cv::rectangle(image, imageMarker.tl(), imageMarker.br(), {0, 0, 255}, 3, cv::LINE_4);
		else if (jj==quad.bottomCorner)
			cv::rectangle(image, imageMarker.tl(), imageMarker.br(), {0, 255, 0}, 3, cv::LINE_4);
		else
			cv::rectangle(image, imageMarker.tl(), imageMarker.br(), {0, 0, 0}, 3, cv::LINE_4);
	}
}


void DisplayQuadCornesRGBB(cv::Mat& image, const Quad quad){
	
	cv::Rect2i imageMarker = cv::Rect2i(1, 1, 7, 7);
	cv::Point2i markerSize(7 ,7);
	
	for (int jj = 0; jj<4 ; jj++) {
		imageMarker.x = quad.corners[jj].x - markerSize.x/2;
		imageMarker.y = quad.corners[jj].y - markerSize.y/2;
		if (jj==0) cv::rectangle(image, imageMarker.tl(), imageMarker.br(), {0,0, 255}, 3, cv::LINE_4);
		if (jj==1) cv::rectangle(image, imageMarker.tl(), imageMarker.br(), {0, 255, 0}, 3, cv::LINE_4);
		if (jj==2) cv::rectangle(image, imageMarker.tl(), imageMarker.br(), {255, 0, 0}, 3, cv::LINE_4);
		if (jj==3) cv::rectangle(image, imageMarker.tl(), imageMarker.br(), {0, 0, 0}, 3, cv::LINE_4);
		
	}
}


void DisplayQuadArea(cv::Mat& image, const Quad quad, const cv::Scalar color){
	
	for (int rowNo = quad.topRow; rowNo < quad.corners[quad.bottomCorner].y +1; rowNo++)
		for (int colNo = quad.border[rowNo - quad.topRow].x; colNo < quad.border[rowNo - quad.topRow].y + 1; colNo++) {
			image.at<uchar>(rowNo, 3*colNo) = color[0];
			image.at<uchar>(rowNo, 3*colNo+1) = color[1];
			image.at<uchar>(rowNo, 3*colNo+2) = color[2];
		}
}


void DisplayQuad_Z(cv::Mat& image, const struct rs2_intrinsics *intr, const Quad quad){
	cv::Point3f pointWC;
	cv::Point2i pixel;
		
	pixel.x = quad.border[0].x;
	pixel.y = quad.topRow;
	pointWC = IntersectionRayPlane(intr, pixel, quad.plane);
	
	for (int rowNo = quad.topRow; rowNo < quad.corners[quad.bottomCorner].y +1; rowNo++)
		for (int colNo = quad.border[rowNo - quad.topRow].x; colNo < quad.border[rowNo - quad.topRow].y + 1; colNo++) {
			pixel.x = colNo;
			pixel.y = rowNo;
			pointWC = IntersectionRayPlane(intr, pixel, quad.plane);
			image.at<uchar>(rowNo, 3*colNo)   = int(255.0*((pointWC.z - 0.5)/5.0));
			image.at<uchar>(rowNo, 3*colNo+1) = int(255.0*((pointWC.z - 0.5)/5.0));
			image.at<uchar>(rowNo, 3*colNo+2) = int(255.0*((pointWC.z - 0.5)/5.0));
		}					
}


void QuadAreaDetector(cv::Mat& image, const cv::Mat& imageRef, const rs2::depth_frame& depth_frame, const struct rs2_intrinsics *intr, const Quad quad){
	cv::Point3f pointWC;
	float udist;
	
	for (int rowNo = quad.topRow; rowNo < quad.corners[quad.bottomCorner].y +1; rowNo++)
		for (int colNo = quad.border[rowNo - quad.topRow].x; colNo < quad.border[rowNo - quad.topRow].y + 1; colNo++) {
			
			pointWC = IntersectionRayPlane(intr, {colNo, rowNo}, quad.plane);
			udist = depth_frame.get_distance(colNo, rowNo);
			if (udist > pointWC.z or udist == 0.0 ) {
				image.at<uchar>(rowNo, 3*colNo)   = imageRef.at<uchar>(rowNo, 3*colNo);
				image.at<uchar>(rowNo, 3*colNo+1) = imageRef.at<uchar>(rowNo, 3*colNo +1 );
				image.at<uchar>(rowNo, 3*colNo+2) = imageRef.at<uchar>(rowNo, 3*colNo +2);
			}
		}					
}

