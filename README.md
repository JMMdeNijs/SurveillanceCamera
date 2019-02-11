# SurveillanceCamera

Our home has a small front garden. During our past summer holidays we were warned that a gang of criminels was checking whether the residents were on holidays. They sealed the front door with some tape and checked the seal after a few days. Lukely, our neighbours  removed the tape for us.

This incident has inspired me to develop a smart surveillance camera, one that only records people entering our front garden. To do so, I bought a Intel Realsense camera D435 which can measure the depth using a stereoscopic infrared camera. To match the left and rights images, the camera is equipped with an infrared transmitter that emits a pattern dots on the viewed area.

The first step is the development of a "detector" to control the recording. Here I present the code and a sample of the result.

This code provides an object detector using a depth sensing Intel Realsense Camera (D435). A virtual plane is defined.
Objects appearing in front of the plane are shown (detected), those behind the plane not.

The virtual plane is selected by first selecting a quandrangular in a flat plane (floor, wall, ...) in the image. This plane
is used as a helper plane. Using the depth capability, the plane is specified in world coordinates by its normal and the distance 
to the origin (ax + by + cz + d =0). Next,a plane perpendicular to this helper plane is defined by selecting a line in the 
first quandrangular: the vector of this line in  world coordinates together with the normal of the of the helper plane define 
the virtual plane of the detector.

To select the helper plane and the line, one should move the cursor to the desired point on the image, click the left mouse button and press the "Enter" key. To select the virtual plane, one can use the following keys:
      '='       enlarge the quandrangular
      '-'       reduce the quandrangular
      'f'     flip the triangular
      'a'     accept the triangular

Copyright (C), Februari 2019, Jan de Nijs
