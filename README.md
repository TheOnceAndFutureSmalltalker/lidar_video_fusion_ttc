# Time to Collision Estimation Using Video and LiDAR

Fusion of Video images and LiDar readings to determine the time to collision of a vehicle in front of our vehicle.

<p align="center">
<img src="https://github.com/TheOnceAndFutureSmalltalker/lidar_video_fusion_ttc/blob/master/images/draggedimage-1.png"  /><br /><b>Detected Image Keypoints</b></p>
<br />

## Dependencies

This project is written in C++ for Ubuntu 16.04.  It may work on other platforms but it is not guaranteed.

This project requires OpenCV 3.4.1 or later.  If you do not already have OpenCV installed on yoursystem, installing it is not trivial.  There are two different approaches:  installing binaries or building from source.  The first approach, installing binaries, is much faster and easier, but does not include certain commercial functions used by this project.  The second approach, building from source, can take a very long time and is fraught with pitfalls.  But if you want to run this project, you need a version of OpenCV built from source.  You can use [this link](https://www.learnopencv.com/install-opencv3-on-ubuntu/) as a guide.

## Installing, Compiling and Running The Project

To acquire, compile, and run the project in Ubuntu, follow the statements below.  Windows and Mac environments should be modified accordingly.

```bash
$> git clone https://github.com/TheOnceAndFutureSmalltalker/lidar_video_fusion_ttc.git
$> cd lidar_video_fusion_ttc
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment 

## Discusion

Below is the basic operational flowchart of this project.

<p align="center">
<img src="https://github.com/TheOnceAndFutureSmalltalker/lidar_video_fusion_ttc/blob/master/images/course_code_structure.png"  /><br /><b>Range/Doppler FFT</b></p>
<br />

This project demonstrates several important techniques in computer vision and sensor fusion.

