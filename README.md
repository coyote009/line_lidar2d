# line_lidar2d

## About

This is the repository of software for a 2D lidar using a line laser and a Raspberry Pi camera.
The repository includes a set of tools that are required for generating calibration charts, performing a series of calibration steps, and estimating depth.

## Tools

* Calibration chart generator
  * Generate calibration chart image to be printed out
* Camera calibration tool
  * Perform intrinsic camera calibration
* Line laser calibration tool
  * Perform calibration of line laser pose
* 2D depth estimator
  * Estimate 2D depth

## HowTo

1. Build raspberry pi camera image acquisition library

First, you need to build the image acquisition library in the following manner.

>> \> cd raspivideocap2  
>> \> mkdir build
>> \> cd build
>> \> cmake ..
>> \> make

After building the library you can test the library if you want.

>> \> cd test_raspivideocap2
>> \> cd build
>> \> cmake ..
>> \> make
>> \> ./test_raspivideocap2

2. Generate calibration chart image(s) if you don't have one

>> \> cd calib_chess_generator
>> \> mkdir build
>> \> cd build
>> \> cmake ..
>> \> make
>> \> ./calib_chess_generator

3. Build and run the intrinsic calibration tool

The procedure is almost the same as step2 but cd into calib_mono_raspicam2


4. Build and run the line laser calibration tool

The procedure is almost the same as step2 but cd into calib_line_chess_raspicam2

5. Build and run the depth estimation tool

The procedure is almost the same as step2 but cd into line_depth2_raspicam2


