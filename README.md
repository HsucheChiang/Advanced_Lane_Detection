# Advanced Lane Detection
>This project is to implement the lane detection algorithm by using C++ language, since many of them are implemented on Python.

***

##  Dependencies
#### 1. CMake ( version 3.5.1 or later )  
#### 2. OpenCV ( version 3.4.1 or later )  
#### 3. Eigen ( version 3.3.4 or later )  

***
##  Installation

#### The following building code and installation procedures are for Linux platform only.  
1. Please make sure your current working directories is under the program's root folder.
2. mkdir build
3. cd build
4. cmake ..
5. make
6. make install
7. Finally, the program will be installed under the LaneDetection folder.
8. ./laneDetection project_video.mp4

***
## Demo
[Video1](https://youtu.be/mP4fvnvR0U8)  
[Video2](https://youtu.be/Ig0B8KG5qYU)  
![Debug Screen](https://github.com/HsucheChiang/Advanced_Lane_Detection/blob/master/Screenshot.png)

***
## Future Works
1. To improve the curve fitting algorithm instead of by just simply using least squrare method to reduce execution time, such that the system can be run in embedded system.  

2. Apply deep learning model to distinguish lane marks from others to make it more precies and robust.

***
## Reference
[https://github.com/rkipp1210/pydata-berlin-2017](https://github.com/rkipp1210/pydata-berlin-2017)





