#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include "laneDetection.h"
#include "calibration.h"

using namespace cv;
using namespace std;

void videoSliderCallback(int, void*);
void onMouse(int Event, int x, int y, int flags, void* param);

int sliderValue = 0;
VideoCapture laneVideo;
Mat videoFrame; // Video Frame.
Mat videoFrameUndistorted; // Video Frame (after calibration).
Mat videoFramePerspective; // Video Frame (after perspective transform).
Mat _videoFrameUndistorted;
Mat debugWindow(540, 1280, CV_8UC3, Scalar(0,0,0)); //The Debug window.
Size videoSize; // Input Variable Size.
Mat cameraMatrix, dist; //Calibration Matrix.
Mat perspectiveMatrix; //Homography Matrix.
String coordinatetext = "";
Point2f perspectiveSrc[] = {Point2f(565,470), Point2f(721,470), Point2f(277,698), Point2f(1142,698)};
Point2f perspectiveDst[] = {Point2f(300,0), Point2f(980,0), Point2f(300,720), Point2f(980,720)};

int main(int argc, char **argv)
{
    //Get the Perspective Matrix.
    perspectiveMatrix = getPerspectiveTransform(perspectiveSrc,perspectiveDst);


    //Load the video.
	if(argc < 2)
	{
		cerr << "There is no input video." << endl;
		return -1;
	}

	laneVideo.open(argv[1]); //Load the video.
	if(!laneVideo.isOpened())
	{
		cerr << "Could not open the video." << endl;
		return -1;
	}
	videoSize = Size((int)laneVideo.get(CAP_PROP_FRAME_WIDTH),
			         (int)laneVideo.get(CAP_PROP_FRAME_HEIGHT));


	//--------------Camera Calibration Start-----------------
	FileStorage fsRead;
	fsRead.open("Intrinsic.xml", FileStorage::READ);
	Mat src = imread("./camera_cal/calibration2.jpg");
    Mat dst;
	if (fsRead.isOpened() == false)
	{
        CameraCalibrator myCameraCalibrator;
        myCameraCalibrator.doCalibration(cameraMatrix, dist);
        FileStorage fs;
        fs.open("Intrinsic.xml", FileStorage::WRITE);
        fs << "CameraMatrix" << cameraMatrix;
        fs << "Dist" << dist;
        fs.release();
        fsRead.release();
        cout << "There is no existing intrinsic parameters XML file." << endl;
        cout << "Start calibraton......" << endl;
	}
	else
	{
        fsRead["CameraMatrix"] >> cameraMatrix;
        fsRead["Dist"] >> dist;
        fsRead.release();
	}
    undistort(src, dst, cameraMatrix, dist);
	//--------------Camera Calibration Finish-----------------

    //Display Video Image
	laneVideo.set(CAP_PROP_POS_FRAMES, 0);
	laneVideo >> videoFrame;
    namedWindow("Original Image", CV_WINDOW_NORMAL);
    imshow("Original Image",videoFrame);
    undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
    _videoFrameUndistorted = videoFrameUndistorted.clone();


    //draw the roi (for perspective transform)
    line(videoFrameUndistorted, perspectiveSrc[0], perspectiveSrc[1], Scalar(0,0,255), 2);
    line(videoFrameUndistorted, perspectiveSrc[1], perspectiveSrc[3], Scalar(0,0,255), 2);
    line(videoFrameUndistorted, perspectiveSrc[3], perspectiveSrc[2], Scalar(0,0,255), 2);
    line(videoFrameUndistorted, perspectiveSrc[2], perspectiveSrc[0], Scalar(0,0,255), 2);
    circle(videoFrameUndistorted, perspectiveSrc[0], 6, Scalar(0,0,255), CV_FILLED);
    circle(videoFrameUndistorted, perspectiveSrc[1], 6, Scalar(0,0,255), CV_FILLED);
    circle(videoFrameUndistorted, perspectiveSrc[2], 6, Scalar(0,0,255), CV_FILLED);
    circle(videoFrameUndistorted, perspectiveSrc[3], 6, Scalar(0,0,255), CV_FILLED);

    int sliderMaxValue = laneVideo.get(CV_CAP_PROP_FRAME_COUNT) - 10;
    createTrackbar("Frame", "Original Image", &sliderValue, sliderMaxValue, videoSliderCallback);
	videoSliderCallback(sliderValue, 0);

    //Start Homography
    warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix, videoSize);


    //Applying lane detection algorithm
    laneDetection LaneAlgo(_videoFrameUndistorted, perspectiveMatrix);
    LaneAlgo.laneDetctAlgo();

    Mat warpEdge; warpEdge = LaneAlgo.getWarpEdgeDetectResult().clone();
    Mat imageRedChannel; imageRedChannel = LaneAlgo.getRedChannel().clone();
    Mat redBinary; redBinary = LaneAlgo.getRedBinary().clone();
	Mat mergeImage; mergeImage = LaneAlgo.getMergeImage().clone();
	Mat histImage; histImage = LaneAlgo.getHistImage().clone();
	Mat maskImage; maskImage = LaneAlgo.getMaskImage().clone();
	Mat warpMask; warpMask = LaneAlgo.getWarpMask().clone();
	Mat finalResult; finalResult = LaneAlgo.getFinalResult().clone();

    //To create debug window
    Mat debugWindowROI;
    Mat resizePic;

    //Show the image before calibration
    debugWindowROI = debugWindow(Rect(0, 0, 320, 180));
    resize(src, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the image after calibration
    debugWindowROI = debugWindow(Rect(320, 0, 320, 180));
    resize(dst, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the undistoted image
    debugWindowROI = debugWindow(Rect(640, 0, 320, 180));
    resize(videoFrameUndistorted, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the perspective view
    debugWindowROI = debugWindow(Rect(960, 0, 320, 180));
    resize(videoFramePerspective, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the red channel
    debugWindowROI = debugWindow(Rect(0, 180, 320, 180));
    resize(imageRedChannel, resizePic, Size(320,180));
    cvtColor(resizePic, resizePic, CV_GRAY2BGR);
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the canny edge detection result
    debugWindowROI = debugWindow(Rect(320, 180, 320, 180));
    resize(warpEdge, resizePic, Size(320,180));
    cvtColor(resizePic, resizePic, CV_GRAY2BGR);
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the thresholding red channel image
    debugWindowROI = debugWindow(Rect(640, 180, 320, 180));
    resize(redBinary, resizePic, Size(320,180));
    cvtColor(resizePic, resizePic, CV_GRAY2BGR);
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the merged image
    debugWindowROI = debugWindow(Rect(960, 180, 320, 180));
    resize(mergeImage, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show histogram
    debugWindowROI = debugWindow(Rect(0, 360, 320, 180));
    resize(histImage, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show mask
    debugWindowROI = debugWindow(Rect(320, 360, 320, 180));
    resize(maskImage, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show warp mask
    debugWindowROI = debugWindow(Rect(640, 360, 320, 180));
    resize(warpMask, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //show final result
    debugWindowROI = debugWindow(Rect(960, 360, 320, 180));
    resize(finalResult, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Draw lines to separte the images
    line(debugWindow,Point2f(320,0),Point2f(320,539),Scalar(0,150,255),1.8);
    line(debugWindow,Point2f(640,0),Point2f(640,539),Scalar(0,150,255),1.8);
    line(debugWindow,Point2f(960,0),Point2f(960,539),Scalar(0,150,255),1.8);
    line(debugWindow,Point2f(0,180),Point2f(1279,180),Scalar(0,150,255),1.8);
    line(debugWindow,Point2f(0,360),Point2f(1279,360),Scalar(0,150,255),1.8);

    namedWindow("DEBUG", CV_WINDOW_AUTOSIZE);
    imshow("DEBUG", debugWindow);


    //===========Start Real Time Processing===========
    float laneDistant = 0;
    stringstream ss;
    namedWindow("Real Time Execution", CV_WINDOW_NORMAL);
    laneVideo.set(CAP_PROP_POS_FRAMES, 0);
    laneVideo >> videoFrame;
    Mat showVideos(videoFrame.size().height*2, videoFrame.size().width * 2, CV_8UC3, Scalar(0,0,0));
    laneDetection LaneAlgoVideo(_videoFrameUndistorted, perspectiveMatrix);
    undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
    _videoFrameUndistorted = videoFrameUndistorted.clone();

    VideoWriter writer;
    writer.open("Results.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, videoSize);
    VideoWriter writer2;
    writer2.open("DEBUG.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, showVideos.size());


    while(!videoFrame.empty())
    {
        //Start Homography
        warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix, videoSize);

        //Applying lane detection algorithm
        //if(videoFrameCount == 0)
        LaneAlgoVideo.laneDetctAlgo();
        finalResult = LaneAlgoVideo.getFinalResult();

        //Detect the distance to lane center.
        laneDistant = LaneAlgoVideo.getLaneCenterDist();
        if(laneDistant > 0)
        {
            ss.str("");
            ss.clear();
            ss << abs(laneDistant) << "m " << " To the Right";
            putText(finalResult, ss.str(), Point(50,50), 0, 2, Scalar(0, 0, 255), 2);
        }
        else
        {
            ss.str("");
            ss.clear();
            ss << abs(laneDistant) << "m " << " To the Left";
            putText(finalResult, ss.str(), Point(50,50), 0, 2, Scalar(0, 0, 255), 2);
        }


        debugWindowROI = showVideos(Rect(0,0,videoFrame.size().width,videoFrame.size().height));
        addWeighted(debugWindowROI, 0, videoFrame, 1, 0, debugWindowROI);

        debugWindowROI = showVideos(Rect(videoFrame.size().width,0,videoFrame.size().width,videoFrame.size().height));
        addWeighted(debugWindowROI, 0, finalResult, 1, 0, debugWindowROI);

        mergeImage = LaneAlgoVideo.getMergeImage().clone();
        debugWindowROI = showVideos(Rect(0,videoFrame.size().height,videoFrame.size().width,videoFrame.size().height));
        addWeighted(debugWindowROI, 0, mergeImage, 1, 0, debugWindowROI);

        debugWindowROI = showVideos(Rect(videoFrame.size().width,videoFrame.size().height,videoFrame.size().width,videoFrame.size().height));
        addWeighted(debugWindowROI, 0, videoFramePerspective, 1, 0, debugWindowROI);


        imshow("Real Time Execution", showVideos);

        //write the video
        writer.write(finalResult);
        writer2.write(showVideos);

        laneVideo >> videoFrame;
        if(videoFrame.empty()) break;

        //videoFrameCount = (videoFrameCount + 1) % 10;

        //Calibration
        undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
        _videoFrameUndistorted = videoFrameUndistorted.clone();
        LaneAlgoVideo.setInputImage(_videoFrameUndistorted);

        if(waitKey(10) == 27) break;
    }
    //===========Finish Real Time Processing===========

	waitKey(0);
	return 0;

}

void videoSliderCallback(int, void*)
{
    laneVideo.set(CAP_PROP_POS_FRAMES, sliderValue);
	laneVideo >> videoFrame;
	imshow("Original Image",videoFrame);
	undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
	_videoFrameUndistorted = videoFrameUndistorted.clone();
	line(videoFrameUndistorted, perspectiveSrc[0], perspectiveSrc[1], Scalar(0,0,255), 2);
    line(videoFrameUndistorted, perspectiveSrc[1], perspectiveSrc[3], Scalar(0,0,255), 2);
    line(videoFrameUndistorted, perspectiveSrc[3], perspectiveSrc[2], Scalar(0,0,255), 2);
    line(videoFrameUndistorted, perspectiveSrc[2], perspectiveSrc[0], Scalar(0,0,255), 2);
    circle(videoFrameUndistorted, perspectiveSrc[0], 6, Scalar(0,0,255), CV_FILLED);
    circle(videoFrameUndistorted, perspectiveSrc[1], 6, Scalar(0,0,255), CV_FILLED);
    circle(videoFrameUndistorted, perspectiveSrc[2], 6, Scalar(0,0,255), CV_FILLED);
    circle(videoFrameUndistorted, perspectiveSrc[3], 6, Scalar(0,0,255), CV_FILLED);

    //To warp the image.
    warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix, videoSize);

    //Applying lane detection algorithm
    laneDetection LaneAlgo(_videoFrameUndistorted, perspectiveMatrix);
    LaneAlgo.laneDetctAlgo();

    Mat warpEdge; warpEdge = LaneAlgo.getWarpEdgeDetectResult().clone();
    Mat imageRedChannel; imageRedChannel = LaneAlgo.getRedChannel().clone();
    Mat redBinary; redBinary = LaneAlgo.getRedBinary().clone();
    Mat mergeImage; mergeImage = LaneAlgo.getMergeImage().clone();
	Mat histImage; histImage = LaneAlgo.getHistImage().clone();
	Mat maskImage; maskImage = LaneAlgo.getMaskImage().clone();
	Mat warpMask; warpMask = LaneAlgo.getWarpMask().clone();
	Mat finalResult; finalResult = LaneAlgo.getFinalResult().clone();

	//to create debug window
	Mat debugWindowROI;
    Mat resizePic;

    //Show the undistoted image
    debugWindowROI = debugWindow(Rect(640, 0, 320, 180));
    resize(videoFrameUndistorted, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the perspective view
    debugWindowROI = debugWindow(Rect(960, 0, 320, 180));
    resize(videoFramePerspective, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the red channel
    debugWindowROI = debugWindow(Rect(0, 180, 320, 180));
    resize(imageRedChannel, resizePic, Size(320,180));
    cvtColor(resizePic, resizePic, CV_GRAY2BGR);
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the canny edge detection result
    debugWindowROI = debugWindow(Rect(320, 180, 320, 180));
    resize(warpEdge, resizePic, Size(320,180));
    cvtColor(resizePic, resizePic, CV_GRAY2BGR);
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the thresholding red channel image
    debugWindowROI = debugWindow(Rect(640, 180, 320, 180));
    resize(redBinary, resizePic, Size(320,180));
    cvtColor(resizePic, resizePic, CV_GRAY2BGR);
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the merged image
    debugWindowROI = debugWindow(Rect(960, 180, 320, 180));
    resize(mergeImage, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show histogram
    debugWindowROI = debugWindow(Rect(0, 360, 320, 180));
    resize(histImage, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show mask
    debugWindowROI = debugWindow(Rect(320, 360, 320, 180));
    resize(maskImage, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show warp mask
    debugWindowROI = debugWindow(Rect(640, 360, 320, 180));
    resize(warpMask, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //show final result
    debugWindowROI = debugWindow(Rect(960, 360, 320, 180));
    resize(finalResult, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Draw lines to separte the images
    line(debugWindow,Point2f(320,0),Point2f(320,539),Scalar(0,150,255),1.8);
    line(debugWindow,Point2f(640,0),Point2f(640,539),Scalar(0,150,255),1.8);
    line(debugWindow,Point2f(960,0),Point2f(960,539),Scalar(0,150,255),1.8);
    line(debugWindow,Point2f(0,180),Point2f(1279,180),Scalar(0,150,255),1.8);
    line(debugWindow,Point2f(0,360),Point2f(1279,360),Scalar(0,150,255),1.8);

    namedWindow("DEBUG", CV_WINDOW_AUTOSIZE);
    imshow("DEBUG", debugWindow);


}


