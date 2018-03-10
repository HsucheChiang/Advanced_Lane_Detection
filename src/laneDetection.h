#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include "math.h"

using namespace cv;
using namespace std;
using namespace Eigen;

class laneDetection
{
private:
    Mat perspectiveMatrix;
    Mat oriImage; //The original input image.
    Mat edgeImage; // The result of applying canny edge detection.
    Mat warpEdgeImage;
    Mat warpOriImage;
    vector<Mat> imageChannels;
    Mat RedBinary;
    Mat mergeImage;
    Mat mergeImageRGB;
    Mat histImage; //Histogram visualization.
    Mat maskImage; //The curve image used to blend to the original image.
    Mat maskImageWarp;
    Mat finalResult;
    vector<int> histogram; //Histogram of the detected features
    vector<Point2f> laneL;
    vector<Point2f> laneR;
    vector<Point2f> curvePointsL;
    vector<Point2f> curvePointsR;
    int laneLcount;
    int laneRcount;
    int midPoint; //The mid position of the view.
    int midHeight;
    int leftLanePos; //The detected left lane boundary position.
    int rightLanePos; //The detected right lane boundary position.
    short initRecordCount; // To record the number of times of executions in the first 5 frames.
    const int blockNum; //Number of windows per line.
    int stepY; //Window moving step.
    const int windowSize; //Window Size (Horizontal).
    Vector3d curveCoefL; //The coefficients of the curve (left).
    Vector3d curveCoefR; //The coefficients of the curve (left).
    Vector3d curveCoefRecordL[5]; //To keep the last five record to smooth the current coefficients (left).
    Vector3d curveCoefRecordR[5]; //To keep the last five record to smooth the current coefficients (right).
    int recordCounter;
    bool failDetectFlag; // To indicate whether the road marks is detected succesfully.
    void calHist();
    void boundaryDetection();
    void laneSearch(const int &lanePos, vector<Point2f> &_line, int &lanecount, vector<Point2f> &curvePoints, char dir);
    bool laneCoefEstimate();
    void laneFitting();
public:
    laneDetection(Mat _oriImage, Mat _perspectiveMatrix);
    ~laneDetection();
    void laneDetctAlgo();
    Mat getEdgeDetectResult();
    Mat getWarpEdgeDetectResult();
    Mat getRedChannel();
    Mat getRedBinary();
    Mat getMergeImage();
    Mat getHistImage();
    Mat getMaskImage();
    Mat getWarpMask();
    Mat getFinalResult();
    float getLaneCenterDist();
    void setInputImage(Mat &image);
};
