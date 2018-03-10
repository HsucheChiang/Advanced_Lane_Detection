#include "laneDetection.h"
laneDetection::laneDetection(const Mat _oriImage, const Mat _perspectiveMatrix)
:oriImage(_oriImage), perspectiveMatrix(_perspectiveMatrix), blockNum(9), windowSize(150), recordCounter(0), initRecordCount(0), failDetectFlag(true)
{
    histogram.resize(_oriImage.size().width);
    midPoint = _oriImage.size().width >> 1;
    midHeight = _oriImage.size().height * 0.55;
    stepY = oriImage.size().height / blockNum;
    Vector3d initV;
    initV << 0, 0, 0;
    for(int i=0; i<5; i++)
    {
        curveCoefRecordL[i] = initV;
    }
}

laneDetection::~laneDetection() {}
//The core of lane detection algorithm.
void laneDetection::laneDetctAlgo()
{
    //Conduct Canny edge detection.
    Mat oriImageGray;
    cvtColor(oriImage, oriImageGray, COLOR_RGB2GRAY);
    Canny(oriImageGray, edgeImage, 100, 150, 3);
    warpPerspective(edgeImage, warpEdgeImage, perspectiveMatrix, edgeImage.size());
    inRange(warpEdgeImage, Scalar(1),Scalar(255),warpEdgeImage);

    //Split the color image into different channels.
    warpPerspective(oriImage, warpOriImage, perspectiveMatrix, oriImage.size());
    split(warpOriImage, imageChannels);

    //Conduct binarization for R channel.
    inRange(imageChannels[2], Scalar(200), Scalar(255),RedBinary);

    //Merge the binarized R channel image with edge detected image.
    add(warpEdgeImage, RedBinary, mergeImage);
    cvtColor(mergeImage, mergeImageRGB, CV_GRAY2RGB);

    //Calculate the histogram.
    calHist();

    //Detect the lane boundary.
    boundaryDetection();

    //Lane curve fitting.
    laneSearch(leftLanePos, laneL, laneLcount, curvePointsL, 'L');
    laneSearch(rightLanePos, laneR, laneRcount, curvePointsR, 'R');
    laneCoefEstimate();
    laneFitting();
    warpPerspective(maskImage, maskImageWarp, perspectiveMatrix, maskImage.size(),WARP_INVERSE_MAP);
}


//Calculate the histogram of the lane features along x axis.
void laneDetection::calHist()
{
    histogram.clear();
    for(int i = 0; i < mergeImage.size().width; i++)
    {
        Mat ROI = mergeImage(Rect(i, oriImage.size().height-midHeight-1, 1, midHeight));
        Mat dst;
        divide(255, ROI, dst);
        histogram.push_back((int)(sum(dst)[0]));
    }
    int maxValue = 0;
    maxValue = (*max_element(histogram.begin(), histogram.end())); //the maximum value of the histogram.
    histImage.create(maxValue, histogram.size(), CV_8UC3);
    histImage = Scalar(255,255,255);

    //To create the histogram image
    for(int i=0; i<histogram.size(); i++)
    {
        line(histImage, Point2f(i,(maxValue-histogram.at(i))), Point2f(i,maxValue), Scalar(0,0,255), 1);
    }
}

//Detect the lane boundary.
void laneDetection::boundaryDetection()
{
    //find the left lane boundary position
    vector<int>::iterator maxLPtr;
    maxLPtr = max_element(histogram.begin(), histogram.begin()+midPoint-1);
    int maxL = *maxLPtr;
    leftLanePos = distance(histogram.begin(),maxLPtr);


    //find the right lane boudary position
    vector<int>::iterator maxRPtr;
    maxRPtr = max_element(histogram.begin()+midPoint, histogram.end());
    int maxR = *maxRPtr;
    rightLanePos = distance(histogram.begin(),maxRPtr);

    //draw the lane boundary on iamge
    if((initRecordCount < 5) || (failDetectFlag == true))
    {
        line(mergeImageRGB, Point2f(leftLanePos, 0), Point2f(leftLanePos, mergeImageRGB.size().height), Scalar(0, 255, 0), 10);
        line(mergeImageRGB, Point2f(rightLanePos, 0), Point2f(rightLanePos, mergeImageRGB.size().height), Scalar(0, 255, 0), 10);
    }
}


//Fitting the lane curve.
void laneDetection::laneSearch(const int &lanePos, vector<Point2f> &_line, int &lanecount, vector<Point2f> &curvePoints, char dir)
{
    _line.clear();

    //Lane search.
    const int skipStep = 4;
    int nextPosX = lanePos;
    int xLU = 0, yLU = 0;
    int xRB = 0, yRB = 0;
    int _windowSize = windowSize;
    int _stepY = stepY;
    int sumX = 0;
    int xcounter = 0;
    lanecount = 0;


    if((initRecordCount < 5) || (failDetectFlag == true)) //Conduct full search.
    {
        for(int i=0; i<blockNum; i++)
        {
            _windowSize = windowSize;
            xLU = nextPosX - (windowSize >> 1); //The x coordinate of the upper left point.
            yLU = stepY*(blockNum-i -1); // The y coordinate of the upper left point.
            xRB = xLU + windowSize; //The x coordinate of the bottom right point.
            yRB = yLU + stepY -1; //The y coordinate of the bottom right point.
            // Avoid marginal effect.
            //TODO: to make the code more simple and rearrange it.
            if((xLU < 0))
            {
                xLU =0;
                xRB = xLU + windowSize;
            }
            if(xRB > (mergeImage.size().width-1))
            {
                _windowSize = windowSize + ((mergeImage.size().width-1) - xRB);
                xRB = (mergeImage.size().width-1);
                xLU += ((mergeImage.size().width-1) - xRB);
            }
            if(xRB-xLU > 0 && xRB >= 0 && xLU >= 0)
            {
                //Detect the samples inside the wiondow.
                sumX = 0;
                xcounter = 0;
                uchar* matPtr;
                for(int j=yLU; j<=yRB; j+=skipStep)
                {
                    matPtr = mergeImage.data + (j*mergeImage.size().width);
                    for(int k=xLU; k<=xRB; k+=skipStep)
                    {
                        if(*(matPtr+k) == 255)
                        {
                            sumX += k; xcounter++;
                        }
                    }
                }
                if (xcounter!=0) sumX /= xcounter; //the average x coordinate inside the window.
                else sumX = nextPosX;

                //Modified the window position based on previous calculated average x coodinate.
                nextPosX = sumX;
                xLU = ((nextPosX-(windowSize>>1))>0)? (nextPosX-(windowSize>>1)) : 0;
                xRB = ((xLU + windowSize) < (mergeImage.size().width))? (xLU + windowSize) : (mergeImage.size().width-1);
                if(xRB-xLU > 0 && xRB >= 0 && xLU >= 0)
                {
                    for(int j=yLU; j<=yRB; j+=skipStep)
                    {
                        matPtr = mergeImage.data + (j*mergeImage.size().width);
                        for(int k=xLU; k<=xRB; k+=skipStep)
                        {
                            if(*(matPtr+k) == 255)
                            {
                                lanecount++;
                                _line.push_back(Point2f(k,j));
                            }
                        }
                    }
                }
                rectangle(mergeImageRGB, Point2f(xLU, yLU), Point2f(xRB, yRB),Scalar(255, 0, 0), 5);
            }

        }
    }
    else //Conduct search based on previous results.
    {
        uchar* matPtr;
        int xtemp;
        for(int i=0; i<mergeImage.size().height; i++)
        {
            matPtr = mergeImage.data + (i*mergeImage.size().width);
            for(int j=-50; j<=50; j+=3 )
            {
                xtemp = (curvePoints[i].x + j);
                if(xtemp>=0 && xtemp<mergeImage.size().width)
                {
                    if(*(matPtr+xtemp) == 255)
                    {
                        lanecount++;
                        _line.push_back(Point2f(xtemp,i));
                        if(i>=(mergeImage.size().height/2))
                        {
                            sumX += xtemp;
                            xcounter++;
                        }

                    }
                    mergeImageRGB.at<Vec3b>(i,xtemp)[0] = 0;
                    mergeImageRGB.at<Vec3b>(i,xtemp)[1] = 255;
                    mergeImageRGB.at<Vec3b>(i,xtemp)[2] = 255;
                }
            }
        }
        sumX /= xcounter;
        if((sumX > 0) && (sumX < mergeImageRGB.size().width))
        {
            if(dir == 'L')
            {
                leftLanePos = sumX;
                line(mergeImageRGB, Point2f(leftLanePos, 0), Point2f(leftLanePos, mergeImageRGB.size().height), Scalar(0, 255, 0), 10);

            }
            else
            {
                rightLanePos = sumX;
                line(mergeImageRGB, Point2f(rightLanePos, 0), Point2f(rightLanePos, mergeImageRGB.size().height), Scalar(0, 255, 0), 10);
            }
        }
        else
        {
            if(dir == 'L') line(mergeImageRGB, Point2f(leftLanePos, 0), Point2f(leftLanePos, mergeImageRGB.size().height), Scalar(0, 255, 0), 10);

            else line(mergeImageRGB, Point2f(rightLanePos, 0), Point2f(rightLanePos, mergeImageRGB.size().height), Scalar(0, 255, 0), 10);
        }
    }

}


//Using SVD to solve the coefficients of the curve.
bool laneDetection::laneCoefEstimate()
{
    //To fitting the lance curve by using least square method
    int countThreshold = 300;
    if((laneLcount > countThreshold) && (laneRcount > countThreshold))
    {
        VectorXd xValueL(laneLcount);
        VectorXd xValueR(laneRcount);
        MatrixXd leftMatrix(laneLcount,3);
        MatrixXd rightMatrix(laneRcount,3);

        //left lane curve coefficients estimation
        for(int i=0; i<laneLcount; i++)
        {
            xValueL(i) = laneL[i].x;
            leftMatrix(i,0) = pow(laneL[i].y, 2);
            leftMatrix(i,1) = laneL[i].y;
            leftMatrix(i,2) = 1;
        }

        //right lane curve coefficients estimation
        for(int i=0; i<laneRcount; i++)
        {
            xValueR(i) = laneR[i].x;
            rightMatrix(i,0) = pow(laneR[i].y, 2);
            rightMatrix(i,1) = laneR[i].y;
            rightMatrix(i,2) = 1;
        }
        //curveCoefL = leftMatrix.jacobiSvd(ComputeThinU | ComputeThinV).solve(xValueL);
        //curveCoefR = rightMatrix.jacobiSvd(ComputeThinU | ComputeThinV).solve(xValueR);
        //curveCoefL = leftMatrix.colPivHouseholderQr().solve(xValueL);
        //curveCoefR = rightMatrix.colPivHouseholderQr().solve(xValueR);
        curveCoefL = (leftMatrix.transpose()*leftMatrix).ldlt().solve(leftMatrix.transpose()*xValueL);
        curveCoefR = (rightMatrix.transpose()*rightMatrix).ldlt().solve(rightMatrix.transpose()*xValueR);

        curveCoefRecordL[recordCounter] = curveCoefL;
        curveCoefRecordR[recordCounter] = curveCoefR;
        recordCounter = (recordCounter + 1) % 5;
        if(initRecordCount<5) initRecordCount++;
        failDetectFlag = false;
        return true;
    }
    else
    {
        cerr < "[Lane Detection Algo] There is no enough detected road marks.";
        failDetectFlag = true;
        return false;
    }
}


//To fit the lane.
void laneDetection::laneFitting()
{
    maskImage.create(mergeImage.size().height, mergeImage.size().width, CV_8UC3);
    maskImage = Scalar(0,0,0);
    curvePointsL.clear();
    curvePointsR.clear();

    //To average the past 5 estimated coefficients.
    if(initRecordCount == 5)
    {
        curveCoefL = (curveCoefRecordL[0] + curveCoefRecordL[1] + curveCoefRecordL[2] + curveCoefRecordL[3] + curveCoefRecordL[4]) / 5;
        curveCoefR = (curveCoefRecordR[0] + curveCoefRecordR[1] + curveCoefRecordR[2] + curveCoefRecordR[3] + curveCoefRecordR[4]) / 5;
    }

    int xL, xR;
    for(int i=0; i<mergeImage.size().height; i++)
    {
         xL= pow(i,2) * curveCoefL(0) + i * curveCoefL(1) + curveCoefL(2);
         xR= pow(i,2) * curveCoefR(0) + i * curveCoefR(1) + curveCoefR(2);
         if(xL < 0) xL=0;
         if(xL >= mergeImage.size().width) xL = mergeImage.size().width -1;
         if(xR < 0) xR=0;
         if(xR >= mergeImage.size().width) xR = mergeImage.size().width -1;
         curvePointsL.push_back(Point2f(xL,i));
         curvePointsR.push_back(Point2f(xR,i));
    }
    Mat curveL(curvePointsL, true);
    curveL.convertTo(curveL, CV_32S);
    polylines(maskImage, curveL, false, Scalar(255,0,0), 20, CV_AA);
    Mat curveR(curvePointsR, true);
    curveR.convertTo(curveR, CV_32S);
    polylines(maskImage, curveR, false, Scalar(0,0,255), 20, CV_AA);

    uchar* matPtr;
    for(int i=0; i<maskImage.size().height; i++)
    {
        matPtr = maskImage.data + i * maskImage.size().width * 3;
        for(int j = curvePointsL[i].x; j <= curvePointsR[i].x; j++)
        {
            *(matPtr + j*3) = 0;
            *(matPtr + j*3 + 1) = 255;
            *(matPtr + j*3 + 2) = 0;
        }
    }

}

Mat laneDetection::getEdgeDetectResult()
{
    return edgeImage;
}


Mat laneDetection::getWarpEdgeDetectResult()
{
    return warpEdgeImage;
}

Mat laneDetection::getRedChannel()
{
    return imageChannels[2];
}

Mat laneDetection::getRedBinary()
{
    return RedBinary;
}

Mat laneDetection::getMergeImage()
{
    return mergeImageRGB;
}

Mat laneDetection::getHistImage()
{
    return histImage;
}

Mat laneDetection::getMaskImage()
{
    return maskImage;
}

Mat laneDetection::getWarpMask()
{
    return maskImageWarp;
}

Mat laneDetection::getFinalResult()
{
    addWeighted(maskImageWarp, 0.5, oriImage, 1, 0, finalResult);
    return finalResult;
}

void laneDetection::setInputImage(Mat &image)
{
    oriImage = image.clone();
}

float laneDetection::getLaneCenterDist()
{
    float laneCenter = ((rightLanePos - leftLanePos) / 2) + leftLanePos;
    float imageCenter = mergeImageRGB.size().width / 2;
    float result;
    result = (laneCenter -imageCenter)* 3.5 / 600; //Assume the lane width is 3.5m and about 600 pixels in our image.
    return result;
}




