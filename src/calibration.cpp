#include "calibration.h"

//------------------------------------------------
// Load the image files for camera calibration
//------------------------------------------------
void CameraCalibrator::setFilename()
{
	m_filenames.clear();
	m_filenames.push_back("./camera_cal/calibration1.jpg");
	m_filenames.push_back("./camera_cal/calibration2.jpg");
	m_filenames.push_back("./camera_cal/calibration3.jpg");
	m_filenames.push_back("./camera_cal/calibration4.jpg");
	m_filenames.push_back("./camera_cal/calibration5.jpg");
	m_filenames.push_back("./camera_cal/calibration6.jpg");
	m_filenames.push_back("./camera_cal/calibration7.jpg");
	m_filenames.push_back("./camera_cal/calibration8.jpg");
	m_filenames.push_back("./camera_cal/calibration9.jpg");
	m_filenames.push_back("./camera_cal/calibration10.jpg");
	m_filenames.push_back("./camera_cal/calibration11.jpg");
	m_filenames.push_back("./camera_cal/calibration12.jpg");
	m_filenames.push_back("./camera_cal/calibration13.jpg");
	m_filenames.push_back("./camera_cal/calibration14.jpg");
	m_filenames.push_back("./camera_cal/calibration15.jpg");
	m_filenames.push_back("./camera_cal/calibration16.jpg");
	m_filenames.push_back("./camera_cal/calibration17.jpg");
	m_filenames.push_back("./camera_cal/calibration18.jpg");
	m_filenames.push_back("./camera_cal/calibration19.jpg");
	m_filenames.push_back("./camera_cal/calibration20.jpg");
}


//------------------------------------------------
//Detect chessboard points
//------------------------------------------------
void CameraCalibrator::addPoints()
{
	vector<Point2f> chessboardCorner;
	vector<Point3f> realWorldCoord;
	Mat image;
	//real wrold coordinates
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<9; j++)
		{
			realWorldCoord.push_back(Point3f(i, j, 0.0f));
		}
	}

	//find chessboard 2D coordinates

	for(int i=0; i<m_filenames.size(); i++)
	{
		image = imread(m_filenames[i], CV_LOAD_IMAGE_GRAYSCALE);
		m_imageSize = image.size();
		findChessboardCorners(image, Size(9,6), chessboardCorner);
		if(chessboardCorner.size() == 54)
		{
			m_dstPoints.push_back(realWorldCoord);
			m_srcPoints.push_back(chessboardCorner);
		}
	}
	//std::cout << m_dstPoints.size() << std::endl;
	//std::cout << m_srcPoints.size() << std::endl;
}



//------------------------------------------------
// Start Calibration
//------------------------------------------------
void CameraCalibrator::doCalibration(Mat &cameraMatrix, Mat &dist)
{
	setFilename();
	addPoints();

	vector<Mat> rvecs, tvecs;
	calibrateCamera(m_dstPoints,
			        m_srcPoints,
					m_imageSize,
					cameraMatrix,
					dist,
					rvecs,
					tvecs);

	//undistort(src, dst, cameraMatrix, dist);
	//dst = src;
}


