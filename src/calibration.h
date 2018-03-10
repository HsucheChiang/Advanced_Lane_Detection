#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

class CameraCalibrator
{
private:
	vector<string> m_filenames;
	vector<vector<Point2f> > m_srcPoints;
	vector<vector<Point3f> > m_dstPoints;
	Size m_imageSize;
public:
	void setFilename();
	void addPoints();
	void doCalibration(Mat &cameraMatrix, Mat &dist);
};
