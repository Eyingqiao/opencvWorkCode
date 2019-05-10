#include<iostream>
#include<opencv2/opencv.hpp>
 
using namespace std;
using namespace cv;
 
 
class SGBM
{
private:
	enum mode_view { LEFT, RIGHT };
	mode_view view;	//输出左视差图or右视差图
 
public:
	SGBM() {};
	SGBM(mode_view _mode_view) :view(_mode_view) {};
	~SGBM() {};
	Mat computersgbm(Mat &L, Mat &R);	//计算SGBM
};
 
Mat SGBM::computersgbm(Mat &L, Mat &R)
/*SGBM_matching SGBM算法
*@param Mat &left_image :左图像
*@param Mat &right_image:右图像
*/
{
	Mat disp;
 
	int numberOfDisparities = ((L.size().width / 8) + 15)&-16;
	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);
	sgbm->setPreFilterCap(32);
 
	int SADWindowSize = 5;
	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
	sgbm->setBlockSize(sgbmWinSize);
	int cn = L.channels();
 
	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setMinDisparity(0);
	sgbm->setNumDisparities(numberOfDisparities);
	sgbm->setUniquenessRatio(10);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);
 
 
	Mat left_gray, right_gray;
	cvtColor(L, left_gray, 6);
	cvtColor(R, right_gray, 6);
 
	view = LEFT;
	if (view == LEFT)	//计算左视差图
	{
		sgbm->compute(left_gray, right_gray, disp);
 
		disp.convertTo(disp, CV_32F, 1.0 / 16);			//除以16得到真实视差值
 
		Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);
		normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
		imwrite("results/SGBM.jpg", disp8U);
 
		return disp8U;
	}
	else if (view == RIGHT)	//计算右视差图
	{
		sgbm->setMinDisparity(-numberOfDisparities);
		sgbm->setNumDisparities(numberOfDisparities);
		sgbm->compute(left_gray, right_gray, disp);
 
		disp.convertTo(disp, CV_32F, 1.0 / 16);			//除以16得到真实视差值
 
		Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);
		normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
		imwrite("results/SGBM.jpg", disp8U);
 
		return disp8U;
	}
	else
	{
		return Mat();
	}
}
 
 
int main()
{
	Mat left = imread("left13.jpg");
	Mat right = imread("right13.jpg");
	//-------图像显示-----------
	namedWindow("leftimag");
	imshow("leftimag", left);
 
	namedWindow("rightimag");
	imshow("rightimag", right);
	//--------由SAD求取视差图-----
	Mat Disparity;
 
	SGBM mySGBM;
	Disparity = mySGBM.computersgbm(left, right);
 
	//-------结果显示------
	namedWindow("Disparity");
	imshow("Disparity", Disparity);
	imwrite("SGBM.jpg", Disparity);
	//-------收尾------
	waitKey(0);
	return 0;
}
