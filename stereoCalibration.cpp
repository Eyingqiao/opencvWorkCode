#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <vector>

using namespace cv;
using namespace std;

void funPoints(vector<vector<Point3f>>& objPoints);

//全局变量

  
    string infilenameL = "sample/left/filenameleft.txt";        //如果是右相机把left改为right
    string infilenameR = "sample/right/filenameright.txt";        //如果是右相机把left改为right

 //图像数量
	int imageCount = 0;
   //棋盘三维信息，设置棋盘在世界坐标系的坐标 //保存标定板上角点的三维坐标
       
   	std::vector<std::vector<cv::Point3f>> objectPoints;
    cv::Size imageSize;//=Size(width,height)width,height都可以在读取时得出,而且左右一致
    //标定板上每行每列的角点数
    cv::Size boardSize = cv::Size(9, 6);
  //实际测量得到标定板上每个棋盘格的大小
        cv::Size squareSize = cv::Size(26, 26);
    //保存检测到的所有角点
    std::vector<std::vector<Point2f>> imagePointL;//imagePointL
    std::vector<std::vector<Point2f>> imagePointR;//imagePointR//
 
        //摄像机内参数矩阵 M=[fx γ u0,0 fy v0,0 0 1]
        cv::Mat cameraMatrixL = cv::Mat(3, 3, CV_64F, Scalar::all(0));
 	cv::Mat cameraMatrixR = cv::Mat(3, 3, CV_64F, Scalar::all(0));
        //摄像机的5个畸变系数k1,k2,p1,p2,k3
        cv::Mat distCoeffL = cv::Mat(1, 5, CV_64F, Scalar::all(0));
        cv::Mat distCoeffR= cv::Mat(1, 5, CV_64F, Scalar::all(0));
   	//每幅图片的旋转向量
        std::vector<cv::Mat> tvecsMatL;
        std::vector<cv::Mat> tvecsMatR;
        //每幅图片的平移向量
        std::vector<cv::Mat> rvecsMatL;
        std::vector<cv::Mat> rvecsMatR;

//+双目的参数

Mat R, T, E, F;                                         //R 旋转矢量 T平移矢量 E本征矩阵 F基础矩阵  

//暂时看不出来用处
//vector<Point2f> cornerL;                              //左边摄像机某一照片角点坐标集合  
//vector<Point2f> cornerR;                              //右边摄像机某一照片角点坐标集合  

//Mat rgbImageL, grayImageL;
//Mat rgbImageR, grayImageR;

//使得两幅图像共面的方法就是把两个摄像头的图像投影到一个公共成像面上，这样每幅图像从本图像平面投影到公共图像平面都需要一个旋转矩阵R
	//stereoRectify 这个函数计算的就是从图像平面投影都公共成像平面的旋转矩阵Rl,Rr。 Rl,Rr即为左右相机平面行对准的校正旋转矩阵。
	//左相机经过Rl旋转，右相机经过Rr旋转之后，两幅图像就已经共面并且行对准了。
	//其中Pl,Pr为两个相机的投影矩阵，其作用是将3D点的坐标转换到图像的2D点的坐标:P*[X Y Z 1]' =[x y w]
	//Q矩阵为重投影矩阵，即矩阵Q可以把2维平面(图像平面)上的点投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的视差

Mat Rl, Rr, Pl, Pr, Q;                                  //校正旋转矩阵R，投影矩阵P 重投影矩阵Q (下面有具体的含义解释）   
Mat mapLx, mapLy, mapRx, mapRy;                         //映射表  
Rect validROIL, validROIR;                              //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  


void getParameters(Mat& cameraMatrix, Mat& distCoeff, vector<Mat>& tvecsMat, vector<Mat>& rvecsMat,vector<vector<Point2f>>& imagePointsSeq,int &imageCount,vector<vector<Point3f>> &objectPoints,string infilename)
{

//保存文件名称

    std::vector<std::string>  filenames;

	ifstream fin(infilename);
    //保存标定的结果  ofstream 是从内存写到硬盘
    //ofstream //fout(outfilename);

    /*
    1.读取毎一幅图像，从中提取出角点，然后对角点进行亚像素精确化、获取每个角点在像素坐标系中的坐标
    像素坐标系的原点位于图像的左上角
    */
    std::cout << "开始提取角点......" << std::endl;
    //缓存每幅图像上检测到的角点
    std::vector<Point2f>  imagePointsBuf;//临时变量
  
    char filename[100];
    if (fin.is_open())
    {
	if(imageCount!=0){
		imageCount=0;
	}
        //读取完毕？
        while (!(fin.peek() == EOF))
        {
            //一次读取一行
            fin.getline(filename, sizeof(filename) / sizeof(char));
            //保存文件名
            filenames.push_back(filename);
            //读取图片
            Mat imageInput = cv::imread(filename);
            //读入第一张图片时获取图宽高信息
		
            if (imageCount == 0)
            {
                imageSize.width = imageInput.cols;
                imageSize.height = imageInput.rows;
                std::cout << "imageSize.width = " << imageSize.width << std::endl;
                std::cout << "imageSize.height = " << imageSize.height << std::endl;
            }

            std::cout << "imageCount = " << imageCount << std::endl;
            imageCount++;



    
            //提取每一张图片的角点
            if (cv::findChessboardCorners(imageInput, boardSize, imagePointsBuf) == 0)
            {
                //找不到角点
                std::cout << "Can not find chessboard corners!" << std::endl;
                exit(1);
            }
            else
            {
                Mat viewGray;
                //转换为灰度图片
                cv::cvtColor(imageInput, viewGray, cv::COLOR_BGR2GRAY);
                //亚像素精确化   对粗提取的角点进行精确化
                cv::find4QuadCornerSubpix(viewGray, imagePointsBuf, cv::Size(5, 5));
                //保存亚像素点
//所有角点,也就是imagePointL/R
                imagePointsSeq.push_back(imagePointsBuf);
                //在图像上显示角点位置
                cv::drawChessboardCorners(viewGray, boardSize, imagePointsBuf, true);
                //显示图片
                //cv::imshow("Camera Calibration", viewGray);
//不要保存标了角点的图片了
               // cv::imwrite("test"+to_string(imageCount)+".jpg", viewGray);
                //等待0.5s
                //waitKey(500);
            }
        }        
        
        //计算每张图片上的角点数 54
        int cornerNum = boardSize.width * boardSize.height;
        //角点总数
        int total = imagePointsSeq.size()*cornerNum;
        std::cout << "total = " << total << std::endl;

        for (int i = 0; i < total; i++)
        {
            int num = i / cornerNum;
            int p = i%cornerNum;
            //cornerNum是每幅图片的角点个数，此判断语句是为了输出，便于调试
            if (p == 0)
            {                                        
                std::cout << "\n第 " << num+1 << "张图片的数据 -->: " << std::endl;
            }
            //输出所有的角点
            std::cout<<p+1<<":("<< imagePointsSeq[num][p].x;
            std::cout << imagePointsSeq[num][p].y<<")\t";
            if ((p+1) % 3 == 0)
            {
                std::cout << std::endl;
            }
        }
        std::cout << "角点提取完成!" << std::endl;
	while(!objectPoints.empty() ) {
   	 objectPoints.pop_back();
	}
	int i, j, t;
        for (t = 0; t < imageCount; t++)
        {
            std::vector<cv::Point3f> tempPointSet;
            //行数
            for (i = 0; i < boardSize.height; i++)
            {
                //列数
                for (j = 0; j < boardSize.width; j++)
                {
                    cv::Point3f realPoint;
                    //假设标定板放在世界坐标系中z=0的平面上。
                    realPoint.x = i*squareSize.width;
                    realPoint.y = j*squareSize.height;
                    realPoint.z = 0;
                    tempPointSet.push_back(realPoint);
                }
            }
            objectPoints.push_back(tempPointSet);
        }




        /*
        2.摄像机标定 世界坐标系原点位于标定板左上角(第一个方格的左上角)
        */
        std::cout << "开始标定" << std::endl;
       //开始标定
        cv::calibrateCamera(objectPoints, imagePointsSeq, imageSize, cameraMatrix, distCoeff, rvecsMat, tvecsMat);
        //保存每张图像的旋转矩阵
        cv::Mat rotationMatrix = cv::Mat(3, 3, CV_32FC1, Scalar::all(0));
        for (int i = 0; i < imageCount; i++)
        {
            //将旋转向量转换为相对应的旋转矩阵
            cv::Rodrigues(tvecsMat[i], rotationMatrix);   
        }

 	//毎幅图片角点数量
        std::vector<int> pointCounts;//临时变量,每次标定都要重新初始化
        //初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板
        for (int i = 0; i < imageCount; i++)
        {
            pointCounts.push_back(boardSize.width*boardSize.height);
        }
      
      
    }
	fin.close();
       
}      





int main()
{
//单目分别标定
	
	getParameters(cameraMatrixL, distCoeffL, tvecsMatL,rvecsMatL,imagePointL,imageCount,objectPoints,infilenameL);
	getParameters(cameraMatrixR, distCoeffR, tvecsMatR,rvecsMatR,imagePointR,imageCount,objectPoints,infilenameR);
	
	
	//双目标定

	//标定摄像头
	//由于左右摄像机分别都经过了单目标定
	//所以在此处选择flag = CALIB_USE_INTRINSIC_GUESS
	double rms = stereoCalibrate(objectPoints, imagePointL, imagePointR,cameraMatrixL, distCoeffL,cameraMatrixR, distCoeffR,
		imageSize, R, T, E, F,CALIB_USE_INTRINSIC_GUESS,TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6));

	cout << "Stereo Calibration done with RMS error = " << rms << endl;
	cout<<"开始参数写入"<<endl;
//	把数据写入txt文件//cameraMatrixL, distCoeffL,cameraMatrixR, distCoeffR, R, T, E, F
	string outfilename="stereoCalibrateParas.txt";
	ofstream fout(outfilename);
	fout << "cameraMatrixL:" << std::endl;
        fout << cameraMatrixL << std::endl << std::endl;

	fout << "distCoeffL:" << std::endl;
        fout << distCoeffL<< std::endl << std::endl;

	fout << "cameraMatrixR:" << std::endl;
        fout << cameraMatrixR << std::endl << std::endl;

	fout << "distCoeffR:" << std::endl;
        fout << distCoeffR << std::endl << std::endl;

	fout << "R:" << std::endl;
        fout << R << std::endl << std::endl;
	fout << "T:" << std::endl;
        fout << T << std::endl << std::endl;
	fout << "E:" << std::endl;
        fout << E << std::endl << std::endl;
	fout << "F:" << std::endl;
        fout << F<< std::endl << std::endl;



 	system("pause");   
	return 0;
}



