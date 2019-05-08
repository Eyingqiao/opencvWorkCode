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

Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;

//加canvas参数
Mat canvas;
int w=320,h=240;
	

//使得两幅图像共面的方法就是把两个摄像头的图像投影到一个公共成像面上，这样每幅图像从本图像平面投影到公共图像平面都需要一个旋转矩阵R
	//stereoRectify 这个函数计算的就是从图像平面投影都公共成像平面的旋转矩阵Rl,Rr。 Rl,Rr即为左右相机平面行对准的校正旋转矩阵。
	//左相机经过Rl旋转，右相机经过Rr旋转之后，两幅图像就已经共面并且行对准了。
	//其中Pl,Pr为两个相机的投影矩阵，其作用是将3D点的坐标转换到图像的2D点的坐标:P*[X Y Z 1]' =[x y w]
	//Q矩阵为重投影矩阵，即矩阵Q可以把2维平面(图像平面)上的点投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的视差

Mat Rl, Rr, Pl, Pr, Q;                                  //校正旋转矩阵R，投影矩阵P 重投影矩阵Q (下面有具体的含义解释）   
Mat mapLx, mapLy, mapRx, mapRy;                         //映射表  
Rect validROIL, validROIR;                              //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  






void getParameters(Mat& cameraMatrix, Mat& distCoeff, vector<Mat>& tvecsMat, vector<Mat>& rvecsMat,vector<vector<Point2f>>& imagePointsSeq,int &imageCount,vector<vector<Point3f>> &objectPoints,string infilename,Mat &rgbImage,Mat& grayImage)
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

//rgbImageL/R读imageInput
//将rgImageL赋给grayImage
		rgbImage=imageInput;
		cvtColor(rgbImage, grayImage, 6);//6
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
	
	getParameters(cameraMatrixL, distCoeffL, tvecsMatL,rvecsMatL,imagePointL,imageCount,objectPoints,infilenameL,rgbImageL,grayImageL);
	getParameters(cameraMatrixR, distCoeffR, tvecsMatR,rvecsMatR,imagePointR,imageCount,objectPoints,infilenameR,rgbImageR,grayImageR);
	
	
	//双目标定

	//标定摄像头
	//由于左右摄像机分别都经过了单目标定
	//所以在此处选择flag = CALIB_USE_INTRINSIC_GUESS
	double rms = stereoCalibrate(objectPoints, imagePointL, imagePointR,cameraMatrixL, distCoeffL,cameraMatrixR, distCoeffR,
		imageSize, R, T, E, F,CALIB_USE_INTRINSIC_GUESS,TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6));

	cout << "Stereo Calibration done with RMS error = " << rms << endl;
	/**cout<<"开始参数写入"<<endl;
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

//

*/

//加入矫正

//立体校正的时候需要两幅图像共面并且行对准 以使得立体匹配更加的可靠
	//使得两幅图像共面的方法就是把两个摄像头的图像投影到一个公共成像面上，这样每幅图像从本图像平面投影到公共图像平面都需要一个旋转矩阵R
	//stereoRectify 这个函数计算的就是从图像平面投影都公共成像平面的旋转矩阵Rl,Rr。 Rl,Rr即为左右相机平面行对准的校正旋转矩阵。
	//左相机经过Rl旋转，右相机经过Rr旋转之后，两幅图像就已经共面并且行对准了。
	//其中Pl,Pr为两个相机的投影矩阵，其作用是将3D点的坐标转换到图像的2D点的坐标:P*[X Y Z 1]' =[x y w]
	//Q矩阵为重投影矩阵，即矩阵Q可以把2维平面(图像平面)上的点投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的时差

	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q,
		CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);

	//根据stereoRectify 计算出来的R 和 P 来计算图像的映射表 mapx,mapy
	//mapx,mapy这两个映射表接下来可以给remap()函数调用，来校正图像，使得两幅图像共面并且行对准
	//ininUndistortRectifyMap()的参数newCameraMatrix就是校正后的摄像机矩阵。在openCV里面，校正后的计算机矩阵Mrect是跟投影矩阵P一起返回的。
	//所以在这里传入投影矩阵P，此函数可以从投影矩阵P中读出校正后的摄像机矩阵

	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);


	Mat rectifyImageL, rectifyImageR;
	cvtColor(grayImageL, rectifyImageL, 8);
	cvtColor(grayImageR, rectifyImageR, 8);//CV_GRAY2BGR=8

	//imshow("Rectify Before", rectifyImageL);
	cout<<"两次图像转进制成功"<<endl;
	//remap后左右相机的图像共面并且行对准
	remap(rectifyImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
	remap(rectifyImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

	//imshow("ImageL", rectifyImageL);
	//imshow("ImageR", rectifyImageR);

canvas.create(h * 3, w * 2, CV_8UC3);
	cout<<"第一次画左右画布1"<<endl;
	//左下图像画到画布上
	Mat canvasPart = canvas(Rect(0, 2*h,w, h));
cout<<"第一次画左右画布2"<<endl;
	resize(rectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
cout<<"第一次画左右画布3"<<endl;
	//右下图像画到画布上
	canvasPart = canvas(Rect(w, h * 2, w, h));
	resize(rectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	putText(canvas, "LeftRectified", cv::Point(5, 495), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
	putText(canvas, "RightRectified", cv::Point(325, 495), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
	imshow("Output", canvas);

	//输出矫正后图像
	//imwrite("ImageL",rectifyImageL);
	//imwrite("ImageR", rectifyImageR);


	/*保存并输出数据*/
	//outputCameraParam();


	//把校正结果显示出来
	//把左右两幅图像显示到同一个画面上
	//这里只显示了最后一副图像的校正结果。并没有把所有的图像都显示出来


cout<<"第二次画左右画布"<<endl;
	Mat canvas_;
	double sf_;
	int w_, h_;
	sf_ = 600. / MAX(imageSize.width, imageSize.height);
	w_ = cvRound(imageSize.width * sf_);
	h_ = cvRound(imageSize.height * sf_);
	canvas_.create(h_, w_ * 2, CV_8UC3);

	//左图像画到画布上
	Mat canvasPart_ = canvas_(Rect(w_ * 0, 0, w_, h_));                                //得到画布的一部分  
	resize(rectifyImageL, canvasPart_, canvasPart_.size(), 0, 0, INTER_AREA);  //把图像缩放到跟canvasPart一样大小  
	Rect vroiL(cvRound(validROIL.x*sf_), cvRound(validROIL.y*sf_),                //获得被截取的区域    
		cvRound(validROIL.width*sf_), cvRound(validROIL.height*sf_));
	rectangle(canvasPart_, vroiL, Scalar(0, 0, 200), 3, 8);                      //画上一个矩形  

	//cout << "Painted ImageL" << endl;

	//右图像画到画布上
	canvasPart_ = canvas_(Rect(w_, 0, w_, h_));                                      //获得画布的另一部分  
	resize(rectifyImageR, canvasPart_, canvasPart_.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validROIR.x * sf_), cvRound(validROIR.y*sf_),
		cvRound(validROIR.width * sf_), cvRound(validROIR.height * sf_));
	rectangle(canvasPart_, vroiR, Scalar(0, 159, 0), 3, 8);

	//cout << "Painted ImageR" << endl;

	//画上对应的线条
	for (int i = 0; i < canvas_.rows;i += 16)
	line(canvas_, Point(0, i), Point(canvas_.cols, i), Scalar(0, 200, 0), 1, 8);

	imshow("Rectified", canvas_);

	cout << "wait key" << endl;
	waitKey(0);





 	system("pause");   
	return 0;
}



