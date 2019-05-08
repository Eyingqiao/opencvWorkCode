# opencvWorkCode
  
 运行须知:1安装好了opencv;
 2图片路径,因为没有设置在线的路径(可能我也不会),所以默认下载程序和示例图片到本地,然后更改程序中infilename变量的值;
 3图片路径是从记事本中读取的,以我的为例:xx/xx/left.jpg...
 4运行选用的cmake,需要相应的设置CMakeLists.txt文件,文件要放在工作过目录下,运行程序名改变,CMakeLists.txt内容就要改变
 	命令:cmake .
	make
	 ./<文件名>

Q8的代码运行:
  张正友的单目标定
 
 
 Q12:
// 双目标定
 先进行两个摄像头的单目标定;
 再双目标定;
 得到的结果保存到stereoCalibrationParas.txt中.
 
 Q14
 在Q12的基础上,加了rectify矫正,使在平行的理想情况,极线画出.
