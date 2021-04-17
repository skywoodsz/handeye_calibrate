#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std;

//相机中13组标定板的位姿，x,y,z，x,y,z,w 四元数
// cam2target
Mat_<double> CalPose = (cv::Mat_<double>(3, 6) <<
	0.00871288683265, -0.0363913290203, 0.320786565542, 136.0530778, -27.5380256, 176.4507743,
	0.0164980571717, -0.0345865190029, 0.231964230537,138.5667167, 28.8360199, 176.549674,
	0.0643906146288, 0.0622828789055, 0.252532303333, 143.0462057,86.1882303,174.5012431
	);

//机械臂末端13组位姿,x,y,z，x,y,z,w 四元数
// base2gripper
Mat_<double> ToolPose = (cv::Mat_<double>(3, 6) <<
	-0.0532452683111, - 0.0375567991277, -0.300068004059, 0, 0, 116.6204837,
	0.000777969773235, -0.002783259773, -0.249780487649, 0, 0, 58.3081058,
	-0.0383422737459, 0.00438014533594, -0.201179737778,0,0,0

	);
//R和T转RT矩阵
Mat R_T2RT(Mat &R, Mat &T)
{
	Mat RT;
	Mat_<double> R1 = (cv::Mat_<double>(4, 3) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
		0.0, 0.0, 0.0);
	cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) << T.at<double>(0, 0), T.at<double>(1, 0), T.at<double>(2, 0), 1.0);

	cv::hconcat(R1, T1, RT);//C=A+B左右拼接
	return RT;
}

//RT转R和T矩阵
void RT2R_T(Mat &RT, Mat &R, Mat &T)
{
	cv::Rect R_rect(0, 0, 3, 3);
	cv::Rect T_rect(3, 0, 1, 3);
	R = RT(R_rect);
	T = RT(T_rect);
}
//判断是否为旋转矩阵
bool isRotationMatrix(const cv::Mat & R)
{
	cv::Mat tmp33 = R({ 0,0,3,3 });
	cv::Mat shouldBeIdentity;

	shouldBeIdentity = tmp33.t()*tmp33;

	cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

	return  cv::norm(I, shouldBeIdentity) < 1e-6;
}

/** @brief 欧拉角 -> 3*3 的R
*	@param 	eulerAngle		角度值
*	@param 	seq				指定欧拉角xyz的排列顺序如："xyz" "zyx"
*/
cv::Mat eulerAngleToRotatedMatrix(const cv::Mat& eulerAngle, const std::string& seq)
{
	CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);

	eulerAngle /= 180 / CV_PI;
	cv::Matx13d m(eulerAngle);
	auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
	auto xs = std::sin(rx), xc = std::cos(rx);
	auto ys = std::sin(ry), yc = std::cos(ry);
	auto zs = std::sin(rz), zc = std::cos(rz);

	cv::Mat rotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, xc, -xs, 0, xs, xc);
	cv::Mat rotY = (cv::Mat_<double>(3, 3) << yc, 0, ys, 0, 1, 0, -ys, 0, yc);
	cv::Mat rotZ = (cv::Mat_<double>(3, 3) << zc, -zs, 0, zs, zc, 0, 0, 0, 1);

	cv::Mat rotMat;

	if (seq == "zyx")		rotMat = rotX * rotY*rotZ;
	else if (seq == "yzx")	rotMat = rotX * rotZ*rotY;
	else if (seq == "zxy")	rotMat = rotY * rotX*rotZ;
	else if (seq == "xzy")	rotMat = rotY * rotZ*rotX;
	else if (seq == "yxz")	rotMat = rotZ * rotX*rotY;
	else if (seq == "xyz")	rotMat = rotZ * rotY*rotX;
	else {
		cv::error(cv::Error::StsAssert, "Euler angle sequence string is wrong.",
			__FUNCTION__, __FILE__, __LINE__);
	}

	if (!isRotationMatrix(rotMat)) {
		cv::error(cv::Error::StsAssert, "Euler angle can not convert to rotated matrix",
			__FUNCTION__, __FILE__, __LINE__);
	}

	return rotMat;
	//cout << isRotationMatrix(rotMat) << endl;
}

/** @brief 四元数转旋转矩阵
*	@note  数据类型double； 四元数定义 q = w + x*i + y*j + z*k
*	@param q 四元数输入{w,x,y,z}向量
*	@return 返回旋转矩阵3*3
*/
cv::Mat quaternionToRotatedMatrix(const cv::Vec4d& q)
{
	double w = q[0], x = q[1], y = q[2], z = q[3];

	double x2 = x * x, y2 = y * y, z2 = z * z;
	double xy = x * y, xz = x * z, yz = y * z;
	double wx = w * x, wy = w * y, wz = w * z;

	cv::Matx33d res{
		1 - 2 * (y2 + z2),	2 * (xy - wz),		2 * (xz + wy),
		2 * (xy + wz),		1 - 2 * (x2 + z2),	2 * (yz - wx),
		2 * (xz - wy),		2 * (yz + wx),		1 - 2 * (x2 + y2),
	};
	return cv::Mat(res);
}


/** @brief ((四元数||欧拉角||旋转向量) && 转移向量) -> 4*4 的Rt
*	@param 	m				1*6 || 1*10的矩阵  -> 6  {x,y,z, rx,ry,rz}   10 {x,y,z, qw,qx,qy,qz, rx,ry,rz}
*	@param 	useQuaternion	如果是1*10的矩阵，判断是否使用四元数计算旋转矩阵
*	@param 	seq				如果通过欧拉角计算旋转矩阵，需要指定欧拉角xyz的排列顺序如："xyz" "zyx" 为空表示旋转向量
*/
cv::Mat attitudeVectorToMatrix(cv::Mat& m, bool useQuaternion, const std::string& seq)
{
	CV_Assert(m.total() == 6 || m.total() == 7);
	if (m.cols == 1)
		m = m.t();
	cv::Mat tmp = cv::Mat::eye(4, 4, CV_64FC1);

	//如果使用四元数转换成旋转矩阵则读取m矩阵的第第四个成员，读4个数据
	if (useQuaternion)	// normalized vector, its norm should be 1.
	{
		cv::Vec4d quaternionVec = m({ 3, 0, 4, 1 }); // 切片 从(3,0)开始切 4,1
		quaternionToRotatedMatrix(quaternionVec).copyTo(tmp({ 0, 0, 3, 3 }));
		// cout << norm(quaternionVec) << endl; 
	}
	else
	{
		cv::Mat rotVec;
		if (m.total() == 6)
			rotVec = m({ 3, 0, 3, 1 });		//6
		else
			rotVec = m({ 7, 0, 3, 1 });		//10

		//如果seq为空表示传入的是旋转向量，否则"xyz"的组合表示欧拉角
		if (0 == seq.compare(""))
			cv::Rodrigues(rotVec, tmp({ 0, 0, 3, 3 }));
		else
			eulerAngleToRotatedMatrix(rotVec, seq).copyTo(tmp({ 0, 0, 3, 3 }));
	}
	tmp({ 3, 0, 1, 3 }) = m({ 0, 0, 3, 1 }).t();
	//std::swap(m,tmp);
	return tmp;
}


int main()
{
	//定义手眼标定矩阵
	std::vector<Mat> R_gripper2base;
	std::vector<Mat> t_gripper2base;
	std::vector<Mat> R_base2gripper;
	std::vector<Mat> t_base2gripper;
	std::vector<Mat> R_target2cam;
	std::vector<Mat> t_target2cam;
	Mat R_cam2base = (Mat_<double>(3, 3));
	Mat t_cam2base = (Mat_<double>(3, 1));
	
	// 读取末端，标定板的姿态矩阵 4*4
	std::vector<cv::Mat> vecHg, vecHc;
	cv::Mat Hcg;//定义相机camera到末端base的位姿矩阵
	vector<Mat> images;
	size_t num_images = 13;//13组手眼标定数据
	Mat tempR, tempT;
	
	for (size_t i = 0; i < num_images; i++)//计算标定板位姿
	{
		cv::Mat CalPoseTmp = CalPose.row(i);
		cv::Mat tmp = attitudeVectorToMatrix(CalPoseTmp, false, ""); //转移向量转旋转矩阵
		cv::invert(tmp, tmp, DECOMP_LU);
		vecHc.push_back(tmp);
		RT2R_T(tmp, tempR, tempT);
		//cout << tempT << endl;

		R_target2cam.push_back(tempR);
		t_target2cam.push_back(tempT);
	}
	
	/*
	for (size_t i = 0; i < num_images; i++)//计算机械臂位姿
	{
		cv::Mat ToolPoseTmp = ToolPose.row(i);
		cv::Mat tmp = attitudeVectorToMatrix(ToolPoseTmp, false, "xyz"); //机械臂位姿为欧拉角-旋转矩阵

		RT2R_T(tmp, tempR, tempT);
		//R_gripper2base.push_back(tempR);
		//t_gripper2base.push_back(tempT);

		//cv::invert(tmp, tmp, DECOMP_LU);

		//RT2R_T(tmp, tempR, tempT);
		
		R_base2gripper.push_back(tempR); // eye_to_hand
		t_base2gripper.push_back(tempT);

		vecHg.push_back(tmp);
		
	}
	
	// 手眼标定，CALIB_HAND_EYE_TSAI法为11ms，最快
	calibrateHandEye(R_base2gripper, t_base2gripper, R_target2cam, t_target2cam, R_cam2base, t_cam2base, CALIB_HAND_EYE_TSAI);

	Hcg = R_T2RT(R_cam2base, t_cam2base);//矩阵合并

	std::cout << "Hcg 矩阵为： " << std::endl;
	std::cout << Hcg << std::endl;
	cout << "是否为旋转矩阵：" << isRotationMatrix(Hcg) << std::endl << std::endl;//判断是否为旋转矩阵
	*/
	return 0;
	 
}