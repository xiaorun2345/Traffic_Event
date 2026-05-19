/*
* camera_calibrator.h
* Created on: 20210530
* Author: yueyingying
* Description: 计算相机标定映射矩阵类定义
*
*/
#ifndef __CAMERA_CALIBOBRATOR_H__
#define __CAMERA_CALIBOBRATOR_H__

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <string>
#include <vector>

namespace camera 
{

class CameraCalibrator
{
public:
    CameraCalibrator();
    ~CameraCalibrator();

    // 加载并解析json文件
    void LoadJsonFile(const char* json_file);
	// 解析cfg文件中的点列表值并赋值给vector对象
    std::vector<cv::Point2f> Read2DPoints(std::string cfg, int pos);
	// 显示计算结果
    void display();
	// 计算映射矩阵
    void CalculateHomoMatrix();
	// 获取2D到3D的映射矩阵
    cv::Mat GetHomoMatrixInv() {return m_homo_matrix_inv;};
	// 获取3D到2D的映射矩阵
    cv::Mat GetHomoMatrix() {return m_homo_matrix;};

private:
    // input sequence of 2D points on the ground plane, necessary when m_bCalSel2dPtFlg == false
	std::vector<cv::Point2f> m_points2d;
	// input sequence of 3D points on the ground plane
	std::vector<cv::Point2f> m_points3d;
	// homography matrix inv
	cv::Mat m_homo_matrix_inv;
	// homography matrix
	cv::Mat m_homo_matrix;   

 	// method used to computed the camera matrix: 
    //0  - a regular method using all the points; 
    //4  - Least-Median robust method; 8 - RANSAC-based robust method; 
    //-1 - Optimum method with minimum reprojection error   
    int m_cal_type;

	// maximum allowed reprojection error to treat a point pair as an inlier, necessary when m_nCalTyp == 8
	double m_reprojection_thresh;

};

}
#endif