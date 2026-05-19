/*
*
* camCalHomMat.cpp
* Created on: 20210530
* Author: yueyingying
* Description: 计算相机标定映射矩阵类实现
*
*/
#include "camera_calibrator.h"
#include <stdio.h>

namespace camera 
{

CameraCalibrator::CameraCalibrator()
{
	m_cal_type = 4;
	m_reprojection_thresh = 0;
}

CameraCalibrator::~CameraCalibrator()
{
	m_homo_matrix_inv.release();
	m_homo_matrix.release();
}

// 解析标定文件
void CameraCalibrator::LoadJsonFile(const char* json_file)
{
    FILE * poCfgFl;
	long nlFlSz, nlRdRst;
	char * pcBuf;
	int nParamPos;

    assert(json_file != NULL);
    poCfgFl = fopen(json_file, "r");

    if (NULL == poCfgFl) 
    { 
        fputs("Error: json file not opened\n", stderr); 
        exit(1); 
    }

    // 设置流 stream 的文件位置为文件末尾
	fseek(poCfgFl, 0, SEEK_END);
	// 返回给定流 stream 的当前文件位置
    nlFlSz = ftell(poCfgFl);
	// 设置文件位置为给定流 stream 的文件的开头
	rewind(poCfgFl);

	// allocate memory to contain the whole file:
	pcBuf = (char*)malloc(sizeof(char)*nlFlSz);
	if (NULL == poCfgFl) 
    { 
        fputs("Memory error in json file\n", stderr); 
        exit(2); 
    }

    // read the file into the buffer:
	nlRdRst = fread(pcBuf, 1, nlFlSz, poCfgFl);

    std::string strCfg(pcBuf);
	// 删除空格符
    strCfg.erase(std::remove_if(strCfg.begin(), strCfg.end(), ::isspace), strCfg.end());	

	// 图像中2D点列表赋值
	nParamPos = strCfg.find("\"cal2dPtLs\"");
	if (std::string::npos != nParamPos)
		m_points2d = Read2DPoints(strCfg, nParamPos);
	
	// 3D点列表赋值
	nParamPos = strCfg.find("\"cal3dPtLs\"");
	if (std::string::npos != nParamPos)
		m_points3d = Read2DPoints(strCfg, nParamPos);

 	// assertion， 点的个数不能小于4， 并且2D点和3D点的个数要相等
	CV_Assert(4 <= m_points3d.size());
	CV_Assert(m_points2d.size() == m_points3d.size());

    fclose(poCfgFl);
	free(pcBuf);
}

// 解析cfg文件中的点列表值并赋值给vector对象
std::vector<cv::Point2f> CameraCalibrator::Read2DPoints(std::string cfg, int pos)
{
	int nValPos, nValLen, nValEnd, nXPos = pos, nXLen, nYPos, nYLen;
	std::vector<cv::Point2f> points_2d;

	// 数据点列表的开头
	nValPos = cfg.find(":", (pos + 1)) + 1;
	// 数据点列表的结尾
	nValEnd = cfg.find("]]", (nValPos + 1));

	while (nValPos < nValEnd)
	{
		nXPos = cfg.find("[", (nValPos + 1)) + 1;
		nXLen = cfg.find(",", (nXPos + 1)) - nXPos;
		nYPos = nXPos + nXLen + 1;
		nYLen = cfg.find("]", (nYPos + 1)) - nYPos;
		points_2d.push_back(cv::Point2f(std::atof(cfg.substr(nXPos, nXLen).c_str()), std::atof(cfg.substr(nYPos, nYLen).c_str())));
		nValPos = nYPos + nYLen + 1;
	}

	return points_2d;
}

// 显示2D点列表，3D点列表及计算结果矩阵值
void CameraCalibrator::display()
{
    printf("****3d point:*****\n");
    for (uint i=0; i < m_points3d.size(); i++)
        printf("%.8f, %.8f\n", m_points3d[i].x, m_points3d[i].y);
    
    printf("*****2d point:*****\n");
    for (uint i=0; i < m_points2d.size(); i++)
        printf("%.1f, %.1f\n", m_points2d[i].x, m_points2d[i].y);

	printf("****homo inv Mat:****\n");
	for (int i=0; i< m_homo_matrix_inv.rows; i++)
	{
		for(int j=0; j< m_homo_matrix_inv.cols; j++)
			printf("%.4f, ", m_homo_matrix_inv.at<double>(i,j));
		printf("\n");
	}

	printf("****homo Mat:****\n");
	for (int i=0; i< m_homo_matrix.rows; i++)
	{
		for(int j=0; j< m_homo_matrix.cols; j++)
			printf("%.4f, ", m_homo_matrix.at<double>(i,j));
		printf("\n");
	}
}

// 计算映射矩阵
void CameraCalibrator::CalculateHomoMatrix()
{
	// homography matrix
	m_homo_matrix_inv = cv::Mat(3, 3, CV_64F);	
	m_homo_matrix = cv::Mat(3,3,CV_64F);
	// 计算2D点到3D点的映射矩阵
	//m_homo_matrix_inv = cv::findHomography(m_points2d, m_points3d, m_cal_type, m_reprojection_thresh);
	m_homo_matrix_inv = cv::findHomography(m_points2d, m_points3d, m_cal_type, 0);
	
	// 计算3D点到2D点的映射矩阵
	// findHomography函数第3个参数的值的取值范围：
	// 0  - a regular method using all the points; 
    // 4  - Least-Median robust method; 
    // 8 - RANSAC-based robust method; 
    // -1 - Optimum method with minimum reprojection error 
    // 验证只有设置为4才正确
	m_homo_matrix = cv::findHomography(m_points3d, m_points2d, 4, m_reprojection_thresh);
}

}
