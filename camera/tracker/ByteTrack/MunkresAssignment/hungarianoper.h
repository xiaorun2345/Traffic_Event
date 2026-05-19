/*
* hungarianoper.h
* Created on: 20210609
* Author: yueyingying
* Description: 匈牙利指派算法对外接口类的定义
*/
#ifndef HUNGARIANOPER_H
#define HUNGARIANOPER_H
#include "munkres/munkres.h"
#include "../../../common/dataType.h"

namespace camera 
{

//匈牙利指派模块
class HungarianOper 
{
public:
    static Eigen::Matrix<float, -1, 2, Eigen::RowMajor> Solve(const DynamicMatrix &cost_matrix);
};

}
#endif // HUNGARIANOPER_H
