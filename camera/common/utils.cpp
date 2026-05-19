/*
* utils.cpp
* Created on: 20210622
* Author: yueyingying
* Description: 预处理与后处理相关函数实现
*
*/
#include <fstream>
#include <experimental/filesystem>
#include <math.h>
#include <dirent.h>

#include "utils.h"

namespace camera
{

bool FindFile(const std::string fileName, bool verbose)
{
    if (!std::experimental::filesystem::exists(std::experimental::filesystem::path(fileName)))
    {
        if (verbose) 
            std::cout << "File does not exist : " << fileName << std::endl;
        return false;
    }
    return true;
}

bool IsPointInPolygon(cv::Point pt, Polygon roi)
{
    int maxX = -1, maxY = -1, minX = INT_INF_MAX, minY = INT_INF_MAX;
    unsigned int i, j;
    float x;
    bool flag = false;

    for (i=0; i<roi.size(); i++)
    {
        maxX = std::max(roi.at(i).x , maxX);
        maxY = std::max(roi.at(i).y , maxY);
        minX = std::min(roi.at(i).x , minX);
        minY = std::min(roi.at(i).y , minY);   
    }
    if(pt.x < minX || pt.x > maxX || pt.y < minY || pt.y > maxY)
    {
        return false;
    }

    for(i=0, j=roi.size()-1; i<roi.size(); j=i++)
    {
        if((roi.at(i).y > pt.y) != (roi.at(j).y > pt.y))
        {
            x = (roi.at(j).x-roi.at(i).x) * (pt.y-roi.at(i).y) * 1.0 / (roi.at(j).y - roi.at(i).y) + roi.at(i).x;
            if (pt.x < x)
                flag = !flag;
        } 
    }

    return flag;
}

int ReadFilesInDir(const char *p_dir_name, std::vector<std::string> &file_names) 
{
    DIR *p_dir = opendir(p_dir_name);
    if (p_dir == nullptr) 
    {
        return -1;
    }

    struct dirent* p_file = nullptr;
    while ((p_file = readdir(p_dir)) != nullptr) 
    {
        if (strcmp(p_file->d_name, ".") != 0 &&
            strcmp(p_file->d_name, "..") != 0) 
        {
            //std::string cur_file_name(p_dir_name);
            //cur_file_name += "/";
            //cur_file_name += p_file->d_name;
            std::string cur_file_name(p_file->d_name);
            file_names.push_back(cur_file_name);
        }
    }

    closedir(p_dir);
    return 0;
}

static void TrimLeft(std::string& s)
{
    s.erase(s.begin(), find_if(s.begin(), s.end(), [](int ch) { return !isspace(ch); }));
}

static void TrimRight(std::string& s)
{
    s.erase(find_if(s.rbegin(), s.rend(), [](int ch) { return !isspace(ch); }).base(), s.end());
}

std::string TrimSpace(std::string s)
{
    TrimLeft(s);
    TrimRight(s);
    return s;
}

std::vector<std::string> LoadListFromTextFile(const std::string filename)
{
    std::cout<< filename << std::endl;
    assert(FindFile(filename, false));
    std::vector<std::string> list;

    std::ifstream f(filename);
    if (!f)
    {
        std::cout << "failed to open " << filename;
        assert(0);
    }

    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty())
            continue;

        else
            list.push_back(TrimSpace(line));
    }

    return list;
}

std::vector<std::string> LoadImageList(const std::string filename, const std::string prefix)
{
    std::vector<std::string> fileList = LoadListFromTextFile(filename);
    for (auto& file : fileList)
    {
        if (FindFile(file, false))
        {
            // std::cout << file << std::endl;
            continue;
        }
        else
        {
            std::string prefixed = prefix + file;
            
            if (FindFile(prefixed, false))
                file = prefixed;
            else
                std::cerr << "WARNING: couldn't find: " << prefixed
                          << " while loading: " << filename << std::endl;
        }
    }
    return fileList;
}

//wgs84 to gcj02
static double TransformLatitude(double lng, double lat) 
{
    double ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * sqrt(abs(lng));
    ret += (20.0 * sin(6.0 * lng * M_PI) + 20.0 * sin(2.0 * lng * M_PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(lat * M_PI) + 40.0 * sin(lat / 3.0 * M_PI)) * 2.0 / 3.0;
    ret += (160.0 * sin(lat / 12.0 * M_PI) + 320 * sin(lat * M_PI / 30.0)) * 2.0 / 3.0;
    return ret;
}

static double TransformLongitude(double lng, double lat) 
{
    double ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * sqrt(abs(lng));
    ret += (20.0 * sin(6.0 * lng * M_PI) + 20.0 * sin(2.0 * lng * M_PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(lng * M_PI) + 40.0 * sin(lng / 3.0 * M_PI)) * 2.0 / 3.0;
    ret += (150.0 * sin(lng / 12.0 * M_PI) + 300.0 * sin(lng / 30.0 * M_PI)) * 2.0 / 3.0;
    return ret;
}

/**
 * 判断是否在国内，不在国内则不做偏移
 * @param lng
 * @param lat
 * @returns {boolean}
 */
static double OutOfChina(double& lng, double& lat) 
{
    return (lng < 72.004 || lng > 137.8347) || ((lat < 0.8293 || lat > 55.8271) || false);
}

/**
 * WGS84转GCj02
 * @param lng
 * @param lat
 * @returns {*[]}
 */
bool WGS84ToGCJ02(double& lng, double& lat) 
{
	lng -= LNG_OFFSET;
    lat += LAT_OFFSET;
	const double a = 6378245.0;
	const double ee = 0.00669342162296594323;
    if (OutOfChina(lng, lat)) 
    {
        return false;
    }
    else 
    {
        double dlat = TransformLatitude(lng - 105.0, lat - 35.0);
        double dlng = TransformLongitude(lng - 105.0, lat - 35.0);
        double radlat = lat / 180.0 * M_PI;
        double magic = sin(radlat);
        magic = 1 - ee * magic * magic;
        double sqrtmagic = sqrt(magic);
        dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * M_PI);
        dlng = (dlng * 180.0) / (a / sqrtmagic * cos(radlat) * M_PI);
        double mglat = lat + dlat;
        double mglng = lng + dlng;
		lng = mglng;
		lat = mglat;
		return true;
    }
}

// // 转换车牌在opencv中显示
// int ToWchar(const char* src, wchar_t* dest, const char *locale)
// {
//     if (src == NULL) 
//     {
//         dest = NULL;
//         return 0;
//     }

//     // 根据环境变量设置locale
//     setlocale(LC_CTYPE, locale);

//     // 得到转化为需要的宽字符大小
//     int w_size = mbstowcs(NULL, src, 0) + 1;

//     // w_size = 0 说明mbstowcs返回值为-1。即在运行过程中遇到了非法字符(很有可能使locale
//     // 没有设置正确)
//     if (w_size == 0) 
//     {
//         dest = NULL;
//         return -1;
//     }

//     //wcout << "w_size" << w_size << endl;
//     // dest = new wchar_t[w_size];
//     if (!dest) 
//     {
//         return -1;
//     }

//     int ret = mbstowcs(dest, src, strlen(src)+1);
//     if (ret <= 0) 
//     {
//         return -1;
//     }
//     return 0;
// }

int Clampvalue(int value, int min, int max) 
{
    return std::max(min, std::min(value, max));
}

}

