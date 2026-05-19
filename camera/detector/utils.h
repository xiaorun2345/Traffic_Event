
#ifndef __UTILS_H__
#define __UTILS_H__

#include <vector>
#include <string>

#define OBJ_NAME_MAX_SIZE 16
#define OBJ_NUMB_MAX_SIZE 64

typedef struct _BOX_RECT
{
    int left;
    int right;
    int top;
    int bottom;
} BOX_RECT;

typedef struct __detect_result_t
{
    // char name[OBJ_NAME_MAX_SIZE];
    int class_idx; //目标类型的下标
    BOX_RECT box;
    float prop;
} detect_result_t;

typedef struct _detect_result_group_t
{
    int id;
    int count;
    detect_result_t results[OBJ_NUMB_MAX_SIZE];
} detect_result_group_t;


unsigned char *LoadModel(const char *filename, int *model_size);
int LoadLabelName(const char* locationFilename, std::vector<std::string>& labels);

#endif