#include "rga.h"
#include "im2d.hpp"
#include "RgaUtils.h"
#include "rknn_api.h"
#include "postprocess.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "utils.h"
#include "rknn_detect.h"
#include <sys/time.h>
#include <unordered_map>

namespace camera 
{
#define PRINT_TIME

using namespace std;



RKNNDetector::RKNNDetector(const char *model_name, const std::vector<std::string> class_names, float conf_thresh, float nms_thresh, bool nosigmoid, int npu_idx)
{
    this->conf_thresh = conf_thresh;
    this->nms_thresh = nms_thresh;
    this->nosigmoid = nosigmoid;

    /* Create the neural network */
    printf("Loading mode...\n");
    int model_data_size = 0;
    // 读取模型文件数据
    model_data = LoadModel(model_name, &model_data_size);
    // 通过模型文件初始化rknn类
    ret = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
    if (ret < 0)
    {
        printf("rknn_init error ret=%d\n", ret);
        exit(-1);
    }
    
    // 加载labels
    labels = class_names;
    object_class_num = labels.size();
    printf("object class number: %d\n", object_class_num);
    for(int i=0; i<labels.size(); i++)
    {
        printf("%s; ", labels[i].c_str());
    }
    printf("\n");


    std::unordered_map<int, rknn_core_mask> core_mask_map = {
        {0, RKNN_NPU_CORE_0},
        {1, RKNN_NPU_CORE_1},
        {2, RKNN_NPU_CORE_2},
        {3, RKNN_NPU_CORE_0_1},
        {4, RKNN_NPU_CORE_0_1_2},
    };

    // 从映射中获取对应的枚举值
    rknn_core_mask core_mask = core_mask_map[npu_idx];

    //rknn_core_mask core_mask = RKNN_NPU_CORE_0;
    int ret = rknn_set_core_mask(ctx, core_mask);
    if (ret < 0)
    {
        printf("rknn_init error ret=%d\n", ret);
        exit(-1);
    }


    // 初始化rknn类的版本
    rknn_sdk_version version;
    ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
    if (ret < 0)
    {
        printf("rknn_init error ret=%d\n", ret);
        exit(-1);
    }
    printf("sdk version: %s driver version: %s\n", version.api_version, version.drv_version);
    
    
    

    // 1 获取模型的输入参数
    //rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret < 0)
    {
        printf("rknn_init error ret=%d\n", ret);
        exit(-1);
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    // 2 设置输入数组
    input_attrs = new rknn_tensor_attr[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (int i = 0; i < io_num.n_input; i++)
    {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret < 0)
        {
            printf("rknn_init error ret=%d\n", ret);
            exit(-1);
        }
        dump_tensor_attr(&(input_attrs[i]));
    }

    // 3 设置输出数组
    output_attrs = new rknn_tensor_attr[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs) );
    for (int i = 0; i < io_num.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret < 0)
        {
            printf("rknn_init error ret=%d\n", ret);
            exit(-1);
        }
        dump_tensor_attr(&(output_attrs[i]));

        out_scales.push_back(output_attrs[i].scale);
        out_zps.push_back(output_attrs[i].zp);
    }

    // 4 设置输入参数
    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
        printf("model is NCHW input fmt\n");
        channel = input_attrs[0].dims[1];
        net_height = input_attrs[0].dims[2];
        net_width = input_attrs[0].dims[3];
    }
    else
    {
        printf("model is NHWC input fmt\n");
        net_height = input_attrs[0].dims[1];
        net_width = input_attrs[0].dims[2];
        channel = input_attrs[0].dims[3];
    }
    
    printf("model input height: %d, width : %d\n, channel=%d\n", net_height, net_width, channel);

    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].size = net_width * net_height * channel;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].pass_through = 0;

    //网络输入申请内存
    net_input_size = net_height *  net_width *  channel;
    net_input_buffer = malloc(net_input_size);
    memset(net_input_buffer, 0x00,  net_height *  net_width *  channel);

    printf("RKNNDetector init success.....\n");
}

RKNNDetector::~RKNNDetector()
{
    
    ret = rknn_destroy(ctx);
    delete[] input_attrs;
    delete[] output_attrs;
    if (model_data)
        free(model_data);
    
    vector<string>().swap(labels);

    if (net_input_buffer)
    {
        free(net_input_buffer);
        net_input_buffer = nullptr;
    }
}

void RKNNDetector::PrepImage(const cv::Mat& ori_img)
{
    cv::Mat img;

    // 获取图像宽高
    int img_width = ori_img.cols;
    int img_height = ori_img.rows;

    cv::cvtColor(ori_img, img, cv::COLOR_BGR2RGB);
    // // init rga context rga是rk自家的绘图库,绘图效率高于OpenCV
    rga_buffer_t src;
    rga_buffer_t dst;
    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    im_rect src_rect;
    im_rect dst_rect;
    memset(&src_rect, 0, sizeof(src_rect));
    memset(&dst_rect, 0, sizeof(dst_rect));

    scale_w = (float) net_width / img_width;
    scale_h = (float) net_height / img_height;

    // You may not need resize when src resulotion equals to dst resulotion
    // void *resize_buf = nullptr;
    // 如果输入图像不是指定格式
    //void* net_input_buffer = nullptr;
    
    if (img_width !=  net_width || img_height !=  net_height)
    {
    	//printf("using my fun");

        //memset(net_input_buffer, 0x00,  net_height *  net_width *  channel);
        src = wrapbuffer_virtualaddr((void *)img.data, img_width, img_height, RK_FORMAT_RGB_888);
        dst = wrapbuffer_virtualaddr((void *)net_input_buffer, net_width, net_height, RK_FORMAT_RGB_888);
        ret = imcheck(src, dst, src_rect, dst_rect);
        if (IM_STATUS_NOERROR !=  ret)
        {
            printf("%d, check error! %s", __LINE__, imStrError((IM_STATUS) ret));
            exit(-1);
        }
        IM_STATUS STATUS = imresize(src, dst);

        cv::Mat resize_img(cv::Size( net_width,  net_height), CV_8UC3, net_input_buffer);
        cv::imwrite("resize_pre_img.jpg", resize_img);
        return;
        inputs[0].buf = net_input_buffer;
    }
    else
    {
        memcpy(net_input_buffer, (void *)img.data, net_input_size);
        //inputs[0].buf = (void *)img.data;
    }

    inputs[0].buf = net_input_buffer;   
}


//...............
void RKNNDetector::PrepImageLetterbox(const cv::Mat& ori_img)
{
    struct timeval time;
    cv::Mat img;  
    rga_buffer_t src;
    rga_buffer_t dst;
    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    im_rect src_rect;
    im_rect dst_rect;
    memset(&src_rect, 0, sizeof(src_rect));
    memset(&dst_rect, 0, sizeof(dst_rect));

    int img_width = ori_img.cols;
    int img_height = ori_img.rows;

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t1 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
#endif

    // 转换
    cv::cvtColor(ori_img, img, cv::COLOR_BGR2RGB);

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t2 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
    printf("cvtColor time = %ld ms\n", t2 - t1);
#endif

    int size = std::max(std::max(img_width, img_height), net_width);
    cv::Mat img1(size, size, CV_8UC3, cv::Scalar(0, 0, 0));

 	if(size != net_width)
	{
		if(img_width > img_height)
		{
			x_offset = 0;
			y_offset = (img_width - img_height) /2 ;
		}
		else
		{
			x_offset = (img_height - img_width) /2;
			y_offset = 0;
		}
	} 
    else
    {
        x_offset = (net_width - img_width)/2;
        y_offset = (net_height - img_height)/2;
    }

    img.copyTo(img1(cv::Rect(x_offset, y_offset, img.cols, img.rows)));
    //cv::imwrite("img1.jpg", img1);

    // width是模型需要的输入宽度, img_width是图片的实际宽度
    scale_w = (float) net_width / size;
    scale_h = (float) net_height / size;
    // printf("xy_offset：%d %d\n", x_offset, y_offset);

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t3 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
    printf("copyTo time = %ld ms\n", t3 - t2);
#endif

    // You may not need resize when src resulotion equals to dst resulotion
    
    if (size != net_width || size != net_height)
    {
        //src = wrapbuffer_virtualaddr((void *)img1.data, size, size, RK_FORMAT_RGB_888);
        //dst = wrapbuffer_virtualaddr((void *)net_input_buffer, net_width, net_height, RK_FORMAT_RGB_888);
        //ret = imcheck(src, dst, src_rect, dst_rect);
        //if (IM_STATUS_NOERROR !=  ret)
         // {
          //printf("%d, check error! %s", __LINE__, imStrError((IM_STATUS) ret));
          //exit(-1);
          //}
        //IM_STATUS STATUS = imresize(src, dst);

        cv::Mat resize_img;
        cv::resize(img1, resize_img, cv::Size( net_width,  net_height));
        //cv::imwrite("resize_img0000.jpg", resize_img);
	memcpy(net_input_buffer, (void *)resize_img.data, net_input_size);
        inputs[0].buf = net_input_buffer;
    }
    else
    {
        //printf("pppppppppppppppppppppppppp");
        memcpy(net_input_buffer, (void *)img1.data, net_input_size);}

    inputs[0].buf = net_input_buffer;
    //memset(net_input_buffer, 0x00,  net_height *  net_width *  channel);
    //cv::Mat resize_img(cv::Size( net_width,  net_height), CV_8UC3, net_input_buffer);
    //cv::imwrite("resize_prebox_img.jpg", resize_img);

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t4 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
    printf("resize time = %ld ms\n", t4 - t3);
#endif
}


void RKNNDetector::PrepImageLetterboxRga(const cv::Mat& ori_img)
{
    struct timeval time;
    cv::Mat img;  
    rga_buffer_t src;
    rga_buffer_t dst;
    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    im_rect src_rect;
    im_rect dst_rect;
    memset(&src_rect, 0, sizeof(src_rect));
    memset(&dst_rect, 0, sizeof(dst_rect));

    rga_buffer_t src_handle, dst_handle;

    int img_width = ori_img.cols;
    int img_height = ori_img.rows;

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t1 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
#endif
printf("cvtColor start\n ");
    // rga_buffer_t src_img, dst_img;
    // memset(&src, 0, sizeof(src_img));
    // memset(&cvtdst, 0, sizeof(cvtdst));
    
    

    int dst_buf_size = img_width * img_height * 3;
    void* dst_buf = malloc(dst_buf_size);
    memset(dst_buf, 0x00, img_width * img_height * 3);

    src = wrapbuffer_virtualaddr((void *)ori_img.data, img_width, img_height, RK_FORMAT_BGR_888);
    dst = wrapbuffer_virtualaddr((void *)dst_buf, img_width, img_height, RK_FORMAT_RGB_888);
    ret = imcheck(src, dst, src_rect, dst_rect);
    if (IM_STATUS_NOERROR != ret) 
    {
        printf("%d, check error! %s", __LINE__, imStrError((IM_STATUS)ret));
        exit(-1);
    }
    ret = imcvtcolor(src, dst, RK_FORMAT_BGR_888, RK_FORMAT_RGB_888);
    //if (IM_STATUS_NOERROR == ret) 
    //{
    //    printf("%d, check noerror! %s", __LINE__, imStrError((IM_STATUS)ret));
        //exit(-1);
    //}

    // 转换
    //cv::cvtColor(ori_img, img, cv::COLOR_BGR2RGB);

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t2 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
    printf("cvtColor time = %ld ms\n", t2 - t1);
#endif

    int size = std::max(std::max(img_width, img_height), net_width);
    int dst_buf_size2 = size * size * 3;
    void* dst_buf2 = malloc(dst_buf_size2);

 	if(size != net_width)
	{
		if(img_width > img_height)
		{
			x_offset = 0;
			y_offset = (img_width - img_height) /2 ;
		}
		else
		{
			x_offset = (img_height - img_width) /2;
			y_offset = 0;
		}
	} 
    else
    {
        x_offset = (net_width - img_width)/2;
        y_offset = (net_height - img_height)/2;
    }

    // cv::Mat img1(size, size, CV_8UC3, cv::Scalar(0, 0, 0));
    // img.copyTo(img1(cv::Rect(x_offset, y_offset, img.cols, img.rows)));
    //cv::Mat img1(cv::Size( size,size), CV_8UC3, dst_buf);
    //cv::imwrite("img1.jpg", img1);

    src = wrapbuffer_virtualaddr((void *)dst_buf, img_width, img_height, RK_FORMAT_RGB_888);
    dst = wrapbuffer_virtualaddr((void *)dst_buf2, size, size, RK_FORMAT_RGB_888);
    memset(&src_rect, 0, sizeof(src_rect));
    memset(&dst_rect, 0, sizeof(dst_rect));
    src_rect.x = 0;
    src_rect.y = 0;
    src_rect.width = img_width;
    src_rect.height = img_height;
    dst_rect.x = x_offset;
    dst_rect.y = y_offset;
    dst_rect.width = img_width;
    dst_rect.height = img_height;
    ret = imcheck(src, dst, src_rect, dst_rect);
    if (IM_STATUS_NOERROR != ret) 
    {
        printf("%d, check error! %s", __LINE__, imStrError((IM_STATUS)ret));
        exit(-1);
    }
    ret = improcess(src, dst, {}, src_rect, dst_rect, {}, IM_SYNC);
    //if (IM_STATUS_NOERROR == ret) 
    //{
    //    printf("%d, check error! %s", __LINE__, imStrError((IM_STATUS)ret));
        //exit(-1);
   // }

    //cv::Mat pad_img(cv::Size( size,size), CV_8UC3, dst_buf2);
    //cv::imwrite("pad.jpg", pad_img);

    // width是模型需要的输入宽度, img_width是图片的实际宽度
    scale_w = (float) net_width / size;
    scale_h = (float) net_height / size;
    // printf("xy_offset：%d %d\n", x_offset, y_offset);

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t3 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
    printf("copyTo time = %ld ms\n", t3 - t2);
#endif

    src = wrapbuffer_virtualaddr((void *)dst_buf2, size, size, RK_FORMAT_RGB_888);
    dst = wrapbuffer_virtualaddr((void *)net_input_buffer, net_width, net_height, RK_FORMAT_RGB_888);
    memset(&src_rect, 0, sizeof(src_rect));
    memset(&dst_rect, 0, sizeof(dst_rect));
    ret = imcheck(src, dst, src_rect, dst_rect);
    if (IM_STATUS_NOERROR !=  ret)
    {
        printf("%d, check error! %s", __LINE__, imStrError((IM_STATUS) ret));
        exit(-1);
    }
    IM_STATUS STATUS = imresize(src, dst);
    //if (IM_STATUS_NOERROR ==  ret)
    //{
        //printf("%d, check error! %s", __LINE__, imStrError((IM_STATUS) ret));
        //exit(-1);
    //}
    printf("x_offset: %d, y_offset: %d, img_width: %d, img_height: %d, net_width: %d, net_height: %d, size: %d\n", x_offset, y_offset, img_width, img_height, net_width, net_height, size);
    //cv::Mat resize_img(cv::Size( net_width,  net_height), CV_8UC3, net_input_buffer);
    //cv::imwrite("resize_img.jpg", resize_img);

    inputs[0].buf = net_input_buffer;

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t4 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
    printf("resize time = %ld ms\n", t4 - t3);
#endif
    free(dst_buf);
    free(dst_buf2);


}

void RKNNDetector::PrepImage_resize_Letterbox(const cv::Mat& ori_img)
{
    struct timeval time;
    cv::Mat img;  
    rga_buffer_t src;
    rga_buffer_t dst;
    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    im_rect src_rect;
    im_rect dst_rect;
    memset(&src_rect, 0, sizeof(src_rect));
    memset(&dst_rect, 0, sizeof(dst_rect));

    int img_width = ori_img.cols;
    int img_height = ori_img.rows;

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t1 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
#endif

    // 转换
    cv::cvtColor(ori_img, img, cv::COLOR_BGR2RGB);

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t2 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
    printf("cvtColor time = %ld ms\n", t2 - t1);
#endif

    int maxsize = std::max(img_width, img_height);

    cv::Mat resize_img;
    scale = (float) net_width / img_width;
    int target_width = net_width;
    int target_height = img_height * scale;
    
    if (maxsize != net_width)
    {
        //src = wrapbuffer_virtualaddr((void *)img.data, img_width, img_height, RK_FORMAT_RGB_888);
        //dst = wrapbuffer_virtualaddr((void *)resize_img.data, target_width, target_height , RK_FORMAT_RGB_888);
        //ret = imcheck(src, dst, src_rect, dst_rect);
        //if (IM_STATUS_NOERROR !=  ret)
         //{
            //printf("%d, check error! %s", __LINE__, imStrError((IM_STATUS) ret));
          //exit(-1);
          //}
        //IM_STATUS STATUS = imresize(src, dst);
        cv::resize(img, resize_img, cv::Size(), scale, scale);
    }
    else
    {
        printf("pppppppppppppppppppppppppp");
        memcpy(net_input_buffer, (void *)img.data, net_input_size);}

    //inputs[0].buf = net_input_buffer;
    //cv::Mat resize_img(cv::Size(net_width,  net_height / img_width * img_height ), CV_8UC3, net_input_buffer);
    //cv::imwrite("resize_prebox_img.jpg", resize_img);

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t3 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
    printf("resize time = %ld ms\n", t3 - t2);
#endif
    int minsize = std::min(target_width, target_height);
    
    cv::Mat img1(target_width, target_width, CV_8UC3, cv::Scalar(128, 128, 128));
 	if(minsize != net_width)
	{
		if(target_width > target_height)
		{
			x_offset = 0;
			y_offset = (target_width - target_height) /2 ;
		}
		else
		{
			x_offset = (target_height - target_width) /2;
			y_offset = 0;
		}
	} 
    else
    {
        x_offset = (net_width - target_width)/2;
        y_offset = (net_height - target_height)/2;
    }

    resize_img.copyTo(img1(cv::Rect(x_offset, y_offset, resize_img.cols, resize_img.rows)));
    //cv::imwrite("img1.jpg", img1);
    inputs[0].buf = img1.data;

    // width是模型需要的输入宽度, img_width是图片的实际宽度
    scale_w = scale;
    scale_h = scale;
    //scale_w = (float) net_width /  img_width ;
    //scale_h = (float) net_height /  img_height ;
    //printf("xy_offset：%d %d\n", x_offset, y_offset);

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t4 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
    printf("copyTo time = %ld ms\n", t4 - t3);
#endif


}


int RKNNDetector::Process(cv::Mat& ori_img, bool isdraw)
{ 
    struct timeval time;
    int img_width = ori_img.cols;
    int img_height = ori_img.rows;

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t1 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
#endif

    memset(net_input_buffer, 0x00, net_input_size);
    
    
    //PrepImage(ori_img);
    PrepImageLetterbox(ori_img);
    //PrepImageLetterboxRga(ori_img);
    // PrepImage_resize_Letterbox(ori_img);

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t2 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
    printf("prep time = %ld ms\n", t2 - t1);
#endif

    // 设置rknn的输入数据
    rknn_inputs_set(ctx,  io_num.n_input,  inputs);

    // 设置输出
    rknn_output outputs[io_num.n_output];
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i <  io_num.n_output; i++)
        outputs[i].want_float = 0;

    // 调用npu进行推演
    ret = rknn_run(ctx, NULL);
    // 获取npu的推演输出结果
    ret = rknn_outputs_get(ctx,  io_num.n_output, outputs, NULL);

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t3 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
    printf("infer time = %ld ms\n", t3 - t2);
#endif

    // post process
    //detect_result_group_t detect_result_group;
    
    // if(nosigmoid)
    if(1)
        post_process_nosigmoid((int8_t *)outputs[0].buf, (int8_t *)outputs[1].buf, (int8_t *)outputs[2].buf, net_height,  net_width,
                 conf_thresh, nms_thresh, scale_w, scale_h, object_class_num, out_zps, out_scales, &detect_result_group);
                 
        //post_process_nosigmoid_addlayer((int8_t *)outputs[0].buf, (int8_t *)outputs[1].buf, (int8_t *)outputs[2].buf,  (int8_t *)outputs[3].buf, net_height,  net_width, conf_thresh, nms_thresh, scale_w, scale_h, object_class_num, out_zps, out_scales, &detect_result_group);
    else
        post_process((int8_t *)outputs[0].buf, (int8_t *)outputs[1].buf, (int8_t *)outputs[2].buf,  net_height,  net_width,
                 conf_thresh, nms_thresh, scale_w, scale_h, object_class_num, out_zps, out_scales, &detect_result_group);

#if defined(PRINT_TIME)
    gettimeofday(&time, nullptr);
    auto t4 = time.tv_sec * 1000 + time.tv_usec / 1000; //ms
    printf("post time = %ld ms\n", t4 - t3);
#endif

    // if(isdraw)
    printf("detect_result_group.count:%d\n", detect_result_group.count);
    if(0)
    {
        // Draw Objects
        char text[256];
        //printf("detect_result_group.count:%d\n", detect_result_group.count);
        for (int i = 0; i < detect_result_group.count; i++)
        {
            detect_result_t *det_result = &(detect_result_group.results[i]);
            int idx = det_result->class_idx;
            sprintf(text, "%s: %.2f", labels[idx].c_str(), det_result->prop);
            //printf("%s @ (%d %d %d %d) %f\n", labels[idx].c_str(), det_result->box.left, det_result->box.top,det_result->box.right, det_result->box.bottom, det_result->prop);
            
            int x1 = det_result->box.left - x_offset;
			int y1 = det_result->box.top - y_offset;
			int x2 = det_result->box.right - x_offset;
			int y2 = det_result->box.bottom - y_offset;
            rectangle(ori_img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 2);
            putText(ori_img, text, cv::Point(x1, y1), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
        }
    }

    ret = rknn_outputs_release( ctx,  io_num.n_output, outputs);

    //deinitPostProcess();

    return 0;
}

}

