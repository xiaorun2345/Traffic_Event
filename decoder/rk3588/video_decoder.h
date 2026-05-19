/*
* camera_app.h
* Created on: 20240225
* Author: fangshuchen
* Description: 视频解码程序对外接口函数定义
*
*/
#ifndef __VIDEO_DECODER__
#define __VIDEO_DECODER__

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <mutex>
#include <thread>
#include <unistd.h>
#define  DEC_MODEL 2

#define RK3588_DECODER 1.2
    
extern "C"{
    #include "rockchip/mpp_common.h"
    #include "rockchip/mpp_time.h"
    #include "rockchip/mpp_env.h"
    #include "rockchip/mpp_mem.h"
    #include "rockchip/mpp_frame.h"
    #include "rockchip/mpp_buffer_impl.h"
    #include "rockchip/mpp_frame_impl.h"
    #include "rockchip/rk_mpi.h"
    #include "rockchip/utils.h"
};


extern "C"
{
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libavutil/imgutils.h>
}

typedef void (*CALLBACK_FUNC)(cv::Mat& , uint64_t& );

namespace camera 
{


    

    #define MPI_DEC_STREAM_SIZE         (SZ_4K)
    #define MPI_DEC_LOOP_COUNT          4
    #define MAX_FILE_NAME_LENGTH        256

    typedef struct
    {
        MppCtx          ctx;
        MppApi          *mpi;
        RK_U32          eos;
        char            *buf;

        MppBufferGroup  frm_grp;
        MppBufferGroup  pkt_grp;
        MppPacket       packet;
        size_t          packet_size;
        MppFrame        frame;

        FILE            *fp_input;
        FILE            *fp_output;
        RK_S32          frame_count;
        RK_S32          frame_num;
        size_t          max_usage;
    } MpiDecLoopData;

    typedef struct
    {
        char            file_input[MAX_FILE_NAME_LENGTH];
        char            file_output[MAX_FILE_NAME_LENGTH];
        MppCodingType   type;
        MppFrameFormat  format;
        RK_U32          width;
        RK_U32          height;
        RK_U32          debug;

        RK_U32          have_input;
        RK_U32          have_output;

        RK_U32          simple;
        RK_S32          timeout;
        RK_S32          frame_num;
        size_t          max_usage;
    } MpiDecTestCmd;

    enum CODEC_TYPE{VIDEO_UNKOWN, H264, H265};

    class VideoDecoder
    {
    public:
        VideoDecoder() = default;
        ~VideoDecoder();

        // 初始化
        bool Init(std::string camera_uri, int width, int height, int codec_type);

        // 获取一帧图像
        void GetImg(cv::Mat img, uint64_t& timestamp);
        void Run(CALLBACK_FUNC func);
        void VideoDecodeThreadFun(CALLBACK_FUNC func);
        
        void Release();
        bool open();
    private:
        // 打印版本信息
        void PrintVersion();
        int InitMpp(CODEC_TYPE codec_type);
        int mpp_decode(AVPacket av_packet,CALLBACK_FUNC func);
        void GetFrameRawData(unsigned char *data, int h_stride, int v_stride, int width, int height, uint64_t timestamp,CALLBACK_FUNC func);

        
        void MatDealFFmpegThread(CALLBACK_FUNC func);

    private:
        MpiDecLoopData data;
        // MppPacket *packet;
        // MppFrame *frame;
        MppCtx ctx          = NULL;
        MppApi *mpi         = NULL;
        // MppPacket packet    = NULL;
        // MppFrame  frame     = NULL;

        AVDictionary *options = NULL;
        // AVPacket *av_packet = NULL;
        // int videoindex = -1;

        uint8_t* yuv_data=nullptr;

        uint64_t timestamp_;
        std::thread video_decode_thread_;
        std::mutex mat_mutex_;

        AVFormatContext* format_context;
        AVCodec* codec;
        AVCodecContext* codec_context;
        int video_stream_index = -1;
        
        unsigned char *p_rgb = NULL;
        unsigned char *d_src = NULL;

        int camera_width, camera_height, datalen, codec_type_;
        std::string camera_uri_;

        cv::Mat mat;

    }; //class VideoDecoder

} // namespace camera
#endif
