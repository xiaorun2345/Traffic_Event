#ifndef LIVERTSPCLIENT_MPPDECODE_H
#define LIVERTSPCLIENT_MPPDECODE_H

#include <string.h>

// #include "utils.h"
// #include "rk_mpi.h"
// #include "mpp_log.h"
// #include "mpp_mem.h"
// #include "mpp_env.h"
// #include "mpp_common.h"

// #include "mpp_frame.h"
// #include "mpp_buffer_impl.h"
// #include "mpp_frame_impl.h"


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

#include <opencv2/opencv.hpp>
#include <iostream>

/*Include ffmpeg header file*/
#ifdef __cplusplus
extern "C" {
#endif

    #include <libavformat/avformat.h>
    #include <libavutil/imgutils.h>
    #include <libswscale/swscale.h>

#ifdef __cplusplus
};
#endif


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



// size_t mpp_frame_get_buf_size(const MppFrame s);
// void dump_mpp_frame_to_file(MppFrame frame, FILE *fp);
// size_t mpp_buffer_group_usage(MppBufferGroup group);

// int decode_simple(MpiDecLoopData *data, AVPacket* av_packet);

// void YUV420SP2Mat(MppFrame  frames, cv::Mat rgbImg );
// void YUV420SP2Mat(MppFrame  frames);

enum CODEC_TYPE{H264, H265};

typedef void (*CALLBACK_FUNC)(unsigned char *, int, int, int, int, uint64_t);

class MppDecoder
{
public:
    MppDecoder()=default;
    ~MppDecoder();

    int Init(std::string camera_uri, int width, int height, CODEC_TYPE codec_type);
    void Decoding(CALLBACK_FUNC func);

private:
    size_t mpp_buffer_group_usage(MppBufferGroup group);
    void dump_mpp_frame_to_file(MppFrame frame, FILE *fp);
    int mpp_decode(CALLBACK_FUNC func);
    
    int InitMpp(CODEC_TYPE codec_type);
    int StartStream();

    void ConvertFrame2NV12(MppFrame frame, cv::Mat yuvImg);

private:
    MpiDecLoopData data;
    std::string camera_uri_ = "";
    // MppPacket *packet;
    // MppFrame *frame;
    MppCtx ctx          = NULL;
    MppApi *mpi         = NULL;
    // MppPacket packet    = NULL;
    // MppFrame  frame     = NULL;

    AVDictionary *options = NULL;
    AVFormatContext *pFormatCtx = NULL;
    AVPacket *av_packet = NULL;
    int videoindex = -1;

    uint8_t* yuv_data=nullptr;
};


#endif //LIVERTSPCLIENT_MPPDECODE_H
