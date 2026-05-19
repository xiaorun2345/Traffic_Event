/*
* camera_app.cpp
* Created on: 20240225
* Author: fangshuchen
* Description: 视频解码程序对外接口函数实现
*
*/

#include "video_decoder.h"

namespace camera 
{

void VideoDecoder::PrintVersion()
{
    std::cout << "*** VideoDecoder Ver RK3588 "<< RK3588_DECODER <<" ***" << std::endl;
}

bool VideoDecoder::open()
{
        // 初始化 FFmpeg 库
	av_register_all();
	avformat_network_init();
	format_context = avformat_alloc_context();
    av_dict_set(&options, "buffer_size", "1024000", 0); //设置缓存大小,1080p可将值跳到最大
    av_dict_set(&options, "rtsp_transport", "tcp", 0); //以tcp的方式打开,
    av_dict_set(&options, "stimeout", "5000000", 0); //设置超时断开链接时间，单位us
    av_dict_set(&options, "max_delay", "500000", 0); //设置最大时延
    while (1)
    {
        
        //打开网络流或文件流
        if (avformat_open_input(&format_context, camera_uri_.c_str(), NULL, &options) != 0)
        {
            printf("Couldn't open input stream.\n");
            usleep(3000*1000);  //等待3秒钟
            continue;
        }

        //获取视频文件信息
        if (avformat_find_stream_info(format_context, NULL)<0)
        {
            printf("Couldn't find stream information.\n");
            usleep(3000*1000);  //等待3秒钟
            continue;
        }

        break;
    }

	for (unsigned int i = 0; i < format_context->nb_streams; i++) 
	{
	    if (format_context->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) 
	    {
	        video_stream_index = i;
	        break;
	     }
	}

    if (video_stream_index == -1) 
    {
        std::cerr << "Failed to find video stream." << std::endl;
        avformat_close_input(&format_context);
        return false;
    }
    av_dump_format(format_context, 0, camera_uri_.c_str(), 0);

    codec_context = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(codec_context, format_context->streams[video_stream_index]->codecpar);

    avcodec_open2(codec_context, codec, nullptr);
    
    return true;

}    

bool VideoDecoder::Init(std::string camera_uri, int width, int height, int codec_type)
{
	camera_uri_ = camera_uri;
	codec_type_ = codec_type;
    PrintVersion();
    
    if(codec_type == VIDEO_UNKOWN)
    {
        camera_width = width;
        camera_height = height;
         // video_decode_thread_ = std::thread(&VideoDecoder::MatDealFFmpegThread, this);
    }
    else if(codec_type == H264 || codec_type == H265)
    {

        // 初始化 FFmpeg 库
        av_register_all();
        avformat_network_init();

        // 打开 RTSP 输入流
        format_context = avformat_alloc_context();
        av_dict_set(&options, "buffer_size", "1024000", 0); //设置缓存大小,1080p可将值跳到最大
        av_dict_set(&options, "rtsp_transport", "tcp", 0); //以tcp的方式打开,
        av_dict_set(&options, "stimeout", "5000000", 0); //设置超时断开链接时间，单位us
        av_dict_set(&options, "max_delay", "500000", 0); //设置最大时延
        if (avformat_open_input(&format_context, camera_uri.c_str(), nullptr, &options) < 0) 
        {
            std::cerr << "Failed to open RTSP stream." << std::endl;
            return false;
        }

        // 查找流信息
        if (avformat_find_stream_info(format_context, nullptr) < 0) 
        {
            std::cerr << "Failed to find stream information." << std::endl;
            avformat_close_input(&format_context);
            return false;
        }

        // 查找视频流
        for (unsigned int i = 0; i < format_context->nb_streams; i++) 
        {
            if (format_context->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) 
            {
                video_stream_index = i;
                break;
            }
        }

        if (video_stream_index == -1) 
        {
            std::cerr << "Failed to find video stream." << std::endl;
            avformat_close_input(&format_context);
            return false;
        }


        InitMpp((CODEC_TYPE)codec_type);

        codec = avcodec_find_decoder(format_context->streams[video_stream_index]->codecpar->codec_id);
        codec_context = avcodec_alloc_context3(codec);
        avcodec_parameters_to_context(codec_context, format_context->streams[video_stream_index]->codecpar);

        avcodec_open2(codec_context, codec, nullptr);

        camera_width  = format_context->streams[video_stream_index]->codec->width;
        camera_height = format_context->streams[video_stream_index]->codec->height;

        if (width == camera_width && height == camera_height)
        {
            std::cout << "Camera fps: " << av_q2d(format_context->streams[video_stream_index]->avg_frame_rate)
            << " width: " << format_context->streams[video_stream_index]->codec->width
            << " height: " << format_context->streams[video_stream_index]->codec->height 
            << std::endl;
        }
        else
        {
            std::cerr << "Error: camera width: " << camera_width << " height: " << camera_height 
            << " Input width: " << width << " height: " << height << std::endl;
            return false; 
        }
        datalen = camera_width * camera_height * 3 / 2;
        // mat.create(camera_height, camera_width, CV_8UC3);
        mat.create(camera_height*3/2, camera_width, CV_8UC1);
        // 开启解码线程
        // video_decode_thread_ = std::thread(&VideoDecoder::VideoDecodeThreadFun, this);
    }
    

    return true;
}
// 获取图像线程执行体
void VideoDecoder::MatDealFFmpegThread(CALLBACK_FUNC func)
{
	std::cout << "mat deal by ffmpeg thread start!\n" << std::endl;
	unsigned long long time1, time2;
	// cv::Mat img;

// A:
	cv::VideoCapture cap;

	cap.open(camera_uri_, cv::CAP_FFMPEG);

	int width = (int)cap.get(3); 
    int height = (int)cap.get(4);
    int fps = (int)cap.get(5);
    std::cout << "cap width = " << width << "  height=" << height << "  fps=" << fps << std::endl;

	if(camera_width>width || camera_height>height)
	{
		camera_width = width;
		camera_height = height;
	}
    std::cout << "cap width = " << camera_width << "  height=" << camera_height << "  fps=" << fps << std::endl;
    datalen = camera_width * camera_height *3;
    mat.create(camera_height, camera_width, CV_8UC3);

	while (true)
	{
        // 
        auto start1 = std::chrono::high_resolution_clock::now();
        // mat_mutex_.lock();
        // printf("%s(%d)-<%s>: \n",__FILE__, __LINE__, __FUNCTION__);
        {
            std::lock_guard<std::mutex> mat_lock(mat_mutex_);
            // printf("%s(%d)-<%s>: \n",__FILE__, __LINE__, __FUNCTION__);
            cap >> mat;
            if(!mat.empty())
            {
                auto now = std::chrono::system_clock::now();
                auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
                timestamp_ = static_cast<uint64_t>(milliseconds);
                // printf("MatDealFFmpegThread timestamp_ = %ld\n ",timestamp_);
                cv::Mat img(camera_height, camera_width, CV_8UC3);
                memcpy(img.data, mat.data, datalen);
                func(img,timestamp_);
                // printf("%s(%d)-<%s>: \n",__FILE__, __LINE__, __FUNCTION__);
            }
            
                
        }
        auto end1 = std::chrono::high_resolution_clock::now();
        auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
        std::cout << "get  decoder time: " << duration1.count() << std::endl;
        // mat_mutex_.unlock();
        if (mat.empty())
        {
            std::cout << "camera disconect, then reconect camera ... \n";
            cap.release();
            sleep(5);
            cap.open(camera_uri_, cv::CAP_FFMPEG);
            // goto A;
        }
        

        // usleep(500*1000/fps);
	}
	
	printf("MatDealFFmpegThread thread exit....\n");

	return ;
}

int VideoDecoder::InitMpp(CODEC_TYPE codec_type)
{
    //mpp初始化
    MPP_RET ret         = MPP_OK;
    size_t file_size    = 0;

    MpiCmd mpi_cmd      = MPP_CMD_BASE;
    MppParam param      = NULL;
    RK_U32 need_split   = 1;
    RK_U32 width        = width;
    RK_U32 height       = height;
    MppCodingType type;
    if (H264==codec_type)
        type = MPP_VIDEO_CodingAVC;
    else if(H265==codec_type)
        type = MPP_VIDEO_CodingHEVC;
    else
        type = MPP_VIDEO_CodingRV;
    
    size_t packet_size  = 8*1024;
    MppBuffer pkt_buf   = NULL;
    MppBuffer frm_buf   = NULL;

    memset(&data, 0, sizeof(data));
    ret = mpp_create(&ctx, &mpi);

    if (MPP_OK != ret) 
    {
        printf("mpp_create failed\n");
        // deInit(&packet, &frame, ctx, buf, data);
        return -1;
    }

    mpi_cmd = MPP_DEC_SET_PARSER_SPLIT_MODE;
    param = &need_split;
    ret = mpi->control(ctx, mpi_cmd, param);
    if (MPP_OK != ret) 
    {
        printf("mpi->control failed\n");
        // deInit(&packet, &frame, ctx, buf, data);
        return -1;
    }

    mpi_cmd = MPP_SET_INPUT_BLOCK;
    param = &need_split;
    ret = mpi->control(ctx, mpi_cmd, param);
    if (MPP_OK != ret) 
    {
        printf("mpi->control failed\n");
        // deInit(&packet, &frame, ctx, buf, data);
        return -1;
    }

    ret = mpp_init(ctx, MPP_CTX_DEC, type);
    if (MPP_OK != ret) 
    {
        printf("mpp_init failed\n");
        // deInit(&packet, &frame, ctx, buf, data);
        return -1;
    }

    data.ctx            = ctx;
    data.mpi            = mpi;
    data.eos            = 0;
    data.packet_size    = packet_size;
    // data.frame          = frame;
    data.frame_count    = 0;

    printf("init mpp done..\n");
    return 0;
}

void VideoDecoder::Run(CALLBACK_FUNC func)
{
    printf(" codec_type_ = %d \n",codec_type_);
     if(codec_type_ == H264 || codec_type_ == H265)
    {
        VideoDecodeThreadFun(func);
    }else{
        MatDealFFmpegThread(func);
    }
}

// 解码线程
void VideoDecoder::VideoDecodeThreadFun(CALLBACK_FUNC func)
{
    std::cout << "VideoDecoder start decodethread func." << std::endl;

    int frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    // 解码 RTSP 流并写入 MP4 文件
    AVPacket packet;
    av_init_packet(&packet);
    
    while (true)
    {	    
        while (av_read_frame(format_context, &packet) >= 0) 
        {
            if (packet.stream_index == video_stream_index) 
            {
                AVFrame* frame = av_frame_alloc();
                int ret = avcodec_send_packet(codec_context, &packet);
                if (ret < 0) 
                {
                    std::cerr << "Error sending a packet for decoding" << std::endl;
                    return;
                }
                while (ret >= 0) 
                {
                    auto start1 = std::chrono::high_resolution_clock::now();
                    ret = avcodec_receive_frame(codec_context, frame);
                    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                        break;
                    else if (ret < 0) 
                    {
                        std::cerr << "Error during decoding" << std::endl;
                        return;
                    }
            
                    mpp_decode(packet,func);

                    auto end1 = std::chrono::high_resolution_clock::now();
                    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
                    std::cout << "get time: " << duration1.count() << std::endl;
                    
                    frame_count++;
                    auto current_time = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
                    if(duration >= 10000)
                    {
                        double fps = static_cast<double>(frame_count) / duration * 1000;
                        std::cout << "VideoDecoder FPS: " << fps << std::endl;
                        frame_count = 0;
                        start_time = std::chrono::high_resolution_clock::now();
                    }
                }
                av_frame_free(&frame);
            }
            av_packet_unref(&packet);
        }
        avcodec_free_context(&codec_context);
        avformat_close_input(&format_context);
        std::cout << "Videodecoder reopen!!!!!!!!!!!!!!!!!!!!!!!!!!!\n" << std::endl;
        open();
    }

    std::cout << "!!!!!!!!!!!! videodec thread exit" << std::endl;
    Release();
}

int VideoDecoder::mpp_decode(AVPacket av_packet,CALLBACK_FUNC func)
{
    RK_U32 pkt_done = 0;
    RK_U32 pkt_eos  = 0;
    RK_U32 err_info = 0;
    MPP_RET ret = MPP_OK;
    MppCtx ctx  = data.ctx;
    MppApi *mpi = data.mpi;
    MppPacket packet = NULL;
    MppFrame  frame  = NULL;
    size_t read_size = 0;
    size_t packet_size = data.packet_size;

    RK_U32 width = 0;
	RK_U32 height = 0;
	RK_U32 h_stride = 0;
	RK_U32 v_stride = 0;
    MppBuffer buffer = NULL;
	RK_U8 *base = NULL;
    struct timespec ts;

    ret = mpp_packet_init(&packet, av_packet.data, av_packet.size);
    mpp_packet_set_pts(packet, av_packet.pts);

    do {
        RK_S32 times = 5;
        // send the packet first if packet is not done
        if (!pkt_done) {
            ret = mpi->decode_put_packet(ctx, packet);
            if (MPP_OK == ret)
                pkt_done = 1;
        }

        // then get all available frame and release
        do {
            RK_S32 get_frm = 0;
            RK_U32 frm_eos = 0;

            try_again:
            ret = mpi->decode_get_frame(ctx, &frame);
            // printf("#####xxxxxxx######");
            if (MPP_ERR_TIMEOUT == ret) {
            printf("----!!!!------xxxxxxx----!!!!----");
            //if (ret){
                if (times > 0) {
                    times--;
                    msleep(2);
                    goto try_again;
                }
                printf("decode_get_frame failed too much time\n");
            }
            
            if (MPP_OK != ret) {
                printf("decode_get_frame failed ret %d\n", ret);
                break;
            }
            
            if (frame) {
                if (mpp_frame_get_info_change(frame)) {
                    RK_U32 width = mpp_frame_get_width(frame);
                    RK_U32 height = mpp_frame_get_height(frame);
                    RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
                    RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
                    RK_U32 buf_size = mpp_frame_get_buf_size(frame);

                    printf("decode_get_frame get info changed found\n");
                    printf("decoder require buffer w:h [%d:%d] stride [%d:%d] buf_size %d",
                            width, height, hor_stride, ver_stride, buf_size);

                    ret = mpp_buffer_group_get_internal(&data.frm_grp, MPP_BUFFER_TYPE_ION);
                    if (ret) {
                        printf("get mpp buffer group  failed ret %d\n", ret);
                        break;
                    }
                    mpi->control(ctx, MPP_DEC_SET_EXT_BUF_GROUP, data.frm_grp);

                    mpi->control(ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
                } 
                else 
                {
                    err_info = mpp_frame_get_errinfo(frame) | mpp_frame_get_discard(frame);
                    if (err_info) 
                    {
                        printf("decoder_get_frame get err info:%d discard:%d.\n",
                                mpp_frame_get_errinfo(frame), mpp_frame_get_discard(frame));
                    }
                    data.frame_count++;
                    printf("decode_get_frame get frame %d\n", data.frame_count);
                    // if (data->fp_output && !err_info){
                    if(!err_info) 
                    {
                        // YUV420SP2Mat(frame);
                        width = mpp_frame_get_width(frame);
                        height = mpp_frame_get_height(frame);
                        h_stride = mpp_frame_get_hor_stride(frame);
                        v_stride = mpp_frame_get_ver_stride(frame);
                        buffer = mpp_frame_get_buffer(frame);

                        base = (RK_U8 *)mpp_buffer_get_ptr(buffer);
                        // RK_U32 buf_size = mpp_frame_get_buf_size(frame);                      
                        clock_gettime(CLOCK_REALTIME, &ts);
                        uint64_t timestamp = ts.tv_sec * 1000 + (ts.tv_nsec / 1000000); //ms
                        GetFrameRawData(base, h_stride, v_stride, width, height, timestamp,func);    
                    }
                }
                frm_eos = mpp_frame_get_eos(frame);
                mpp_frame_deinit(&frame);

                frame = NULL;
                get_frm = 1;
            }else{
                // std::cerr << "error , frame is empty , please check config GetMatMode is right ???" << std::endl;
            }

            // try get runtime frame memory usage
            if (data.frm_grp) {
                size_t usage = mpp_buffer_group_usage(data.frm_grp);
                if (usage > data.max_usage)
                    data.max_usage = usage;
            }else{
                // printf("(data.frm_grp) \n");
            }

            // if last packet is send but last frame is not found continue
            if (pkt_eos && pkt_done && !frm_eos) {
                msleep(10);
                continue;
            }

            if (frm_eos) {
                printf("found last frame\n");
                break;
            }

            if (data.frame_num > 0 && data.frame_count >= data.frame_num) {
                data.eos = 1;
                break;
            }

            if (get_frm)
                continue;
            break;
        } while (1);

        if (data.frame_num > 0 && data.frame_count >= data.frame_num) {
            data.eos = 1;
            printf("reach max frame number %d\n", data.frame_count);
            break;
        }

        if (pkt_done)
            break;

        /*
         * why sleep here:
         * mpi->decode_put_packet will failed when packet in internal queue is
         * full,waiting the package is consumed .Usually hardware decode one
         * frame which resolution is 1080p needs 2 ms,so here we sleep 3ms
         * * is enough.
         */
        msleep(3);
    } while (1);
    mpp_packet_deinit(&packet);

    return ret;
}

//rtsp流原始数据为NV12格式
void VideoDecoder::GetFrameRawData(unsigned char *data, int h_stride, int v_stride, int width, int height, uint64_t timestamp,CALLBACK_FUNC func)
{
    // DealRCF deal_rcf;
    int i;
	unsigned char *base_y = data;
	unsigned char *base_c = data + h_stride * v_stride;

    
	//转为Mat
    {	
        std::lock_guard<std::mutex> mat_lock(mat_mutex_);
        auto now = std::chrono::system_clock::now();
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        timestamp_ = static_cast<uint64_t>(milliseconds);
        int idx = 0;
        for (i = 0; i < height; i++, base_y += h_stride) 
        {
            memcpy(mat.data + idx, base_y, width);
            idx += width;
        }
        for (i = 0; i < height / 2; i++, base_c += h_stride) 
        {
            memcpy(mat.data + idx, base_c, width);
            idx += width;
        }
        printf("GetFrameRawData timestamp_ = %ld\n ",timestamp_);
    }

    cv::Mat rgbImg;
    cv::cvtColor(mat, rgbImg, cv::COLOR_YUV2BGR_NV12);
    func(rgbImg,timestamp_);
    

}

// 获取一帧图像
void VideoDecoder::GetImg(cv::Mat img, uint64_t& timestamp)
{
    while (true)
    {
        // printf("%s(%d)-<%s>: \n",__FILE__, __LINE__, __FUNCTION__);
        if (timestamp_ != 0)
        {
            // printf("%s(%d)-<%s>: \n",__FILE__, __LINE__, __FUNCTION__);
            
            {
                std::lock_guard<std::mutex> mat_lock(mat_mutex_);
                if(codec_type_ != (int)VIDEO_UNKOWN)
                {
                    // printf("%s(%d)-<%s>: \n",__FILE__, __LINE__, __FUNCTION__);
                    // cv::Mat rgbImg;
                    cv::cvtColor(mat, img, cv::COLOR_YUV2BGR_NV12);
                    // cv::imwrite("VideoDecoder_mat.jpg",mat);
                    // cv::imwrite("VideoDecoder_rgbImg.jpg",img);
                    // sleep(1);
                    // exit(1);
                    // memcpy(img.data, rgbImg.data, datalen);
                    // printf("%s(%d)-<%s>: \n",__FILE__, __LINE__, __FUNCTION__);
                }else
                {
                    // printf("%s(%d)-<%s>: \n",__FILE__, __LINE__, __FUNCTION__);
                    memcpy(img.data, mat.data, datalen);
                    // printf("%s(%d)-<%s>: \n",__FILE__, __LINE__, __FUNCTION__);
                }
                // printf("%s(%d)-<%s>: \n",__FILE__, __LINE__, __FUNCTION__);
                timestamp = timestamp_;
		        //std::cout << "get onr img, time: " << timestamp_ << " size: " << img.cols << " " << img.rows << std::endl;
                timestamp_ = 0;
            }
            break;
        }
        else
        {
            //std::cout << "VideoDecoder: img empty wait 10ms." << std::endl;
            usleep(1000);
        }
    }
}


void VideoDecoder::Release()
{
        // if (av_packet) 
    // {
    //     mpp_packet_deinit(av_packet);
    //     av_packet = NULL;
    // }

    // if (frame) 
    // {
    //     mpp_frame_deinit(frame);
    //     frame = NULL;
    // }

    if (ctx) 
    {
        mpp_destroy(ctx);
        ctx = NULL;
    }

    if (data.pkt_grp) 
    {
        mpp_buffer_group_put(data.pkt_grp);
        data.pkt_grp = NULL;
    }

    if (data.frm_grp) 
    {
        mpp_buffer_group_put(data.frm_grp);
        data.frm_grp = NULL;
    }

    if (data.fp_output) 
    {
        fclose(data.fp_output);
        data.fp_output = NULL;
    }

    if (data.fp_input) 
    {
        fclose(data.fp_input);
        data.fp_input = NULL;
    }

    // av_free(av_packet);
    avformat_close_input(&format_context);

    std::cout << "VideoDecoder release." << std::endl;
}

VideoDecoder::~VideoDecoder()
{
        // if (av_packet) 
    // {
    //     mpp_packet_deinit(av_packet);
    //     av_packet = NULL;
    // }

    // if (frame) 
    // {
    //     mpp_frame_deinit(frame);
    //     frame = NULL;
    // }

    if (ctx) 
    {
        mpp_destroy(ctx);
        ctx = NULL;
    }

    if (data.pkt_grp) 
    {
        mpp_buffer_group_put(data.pkt_grp);
        data.pkt_grp = NULL;
    }

    if (data.frm_grp) 
    {
        mpp_buffer_group_put(data.frm_grp);
        data.frm_grp = NULL;
    }

    if (data.fp_output) 
    {
        fclose(data.fp_output);
        data.fp_output = NULL;
    }

    if (data.fp_input) 
    {
        fclose(data.fp_input);
        data.fp_input = NULL;
    }

    // av_free(av_packet);
    avformat_close_input(&format_context);

    std::cout << "VideoDecoder release." << std::endl;
}


} // namespace camera

