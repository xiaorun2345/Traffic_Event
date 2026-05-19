#include "MppDecoder.h"

int MppDecoder::Init(std::string camera_uri, int width, int height, CODEC_TYPE codec_type)
{
    //ffmpeg 初始化
    // AVDictionary *options = NULL;
    // av_register_all();  //函数在ffmpeg4.0以上版本已经被废弃，所以4.0以下版本就需要注册初始函数
    camera_uri_ = camera_uri;
    avformat_network_init();
    
    pFormatCtx = avformat_alloc_context(); //用来申请AVFormatContext类型变量并初始化默认参数,申请的空间

    int ret_s = StartStream();
    if(ret_s <0)
        return ret_s;

    unsigned i = 0;
    for (i = 0; i<pFormatCtx->nb_streams; i++)
    {
        if (pFormatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO)
        {
            videoindex = i;
            break;
        }        
    }
    if (videoindex == -1)
    {
        printf("Didn't find a video stream.\n");
        return -1;
    }
    std::cout << "videoindex: " << videoindex << std::endl;

    yuv_data = new uint8_t[width*height*3/2];

    av_packet = (AVPacket *)av_malloc(sizeof(AVPacket)); // 申请空间，存放的每一帧数据 （h264、h265）

    int ret = InitMpp(codec_type);

    return ret;
}

int MppDecoder::StartStream()
{
    if(camera_uri_ == "")
    {
        std::cout<< "camera_uri_ is null !!!" << std::endl;
        return -1;
    }
        
    av_dict_set(&options, "buffer_size", "1024000", 0); //设置缓存大小,1080p可将值跳到最大
    av_dict_set(&options, "rtsp_transport", "tcp", 0); //以tcp的方式打开,
    av_dict_set(&options, "stimeout", "5000000", 0); //设置超时断开链接时间，单位us
    av_dict_set(&options, "max_delay", "500000", 0); //设置最大时延
    while (1)
    {
        
        //打开网络流或文件流
        if (avformat_open_input(&pFormatCtx, camera_uri_.c_str(), NULL, &options) != 0)
        {
            printf("Couldn't open input stream.\n");
            usleep(3000*1000);  //等待3秒钟
            continue;
        }

        //获取视频文件信息
        if (avformat_find_stream_info(pFormatCtx, NULL)<0)
        {
            printf("Couldn't find stream information.\n");
            usleep(3000*1000);  //等待3秒钟
            continue;
        }

        break;
    }
    
    av_dump_format(pFormatCtx, 0, camera_uri_.c_str(), 0);
    return 0;
}

int MppDecoder::InitMpp(CODEC_TYPE codec_type)
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

MppDecoder::~MppDecoder()
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

    av_free(av_packet);
    avformat_close_input(&pFormatCtx);
}

int MppDecoder::mpp_decode(CALLBACK_FUNC func)
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

    ret = mpp_packet_init(&packet, av_packet->data, av_packet->size);
    mpp_packet_set_pts(packet, av_packet->pts);

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
            printf("#####xxxxxxx######");
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
                        func(base, h_stride, v_stride, width, height, timestamp);    
                    }
                }
                frm_eos = mpp_frame_get_eos(frame);
                mpp_frame_deinit(&frame);

                frame = NULL;
                get_frm = 1;
            }else{
                std::cerr << "error , frame is empty , please check config GetMatMode is right ???" << std::endl;
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

void MppDecoder::Decoding(CALLBACK_FUNC func)
{
    // int count = 0;
    //这边可以调整i的大小来改变文件中的视频时间,每 +1 就是一帧数据
    while (1)
    {
        if (av_read_frame(pFormatCtx, av_packet) >= 0)
        {
            if (av_packet->stream_index == videoindex)
            {
                printf("--------------\ndata size is: %d\n-------------", av_packet->size);
                mpp_decode(func);
            }
            if (av_packet != NULL)
                av_packet_unref(av_packet);
            // printf("%d", i);
        }else{
            std::cout << "Retry..." << std::endl;
            usleep(1000*1000);  //等待1秒钟
            avformat_close_input(&pFormatCtx);
            while(1)
            {
                int ret = StartStream();
                if(ret != -1)
                    break;
            }

        }
    }   
}

// void YUV420SP2Mat(MppFrame  frame, cv::Mat rgbImg ) {
void ConvertYuvData(MppFrame frame, int& size) 
{
	RK_U32 width = 0;
	RK_U32 height = 0;
	RK_U32 h_stride = 0;
	RK_U32 v_stride = 0;

	MppBuffer buffer = NULL;
	RK_U8 *base = NULL;

	width = mpp_frame_get_width(frame);
	height = mpp_frame_get_height(frame);
	h_stride = mpp_frame_get_hor_stride(frame);
	v_stride = mpp_frame_get_ver_stride(frame);
	buffer = mpp_frame_get_buffer(frame);

	base = (RK_U8 *)mpp_buffer_get_ptr(buffer);
	RK_U32 buf_size = mpp_frame_get_buf_size(frame);
	size_t base_length = mpp_buffer_get_size(buffer);
	// printf("base_length = %d\n",base_length);
    // printf("width=%d, height=%d, h_stride=%d, v_stride=%d\n", width, height, h_stride, v_stride);

	RK_U32 i;
	RK_U8 *base_y = base;
	RK_U8 *base_c = base + h_stride * v_stride;

	cv::Mat yuvImg;
	yuvImg.create(height * 3 / 2, width, CV_8UC1);

	//转为YUV420p格式
	int idx = 0;
	for (i = 0; i < height; i++, base_y += h_stride) 
    {
        memcpy(yuvImg.data +idx, base_y, width);
		idx += width;
	}
	for (i = 0; i < height / 2; i++, base_c += h_stride) 
    {
		memcpy(yuvImg.data + idx, base_c, width);
		idx += width;
	}

    //yuv转bgr, 返回yuv的格式
    // MppFrameFormat fmt = mpp_frame_get_fmt(frame);
    // cv::Mat rgbImg;
    // if(fmt==MPP_FMT_YUV420SP)  //NV21 rtsp流给的是NV12
    //     cv::cvtColor(yuvImg, rgbImg, cv::COLOR_YUV2BGR_NV12);
    // else if(fmt==MPP_FMT_YUV420SP_VU) //NV12
    //     cv::cvtColor(yuvImg, rgbImg, cv::COLOR_YUV2BGR_NV21);
    // else if(fmt==MPP_FMT_YUV420P) //I420
    //     cv::cvtColor(yuvImg, rgbImg, cv::COLOR_YUV2BGR_I420);
    
    // cv::imwrite("./result/"+std::to_string(count++)+".jpg", rgbImg);
}

size_t MppDecoder::mpp_buffer_group_usage(MppBufferGroup group)
{
    if (NULL == group)
    {
        mpp_err_f("input invalid group %p\n", group);
        return MPP_BUFFER_MODE_BUTT;
    }

    MppBufferGroupImpl *p = (MppBufferGroupImpl *)group;
    return p->usage;
}


void MppDecoder::dump_mpp_frame_to_file(MppFrame frame, FILE *fp)
{
    RK_U32 width    = 0;
    RK_U32 height   = 0;
    RK_U32 h_stride = 0;
    RK_U32 v_stride = 0;

    MppBuffer buffer    = NULL;
    RK_U8 *base = NULL;

    width    = mpp_frame_get_width(frame);
    height   = mpp_frame_get_height(frame);
    h_stride = mpp_frame_get_hor_stride(frame);
    v_stride = mpp_frame_get_ver_stride(frame);
    buffer   = mpp_frame_get_buffer(frame);

    base = (RK_U8 *)mpp_buffer_get_ptr(buffer);
    RK_U32 buf_size = mpp_frame_get_buf_size(frame);
    size_t base_length = mpp_buffer_get_size(buffer);
    printf("base_length = %d\n",base_length);

    RK_U32 i;
    RK_U8 *base_y = base;
    RK_U8 *base_c = base + h_stride * v_stride;

    //保存为YUV420sp格式
    /*for (i = 0; i < height; i++, base_y += h_stride)
    {
        fwrite(base_y, 1, width, fp);
    }
    for (i = 0; i < height / 2; i++, base_c += h_stride)
    {
        fwrite(base_c, 1, width, fp);
    }*/

    //保存为YUV420p格式
    for(i = 0; i < height; i++, base_y += h_stride)
    {
        fwrite(base_y, 1, width, fp);
    }
    for(i = 0; i < height * width / 2; i+=2)
    {
        fwrite((base_c + i), 1, 1, fp);
    }
    for(i = 1; i < height * width / 2; i+=2)
    {
        fwrite((base_c + i), 1, 1, fp);
    }
}
