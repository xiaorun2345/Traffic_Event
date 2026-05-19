#include <DealRCF.h>

int main(int argc, char *argv[])
{

    // 主程序处理对象
    DealRCF deal_rcf;
    memset(DealRCF::deal_info_.RsuIp, 0, IP_NUM);
    DealRCF::sensor_object_lists_.clear();
    pthread_mutex_init(&DealRCF::sensor_object_lists_mutex_, NULL);
    pthread_mutex_init(&DealRCF::mat_object_list_mutex_, NULL);
    pthread_mutex_init(&DealRCF::yushi_mat_object_list_mutex_, NULL);
    pthread_rwlock_init(&DealRCF::yushi_mat_rwlock_, NULL);
    //DealRCF::width_ = 1920;
    //DealRCF::height_ = 1080;

    // 打印版本号
    DealRCF::PrintMainProcessInfo();

    // 判断参数个数
    if (argc < ERROR_ARG_NUM)
    {
        cerr << "Missing argument !" << endl;
        cerr << "Usage : process <configFile> " << endl;

        // 加载配置文件
        if (RETURN_ZERO > deal_rcf.LoadConfig("../config/DealRCF.cfg"))
        {
            cerr << "RETURN_ZERO > deal_rcf.LoadConfig(argv[1])" << endl;
            return RETURN_NEGATIVE;
        }
    }
    else
    {
        // 加载配置文件
        if (RETURN_ZERO > deal_rcf.LoadConfig(argv[1]))
        {
            cerr << "RETURN_ZERO > deal_rcf.LoadConfig(argv[1])" << endl;
            return RETURN_NEGATIVE;
        }
        // sleep(PRINT_DELAY_TIME);
    }

    // 初始化UDP信息
    if (RETURN_ZERO > deal_rcf.InitUDP(NULL))
    {
        cerr << "RETURN_ZERO > deal_rcf.InitUDP(NULL)" << endl;
        return RETURN_NEGATIVE;
    }

    // 开启与pad端联调线程
    if (1 == DealRCF::deal_info_.IsPadDebug)
    {
        if (RETURN_ZERO > deal_rcf.StartDealPadThread(NULL))
        {
            cerr << "RETURN_ZERO > deal_rcf.StartDealPadThread(NULL)" << endl;
            return RETURN_NEGATIVE;
        }
        sleep(PRINT_DELAY_TIME);
    }

    // 开始获取图像线程
    if (RETURN_ZERO > deal_rcf.StartMatDealThread(NULL))
    {
        cerr << "RETURN_ZERO > deal_rcf.MatDealThread(NULL)" << endl;
        return RETURN_NEGATIVE;
    }
    sleep(1);

    // 开始视觉检测线程
    if (RETURN_ZERO > deal_rcf.StartCameraDealThread(NULL))
    {
        cerr << "RETURN_ZERO > deal_rcf.StartCameraDealThread(NULL)" << endl;
        return RETURN_NEGATIVE;
    }

    // 主线程不结束
    while (true)
    {
        sleep(MAIN_WHILE_TIME);
    }

    return 0;
}
