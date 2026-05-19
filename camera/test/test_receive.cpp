#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>
#include <fstream>

#include "../proto/nebulalink.perceptron3.0.5.pb.h"

using namespace std;

int main(int argc, char* argv[]) 
{
    unsigned char recv_buf[40960];
    int recv_num = 0;
    string ip;
    int port;
    int socket_id = 0;
    bool isSave = false; //是否保存txt, 默认不保存
    ofstream fout;
    struct timespec time_stamp;
    uint64_t timestamp;
    
    if(argc < 3)
    {
        cout << "ERROR:  must input ip & port....\n";
        cout << "run script: ./test_receive ip port savetxt(0/1)\n";
        cout << "sample(default, not save): ./test_receive 192.168.8.90 10086\n";
        cout << "sample(savetxt): ./test_receive 192.168.8.90 10086 1\n";
        return -1;
    }  

    ip = string(argv[1]);
    port = atoi(argv[2]);
    cout << "ip = " << ip << "  port = " << port << endl;

    if(argc == 4)
        isSave = atoi(argv[3]);


    socket_id = socket(AF_INET, SOCK_DGRAM, 0);
    if(socket_id == -1)
    {
        printf("create socket error: errno:%d", errno);
        return -1;
    }
    else
    {
        printf("create socket success...\n");
    }
    
    struct sockaddr_in addr_serv;
    memset(&addr_serv, 0, sizeof(struct sockaddr_in));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_addr.s_addr = inet_addr(ip.c_str());
    addr_serv.sin_port = htons(port);
    int len = sizeof(addr_serv);

    if(-1 == bind(socket_id, (struct sockaddr*)&addr_serv, len))
    {
        std::cout << "bind error, errno = " << errno << std::endl;
        return -1;
    }

    if(isSave)
    {
        clock_gettime(CLOCK_REALTIME, &time_stamp);
        timestamp = time_stamp.tv_sec * 1e3 + time_stamp.tv_nsec/1e6;
        
        string fileName = to_string(timestamp) + "_camera_record.txt";
        fout.open(fileName);
    }


    while (1)
    {
        recv_num = recvfrom(socket_id, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_serv, (socklen_t *)&len);
        if(recv_num < 0)
        {
            std::cout << "receive error, errno = " << errno << std::endl;
        }

        clock_gettime(CLOCK_REALTIME, &time_stamp);
        timestamp = time_stamp.tv_sec * 1e3 + time_stamp.tv_nsec/1e6;  //ms

        // printf("recv_num=%d\n",recv_num);
        unsigned char l[2] = {0};
        l[0] = recv_buf[7];
        l[1] = recv_buf[6];
        unsigned short send_len = *((unsigned short*)l);
        printf("recev len: %d\n", send_len);
        nebulalink::perceptron3::PerceptronSet perceptronset = nebulalink::perceptron3::PerceptronSet();
        if(!perceptronset.ParseFromArray(recv_buf+8, send_len))
        {
            cerr << "failed to parse message...." << endl;
            return -1;
        }

        printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
        printf("~~~~~~~~~~~~obj_msg~~~~~~~~~~~~\n");
        // printf("receive_time = %ld\n", timestamp);
        // printf("devide_id = %s\n", perceptronset.devide_id().c_str());
        // printf("timestamp = %ld\n", perceptronset.time_stamp());
        printf("number_frame = %d\n", perceptronset.number_frame());
        // printf("perceptron size = %d\n", perceptronset.perceptron().size());
        for(unsigned int i = 0; i < perceptronset.perceptron().size(); ++i)
        {
            printf("*************\n");
            printf("object_id = %d\n", perceptronset.perceptron(i).object_id());
            printf("object_class_type = %d\n", perceptronset.perceptron(i).object_class_type());
            // printf("camera_x = %d\n", perceptronset.perceptron(i).point4f().camera_x());
            // printf("camera_y = %d\n", perceptronset.perceptron(i).point4f().camera_y());
            // printf("camera_w = %d\n", perceptronset.perceptron(i).point4f().camera_w());
            // printf("camera_h = %d\n", perceptronset.perceptron(i).point4f().camera_h());
            // printf("object_speed = %f\n", perceptronset.perceptron(i).object_speed());
            // printf("object_heading = %f\n", perceptronset.perceptron(i).object_heading());
            // printf("object_length = %f\n", perceptronset.perceptron(i).target_size().object_length());
            // printf("object_width = %f\n", perceptronset.perceptron(i).target_size().object_width());
            // printf("object_height = %f\n", perceptronset.perceptron(i).target_size().object_height());
            // printf("object_longitude = %lf\n", perceptronset.perceptron(i).point_gps().object_longitude());
            // printf("object_latitude = %lf\n", perceptronset.perceptron(i).point_gps().object_latitude());
            // printf("lane_id = %s\n", perceptronset.perceptron(i).lane_id().c_str());

            // if(perceptronset.perceptron(i).plate_num() != "0")
            printf("object_plate_num = %s\n", perceptronset.perceptron(i).plate_num().c_str());
        }
        
        // printf("~~~~~~~~~~~~link_msg~~~~~~~~~~~~\n");
        // for(uint i=0; i<perceptronset.link_jam_sense_params().size(); i++)
        // {
        //     printf("*****link: %d*******\n", i+1);
        //     printf("link_id = %s\n", perceptronset.link_jam_sense_params(i).road_id().c_str());
        //     printf("avg_speed = %.2f\n", perceptronset.link_jam_sense_params(i).avg_speed());
        //     printf("veh_num = %d\n", perceptronset.link_jam_sense_params(i).veh_num());
        // }
        // printf("~~~~~~~~~~~~lane_msg~~~~~~~~~~~~\n");
        // for(uint i=0; i<perceptronset.lane_jam_sense_params().size(); i++)
        // {
        //     printf("*****lane: %d*******\n", i+1);
        //     printf("lane_id = %s\n", perceptronset.lane_jam_sense_params(i).lane_id().c_str());
        //     printf("avg_speed = %.2f\n", perceptronset.lane_jam_sense_params(i).avg_speed());
        //     printf("veh_num = %d\n", perceptronset.lane_jam_sense_params(i).veh_num());
        // }

        if(isSave)
        {
            string tmp = perceptronset.devide_id() + " " + to_string(timestamp) + " ";
            for(unsigned int i = 0; i < perceptronset.perceptron().size(); ++i)
            {
                char one_obj[1024];
                sprintf(one_obj, "%d %d %.8f %.8f %.2f %.2f %.2f %.2f", 
                    perceptronset.perceptron(i).object_id(),
                    perceptronset.perceptron(i).object_class_type(),
                    perceptronset.perceptron(i).point_gps().object_longitude(),
                    perceptronset.perceptron(i).point_gps().object_latitude(),
                    perceptronset.perceptron(i).object_speed(),
                    perceptronset.perceptron(i).object_heading(),
                    perceptronset.perceptron(i).target_size().object_width(),
                    perceptronset.perceptron(i).target_size().object_height()
                );
                // tmp = tmp + to_string(perceptronset.perceptron(i).object_id()) + " " 
                //         + to_string(perceptronset.perceptron(i).object_class_type()) + " " 
                //         + to_string(perceptronset.perceptron(i).point_gps().object_longitude()) + " "
                //         + to_string(perceptronset.perceptron(i).point_gps().object_latitude()) + " "
                //         + to_string(perceptronset.perceptron(i).object_speed()) + " "
                //         + to_string(perceptronset.perceptron(i).object_heading()) + " "
                //         + to_string(perceptronset.perceptron(i).target_size().object_width()) + " "
                //         + to_string(perceptronset.perceptron(i).target_size().object_height()) + ";";

                tmp = tmp + string(one_obj) + ";";
            }

            tmp += "\n";
            fout << tmp;            
        }

    }

    return 0;
}
