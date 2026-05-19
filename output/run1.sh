export LD_LIBRARY_PATH=../lib/
#./camera_app_demo  rtsp://USER:PASSWORD@CAMERA_IP:554/Streaming/Channels/102  ./config/camera_config_lane.cfg  0 0 RSU_IP RSU_PORT 1
./camera_app_demo  rtsp://USER:PASSWORD@CAMERA_IP:554/Streaming/Channels/102 ../config/camera_config_lane.cfg  0 0 RSU_IP RSU_PORT 

#./camera_app_demo  ./hefei.mp4 ../config/camera_config_lane.cfg  0 0 192.168.8.123 10086 
