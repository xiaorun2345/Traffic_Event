/*
 *  SerialPortCommunication.h
 *  Created on: 2024.2.20
 *  Author: DouDianSong
 *  Description: 与哨兵系统的串口通信类
 *
 */
// 避免头文件重复定义
#ifndef SERIALPORTCOMMUNICATION_H
#define SERIALPORTCOMMUNICATION_H

#include <iostream>
#include <boost/asio.hpp>

// 保证整个文件变量为1字节对齐
#pragma pack(push, 1)
//using namespace std;
// 创建SerialPortCommunication类
class SerialPortCommunication
{
public:
    // 默认构造函数
    SerialPortCommunication() = default;
    // 构造函数
    SerialPortCommunication(std::string port_name, int baud_rate);
    // 析构函数
    ~SerialPortCommunication();

private:
    // 类成员变量
    // 创建串口对象
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    
    std::map<int, int> event_map = {
        {405, 1}, 
        {814, 1}, 
        {1, 2}, 
        {2, 3}, 
        {3, 4}, 
        {4, 5}, 
        {5, 6}, 
        {6, 7}, 
        {7, 8}, 
        {8, 9}
    };


public:
    // 类成员方法 显示内容编号、喇叭声音编号
    void SendInstructToSentry(int show_number);
    // 接收串口指令
    int RecvInstructFromSentry(std::string &receive_data, char end_byte = '\n');
};

#pragma pack(pop)

#endif