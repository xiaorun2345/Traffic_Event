/*
 *  SerialPortCommunication.cpp
 *  Created on: 2024.2.20
 *  Author: DouDianSong
 *  Description: 与哨兵系统的串口通信类
 *
 */
#include "SerialPortCommunication.h"

// 设置串口
SerialPortCommunication::SerialPortCommunication(std::string port_name, int baud_rate):io_(), 
serial_(io_, port_name)
{
    // 波特率
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    // 数据8位
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
    // 停止1
    serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    // 无奇偶校验
    serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
}



// 析构函数
SerialPortCommunication::~SerialPortCommunication()
{
    serial_.close();
}

// 类成员方法 显示内容编号、喇叭声音编号
void SerialPortCommunication::SendInstructToSentry(int show_number)
{
    int program_num = event_map.count(show_number) ? event_map[show_number] : 0;

    unsigned char send_data[13];
    send_data[0] = 0xaa;
    send_data[1] = 0x55;
    send_data[2] = 0x00;
    send_data[3] = 0x0d;
    send_data[4] = 0x93;
    send_data[5] = 0x00;
    send_data[6] = 0x00;
    send_data[7] = 0x01;
    send_data[8] = program_num;
    send_data[9] = program_num;
    send_data[12] = 0x88;
    short sum = 0;
    for(int i = 0; i < 10; ++i)
    {
        sum += send_data[i];
    }
    *((unsigned short *)(send_data + 10)) = htons(sum);
    boost::asio::write(serial_, boost::asio::buffer(send_data));
    printf("set program_num %d sucess \n",program_num);
    // std::cout << "设置预警成功!\n";
}

// 接收串口指令，可设置结束符
int SerialPortCommunication::RecvInstructFromSentry(std::string &receive_data, char end_byte)
{
    std::size_t bytes_read = 0, sum_bytes_read = 0;
    boost::asio::streambuf buffer;

    while (true) 
    {
        boost::system::error_code error;
        bytes_read = boost::asio::read(serial_, buffer, boost::asio::transfer_at_least(1), error);
        if (error == boost::asio::error::eof) {
            break;  // 串口中没有更多可读数据，结束循环
        }
        else if (error) {
            std::cout << "Error reading from serial port: " << error.message() << std::endl;
            break;  // 发生错误，结束循环
        }

        sum_bytes_read += bytes_read;

        // 在这里处理读取到的数据
        std::string data(boost::asio::buffers_begin(buffer.data()), boost::asio::buffers_end(buffer.data()));
        //std ::cout << "bytes num = " << bytes_read << ", received data: " << data <<  std ::endl;
        receive_data += data;
        // 结束标志，可自定义设置
        if(end_byte == data.back())
        {
            break;
        }

        buffer.consume(bytes_read);  // 清空缓冲区
    }

    return sum_bytes_read;
}

// test 使用样例
// int main() 
// {
//     std::string receive_data;
//     // 创建对象
//     SerialPortCommunication *demo = new SerialPortCommunication("/dev/ttyS7", 57600);
//     // 发送指令
//     demo->SendInstructToSentry(1, 2);
//     // 从串口读取数据
//     demo->RecvInstructFromSentry(receive_data);
//     std::cout << "Received data: " << receive_data << std::endl;
//     delete demo;
    
//     return 0;
// }