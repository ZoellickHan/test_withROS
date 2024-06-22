#ifndef SERIAL_DRIVER_MY_NODE_HPP_
#define SERIAL_DRIVER_MY_NODE_HPP_

#define READER_BUFFER_SIZE 64
#define MAX_BUFFER_SIZE 2024
#define DECODE_BUFFER_SIZE 512

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <deque>

#include "serial_driver/protocol.hpp"
#include "serial_driver/crc.hpp"
#include  "serialDriver.hpp"
namespace serial_driver
{

class ReadNode : public rclcpp::Node
{
public:
  ReadNode();

private:
  //info
    rclcpp::Publisher<TwoCRC_GimbalMsg>::SharedPtr gimabal_msg_;
    rclcpp::Publisher<TwoCRC_SentryGimbalMsg>::SharedPtr sentry_gimbal_msg_;
    
    //function
    PkgState decode();
    int receive();
    void classify(uint8_t* data);

    std::shared_ptr<SerialConfig> config =  
    std::make_shared<SerialConfig>(2000000,8,false,StopBit::TWO,Parity::NONE);

    std::shared_ptr<Port> port = 
    std::make_shared<Port>(config);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::deque<uint8_t> buffer;
    uint8_t decodeBuffer[DECODE_BUFFER_SIZE];
    uint8_t receiveBuffer[READER_BUFFER_SIZE];

    //protocol 
    Header header;

    //debug info 
    int error_sum_header = 0;
    bool crc_ok          = false;
    bool crc_ok_header   = false;
    int  bag_sum         = 0;
    int  error_sum_payload = 0;
 
};

}
#endif  // SERIAL_DRIVER_MY_NODE_HPP_