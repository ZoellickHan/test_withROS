#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <deque>
#include "protocol.hpp"

class TestNode : public rclcpp :: Node
{
public:
    TestNode() : Node("test")
    {
        gimbal_msg_sub_ = create_subscription<serial_driver::TwoCRC_GimbalMsg>( "gimbal_command", rclcpp::SensorDataQoS(),
        std::bind(debug1, this, std::placeholders::_1));

        sentry_gimbal_msg_sub_ = create_subscription<serial_driver::TwoCRC_SentryGimbalMsg>("sentry_gimbal",rclcpp::SensorDataQoS(),
        std::bind(debug2,this,std::placeholders::_1));

        

    }
    ~TestNode();

private:

    void debug1();

    void debug2();
    rclcpp::Publisher<serial_driver::TwoCRC_ChassisCommand>::SharedPtr chassis_cmd_;
    rclcpp::Publisher<serial_driver::TwoCRC_GimbalCommand>::SharedPtr gimbal_cmd_;
    rclcpp::Subscription<serial_driver::TwoCRC_SentryGimbalMsg>::SharedPtr sentry_gimbal_msg_sub_;
    rclcpp::Subscription<serial_driver::TwoCRC_GimbalMsg>::SharedPtr gimbal_msg_sub_;


};