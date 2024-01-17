#include "rclcpp/rclcpp.hpp"
#include "leica_streaming_node/leica_streaming_node.h"

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<leica_streaming_node::LeicaStreamingNode>());
    rclcpp::shutdown();
    return 0;
}

