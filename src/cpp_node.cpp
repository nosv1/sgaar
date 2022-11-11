#include "rclcpp/rclcpp.hpp"
#include "cpp_header.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("my_node_name");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}