#include "moment_navigation/go_to_goal.hpp"

#include <memory>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<moment_navigation::GoToGoalServer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}