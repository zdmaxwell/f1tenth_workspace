#include <rclcpp/rclcpp.hpp>
#include <IpIpoptApplication.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("slash_mpc"), "MPC node running");
    rclcpp::shutdown();
    return 0;
}