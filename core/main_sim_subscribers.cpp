#include "sim_subscribers.hpp"

// Standalone node: subscribe to every sim topic (sensors + ground truth)
// and log what's coming in. Run alongside the sim:
//   ros2 launch sensor_pkg sim.launch.py
//   ros2 run sensor_pkg sim_subscribers
// No estimation/fusion here -- this is just for confirming the topics are
// wired up and as a starting point for whatever consumes them next.
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimSubscribers>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
