//ros2 launch turtlebot3_gazebo turtlebot3_office.launch.py
//ros2 run turtlebot3_teleop teleop_keyboard
//ros2 run sprint1 sprint1_node
//ros2 launch turtlebot3_bringup rviz2.launch.py
//if required: ros2 topic echo scan_filtered 
//ros2 run rqt_image_view rqt_image_view
//ros2 topic list

//colcon build --symlink-install
//source ~/.bashrc
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"

#include <limits> // For NaN values

class Sprint1 : public rclcpp::Node {
public:
  Sprint1() : Node("sprint1_node"), nth_(2), counter_(0) {

    // Subscription to the LaserScan topic
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Sprint1::scan_callback, this, std::placeholders::_1));

    // Publisher for the processed scan data
    scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_filtered", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  const int nth_;       
  int counter_;     
  // Callback for LaserScan processing
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    counter_++;

    // Create a new LaserScan message with the same size as the input
    auto filtered = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);

    // Replace non-nth ranges with NaN
    for (size_t i = 0; i < filtered->ranges.size(); ++i) {
      if (i % nth_ != 0) {
        filtered->ranges[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }

    // Publish the new filtered scan
    scan_publisher_->publish(*filtered);
  }

   
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Sprint1>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
