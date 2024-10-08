#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  
#include <tf2/exceptions.h>
#include <cmath>
#include <vector>

class CylinderDetector : public rclcpp::Node
{
public:
    CylinderDetector() : Node("cylinder_detector"),tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),  
      tf_listener_(tf_buffer_)                                    
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetector::scanCallback, this, std::placeholders::_1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cylinder_marker", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cylinder_pose", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        cylinder_diameter_ = 0.30;  
        cylinder_radius_ = cylinder_diameter_ / 2.0;
    }

    private:

        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
        {
            std::vector<float> ranges = scan_msg->ranges;
            size_t n = ranges.size();

            double angle_min = scan_msg->angle_min;
            double angle_increment = scan_msg->angle_increment;

            std::vector<double> xs, ys;

            // convert polar coordinates to Cartesian coordinates
            for (size_t i = 0; i < n; ++i)
            {
                double angle = angle_min + i * angle_increment;
                double x = ranges[i] * std::cos(angle);
                double y = ranges[i] * std::sin(angle);
                xs.push_back(x);
                ys.push_back(y);
            }

        
            int cylinder_idx = detectCylinder(xs, ys);
            if (cylinder_idx != -1)
            {
                // calculate the mean position of the cylinder in laser frame
                double cylinder_x = xs[cylinder_idx];
                double cylinder_y = ys[cylinder_idx];

                
                publishCylinderMarker(cylinder_x, cylinder_y);

                
                publishCylinderTF(cylinder_x, cylinder_y);
            }
        }

        int detectCylinder(const std::vector<double>& xs, const std::vector<double>& ys)
        {
            size_t n = xs.size();
            std::vector<double> distances(n - 1);

            // calculate distances between consecutive points
            for (size_t i = 0; i < n - 1; ++i)
            {
                distances[i] = std::sqrt(std::pow(xs[i + 1] - xs[i], 2) + std::pow(ys[i + 1] - ys[i], 2));
            }

            // threshold to consider consecutive points as part of the same cluster
            double distance_threshold = 0.15;  // 15 cm

            // loop over the points and find clusters
            for (size_t i = 0; i < n - 1; ++i)
            {
                std::vector<size_t> cluster_indices;  // To store indices of points in the current cluster

                // new cluster
                cluster_indices.push_back(i);

                // continue adding points to the cluster as long as the distance is below the threshold
                while (i < n - 1 && distances[i] < distance_threshold)
                {
                    cluster_indices.push_back(i + 1);  // Add the next point to the cluster
                    ++i; 
                }

                // Calculate the total width of the cluster 
                double cluster_x_min = xs[cluster_indices.front()];
                double cluster_x_max = xs[cluster_indices.back()];

                double cluster_y_min = ys[cluster_indices.front()];
                double cluster_y_max = ys[cluster_indices.back()];

                double cluster_width = std::sqrt(std::pow(cluster_x_max - cluster_x_min, 2) + std::pow(cluster_y_max - cluster_y_min, 2));

                // if the detected cluster width is within the expected range for the cylinder (25 cm to 30 cm)
                if (cluster_width >= 0.25 && cluster_width <= 0.30)
                {
                    // return the index of the first point in the cluster
                    return cluster_indices.front();
                }
            }

            return -1;  
        }

        void publishCylinderMarker(double x, double y)
        {
            // create a marker
            visualization_msgs::msg::Marker marker;
            marker.ns = "cylinder";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = cylinder_diameter_;
            marker.scale.y = cylinder_diameter_;
            marker.scale.z = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            // transform the position from base_link to map 
            geometry_msgs::msg::TransformStamped transformStamped;

            try
            {
                // lookup the transform from base_link to map
                transformStamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

                // apply the transform to the cylinder position (x, y)
                geometry_msgs::msg::PointStamped cylinder_point_in_base;
                cylinder_point_in_base.header.frame_id = "base_link";
                cylinder_point_in_base.point.x = x;
                cylinder_point_in_base.point.y = y;
                cylinder_point_in_base.point.z = 0;

                geometry_msgs::msg::PointStamped cylinder_point_in_map;
                tf2::doTransform(cylinder_point_in_base, cylinder_point_in_map, transformStamped);

                // set the transformed position in the marker
                marker.pose.position.x = cylinder_point_in_map.point.x;
                marker.pose.position.y = cylinder_point_in_map.point.y;
                marker.pose.position.z = 0.0;  
                marker.pose.orientation.w = 1.0;

                // set the frame to map 
                marker.header.frame_id = "map";  
                marker.header.stamp = this->get_clock()->now();

            
                marker_pub_->publish(marker);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
            }
        }

        void publishCylinderTF(double x, double y)
        {
            // create and broadcast a TransformStamped for the detected cylinder
            geometry_msgs::msg::TransformStamped transformStamped;

            transformStamped.header.stamp = this->get_clock()->now();
            transformStamped.header.frame_id = "base_link";
            transformStamped.child_frame_id = "cylinder";
            transformStamped.transform.translation.x = x;
            transformStamped.transform.translation.y = y;
            transformStamped.transform.translation.z = 0.0;
            transformStamped.transform.rotation.w = 1.0;

            tf_broadcaster_->sendTransform(transformStamped);
        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        tf2_ros::Buffer tf_buffer_;  
        tf2_ros::TransformListener tf_listener_;  
        double cylinder_diameter_;
        double cylinder_radius_;

    
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
