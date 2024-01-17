#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "leica_streaming_node/total_station_interface.h"
#include "leica_streaming_msgs/msg/leica_point.hpp"
#include "leica_streaming_msgs/msg/leica_point_array.hpp"

namespace leica_streaming_node {

    class LeicaStreamingNode : public rclcpp::Node {
        public:
            LeicaStreamingNode();
            ~LeicaStreamingNode();

        private:
            void connectCb();
            void disconnectCb();
            void positionCb(const nav_msgs::msg::Odometry::SharedPtr msg);
            bool startStopCb(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res);
            void locationTSCallback(const TSMessage &);

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_sub_;
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_srv_;
            rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr prism_pos_pub_;
            rclcpp::Publisher<leica_streaming_msgs::msg::LeicaPoint>::SharedPtr lpoint_pub_;
            rclcpp::Publisher<leica_streaming_msgs::msg::LeicaPointArray>::SharedPtr lpoints_pub_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            geometry_msgs::msg::TransformStamped transformStamped_;
            leica_streaming_msgs::msg::LeicaPoint lpoint_;
            leica_streaming_msgs::msg::LeicaPointArray lpoints_;

            bool publish_tf_;
            bool inverse_tf_;
            std::string base_frame_;
            std::string point_frame_;

            std::shared_ptr<TSInterface> ts_;
    };
} // namespace leia_streaming_app
