/*
   g++ main.cpp -lboost_system -lboost_thread -lpthread -o leica_streaming_receiver
   */

#include <iostream>
#include <string>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "leica_streaming_node/tcp_total_station_interface.h"
#include "leica_streaming_node/udp_total_station_interface.h"
#include "leica_streaming_node/serial_total_station_interface.h"
#include "leica_streaming_node/leica_streaming_node.h"
#include "leica_streaming_msgs/msg/leica_point.hpp"
#include "leica_streaming_msgs/msg/leica_point_array.hpp"

namespace leica_streaming_node {


    LeicaStreamingNode::~LeicaStreamingNode() {
    }

    LeicaStreamingNode::LeicaStreamingNode() : rclcpp::Node("leica_streaming_node") {

        std::string ip,comport,connection;
        int port;
        this->declare_parameter("~/connection", "serial");
        this->declare_parameter("~/comport", "/dev/ttyUSB0");
        this->declare_parameter("~/ip", "10.2.86.54");
        this->declare_parameter("~/publish_tf", true);
        this->declare_parameter("~/inverse_tf", false);
        this->declare_parameter("~/port", 5001);
        this->declare_parameter("~/base_frame", std::string("world"));
        this->declare_parameter("~/point_frame", std::string("point"));

        connection = this->get_parameter("~/connection").as_string();
        comport = this->get_parameter("~/comport").as_string();
        ip = this->get_parameter("~/ip").as_string();
        publish_tf_ = this->get_parameter("~/publish_tf").as_bool();
        inverse_tf_ = this->get_parameter("~/inverse_tf").as_bool();
        port = this->get_parameter("~/port").as_int();
        base_frame_ = this->get_parameter("~/base_frame").as_string();
        point_frame_ = this->get_parameter("~/point_frame").as_string();
        rclcpp::Time now = this->get_clock()->now();
        transformStamped_.header.stamp = now;
        if (inverse_tf_) {
            transformStamped_.header.frame_id = point_frame_;
            transformStamped_.child_frame_id = base_frame_;
        } else {
            transformStamped_.header.frame_id = base_frame_;
            transformStamped_.child_frame_id = point_frame_;
        }
        transformStamped_.transform.translation.x = 0;
        transformStamped_.transform.translation.y = 0;
        transformStamped_.transform.translation.z = 0;

        transformStamped_.transform.rotation.x = 0;
        transformStamped_.transform.rotation.y = 0;
        transformStamped_.transform.rotation.z = 0;
        transformStamped_.transform.rotation.w = 1;

        if (connection == "serial") {
            SerialTSInterface * sts = new SerialTSInterface(this,std::bind(&LeicaStreamingNode::locationTSCallback,
                        this, std::placeholders::_1));
            sts->connect(comport);
            ts_.reset(sts);
        } else if (connection == "tcp") {
            TCPTSInterface *tts = new TCPTSInterface(this,std::bind(&LeicaStreamingNode::locationTSCallback,
                        this, std::placeholders::_1));
            tts->connect(ip, port);
            ts_.reset(tts);
        } else if (connection == "udp") {
            UDPTSInterface *tts = new UDPTSInterface(this,std::bind(&LeicaStreamingNode::locationTSCallback,
                        this, std::placeholders::_1));
            tts->connect(ip, port);
            ts_.reset(tts);
        } else {
            RCLCPP_ERROR(this->get_logger(),"Connection type '%s' is not recognized. Valid options are 'serial' or 'tcp'.",connection.c_str());
        }

        lpoint_pub_ = this->create_publisher<leica_streaming_msgs::msg::LeicaPoint>("~/leica_point", 1);
        lpoints_pub_ = this->create_publisher<leica_streaming_msgs::msg::LeicaPointArray>("~/leica_stored_points",  
                rclcpp::QoS(1).transient_local());
        prism_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("~/position", 1);
        pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("~/target_position", 1, 
                std::bind(&LeicaStreamingNode::positionCb, this, std::placeholders::_1));

        start_stop_srv_ = this->create_service<std_srvs::srv::SetBool>("start_stop", 
                std::bind(&LeicaStreamingNode::startStopCb, this, std::placeholders::_1, std::placeholders::_2));
    }

    void LeicaStreamingNode::positionCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (ts_) {
            ts_->setPrismPosition(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        }
    }

    bool LeicaStreamingNode::startStopCb(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
        if (ts_) {
            if (req->data) {
                ts_->start();
                res->success=true;
                res->message="ok";
            } else {
                ts_->end();
                res->success=true;
                res->message="ok";
            }
            return true;
        }
        res->success=false;
        res->message="TS not connected";
        return true;
    }

    void LeicaStreamingNode::locationTSCallback(const TSMessage & msg) {
#if 0
        std::cout << "Prism is at x: " << msg.x 
            << " y: " << msg.y
            << " z: " << msg.z << std::endl;
        std::cout << std::endl;
#endif
        rclcpp::Time now = this->get_clock()->now();

        if (!lpoint_.point_id.empty() && (msg.point_id != lpoint_.point_id)) {
            lpoints_.points.push_back(lpoint_);
            lpoints_pub_->publish(lpoints_);
        }

        lpoint_.header.stamp = now;
        lpoint_.header.frame_id = base_frame_;
        lpoint_.point_id = msg.point_id;
        lpoint_.x = msg.x;
        lpoint_.y = msg.y;
        lpoint_.z = msg.z;
        lpoint_.date = msg.date;
        lpoint_.sensor_time = msg.sensor_time;
        lpoint_pub_->publish(lpoint_);


        geometry_msgs::msg::PointStamped pmsg;
        pmsg.header.stamp = now;
        pmsg.header.frame_id = base_frame_;
        pmsg.point.x = msg.x;
        pmsg.point.y = msg.y;
        pmsg.point.z = msg.z;



        prism_pos_pub_->publish(pmsg);

        if (publish_tf_) {
            transformStamped_.header.stamp = now;
            if (inverse_tf_) {
                transformStamped_.transform.translation.x = -msg.x;
                transformStamped_.transform.translation.y = -msg.y;
                transformStamped_.transform.translation.z = -msg.z;
            } else {
                transformStamped_.transform.translation.x = msg.x;
                transformStamped_.transform.translation.y = msg.y;
                transformStamped_.transform.translation.z = msg.z;
            }
            tf_broadcaster_->sendTransform(transformStamped_);
        }

    }

} // namespace leica_streaming_node
