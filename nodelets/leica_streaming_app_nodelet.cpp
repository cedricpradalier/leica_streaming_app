/*
   g++ main.cpp -lboost_system -lboost_thread -lpthread -o leica_streaming_receiver
   */

#include <iostream>
#include <string>
#include <boost/asio.hpp>

#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"

#include "leica_streaming_app/tcp_total_station_interface.h"
#include "leica_streaming_app/serial_total_station_interface.h"
#include "leica_streaming_app/leica_streaming_app_nodelet.h"

namespace leica_streaming_app {

    LeicaStreamingAppNodelet::LeicaStreamingAppNodelet() {}

    LeicaStreamingAppNodelet::~LeicaStreamingAppNodelet() {
    }

    void LeicaStreamingAppNodelet::onInit() {
        nh_ = getNodeHandle();
        private_nh_ = getPrivateNodeHandle();

        std::string ip,comport,connection;
        int port;
        private_nh_.param<std::string>("connection", connection, "serial");
        private_nh_.param<std::string>("comport", comport, "/dev/ttyUSB0");
        private_nh_.param<std::string>("ip", ip, "10.2.86.54");
        private_nh_.param("publish_tf", publish_tf_, true);
        private_nh_.param("port", port, 5001);
        private_nh_.param("base_frame", base_frame_, std::string("world"));
        private_nh_.param("point_frame", point_frame_, std::string("point"));
        transformStamped_.header.stamp = ros::Time::now();
        transformStamped_.header.frame_id = base_frame_;
        transformStamped_.child_frame_id = point_frame_;
        transformStamped_.transform.translation.x = 0;
        transformStamped_.transform.translation.y = 0;
        transformStamped_.transform.translation.z = 0;

        transformStamped_.transform.rotation.x = 0;
        transformStamped_.transform.rotation.y = 0;
        transformStamped_.transform.rotation.z = 0;
        transformStamped_.transform.rotation.w = 1;

        if (connection == "serial") {
            SerialTSInterface * sts = new SerialTSInterface(std::bind(&LeicaStreamingAppNodelet::locationTSCallback,
                            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            sts->connect(comport);
            ts_.reset(sts);
        } else if (connection == "tcp") {
            TCPTSInterface *tts = new TCPTSInterface(std::bind(&LeicaStreamingAppNodelet::locationTSCallback,
                            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            tts->connect(ip, port);
            ts_.reset(tts);
        } else {
            ROS_ERROR("Connection type '%s' is not recognized. Valid options are 'serial' or 'tcp'.",connection.c_str());
        }

        prism_pos_pub_ = private_nh_.advertise<geometry_msgs::PointStamped>("position", 1,
                boost::bind(&LeicaStreamingAppNodelet::connectCb, this),
                boost::bind(&LeicaStreamingAppNodelet::disconnectCb, this));

        start_stop_srv_ = private_nh_.advertiseService("start_stop", &LeicaStreamingAppNodelet::startStopCb, this);
    }

    void LeicaStreamingAppNodelet::connectCb() {
        if (!prism_pos_pub_ && prism_pos_pub_.getNumSubscribers() > 0) {
            NODELET_INFO("Connecting to odom/vicon position topic.");
            pos_sub_ = private_nh_.subscribe("position", 1, &LeicaStreamingAppNodelet::positionCb, this);
        }
    }

    void LeicaStreamingAppNodelet::disconnectCb() {
        if (prism_pos_pub_.getNumSubscribers() == 0) {
            NODELET_INFO("Unsubscribing from odom/vicon position topic.");
            pos_sub_.shutdown();
        }
    }

    void LeicaStreamingAppNodelet::positionCb(const nav_msgs::Odometry::ConstPtr& msg) {
        if (ts_) {
            ts_->setPrismPosition(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        }
    }

    bool LeicaStreamingAppNodelet::startStopCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response & res) {
        if (ts_) {
            if (req.data) {
                ts_->start();
                res.success=true;
                res.message="ok";
            } else {
                ts_->end();
                res.success=true;
                res.message="ok";
            }
            return true;
        }
        res.success=false;
        res.message="TS not connected";
        return true;
    }

    void LeicaStreamingAppNodelet::locationTSCallback(const double x,
            const double y,
            const double z) {
        /*
           std::cout << "Prism is at x: " << x 
           << " y: " << y
           << " z: " << z << std::endl;
           std::cout << std::endl;
           */
        geometry_msgs::PointStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = base_frame_;
        msg.point.x = x;
        msg.point.y = y;
        msg.point.z = z;

        prism_pos_pub_.publish(msg);

        if (publish_tf_) {
            transformStamped_.header.stamp = ros::Time::now();
            transformStamped_.transform.translation.x = x;
            transformStamped_.transform.translation.y = y;
            transformStamped_.transform.translation.z = z;

            br_.sendTransform(transformStamped_);
        }

    }

} // namespace leica_streaming_app
// PLUGINLIB_DECLARE_CLASS(leica_streaming_app, LeicaStreamingAppNodelet, leica_streaming_app::LeicaStreamingAppNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(leica_streaming_app::LeicaStreamingAppNodelet, nodelet::Nodelet)
