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
#include "leica_streaming_app/udp_total_station_interface.h"
#include "leica_streaming_app/serial_total_station_interface.h"
#include "leica_streaming_app/leica_streaming_app_nodelet.h"
#include "leica_streaming_app/LeicaPoint.h"
#include "leica_streaming_app/LeicaPointArray.h"

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
        private_nh_.param("inverse_tf", inverse_tf_, false);
        private_nh_.param("port", port, 5001);
        private_nh_.param("base_frame", base_frame_, std::string("world"));
        private_nh_.param("point_frame", point_frame_, std::string("point"));
        transformStamped_.header.stamp = ros::Time::now();
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
            SerialTSInterface * sts = new SerialTSInterface(std::bind(&LeicaStreamingAppNodelet::locationTSCallback,
                        this, std::placeholders::_1));
            sts->connect(comport);
            ts_.reset(sts);
        } else if (connection == "tcp") {
            TCPTSInterface *tts = new TCPTSInterface(std::bind(&LeicaStreamingAppNodelet::locationTSCallback,
                        this, std::placeholders::_1));
            tts->connect(ip, port);
            ts_.reset(tts);
        } else if (connection == "udp") {
            UDPTSInterface *tts = new UDPTSInterface(std::bind(&LeicaStreamingAppNodelet::locationTSCallback,
                        this, std::placeholders::_1));
            tts->connect(ip, port);
            ts_.reset(tts);
        } else {
            ROS_ERROR("Connection type '%s' is not recognized. Valid options are 'serial' or 'tcp'.",connection.c_str());
        }

        lpoint_pub_ = private_nh_.advertise<leica_streaming_app::LeicaPoint>("leica_point", 1);
        lpoints_pub_ = private_nh_.advertise<leica_streaming_app::LeicaPointArray>("leica_stored_points", 1, true);
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

    void LeicaStreamingAppNodelet::locationTSCallback(const TSMessage & msg) {
#if 0
        std::cout << "Prism is at x: " << msg.x 
            << " y: " << msg.y
            << " z: " << msg.z << std::endl;
        std::cout << std::endl;
#endif

        if (!lpoint_.point_id.empty() && (msg.point_id != lpoint_.point_id)) {
            lpoints_.points.push_back(lpoint_);
            lpoints_pub_.publish(lpoints_);
        }

        lpoint_.header.stamp = ros::Time::now();
        lpoint_.header.frame_id = base_frame_;
        lpoint_.point_id = msg.point_id;
        lpoint_.x = msg.x;
        lpoint_.y = msg.y;
        lpoint_.z = msg.z;
        lpoint_.date = msg.date;
        lpoint_.sensor_time = msg.sensor_time;
        lpoint_pub_.publish(lpoint_);


        geometry_msgs::PointStamped pmsg;
        pmsg.header.stamp = ros::Time::now();
        pmsg.header.frame_id = base_frame_;
        pmsg.point.x = msg.x;
        pmsg.point.y = msg.y;
        pmsg.point.z = msg.z;



        prism_pos_pub_.publish(pmsg);

        if (publish_tf_) {
            transformStamped_.header.stamp = ros::Time::now();
            if (inverse_tf_) {
                transformStamped_.transform.translation.x = -msg.x;
                transformStamped_.transform.translation.y = -msg.y;
                transformStamped_.transform.translation.z = -msg.z;
            } else {
                transformStamped_.transform.translation.x = msg.x;
                transformStamped_.transform.translation.y = msg.y;
                transformStamped_.transform.translation.z = msg.z;
            }
            br_.sendTransform(transformStamped_);
        }

    }

} // namespace leica_streaming_app
// PLUGINLIB_DECLARE_CLASS(leica_streaming_app, LeicaStreamingAppNodelet, leica_streaming_app::LeicaStreamingAppNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(leica_streaming_app::LeicaStreamingAppNodelet, nodelet::Nodelet)
