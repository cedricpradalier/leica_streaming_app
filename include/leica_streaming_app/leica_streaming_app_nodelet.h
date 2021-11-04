#pragma once

#include <memory>
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "leica_streaming_app/total_station_interface.h"

namespace leica_streaming_app {

    class LeicaStreamingAppNodelet : public nodelet::Nodelet {
        public:
            LeicaStreamingAppNodelet();
            ~LeicaStreamingAppNodelet();

        private:
            virtual void onInit();
            void connectCb();
            void disconnectCb();
            void positionCb(const nav_msgs::Odometry::ConstPtr& msg);
            bool startStopCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response & res);
            void locationTSCallback(double x, double y, double z);

            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;
            ros::Subscriber pos_sub_;
            ros::ServiceServer start_stop_srv_;
            ros::Publisher prism_pos_pub_;
            tf2_ros::TransformBroadcaster br_;
            geometry_msgs::TransformStamped transformStamped_;

            bool publish_tf_;
            std::string base_frame_;
            std::string point_frame_;

            std::shared_ptr<TSInterface> ts_;
    };
} // namespace leia_streaming_app
