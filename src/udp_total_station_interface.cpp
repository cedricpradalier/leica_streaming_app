/**
 * \file udp_total_station_interface.cpp
 * \author Andreas Ziegler
 * \date 31.05.2018
 * \brief Implementation of the udp total station interface
 */

#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>

#include "leica_streaming_app/udp_total_station_interface.h"

#include <ros/ros.h>

using boost::asio::ip::udp;
boost::asio::io_service UDPTSInterface::io_service;

UDPTSInterface::UDPTSInterface(std::function<void(const double, const double, const double)> locationCallback)
  : TSInterface(locationCallback), receiveThread(NULL) {}

UDPTSInterface::~UDPTSInterface() {
    delete receiveThread;
}

void UDPTSInterface::recv_thread() {
    ROS_INFO("%s: Listening on port %d",sensor_host.c_str(),sensor_port);
    udp::socket socket(io_service, udp::endpoint(udp::v4(), sensor_port));
    while (ros::ok()) {
        boost::array<uint8_t, 1024> recv_buf;
        udp::endpoint sender_endpoint;
        int len = socket.receive_from(
                boost::asio::buffer(recv_buf), sender_endpoint);
        if (len>0) {
            std::string data(recv_buf.begin(),recv_buf.end());


        } else {
            ros::Duration(0.001).sleep();
        }
    }
}


void UDPTSInterface::connect(std::string ip, int port) {
    sensor_host = ip;
    sensor_port = port;

    receiveThread = new std::thread(&UDPTSInterface::recv_thread,this);

}

void UDPTSInterface::write(std::vector<char> str) {
    ROS_ERROR("Write command ignored");
}

void UDPTSInterface::readHandler(const std::string & data) {

    // Print received message
    // std::cout << data << std::endl;

    // Check for responses if the total station searches the prism
    if (searchingPrismFlag_) {
        // Catch the response
        if (data.find("%R8P,0,0:") != std::string::npos) {
            std::cout << "Got an answer." << std::endl;

            // Catch the negative response
            if (data.find(":31") != std::string::npos) {
                std::cout << "Prism not found!" << std::endl;
                searchPrism();
            } else if (data.find(":0") != std::string::npos) { // Catch the positive response
                std::cout << "Prism found" << std::endl;
                {
                    std::lock_guard<std::mutex> guard(searchingPrismMutex_);
                    searchingPrismFlag_ = false;
                }
                {
                    std::lock_guard<std::mutex> guard1(messageReceivedMutex_);
                    messagesReceivedFlag_ = true;
                }
            }
        }
    } else if (data[0] == 'T') { // Forward x, y and z coordinate if location was received
        // Split the received message to access the coordinates
        std::vector<std::string> results;
        boost::split(results, data, [](char c){return c == ',';});

        double x = std::stod(results[1]);
        double y = std::stod(results[2]);
        double z = std::stod(results[3]);

        locationCallback_(x, y, z);

    }

}
