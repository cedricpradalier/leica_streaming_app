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

#include "leica_streaming_node/udp_total_station_interface.h"

#include <rclcpp/rclcpp.hpp>

using boost::asio::ip::udp;
boost::asio::io_service UDPTSInterface::io_service;

UDPTSInterface::UDPTSInterface(rclcpp::Node * node, std::function<void(const TSMessage &)> locationCallback)
  : TSInterface(node,locationCallback), receiveThread(NULL), terminate(false) {}

UDPTSInterface::~UDPTSInterface() {
    if (receiveThread) {
        terminate = true;
        receiveThread->join();
        delete receiveThread;
    }
}

void UDPTSInterface::recv_thread() {
    RCLCPP_INFO(node_->get_logger(),"%s: UDP Listening on port %d",sensor_host.c_str(),sensor_port);
    udp::socket socket(io_service, udp::endpoint(udp::v4(), sensor_port));
    while (!terminate) {
        boost::array<uint8_t, 1024> recv_buf;
        udp::endpoint sender_endpoint;
        int len = socket.receive_from(
                boost::asio::buffer(recv_buf), sender_endpoint);
        if (len>0) {
            std::string data(recv_buf.begin(),recv_buf.end());
            for (size_t i=0;i<data.size();i++) {
                if ((data[i]=='\r') || (data[i]=='\n')) {
                    data = data.substr(0,i);
                    break;
                }
            }
            readHandler(data);
        } else {
            usleep(1000);
        }
    }
}


void UDPTSInterface::connect(std::string ip, int port) {
    sensor_host = ip;
    sensor_port = port;

    receiveThread = new std::thread(&UDPTSInterface::recv_thread,this);

    udp::resolver resolver(io_service);
    char sensor_port_cstr[128];
    sprintf(sensor_port_cstr,"%d",port);
    udp::resolver::query query(udp::v4(), sensor_host,std::string(sensor_port_cstr));
    receiver_endpoint = *resolver.resolve(query);
    socket_.reset(new udp::socket(io_service));
    socket_->open(udp::v4());
}

void UDPTSInterface::write(std::vector<char> str) {
    socket_->send_to(boost::asio::buffer(str), receiver_endpoint);
}

void UDPTSInterface::readHandler(const std::string & data) {

    // Print received message
    std::cout << data << std::endl;

    // Check for responses if the total station searches the prism
    if (searchingPrismFlag_) {
        // Catch the response
        if (data.find("%R8P,0,0:") != std::string::npos) {
            // Catch the negative response
            if (data.find(":31") != std::string::npos) {
                RCLCPP_WARN(node_->get_logger(),"Prism not found!");
                searchPrism();
            } else if (data.find(":0") != std::string::npos) { // Catch the positive response
                RCLCPP_INFO(node_->get_logger(),"Prism found");
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
    } else { // Forward x, y and z coordinate if location was received
             // Split the received message to access the coordinates
        std::vector<std::string> results;
        boost::split(results, data, [](char c){return c == ',';});
        if (results.size() >= 6) {

            double y = std::stod(results[1]);
            double x = std::stod(results[2]);
            double z = std::stod(results[3]);

            TSMessage msg(results[0],x,y,z,results[4],results[5]);
            locationCallback_(msg);
        } else {
            RCLCPP_WARN(node_->get_logger(),"Could not split input stream into 6 fields");
        }

    }

}
