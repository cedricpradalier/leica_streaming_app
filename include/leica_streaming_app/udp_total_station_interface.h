/**
 * \file udp_total_station_interface.h
 * \author Andreas Ziegler
 * \date 31.05.2018
 * \brief Header file containing the required defintion for the udp total station interface
 */

#pragma once

#include <vector>
#include <string>
#include <memory>
#include <thread>

#include <boost/asio.hpp>

#include "total_station_interface.h"

/**
 * @brief UDP interface for the Leica total station
 */
class UDPTSInterface: public TSInterface {
 public:
  /**
   * @brief Constructor
   *
   * @param f Callback function to receive the x, y and z location of the tracked prism.
   */
  explicit UDPTSInterface(std::function<void(const double, const double, const double)> locationCallback);

  /**
   * @brief Close socket and join io_context thread.
   */
  virtual ~UDPTSInterface();

  /**
   * @brief Tries to open a udp connection to the total station
   *        Initializes the udp connection to the total station
   *        and the timer to detect if no message are received anymore.
   *
   * @param ip The ip address as std::string
   * @param port The port number as int
   */
  void connect(std::string ip, int port);

 private:

  void recv_thread() ;

  virtual void write(std::vector<char> str);

  /**
   * @brief Callback method when a message was received.
   *        Calls the registered callback function with the
   *        x, y and z coordinates of the prism. Also sets flag
   *        to indicate that a message was received.
   *
   * @param ec error code
   * @param bytes_transferred Amount of bytes received
   */
  void readHandler(const std::string & data);

  std::string sensor_host;
  int sensor_port;
  static boost::asio::io_service io_service;

  std::thread* receiveThread;

};
