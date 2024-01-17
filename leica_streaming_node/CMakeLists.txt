cmake_minimum_required(VERSION 3.8)
project(leica_streaming_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(leica_streaming_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(Boost_USE_STATIC_LIBS        OFF) # only find static libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost COMPONENTS system thread)
if(Boost_FOUND)
      include_directories(${Boost_INCLUDE_DIRS})
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)


add_library(leica_streaming_receiver SHARED src/total_station_interface.cpp
      src/serial_total_station_interface.cpp
      src/tcp_total_station_interface.cpp src/udp_total_station_interface.cpp)
target_include_directories(leica_streaming_receiver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(leica_streaming_receiver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  leica_streaming_receiver
  "leica_streaming_msgs"
  "std_msgs"
  "std_srvs"
  "geometry_msgs"
  "tf2_ros"
  "rclcpp"
)

target_link_libraries(leica_streaming_receiver ${Boost_LIBRARIES})


add_executable(leica_streaming_node src/leica_streaming_main.cpp src/leica_streaming_node.cpp)
target_include_directories(leica_streaming_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(leica_streaming_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  leica_streaming_node
  "leica_streaming_msgs"
  "std_msgs"
  "std_srvs"
  "geometry_msgs"
  "tf2_ros"
  "rclcpp"
  "nav_msgs"
)
target_link_libraries(leica_streaming_node leica_streaming_receiver)

install(TARGETS leica_streaming_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
