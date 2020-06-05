#ifndef SIMULATOR_SAUVC_TEST_H
#define SIMULATOR_SAUVC_TEST_H

#include <ros/ros.h>
#include<ros/package.h>

#include <chrono>
#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <hammerhead/hammerhead.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <pid_controller/Setpoint.h>
#include <sensor_msgs/Image.h>
#include <simulator_sauvc_test/Coordinates.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <vectornav/VectorNavData.h>
//#include <sauvc_image_processing/image_processor.h>
#include "simulator_sauvc_test/image_processor.h"

using namespace std;
using namespace cv;

class SimSauvcTest {
public:
  explicit SimSauvcTest(ros::NodeHandle);
  ~SimSauvcTest();

private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it;

  image_transport::Subscriber front_camera_sub, bottom_camera_sub;
  ros::Subscriber enable_ip_front_sub, enable_ip_bottom_sub,
      enable_gate_server_sub, enable_bucket_server_sub, enable_flare_server_sub,
      enable_yellow_flare_server_sub;

  ros::ServiceServer conf_service;

  cv::Mat front_image, bottom_image;

  bool ip_status_front, ip_status_bottom, status_gate, status_bucket,
      status_flare,status_yellow_flare;

  ImageProcessor YF,RF,GT,BU;

  bool isEnabledFront();
  bool isEnabledBottom();
  bool isEnabledGateServer();
  bool isEnabledBucketServer();
  bool isEnabledFlareServer();

  bool isEnabledYellowFlareServer();
  bool resolve(int x,int y);

  void enableFront(const std_msgs::UInt8::ConstPtr &msg);
  void enableBottom(const std_msgs::UInt8::ConstPtr &msg);
  void process_next_image_front(const sensor_msgs::ImageConstPtr &msg);
  void process_next_image_bottom(const sensor_msgs::ImageConstPtr &msg);

  void enableGateServer(const std_msgs::UInt8::ConstPtr &msg);
  bool getGateCoordinates(simulator_sauvc_test::Coordinates::Request &req,
                          simulator_sauvc_test::Coordinates::Response &res);
  void imageProcessingGate();

  void enableBucketServer(const std_msgs::UInt8::ConstPtr &msg);
  bool getBucketCoordinates(simulator_sauvc_test::Coordinates::Request &req,
                            simulator_sauvc_test::Coordinates::Response &res);
  void imageProcessingBucket();

  void enableFlareServer(const std_msgs::UInt8::ConstPtr &msg);
  bool getFlareCoordinates(simulator_sauvc_test::Coordinates::Request &req,
                           simulator_sauvc_test::Coordinates::Response &res);
  void imageProcessingFlare();
// added ------
  void enableYellowFlareServer(const std_msgs::UInt8::ConstPtr &msg);
  bool getYellowFlareCoordinates(simulator_sauvc_test::Coordinates::Request &req,
                           simulator_sauvc_test::Coordinates::Response &res);
// end --------
};

#endif // SIMULATOR_SAUVC_TEST_H
