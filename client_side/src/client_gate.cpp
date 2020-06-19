#include "ros/ros.h"
#include "simulator_sauvc_test/Coordinates.h"
#include <cstdlib>
#include <iostream>
#include <std_msgs/Int8.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include <simulator_sauvc_test/Coordinates.h>

using namespace std;
using namespace cv;

cv::Mat src;
simulator_sauvc_test::Coordinates gate_server;
ros::ServiceClient gate_client;

void make_boundig_box(float x1,float x2,float y1,float y2);

void imageCallback(const sensor_msgs::ImageConstPtr& frame) {
  cout<<"## inside callback ##\n";
  if(gate_client.call(gate_server)){
    ROS_INFO("Coordinates are: [%f,%f,%f,%f] ##",
              gate_server.response.x[0],gate_server.response.y[0],
              gate_server.response.x[1],gate_server.response.y[1]);

  cv::Mat src1;
  try{
    src=cv_bridge::toCvShare(frame, "bgr8")->image;
    ROS_INFO("[Image Received]\n");
  }catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
              frame->encoding.c_str());
        }
  src1=src.clone();

  src1=src.clone();
  //----do the bounding box from coordinates---
  rectangle(src1,Point(gate_server.response.x[0],gate_server.response.y[0]), Point(gate_server.response.x[1],gate_server.response.y[1]), Scalar(0,0,255),2,8,0);
  cv::imshow("src",src1);
  waitKey(10);

    }
    
  else{
    ROS_INFO("Failed to call service : [gate_flare_coordinates]");
  }


}


int main(int argc, char **argv){
  ros::init(argc,argv,"gate_coordinates_client");
  if(argc!=2){
    ROS_INFO("[usage]: enter 1 to request for image_processing");
  }
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber YF_sub=it.subscribe("/front_camera/image_rect_color", 1,
                     imageCallback);
  gate_client=nh.serviceClient<simulator_sauvc_test::Coordinates>("gate_coordinates");

  gate_server.request.dummy=atoll(argv[1]);

  ros::spin();
  return 0;
}

