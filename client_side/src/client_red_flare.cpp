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
simulator_sauvc_test::Coordinates YF_server;
ros::ServiceClient YF_client;

void imageCallback(const sensor_msgs::ImageConstPtr& frame) {
  cout<<"## inside callback ##\n";
  //-----print the coordinates------
  if(YF_client.call(YF_server)){
    ROS_INFO("Coordinates are: [%f,%f],[%f,%f],[%f,%f],[%f,%f] ##",
              YF_server.response.x[0],YF_server.response.y[0],
              YF_server.response.x[1],YF_server.response.y[1]);
  }
  else{
    ROS_INFO("Failed to call service : [yellow_flare_coordinates]");
  }


  cv::Mat src1;
  try{
    src=cv_bridge::toCvShare(frame, "bgr8")->image;
    ROS_INFO("[Image Received]\n");
  }catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
              frame->encoding.c_str());
        }
  src1=src.clone();
  rectangle(src1,Point(YF_server.response.x[1],YF_server.response.y[1]), Point(YF_server.response.x[0],YF_server.response.y[0]), Scalar(0,0,255),2,8,0);
  cv::imshow("src",src1);
  waitKey(1);

}


int main(int argc, char **argv){
  ros::init(argc,argv,"flare_coordinates_client");
  if(argc!=2){
    ROS_INFO("[usage]: enter 1 to request for image_processing");
  }
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber YF_sub=it.subscribe("/front_camera/image_rect_color", 1,
                     imageCallback);
  YF_client=nh.serviceClient<simulator_sauvc_test::Coordinates>("flare_coordinates");

  YF_server.request.dummy=atoll(argv[1]);

  ros::spin();
  return 0;
}

