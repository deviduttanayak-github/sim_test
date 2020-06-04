#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "video_pub");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/front_camera/image_rect_color", 1);
  ros::Publisher pub_to_enable=nh.advertise<std_msgs::UInt8>("/enable_yellow_flare_server",1);
  ros::Publisher pub_cam=nh.advertise<std_msgs::UInt8>("/enable_ip_front_cam",1);

  cv::VideoCapture cap("/media/devid/D_D_N/simulator/videos/Gate_YFlare_Buckets.avi");

  if(!cap.isOpened()) return 1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;
  std_msgs::UInt8 k1,k2;
  k1.data=1;
  k2.data=1;

  ros::Rate loop_rate(15);

  while (nh.ok()) {
    cap >> frame;
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::imshow("open_cam",frame);
      cv::waitKey(1);
    }
    pub_to_enable.publish(k1);
    pub_cam.publish(k2);
    ROS_INFO("publishing");
    ros::spinOnce();
    loop_rate.sleep();
  }
}
