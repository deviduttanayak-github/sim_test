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

  ros::Publisher pub_cam=nh.advertise<std_msgs::UInt8>("/enable_ip_front_cam",1);

  image_transport::Publisher pub = it.advertise("/front_camera/image_rect_color", 1);

  //ros::Publisher enable_yellow_flare_server_pub=nh.advertise<std_msgs::UInt8>("/enable_yellow_flare_server",1);
  ros::Publisher enable_gate_server_pub=nh.advertise<std_msgs::UInt8>("/enable_gate_server",1);
  //ros::Publisher enable_bucket_server_pub = nh.advertise<std_msgs::UInt8>("/enable_bucket_server", 1);
  //ros::Publisher enable_flare_server_pub = nh.advertise<std_msgs::UInt8>("/enable_flare_server", 1);


  cv::VideoCapture cap("/media/devid/D_D_N/simulator/videos/Gate_YFlare_Buckets.avi");

  if(!cap.isOpened()) return 1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;
  std_msgs::UInt8 k1,k2,k3,k4,k5;
  k1.data=1;
  k2.data=1;//for yellow flare
  k3.data=1;//for gate
  k4.data=1;//for bucket
  k5.data=1;//for red flare

  ros::Rate loop_rate(15);

  while (nh.ok()) {
    cap >> frame;
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::imshow("open_cam",frame);
      cv::waitKey(30);
    }
    pub_cam.publish(k1);
    //enable_yellow_flare_server_pub.publish(k2);
    enable_gate_server_pub.publish(k3);
    //enable_bucket_server_pub.publish(k4);
    //enable_flare_server_pub.publish(k5);
    ROS_INFO("publishing");
    ros::spinOnce();
    loop_rate.sleep();
  }
}
