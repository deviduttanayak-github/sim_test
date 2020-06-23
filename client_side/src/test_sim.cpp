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

  ros::Publisher enable_yellow_flare_server_pub=nh.advertise<std_msgs::UInt8>("/enable_yellow_flare_server",1);
  ros::Publisher enable_gate_server_pub=nh.advertise<std_msgs::UInt8>("/enable_gate_server",1);
  ros::Publisher enable_bucket_server_pub = nh.advertise<std_msgs::UInt8>("/enable_bucket_server", 1);
  ros::Publisher enable_flare_server_pub = nh.advertise<std_msgs::UInt8>("/enable_flare_server", 1);


  cv::VideoCapture cap("/home/devid/catkin_ws/src/client_side/Gate_YFlare_Buckets.mp4");//gate yflare buckets video

  cv::VideoCapture cap2("/home/pratyush/Downloads/Gate_YFlare_Buckets.avi");//gate yflare bucktes video(containing only buckets part)

  if(!cap.isOpened()) return 1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;
  std_msgs::UInt8 k1,k2;
  k1.data=1;
  k2.data=1;
  //----code for yellow flare 0,gate 1,bucket 2,red flare 3------
  int code=atoll(argv[1]);

  ros::Rate loop_rate(15);

  while (nh.ok()) {
    if(code!=2)
    cap2 >> frame;
    else
    cap>>frame;
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::imshow("open_cam",frame);
      cv::waitKey(30);
    }
    pub_cam.publish(k1);
    if(code==0)
      enable_yellow_flare_server_pub.publish(k2);
    if(code==1)
      enable_gate_server_pub.publish(k2);
    if(code==2)
      enable_bucket_server_pub.publish(k2);
    if(code==3)
      enable_flare_server_pub.publish(k2);
    //ROS_INFO("publishing");
    ros::spinOnce();
    loop_rate.sleep();
  }
}
