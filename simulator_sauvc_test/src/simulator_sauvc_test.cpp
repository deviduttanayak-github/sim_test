#include "simulator_sauvc_test/simulator_sauvc_test.h"
using namespace std;
using namespace cv;

SimSauvcTest::SimSauvcTest(ros::NodeHandle _nh) : nh(_nh), it(_nh) {
  enable_ip_front_sub =
      nh.subscribe("/enable_ip_front_cam", 1, &SimSauvcTest::enableFront, this);
  enable_ip_bottom_sub = nh.subscribe("/enable_ip_bottom_cam", 1,
                                      &SimSauvcTest::enableBottom, this);
  enable_gate_server_sub = nh.subscribe("/enable_gate_server", 1,
                                        &SimSauvcTest::enableGateServer, this);
  enable_bucket_server_sub = nh.subscribe(
      "/enable_bucket_server", 1, &SimSauvcTest::enableBucketServer, this);
  enable_flare_server_sub = nh.subscribe(
      "/enable_flare_server", 1, &SimSauvcTest::enableFlareServer, this);

  enable_yellow_flare_server_sub = nh.subscribe(
      "/enable_yellow_flare_server", 1, &SimSauvcTest::enableYellowFlareServer, this);

  YF=ImageProcessor(ros::package::getPath("simulator_sauvc_test")+"/config/yellow_flare_ip.conf");

  status_gate = status_bucket = status_flare = status_yellow_flare =0;
}

SimSauvcTest::~SimSauvcTest() {}

void SimSauvcTest::enableFront(const std_msgs::UInt8::ConstPtr &msg) {

  if (isEnabledFront() == (msg->data))
    return;

  if (msg->data) {
    ip_status_front = 1;
    front_camera_sub =
        it.subscribe("/front_camera/image_rect_color", 1,
                     &SimSauvcTest::process_next_image_front, this);

  } else {

    ip_status_front = 0;
    front_camera_sub.shutdown();
  }
}

void SimSauvcTest::process_next_image_front(
    const sensor_msgs::ImageConstPtr &image_frame) {

  try {
    cv_bridge::toCvShare(image_frame, "bgr8")->image.copyTo(front_image);
    imshow("server_side",front_image);
    waitKey(30);
    //ROS_INFO("processing next image front camera");
    //flip(front_image, front_image, -1);
    // flip(frame_bottom,frame_bottom,0);

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
              image_frame->encoding.c_str());
    return;
  }
}

void SimSauvcTest::enableBottom(const std_msgs::UInt8::ConstPtr &msg) {

  if (isEnabledBottom() == (msg->data))
    return;

  if (msg->data) {
    ip_status_bottom = 1;
    bottom_camera_sub =
        it.subscribe("/bottom_camera/image_rect_color", 1,
                     &SimSauvcTest::process_next_image_bottom, this);
  } else {

    ip_status_bottom = 0;
    bottom_camera_sub.shutdown();
  }
}

void SimSauvcTest::process_next_image_bottom(
    const sensor_msgs::ImageConstPtr &image_frame) {

  try {
    cv_bridge::toCvShare(image_frame, "bgr8")->image.copyTo(bottom_image);
    ROS_INFO("processing next image bottom camera");
    flip(front_image, front_image, -1);
    // flip(frame_bottom,frame_bottom,0);

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
              image_frame->encoding.c_str());
    return;
  }
}

bool SimSauvcTest::isEnabledFront() { return ip_status_front; }
bool SimSauvcTest::isEnabledBottom() { return ip_status_bottom; }
bool SimSauvcTest::isEnabledGateServer() { return status_gate; }
bool SimSauvcTest::isEnabledBucketServer() { return status_bucket; }
bool SimSauvcTest::isEnabledFlareServer() { return status_flare; }
bool SimSauvcTest::isEnabledYellowFlareServer() { return status_yellow_flare; }

//--------Gate server----------------------------------------------------

void SimSauvcTest::enableGateServer(const std_msgs::UInt8::ConstPtr &msg) {
  if (isEnabledGateServer() == (msg->data))
    return;

  if (msg->data) {
    status_gate = 1;
    conf_service = nh.advertiseService("gate_coordinates",
                                       &SimSauvcTest::getGateCoordinates, this);
    ROS_INFO("Gate server started");
  } else {
    status_gate = 0;
    conf_service.shutdown();
    ROS_INFO("Gate server shutdown");
  }
}

bool SimSauvcTest::getGateCoordinates(
    simulator_sauvc_test::Coordinates::Request &req,
    simulator_sauvc_test::Coordinates::Response &res) {
  return true;
}

void SimSauvcTest::imageProcessingGate() {}

//-------------end gate server----------------------------------------------

//--------Bucket server----------------------------------------------------

void SimSauvcTest::enableBucketServer(const std_msgs::UInt8::ConstPtr &msg) {
  if (isEnabledBucketServer() == (msg->data))
    return;

  if (msg->data) {
    status_bucket = 1;
    conf_service = nh.advertiseService(
        "bucket_coordinates", &SimSauvcTest::getBucketCoordinates, this);
    ROS_INFO("Bucket server started");
  } else {
    status_bucket = 0;
    conf_service.shutdown();
    ROS_INFO("Bucket server started");
  }
}

bool SimSauvcTest::getBucketCoordinates(
    simulator_sauvc_test::Coordinates::Request &req,
    simulator_sauvc_test::Coordinates::Response &res) {
  return true;
}

void SimSauvcTest::imageProcessingBucket() {}
//-------------end bucket server----------------------------------------------

//--------Flare server----------------------------------------------------

void SimSauvcTest::enableFlareServer(const std_msgs::UInt8::ConstPtr &msg) {
  if (isEnabledFlareServer() == (msg->data))
    return;

  if (msg->data) {
    status_flare = 1;
    conf_service = nh.advertiseService(
        "flare_coordinates", &SimSauvcTest::getFlareCoordinates, this);
    ROS_INFO("Flare server started");
  } else {
    status_flare = 0;
    conf_service.shutdown();
    ROS_INFO("Flare server shutdown");
  }
}

bool SimSauvcTest::getFlareCoordinates(
    simulator_sauvc_test::Coordinates::Request &req,
    simulator_sauvc_test::Coordinates::Response &res) {
  return true;
}

void SimSauvcTest::imageProcessingFlare() {}

//-------------end flare server----------------------------------------------

// ---------- yellow_flare -------------------------------------------------
std::vector<int> tl_x, tl_y;
// int thresh_l_B = 0, thresh_l_G = 112, thresh_l_R = 43;
// int thresh_h_B = 51, thresh_h_G = 255, thresh_h_R = 255;
//
// int canny_low_thresh = 0, canny_ratio = 3, canny_kernel_size = 3;
// int hl_thresh_detect = 50, hl_min_line_length = 1, hl_max_line_gap = 1;//change the values
//
// int morph_operator = 0, morph_elem = 0, morph_size = 1;
// int x_order=2, d_depth=CV_16S;
// double scale=1,delta=0;
// int sobel_kernel_size=1;
// int dilation_size=2;
// int x_parameter=50 , y_parameter=20;

void SimSauvcTest::enableYellowFlareServer(const std_msgs::UInt8::ConstPtr &msg) {
  //std::cout << "/* message */" << '\n';
  if (isEnabledYellowFlareServer() == (msg->data))
    return;
  std::cout<<msg->data<<'\n';

  if (msg->data) {
    status_yellow_flare = 1;
    conf_service = nh.advertiseService(
        "yellow_flare_coordinates", &SimSauvcTest::getYellowFlareCoordinates, this);
    ROS_INFO("Yellow Flare server started");
  } else {
    status_yellow_flare = 0;
    conf_service.shutdown();
    ROS_INFO("Yellow Flare server shutdown");
  }
}

bool SimSauvcTest::getYellowFlareCoordinates(
    simulator_sauvc_test::Coordinates::Request &req,
    simulator_sauvc_test::Coordinates::Response &res) {

    if(req.dummy!=1){
      std::cout<<"\n## Request Denied ##\n";
      return false;
    }
    std::cout << "## Request Accepted ##" << '\n';
    cv::cvtColor(front_image,YF.hsv_frame, cv::COLOR_BGR2HSV);
    cv::blur( YF.hsv_frame,YF.gauss_frame, Size(3,3) );
    cv::inRange(YF.gauss_frame, Scalar(YF.thresh_l_B, YF.thresh_l_G, YF.thresh_l_R), Scalar(YF.thresh_h_B, YF.thresh_h_G ,YF.thresh_h_R),YF.gray_frame);
    YF.morph_frame=YF.morph_op(YF.gray_frame);
    cv::Canny( YF.morph_frame,YF.canny_frame,YF. canny_low_thresh,YF.canny_ratio,YF.canny_kernel_size );
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    imshow("server_canny",YF.canny_frame);
    waitKey(200);

    findContours(YF.canny_frame,contours, hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    vector<Point2f> centres(contours.size());
    cout<<"contours size: "<<contours.size()<<endl;
    for(size_t i=0;i<contours.size();i++){
    approxPolyDP(contours[i],contours_poly[i],9,true);
    boundRect[i]=boundingRect(contours_poly[i]);
    }

    for(size_t i=0;i<contours.size();i++){
      cout<<(boundRect[i].tl()).x<<" "<<(boundRect[i].tl()).y<<endl;
      cout<<(boundRect[i].br()).x<<" "<<(boundRect[i].br()).y<<endl;
      if(resolve((boundRect[i].tl()).x,(boundRect[i].tl()).y)){
        res.x.push_back(boundRect[i].tl().x);
        res.y.push_back(boundRect[i].tl().y);
        res.x.push_back(boundRect[i].br().x);
        res.y.push_back(boundRect[i].br().y);
      }
    // drawContours(drawing,contours_poly,int(i),Scalar(0,255,0));
    // rectangle(front_image,boundRect[i].tl(), boundRect[i].br(),Scalar(0,255,0));
    }
    imshow("serve_final_bbox",front_image);
    waitKey(300);
    // end contouring-------
  //cout<<"tl_x.size():"<<tl_x.size();
  cout<<"\nresponse array size : "<<res.x.size();
  tl_x.clear(); tl_y.clear();
  return true;

}

bool SimSauvcTest::resolve(int x, int y){
  for(int i=0; i<tl_x.size(); i++){
    if(abs(tl_x[i]-x)<15) return false;
    else if(abs(tl_y[i]-y)<15) return false;
  }
  tl_x.push_back(x); tl_y.push_back(y);
  return true;
}


// ---------- end ---------------------------------------------------------
