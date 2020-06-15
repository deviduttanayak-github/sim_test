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
  RF=ImageProcessor(ros::package::getPath("simulator_sauvc_test")+"/config/red_flare_ip.conf");
  BU=ImageProcessor(ros::package::getPath("simulator_sauvc_test")+"/config/bucket_ip.conf");
  GT=ImageProcessor(ros::package::getPath("simulator_sauvc_test")+"/config/gate_ip.conf");
  //GT_green=ImageProcessor(ros::package::getPath("simulator_sauvc_test")+"/config/gate_green_ip.conf");

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
    //imshow("server_side",front_image);
    //waitKey(30);
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

    if(req.dummy!=1){
      std::cout<<"\n## Request Denied ##\n";
      return false;
    }
    std::cout << "## Request Accepted ##" << '\n';
    cv::cvtColor(front_image,GT.hsv_frame, cv::COLOR_BGR2HSV);
    cv::blur( GT.hsv_frame,GT.gauss_frame, Size(3,3) );
    //GT_green.gauss_frame=GT_red.gauss_frame.clone();
    cv::inRange(GT.gauss_frame, Scalar(GT.thresh_l_B, GT.thresh_l_G, GT.thresh_l_R), Scalar(GT.thresh_h_B, GT.thresh_h_G ,GT.thresh_h_R),GT.gray_frame);
    //cv::inRange(GT_green.gauss_frame, Scalar(GT_green.thresh_l_B, GT_green.thresh_l_G, GT_green.thresh_l_R), Scalar(GT_green.thresh_h_B, GT_green.thresh_h_G ,GT_green.thresh_h_R),GT_green.gray_frame);
   // GT_red.gray_frame=(GT_red.gray_frame | GT_green.gray_frame);
    GT.morph_frame=GT.morph_op(GT.gray_frame);
    cv::Canny( GT.morph_frame,GT.canny_frame,GT.canny_low_thresh,GT.canny_ratio,GT.canny_kernel_size);

   /* vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(GT_red.canny_frame,contours, hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    vector<Point2f> centres(contours.size());
    for(size_t i=0;i<contours.size();i++){
      approxPolyDP(contours[i],contours_poly[i],9,true);
      boundRect[i]=boundingRect(contours_poly[i]);
    }
    //top-left(tl) and bottom-right(br)
    int min_x=1000,min_y=1000, max_x=0, max_y=0;
    for(size_t i=0;i<contours.size();i++){
      int x1,x2,y1,y2;
      min_x=min(min_x,(boundRect[i].tl()).x);
      max_x=max(max_x,(boundRect[i].br()).x);
      min_y=min(min_y,(boundRect[i].br()).y);
      max_y-max(max_y,(boundRect[i].tl()).y);
      }
    res.x.push_back(min_x);
    res.x.push_back(max_x);
    res.y.push_back(min_y);
    res.y.push_back(max_y);*/
   
   //--------using hough line------------
    
    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(GT.canny_frame, linesP, 1, CV_PI/180,50, 50, 10 ); // runs the actual detection
    
    float a,miny=1000,maxy=0,maxx=0;
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];
        if(l[2]!=l[0])
        {  float slope =(float) ((l[3] -l[1])/(l[2]-l[0]));
        a = -1*atan(slope)*(180/CV_PI);
    	}
       else a=90;

    if (abs(a)>85 && min(l[1],l[3])>150){
        float y=min(l[3],l[1]);
         miny=min(miny,y);
    float y2=max(l[3],l[1]);
         maxy=max(maxy,y2);
    float x= max(l[0],l[2]);
         maxx=max(maxx,x);
     ROS_INFO("detected gate");
    res.x.push_back(maxx);
    res.y.push_back(miny);
    res.y.push_back(maxy);
     //line( gauss_frame_3, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
     //imshow("Detected Lines (in red) - Probabilistic Line Transform", gauss_frame_3);
     //waitKey(1);
    }  }
    int r=maxy-miny;
    //-------written in client side---------
   /* if(maxx!=0 && maxy!=0 && r<190)
        {  //ROS_INFO("inside loop");
           line( front_image, Point(maxx,miny), Point(maxx,maxy), Scalar(0,0,255), 3, LINE_AA);
           line( front_image, Point(maxx,miny), Point((maxx+r*1.732),miny), Scalar(0,0,255), 3, LINE_AA);
           line( front_image, Point(maxx,maxy), Point((maxx+r*1.732),maxy), Scalar(0,0,255), 3, LINE_AA);
           line( front_image, Point((maxx+r*1.732), miny), Point((maxx+r*1.732),maxy), Scalar(0,0,255), 3, LINE_AA);
           circle(front_image,Point((maxx+r*0.860),(maxy+miny)/2),400/32,Scalar(255,0,0),FILLED,LINE_8);
               
        }*/
    res.x.push_back(maxx);
    res.y.push_back(miny);
    res.y.push_back(maxy);

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

      if(req.dummy!=1){
        std::cout<<"\n## Request Denied ##\n";
        return false;
      }
      std::cout << "## Request Accepted ##" << '\n';
      cv::cvtColor(front_image,BU.hsv_frame, cv::COLOR_BGR2HSV);
      cv::blur( BU.hsv_frame,BU.gauss_frame, Size(3,3) );
      cv::inRange(BU.gauss_frame, Scalar(BU.thresh_l_B, BU.thresh_l_G, BU.thresh_l_R), Scalar(BU.thresh_h_B, BU.thresh_h_G ,BU.thresh_h_R),BU.gray_frame);
      BU.morph_frame=BU.morph_op(BU.gray_frame);
      cv::Canny( BU.morph_frame,BU.canny_frame,BU.canny_low_thresh,BU.canny_ratio,BU.canny_kernel_size );
      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;

      findContours(BU.canny_frame,contours, hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);
      vector<vector<Point> > contours_poly(contours.size());
      vector<Rect> boundRect(contours.size());
      vector<Point2f> centres(contours.size());
      for(size_t i=0;i<contours.size();i++){
        approxPolyDP(contours[i],contours_poly[i],9,true);
        boundRect[i]=boundingRect(contours_poly[i]);
      }

      for(size_t i=0;i<contours.size();i++){
        if(BU.filter_points((boundRect[i].tl()).x)){
          res.x.push_back(boundRect[i].tl().x);
          res.y.push_back(boundRect[i].tl().y);
          res.x.push_back(boundRect[i].br().x);
          res.y.push_back(boundRect[i].br().y);
        }
      }
    BU.tl_x.clear();
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

      if(req.dummy!=1){
        std::cout<<"\n## Request Denied ##\n";
        return false;
      }
      std::cout << "## Request Accepted ##" << '\n';
      cv::cvtColor(front_image,RF.hsv_frame, cv::COLOR_BGR2HSV);
      cv::blur( RF.hsv_frame,RF.gauss_frame, Size(3,3) );
      cv::inRange(RF.gauss_frame, Scalar(RF.thresh_l_B, RF.thresh_l_G, RF.thresh_l_R), Scalar(RF.thresh_h_B, RF.thresh_h_G ,RF.thresh_h_R),RF.gray_frame);
      RF.morph_frame=RF.morph_op(RF.gray_frame);
      cv::Canny( RF.morph_frame,RF.canny_frame,RF. canny_low_thresh,RF.canny_ratio,RF.canny_kernel_size );
      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;

      findContours(RF.canny_frame,contours, hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);
      vector<vector<Point> > contours_poly(contours.size());
      vector<Rect> boundRect(contours.size());
      vector<Point2f> centres(contours.size());
      for(size_t i=0;i<contours.size();i++){
        approxPolyDP(contours[i],contours_poly[i],9,true);
        boundRect[i]=boundingRect(contours_poly[i]);
      }

      for(size_t i=0;i<contours.size();i++){
        if(RF.filter_points((boundRect[i].tl()).x)){
          res.x.push_back(boundRect[i].tl().x);
          res.y.push_back(boundRect[i].tl().y);
          res.x.push_back(boundRect[i].br().x);
          res.y.push_back(boundRect[i].br().y);
        }
      }
    RF.tl_x.clear();
    return true;
}

void SimSauvcTest::imageProcessingFlare() {}

//-------------end flare server----------------------------------------------

// ---------- yellow_flare -------------------------------------------------
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
    //imshow("server_canny",YF.canny_frame);
    //waitKey(200);

    findContours(YF.canny_frame,contours, hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    vector<Point2f> centres(contours.size());
    //cout<<"contours size: "<<contours.size()<<endl;
    for(size_t i=0;i<contours.size();i++){
      approxPolyDP(contours[i],contours_poly[i],9,true);
      boundRect[i]=boundingRect(contours_poly[i]);
    }

    for(size_t i=0;i<contours.size();i++){
      //cout<<(boundRect[i].tl()).x<<" "<<(boundRect[i].tl()).y<<endl;
      //cout<<(boundRect[i].br()).x<<" "<<(boundRect[i].br()).y<<endl;
      if(YF.filter_points((boundRect[i].tl()).x)){
        res.x.push_back(boundRect[i].tl().x);
        res.y.push_back(boundRect[i].tl().y);
        res.x.push_back(boundRect[i].br().x);
        res.y.push_back(boundRect[i].br().y);
      }
    // drawContours(drawing,contours_poly,int(i),Scalar(0,255,0));
    //cout<<"## array size ##\n";
    //cout<<YF.tl_x.size()<<" : "<<i<<" \n";
    rectangle(front_image,boundRect[i].tl(), boundRect[i].br(),Scalar(0,255,0));
    }
    imshow("serve_final_bbox",front_image);
    waitKey(1);
    // waitKey(300);
    // end contouring-------
  //cout<<"tl_x.size():"<<tl_x.size();
  cout<<"\nresponse array size : "<<res.x.size();
  YF.tl_x.clear();
  return true;

}

// ---------- end ---------------------------------------------------------
