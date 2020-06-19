#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <ros/ros.h>
using namespace cv;
using namespace std;
Mat src,front_image,hsv_frame,gauss_frame,gauss_frame_2,gauss_frame_3,inrange_frame_1,inrange_frame_2,final_frame,final_morph_frame,final_morph_canny_frame;
int canny_low_thresh=0;
int canny_ratio=3;
int canny_kernel_size=3;
int morph_operator=0;
int morph_elem=0;
int morph_size=1;

Mat morph_op(Mat src)
{

   Mat dest;
   int operation = morph_operator + 2;

   Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

   morphologyEx( src, dest, operation, element );
   return dest;

}


int main(int argc,char **argv)
 {  cv::VideoCapture cap("/home/pratyush/Downloads/Gate_YFlare_Buckets.mp4");
   while(1)
   {cap>>src;
    front_image=src.clone();
    cv::cvtColor(front_image,hsv_frame, cv::COLOR_BGR2HSV);
    cv::blur(hsv_frame,gauss_frame, Size(3,3) );

    gauss_frame_3=gauss_frame.clone();
    cv::inRange(gauss_frame_3, Scalar(24,0,0), Scalar(130,255,255),gauss_frame_3);    

    gauss_frame_3=morph_op(gauss_frame_3);
    cv::Canny(gauss_frame_3,gauss_frame_3,canny_low_thresh,canny_ratio,canny_kernel_size);
    Mat frame=gauss_frame_3.clone();

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

      findContours(frame,contours, hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);
      vector<vector<Point> > contours_poly(contours.size());
      vector<Rect> boundRect(contours.size());
      vector<Point2f> centres(contours.size());
      int maxx=0;
      for(size_t i=0;i<contours.size();i++){
        approxPolyDP(contours[i],contours_poly[i],9,true);
        boundRect[i]=boundingRect(contours_poly[i]);
        int diff=(boundRect[i].br().x-boundRect[i].tl().x);
        maxx=max(maxx,(boundRect[i].br().x-boundRect[i].tl().x));
        if(diff>=50)    
        rectangle(front_image,boundRect[i].tl(), boundRect[i].br(),Scalar(0,255,0));}
        cout<<maxx<<endl;

    imshow("original frame",src);
    waitKey(1);
    imshow("test frame",gauss_frame_3);
    waitKey(1);
    imshow("contour frame",front_image);
    waitKey(1);
    

  } 
  return 0;
  }
