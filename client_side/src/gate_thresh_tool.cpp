#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
using namespace cv;
using namespace std;

  //Mat frame,fra
  Mat frame,frame_hsv,dst,frame_threshold,erosion_dst, dilation_dst;
  Mat grad,cdst;
  Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

const int max_value_H = 360/2;
  const int max_value = 255;
  const String window_detection_name = "view3";
  int low_H = 0, low_S = 0, low_V = 0;
  int high_H = max_value_H, high_S = max_value, high_V = max_value;
const String window_name = "smoothing Demo";
const String trackbar_value = "View4";
  int i_value = 0;
  int M_value=15;
  int DELAY_BLUR = 100; 
  int i;
  int ddepth=CV_16S;
  int ksize;
  int kSize;
  int scale=1;
  int delta=0;
  

int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;


void Erosion( int, void* );
void Dilation( int, void* );
void smooth_Demo( int, void* );


void sobel_Demo (int,void*)
{Sobel(frame_threshold, grad_x, ddepth, 1, 0, kSize, scale, delta, BORDER_DEFAULT);
    Sobel(frame_threshold, grad_y, ddepth, 0, 1, kSize, scale, delta, BORDER_DEFAULT);
    // converting back to CV_8U
    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
    imshow("sobel", grad);
    char key = (char)waitKey(1);
}

static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}




int main(int argc,char** argv)
{


  VideoCapture cap("/home/pratyush/Downloads/Gate_YFlare_Buckets.avi");
//VideoWriter vd("/home/prats/catkin_ws/src/saved1.mp4",CV_FOURCC('M','J','P','G'),15,Size(100,100),true);

  while (1) {
    cap>>frame;
    
   // waitKey(50);
    cvtColor(frame,frame_hsv,COLOR_BGR2HSV);
    imshow("view2",frame_hsv); 
    waitKey(1);
   
   namedWindow(window_detection_name);
  
   // Trackbars to set thresholds for HSV values
    createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
       
    
        // Detect the object based on HSV Range Values
        inRange(frame_hsv, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        // Show the frames
        imshow(window_detection_name, frame_threshold);
       
        createTrackbar( trackbar_value,
                  window_name, &i_value,
                  M_value, smooth_Demo );
       
          i=(i_value)*2+1;
           
         
  namedWindow( "Erosion Demo", WINDOW_AUTOSIZE );
  namedWindow( "Dilation Demo", WINDOW_AUTOSIZE );
  moveWindow( "Dilation Demo", dst.cols, 0 );
  createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo",
          &erosion_elem, max_elem,
          Erosion );
  createTrackbar( "Kernel size:\n 2n +1", "Erosion Demo",
          &erosion_size, max_kernel_size,
          Erosion );
  createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation Demo",
          &dilation_elem, max_elem,
          Dilation );
  createTrackbar( "Kernel size:\n 2n +1", "Dilation Demo",
          &dilation_size, max_kernel_size,
          Dilation );
  sobel_Demo(0,0);
  waitKey(1);
  Erosion( 0, 0 );
  smooth_Demo(0,0);
  Dilation( 0, 0 );
  vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(grad, linesP, 1, CV_PI/180,50, 50, 10 ); // runs the actual detection
    // Draw the lines
     Mat cdstP,src;
   cvtColor(grad, cdst, COLOR_GRAY2BGR);
    cdstP = cdst.clone();
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
        miny =min(miny,y);
    float y2=max(l[3],l[1]);
         maxy=max(maxy,y2);
    float x= max(l[0],l[2]);
         maxx=max(maxx,x);

     line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
     imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);
  waitKey(1);
    }  }
    int r=maxy-miny;
    if(maxx!=0 && maxy!=0 && r<190)
        {  ROS_INFO("inside loop");
           line( frame, Point(maxx,miny), Point(maxx,maxy), Scalar(0,0,255), 3, LINE_AA);
           line( frame, Point(maxx,miny), Point((maxx+r*1.732),miny), Scalar(0,0,255), 3, LINE_AA);
          line( frame, Point(maxx,maxy), Point((maxx+r*1.732),maxy), Scalar(0,0,255), 3, LINE_AA);
            line( frame, Point((maxx+r*1.732), miny), Point((maxx+r*1.732),maxy), Scalar(0,0,255), 3, LINE_AA);
        circle(frame,Point((maxx+r*0.860),(maxy+miny)/2),400/32,Scalar(255,0,0),FILLED,LINE_8);
               
}
     imshow("view",frame);   
                                                                                                                                                               
    
    // Show results
    //imshow("Source", src);
    //imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
    
    

}
return 0;
}

/*vd.write(dilation_dst);i
  int key=waitKey(100);
    if(key==27)break;*/
    
    

void smooth_Demo( int, void* ){

      blur( erosion_dst, dst, Size( i, i ), Point(-1,-1) );
      imshow( window_name, dst );    
}
void Erosion( int, void* )
{
  int erosion_type = 0;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
  Mat element = getStructuringElement( erosion_type,
                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       Point( erosion_size, erosion_size ) );
  erode( frame_threshold, erosion_dst, element );
  imshow( "Erosion Demo", erosion_dst );
}
void Dilation( int, void* )
{
  int dilation_type = 0;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
  Mat element = getStructuringElement( dilation_type,
                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       Point( dilation_size, dilation_size ) );
  dilate( dst, dilation_dst, element );
  imshow( "Dilation Demo", dilation_dst );
}
        
      
   

