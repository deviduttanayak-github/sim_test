#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H
#include <iostream>
#include <limits.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <cmath>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#define PI 3.14159265

using namespace cv;
using namespace std;

class ImageProcessor
{
public:
  ImageProcessor();
  ImageProcessor(string path);
  ~ImageProcessor();

  Mat initial_frame, hsv_frame, gauss_frame, erosion_frame, dilation_frame,
      gray_frame, gray_frame_1, canny_frame,morph_frame, out_frame;

  Mat element;

  int thresh_l_B , thresh_l_G , thresh_l_R ;
  int thresh_h_B , thresh_h_G , thresh_h_R ;
  int canny_low_thresh , canny_ratio , canny_kernel_size ;
  int hl_thresh_detect , hl_min_line_length , hl_max_line_gap;
  int morph_operator ,morph_elem ,morph_size;

  int x_parameter , y_parameter;

  int speed_parameter;

  float angle;

  std::vector<Vec4i> lines;

  Vec4i line1,line2;

  Mat morph_op(Mat src);

  float angle_finder(Vec4i line);

};
#endif
