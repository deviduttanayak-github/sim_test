#include "simulator_sauvc_test/image_processor.h"

ImageProcessor::ImageProcessor()
{
std::cout << "/* image_processor */" << '\n';
}

ImageProcessor::ImageProcessor(string path)
{
  ifstream confFile(path.c_str());

  string data;

  while(!confFile.eof())
  {
    confFile>>data;
    int k=data.find(":");
    string para=data.substr(0,k);
    string v=data.substr(k+1);
    int val=std::stoi(v);
    if(para == "thresh_l_B")
      thresh_l_B = val;
    else if(para == "thresh_l_G")
      thresh_l_G = val;
    else if(para == "thresh_l_R")
      thresh_l_R = val;
    else if(para == "thresh_h_B")
      thresh_h_B = val;
    else if(para == "thresh_h_G")
      thresh_h_G = val;
    else if(para == "thresh_h_R")
      thresh_h_R = val;
    else if(para == "canny_low_thresh")
      canny_low_thresh = val;
    else if(para == "canny_ratio")
      canny_ratio = val;
    else if(para == "canny_kernel_size")
      canny_kernel_size = val;
    else if(para == "hl_thresh_detect")
      hl_thresh_detect = val;
    else if(para == "hl_min_line_length")
      hl_min_line_length = val;
    else if(para == "hl_max_line_gap")
      hl_max_line_gap = val;
    else if(para == "morph_operator")
      morph_operator = val;
    else if(para == "morph_elem")
      morph_elem = val;
    else if(para == "morph_size")
      morph_size = val;
    else if(para == "x_parameter")
      x_parameter = val;
    else if(para == "y_parameter")
      y_parameter = val;
    else if(para == "speed_parameter")
      speed_parameter=val;
  }

  confFile.close();
  //parameters set
}


Mat ImageProcessor::morph_op(Mat src)
{

   Mat dest;
   int operation = morph_operator + 2;

   Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

     /// Apply the specified morphology operation
   morphologyEx( src, dest, operation, element );
   return dest;

}
float ImageProcessor::angle_finder(Vec4i line)
{
  float angle;
  if(line[2] - line[0]==0)
        angle=90;
      else
      {
        angle=atan((line[3]-line[1])/(line[2]-line[0]));
        angle=angle*180/PI;
      }
      return angle;
}

ImageProcessor::~ImageProcessor(){
  std::cout << "destructed" << '\n';

}
//-----function to filter out points------
bool ImageProcessor::filter_points(int x){
  for(int i=0; i<tl_x.size(); i++){
    if(abs(tl_x[i]-x)<15) return false;
  }
  tl_x.push_back(x);
  std::cout << "## filter_points : true : " << tl_x.size()<<'\n';
  return true;
}
