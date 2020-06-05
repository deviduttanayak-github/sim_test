#include "simulator_sauvc_test/simulator_sauvc_test.h"
#include "simulator_sauvc_test/image_processor.h"
SimSauvcTest *obj;

int main(int argc, char **argv) {
  ros::init(argc, argv, "simulator_sauvc_test");
  ros::NodeHandle nh;
  ros::Rate loop_rate(30);

  obj = new SimSauvcTest(nh);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    // add code here
  }

  return 0;
}
