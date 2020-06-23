#!bin/bash
#demo code to run script file

echo "starting script..."
echo "------------- building simulator_sauvc_test -------------"
catkin build simulator_sauvc_test -j8
echo "------------- building finished -------------"
echo "------------- building client_test -------------"
catkin build client_test -j8
echo "------------- building finished -------------"
echo "------------- starting roscore -------------"
roscore
echo "------------- ready to run -------------"

