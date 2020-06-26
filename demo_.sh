#!bin/bash

echo "killing if roscore has started..."
killall -9 roscore

echo "starting script..."
echo "building simulator_sauvc_test..."
catkin build simulator_sauvc_test -j8 
echo "building client_test..."
catkin build client_side -j8
echo "building finished..."

echo "starting roscore..."
roscore &
sleep 2
echo "starting server-side..."
rosrun simulator_sauvc_test simulator_sauvc_test &
sleep 3

echo "what you want to start? type the corresponding code.."
echo "bu - bucket part"
echo "yf - yellow_flare part"
echo "rf - red_flare part"
echo "gt - gate part"
echo -n "enter your choice$ "
read choice

echo "starting client_side test_ser..."

if [ $choice = "bu" ]
then
	rosrun client_side test_ser 2 &
	sleep 2
	echo "starting bucket..."
	rosrun client_side bucket_red 1
fi

if [ $choice = "yf" ]
then
	rosrun client_side test_ser 0 &
	sleep 2
	echo "starting yellow flare..."
	rosrun client_side yellow 1
fi

if [ $choice = "rf" ]
then
	rosrun client_side test_ser 3 &
	sleep 2
	echo "starting red flare..."
	rosrun client_side red 1
fi

if [ $choice = "gt" ]
then
	rosrun client_side test_ser 1 &
	sleep 2
	echo "starting gate..."
	rosrun client_side gate 1
fi

sleep 1

while [ 1 ]; do
	echo -n "to end Enter (end/e/E)$ "
	read getend
	if [ $getend = "end" ] || [ $getend = "e" ] || [ $getend = "E" ]
	then
		echo "exiting..."
		break
	fi
done
exit 0

