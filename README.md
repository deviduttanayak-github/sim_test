# sim_test
simulator_sauvc_test package is written to process images using opencv and gives the coordinates of the detected object.
client is a test package written to test server.
to run:
1. first build the 2 packages
2. If it shows conflicting files with same name(if u have sauvc2020 repo),then replace the simulator_sauvc_test with the one in this repo den build it separately. 
3. the start the server by 'rosrun simulator_sauvc_test simulator_sauvc_test'
4.make sure the video paths in this client_side/src/test_sim.cpp should be valid
5.There r 2 video paths in client_side/src/test_sim.cpp 
6.replace the first one with gate_YFlare_Buckets.avi from the link https://drive.google.com/drive/folders/1pCGvZHEk6q7VWagi3hgZysiZJcvMBvwI
7.replace the 2nd one with gate_YFlare_Buckets.mp4 which is available in the repo in the client_side folder
8. then to provide data to necessary topic run 'rosrun client_side test_ser code' [code = 0(for yellow flare),1(for gate),2(for red bucket),3(for red flare)]
9. then make request to server:
	rosrun client_side yellow 1	-> for yellow flare detection
	rosrun client_side gate 1	-> for gate detection
	rosrun client_side bucket_red 1	-> for red bucket detection
	rosrun client_side red 1	-> for yellow flare detection
10. u can see the requested bounding coordinates in the terminal and and bounding box drawn in the displayed frame named src
