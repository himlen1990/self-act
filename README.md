# self-act

How to use:
#0.install caffe dependencies:
sudo apt-get install libatlas-base-dev libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
#1.build caffe:

cd self-act/caffe

make all -j5

make distribute

if you want the system run faster, compile caffe with CUDA. 

#2. build ros package

cd self-act

catkin bt

#3.run trajectory-prediciton

roscore

roslaunch self-act traj.launch

this will open rviz

add display type "Path" in rviz

set the topic to trajectory_predict_leftHand

add another display type "Path" in rviz

set the topic to trajectory_predict_RightHand


#4.run package

roscd self-act

rosrun self-act self_rec

#5.play rosbag

rosrun rosbag play xxx.bag

#6.abnormal behavior detect

rosrun self-act killnode.py

do step 4 and 5 again