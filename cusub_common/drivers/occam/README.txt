source /opt/ros/indigo/setup.bash
mkdir build
cd build
cmake -DINDIGOSDK_PATH=`pwd`/../../indigosdk-2-0 ..

source /opt/ros/indigo/setup.bash && cd /home/x/ros_indigosdk-2-0/build && make -kj

