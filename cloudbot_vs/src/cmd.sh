#!/bin/bash
#achd push 159.203.67.159 15920367159VSTaskImg &
#achd pull 159.203.67.159 15920367159VSResp &
#roslaunch libuvc_camera libuvc_camera.launch &
#rosrun cloudbot_vs CloudVSWithRos.py


for i in `cat ipaddresses.txt`
do
foo=$i
achd push $i "${foo//./}"VSTaskImg &
achd pull $i "${foo//./}"VSResp &
done


