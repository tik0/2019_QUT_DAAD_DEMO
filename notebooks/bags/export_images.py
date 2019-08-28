#!/usr/bin/python3
from subprocess import Popen
import time

print('Start -- extract images')
p_r = Popen('. /opt/ros/kinetic/setup.sh; roscore', shell=True, bufsize=-1)
time.sleep(1.5)
p = Popen('. /opt/ros/kinetic/setup.sh; rosparam set use_sim_time false', shell=True, bufsize=-1)
p.wait()
p_i = Popen('. /opt/ros/kinetic/setup.sh; rosrun image_view image_saver image:=/camera/color/image_rect_color _filename_format:=%04d.png', shell=True, bufsize=-1)
time.sleep(1.5)
p_b = Popen('. /opt/ros/kinetic/setup.sh; for bag in $(ls *bag); do rosbag play --clock --wait-for-subscribers --topics /camera/color/image_rect_color -r 1 ${bag}; done', shell=True, bufsize=-1)
p_b.wait()
p_i.terminate()
p_r.terminate()
time.sleep(1.5)
p = Popen('killall roscore; killall image_saver; killall rosbag; killall rosmaster', shell=True, bufsize=-1)
p.wait()
print('Done -- extract images')
