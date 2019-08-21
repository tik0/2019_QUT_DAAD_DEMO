#!/usr/bin/python3
from subprocess import Popen
import numpy as np
import time

# gz model --model-name camera -x 0.2 -y -0.0 -z 0.05 -R 0.0 -P 0.0 -Y $(echo  "3.14" | bc )
yaw_offset = np.pi
z_offset = 0.05 # helf hight of cube
#pitch_steps = 90 # in 1 deg steps
#yaw_steps = 360 # in 1 deg steps
#pitch_steps = 45 # in 2 deg steps
#yaw_steps = 180 # in 2 deg steps
pitch_steps = 9 # in 10 deg steps
yaw_steps = 36 # in 10 deg steps
#pitch_steps = 2 # in 10 deg steps
#yaw_steps = 4 # in 10 deg steps

# Radius of sphere on which the camera trajectory is defined
r = 0.2

# killall
def killall():
	p = Popen('killall rosmaster; killall gzserver; killall play; killall gzclient', shell=True) # setting the camera to some dummy position to avoid wired samping 
	p.wait()
# Dump function
def dump_images():
	# dump two messages which is the current buffer size of the camera plugin
	# https://github.com/ros-simulation/gazebo_ros_pkgs/blob/dd51b73ccc54a54ae41dd5c4901252a49ca51177/gazebo_plugins/src/gazebo_ros_camera_utils.cpp#L317
	p = Popen('rostopic echo -n 2 /camera/cam/image_raw > /dev/null', shell=True)
	p.wait()
# Current position
def pose(_pitch, _yaw):
	x = np.cos(_pitch) * np.cos(_yaw) * r
	y = np.cos(_pitch) * np.sin(_yaw) * r
	z = z_offset + np.sin(_pitch) * r
	roll = 0.0
	pitch = _pitch
	yaw = yaw_offset + _yaw
	return x, y, z, roll, pitch, yaw

killall()
# Sampling the trajectory
for _pitch, _pitch_iter in zip(np.linspace(0.0, np.pi / 2.0, num=pitch_steps+1, endpoint=True), range(0, pitch_steps+1)):
	# Setting the position in the sphere
	x, y, z, roll, pitch, yaw = pose(_pitch, 0.0)
	# Startup Gazebo
	p_r = Popen('. devel/setup.sh; roslaunch rubiks_description cube.launch 2>&1 > /dev/null', shell=True)
	time.sleep(3.0)
	dump_images()
	for _yaw, _yaw_iter in zip(np.linspace(0.0, 2.0 * np.pi, num=yaw_steps, endpoint=False), range(0, yaw_steps)):
		x, y, z, roll, pitch, yaw = pose(_pitch, _yaw)
		p = Popen('/usr/bin/gz model --model-name camera -x' + str(x) + ' -y ' + str(y) + ' -z ' + str(z) + ' -R ' + str(roll) + ' -P ' + str(pitch) + ' -Y ' + str(yaw), shell=True)
		p.wait()
		dump_images()
		p = Popen('rosbag record -j -O cube_pitch_' + str(_pitch_iter).zfill(3) + '_yaw_' + str(_yaw_iter).zfill(3) + '.bag -l 1 /camera/cam/image_raw > /dev/null', shell=True)
		p.wait()
		print('-------------------- pitch ' + str(_pitch_iter).zfill(3)  + ' yaw ' + str(_yaw_iter).zfill(3))
	p_r.terminate()
	killall()
	time.sleep(1.0) # Wait to kill everthing gracefully

# Convert bags to images
killall()
print('Done -- convert images')
p_r = Popen('. /opt/ros/kinetic/setup.sh; roscore', shell=True, bufsize=-1)
time.sleep(1.5)
p = Popen('rosparam set use_sim_time false', shell=True, bufsize=-1)
p.wait()
p_i = Popen('. /opt/ros/kinetic/setup.sh; rosrun image_view image_saver image:=/camera/cam/image_raw _filename_format:=%04d.png', shell=True, bufsize=-1)
time.sleep(3.0)
p_b = Popen('. /opt/ros/kinetic/setup.sh; for bag in $(ls *bag); do rosbag play --clock --topics /camera/cam/image_raw --wait-for-subscribers -r 5 ${bag}; done', shell=True, bufsize=-1)
p_b.wait()
time.sleep(3)
p_i.terminate()
p_r.terminate()
time.sleep(0.5)
killall()

print('Done -- create video')
p_b = Popen('ffmpeg -y -r 10 -i %04d.png mov.gif > /dev/null', shell=True, bufsize=-1)
p.wait()