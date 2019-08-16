#!/usr/bin/python3
from subprocess import Popen
import numpy as np
import time

# gz model --model-name camera -x 0.2 -y -0.0 -z 0.05 -R 0.0 -P 0.0 -Y $(echo  "3.14" | bc )
yaw_offset = np.pi
z_offset = 0.05 # helf hight of cube
pitch_steps = 45 # in 2 deg steps
yaw_steps = 180 # in 2 deg steps
pitch_steps = 9 # in 10 deg steps
yaw_steps = 36 # in 10 deg steps
for _pitch, _pitch_iter in zip(np.linspace(0.0, np.pi / 2.0, num=pitch_steps+1, endpoint=True), range(0, pitch_steps+1)):
	p_r = Popen('cd /opt/repositories/ml4pro_dev_miele/catkin_ws; . devel/setup.sh; roslaunch rubiks_description cube.launch', shell=True)
	time.sleep(5.0)
	p = Popen('rosparam set use_sim_time true', shell=True)
	p.wait()
	for _yaw, _yaw_iter in zip(np.linspace(0.0, 2.0 * np.pi, num=yaw_steps, endpoint=False), range(0, yaw_steps)):
		r = 0.2
		x = np.cos(_pitch) * np.cos(_yaw) * r
		y = np.cos(_pitch) * np.sin(_yaw) * r
		z = z_offset + np.sin(_pitch) * r
		roll = 0.0
		pitch = _pitch
		yaw = yaw_offset + _yaw
		#print(['/usr/bin/gz', 'model --model-name camera -x' + str(x) + ' -y ' + str(y) + ' -z ' + str(z) + ' -R ' + str(roll) + ' -P ' + str(pitch) + ' -Y ' + str(yaw) ])
		p = Popen('/usr/bin/gz model --model-name camera -x' + str(x) + ' -y ' + str(y) + ' -z ' + str(z) + ' -R ' + str(roll) + ' -P ' + str(pitch) + ' -Y ' + str(yaw), shell=True)
		p.wait()
		time.sleep(0.1)
		p = Popen('rosbag record -j -O cube_pitch_' + str(_pitch_iter).zfill(3) + '_yaw_' + str(_yaw_iter).zfill(3) + '.bag -l 1 /camera/cam/image_raw', shell=True)
		p.wait()
		print('pitch ' + str(_pitch_iter).zfill(3)  + ' yaw ' + str(_yaw_iter).zfill(3))
	time.sleep(0.5)
	p_r.terminate()
	p = Popen('killall gzserver', shell=True)
	p.wait()
	p = Popen('killall rosmaster', shell=True)
	p.wait()
	time.sleep(0.5)
p = Popen('killall rosmaster', shell=True, bufsize=-1)
p.wait()
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
p = Popen('killall rosmaster', shell=True, bufsize=-1)
p.wait()

print('Done -- create video')
p_b = Popen('ffmpeg -r 10 -i %04d.png mov.gif', shell=True, bufsize=-1)
p.wait()