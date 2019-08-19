#!/bin/bash


cp 000-add_gui_args.diff ../catkin_ws/src/panda_simulation/launch/
cd ../catkin_ws/src/panda_simulation/launch/
git apply 000-add_gui_args.diff
cd -


