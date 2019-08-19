#!/bin/bash

P="000-add_gui_args.diff"
F="../catkin_ws/src/panda_simulation/launch/"
cp $P $F
cd $F
git apply $P
cd -

P="001-fix-robot_control_node-to-publish-in-desired-frame.diff"
F="../catkin_ws/src/panda_simulation/src/"
cp $P $F
cd $F
git apply $P
cd -


# Create objects
cp -r ./panda_simulation ~/.panda_simulation

