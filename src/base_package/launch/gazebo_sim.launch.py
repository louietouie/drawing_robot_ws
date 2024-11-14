# this will launch gazebo, without having to individually run these 3 commands...
# 1. ros2 launch base_package rsp.launch.py use_sim_time:=true (this starts the robot state publisher)
# 2. ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf (launch file is provided by the gazebo package. this command starts gazebo)
# 3. ros2 run gazebo_ros spawn_entity.py -topic robot_description (spawn script is provided by gazebo package to spawn our robot. The URDF is being published by robot state publisher on a topic called robot_description)
# 3.5. the above spawn command might be outdated, it might actually be ros2 launch ros_gz_sim gz_spawn_model.launch.py. But it doesn't really matter, because these commands can all be consolidated into one launch file, that references other launch files

