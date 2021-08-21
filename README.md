# teb_local_planner_tutorials
This package contains supplementary material and examples for [teb_local_planner](http://wiki.ros.org/teb_local_planner) tutorials.

The tutorials package mainly contains fully working robot navigation examples in combination with the teb_local_planner.
Currently it provides a differential drive and a carlike robot simulation setup.
In order to depend on as few dependencies as possible, the simulations are performed with [stage_ros](http://wiki.ros.org/stage_ros)
and without any URDF models. However, they are easily extendable and integrable (e.g. Gazebo, URDF models, voxel costmaps, robot hardware nodes, ...).

Refer to the [teb_local_planner](http://wiki.ros.org/teb_local_planner) ROS wiki page for more information.

**Dependencies:**

 * *navigation stack* and *teb_local_planner* package
 * *stage*: `sudo apt-get install ros-kinetic-stage-ros`

**How to run chy simulation code:**

1. `roslaunch teb_local_planner_tutorials robot_omnidir_empty_box_in_stage.launch`

2. `rosrun teb_local_planner_tutorials send_goal.py`

3. `rosrun teb_local_planner_tutorials traj_pred.py`

4. `rosrun teb_local_planner_tutorials MPDM.py`

5. `rosrun teb_local_planner_tutorials sim_summarize.py`

6. `rosrun teb_local_planner_tutorials publish_human_obstacle.py`

**How to config chy simulation environment:**

* change parameters in `teb_local_planner_tutorials/cfg/trial_cfg.yaml`
* In `teb_local_planner_tutorials/launch/robot_omnidir_empty_box_in_stage.launch` change line 40 for localization map and change line 46-48 for localization initialization
    * Note: robot heading in rad
* In `teb_local_planner_tutorials/stage/empty_omnidir.world` change line 34,35 for environment map, change line 42,43 for robot initial location
    * Note: robot heading in degree 