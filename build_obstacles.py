#!/usr/bin/python

import numpy as np
import pandas as pd
import os

OBS_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "src", "rosbot_navigation", "src", "obs.csv")
OBS_TEMPLATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "src", "rosbot_gazebo", "obstacles", "obstacle_{}.urdf")
OBS_LAUNCH_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "src", "rosbot_gazebo", "launch", "obstacles.launch")
CYLINDER_TEMPLATE_URDF = """<robot name="obstacle_{}">
  <link name="obstacle_{}">
    <inertial>
      <origin xyz="{} {} 0" /> 
      <mass value="1000.0" />
      <inertia  ixx="1000.0" ixy="0.0"  ixz="0.0"  iyy="1000.0"  iyz="0.0"  izz="1000.0" />
    </inertial>
    <visual>
      <origin xyz="{} {} 0"/>
      <geometry>
        <cylinder radius="{}" length="0.1" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="{} {} 0"/>
      <geometry>
        <cylinder radius="{}" length="0.1" />
      </geometry>
    </collision>
  </link>
  <gazebo reference="obstacle_{}">
    <static>true</static>
    <material>Gazebo/Blue</material>
  </gazebo>
</robot>"""
OBST_URDF = """
    <node name="spawn_obj_{}" pkg="gazebo_ros" type="spawn_model" 
     args="-urdf -file $(find rosbot_gazebo)/obstacles/obstacle_{}.urdf -urdf -model obstacle_{}" 
     respawn="false" output="screen" />
"""


def calculate_radius(mu, alpha):
    """
    BUILD THIS FUNCTION TO RETURN A VALUE IN METERS
    """
    return float(mu + alpha) / 10.


def main():

    obs = pd.read_csv(OBS_FILE_PATH, header=None)
    launch = open(OBS_LAUNCH_PATH, "w+")
    launch.write("""<?xml version="1.0" encoding="UTF-8"?>\n<launch>""")

    for idx, o in enumerate(obs.values.astype(float).reshape((2, -1))):
        r = calculate_radius(o[2], o[3])

        if r <= 0:
            continue

        temp_obs = CYLINDER_TEMPLATE_URDF.format(idx+1, idx+1,
                                                 o[0], o[1],
                                                 o[0], o[1], r,
                                                 o[0], o[1], r,
                                                 idx+1)

        with open(OBS_TEMPLATE_PATH.format(idx + 1), "w+") as f:
            f.write(temp_obs)

        launch.write(OBST_URDF.format(idx + 1, idx + 1, idx + 1))

    launch.write("""</launch>""")
    launch.close()


if __name__ == "__main__":
    main()
