#!/usr/bin/env python3
"""Spawn an optimized static pose and its EE torque marker in Gazebo."""

import math

import rospy
import yaml
from gazebo_msgs.srv import SetModelConfiguration, SpawnModel
from geometry_msgs.msg import Pose


def normalized(vector):
    length = math.sqrt(sum(value * value for value in vector))
    if length < 1.0e-12:
        raise ValueError("cannot normalize a zero vector")
    return [value / length for value in vector]


def axis_rpy(axis):
    axis = normalized(axis)
    return math.atan2(math.hypot(axis[0], axis[1]), axis[2]), math.atan2(axis[1], axis[0])


def torque_marker_sdf(name, wall_target, target, axis, torque):
    direction = 1.0 if torque >= 0.0 else -1.0
    axis = [direction * value for value in normalized(axis)]
    pitch, yaw = axis_rpy(axis)
    length = max(0.15, 0.02 * abs(torque))
    offset_vector = [target[i] - wall_target[i] for i in range(3)]
    offset_length = math.sqrt(sum(value * value for value in offset_vector))
    offset_link = ""
    if offset_length > 1.0e-12:
        offset_pitch, offset_yaw = axis_rpy(offset_vector)
        offset_midpoint = [0.5 * (wall_target[i] + target[i]) for i in range(3)]
        offset_link = """
    <link name="offset">
      <pose>{offset_x} {offset_y} {offset_z} 0 {offset_pitch} {offset_yaw}</pose>
      <visual name="offset_line">
        <geometry><cylinder><radius>0.005</radius><length>{offset_length}</length></cylinder></geometry>
        <material><ambient>1 0.55 0 1</ambient><diffuse>1 0.55 0 1</diffuse></material>
      </visual>
    </link>""".format(
            offset_x=offset_midpoint[0],
            offset_y=offset_midpoint[1],
            offset_z=offset_midpoint[2],
            offset_pitch=offset_pitch,
            offset_yaw=offset_yaw,
            offset_length=offset_length,
        )
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="target">
      <pose>{x} {y} {z} 0 0 0</pose>
      <visual name="target">
        <geometry><sphere><radius>0.018</radius></sphere></geometry>
        <material><ambient>0.2 1 0.35 1</ambient><diffuse>0.2 1 0.35 1</diffuse></material>
      </visual>
    </link>
    <link name="torque">
      <pose>{x} {y} {z} 0 {pitch} {yaw}</pose>
      <visual name="torque_shaft">
        <pose>0 0 {half_length} 0 0 0</pose>
        <geometry><cylinder><radius>0.009</radius><length>{length}</length></cylinder></geometry>
        <material><ambient>0 0 1 1</ambient><diffuse>0 0 1 1</diffuse></material>
      </visual>
      <visual name="torque_head">
        <pose>0 0 {length} 0 0 0</pose>
        <geometry><sphere><radius>0.028</radius></sphere></geometry>
        <material><ambient>0 0 1 1</ambient><diffuse>0 0 1 1</diffuse></material>
      </visual>
    </link>
    {offset_link}
  </model>
</sdf>
""".format(
        name=name,
        x=target[0],
        y=target[1],
        z=target[2],
        pitch=pitch,
        yaw=yaw,
        half_length=0.5 * length,
        length=length,
        offset_link=offset_link,
    )


def main():
    rospy.init_node("static_wrench_gazebo_spawner")
    result_file = rospy.get_param("~result")
    model_name = rospy.get_param("~model_name", "flying_humanoid")
    robot_description_name = rospy.resolve_name("robot_description")
    robot_description = rospy.get_param(robot_description_name)

    with open(result_file, "r", encoding="utf-8") as stream:
        result = yaml.safe_load(stream)
    configuration = result.get("configuration", [])
    joint_names = result.get("joint_names", [])
    joint_positions = result.get("joint_positions", [])
    wall_target = result.get("ee_wall_target_point_world", [])
    target = result.get("ee_target_point_world", [])
    axis = result.get("ee_target_axis_world", [])
    wrench = result.get("ee_wrench_arm_ee", [])
    if len(configuration) < 7 or len(joint_names) != len(joint_positions):
        raise ValueError("optimizer result contains an invalid robot configuration")
    if len(wall_target) != 3 or len(target) != 3 or len(axis) != 3 or len(wrench) != 6:
        raise ValueError("optimizer result contains invalid EE visualization data")

    initial_pose = Pose()
    initial_pose.position.x, initial_pose.position.y, initial_pose.position.z = configuration[:3]
    initial_pose.orientation.x = configuration[3]
    initial_pose.orientation.y = configuration[4]
    initial_pose.orientation.z = configuration[5]
    initial_pose.orientation.w = configuration[6]

    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    spawn_urdf = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    response = spawn_urdf(model_name, robot_description, rospy.get_namespace(), initial_pose, "world")
    if not response.success:
        raise RuntimeError("failed to spawn robot: " + response.status_message)

    rospy.wait_for_service("/gazebo/set_model_configuration")
    set_configuration = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
    response = set_configuration(model_name, robot_description_name, joint_names, joint_positions)
    if not response.success:
        raise RuntimeError("failed to set optimized joints: " + response.status_message)

    marker_name = model_name + "_static_wrench_marker"
    marker_sdf = torque_marker_sdf(marker_name, wall_target, target, axis, wrench[5])
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    response = spawn_sdf(marker_name, marker_sdf, rospy.get_namespace(), Pose(), "world")
    if not response.success:
        raise RuntimeError("failed to spawn EE torque marker: " + response.status_message)
    rospy.loginfo("spawned %s and %.3f Nm EE torque marker", model_name, wrench[5])


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError, yaml.YAMLError, rospy.ROSException) as error:
        rospy.logfatal("failed to spawn static-wrench Gazebo visualization: %s", error)
        raise
