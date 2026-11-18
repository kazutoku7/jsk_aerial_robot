#!/usr/bin/env python3
"""Visualize a static-wrench result, its wall, target, and output torque."""

import math
import yaml

import rospy
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


def load_result(filename):
    with open(filename, "r", encoding="utf-8") as stream:
        result = yaml.safe_load(stream)

    configuration = result.get("configuration", [])
    names = result.get("joint_names", [])
    positions = result.get("joint_positions", [])
    if len(configuration) < 7:
        raise ValueError("result configuration must contain a free-flyer pose")
    if len(names) != len(positions):
        raise ValueError("joint_names and joint_positions have different lengths")
    for key in (
        "wall_point_world",
        "wall_normal_world",
        "ee_wall_target_point_world",
        "ee_target_point_world",
        "ee_target_axis_world",
        "ee_position_world",
        "ee_axis_world",
        "ee_wrench_arm_ee",
    ):
        if len(result.get(key, [])) != (6 if key == "ee_wrench_arm_ee" else 3):
            raise ValueError("result field '%s' is missing or malformed" % key)
    return result, configuration, names, positions


def prefixed(prefix, frame):
    return "/".join(value.strip("/") for value in (prefix, frame) if value.strip("/"))


def normalized(vector):
    length = math.sqrt(sum(value * value for value in vector))
    if length < 1.0e-12:
        raise ValueError("cannot normalize a zero vector")
    return [value / length for value in vector]


def cross(lhs, rhs):
    return [
        lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[2] * rhs[0] - lhs[0] * rhs[2],
        lhs[0] * rhs[1] - lhs[1] * rhs[0],
    ]


def point(vector):
    return Point(x=vector[0], y=vector[1], z=vector[2])


def marker(marker_id, marker_type, frame, namespace="static_wrench"):
    result = Marker()
    result.header.frame_id = frame
    result.header.stamp = rospy.Time.now()
    result.ns = namespace
    result.id = marker_id
    result.type = marker_type
    result.action = Marker.ADD
    result.pose.orientation.w = 1.0
    return result


def visualization_markers(result, world_frame):
    wall_normal = normalized(result["wall_normal_world"])
    wall_target = result["ee_wall_target_point_world"]
    target = result["ee_target_point_world"]
    torque_axis = normalized(result["ee_target_axis_world"])
    torque = result["ee_wrench_arm_ee"][5]
    wall_width = rospy.get_param("~wall_width", 1.2)
    wall_height = rospy.get_param("~wall_height", 1.5)
    wall_thickness = rospy.get_param("~wall_thickness", 0.05)
    wall_hatch_spacing = rospy.get_param("~wall_hatch_spacing", 0.10)
    torque_scale = rospy.get_param("~torque_arrow_scale", 0.02)
    contact_force_scale = rospy.get_param("~contact_force_arrow_scale", 0.01)
    if wall_thickness <= 0.0:
        raise ValueError("wall_thickness must be positive")
    if wall_hatch_spacing <= 0.0:
        raise ValueError("wall_hatch_spacing must be positive")
    if contact_force_scale <= 0.0:
        raise ValueError("contact_force_arrow_scale must be positive")

    # This task uses a vertical wall. Project world +z into the wall to obtain
    # its vertical direction, with a generic fallback for a horizontal plane.
    world_up = [0.0, 0.0, 1.0]
    normal_up = sum(lhs * rhs for lhs, rhs in zip(wall_normal, world_up))
    vertical = [world_up[i] - normal_up * wall_normal[i] for i in range(3)]
    if math.sqrt(sum(value * value for value in vertical)) < 1.0e-9:
        vertical = [0.0, 1.0, 0.0]
    vertical = normalized(vertical)
    horizontal = normalized(cross(vertical, wall_normal))

    center = list(wall_target)
    # For the default floor z=0, draw the wall from z=0 upward while keeping it
    # horizontally centered at the valve target.
    center[2] = 0.5 * wall_height
    corners = []
    for horizontal_sign, vertical_sign in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        corners.append(
            [
                center[i]
                + horizontal_sign * 0.5 * wall_width * horizontal[i]
                + vertical_sign * 0.5 * wall_height * vertical[i]
                for i in range(3)
            ]
        )

    # The front face stays on x=0. The volume extends only opposite to the
    # robot, i.e. from x=0 toward -x for the default +x wall normal.
    wall = marker(0, Marker.CUBE, world_frame)
    wall.pose.position = point(
        [center[i] - 0.5 * wall_thickness * wall_normal[i] for i in range(3)]
    )
    wall.scale.x = wall_thickness
    wall.scale.y = wall_width
    wall.scale.z = wall_height
    wall.color.r = 0.10
    wall.color.g = 0.42
    wall.color.b = 0.58
    wall.color.a = 0.28

    # Draw horizontal lines (perpendicular to world z) at regular height
    # intervals. Lift them slightly toward the robot to avoid z-fighting.
    hatching = marker(6, Marker.LINE_LIST, world_frame)
    hatching.scale.x = 0.012
    hatching.color.r = 0.24
    hatching.color.g = 0.62
    hatching.color.b = 0.70
    hatching.color.a = 0.70
    half_width = 0.5 * wall_width
    half_height = 0.5 * wall_height
    hatch_lift = 0.002
    vertical_coordinate = -half_height
    while vertical_coordinate <= half_height + 1.0e-12:
        for horizontal_coordinate in (-half_width, half_width):
            hatching.points.append(
                point(
                    [
                        center[i]
                        + horizontal_coordinate * horizontal[i]
                        + vertical_coordinate * vertical[i]
                        + hatch_lift * wall_normal[i]
                        for i in range(3)
                    ]
                )
            )
        vertical_coordinate += wall_hatch_spacing

    outline = marker(1, Marker.LINE_STRIP, world_frame)
    outline.points = [point(value) for value in corners + [corners[0]]]
    outline.scale.x = 0.008
    outline.color.r = 0.20
    outline.color.g = 0.55
    outline.color.b = 0.65
    outline.color.a = 0.85

    target_marker = marker(2, Marker.SPHERE, world_frame)
    target_marker.pose.position = point(target)
    target_marker.scale.x = target_marker.scale.y = target_marker.scale.z = 0.035
    target_marker.color.r = 0.2
    target_marker.color.g = 1.0
    target_marker.color.b = 0.35
    target_marker.color.a = 1.0

    offset_line = marker(5, Marker.LINE_LIST, world_frame)
    offset_line.points = [point(wall_target), point(target)]
    offset_line.scale.x = 0.01
    offset_line.color.r = 1.0
    offset_line.color.g = 0.55
    offset_line.color.b = 0.0
    offset_line.color.a = 1.0

    arrow_length = max(0.15, abs(torque) * torque_scale)
    direction = 1.0 if torque >= 0.0 else -1.0
    arrow_end = [target[i] + direction * arrow_length * torque_axis[i] for i in range(3)]
    torque_arrow = marker(3, Marker.ARROW, world_frame)
    torque_arrow.points = [point(target), point(arrow_end)]
    torque_arrow.scale.x = 0.018
    torque_arrow.scale.y = 0.04
    torque_arrow.scale.z = 0.055
    torque_arrow.color.r = 0.0
    torque_arrow.color.g = 0.0
    torque_arrow.color.b = 1.0
    torque_arrow.color.a = 1.0

    contact_force_arrows = []
    for contact_index in (1, 2):
        contact_point = result.get("contact_point_%d_world" % contact_index, [])
        contact_wrench = result.get("contact_wrench_%d_world" % contact_index, [])
        # Older archived results do not contain contact-point positions. They
        # remain viewable, but contact-force arrows are omitted for them.
        if len(contact_point) != 3 or len(contact_wrench) != 6:
            continue
        force_end = [
            contact_point[i] + contact_force_scale * contact_wrench[i] for i in range(3)
        ]
        force_arrow = marker(6 + contact_index, Marker.ARROW, world_frame)
        force_arrow.points = [point(contact_point), point(force_end)]
        force_arrow.scale.x = 0.014
        force_arrow.scale.y = 0.032
        force_arrow.scale.z = 0.045
        force_arrow.color.r = 0.90
        force_arrow.color.g = 0.12
        force_arrow.color.b = 0.08
        force_arrow.color.a = 1.0
        contact_force_arrows.append(force_arrow)

    label = marker(4, Marker.TEXT_VIEW_FACING, world_frame)
    label.pose.position = point([arrow_end[0], arrow_end[1], arrow_end[2] + 0.055])
    label.scale.z = 0.045
    label.color.r = 1.0
    label.color.g = 0.85
    label.color.b = 1.0
    label.color.a = 1.0
    label.text = "EE torque: %.2f Nm, offset: %.3f m" % (
        torque,
        result["ee_offset_along_wall_normal"],
    )
    return MarkerArray(
        markers=[wall, hatching, outline, target_marker, offset_line, torque_arrow, label]
        + contact_force_arrows
    )


def main():
    rospy.init_node("static_wrench_result_visualizer")
    filename = rospy.get_param("~result")
    world_frame = rospy.get_param("~world_frame", "world")
    root_frame = rospy.get_param("~root_frame", "root")
    tf_prefix = rospy.get_param("~tf_prefix", "flying_humanoid")
    rate = rospy.Rate(rospy.get_param("~rate", 20.0))

    result, configuration, names, positions = load_result(filename)
    joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1, latch=True)
    marker_pub = rospy.Publisher("visualization_marker", MarkerArray, queue_size=1, latch=True)
    broadcaster = tf2_ros.TransformBroadcaster()
    rospy.loginfo("visualizing static-wrench result from %s", filename)
    marker_pub.publish(visualization_markers(result, world_frame))

    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = names
        joints.position = positions
        joint_pub.publish(joints)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = world_frame
        transform.child_frame_id = prefixed(tf_prefix, root_frame)
        transform.transform.translation.x = configuration[0]
        transform.transform.translation.y = configuration[1]
        transform.transform.translation.z = configuration[2]
        transform.transform.rotation.x = configuration[3]
        transform.transform.rotation.y = configuration[4]
        transform.transform.rotation.z = configuration[5]
        transform.transform.rotation.w = configuration[6]
        broadcaster.sendTransform(transform)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, yaml.YAMLError) as error:
        rospy.logfatal("failed to visualize static-wrench result: %s", error)
