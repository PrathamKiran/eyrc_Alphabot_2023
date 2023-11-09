#!/usr/bin/env python3

from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from os import path
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5


def main():
    rclpy.init()

    # Create node for this example
    node = Node("move_arm")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # # Get parameters
    # position = node.get_parameter("position").get_parameter_value().double_array_value
    # quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    # cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Defining the points to be covered in a sequence
    # p1 = (0.35, 0.1, 0.68)
    # p2 = (0.194, -0.43, 0.701)
    # d = (-0.37, 0.12, 0.397)
    pointx = (0, 0.7071068, 0, 0.7071068)
    pointx_ = (0, -0.7071068, 0, 0.7071068)
    pointy_ = (0.7071068, 0, 0, 0.7071068)
    positions = [(0.35, 0.1, 0.68),(-0.37, 0.12, 0.397),(0.194, -0.43, 0.701),(-0.37, 0.12, 0.397)]
    # positions = [(0.35, 0.0, 0.0),(0.0, 0.35, 0.0),(0.0, 0.0, 0.35)]
    quat = [pointx , pointx_, pointy_, pointx_]
    cartesian = False

    # Add Collision objects
    # Define path to rack stl
    box_path = path.join(path.dirname(path.realpath(__file__)), "assets", "box.stl")
    # Add objects
    moveit2.add_collision_mesh(filepath=box_path, id="box1", position=(0.51, 0.00, 0.51), quat_xyzw=(0.0, 0.0, 0.707, 0.707), frame_id=ur5.base_link_name())
    moveit2.add_collision_mesh(filepath=box_path, id="box2", position=(0.24 , -0.55 , 0.51), quat_xyzw=(0.0, 0.0, 0.7071068, 0.7071068 ), frame_id=ur5.base_link_name())
    moveit2.add_collision_mesh(filepath=box_path, id="box3", position=(0.25, -0.55, 0.19), quat_xyzw=(0.0, 0.0, 0.7071068, 0.7071068 ), frame_id=ur5.base_link_name())


    for i in range(len(positions)):
        # positionT = (position[2],position[1],position[0])
        node.get_logger().info(
            f"Moving to {{position: {list(positions[i])}, quat_xyzw: {list(quat[i])}}}"
        )
        moveit2.move_to_pose(position=positions[i], quat_xyzw=quat[i], cartesian=cartesian)
        moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
