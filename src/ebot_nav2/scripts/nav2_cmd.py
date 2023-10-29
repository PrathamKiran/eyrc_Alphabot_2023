#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import action_msgs.msg
import action_msgs

class NavigationClient(Node):

    def __init__(self):
        super().__init__('navigation_client')
        self.goal_handle = None

        self.navigation_client = self.create_client(NavigateToPose, 'navigate_to_pose')
        while not self.navigation_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Navigation service not available, waiting...')

    def send_goal(self, goal):
        self.get_logger().info('Sending goal...')
        self.goal_handle = self.navigation_client.send_goal_async(goal, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback received: {feedback_msg}')
        if self.goal_handle.is_active():
            if feedback_msg.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigation goal achieved!')
                self.goal_handle.set_succeeded()
            elif feedback_msg.status == GoalStatus.STATUS_ABORTED:
                self.get_logger().info('Navigation goal aborted!')
                self.goal_handle.set_aborted()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationClient()

    # Defining goal poses
    goal_p1 = PoseStamped()
    # Defining p1 coordinates
    goal_p1.header.frame_id = "map"
    goal_p1.header.stamp = node.get_clock().now().to_msg()
    goal_p1.pose.position.x =1.8
    goal_p1.pose.position.y =1.5
    goal_p1.pose.position.z =0.0 
    goal_p1.pose.orientation.x = 1.57
    goal_p1.pose.orientation.y = 0.0
    goal_p1.pose.orientation.z = 0.0
    goal_p1.pose.orientation.w = 1.0
    

    goal_p2 = PoseStamped()
    # Defining p2 coordinates
    goal_p2.header.frame_id = "map"
    goal_p2.header.stamp = node.get_clock().now().to_msg()
    goal_p2.pose.position.x =2.0
    goal_p2.pose.position.y =-7.0
    goal_p2.pose.position.z =0.0 
    goal_p2.pose.orientation.x = -1.57
    goal_p2.pose.orientation.y = 0.0
    goal_p2.pose.orientation.z = 0.0
    goal_p2.pose.orientation.w = 1.0
    
    
    goal_p3 = PoseStamped()
    # Defining p3 coordinates
    goal_p3.header.frame_id = "map"
    goal_p3.header.stamp = node.get_clock().now().to_msg()
    goal_p3.pose.position.x =-3.0
    goal_p3.pose.position.y =2.5
    goal_p3.pose.position.z =0.0 
    goal_p3.pose.orientation.x = 1.57
    goal_p3.pose.orientation.y = 0.0
    goal_p3.pose.orientation.z = 0.0
    goal_p3.pose.orientation.w = 1.0

    # Sending the goals sequentially
    node.send_goal(NavigateToPose.Goal(target_pose=goal_p1))
    rclpy.spin_until_future_complete(node, node.goal_handle)
    node.send_goal(NavigateToPose.Goal(target_pose=goal_p2))
    rclpy.spin_until_future_complete(node, node.goal_handle)
    node.send_goal(NavigateToPose.Goal(target_pose=goal_p3))
    rclpy.spin_until_future_complete(node, node.goal_handle)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
