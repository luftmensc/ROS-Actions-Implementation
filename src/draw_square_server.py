#!/usr/bin/env python3

import rospy
import actionlib
from square.msg import DrawSquareAction, DrawSquareResult, DrawSquareFeedback
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math
from typing import Optional

class DrawSquareActionServer:
    def __init__(self) -> None:
        self.server = actionlib.SimpleActionServer('draw_square', DrawSquareAction, self.execute, False)
        self.server.start()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.current_pose: Optional[Pose] = None
        self.start_pose: Optional[Pose] = None

    def odom_callback(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose

    def execute(self, goal: DrawSquareAction) -> None:
        success: bool = True
        feedback: DrawSquareFeedback = DrawSquareFeedback()
        result: DrawSquareResult = DrawSquareResult()
        
        # Wait for the first odometry message to set start_pose
        while self.current_pose is None:
            rospy.sleep(0.1)
        
        self.start_pose = self.current_pose
        
        # Draw the square
        for _ in range(4):
            if not self.go_straight(goal.side_length, goal.speed):
                success = False
                break
            if not self.turn_90_degrees():
                success = False
                break
        
        result.success = success
        self.server.set_succeeded(result)

    def go_straight(self, distance: float, speed: float) -> bool:
        twist: Twist = Twist()
        twist.linear.x = speed
        start_position = self.current_pose.position
        
        while not rospy.is_shutdown():
            current_position = self.current_pose.position
            traveled_distance = self.calculate_distance(start_position, current_position)
            
            feedback = DrawSquareFeedback()
            feedback.current_distance_traveled = traveled_distance
            self.server.publish_feedback(feedback)
            
            if traveled_distance >= distance:
                break
            
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)
        return True

    def turn_90_degrees(self) -> bool:
        twist: Twist = Twist()
        twist.angular.z = math.radians(45)  # 45 degrees/sec, takes 2 seconds for 90 degrees
        turn_time: float = 2
        
        for _ in range(int(turn_time * 10)):
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)
        return True

    def calculate_distance(self, start: Pose, current: Pose) -> float:
        return math.sqrt(
            (current.x - start.x) ** 2 +
            (current.y - start.y) ** 2
        )

if __name__ == '__main__':
    rospy.init_node('draw_square_server')
    server = DrawSquareActionServer()
    rospy.spin()
