#!/usr/bin/env python3

import rospy
import actionlib
from square.msg import DrawSquareAction, DrawSquareGoal, DrawSquareResult

def draw_square_client() -> DrawSquareResult:
    client = actionlib.SimpleActionClient('draw_square', DrawSquareAction)
    client.wait_for_server()
    
    goal: DrawSquareGoal = DrawSquareGoal()
    goal.side_length = 1.0  # 1 meter
    goal.speed = 0.5        # 0.5 m/s
    
    client.send_goal(goal)
    
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('draw_square_client')
        result: DrawSquareResult = draw_square_client()
        rospy.loginfo("Action completed: %s", result.success)
    except rospy.ROSInterruptException:
        pass
