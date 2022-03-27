#!/usr/bin/env python

import rospy
import roslib
import unittest
import actionlib

from behavior_manager_msgs.msg import ExecuteBehaviorsAction
from behavior_manager_msgs.msg import ExecuteBehaviorsGoal

PKG = 'behavior_manager_server'
roslib.load_manifest(PKG)


class TestBehaviorManager(unittest.TestCase):

    def test_behavior_manager_client(self):
        target_node_id = rospy.get_param('~target_node_id')
        client = actionlib.SimpleActionClient('~execute_behaviors', ExecuteBehaviorsAction)
        client.wait_for_server()
        goal = ExecuteBehaviorsGoal()
        goal.target_node_id = target_node_id
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        self.assertTrue(result.success)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_behavior_manager_client')
    rostest.rosrun(PKG, 'test_behavior_manager', TestBehaviorManager)
