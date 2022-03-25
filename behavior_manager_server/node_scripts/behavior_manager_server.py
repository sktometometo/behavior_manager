#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from behavior_manager_server.behavior_manager_node import BehaviorManagerNode

def main():

    rospy.init_node('behavior_manager_node')
    node = BehaviorManagerNode()
    node.run()

if __name__ == '__main__':
    main()
