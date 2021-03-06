# -*- coding: utf-8 -*-

import rospy


def load_behavior_class(class_string):
    package_name = class_string.split('.')[0]
    module_name = class_string.split('.')[1]
    class_name = class_string.split('.')[2]
    behavior_class = getattr(
        getattr(__import__(package_name), module_name), class_name)
    return behavior_class


class BaseBehavior(object):

    def __init__(self, client):

        self.client = client

    def run_initial(self, start_node, end_node, edge, pre_edge):
        pass

    def run_main(self, start_node, end_node, edge, pre_edge):
        pass

    def run_final(self, start_node, end_node, edge, pre_edge):
        pass


class SimpleBehavior(BaseBehavior):

    def run_initial(self, start_node, end_node, edge, pre_edge):

        rospy.loginfo('__run_initial() called')
        return True

    def run_main(self, start_node, end_node, edge, pre_edge):

        rospy.loginfo('__run_main() called')
        rospy.loginfo('start_node: {}'.format(start_node))
        rospy.loginfo('end_node: {}'.format(end_node))
        rospy.loginfo('edge: {}'.format(edge))
        rospy.loginfo('pre_edge: {}'.format(pre_edge))
        if 'success' in edge.args:
            return edge.args['success']
        else:
            rospy.logerr('No \'success\' field in args of edge.')
            return False

    def run_final(self, start_node, end_node, edge, pre_edge):

        rospy.loginfo('__run_finalize() called')
