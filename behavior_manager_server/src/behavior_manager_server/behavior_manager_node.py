# -*- coding: utf-8 -*-

import copy
import sys

import actionlib
import rospy
import roslaunch

from behavior_manager_server.support_behavior_graph import SupportBehaviorGraph
from behavior_manager_server.support_behavior_graph import GraphEdge
from behavior_manager_server.support_behavior_graph import GraphNode
from behavior_manager_server.base_behavior import load_behavior_class
from behavior_manager_server.base_client import load_client_class

from std_msgs.msg import String
from behavior_manager_msgs.msg import ExecuteBehaviorsAction
from behavior_manager_msgs.msg import ExecuteBehaviorsResult
from behavior_manager_msgs.msg import ExecuteBehaviorsFeedback
from behavior_manager_msgs.srv import ResetCurrentNode
from behavior_manager_msgs.srv import ResetCurrentNodeResponse


class BehaviorManagerNode(object):

    def __init__(self):

        # ROS Parameters
        raw_edges = rospy.get_param('~map/edges')
        raw_nodes = rospy.get_param('~map/nodes')
        initial_node_id = rospy.get_param('~initial_node_id')
        client_class = rospy.get_param('~client_class')

        # check initial node is valid
        if initial_node_id not in raw_nodes:
            rospy.logerr(
                'Node id {} is not in node list.'.format(initial_node_id))
            sys.exit(1)

        # navigation dictonary
        self.edges = raw_edges
        self.nodes = raw_nodes
        self.graph = SupportBehaviorGraph(raw_edges, raw_nodes)
        self.current_node = GraphNode(
            initial_node_id, self.nodes[initial_node_id])
        self.pre_edge = GraphEdge()

        # Load client class
        self.client = load_client_class(client_class)

        # publisher
        self.pub_current_node_id = rospy.Publisher(
            '~current_node_id', String, queue_size=1)

        # Enable to launch roslaunch from this process
        roslaunch.pmon._init_signal_handlers()

        # ROS Service Server
        self.service_reset_current_node_id = rospy.Service(
            '~reset_current_node_id',
            ResetCurrentNode,
            self.handler_reset_current_node_id
        )

        # ROS Action Server
        self.server_execute_behaviors = actionlib.SimpleActionServer(
            '~execute_behaviors',
            ExecuteBehaviorsAction,
            execute_cb=self.handler_execute_behaviors,
            auto_start=False
        )
        self.server_execute_behaviors.start()

        rospy.loginfo('Initialized!')

    def run(self):

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            self.pub_current_node_id.publish(
                String(data=self.current_node.node_id))

    def handler_reset_current_node_id(self, req):

        node_id = req.current_node_id
        if node_id not in self.nodes:
            rospy.logerr('Node {} is not in node list.'.format(node_id))
        else:
            rospy.logdebug('Current Node is set to {}'.format(node_id))
            self.current_node = GraphNode(node_id, self.nodes[node_id])
            self.pre_edge = GraphEdge()
            return ResetCurrentNodeResponse(success=True)

    def handler_execute_behaviors(self, goal):

        current_graph = copy.deepcopy(self.graph)
        while True:
            # path calculation
            path = current_graph.calcPath(
                self.current_node.node_id, goal.target_node_id)
            if path is None:
                rospy.logerr('No path from {} to {}'.format(
                    self.current_node.node_id, goal.target_node_id))
                self.server_execute_behaviors.set_aborted(
                    ExecuteBehaviorsResult(success=False, message='No path found'))
                return
            else:
                success_navigation = True
                for edge in path:
                    rospy.loginfo('Navigating Edge {}'.format(edge))
                    try:
                        success = self.navigate_edge(
                            edge, self.pre_edge, self.client, self.graph)
                        if success:
                            rospy.loginfo('Edge {} succeeded.'.format(edge))
                            self.current_node = GraphEdge(
                                edge.node_id_to, self.nodes[edge.node_id_to])
                            self.server_execute_behaviors.publish_feedback(
                                ExecuteBehaviorsFeedback(current_node_id=self.current_node.node_id))
                            self.pre_edge = edge
                        else:
                            rospy.logwarn(
                                'Edge {} failed. Trying to find a new path.'.format(edge))
                            current_graph.remove_edge(
                                edge.node_id_from, edge.node_id_to)
                            # Check if there is a new way.
                            path = current_graph.calcPath(
                                self.current_node.node_id, goal.target_node_id)
                            if path is not None:
                                rospy.logerr('There is no path from {} to {} with edge failure'.format(
                                    self.current_node.node_id, goal.target_node_id))
                                self.server_execute_behaviors.set_aborted(ExecuteBehaviorsResult(
                                    success=False, message='There is no path from {} to {} with edge failure'.format(self.current_node.node_id, goal.target_node_id)))
                                return
                            else:
                                self.server_execute_behaviors.publish_feedback(
                                    ExecuteBehaviorsFeedback(current_node_id=self.current_node.node_id))
                                success_navigation = False
                                break
                    except Exception as e:
                        rospy.logerr(
                            'Got an error while navigating edge {}: {}'.format(edge, e))
                        self.server_execute_behaviors.set_aborted(ExecuteBehaviorsResult(
                            success=False, message='Got an error while navigating edge {}: {}'.format(edge, e)))
                        self.pre_edge = GraphEdge()
                        return
                if success_navigation:
                    break

        rospy.loginfo('Goal Reached!')
        self.server_execute_behaviors.set_succeeded(
            ExecuteBehaviorsResult(success=True, message='Goal Reached.'))
        return

    def navigate_edge(self, edge, pre_edge, client, graph):

        # load behavior class
        try:
            behavior_class = load_behavior_class(edge.behavior_type)
            behavior = behavior_class(client)
        except Exception as e:
            rospy.logerr('Failed to load behavior class: {}'.format(e))
            return False

        node_from = graph.nodes[edge.node_id_from]
        node_to = graph.nodes[edge.node_id_to]

        # Exception from behavior will be caught in handler
        success = behavior.run_initial(node_from, node_to, edge, self.pre_edge)
        if success is False:
            behavior.run_final(node_from, node_to, edge, self.pre_edge)
            return False

        success = behavior.run_main(node_from, node_to, edge, self.pre_edge)
        behavior.run_final(node_from, node_to, edge, self.pre_edge)
        return success
