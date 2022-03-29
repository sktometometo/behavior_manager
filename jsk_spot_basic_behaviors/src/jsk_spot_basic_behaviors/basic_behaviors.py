# -*- coding: utf-8 -*-

import actionlib
import math
import roslaunch
import rospkg
import rospy

from behavior_manager_server.base_behavior import BaseBehavior

from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2
from spinal.msg import Barometer
from spinal.msg import Imu
from std_msgs.msg import Bool
from switchbot_ros.msg import SwitchBotCommandAction
from switchbot_ros.msg import SwitchBotCommandGoal


class WalkBehavior(BaseBehavior):

    def run_initial(self, start_node, end_node, edge, pre_edge):

        rospy.logdebug('run_initial() called')
        self.silent_mode = rospy.get_param('~silent_mode', True)
        self.sound_client = self.client.sound_client
        self.spot_client = self.client.spot_ros_client
        return True

    def run_main(self, start_node, end_node, edge, pre_edge):

        rospy.logdebug('run_main() called')

        graph_name = edge.args['graph']
        start_id = filter(
            lambda x: x['graph'] == graph_name,
            start_node.properties['waypoints_on_graph']
        )[0]['id']
        end_id = filter(
            lambda x: x['graph'] == graph_name,
            end_node.properties['waypoints_on_graph']
        )[0]['id']
        localization_method = filter(
            lambda x: x['graph'] == graph_name,
            start_node.properties['waypoints_on_graph']
        )[0]['localization_method']

        # graph uploading and localization
        if pre_edge is not None and \
                graph_name == pre_edge.args['graph']:
            rospy.loginfo('graph upload and localization skipped.')
        else:
            # Upload
            ret = self.spot_client.upload_graph(graph_name)
            if ret[0]:
                rospy.loginfo('graph {} uploaded.'.format(graph_name))
            else:
                rospy.logerr('graph uploading failed: {}'.format(ret[1]))
                return False
            # Localization
            if localization_method == 'fiducial':
                ret = self.spot_client.set_localization_fiducial()
            elif localization_method == 'waypoint':
                ret = self.spot_client.set_localization_waypoint(start_id)
            else:
                ret = (False, 'Unknown localization method')
            if ret[0]:
                rospy.loginfo('robot is localized on the graph.')
            else:
                rospy.logwarn('Localization failed: {}'.format(ret[1]))
                return False

        # start navigation
        success = False
        rate = rospy.Rate(10)
        if not self.silent_mode:
            self.sound_client.say('移動します', blocking=True)
        self.spot_client.navigate_to(end_id, blocking=False)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.spot_client.wait_for_navigate_to_result(rospy.Duration(0.1)):
                result = self.spot_client.get_navigate_to_result()
                success = result.success
                rospy.loginfo('result: {}'.format(result))
                break

        # recovery on failure
        if not success:
            if not self.silent_mode:
                self.sound_client.say('失敗したので元に戻ります', blocking=True)
            self.spot_client.navigate_to(start_id, blocking=True)
            self.spot_client.wait_for_navigate_to_result()

        return success


class ElevatorBehavior(BaseBehavior):

    def door_point_cloud_callback(self, msg):
        if len(msg.data) == 0:
            self.door_is_open = True
        else:
            self.door_is_open = False

    def barometer_callback(self, msg):

        rospy.logdebug('altitude: {}'.format(msg.altitude))
        if math.fabs(msg.altitude - self.target_altitude) < self.threshold_altitude:
            self.is_target_floor = True
        else:
            self.is_target_floor = False

    def imu_callback(self, msg):

        rospy.logdebug('acc z: {}'.format(msg.acc_data[2]))
        if math.fabs(msg.acc_data[2] - self.stable_acc_z) < self.threshold_acc:
            self.elevator_stop_acc = True
        else:
            self.elevator_stop_acc = False

    def run_initial(self, start_node, end_node, edge, pre_edge):

        rospy.logdebug('run_initial() called')

        self.silent_mode = rospy.get_param('~silent_mode', True)
        self.sound_client = self.client.sound_client
        self.spot_client = self.client.spot_ros_client

        # launch recognition launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        roslaunch_path = rospkg.RosPack().get_path('spot_basic_behaviors') +\
            '/launch/elevator_detection.launch'
        roslaunch_cli_args = [roslaunch_path]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
            roslaunch_cli_args)
        self.roslaunch_parent = roslaunch.parent.ROSLaunchParent(
            uuid,
            roslaunch_file
        )
        self.roslaunch_parent.start()

        # value for door openring checker
        self.door_is_open = False
        self.subscriber_door_check = None

        # value for floor detection
        self.threshold_altitude = rospy.get_param(
            '/elevator_behavior/threshold_altitude', 2)
        self.is_target_floor = False
        self.target_altitude = 0
        self.subscriber_floor_detection = None

        #
        self.threshold_acc = rospy.get_param(
            '/elevator_behavior/threshold_acc', 0.2)
        self.stable_acc_z = 0
        self.elevator_stop_acc = False
        self.subscriber_imu = None

        # value for switchbot
        self.action_client_switchbot = actionlib.SimpleActionClient(
            '/switchbot_ros/switch',
            SwitchBotCommandAction
        )

        return True

    def run_main(self, start_node, end_node, edge, pre_edge):

        rospy.logdebug('run_main() called')

        graph_name = edge.args['graph']
        start_id = filter(
            lambda x: x['graph'] == graph_name,
            start_node.properties['waypoints_on_graph']
        )[0]['id']
        end_id = edge.args['goal_waypoint_id']
        rest_waypoint_id = edge.args['rest_waypoint_id']
        localization_method = filter(
            lambda x: x['graph'] == graph_name,
            start_node.properties['waypoints_on_graph']
        )[0]['localization_method']

        start_altitude = start_node.properties['floor_height']
        end_altitude = end_node.properties['floor_height']
        end_floor = end_node.properties['floor']

        # graph uploading and localization
        if pre_edge is not None and \
                graph_name == pre_edge.args['graph']:
            rospy.loginfo('graph upload and localization skipped.')
        else:
            # Upload
            ret = self.spot_client.upload_graph(graph_name)
            if ret[0]:
                rospy.loginfo('graph {} uploaded.'.format(graph_name))
            else:
                rospy.logerr('graph uploading failed: {}'.format(ret[1]))
                return False
            # Localization
            if localization_method == 'fiducial':
                ret = self.spot_client.set_localization_fiducial()
            elif localization_method == 'waypoint':
                ret = self.spot_client.set_localization_waypoint(start_id)
            else:
                ret = (False, 'Unknown localization method')
            if ret[0]:
                rospy.loginfo('robot is localized on the graph.')
            else:
                rospy.logwarn('Localization failed: {}'.format(ret[1]))
                return False

        # start floor detection
        try:
            current_altitude = rospy.wait_for_message(
                '/spinal/baro', Barometer, rospy.Duration(5)).altitude
            self.target_altitude = current_altitude - \
                (start_altitude - end_altitude)
            self.subscriber_floor_detection = rospy.Subscriber(
                '/spinal/baro',
                Barometer,
                self.barometer_callback)
        except Exception as e:
            rospy.logerr('{}'.format(e))
            return False

        # start imu
        try:
            self.stable_acc_z = rospy.wait_for_message(
                '/spinal/imu', Imu, rospy.Duration(5)).acc_data[2]
            self.subscriber_imu = rospy.Subscriber(
                '/spinal/imu',
                Imu,
                self.imu_callback)
        except Exception as e:
            rospy.logerr('{}'.format(e))
            return False

        # start door opening check from outside
        self.subscriber_door_check = rospy.Subscriber(
            '/spot_recognition/elevator_door_points',
            PointCloud2,
            self.door_point_cloud_callback)

        # push button with switchbot
        rospy.loginfo('calling elevator when riding...')
        if not self.action_client_switchbot.wait_for_server(rospy.Duration(10)):
            rospy.logerr('switchbot server seems to fail.')
            return False
        else:
            switchbot_goal = SwitchBotCommandGoal()
            switchbot_goal.device_name = start_node.properties['switchbot_device']
            switchbot_goal.command = 'press'
            count = 0
            while True:
                self.action_client_switchbot.send_goal(switchbot_goal)
                if self.action_client_switchbot.wait_for_result(rospy.Duration(10)):
                    break
                count += 1
            if count >= 3:
                rospy.logerr('switchbot calling failed.')
                return False
            result = self.action_client_switchbot.get_result()
            rospy.loginfo('switchbot result: {}'.format(result))
            if not result.done:
                rospy.logerr('switchbot calling failed.')
                return False
        rospy.loginfo('elevator calling when riding on has succeeded')

        # wait for elevator
        rate = rospy.Rate(1)
        door_open_count = 0
        while not rospy.is_shutdown():
            rate.sleep()
            if self.door_is_open:
                door_open_count += 1
            else:
                door_open_count = 0
            if door_open_count >= 2:
                break
        rospy.loginfo('elevator door opened.')
        self.subscriber_door_check.unregister()
        self.subscriber_door_check = None

        # start navigation to rest point
        rate = rospy.Rate(10)
        if not self.silent_mode:
            self.sound_client.say('エレベータに乗り込みます。ご注意ください。', blocking=False)
        self.spot_client.navigate_to(rest_waypoint_id, blocking=False)
        # call elevator from destination floor while
        rospy.loginfo('calling elevator when getting off...')
        switchbot_goal = SwitchBotCommandGoal()
        switchbot_goal.device_name = end_node.properties['switchbot_device']
        switchbot_goal.command = 'press'
        self.action_client_switchbot.send_goal(switchbot_goal)
        ##
        if not self.action_client_switchbot.wait_for_result(timeout=rospy.Duration(20)):
            rospy.logerr('Switchbot timeout')
            self.spot_client.wait_for_navigate_to_result()
            self.spot_client.navigate_to(start_id, blocking=True)
            self.spot_client.wait_for_navigate_to_result()
            return False
        result_switchbot = self.action_client_switchbot.get_result()
        self.spot_client.wait_for_navigate_to_result()
        result_navigation = self.spot_client.get_navigate_to_result()
        # recovery when riding on
        if not result_navigation.success or not result_switchbot.done:
            rospy.logerr('Failed to ride on a elevator. result_navigation: {}, result_switchbot: {}'.format(
                result_navigation, result_switchbot))
            self.spot_client.navigate_to(start_id, blocking=True)
            self.spot_client.wait_for_navigate_to_result()
            return False
        else:
            rospy.loginfo('Riding on succeded.')

        if not self.silent_mode:
            self.sound_client.say('{}階に行きます'.format(end_floor), blocking=False)

        # start door openning check from inside
        self.subscriber_door_check = rospy.Subscriber(
            '/spot_recognition/elevator_door_points',
            PointCloud2,
            self.door_point_cloud_callback)

        # check if the door is closed
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self.door_is_open:
                break
        rospy.loginfo('elevator door closed')

        # check if the door is open and at the target floor
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            rospy.loginfo('door_is_open: {}, is_target_floor: {}, stop from acc: {}'.format(
                self.door_is_open, self.is_target_floor, self.elevator_stop_acc))
            if self.door_is_open and self.is_target_floor and self.elevator_stop_acc:
                break
        rospy.loginfo('elevator door opened and at the target_floor')

        # dance before starting to move
        self.spot_client.pub_body_pose(0.0, Quaternion(w=1))
        self.spot_client.stand()
        rospy.sleep(0.5)
        self.spot_client.pub_body_pose(-0.2, Quaternion(w=1))
        self.spot_client.stand()
        rospy.sleep(0.5)
        self.spot_client.pub_body_pose(0.0, Quaternion(w=1))
        self.spot_client.stand()

        # get off the elevator
        self.spot_client.navigate_to(end_id, blocking=True)
        result = self.spot_client.get_navigate_to_result()

        return result.success

    def run_final(self, start_node, end_node, edge, pre_edge):

        rospy.logdebug('run_finalize() called')

        if self.subscriber_door_check is not None:
            self.subscriber_door_check.unregister()
            self.subscriber_door_check = None

        if self.subscriber_floor_detection is not None:
            self.subscriber_floor_detection.unregister()
            self.subscriber_floor_detection = None

        if self.subscriber_imu is not None:
            self.subscriber_imu.unregister()
            self.subscriber_imu = None

        self.roslaunch_parent.shutdown()


class CrosswalkBehavior(BaseBehavior):

    def callback_car_visible(self, msg):

        if self.car_state_visible != msg.data:
            self.car_starttime_visibility = rospy.Time.now()
            self.car_duration_visibility = rospy.Duration()
            self.car_state_visible = msg.data
        else:
            self.car_duration_visibility = rospy.Time.now() - self.car_starttime_visibility

    def run_initial(self, start_node, end_node, edge, pre_edge):

        rospy.logdebug('run_initial() called')

        self.silent_mode = rospy.get_param('~silent_mode', True)
        self.sound_client = self.client.sound_client
        self.spot_client = self.client.spot_ros_client

        # launch recognition launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        roslaunch_path = rospkg.RosPack().get_path('spot_basic_behaviors') +\
            '/launch/crosswalk_detection.launch'
        roslaunch_cli_args = [roslaunch_path]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
            roslaunch_cli_args)
        self.roslaunch_parent = roslaunch.parent.ROSLaunchParent(
            uuid,
            roslaunch_file
        )
        self.roslaunch_parent.start()

        # value for car checker
        self.subscriber_car_visible = None
        self.car_state_visible = False
        self.car_starttime_visibility = rospy.Time.now()
        self.car_duration_visibility = rospy.Duration(10)

        # start subscribers
        try:
            self.subscriber_car_visible = rospy.Subscriber(
                '/crosswalk_detection_car_tracker/visible',
                Bool,
                self.callback_car_visible
            )
        except Exception as e:
            rospy.logerr('{}'.format(e))
            return False

        return True

    def run_main(self, start_node, end_node, edge, pre_edge):

        rospy.logdebug('run_main() called')

        graph_name = edge.args['graph']
        start_id = filter(
            lambda x: x['graph'] == graph_name,
            start_node.properties['waypoints_on_graph']
        )[0]['id']
        end_id = filter(
            lambda x: x['graph'] == graph_name,
            end_node.properties['waypoints_on_graph']
        )[0]['id']
        localization_method = filter(
            lambda x: x['graph'] == graph_name,
            start_node.properties['waypoints_on_graph']
        )[0]['localization_method']

        # graph uploading and localization
        if pre_edge is not None and \
                graph_name == pre_edge.args['graph']:
            rospy.loginfo('graph upload and localization skipped.')
        else:
            # Upload
            ret = self.spot_client.upload_graph(graph_name)
            if ret[0]:
                rospy.loginfo('graph {} uploaded.'.format(graph_name))
            else:
                rospy.logerr('graph uploading failed: {}'.format(ret[1]))
                return False
            # Localization
            if localization_method == 'fiducial':
                ret = self.spot_client.set_localization_fiducial()
            elif localization_method == 'waypoint':
                ret = self.spot_client.set_localization_waypoint(start_id)
            else:
                ret = (False, 'Unknown localization method')
            if ret[0]:
                rospy.loginfo('robot is localized on the graph.')
            else:
                rospy.logwarn('Localization failed: {}'.format(ret[1]))
                return False

        # checking if there is a moving car visible or not
        if not self.silent_mode:
            self.sound_client.say('車が通るかみています', blocking=True)
        safety_count = 0
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            rospy.loginfo('safety_count: {}'.format(safety_count))
            if safety_count > 10:
                break
            try:
                rospy.loginfo('car visible: {}'.format(self.car_state_visible))
                if self.car_state_visible:
                    safety_count = 0
                    if not self.silent_mode:
                        self.sound_client.say('車が通ります', blocking=True)
                else:
                    safety_count += 1
            except Exception as e:
                rospy.logwarn('{}'.format(e))
                safety_count = 0

        # start leading
        success = False
        rate = rospy.Rate(10)
        if not self.silent_mode:
            self.sound_client.say('移動します', blocking=True)
        self.spot_client.navigate_to(end_id, blocking=False)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.spot_client.wait_for_navigate_to_result(rospy.Duration(0.1)):
                result = self.spot_client.get_navigate_to_result()
                success = result.success
                rospy.loginfo('result: {}'.format(result))
                break

        # recovery on failure
        if not success:
            if not self.silent_mode:
                self.sound_client.say('失敗したので元に戻ります', blocking=True)
            self.spot_client.navigate_to(start_id, blocking=True)
            self.spot_client.wait_for_navigate_to_result()

        return success

    def run_final(self, start_node, end_node, edge, pre_edge):

        rospy.logdebug('run_finalize() called')

        self.spot_client.cancel_navigate_to()

        if self.subscriber_car_visible is not None:
            self.subscriber_car_visible.unregister()
            self.subscriber_car_visible = None

        self.roslaunch_parent.shutdown()
