cmake_minimum_required(VERSION 2.8.3)
project(behavior_manager_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib actionlib_msgs)

add_action_files(
    DIRECTORY action
    FILES ExecuteBehaviors.action
)

add_service_files(
    DIRECTORY srv
    FILES ResetCurrentNode.srv
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  LIBRARIES behavior_manager_msgs
  CATKIN_DEPENDS message_runtime
)
