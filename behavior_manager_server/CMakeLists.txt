cmake_minimum_required(VERSION 2.8.3)
project(behavior_manager_server)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS node_scripts/behavior_manager_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_behavior_manager_with_failed_edge.test)
    #add_rostest(test/test_behavior_manager_without_failed_edge.test)
endif()
