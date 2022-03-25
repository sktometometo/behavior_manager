cmake_minimum_required(VERSION 2.8.3)
project(behavior_manager_server)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS node_scripts/behavior_manager_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/behavior_manager_node.test)
endif()
