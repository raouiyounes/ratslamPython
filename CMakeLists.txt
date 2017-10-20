cmake_minimum_required(VERSION 2.8.3)
project(node_example)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation roscpp rosgraph_msgs nav_msgs rospy std_msgs)


add_message_files(
  FILES
  NodeExampleData.msg
  TopologicalAction.msg
  TopologicalEdge.msg
  TopologicalMap.msg
  TopologicalNode.msg
  ViewTemplate.msg
  )


generate_messages(
  DEPENDENCIES
     geometry_msgs  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/nodeExample.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp rosgraph_msgs rospy std_msgs visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})


install(PROGRAMS src/main_lv.py src/pylistener.py  src/pytalker.py src/ratslam/posecell_networkv.py  src/ratslam/experience_map_younes.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



install(PROGRAMS   src/main_lv.py src/main_em.py src/pytalker.py src/ratslam/posecell_networkv.py  src/ratslam/experience_map_younes.py  src/ratslam/local_view_match.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
