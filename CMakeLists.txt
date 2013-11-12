cmake_minimum_required(VERSION 2.8.3)
project(annotations_store)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  rosconsole
  roscpp
  rostest
  warehouse_ros
  ar_track_alvar
  yocs_msgs
  visualization_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  MapListEntry.msg
)
add_service_files(
  DIRECTORY srv
  FILES
  DeleteAnnotations.srv
  ListAnnotations.srv
  PublishAnnotations.srv
  RenameAnnotations.srv
  SaveAnnotations.srv
  SaveMarkers.srv
)
generate_messages(DEPENDENCIES ar_track_alvar)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(annotations_saver src/annotations_saver.cpp)
add_dependencies(annotations_saver annotations_store_gencpp)
target_link_libraries(annotations_saver uuid ${catkin_LIBRARIES})

add_executable(annotations_manager src/annotations_manager.cpp)
add_dependencies(annotations_manager annotations_store_gencpp)
target_link_libraries(annotations_manager uuid ${catkin_LIBRARIES})

add_rostest(test/annotations_store.test)

install(TARGETS annotations_saver
	            annotations_manager
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/add_map.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
