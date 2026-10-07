cmake_minimum_required(VERSION 3.0.2)
project(mando_localization)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  erp42_msgs
  geometry_msgs
  message_generation
  nav_msgs
  robot_localization
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  topic_tools
  visualization_msgs
  ublox_msgs
)

add_message_files(
  FILES
    GpsGateReanchor.msg
    RddfCandidate.msg
    RddfMatch.msg
)

add_service_files(FILES SetInitialHeading.srv)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  LIBRARIES
    mando_localization_common
    mando_localization_interface_adapter
    mando_encoder_to_twist
    mando_imu_normalizer
    mando_odometry_gps_fusion
    mando_localization_status
    mando_relocalization
    mando_localization_supervisor
    mando_localization_output_gate
    mando_localization_tf
    mando_localization_visualization
  CATKIN_DEPENDS
    diagnostic_msgs erp42_msgs geometry_msgs message_runtime nav_msgs robot_localization roscpp
    sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros topic_tools visualization_msgs ublox_msgs
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

add_library(mando_localization_common
  src/common/consecutive_recovery_gate.cpp
  src/common/message_validation.cpp
  src/common/parameter_utils.cpp
)
target_link_libraries(mando_localization_common ${catkin_LIBRARIES})

add_library(mando_localization_interface_adapter
  src/common/localization_interface_adapter.cpp
)
target_link_libraries(mando_localization_interface_adapter
  mando_localization_common ${catkin_LIBRARIES}
)
add_executable(localization_interface_adapter_node
  src/common/localization_interface_adapter_node.cpp
)
target_link_libraries(localization_interface_adapter_node
  mando_localization_interface_adapter ${catkin_LIBRARIES}
)

add_library(mando_encoder_to_twist
  src/imu_encoder_fusion/encoder_to_twist_adapter.cpp
)
add_dependencies(mando_encoder_to_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(mando_encoder_to_twist mando_localization_common ${catkin_LIBRARIES})
add_executable(encoder_to_twist_adapter_node
  src/imu_encoder_fusion/encoder_to_twist_adapter_node.cpp
)
add_dependencies(encoder_to_twist_adapter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(encoder_to_twist_adapter_node mando_encoder_to_twist ${catkin_LIBRARIES})

add_library(mando_imu_normalizer
  src/imu_encoder_fusion/imu_normalizer.cpp
)
target_link_libraries(mando_imu_normalizer mando_localization_common ${catkin_LIBRARIES})
add_executable(imu_normalizer_node
  src/imu_encoder_fusion/imu_normalizer_node.cpp
)
target_link_libraries(imu_normalizer_node mando_imu_normalizer ${catkin_LIBRARIES})

add_library(mando_odometry_gps_fusion
  src/odometry_gps_fusion/odometry_gps_fusion.cpp
)
add_dependencies(mando_odometry_gps_fusion
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mando_odometry_gps_fusion mando_localization_common ${catkin_LIBRARIES})
add_executable(odometry_gps_fusion_node
  src/odometry_gps_fusion/odometry_gps_fusion_node.cpp
)
add_dependencies(odometry_gps_fusion_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(odometry_gps_fusion_node mando_odometry_gps_fusion ${catkin_LIBRARIES})


add_library(mando_localization_status
  src/status_manager/localization_state_evaluator.cpp
  src/status_manager/localization_status_manager.cpp
)
add_dependencies(mando_localization_status ${catkin_EXPORTED_TARGETS})
target_link_libraries(mando_localization_status ${catkin_LIBRARIES})

add_library(mando_relocalization
  src/relocalization/relocalization_policy.cpp
  src/relocalization/relocalization_coordinator.cpp
)
add_dependencies(mando_relocalization ${catkin_EXPORTED_TARGETS})
add_dependencies(mando_relocalization ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(mando_relocalization
  mando_localization_common ${catkin_LIBRARIES}
)

add_library(mando_localization_supervisor
  src/supervisor/localization_supervisor.cpp
)
target_link_libraries(mando_localization_supervisor
  mando_localization_common ${catkin_LIBRARIES}
)
add_executable(localization_supervisor_node
  src/supervisor/localization_supervisor_node.cpp
)
add_dependencies(localization_supervisor_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
add_dependencies(localization_supervisor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(localization_supervisor_node
  mando_localization_status
  mando_relocalization
  mando_localization_supervisor
  ${catkin_LIBRARIES}
)

add_library(mando_localization_output_gate
  src/output_gate/localization_output_gate.cpp
)
target_link_libraries(mando_localization_output_gate
  mando_localization_common ${catkin_LIBRARIES}
)
add_executable(localization_output_gate_node
  src/output_gate/localization_output_gate_node.cpp
)
target_link_libraries(localization_output_gate_node mando_localization_output_gate ${catkin_LIBRARIES})

add_library(mando_localization_tf
  src/tf/transform_configuration.cpp
  src/tf/static_transform_publisher.cpp
)
target_link_libraries(mando_localization_tf ${catkin_LIBRARIES})
add_executable(static_transform_publisher_node
  src/tf/static_transform_publisher_node.cpp
)
target_link_libraries(static_transform_publisher_node mando_localization_tf ${catkin_LIBRARIES})

add_library(mando_localization_visualization
  src/visualization/localization_visualization.cpp
)
target_link_libraries(mando_localization_visualization ${catkin_LIBRARIES})
add_executable(localization_visualization_node
  src/visualization/localization_visualization_node.cpp
)
target_link_libraries(localization_visualization_node mando_localization_visualization ${catkin_LIBRARIES})

set(MANDO_NODE_TARGETS
  localization_interface_adapter_node
  encoder_to_twist_adapter_node
  imu_normalizer_node
  odometry_gps_fusion_node
  localization_supervisor_node
  localization_output_gate_node
  static_transform_publisher_node
  localization_visualization_node
)
set(MANDO_LIBRARY_TARGETS
  mando_localization_common
  mando_localization_interface_adapter
  mando_encoder_to_twist
  mando_imu_normalizer
  mando_odometry_gps_fusion
  mando_localization_status
  mando_relocalization
  mando_localization_supervisor
  mando_localization_output_gate
  mando_localization_tf
  mando_localization_visualization
)

install(TARGETS ${MANDO_NODE_TARGETS} ${MANDO_LIBRARY_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config docs launch maps rddf udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS
  scripts/rddf_initialization_core.py
  scripts/rddf_tracking_core.py
  scripts/rddf_tracker_node.py
  scripts/rddf_initializer_node.py
  scripts/calibrated_imu_core.py
  scripts/calibrated_imu_node.py
  scripts/timing_core.py
  scripts/check_time_sync.py
  scripts/sensor_timing_monitor.py
  scripts/localization_viewer.py
  scripts/lidar_front_scan_visualizer.py
  scripts/live_imu_test_relay.py
  scripts/keyboard_encoder_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS
  scripts/localization_command.sh
  scripts/localization_record_command.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/startup_gate.test)
  add_rostest(test/rddf_initialization.test)
  add_rostest(test/rddf_initialization_manual.test)
  add_rostest(test/rddf_tracking.test)
  catkin_add_gtest(test_state_evaluator
    test/gtest_main.cpp
    test/test_state_evaluator.cpp
  )
  if(TARGET test_state_evaluator)
    target_link_libraries(test_state_evaluator mando_localization_status ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_output_gate
    test/gtest_main.cpp
    test/test_output_gate.cpp
  )
  if(TARGET test_output_gate)
    target_link_libraries(test_output_gate mando_localization_output_gate ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_relocalization_policy
    test/gtest_main.cpp
    test/test_relocalization_policy.cpp
  )
  if(TARGET test_relocalization_policy)
    target_link_libraries(test_relocalization_policy
      mando_relocalization ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_localization_supervisor
    test/gtest_main.cpp
    test/test_localization_supervisor.cpp
  )
  if(TARGET test_localization_supervisor)
    target_link_libraries(test_localization_supervisor
      mando_localization_supervisor ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_transform_configuration
    test/gtest_main.cpp
    test/test_transform_configuration.cpp
  )
  if(TARGET test_transform_configuration)
    target_link_libraries(test_transform_configuration mando_localization_tf ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_consecutive_recovery_gate
    test/gtest_main.cpp
    test/test_consecutive_recovery_gate.cpp
  )
  if(TARGET test_consecutive_recovery_gate)
    target_link_libraries(test_consecutive_recovery_gate
      mando_localization_common ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_message_validation
    test/gtest_main.cpp
    test/test_message_validation.cpp
  )
  if(TARGET test_message_validation)
    target_link_libraries(test_message_validation
      mando_localization_common ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_encoder_to_twist
    test/gtest_main.cpp
    test/test_encoder_to_twist.cpp
  )
  if(TARGET test_encoder_to_twist)
    add_dependencies(test_encoder_to_twist
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(test_encoder_to_twist
      mando_encoder_to_twist ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_local_odometry_history
    test/gtest_main.cpp
    test/test_local_odometry_history.cpp
  )
  if(TARGET test_local_odometry_history)
    target_link_libraries(test_local_odometry_history
      mando_localization_common ${catkin_LIBRARIES})
  endif()

  add_rostest(test/gps_gate_reanchor.test
    DEPENDENCIES odometry_gps_fusion_node
  )
  add_rostest(test/delayed_gps_alignment.test
    DEPENDENCIES odometry_gps_fusion_node
  )
  add_rostest(test/gps_clock_ready.test
    DEPENDENCIES odometry_gps_fusion_node
  )
  add_rostest(test/delayed_gps_ekf.test
    DEPENDENCIES odometry_gps_fusion_node localization_interface_adapter_node
                 imu_normalizer_node encoder_to_twist_adapter_node
                 localization_supervisor_node localization_output_gate_node
                 static_transform_publisher_node
  )
  add_rostest(test/gps_only_relocalization.test
    DEPENDENCIES localization_supervisor_node
  )
  add_rostest(test/calibrated_imu.test
    DEPENDENCIES imu_normalizer_node localization_interface_adapter_node
                 encoder_to_twist_adapter_node static_transform_publisher_node
  )
  add_rostest(test/calibrated_imu_future.test
    DEPENDENCIES imu_normalizer_node localization_interface_adapter_node
                 encoder_to_twist_adapter_node static_transform_publisher_node
  )
  add_rostest(test/initial_heading.test
    DEPENDENCIES imu_normalizer_node localization_interface_adapter_node
                 encoder_to_twist_adapter_node
  )
  add_rostest(test/encoder_yaw_hold.test
    DEPENDENCIES imu_normalizer_node localization_interface_adapter_node
                 encoder_to_twist_adapter_node static_transform_publisher_node
  )

  catkin_add_nosetests(test/test_localization_viewer_initialization.py)
  catkin_add_nosetests(test/test_localization_viewer.py)
  catkin_add_nosetests(test/test_rddf_tracking.py)
  catkin_add_nosetests(test/test_configuration.py)
  catkin_add_nosetests(test/test_rddf_initialization_core.py)
  catkin_add_nosetests(test/test_rddf_heading.py)
  catkin_add_nosetests(test/test_calibrated_imu.py)
  catkin_add_nosetests(test/test_encoder_yaw_hold.py)
  catkin_add_nosetests(test/test_sensor_timing.py)
  catkin_add_nosetests(test/test_lidar_front_scan_visualizer.py)
endif()
