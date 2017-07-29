cmake_minimum_required(VERSION 3.5)

project(rcljava)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_export_jars REQUIRED)
find_package(rcl REQUIRED)
find_package(rcljava_common REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)

include(CrossCompilingExtra)

if(CMAKE_CROSSCOMPILING)
  find_host_package(Java COMPONENTS Development REQUIRED)
else()
  find_package(Java COMPONENTS Development REQUIRED)
endif()
if(NOT ANDROID)
  find_package(JNI REQUIRED)
endif()
include(UseJava)
include(JavaExtra)

set(CMAKE_JAVA_COMPILE_FLAGS "-source" "1.6" "-target" "1.6")

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

include_directories(include)

function(set_properties _targetname _build_type)
  set_target_properties(${_targetname} PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}"
    RUNTIME_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}"
  )
endfunction()

# Only build the library if a C typesupport exists
get_rmw_typesupport(typesupport_impls "rmw_implementation" LANGUAGE "c")
if(typesupport_impls STREQUAL "")
  message(STATUS "Skipping rcljava because no C typesupport library was found.")
  return()
endif()

set(${PROJECT_NAME}_jni_sources
  "src/main/cpp/org_ros2_rcljava_RCLJava.cpp"
  "src/main/cpp/org_ros2_rcljava_Time.cpp"
  "src/main/cpp/org_ros2_rcljava_client_ClientImpl.cpp"
  "src/main/cpp/org_ros2_rcljava_executors_BaseExecutor.cpp"
  "src/main/cpp/org_ros2_rcljava_node_NodeImpl.cpp"
  "src/main/cpp/org_ros2_rcljava_publisher_PublisherImpl.cpp"
  "src/main/cpp/org_ros2_rcljava_service_ServiceImpl.cpp"
  "src/main/cpp/org_ros2_rcljava_subscription_SubscriptionImpl.cpp"
  "src/main/cpp/org_ros2_rcljava_timer_WallTimerImpl.cpp"
)

foreach(_jni_source ${${PROJECT_NAME}_jni_sources})

  get_filename_component(_target_name "${_jni_source}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_target_name}" _target_name)

  add_library(
    ${_target_name}
    SHARED ${_jni_source}
  )

  set_properties("${target_name}" "")
  if(WIN32)
    set_properties("${target_name}" "_DEBUG")
    set_properties("${target_name}" "_MINSIZEREL")
    set_properties("${target_name}" "_RELEASE")
    set_properties("${target_name}" "_RELWITHDEBINFO")
  endif()

  ament_target_dependencies(${_target_name}
    "rcl"
    "rcljava_common"
    "builtin_interfaces"
  )

  target_include_directories(${_target_name}
    PUBLIC
    ${JNI_INCLUDE_DIRS}
  )

  ament_export_libraries(${_target_name})

  install(TARGETS ${_target_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
  )

endforeach()

set(${PROJECT_NAME}_sources
  "src/main/java/org/ros2/rcljava/RCLJava.java"
  "src/main/java/org/ros2/rcljava/Time.java"
  "src/main/java/org/ros2/rcljava/client/ClientImpl.java"
  "src/main/java/org/ros2/rcljava/client/Client.java"
  "src/main/java/org/ros2/rcljava/concurrent/Callback.java"
  "src/main/java/org/ros2/rcljava/concurrent/RCLFuture.java"
  "src/main/java/org/ros2/rcljava/consumers/BiConsumer.java"
  "src/main/java/org/ros2/rcljava/consumers/Consumer.java"
  "src/main/java/org/ros2/rcljava/consumers/TriConsumer.java"
  "src/main/java/org/ros2/rcljava/executors/AnyExecutable.java"
  "src/main/java/org/ros2/rcljava/executors/BaseExecutor.java"
  "src/main/java/org/ros2/rcljava/executors/Executor.java"
  "src/main/java/org/ros2/rcljava/executors/MultiThreadedExecutor.java"
  "src/main/java/org/ros2/rcljava/executors/SingleThreadedExecutor.java"
  "src/main/java/org/ros2/rcljava/node/BaseComposableNode.java"
  "src/main/java/org/ros2/rcljava/node/ComposableNode.java"
  "src/main/java/org/ros2/rcljava/node/NodeImpl.java"
  "src/main/java/org/ros2/rcljava/node/Node.java"
  "src/main/java/org/ros2/rcljava/publisher/PublisherImpl.java"
  "src/main/java/org/ros2/rcljava/publisher/Publisher.java"
  "src/main/java/org/ros2/rcljava/qos/policies/Durability.java"
  "src/main/java/org/ros2/rcljava/qos/policies/History.java"
  "src/main/java/org/ros2/rcljava/qos/policies/QoSPolicy.java"
  "src/main/java/org/ros2/rcljava/qos/policies/Reliability.java"
  "src/main/java/org/ros2/rcljava/qos/QoSProfile.java"
  "src/main/java/org/ros2/rcljava/service/RMWRequestId.java"
  "src/main/java/org/ros2/rcljava/service/ServiceImpl.java"
  "src/main/java/org/ros2/rcljava/service/Service.java"
  "src/main/java/org/ros2/rcljava/subscription/SubscriptionImpl.java"
  "src/main/java/org/ros2/rcljava/subscription/Subscription.java"
  "src/main/java/org/ros2/rcljava/time/ClockType.java"
  "src/main/java/org/ros2/rcljava/timer/WallTimer.java"
  "src/main/java/org/ros2/rcljava/timer/WallTimerImpl.java"
)

add_jar("${PROJECT_NAME}_jar"
  ${${PROJECT_NAME}_sources}
  OUTPUT_NAME
  ${PROJECT_NAME}
  INCLUDE_JARS
  ${rcljava_common_JARS}
  ${builtin_interfaces_JARS}
)

install_jar("${PROJECT_NAME}_jar" "share/${PROJECT_NAME}/java")
ament_export_jars("share/${PROJECT_NAME}/java/${PROJECT_NAME}.jar")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(std_msgs REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(${PROJECT_NAME}_message_files
    "msg/BoundedArrayNested.msg"
    "msg/BoundedArrayPrimitives.msg"
    "msg/Builtins.msg"
    "msg/DynamicArrayNested.msg"
    "msg/DynamicArrayPrimitives.msg"
    "msg/Empty.msg"
    "msg/FieldsWithSameType.msg"
    "msg/Nested.msg"
    "msg/Primitives.msg"
    "msg/StaticArrayNested.msg"
    "msg/StaticArrayPrimitives.msg"
    "msg/UInt32.msg"
  )

  set(${PROJECT_NAME}_service_files
    "srv/AddTwoInts.srv"
  )

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${${PROJECT_NAME}_message_files}
    ${${PROJECT_NAME}_service_files}
    DEPENDENCIES builtin_interfaces
    SKIP_INSTALL
  )

  get_property(_${PROJECT_NAME}_jar_file
    TARGET "${PROJECT_NAME}_jar"
    PROPERTY "JAR_FILE")

  get_property(_${PROJECT_NAME}_messages_jar_file
    TARGET "${PROJECT_NAME}_messages_jar"
    PROPERTY "JAR_FILE")

  set(${PROJECT_NAME}_test_sources
    "src/test/java/org/ros2/rcljava/RCLJavaTest.java"
    "src/test/java/org/ros2/rcljava/TimeTest.java"
    "src/test/java/org/ros2/rcljava/client/ClientTest.java"
    "src/test/java/org/ros2/rcljava/node/NodeTest.java"
    "src/test/java/org/ros2/rcljava/publisher/PublisherTest.java"
    "src/test/java/org/ros2/rcljava/subscription/SubscriptionTest.java"
    "src/test/java/org/ros2/rcljava/timer/TimerTest.java"
  )

  set(${PROJECT_NAME}_testsuites
    "org.ros2.rcljava.RCLJavaTest"
    "org.ros2.rcljava.TimeTest"
    "org.ros2.rcljava.client.ClientTest"
    "org.ros2.rcljava.node.NodeTest"
    "org.ros2.rcljava.publisher.PublisherTest"
    "org.ros2.rcljava.subscription.SubscriptionTest"
    "org.ros2.rcljava.timer.TimerTest"
  )

  set(_deps_library_paths "")
  foreach(_dep_lib ${rcljava_test_msgs_LIBRARIES})
    get_filename_component(_dep_dir "${_dep_lib}" DIRECTORY)
    list(APPEND _deps_library_paths ${_dep_dir})
  endforeach()
  foreach(_dep_lib ${std_msgs_LIBRARIES})
    get_filename_component(_dep_dir "${_dep_lib}" DIRECTORY)
    list(APPEND _deps_library_paths ${_dep_dir})
  endforeach()
  foreach(_dep_lib ${builtin_interfaces_LIBRARIES})
    get_filename_component(_dep_dir "${_dep_lib}" DIRECTORY)
    list(APPEND _deps_library_paths ${_dep_dir})
  endforeach()
  list(APPEND _deps_library_paths ${CMAKE_CURRENT_BINARY_DIR})
  list(APPEND _deps_library_paths ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_java/rcljava/msg/)
  list(APPEND _deps_library_paths ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_java/rcljava/srv/)

  foreach(testsuite ${${PROJECT_NAME}_testsuites})
    add_junit_tests("${PROJECT_NAME}_tests_${testsuite}"
      "${${PROJECT_NAME}_test_sources}"
      TESTS
      "${testsuite}"
      INCLUDE_JARS
      "${rcljava_common_JARS}"
      "${rcljava_test_msgs_JARS}"
      "${std_msgs_JARS}"
      "${builtin_interfaces_JARS}"
      "${_${PROJECT_NAME}_jar_file}"
      "${_${PROJECT_NAME}_messages_jar_file}"
      LIBRARY_PATHS
      "${_deps_library_paths}"
    )

    add_dependencies("${PROJECT_NAME}_tests_${testsuite}" "${PROJECT_NAME}_messages_jar")
  endforeach()

endif()

ament_package()
