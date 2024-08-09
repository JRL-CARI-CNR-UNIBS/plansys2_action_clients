cmake_minimum_required(VERSION 3.8)
project(plansys2_action_clients)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(state_observers REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
# find_package(plansys2_actions_cost REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    plansys2_executor
    state_observers
    plansys2_msgs
    pluginlib
    # plansys2_actions_cost
)

include_directories(include
  # ${plansys2_actions_cost_INCLUDE_DIRS}
  # ${state_observers_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/action_observed_cost_client.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(DIRECTORY include/
  DESTINATION include/
)
install(TARGETS
  ${PROJECT_NAME}
  # EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
# ament_export_targets(
#   ${PROJECT_NAME}
#   HAS_LIBRARY_TARGET
# )
ament_package()
