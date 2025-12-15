cmake_minimum_required(VERSION 3.8)
project(turtlebot3_large_warehouse_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Warehouse assets (worlds, models, maps)
install(DIRECTORY warehouse_assets
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

