cmake_minimum_required(VERSION 3.8)
project(turtlebot3_warehouse_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Optional vendored assets location (worlds/models/maps) under this package.
# We intentionally don't commit huge assets by default; a helper script can copy them in.
install(DIRECTORY warehouse_assets
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


