cmake_minimum_required(VERSION 3.8)
project(reemc_description)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes robots
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_package()
