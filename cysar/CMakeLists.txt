cmake_minimum_required(VERSION 3.5)
project(cysar)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Build msg files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Joystick.msg"
  "msg/FlipperPosition.msg"
  "msg/DriveTrain.msg"
  "msg/ArmPosition.msg"
)

ament_package()

install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*"
)

file(GLOB files src/*)

foreach(file ${files})
  if(NOT IS_DIRECTORY "${file}" AND EXISTS "${file}")
    install(
      PROGRAMS "${file}"
      DESTINATION "lib/${PROJECT_NAME}"
    )
  endif()
endforeach()