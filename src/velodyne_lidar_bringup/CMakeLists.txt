cmake_minimum_required(VERSION 3.8)
project(velodyne_lidar_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

#----Lines added to the existing file 

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

#------------------------------------------
##################################

ament_package()
