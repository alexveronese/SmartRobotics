cmake_minimum_required(VERSION 3.8)
project(nuovo_progetto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Trova le dipendenze
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Installa le cartelle principali
install(
  DIRECTORY launch urdf config worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

# Rende eseguibili gli script Python nella cartella scripts
install(
  PROGRAMS scripts/smistatore_colori.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()