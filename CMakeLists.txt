cmake_minimum_required(VERSION 3.8)
project(nuovo_progetto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Trova le dipendenze
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(tic_tac_toe)

# Installa le cartelle principali
install(
  DIRECTORY launch urdf config worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Rende eseguibili gli script Python nella cartella scripts
install(
  PROGRAMS scripts/tic_tac_toe_game.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_game test/test_game.py)
  ament_add_pytest_test(test_scene test/test_scene.py)
  ament_add_pytest_test(test_vision test/test_vision.py)
endif()

ament_package()
