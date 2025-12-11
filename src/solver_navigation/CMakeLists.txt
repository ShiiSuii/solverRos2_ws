cmake_minimum_required(VERSION 3.5)
project(solver_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch config resource
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    # No scripts aquí: el ejecutable se instala desde entry_points en setup.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
