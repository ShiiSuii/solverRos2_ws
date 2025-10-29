cmake_minimum_required(VERSION 3.22)
project(solver_cartographer)

# === Dependencias básicas ===
find_package(ament_cmake REQUIRED)
find_package(cartographer_ros REQUIRED)

# === Instalar archivos de configuración y launch ===
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

