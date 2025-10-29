# 🤖 Solver ROS 2 Workspace

Workspace ROS 2 (Humble) del robot móvil **Solver**, desarrollado para mapeo, localización y navegación autónoma en entornos reales.

---

## 🧩 Estructura del Workspace

solverRos2_ws/
├── src/
│ ├── solver_description/ # URDF y meshes del robot
│ ├── solver_md49/ # Controlador del driver MD49
│ ├── solver_velocity/ # Conversión de encoders a velocidad lineal y angular
│ ├── solver_odometry/ # Integración de odometría diferencial
│ ├── solver_localization/ # EKF (fusión de sensores)
│ ├── solver_cartographer/ # SLAM Cartographer
│ ├── solver_navigation/ # Configuración para navegación (Nav2)
│ ├── solver_sensors/ # Sensores adicionales (IMU, LIDAR, etc.)
│ └── urg_node2/ # Nodo del LIDAR Hokuyo UST-05LX




Launcher principal de motores + Odom


ros2 launch solver_localization solver_nav_base.launch.py

Launcher del Cartographer


ros2 launch solver_cartographer cartographer.launch.py

Launcher del Lidar 


ros2 launch solver_sensors urg_node2.launch.py 




Guardar Mapa


ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
"{filename: '/home/atwork/mapa_solver.pbstream'}"


Conversion

ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -map_filestem ~/mapa_solver \
  -pbstream_filename ~/mapa_solver.pbstream \
  -resolution 0.05

