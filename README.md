# 🤖 Solver ROS2 – Launchers principales

Sistema de navegación del robot **Solver** en **ROS 2 Humble**, con módulos de odometría, mapeo y navegación.

1️⃣ ODOMETRÍA

Archivo: `solver_odometry/launch/solver_odometry.launch.py`  
Función: Publica la odometría base del robot a partir de los encoders y fusiona datos con el filtro EKF.

Comando:

ros2 launch solver_localization solver_nav_base.launch.py    

2️⃣ SLAM (CARTOGRAPHER)

Archivo: solver_mapping/launch/cartographer_solver.launch.py
Función: Genera un mapa en tiempo real usando el LIDAR y la odometría filtrada.

Comando:

ros2 launch solver_cartographer cartographer.launch.py

3️⃣ NAVEGACIÓN (NAV2)

Archivo: solver_navigation/launch/nav2_bringup.launch.py
Función: Permite al robot navegar de forma autónoma en el mapa guardado usando AMCL y los módulos de Nav2.

Comando:

ros2 launch solver_navigation nav2_bringup.launch.py
