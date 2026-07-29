# robot_vision_gazebo

Vehículo autónomo con navegación basada **exclusivamente en visión artificial**,
simulado en Gazebo (ROS2 Humble + Gazebo Garden/Harmonic + OpenCV).

## Estructura
```
robot_vision_gazebo/
├── launch/robot.launch.py       # Orquestación 1-clic de la simulación
├── urdf/robot.urdf              # Gemelo digital (chasis 2WD + cámara)
├── worlds/line_world.world      # Pista de alto contraste + obstáculo rojo
├── config/params.yaml           # Kp, Kd, umbrales HSV (sin recompilar)
└── robot_vision_gazebo/
    ├── vision_node.py           # Nodo principal: ROI, HSV, centroide, PID/FSM
    ├── pid_controller.py        # Regulador PD/PID de velocidad angular
    └── fsm_rebase.py            # Máquina de estados de evasión
```

## Compilar
En Linux/macOS:
```bash
cd ~/robot_vision_workspace
colcon build --merge-install
source install/setup.bash
```

En Windows PowerShell:
```powershell
cd D:\UserPC\Descargas\src\robot_vision_gazebo
colcon build --merge-install
install\setup.bat
```

## Ejecutar
```bash
ros2 launch robot_vision_gazebo robot.launch.py
```

## Verificar tópicos
```bash
ros2 topic list
# /camera/image_raw
# /cmd_vel
# /vision_controller/error
# /vision_controller/fps
# /vision_controller/obstacle
```

## Métricas disponibles
- `/vision_controller/error`: error de trayectoria en pixeles.
- `/vision_controller/fps`: tasa de procesamiento de imágenes suavizada.
- `/vision_controller/obstacle`: detección de obstáculo superior.

## Validar (rqt_plot + rosbag)
```bash
ros2 bag record /camera/image_raw /cmd_vel /tf
ros2 run rqt_plot rqt_plot /vision_controller/error
```

## Ajustar ganancias sin recompilar
Edita `config/params.yaml` (Kp, Kd, umbrales HSV, red_area_threshold) y vuelve a lanzar.

## Métricas de aceptación (ver blueprint)
- Estabilidad visual: ≥ 15 FPS
- Precisión (RMSE del centroide): ≤ 5% del ancho de imagen en tramos rectos
- Eficacia de evasión: ≥ 90% de éxito en rebase sin colisión
