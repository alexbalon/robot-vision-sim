# Robot-Vision: Vehículo Autónomo Guiado por Visión (Simulación)

Sistema de navegación autónoma que mantiene un robot diferencial centrado sobre una pista mediante visión computacional (cámara única, sin sensores de distancia), y que detecta un obstáculo estático, ejecuta una maniobra de rebase y se reincorpora automáticamente a la trayectoria original.

Implementado y validado en **ROS2 Humble + Gazebo Sim (Garden)**, siguiendo la Guía del Proyecto de Diseño (Ing. Joffre L. León Veas).

## Estado del proyecto

✅ Seguimiento de línea (PID) — funcional y estable
✅ Detección y evasión de obstáculos (FSM) — 100% de éxito en 10 pruebas
✅ Métricas de validación documentadas (ver `Informe_Tecnico_Robot_Vision.pdf`)

## Requisitos

| Requisito | Versión |
|---|---|
| Sistema Operativo | Ubuntu 22.04 LTS |
| ROS2 | Humble Hawksbill |
| Simulador | Gazebo Sim (Garden) |
| Python | 3.10+ |
| RAM | ≥ 8 GB recomendado |

## Instalación

### 1. Preparar el sistema e instalar ROS2

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales software-properties-common curl gnupg2 lsb-release
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
```

### 2. Instalar Gazebo Sim (Garden) y el puente ROS2

```bash
sudo apt install -y gz-garden ros-humble-ros-gz
```

### 3. Instalar OpenCV (vía apt, NUNCA con pip)

```bash
sudo apt install -y python3-opencv ros-humble-cv-bridge ros-humble-image-transport
```

> ⚠️ **Importante:** `pip install opencv-python` genera conflictos de ABI con `cv_bridge`. Usa siempre los paquetes del sistema.

### 4. Clonar y compilar el workspace

```bash
mkdir -p ~/robot-vision-sim/src
cd ~/robot-vision-sim
git clone https://github.com/alexbalon/robot-vision-sim.git src/robot_vision_gazebo_pkg
# (o copia el contenido de este repositorio directamente en src/)

colcon build
source install/setup.bash
echo "source ~/robot-vision-sim/install/setup.bash" >> ~/.bashrc
```

## Ejecución

```bash
cd ~/robot-vision-sim
source install/setup.bash

# Si tu GPU no soporta renderizado acelerado (ver "Problemas conocidos" abajo):
export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch robot_vision_gazebo robot.launch.py
```

Esto abre Gazebo con el mundo de la pista (`worlds/line_world.world`), el robot diferencial con cámara frontal, y arranca el nodo de control de visión. El robot debería:
1. Detectar y seguir la línea blanca automáticamente.
2. Al acercarse al obstáculo rojo, ejecutar la maniobra de evasión (girar, avanzar, regresar).
3. Reincorporarse a la línea y continuar el recorrido.

### Verificar tópicos activos

```bash
ros2 topic list
ros2 topic echo /vision_controller/error   # error de trayectoria en tiempo real
ros2 topic echo /vision_controller/fps     # FPS reportado
```

## Estructura del repositorio

robot_vision_gazebo/
├── robot_vision_gazebo/
│ ├── vision_node.py # Nodo principal: visión, PID, integración FSM
│ ├── pid_controller.py # Controlador PID del seguimiento de línea
│ └── fsm_rebase.py # Máquina de estados de evasión de obstáculos
├── launch/
│ └── robot.launch.py # Lanza Gazebo + robot + puente + nodo de visión
├── worlds/
│ └── line_world.world # Mundo SDF: pista, obstáculo, iluminación
├── urdf/
│ └── robot.urdf # Modelo del robot diferencial con cámara
├── config/
│ └── params.yaml # Umbrales HSV, ganancias PID, parámetros FSM
├── scripts/
│ └── metrics_logger.py # Captura de métricas (error/FPS) a CSV
└── README.md


## Parámetros clave (config/params.yaml)

| Parámetro | Valor | Descripción |
|---|---|---|
| `hsv_line_low` / `hsv_line_high` | `[0,0,180]` / `[180,60,255]` | Umbral HSV para detectar la línea blanca |
| `red_area_threshold` | `6000` px² | Área mínima para disparar la evasión |
| `Kp` / `Kd` | `0.003` / `0.004` | Ganancias del PID de seguimiento |
| `linear_speed` | `0.12` m/s | Velocidad lineal de avance |

## Resultados de validación

| Métrica | Requisito | Resultado |
|---|---|---|
| Tasa de detección de obstáculos | ≥ 90% | **100%** (10/10 pruebas) |
| FPS | ≥ 15 | 20.0 (ver limitación en el informe) |
| Error de trayectoria (mediana) | ≤ 5% del ancho de pista | 3 px (~1.9%) |

Detalle completo, gráficas y análisis de fallas en [`Informe_Tecnico_Robot_Vision.pdf`](./Informe_Tecnico_Robot_Vision.pdf).

## Problemas conocidos

- **Renderizado GPU falla en algunos equipos** (`amdgpu_device_initialize failed`, `libEGL warning`): usar `export LIBGL_ALWAYS_SOFTWARE=1` antes de lanzar (fuerza renderizado por software; más lento pero estable).
- **FPS reportado puede no reflejar throughput real de cómputo** — ver Sección 7 del informe técnico.
- **Error de trayectoria elevado justo después del reenganche** tras la evasión, mientras el PID re-estabiliza — ver Sección 5.1 y 7 del informe técnico.

## Historial de desarrollo

Este proyecto documenta 9 ciclos de depuración (detección de línea, control PID, temporización de la FSM, geometría de la maniobra de evasión, entre otros), detallados en la Sección 6 del informe técnico, como evidencia del proceso iterativo de diseño ingenieril.

## Referencias

- Guía del Proyecto de Diseño — Robot-Vision (Ing. Joffre L. León Veas)
- Guía Técnica de Instalación y Configuración — ROS2 + OpenCV para Simulación
- [Documentación ROS2 Humble](https://docs.ros.org/en/humble/)
- [Documentación Gazebo Sim (Garden)](https://gazebosim.org/docs/garden/ros2)

## Autor

Alex Balón Garófalo
