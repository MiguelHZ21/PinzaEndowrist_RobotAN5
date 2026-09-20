 Pinza End-Effector para Robot
==================================================================================

Este proyecto contiene una pinza inteligente programable para el robot FR-5, desarrollada como parte del proyecto de investigación "Automatización de Procesos de Ensamble y Manipulación mediante Robótica Avanzada".

La pinza está equipada con:
- Un actuador de rotación que permite girar hasta 100°.
- Un sensor de fuerza que mide hasta 50 N.
- Un sensor de posición que registra hasta 12 mm.

Características principales:
- **Interacción con el usuario** mediante interfaz gráfica en Unity 3D.
- **Programación sencilla** de secuencias de movimiento y control de fuerza.
- **Comunicación en tiempo real** con el robot.

## 🚀 Estructura del Proyecto

El repositorio está organizado en dos grandes módulos principales:

### 📁 fr5_ik
Contiene la lógica de control y cinematográfica del robot.
- **fr5_kinematics.py**: Módulo de cinemática inversa.
- **fr5_ik_node.py**: Nodo ROS 2 para la integración con el robot.
- **fr5_interface.py**: Interfaz de comunicación con el sistema.

### 📁 Unity-fr5-Interface
Contiene la aplicación de interfaz gráfica desarrollada en Unity.
- **Assets/**: Modelos 3D del robot, scripts y recursos visuales.
- **Scenes/**: Escenas de la aplicación (MainScene).
- **Scripts/**: Controladores de la interfaz y comunicación con ROS 2.

## 🛠️ Instalación y Configuración

### Prerrequisitos
- **ROS 2 Humble** instalado.
- **Python 3.8+**.
- **Unity** (versión compatible con ROS 2).

### Instalación del Backend (ROS 2)
1. Clona el repositorio:
   ```bash
   git clone <url-del-repositorio>
   cd fr5_ik
   ```

2. Crea un entorno virtual e instálalo:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

### Instalación del Frontend (Unity)
1. Abre el proyecto en Unity:
   - Navega a la carpeta `Unity-fr5-Interface`.
   - Ábrela con Unity Hub.

## ⚙️ Uso

### Ejecutar la Interfaz Gráfica
1. Asegúrate de que el backend esté corriendo y de que ROS esté inicializado:
   ```bash
   # En una terminal (fr5_ik)
   source /opt/ros/humble/setup.bash
   source venv/bin/activate
   python3 fr5_interface.py
   ```

2. En Unity, presiona **Play** en el editor o ejecuta la build.
3. Configura la dirección IP del robot en la interfaz si es necesario.
4. Usa los controles para mover el robot y probar la pinza.


