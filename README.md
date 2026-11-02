# Interfaz Unity 3D y Plataforma de Control — Robot AN5 (FAIRINO FR5) + Pinza EndoWrist Xi
==================================================================================

Este proyecto integra un gemelo digital interactivo en **Unity 3D**, middleware **ROS 2 Humble** y cinemática analítica cerrada en **Python** para el control coordinado del robot colaborativo de 6 GDL **AN5 (FAIRINO FR5)** y la pinza quirúrgica de 4 GDL **EndoWrist Xi** accionada por servomotores de bus **ST3215**.

La pinza quirúrgica EndoWrist Xi cuenta con 4 grados de libertad:
- **`shaft` (`Endo_eje`)**: Rotación axial del vástago ($[-180^\circ, +180^\circ]$).
- **`wrist` (`Endo_muneca`)**: Flexión/inclinación de la muñeca ($[-70^\circ, +70^\circ]$).
- **`jaw_dx` / `jaw_sx` (`Endo_mandibula_dx/sx`)**: Apertura independiente de mandíbulas ($[-60^\circ, +60^\circ]$).

Características principales:
- **Interacción con el usuario** mediante interfaz gráfica en Unity 3D (Modos Manual Articular, Manual Cartesiano, Panel EndoWrist, Grabación Free Drive y Ejecución/Preview de trayectorias TXT).
- **Seguridad operativa multicapa** (`SafetyRobot.cs` en Unity y validación redundante en Python con distancia angular mínima en $\pm 180^\circ$).
- **Sincronización en lazo cerrado** entre el brazo robótico AN5 y la pinza EndoWrist Xi sobre ROS 2 Humble.

## 🚀 Estructura del Proyecto

El repositorio está organizado en dos grandes módulos principales:

### 📁 Main_node
Contiene la lógica de control, cinemática y gestión de trayectorias del robot y la pinza.
- **fr5_kinematics.py**: Módulo de cinemática directa e inversa analítica cerrada del robot AN5 (FR5).
- **ST3215_kinematics.py**: Módulo para el control de los servomotores ST3215 de la pinza EndoWrist Xi.
- **fr5_trajectory_manager.py**: Gestor de validación y ejecución de trayectorias en modos `con_pinza` y `sin_pinza`.
- **Run_node.py**: Nodo principal ROS 2 para la integración con el robot y la pinza.

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
   cd Main_node
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
   # En una terminal (Main_node)
   source /opt/ros/humble/setup.bash
   python3 Run_node.py
   ```

2. En Unity, presiona **Play** en el editor o ejecuta la build.
3. Configura la dirección IP del robot en la interfaz si es necesario.
4. Usa los controles para mover el robot y probar la pinza.


