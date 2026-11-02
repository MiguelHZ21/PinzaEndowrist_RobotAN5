#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ***************************************************************
# Autores:    Miguel Hernandez (miguelhernandez@unicauca.edu.co)
#             Cristian Gonzalez (cgonzalezg@unicauca.edu.co)
# ***************************************************************
# =========================================================================
# ROS2 Sync Robot (String) + EndoWrist (String)
# Emulador de trayectoria que carga el archivo ArticularPoints/CartesianPoints
# con la fecha más reciente y publica sincronizadamente en
# /current_joint_position y /current_endowrist_position (ambos en GRADOS).
# =========================================================================

import glob
import math
import os
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

# Importar solucionador de cinemática para FR5 desde Main_node
MAIN_NODE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'Main_node')
if os.path.exists(MAIN_NODE_PATH) and MAIN_NODE_PATH not in sys.path:
    sys.path.insert(0, MAIN_NODE_PATH)

try:
    # pyrefly: ignore [missing-import]
    from fr5_kinematics import ik_mejor, pose_a_T, ParamsFR5
    HAS_FR5_IK = True
except ImportError:
    HAS_FR5_IK = False


def find_latest_trajectory_file(search_dirs):
    """
    Busca el archivo de trayectoria más reciente en las rutas especificadas.
    Prioriza patrones ArticularPoints_*.txt, CartesianPoints_*.txt, ArticularPoints.txt, CartesianPoints.txt.
    Regresa la ruta absoluta del archivo más reciente según timestamp o fecha de modificación.
    """
    patterns = [
        "ArticularPoints_*.txt",
        "CartesianPoints_*.txt",
        "ArticularPoints.txt",
        "CartesianoPoints.txt",
        "RecordPoints.txt",
    ]

    matching_files = []
    for d in search_dirs:
        if os.path.exists(d):
            for pattern in patterns:
                matching_files.extend(glob.glob(os.path.join(d, pattern)))

    if not matching_files:
        return None

    matching_files = list(set(matching_files))

    def get_sort_key(filepath):
        filename = os.path.basename(filepath)
        digits = ''.join(filter(str.isdigit, filename))
        if len(digits) >= 14:
            return (1, digits)
        return (0, os.path.getmtime(filepath))

    matching_files.sort(key=get_sort_key, reverse=True)
    return matching_files[0]


def load_trajectory_file(filepath):
    """
    Lee un archivo de trayectoria (ArticularPoints o CartesianPoints).
    Devuelve (traj_robot, traj_gripper) en grados.
    """
    robot_rows = []
    gripper_rows = []

    with open(filepath, 'r') as f:
        lines = [line.strip() for line in f if line.strip() and not line.startswith('#')]

    if not lines:
        raise ValueError(f"El archivo {filepath} está vacío.")

    header = lines[0].lower()
    data_lines = lines[1:] if header in ['articular', 'cartesiano'] else lines

    # MGD_Node.cs en Unity calcula la pose cartesiana MGD usando d6 = 0.457 m (brida + pinza)
    params = ParamsFR5(d6=0.457) if HAS_FR5_IK else None
    semilla = np.radians([0, -90, 90, -90, -90, 90])

    for idx, line in enumerate(data_lines, 1):
        if ',' in line:
            parts = [x.strip() for x in line.split(',')]
        else:
            parts = line.split()

        if len(parts) >= 12:
            c0_5 = [float(p) for p in parts[0:6]]

            # Si el valor excede 180, es una coordenada cartesiana X,Y,Z en mm -> resolver IK
            if abs(c0_5[0]) > 180.0 or abs(c0_5[1]) > 180.0 or abs(c0_5[2]) > 180.0:
                if HAS_FR5_IK:
                    T = pose_a_T(c0_5[0] / 1000.0, c0_5[1] / 1000.0, c0_5[2] / 1000.0,
                                 math.radians(c0_5[3]), math.radians(c0_5[4]), math.radians(c0_5[5]))
                    sol = ik_mejor(T, q_semilla=semilla, params=params)
                    if sol:
                        q_deg = sol.grados()
                        semilla = sol.q
                    else:
                        q_deg = np.degrees(semilla)
                else:
                    q_deg = c0_5
            else:
                q_deg = c0_5

            # Pinza: columnas 8, 9, 10, 11 (shaft, wrist, jaw_dx, jaw_sx en grados)
            raw_pinza = [float(p) for p in parts[8:12]]
            pinza_deg = [raw_pinza[0], raw_pinza[1], raw_pinza[2], raw_pinza[3]]
            robot_rows.append(q_deg)
            gripper_rows.append(pinza_deg)

        elif len(parts) >= 6:
            # Archivo antiguo sin pinza
            robot_rows.append([float(p) for p in parts[0:6]])
            gripper_rows.append([0.0, 0.0, 0.0, 0.0])

    return np.array(robot_rows, dtype=np.float64), np.array(gripper_rows, dtype=np.float64)


class RobotEndowristSyncNode(Node):
    def __init__(self):
        super().__init__('robot_endowrist_sync')

        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # =========================
        # PUBLICADORES (AMBOS STRING EN GRADOS)
        # =========================
        self.pub_robot = self.create_publisher(String, '/current_joint_position', qos_profile)
        self.pub_gripper = self.create_publisher(String, '/current_endowrist_position', qos_profile)

        # =========================
        # BUSQUEDA DEL ARCHIVO MÁS RECIENTE
        # =========================
        script_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_dir = os.path.abspath(os.path.join(script_dir, '..'))
        search_dirs = [script_dir, workspace_dir, os.getcwd()]

        latest_file = find_latest_trajectory_file(search_dirs)

        if latest_file and os.path.exists(latest_file):
            self.get_logger().info(f'Cargando archivo más reciente: {latest_file}')
            self.traj_robot, self.traj_gripper = load_trajectory_file(latest_file)
            self.get_logger().info(f'Cargados {len(self.traj_robot)} frames de trayectoria desde {os.path.basename(latest_file)}')
        else:
            self.get_logger().error(f'No se encontró ningún archivo de trayectoria en: {search_dirs}')
            sys.exit(1)

        self.num_samples = len(self.traj_robot)
        self.index = 0

        # =========================
        # FRECUENCIA DE EMULACIÓN (40 Hz)
        # =========================
        timer_period = 1.0 / 40.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Emulador iniciado. Publicando en /current_joint_position y /current_endowrist_position (GRADOS)...')

    def timer_callback(self):
        # =====================================================
        # ROBOT (STRING EN GRADOS)
        # =====================================================
        robot_vals = self.traj_robot[self.index]
        msg_robot = String()
        msg_robot.data = ','.join([f'{val:.4f}' for val in robot_vals])

        # =====================================================
        # PINZA (STRING EN GRADOS: shaft,wrist,jaw_dx,jaw_sx)
        # =====================================================
        shaft_deg, wrist_deg, jaw_dx_deg, jaw_sx_deg = self.traj_gripper[self.index]

        # Aplicar límites suaves para protección en grados
        grip_deg = [
            max(min(shaft_deg, 180.0), -180.0),   # +/- 180°
            max(min(wrist_deg, 70.0), -70.0),     # +/- 70°
            max(min(jaw_dx_deg, 60.0), -60.0),    # +/- 60°
            max(min(jaw_sx_deg, 60.0), -60.0)     # +/- 60°
        ]

        msg_gripper = String()
        msg_gripper.data = ','.join([f'{val:.6f}' for val in grip_deg])

        # =====================================================
        # PUBLICACIÓN SINCRONIZADA
        # =====================================================
        self.pub_robot.publish(msg_robot)
        self.pub_gripper.publish(msg_gripper)

        # LOG DEBUG EN CONSOLA
        print(f'[{self.index + 1}/{self.num_samples}] Robot(deg): {msg_robot.data[:30]}... | Pinza(deg): {msg_gripper.data}')

        # REPETICIÓN EN BUCLE
        self.index += 1
        if self.index >= self.num_samples:
            self.index = 0
            print('--- BUCLE REINICIADO ---')


def main(args=None):
    rclpy.init(args=args)
    node = RobotEndowristSyncNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
