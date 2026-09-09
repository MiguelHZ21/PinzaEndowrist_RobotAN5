#!/usr/bin/env python3
# =========================================================================
# ROS2 Sync Robot (String) + EndoWrist (JointState)
# Python version of SimuladorCompleto.m (Updated with UR5 & PSM files)
# =========================================================================

import math
import os
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState


def load_ur5_data(filepath):
    """Carga q1 a q6 (grados) del archivo de UR5."""
    rows = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            # q1..q6 son los índices 2, 3, 4, 5, 6, 7
            q1_6 = [float(x) for x in parts[2:8]]
            rows.append(q1_6)
    return np.array(rows, dtype=np.float64)


def load_psm_data(filepath):
    """Carga q4, q5, q6, q7 (grados) del archivo de PSM."""
    rows = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            # q4..q7 (tool_roll, wrist_pitch, wrist_yaw, jaw) son los índices 8, 9, 10, 11
            q4_7 = [float(x) for x in parts[8:12]]
            rows.append(q4_7)
    return np.array(rows, dtype=np.float64)


class RobotEndowristSyncNode(Node):
    def __init__(self):
        super().__init__('robot_endowrist_sync')

        # Perfiles QoS (Transient Local para String, Volatile para JointState)
        qos_robot = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        qos_gripper = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # =========================
        # ROBOT (6 DOF) - STRING
        # =========================
        self.pub_robot = self.create_publisher(String, '/current_joint_position', qos_robot)

        # =========================
        # PINZA (4 DOF) - JOINTSTATE
        # =========================
        self.pub_gripper = self.create_publisher(JointState, '/endowrist', qos_gripper)

        # =========================
        # CARGA DE MATRICES
        # =========================
        script_dir = os.path.dirname(os.path.abspath(__file__))
        ur5_file = os.path.join(script_dir, 'posiciones_articulares_ur5_der_20260906_102446.txt')
        psm_file = os.path.join(script_dir, 'posiciones_articulares_psm_der_20260906_102446.txt')

        if os.path.exists(ur5_file):
            self.traj_robot = load_ur5_data(ur5_file)
            self.get_logger().info(f'Cargados {len(self.traj_robot)} frames de robot UR5 desde {os.path.basename(ur5_file)}')
        else:
            self.get_logger().error(f'Archivo UR5 no encontrado: {ur5_file}')
            sys.exit(1)

        if os.path.exists(psm_file):
            self.traj_gripper = load_psm_data(psm_file)
            self.get_logger().info(f'Cargados {len(self.traj_gripper)} frames de pinza PSM desde {os.path.basename(psm_file)}')
        else:
            self.get_logger().error(f'Archivo PSM no encontrado: {psm_file}')
            sys.exit(1)

        self.num_samples = min(len(self.traj_robot), len(self.traj_gripper))
        self.index = 0

        # =========================
        # FRECUENCIA (20 Hz)
        # =========================
        timer_period = 1.0 / 20.0  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Starting synchronized system...')
        print('Starting synchronized system...')

    def timer_callback(self):
        # =====================================================
        # ROBOT (STRING)
        # =====================================================
        robot_vals = self.traj_robot[self.index]
        msg_robot = String()
        msg_robot.data = ','.join([f'{val:.4f}' for val in robot_vals])

        # =====================================================
        # PINZA (JOINTSTATE)
        # =====================================================
        # Contenido de traj_gripper: [q4 (tool_roll), q5 (wrist_pitch), q6 (wrist_yaw), q7 (jaw)]
        q4, q5, q6, q7 = self.traj_gripper[self.index]

        # Convertir a radianes
        q4_rad = math.radians(q4)
        q5_rad = math.radians(q5)
        
        # Apertura simétrica de las mandíbulas (cada una abre la mitad del ángulo total q7)
        # jaw_half_rad = apertura por mandíbula (siempre >= 0 para evitar traslape)
        jaw_half_rad = math.radians(max(0.0, q7 / 2.0))

        grip_rad = [
            q4_rad,          # shaft (tool roll)
            q5_rad,          # wrist (wrist pitch)
            jaw_half_rad,    # jaw_dx (mandíbula derecha abre +jaw_half)
            jaw_half_rad     # jaw_sx (mandíbula izquierda abre +jaw_half)
        ]

        # Límites físicos EndoWrist
        grip_rad[0] = max(min(grip_rad[0], math.pi), -math.pi)
        grip_rad[1] = max(min(grip_rad[1], 1.57), -1.57)
        grip_rad[2] = max(min(grip_rad[2], 1.57), 0.0)      # Mínimo 0 para evitar que traspase
        grip_rad[3] = max(min(grip_rad[3], 1.57), 0.0)      # Mínimo 0 para evitar que traspase

        msg_gripper = JointState()
        msg_gripper.header.stamp = self.get_clock().now().to_msg()
        msg_gripper.header.frame_id = 'endowrist'
        msg_gripper.name = ['shaft', 'wrist', 'jaw_dx', 'jaw_sx']
        msg_gripper.position = grip_rad

        # =====================================================
        # PUBLICACIÓN SINCRONIZADA
        # =====================================================
        self.pub_robot.publish(msg_robot)
        self.pub_gripper.publish(msg_gripper)

        # DEBUG
        print(f'[{self.index + 1}/{self.num_samples}] Robot(String) + Gripper(JointState) enviados')

        # INDEX LOOP
        self.index += 1
        if self.index >= self.num_samples:
            self.index = 0
            print('--- LOOP RESTART ---')


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
