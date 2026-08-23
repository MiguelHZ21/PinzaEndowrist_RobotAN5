#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fr5_ik_node.py — Nodo ROS 2 que reemplaza a MATLAB en el lazo de cinematica inversa
====================================================================================

Habla los MISMOS topicos que el nodo de MATLAB, asi que es un reemplazo directo:

  ENTRADA
    /current_joint_position   std_msgs/String   "j1,...,j6"  en GRADOS
                              (o sensor_msgs/JointState en radianes, ver --joint-tipo)
                              -> es la posicion REAL del robot; se usa como semilla
    /input_cartesian_position std_msgs/String   "x,y,z,rx,ry,rz"  en mm y GRADOS

  SALIDA
    /output_joint_position    std_msgs/String   "j1,...,j6"  en GRADOS, 6 decimales
    /ik_status                std_msgs/String   diagnostico legible de cada pedido

DIFERENCIAS CLAVE CONTRA EL NODO DE MATLAB
------------------------------------------
  · Solucion CERRADA: no itera, no falla por convergencia, no depende de semilla.
  · Usa la posicion REAL del robot para elegir, entre las hasta 8 soluciones, la
    que menos mueve el brazo. MATLAB siempre arrancaba de [0,-90,90,-90,-90,90].
  · NUNCA publica NaN. Si no hay solucion valida publica el motivo en /ik_status
    y no publica nada en /output_joint_position. El silencio es la señal de error;
    un NaN con formato correcto no lo es.
  · Verifica los 6 limites del URDF (no solo j4/j5) y prueba las representaciones
    +/-360 deg que j2 y j4 admiten por tener rango de 350 deg.
  · Rechaza saltos de rama mayores a --salto-max grados respecto de donde esta el
    robot (red de seguridad contra el latigazo entre waypoints vecinos).

USO
---
    # con el formato de topicos que ya usa tu sistema
    python3 fr5_ik_node.py

    # si tu robot publica sensor_msgs/JointState en radianes
    python3 fr5_ik_node.py --joint-topico /joint_states --joint-tipo jointstate

    # si el TCP del controlador no es el tool_Link del URDF (0.267 m)
    python3 fr5_ik_node.py --d6 0.100

ANTES DE USARLO CON EL ROBOT: corré `fr5_kinematics.py --verificar` con una lectura
real simultanea de juntas y TCP. Si el modelo no coincide, nada de esto sirve.
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import time

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from std_msgs.msg import String
except ImportError:  # pragma: no cover
    sys.exit("Falta ROS 2 (rclpy). Hacé:  source /opt/ros/humble/setup.bash")

from fr5_kinematics import (FR5, LIMITES, ParamsFR5, ik, ik_mejor, pose_a_T)


def _qos_comandos(profundidad: int = 10) -> QoSProfile:
    """
    Reliable + VOLATILE a proposito.

    El nodo de MATLAB publicaba comandos con Durability=transientlocal, o sea
    LATCHED: cualquier suscriptor que se conectara despues — o el driver del
    robot al reiniciarse — recibia el ULTIMO comando guardado y el robot se podia
    mover solo. Para un canal de comandos eso no va.
    """
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=profundidad,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class NodoIK(Node):
    def __init__(self, args):
        super().__init__("fr5_ik_node")
        self.params = ParamsFR5(d6=args.d6)
        self.salto_max = (None if args.salto_max is None
                          else math.radians(args.salto_max))
        self.q_actual: np.ndarray | None = None
        self.edad_q = None

        qos = _qos_comandos()
        self.pub_q = self.create_publisher(String, args.salida_topico, qos)
        self.pub_estado = self.create_publisher(String, "/ik_status", qos)
        self.pub_api = self.create_publisher(String, "/api_command", qos)

        self.create_subscription(String, args.pose_topico, self.cb_pose, qos)
        self.create_subscription(String, "/input_cartesian_path", self.cb_path, qos)

        self.processing_commands = False

        if args.joint_tipo == "string":
            self.create_subscription(String, args.joint_topico,
                                     self.cb_juntas_string, qos)
        else:
            from sensor_msgs.msg import JointState
            self.create_subscription(JointState, args.joint_topico,
                                     self.cb_juntas_jointstate, qos)

        self.get_logger().info("fr5_ik esperando mensajes...")

    # ---------------- posicion actual del robot real ----------------------
    def cb_juntas_string(self, msg: String) -> None:
        try:
            v = [float(x) for x in msg.data.split(",")]
        except ValueError:
            self.get_logger().warn(f"juntas ilegibles: {msg.data!r}")
            return
        if len(v) != 6 or not all(math.isfinite(x) for x in v):
            self.get_logger().warn(f"juntas invalidas: {msg.data!r}")
            return
        self.q_actual = np.radians(v)
        self.edad_q = self.get_clock().now()

    def cb_juntas_jointstate(self, msg) -> None:
        # Ordena por nombre j1..j6: no confies en el orden de llegada.
        try:
            idx = [list(msg.name).index(f"j{i}") for i in range(1, 7)]
        except ValueError:
            if len(msg.position) < 6:
                return
            idx = list(range(6))
        q = np.array([msg.position[i] for i in idx], dtype=float)
        if not np.all(np.isfinite(q)):
            return
        self.q_actual = q
        self.edad_q = self.get_clock().now()

    # ---------------- pedido de cinematica inversa ------------------------
    def _estado(self, texto: str, nivel: str = "info") -> None:
        self.pub_estado.publish(String(data=texto))
        if nivel == "error":
            self.get_logger().error(texto)
        elif nivel in ("warn", "warning"):
            self.get_logger().warning(texto)
        else:
            self.get_logger().info(texto)

    def cb_pose(self, msg: String) -> None:
        try:
            v = [float(x) for x in msg.data.split(",")]
        except ValueError:
            self._estado(f"ERROR pose ilegible: {msg.data!r}", "error")
            return
        if len(v) != 6 or not all(math.isfinite(x) for x in v):
            self._estado(f"ERROR pose invalida (esperaba 6 numeros finitos): "
                         f"{msg.data!r}", "error")
            return

        x, y, z, rx, ry, rz = v
        T = pose_a_T(x / 1000.0, y / 1000.0, z / 1000.0,
                     math.radians(rx), math.radians(ry), math.radians(rz))

        semilla = self.q_actual
        if semilla is None:
            semilla = np.radians([0.0, -90.0, 90.0, -90.0, -90.0, 90.0])
            self._estado("AVISO: Posicion real no recibida. Usando postura Home [0,-90,90,-90,-90,90]", "warn")

        sol = ik_mejor(T, q_semilla=semilla,
                       salto_max=self.salto_max if self.q_actual is not None else None,
                       params=self.params)

        if sol is None:
            todas = ik(T, q_semilla=semilla, params=self.params)
            if not todas:
                self._estado(
                    f"SIN SOLUCION: pose inalcanzable o fuera de limites "
                    f"articulares -> {msg.data}", "error")
            else:
                d = np.degrees(np.max(np.abs(todas[0].q - semilla)))
                self._estado(
                    f"RECHAZADA: hay {len(todas)} solucion(es) pero la mejor "
                    f"exige mover una junta {d:.1f} deg (limite "
                    f"{math.degrees(self.salto_max):.0f}) -> {msg.data}", "error")
            return

        grados = sol.grados()
        self.pub_q.publish(String(data=",".join(f"{g:.6f}" for g in grados)))

        aviso = ""
        if sol.manipulabilidad < 0.01:
            aviso = f" | AVISO: Cerca de singularidad (sigma_min={sol.manipulabilidad:.4f})"
        if sol.singular:
            aviso += " | AVISO: Muñeca singular"
            
        q_str = f"J1:{grados[0]:.2f}, J2:{grados[1]:.2f}, J3:{grados[2]:.2f}, J4:{grados[3]:.2f}, J5:{grados[4]:.2f}, J6:{grados[5]:.2f}"
        self._estado(f"Cinemática calculada: {q_str}{aviso}")

    # ---------------- lectura y ejecucion de archivos TXT de trayectoria ------------------------
    def cb_path(self, msg: String) -> None:
        if self.processing_commands:
            self._estado("Ya se están procesando comandos. Ignorando nuevo archivo.", "warn")
            return

        file_path = msg.data.strip()
        if not os.path.exists(file_path):
            self._estado(f"Archivo no encontrado: {file_path}", "error")
            return

        self.processing_commands = True
        try:
            self._procesar_archivo_trayectoria(file_path)
        except Exception as e:
            self._estado(f"Error procesando trayectoria {file_path}: {e}", "error")
        finally:
            self.processing_commands = False

    def _procesar_archivo_trayectoria(self, file_path: str) -> None:
        with open(file_path, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f if line.strip()]

        if not lines:
            self._estado(f"Archivo vacío: {file_path}", "error")
            return

        header = lines[0].lower()
        data_lines = lines[1:]

        x_lim = (-830.0, -320.0)
        y_lim = (-500.0, 500.0)
        z_lim = (0.0, 720.0)
        rx_lim_1 = (-180.0, -20.0)
        rx_lim_2 = (20.0, 180.0)

        valid_rows = []
        valid_joints = []
        semilla_actual = self.q_actual if self.q_actual is not None else np.radians([0.0, -90.0, 90.0, -90.0, -90.0, 90.0])

        for idx, line in enumerate(data_lines, 1):
            parts = [x.strip() for x in line.split(",")]
            if len(parts) < 8:
                self._estado(f"Fila {idx} invalida (menos de 8 columnas): {line}", "error")
                return

            try:
                # Tomamos estrictamente las 6 primeras columnas para las coordenadas del robot (X,Y,Z,Rx,Ry,Rz)
                x, y, z, rx, ry, rz = [float(p) for p in parts[:6]]
                velocidad = float(parts[6])
                control = float(parts[7])
            except ValueError:
                self._estado(f"Error al parsear numeros en fila {idx}: {line}", "error")
                return

            # Validar limites cartesianos del robot
            if not (x_lim[0] <= x <= x_lim[1] and y_lim[0] <= y <= y_lim[1] and z_lim[0] <= z <= z_lim[1]):
                self._estado(f"Posicion fuera de limites en fila {idx}: X={x}, Y={y}, Z={z}", "error")
                return

            if not ((rx_lim_1[0] <= rx <= rx_lim_1[1]) or (rx_lim_2[0] <= rx <= rx_lim_2[1])):
                self._estado(f"Orientacion Rx fuera de limites en fila {idx}: Rx={rx}", "error")
                return

            # Resolver IK
            T = pose_a_T(x / 1000.0, y / 1000.0, z / 1000.0,
                         math.radians(rx), math.radians(ry), math.radians(rz))
            sol = ik_mejor(T, q_semilla=semilla_actual, params=self.params)

            if sol is None:
                self._estado(f"Sin solucion IK para fila {idx}: {x},{y},{z},{rx},{ry},{rz}", "error")
                return

            q_deg = sol.grados()
            j4, j5 = q_deg[3], q_deg[4]

            # Validacion J4/J5 igual a MATLAB
            if not ((0 <= j4 <= 90 and -90 <= j5 <= 90) or
                    (-180 <= j4 <= 0) or
                    (-267 <= j4 <= -180 and -90 <= j5 <= 90)):
                self._estado(f"Valores J4/J5 fuera de limites en fila {idx}: J4={j4:.2f}, J5={j5:.2f}", "error")
                return

            semilla_actual = sol.q
            valid_rows.append((x, y, z, rx, ry, rz, velocidad, control))
            valid_joints.append((q_deg, velocidad, control))

        self._estado(f"Archivo {file_path} validado correctamente ({len(valid_rows)} puntos). Header: {header}")

        save_dir = "/home/miguel/Interfaz AppDesigner AN5"
        os.makedirs(save_dir, exist_ok=True)

        if header == "cartesiano":
            # 1. Escribir python_position.txt con las 8 primeras columnas (X,Y,Z,Rx,Ry,Rz,speed,control)
            py_pos_file = os.path.join(save_dir, "python_position.txt")
            with open(py_pos_file, "w", encoding="utf-8") as f:
                for row in valid_rows:
                    f.write(f"{row[0]},{row[1]},{row[2]},{row[3]},{row[4]},{row[5]},{row[6]},{row[7]}\n")

            # 2. Escribir joint_python_position.txt con los angulos articulares
            joint_py_file = os.path.join(save_dir, "joint_python_position.txt")
            with open(joint_py_file, "w", encoding="utf-8") as f:
                for (q_deg, vel, ctrl) in valid_joints:
                    q_str = ",".join(f"{g:.17f}" for g in q_deg)
                    f.write(f"{q_str},{vel:.2f},{ctrl:.2f}\n")

            self._estado("Archivos python_position.txt y joint_python_position.txt generados. Ejecutando MoveL.py...")

            # 3. Lanzar MoveL.py
            script_path = "/home/miguel/ros2_ws/src/code/code/MoveL.py"
            cmd = [sys.executable, script_path]
            proc = subprocess.Popen(cmd)
            self._estado(f"MoveL.py lanzado (PID: {proc.pid})")

        elif header == "articular":
            # Enviar comandos articulares al robot por /api_command
            all_control_zero = all(row[7] == 0 for row in valid_rows)
            if all_control_zero:
                self.pub_api.publish(String(data="SplineStart()"))
                time.sleep(0.05)

            max_index = 5
            for i, (q_deg, vel, ctrl) in enumerate(valid_joints):
                index = (i % max_index) + 1
                cmd_jnt = f"JNTPoint({index},{q_deg[0]:.2f},{q_deg[1]:.2f},{q_deg[2]:.2f},{q_deg[3]:.2f},{q_deg[4]:.2f},{q_deg[5]:.2f})"
                self.pub_api.publish(String(data=cmd_jnt))
                time.sleep(0.01)

                if ctrl != 0:
                    cmd_mov = f"MoveJ(JNT{index},{vel:.2f})"
                    self.pub_api.publish(String(data=cmd_mov))
                    time.sleep(ctrl)
                else:
                    cmd_mov = f"SplinePTP(JNT{index},{vel:.2f})"
                    self.pub_api.publish(String(data=cmd_mov))
                    time.sleep(0.05)

            if all_control_zero:
                self.pub_api.publish(String(data="SplineEnd()"))

            self._estado("Comandos de trayectoria articular enviados correctamente a /api_command.")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint-topico", default="/current_joint_position")
    ap.add_argument("--joint-tipo", choices=("string", "jointstate"),
                    default="string",
                    help="string = std_msgs/String en grados (como MATLAB); "
                         "jointstate = sensor_msgs/JointState en radianes")
    ap.add_argument("--pose-topico", default="/input_cartesian_position")
    ap.add_argument("--salida-topico", default="/output_joint_position")
    ap.add_argument("--d6", type=float, default=FR5.d6,
                    help=f"offset del TCP sobre z6 en metros (def. {FR5.d6})")
    ap.add_argument("--salto-max", type=float, default=90.0,
                    help="rechaza soluciones que muevan una junta mas que esto "
                         "(grados). 0 o negativo = desactivar")
    args = ap.parse_args()
    if args.salto_max is not None and args.salto_max <= 0:
        args.salto_max = None

    rclpy.init()
    nodo = NodoIK(args)
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
