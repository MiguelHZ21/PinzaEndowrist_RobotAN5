#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ***************************************************************
# Autores:    Miguel Hernandez (miguelhernandez@unicauca.edu.co)
#             Cristian Gonzalez (cgonzalezg@unicauca.edu.co)
# ***************************************************************
"""
Run_node.py — Nodo ROS 2 principal de cinematica inversa y gestion de trayectorias
==================================================================================

Habla los topicos del sistema para el Robot AN5 (FR5) y la Pinza EndoWrist Xi:

  ENTRADA
    /current_joint_position     std_msgs/String   "j1,...,j6" en GRADOS
                                -> posicion REAL del robot; se usa como semilla y verificacion
    /current_endowrist_position std_msgs/String   "shaft,wrist,jaw_dx,jaw_sx" en GRADOS
                                -> posicion REAL de la pinza (servomotores)
    /input_cartesian_position   std_msgs/String   "x,y,z,rx,ry,rz" en mm y GRADOS

  SALIDA
    /output_joint_position      std_msgs/String   "j1,...,j6" en GRADOS, 17 decimales
    /endowrist_command          std_msgs/String   "shaft,wrist,jaw_dx,jaw_sx" en GRADOS
    /ik_status                  std_msgs/String   diagnostico legible de cada pedido

MODOS DE OPERACION (--modo con_pinza | sin_pinza)
-------------------------------------------------
  · con_pinza: Verifica que TANTO el robot AN5 como la pinza EndoWrist Xi lleguen
               al setpoint antes de pasar al siguiente paso.
  · sin_pinza: Verifica UNICAMENTE que el robot AN5 llegue al setpoint.

USO
---
    # Con menu interactivo al iniciar para elegir con_pinza o sin_pinza:
    python3 Run_node.py

    # Especificando el modo directamente por linea de comandos:
    python3 Run_node.py --modo con_pinza
    python3 Run_node.py --modo sin_pinza
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import ReentrantCallbackGroup
    from std_msgs.msg import String
except ImportError:  # pragma: no cover
    sys.exit("Falta ROS 2 (rclpy). Hacé:  source /opt/ros/humble/setup.bash")

from fr5_kinematics import (FR5, LIMITES, ParamsFR5, ik, ik_mejor, pose_a_T)
from fr5_trajectory_manager import (
    TOPICO_PINZA_COMANDO,
    TOPICO_PINZA_POSICION_REAL,
    GestorTrayectoria,
    seleccionar_modo_operacion,
)


def _qos_comandos(profundidad: int = 10) -> QoSProfile:
    """
    Perfil QoS para canales de comando y estado en el nodo IK.
    Se usa DurabilityPolicy.VOLATILE para garantizar compatibilidad total en ROS 2
    tanto con publicadores VOLATILE (publisher_subscriber.py, rosbridge_websocket / Unity)
    como con publicadores TRANSIENT_LOCAL (simulador_completo.py).
    """
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=profundidad,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class NodoIK(Node):
    def __init__(self, args, modo_operacion: str = "con_pinza"):
        super().__init__("fr5_ik_node")
        self.params = ParamsFR5(d6=args.d6)
        self.salto_max = (None if args.salto_max is None
                          else math.radians(args.salto_max))
        self.modo_operacion = modo_operacion

        # Estado actual del robot (radianes) y de la pinza (grados)
        self.q_actual: np.ndarray | None = None
        self.edad_q = None
        self.pinza_actual_deg: np.ndarray | None = None
        self.edad_pinza = None

        # Variables de espera de llegada a setpoint (Robot y Pinza)
        self.target_joint_position: np.ndarray | None = None
        self.target_pinza_position_deg: np.ndarray | None = None
        self.tolerance_robot_deg: float = 0.5
        self.tolerance_pinza_deg: float = 2.0
        self.check_pinza_in_wait: bool = False
        self.is_waiting_for_position: bool = False

        # Gestor de trayectorias (con_pinza / sin_pinza)
        self.gestor_trayectoria = GestorTrayectoria(self, modo=self.modo_operacion)

        cbg = ReentrantCallbackGroup()
        qos = _qos_comandos()

        # Publicadores
        self.pub_q = self.create_publisher(String, args.salida_topico, qos)
        self.pub_estado = self.create_publisher(String, "/ik_status", qos)
        self.pub_api = self.create_publisher(String, "/api_command", qos)
        self.pub_endowrist = self.create_publisher(String, TOPICO_PINZA_COMANDO, qos)
        self.pub_preview = self.create_publisher(String, "/preview_joint_position", qos)

        # Suscriptores de comandos y rutas TXT
        self.create_subscription(String, args.pose_topico, self.cb_pose, qos, callback_group=cbg)
        self.create_subscription(String, "/input_cartesian_path", self.cb_path, qos, callback_group=cbg)
        self.create_subscription(String, "/input_cartesian_path_preview", self.cb_path_preview, qos, callback_group=cbg)

        # Suscriptor a la posición real de la pinza (/current_endowrist_position en GRADOS)
        self.create_subscription(
            String,
            args.pinza_topico,
            self.cb_pinza_string,
            qos,
            callback_group=cbg,
        )

        self.processing_commands = False

        if args.joint_tipo == "string":
            self.create_subscription(String, args.joint_topico,
                                     self.cb_juntas_string, qos, callback_group=cbg)
        else:
            from sensor_msgs.msg import JointState
            self.create_subscription(JointState, args.joint_topico,
                                     self.cb_juntas_jointstate, qos, callback_group=cbg)

        self.get_logger().info(
            f"Nodo principal iniciado en modo '{self.modo_operacion.upper()}'. "
            f"Tópicos pinza: lectura='{args.pinza_topico}', comando='{TOPICO_PINZA_COMANDO}'."
        )

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
        self._check_wait_condition()

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
        self._check_wait_condition()

    # ---------------- posicion actual de la pinza real (servos) -----------
    def cb_pinza_string(self, msg: String) -> None:
        """
        Lee la posicion real de la pinza desde /current_endowrist_position
        en formato 'shaft,wrist,jaw_dx,jaw_sx' (en GRADOS).
        """
        try:
            v = [float(x.strip()) for x in msg.data.split(",")]
        except ValueError:
            self.get_logger().warn(f"Posicion de pinza ilegible: {msg.data!r}")
            return
        if len(v) < 4 or not all(math.isfinite(x) for x in v[:4]):
            self.get_logger().warn(f"Posicion de pinza invalida (esperaba 4 valores): {msg.data!r}")
            return
        self.pinza_actual_deg = np.array(v[:4], dtype=float)
        self.edad_pinza = self.get_clock().now()
        self._check_wait_condition()

    # ---------------- verificacion de llegada a setpoint ------------------
    def _check_wait_condition(self) -> None:
        if not self.is_waiting_for_position or self.target_joint_position is None:
            return
        if self.q_actual is None:
            return

        diff_robot_deg = np.degrees(np.abs(self.q_actual - self.target_joint_position))
        robot_ok = bool(np.all(diff_robot_deg <= self.tolerance_robot_deg))

        if not robot_ok:
            return

        if self.check_pinza_in_wait and self.target_pinza_position_deg is not None:
            if self.pinza_actual_deg is None:
                return
            diff_pinza_deg = np.abs(self.pinza_actual_deg - self.target_pinza_position_deg)
            pinza_ok = bool(np.all(diff_pinza_deg <= self.tolerance_pinza_deg))
            if not pinza_ok:
                return
            self._estado("El robot y la pinza han alcanzado la posicion objetivo.")
            self.is_waiting_for_position = False
        else:
            self._estado("El robot ha alcanzado la posicion objetivo.")
            self.is_waiting_for_position = False

    def waitForSetpointReached(
        self,
        target_joint_rad,
        target_pinza_deg=None,
        check_pinza: bool = False,
        tolerance_robot_deg: float = 0.5,
        tolerance_pinza_deg: float = 2.0,
        timeout_sec: float = 60.0,
    ) -> None:
        """
        Espera hasta que el robot (y opcionalmente la pinza cuando estamos en modo con_pinza)
        alcance la posicion objetivo dentro de la tolerancia indicada.
        """
        self.target_joint_position = np.array(target_joint_rad, dtype=float)
        self.tolerance_robot_deg = tolerance_robot_deg
        self.tolerance_pinza_deg = tolerance_pinza_deg
        self.check_pinza_in_wait = bool(check_pinza and target_pinza_deg is not None)
        self.target_pinza_position_deg = (
            np.array(target_pinza_deg[:4], dtype=float)
            if self.check_pinza_in_wait
            else None
        )
        self.is_waiting_for_position = True

        # Evaluar inmediatamente por si ya se encuentra en la posicion objetivo
        self._check_wait_condition()

        start_time = time.time()
        while rclpy.ok() and self.is_waiting_for_position:
            if timeout_sec > 0 and (time.time() - start_time) > timeout_sec:
                if self.check_pinza_in_wait and self.pinza_actual_deg is None:
                    self._estado(
                        "AVISO: Tiempo de espera agotado esperando lectura de /current_endowrist_position. Continuando...",
                        "warn",
                    )
                else:
                    self._estado(
                        "AVISO: Tiempo de espera agotado esperando llegada al setpoint. Continuando...",
                        "warn",
                    )
                self.is_waiting_for_position = False
                break
            time.sleep(0.05)

    def waitForRobotToReachPosition(self, target_position_rad, tolerance_deg: float = 0.5) -> None:
        """Compatibilidad hacia atras para esperar solo al robot."""
        self.waitForSetpointReached(
            target_joint_rad=target_position_rad,
            target_pinza_deg=None,
            check_pinza=False,
            tolerance_robot_deg=tolerance_deg,
        )

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
            self.pub_q.publish(String(data="NaN,NaN,NaN,NaN,NaN,NaN"))
            return
        if len(v) != 6 or not all(math.isfinite(x) for x in v):
            self._estado(f"ERROR pose invalida (esperaba 6 numeros finitos): "
                         f"{msg.data!r}", "error")
            self.pub_q.publish(String(data="NaN,NaN,NaN,NaN,NaN,NaN"))
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
            self.pub_q.publish(String(data="NaN,NaN,NaN,NaN,NaN,NaN"))
            return

        grados = sol.grados()
        self.pub_q.publish(String(data=",".join(f"{g:.17f}" for g in grados)))

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
            self.gestor_trayectoria.procesar_archivo_trayectoria(file_path)
        except Exception as e:
            self._estado(f"Error procesando trayectoria {file_path}: {e}", "error")
        finally:
            self.processing_commands = False

    # ---------------- preview visual de trayectoria (solo Unity, NO robot fisico) ------
    def cb_path_preview(self, msg: String) -> None:
        if self.processing_commands:
            self._estado("Ya se están procesando comandos. Ignorando preview.", "warn")
            return

        file_path = msg.data.strip()
        if not os.path.exists(file_path):
            self._estado(f"Archivo no encontrado: {file_path}", "error")
            return

        self.processing_commands = True
        try:
            self.gestor_trayectoria.reproducir_preview(file_path)
        except Exception as e:
            self._estado(f"Error en preview {file_path}: {e}", "error")
        finally:
            self.processing_commands = False


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--modo", choices=("con_pinza", "sin_pinza"), default=None,
                    help="Modo de operacion: 'con_pinza' (verifica robot y pinza) "
                         "o 'sin_pinza' (verifica solo el robot). Si se omite, pregunta al iniciar.")
    ap.add_argument("--joint-topico", default="/current_joint_position")
    ap.add_argument("--joint-tipo", choices=("string", "jointstate"),
                    default="string",
                    help="string = std_msgs/String en grados (como MATLAB); "
                         "jointstate = sensor_msgs/JointState en radianes")
    ap.add_argument("--pinza-topico", default=TOPICO_PINZA_POSICION_REAL,
                    help=f"Topico de posicion real de la pinza en grados (def. {TOPICO_PINZA_POSICION_REAL})")
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

    modo_seleccionado = seleccionar_modo_operacion(args.modo)

    rclpy.init()
    nodo = NodoIK(args, modo_operacion=modo_seleccionado)
    executor = MultiThreadedExecutor()
    executor.add_node(nodo)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
