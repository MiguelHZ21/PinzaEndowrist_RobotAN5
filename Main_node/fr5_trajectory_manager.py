#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ***************************************************************
# Autores:    Miguel Hernandez (miguelhernandez@unicauca.edu.co)
#             Cristian Gonzalez (cgonzalezg@unicauca.edu.co)
# ***************************************************************
"""
fr5_trajectory_manager.py — Gestor de ejecucion de trayectorias (AN5 + EndoWrist Xi)
====================================================================================

Gestiona la validacion, generacion de archivos intermedios y ejecucion sincronizada
de trayectorias tanto en modo CON PINZA (`con_pinza`) como SIN PINZA (`sin_pinza`).

  · Modo `con_pinza`:
      En cada punto se envia el movimiento del robot y el comando de la pinza
      (`/endowrist_command`), y se verifica en lazo cerrado que TANTO el robot
      (`/current_joint_position`) COMO la pinza (`/current_endowrist_position`)
      hayan alcanzado su setpoint antes de avanzar al siguiente paso.

  · Modo `sin_pinza`:
      Solo se verifica que el robot AN5 alcance su setpoint articular antes de
      avanzar al siguiente paso (ignorando la espera de los servomotores).

Trama de 12 columnas en archivos TXT:
    X,Y,Z,Rx,Ry,Rz,speed,delay,shaft,wrist,jaw_dx,jaw_sx
"""

from __future__ import annotations

import math
import os
import subprocess
import time
from typing import TYPE_CHECKING

import numpy as np
import rclpy
from std_msgs.msg import String

from fr5_kinematics import ik_mejor, pose_a_T

if TYPE_CHECKING:
    from Run_node import NodoIK


# =============================================================================
# CONSTANTES DE TOPICOS UNIFICADOS DE LA PINZA
# =============================================================================
TOPICO_PINZA_POSICION_REAL = "/current_endowrist_position"
TOPICO_PINZA_COMANDO = "/endowrist_command"


def seleccionar_modo_operacion(modo_arg: str | None) -> str:
    """
    Permite seleccionar el modo de operacion ('con_pinza' o 'sin_pinza')
    ya sea por argumento de linea de comandos o mediante menu interactivo.
    """
    if modo_arg in ("con_pinza", "sin_pinza"):
        return modo_arg

    print("\n" + "=" * 65)
    print("   SELECCIÓN DE MODO DE OPERACIÓN — ROBOT AN5 + ENDOWRIST XI")
    print("=" * 65)
    print("  [1] CON PINZA (Verifica llegada de Robot AN5 + Servos Pinza)")
    print("  [2] SIN PINZA (Verifica únicamente llegada de Robot AN5)")
    print("=" * 65)

    while True:
        try:
            opcion = input("Seleccione modo [1/2] (Enter por defecto = 1 CON PINZA): ").strip().lower()
        except EOFError:
            return "con_pinza"

        if opcion in ("", "1", "con_pinza", "con"):
            return "con_pinza"
        if opcion in ("2", "sin_pinza", "sin"):
            return "sin_pinza"
        print("Opción no válida. Ingrese 1 (con_pinza) o 2 (sin_pinza).")


class GestorTrayectoria:
    """
    Coordina la validacion y ejecucion de trayectorias (articulares, cartesianas
    y previews) en conjunto con el nodo principal `NodoIK`.
    """

    def __init__(self, nodo: "NodoIK", modo: str = "con_pinza") -> None:
        self.nodo = nodo
        self.modo = modo  # "con_pinza" o "sin_pinza"
        self.workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

    @property
    def con_pinza(self) -> bool:
        return self.modo == "con_pinza"

    # -------------------------------------------------------------------------
    # 1. Validación y parseo unificado de archivos de trayectoria (12 columnas)
    # -------------------------------------------------------------------------
    def cargar_y_validar_txt(self, file_path: str, es_preview: bool = False):
        """
        Lee y valida un archivo TXT de trayectoria (articular o cartesiano).
        Devuelve (header, valid_rows, valid_joints, valid_pinza) o None si falla.
        """
        with open(file_path, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f if line.strip()]

        if not lines:
            self.nodo._estado(f"Archivo vacío: {file_path}", "error")
            return None

        header = lines[0].lower()
        data_lines = lines[1:]

        x_lim = (-830.0, -320.0)
        y_lim = (-500.0, 500.0)
        z_lim = (0.0, 720.0)
        rx_lim_1 = (-180.0, -20.0)
        rx_lim_2 = (20.0, 180.0)

        valid_rows = []
        valid_joints = []
        valid_pinza = []
        semilla_actual = (
            self.nodo.q_actual
            if self.nodo.q_actual is not None
            else np.radians([0.0, -90.0, 90.0, -90.0, -90.0, 90.0])
        )
        prefijo = "preview " if es_preview else ""

        for idx, line in enumerate(data_lines, 1):
            parts = [x.strip() for x in line.split(",")]
            if len(parts) < 8:
                self.nodo._estado(f"Fila {idx} invalida (menos de 8 columnas): {line}", "error")
                return None

            try:
                # Columnas 0-5: X, Y, Z, Rx, Ry, Rz
                x, y, z, rx, ry, rz = [float(p) for p in parts[:6]]
                # Columna 6: speed | Columna 7: delay/control
                velocidad = float(parts[6])
                control = float(parts[7])
                # Columnas 8-11: shaft, wrist, jaw_dx, jaw_sx (en GRADOS)
                if len(parts) >= 12:
                    pinza = [float(p) for p in parts[8:12]]
                else:
                    pinza = [0.0, 0.0, 0.0, 0.0]
            except ValueError:
                self.nodo._estado(f"Error al parsear numeros en fila {idx}: {line}", "error")
                return None

            # Validar límites cartesianos (X, Y, Z)
            if not (x_lim[0] <= x <= x_lim[1] and y_lim[0] <= y <= y_lim[1] and z_lim[0] <= z <= z_lim[1]):
                self.nodo._estado(f"Posicion fuera de limites en fila {idx}: X={x}, Y={y}, Z={z}", "error")
                return None

            # Validar orientación Rx (protección de muñeca [-180, -20] U [20, 180] igual al proyecto base)
            if not ((rx_lim_1[0] <= rx <= rx_lim_1[1]) or (rx_lim_2[0] <= rx <= rx_lim_2[1])):
                self.nodo._estado(
                    f"Orientacion Rx fuera de limites en fila {idx}: Rx={rx} "
                    f"(rango permitido: [-180, -20] o [20, 180])",
                    "error",
                )
                return None

            # ── SEGURIDAD: Salto cartesiano y de orientación entre puntos consecutivos ──
            if header == "cartesiano" and valid_rows:
                MAX_SALTO_MM = 250.0
                MAX_ORI_DEG = 45.0
                xp, yp, zp, rxp, ryp, rzp = valid_rows[-1][:6]
                dist_mm = math.sqrt((x - xp) ** 2 + (y - yp) ** 2 + (z - zp) ** 2)
                if dist_mm > MAX_SALTO_MM:
                    self.nodo._estado(
                        f"BLOQUEO SEGURIDAD {prefijo}fila {idx}: salto XYZ de {dist_mm:.1f} mm "
                        f"supera el limite de {MAX_SALTO_MM} mm. Trayectoria rechazada.",
                        "error",
                    )
                    return None

                # Distancia angular mínima en [-180°, 180°] para evitar bloqueos falsos en ±180°
                delta_rx = abs((rx - rxp + 180.0) % 360.0 - 180.0)
                delta_ry = abs((ry - ryp + 180.0) % 360.0 - 180.0)
                delta_rz = abs((rz - rzp + 180.0) % 360.0 - 180.0)
                max_ori_delta = max(delta_rx, delta_ry, delta_rz)
                if max_ori_delta > MAX_ORI_DEG:
                    self.nodo._estado(
                        f"BLOQUEO SEGURIDAD {prefijo}fila {idx}: variacion de orientacion de {max_ori_delta:.1f}° "
                        f"supera el limite de {MAX_ORI_DEG}°. Trayectoria rechazada.",
                        "error",
                    )
                    return None

            # Resolver IK
            T = pose_a_T(
                x / 1000.0,
                y / 1000.0,
                z / 1000.0,
                math.radians(rx),
                math.radians(ry),
                math.radians(rz),
            )
            sol = ik_mejor(
                T,
                q_semilla=semilla_actual,
                salto_max=None if idx == 1 else self.nodo.salto_max,
                params=self.nodo.params,
            )

            if sol is None:
                self.nodo._estado(f"Sin solucion IK para fila {idx}: {x},{y},{z},{rx},{ry},{rz}", "error")
                return None

            q_deg = sol.grados()
            j4, j5 = q_deg[3], q_deg[4]

            if not es_preview:
                # Validacion J4/J5 igual a MATLAB
                if not (
                    (0 <= j4 <= 90 and -90 <= j5 <= 90)
                    or (-180 <= j4 <= 0)
                    or (-267 <= j4 <= -180 and -90 <= j5 <= 90)
                ):
                    self.nodo._estado(
                        f"Valores J4/J5 fuera de limites en fila {idx}: J4={j4:.2f}, J5={j5:.2f}",
                        "error",
                    )
                    return None

            # ── SEGURIDAD: Salto articular entre puntos consecutivos ──
            if header == "articular" and valid_joints:
                MAX_JOINT_JUMP_DEG = 45.0
                prev_q = valid_joints[-1][0]
                max_jump = np.max(np.abs(np.array(q_deg) - np.array(prev_q)))
                if max_jump > MAX_JOINT_JUMP_DEG:
                    self.nodo._estado(
                        f"BLOQUEO SEGURIDAD {prefijo}fila {idx}: salto articular de {max_jump:.1f}° "
                        f"supera el limite de {MAX_JOINT_JUMP_DEG}°. Trayectoria rechazada.",
                        "error",
                    )
                    return None

            semilla_actual = sol.q
            valid_rows.append((x, y, z, rx, ry, rz, velocidad, control))
            valid_joints.append((q_deg, velocidad, control))
            valid_pinza.append(pinza)

        return header, valid_rows, valid_joints, valid_pinza

    # -------------------------------------------------------------------------
    # 2. Ejecución de trayectoria en el robot físico (y pinza según modo)
    # -------------------------------------------------------------------------
    def procesar_archivo_trayectoria(self, file_path: str) -> None:
        resultado = self.cargar_y_validar_txt(file_path, es_preview=False)
        if resultado is None:
            return

        header, valid_rows, valid_joints, valid_pinza = resultado
        self.nodo._estado(
            f"Archivo {file_path} validado ({len(valid_rows)} puntos). "
            f"Header: {header} | Modo: {self.modo.upper()}"
        )

        os.makedirs(self.workspace_root, exist_ok=True)

        if header == "cartesiano":
            self._ejecutar_trayectoria_cartesiana(valid_rows, valid_joints, valid_pinza)
        elif header == "articular":
            self._ejecutar_trayectoria_articular(valid_rows, valid_joints, valid_pinza)
        else:
            self.nodo._estado(f"Encabezado desconocido '{header}' en {file_path}", "error")

    def _ejecutar_trayectoria_cartesiana(self, valid_rows, valid_joints, valid_pinza) -> None:
        # 1. Escribir python_position.txt con las 8 primeras columnas (X,Y,Z,Rx,Ry,Rz,speed,control)
        py_pos_file = os.path.join(self.workspace_root, "python_position.txt")
        with open(py_pos_file, "w", encoding="utf-8") as f:
            for row in valid_rows:
                f.write(f"{row[0]},{row[1]},{row[2]},{row[3]},{row[4]},{row[5]},{row[6]},{row[7]}\n")

        # 2. Escribir joint_python_position.txt con los angulos articulares
        joint_py_file = os.path.join(self.workspace_root, "joint_python_position.txt")
        with open(joint_py_file, "w", encoding="utf-8") as f:
            for (q_deg, vel, ctrl) in valid_joints:
                q_str = ",".join(f"{g:.17f}" for g in q_deg)
                f.write(f"{q_str},{vel:.2f},{ctrl:.2f}\n")

        # 3. Escribir endowrist_python_position.txt con los valores de la pinza
        ew_file = os.path.join(self.workspace_root, "endowrist_python_position.txt")
        with open(ew_file, "w", encoding="utf-8") as f:
            for pz in valid_pinza:
                f.write(f"{pz[0]},{pz[1]},{pz[2]},{pz[3]}\n")

        self.nodo._estado(
            f"Archivos cartesianos generados. Ejecutando MoveL.py en modo '{self.modo}'..."
        )

        # 4. Lanzar MoveL.py pasando el modo de operación (--modo con_pinza / sin_pinza)
        script_path = "/home/miguel/ros2_ws/src/code/code/MoveL.py"
        cmd = (
            "bash -c '"
            "source /opt/ros/humble/setup.bash && "
            "source /home/miguel/ros2_ws/install/setup.bash && "
            f"python3 {script_path} --modo {self.modo}'"
        )
        proc = subprocess.Popen(cmd, shell=True)
        self.nodo._estado(f"MoveL.py lanzado (PID: {proc.pid}, modo: {self.modo})")

    def _ejecutar_trayectoria_articular(self, valid_rows, valid_joints, valid_pinza) -> None:
        self.nodo._estado(
            f"Iniciando ejecución articular ({len(valid_joints)} puntos, modo: {self.modo})."
        )

        all_control_zero = all(row[7] == 0 for row in valid_rows)
        if all_control_zero:
            self.nodo.pub_api.publish(String(data="SplineStart()"))
            time.sleep(0.05)

        max_index = 5
        batch_size = max_index
        num_batches = math.ceil(len(valid_joints) / batch_size)

        for batch in range(num_batches):
            start_idx = batch * batch_size
            end_idx = min((batch + 1) * batch_size, len(valid_joints))
            batch_joints = valid_joints[start_idx:end_idx]

            # 1. Enviar comandos JNTPoint para registrar el lote en el controlador
            for i, (q_deg, vel, ctrl) in enumerate(batch_joints):
                index = (i % max_index) + 1
                cmd_jnt = (
                    f"JNTPoint({index},{q_deg[0]:.2f},{q_deg[1]:.2f},{q_deg[2]:.2f},"
                    f"{q_deg[3]:.2f},{q_deg[4]:.2f},{q_deg[5]:.2f})"
                )
                self.nodo.pub_api.publish(String(data=cmd_jnt))
                time.sleep(0.01)

            # 2. Enviar comandos de movimiento y verificar llegada según el modo
            for i, (q_deg, vel, ctrl) in enumerate(batch_joints):
                index = (i % max_index) + 1
                global_idx = start_idx + i
                pz = valid_pinza[global_idx]

                if ctrl != 0:
                    cmd_mov = f"MoveJ(JNT{index},{vel:.2f})"
                    self.nodo.pub_api.publish(String(data=cmd_mov))

                    if self.con_pinza:
                        # Publicar comando de la pinza al mismo tiempo que el robot
                        self.nodo.pub_endowrist.publish(
                            String(data=f"{pz[0]},{pz[1]},{pz[2]},{pz[3]}")
                        )
                        self.nodo._estado(
                            f"[CON PINZA] Punto {global_idx + 1}: esperando llegada de Robot y Pinza "
                            f"(shaft={pz[0]:.2f}, wrist={pz[1]:.2f}, jaw_dx={pz[2]:.2f}, jaw_sx={pz[3]:.2f})..."
                        )
                        # Verificar que TANTO el robot COMO la pinza lleguen al setpoint
                        self.nodo.waitForSetpointReached(
                            target_joint_rad=np.radians(q_deg),
                            target_pinza_deg=pz,
                            check_pinza=True,
                            tolerance_robot_deg=0.5,
                            tolerance_pinza_deg=2.0,
                        )
                    else:
                        self.nodo._estado(
                            f"[SIN PINZA] Punto {global_idx + 1}: esperando únicamente llegada del Robot..."
                        )
                        # Verificar SOLO que el robot llegue al setpoint
                        self.nodo.waitForSetpointReached(
                            target_joint_rad=np.radians(q_deg),
                            target_pinza_deg=None,
                            check_pinza=False,
                            tolerance_robot_deg=0.5,
                        )

                    self.nodo._estado(f"Esperando {ctrl} segundos antes del siguiente comando.")
                    time.sleep(ctrl)
                else:
                    cmd_mov = f"SplinePTP(JNT{index},{vel:.2f})"
                    self.nodo.pub_api.publish(String(data=cmd_mov))
                    if self.con_pinza:
                        self.nodo.pub_endowrist.publish(
                            String(data=f"{pz[0]},{pz[1]},{pz[2]},{pz[3]}")
                        )
                    time.sleep(0.05)

            time.sleep(0.05)

        if all_control_zero:
            self.nodo.pub_api.publish(String(data="SplineEnd()"))

        self.nodo._estado(
            f"Comandos de trayectoria articular completados en modo '{self.modo}'."
        )

    # -------------------------------------------------------------------------
    # 3. Reproducción de Preview Visual (solo Unity, NO mueve el robot físico)
    # -------------------------------------------------------------------------
    def reproducir_preview(self, file_path: str) -> None:
        """Reproduce visualmente la trayectoria en Unity sin enviar al robot fisico."""
        resultado = self.cargar_y_validar_txt(file_path, es_preview=True)
        if resultado is None:
            return

        _, _, valid_joints, valid_pinza = resultado
        self.nodo._estado(f"Preview: {len(valid_joints)} puntos validados. Reproduciendo...")

        for i, ((q_deg, _, control), pinza) in enumerate(zip(valid_joints, valid_pinza)):
            delay = max(control, 0.5)

            # Publicar ángulos articulares del robot + pinza en /preview_joint_position
            # (SOLO mueve el robot y la pinza virtuales en Unity, NUNCA publica en /endowrist_command ni /api_command)
            q_str = ",".join(f"{g:.6f}" for g in q_deg)
            pz_str = f"{pinza[0]:.6f},{pinza[1]:.6f},{pinza[2]:.6f},{pinza[3]:.6f}"
            self.nodo.pub_preview.publish(String(data=f"{q_str},{pz_str}"))

            self.nodo._estado(
                f"Preview punto {i + 1}/{len(valid_joints)}: "
                f"J=[{q_deg[0]:.1f},{q_deg[1]:.1f},{q_deg[2]:.1f},"
                f"{q_deg[3]:.1f},{q_deg[4]:.1f},{q_deg[5]:.1f}] | "
                f"Pinza=[{pinza[0]:.1f},{pinza[1]:.1f},{pinza[2]:.1f},{pinza[3]:.1f}]"
            )
            time.sleep(delay)

        self.nodo._estado(f"Preview completado: {len(valid_joints)} puntos reproducidos.")

