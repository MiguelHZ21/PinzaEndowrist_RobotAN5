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
import sys

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

        self.create_subscription(String, args.pose_topico, self.cb_pose, qos)

        if args.joint_tipo == "string":
            self.create_subscription(String, args.joint_topico,
                                     self.cb_juntas_string, qos)
        else:
            from sensor_msgs.msg import JointState
            self.create_subscription(JointState, args.joint_topico,
                                     self.cb_juntas_jointstate, qos)

        self.get_logger().info(
            f"fr5_ik_node listo | d6={self.params.d6} m | "
            f"juntas: {args.joint_topico} ({args.joint_tipo}) | "
            f"poses: {args.pose_topico} -> {args.salida_topico} | "
            f"salto_max={args.salto_max} deg")

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
            # Si aún no se recibe la posición del robot real, usamos la postura "Home" natural [0, -90, 90, -90, -90, 90] deg
            # para seleccionar la rama con codo hacia abajo igual que MATLAB, evitando codo invertido.
            semilla = np.radians([0.0, -90.0, 90.0, -90.0, -90.0, 90.0])
            self._estado("AVISO: todavia no llego la posicion real del robot; "
                         "usando postura Home por defecto [0,-90,90,-90,-90,90] deg", "warn")

        sol = ik_mejor(T, q_semilla=semilla,
                       salto_max=self.salto_max if self.q_actual is not None else None,
                       params=self.params)

        if sol is None:
            # Distinguir "inalcanzable" de "alcanzable pero salto peligroso":
            # al usuario le cambia por completo que hacer.
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
                    f"{math.degrees(self.salto_max):.0f}). Reposiciona el robot "
                    f"o subi --salto-max a conciencia -> {msg.data}", "error")
            return

        grados = sol.grados()
        self.pub_q.publish(String(data=",".join(f"{g:.6f}" for g in grados)))

        aviso = ""
        if sol.manipulabilidad < 0.01:
            aviso = (f" | CERCA DE SINGULARIDAD (sigma_min="
                     f"{sol.manipulabilidad:.4f}): bajá la velocidad")
        if sol.singular:
            aviso += " | muñeca singular: q6 tomado de la posicion actual"
        self._estado(
            f"OK {sol.rama} | q=[{', '.join(f'{g:.3f}' for g in grados)}]"
            f"{aviso}")


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
