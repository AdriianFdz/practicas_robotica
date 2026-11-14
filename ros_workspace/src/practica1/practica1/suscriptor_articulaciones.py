#!/usr/bin/python3

import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ur3e_interfaces.msg import EstadoRobot


class SuscriptorArticulaciones(Node):
    """Nodo que se suscribe a /joint_states y procesa el estado de las articulaciones.

    Calcula la aceleración de cada articulación, guarda las muestras mientras el
    robot se mueve y las publica con el mensaje personalizado EstadoRobot.
    """

    def __init__(self):
        """Inicializa el nodo, la suscripción y el publicador."""
        super().__init__('suscriptor_articulaciones')
        self.create_subscription(JointState, '/joint_states', self._procesar_estado, 10)
        self.estados_actuales: dict[str, EstadoRobot] = {}
        self.historial: list[dict] = []
        self.muestras_anteriores: dict[str, tuple[float, float]] = {}
        self.trayectoria_actual: str = 'sin_trayectoria'
        self.publicador_estado = self.create_publisher(EstadoRobot, '/estado_articulaciones', 10)
        self.umbral_velocidad: float = 1e-3

    def _procesar_estado(self, mensaje: JointState) -> None:
        """Guarda los datos recibidos de cada articulación.

        Args:
            mensaje (JointState): Mensaje recibido del topic /joint_states.
        """
        if 'finger_joint' in mensaje.name:
            return

        tiempo = (
            mensaje.header.stamp.sec
            + mensaje.header.stamp.nanosec / 1_000_000_000
        )

        en_movimiento = False
        for velocidad in mensaje.velocity:
            if abs(velocidad) > self.umbral_velocidad:
                en_movimiento = True

        for i, nombre in enumerate(mensaje.name):
            estado = EstadoRobot()
            estado.header = mensaje.header
            estado.nombre = nombre
            estado.posicion = mensaje.position[i]
            estado.velocidad = mensaje.velocity[i]
            self.estados_actuales[nombre] = estado
            aceleracion = None
            velocidad_actual = estado.velocidad

            if nombre in self.muestras_anteriores:
                tiempo_anterior, velocidad_anterior = (
                    self.muestras_anteriores[nombre]
                )
                diferencia_tiempo = tiempo - tiempo_anterior

                if diferencia_tiempo > 0:
                    aceleracion = (
                        velocidad_actual - velocidad_anterior
                    ) / diferencia_tiempo
                    estado.aceleracion = aceleracion

            self.muestras_anteriores[nombre] = (
                tiempo,
                velocidad_actual,
            )
            if en_movimiento:
                self.publicador_estado.publish(estado)
                self.historial.append({
                    'tiempo': tiempo,
                    'articulacion': nombre,
                    'posicion': estado.posicion,
                    'velocidad': estado.velocidad,
                    'aceleracion': aceleracion,
                    'trayectoria': self.trayectoria_actual,
                })

    def guardar_csv(self, ruta: str) -> None:
        """Guarda el historial en un CSV con una fila por muestra."""
        archivo = Path(ruta)
        archivo.parent.mkdir(parents=True, exist_ok=True)

        campos = [
            'trayectoria',
            'tiempo',
            'articulacion',
            'posicion',
            'velocidad',
            'aceleracion',
        ]

        muestras = self.historial.copy()

        with archivo.open('w', newline='', encoding='utf-8') as fichero:
            escritor = csv.DictWriter(fichero, fieldnames=campos)
            escritor.writeheader()
            escritor.writerows(muestras)

    def obtener_datos(self) -> dict[str, EstadoRobot]:
        """Obtiene los datos de las articulaciones del robot.

        Returns:
            dict[str, EstadoRobot]: Diccionario con los datos de las articulaciones del robot.
        """
        return self.estados_actuales


def main() -> None:
    """Función principal del nodo SuscriptorArticulaciones."""
    rclpy.init()
    nodo = SuscriptorArticulaciones()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
