import os
import csv
import math
import re
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
import tf2_ros
from tf2_ros import TransformException

# Identified steering regression model coefficients (signal <-> angle in degrees).
STEERING_MODEL_A = 10.422533
STEERING_MODEL_B = 1.026880


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def signal_to_angle_deg(signal: float) -> float:
    signal = clamp(signal, -3.0, 3.0)
    return math.copysign(STEERING_MODEL_A * abs(signal) ** STEERING_MODEL_B, signal)



FIELDNAMES = [
    "row_type",      # sample | planned_path
    "timestamp_s",
    "elapsed_s",
    "v_real",        # velocidad lineal real medida (/encoder/vel)
    "w_real",        # velocidad angular real estimada (map->base_link)
    "steering_angle_rad",  # angulo de direccion comando convertido a radianes
    "brake",         # cantidad de flancos de subida de freno desde la ultima fila
    "x",             # posicion X del robot en frame map
    "y",             # posicion Y del robot en frame map
    "distance_m",    # distancia acumulada en X-Y hasta esta muestra
    "heading_error_rad", # error de orientación respecto al camino ideal (nuevo campo)
    "kappa", 
    "path_index",    # indice del punto en el path ideal
    "path_x",        # X del path ideal
    "path_y",        # Y del path ideal
]

class MapMetricsLogger(Node):

    def __init__(self):
        super().__init__('map_metrics_logger')

        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)

        # ─── Estado ──────────────────────────────────────────────────────
        self.start_time = None

        self.total_distance = 0.0
        self.brake_active = False
        self.pending_brake_events = 0  # flancos de subida acumulados desde la ultima fila
        self.start_flag = False

        self.latest_x = None
        self.latest_y = None
        self.prev_row_x = None
        self.prev_row_y = None
        self.latest_v_real = 0.0
        self.latest_w_real = 0.0
        self.prev_yaw = None
        self.prev_yaw_time = None

        self.heading_error = 0.0
        self.kappa = 0.0

        self.path_saved = False
        self.pending_path_points = []

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._closed = False

        # ─── Archivo CSV ─────────────────────────────────────────────────
        data_dir = os.path.abspath("/home/santy-estrada/mrad_ws_2601_delta/src/delta_measure/data")
        self.get_logger().info(f"Directorio de datos: {data_dir}")
        os.makedirs(data_dir, exist_ok=True)

        existing = os.listdir(data_dir)
        pattern = re.compile(r"^(\d+)_rc_map_metrics_run\.csv$")
        max_n = 0
        for name in existing:
            match = pattern.match(name)
            self.get_logger().debug(f"Archivo encontrado: {name} → match={bool(match)}")
            if match:
                max_n = max(max_n, int(match.group(1)))
        next_n = max_n + 1

        output_path = os.path.join(data_dir, f"{next_n}_rc_map_metrics_run.csv")
        self._csv_file = open(output_path, "w", newline="", encoding="utf-8")
        self._writer   = csv.DictWriter(self._csv_file, fieldnames=FIELDNAMES)
        self._writer.writeheader()
        self._csv_file.flush()

        self.get_logger().info(f"Map Metrics Logger iniciado → {output_path}")

        # ─── Subscripciones ──────────────────────────────────────────────
        self.create_subscription(Bool,         '/start',                         self.start_cb, 10)
        self.create_subscription(Bool,         '/brake_active',                  self.brake_cb, 10)
        self.create_subscription(TwistStamped, '/cmd_vel_nav',                   self.cmd_cb, 10)
        self.create_subscription(Float64,      '/encoder/vel',                   self.encoder_vel_cb, 10)
        self.create_subscription(Float64,      '/heading_error',             self.heading_error_cb, 10)  # Error de orientación
        self.create_subscription(Float64,      '/kappa', self.kappa_cb, 10)  # Curvatura
        self.create_subscription(Path,         self.path_topic,               self.path_cb, 10)
        self.create_subscription(Bool,         '/stop',                          self.stop_cb,  10)

        self.get_logger().info(f"Escuchando path ideal en: {self.path_topic}")
        self.get_logger().info(
            f"Estimando pose en mapa con TF {self.map_frame}->{self.base_frame}."
        )

    # ─── Callbacks ───────────────────────────────────────────────────────

    def heading_error_cb(self, msg: Float64):
        self.heading_error = msg.data


    def kappa_cb(self, msg: Float64):
        self.kappa = msg.data

    def stop_cb(self, msg: Bool):
        #ros2 topic pub --once /stop std_msgs/msg/Bool "{data: false}"
        if msg.data:
            self.get_logger().info("[map_metrics_logger] Señal /stop recibida — cerrando.")
            self.close()
            raise SystemExit   # sale del spin() limpiamente

    def start_cb(self, msg: Bool):
        if msg.data and not self.start_flag:
            self.start_flag = True
            self.start_time = None

            # Reinicia acumuladores para que cada corrida arranque limpia.
            self.total_distance = 0.0
            self.pending_brake_events = 0
            self.prev_row_x = None
            self.prev_row_y = None
            self.prev_yaw = None
            self.prev_yaw_time = None
            self.latest_w_real = 0.0

            # Si ya hay path recibido, lo guarda una sola vez al iniciar la corrida.
            self._write_pending_path_once()

            self.get_logger().info("Señal /start recibida; esperando /cmd_vel_nav para fijar tiempo inicial.")
            
    def brake_cb(self, msg: Bool):
        if msg.data and not self.brake_active:
            self.pending_brake_events += 1
        self.brake_active = msg.data
        
    def encoder_vel_cb(self, msg: Float64):
        self.latest_v_real = float(msg.data)

    def cmd_cb(self, msg: TwistStamped):
        """Una muestra → una fila en el CSV."""
        if not self.start_flag:
            return

        cmd_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        sample_time = cmd_stamp if cmd_stamp > 0.0 else self.get_clock().now().nanoseconds * 1e-9

        if self.start_time is None:
            self.start_time = sample_time
            self.get_logger().info(f"Comenzando medicion en t={self.start_time:.6f}s (primer /cmd_vel_nav).")

        elapsed = sample_time - self.start_time
        if elapsed < 0.0:
            elapsed = 0.0

        steering_signal = float(msg.twist.angular.z)
        steering_angle_rad = math.radians(signal_to_angle_deg(steering_signal))

        x, y = self._update_map_pose_and_omega(sample_time)
        v = self.latest_v_real
        w = self.latest_w_real

        if self.prev_row_x is None or self.prev_row_y is None:
            kappa_distance = 0.0
        else:
            kappa_distance = math.hypot(x - self.prev_row_x, y - self.prev_row_y)

        # La distancia acumulada avanza exactamente con el salto entre puntos guardados en filas consecutivas.
        self.total_distance += kappa_distance

        row = {
            "row_type": "sample",
            "timestamp_s": sample_time,
            "elapsed_s": elapsed,
            "v_real":      v,
            "w_real":      w,
            "steering_angle_rad": steering_angle_rad,
            "brake":       self.pending_brake_events,
            "x":           x,
            "y":           y,
            "distance_m":  self.total_distance,
            "heading_error_rad": self.heading_error,
            "kappa": self.kappa,
            "path_index": None,
            "path_x": None,
            "path_y": None,
        }

        # Consume eventos de freno en el instante de escritura de la fila.
        self.pending_brake_events = 0

        self.prev_row_x = x
        self.prev_row_y = y

        self._writer.writerow(row)
        self._csv_file.flush()

    def _update_map_pose_and_omega(self, sample_time: float):
        x = self.latest_x if self.latest_x is not None else 0.0
        y = self.latest_y if self.latest_y is not None else 0.0

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            q = tf.transform.rotation

            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            x = tx
            y = ty
            self.latest_x = x
            self.latest_y = y

            if self.prev_yaw is not None and self.prev_yaw_time is not None:
                dt = sample_time - self.prev_yaw_time
                if dt > 1e-6:
                    yaw_delta = math.atan2(math.sin(yaw - self.prev_yaw), math.cos(yaw - self.prev_yaw))
                    self.latest_w_real = yaw_delta / dt
                else:
                    self.latest_w_real = 0.0
            else:
                self.latest_w_real = 0.0

            self.prev_yaw = yaw
            self.prev_yaw_time = sample_time

        except TransformException as ex:
            self.get_logger().warn(
                f"TF lookup fallido ({self.map_frame}->{self.base_frame}): {ex}",
                throttle_duration_sec=2.0,
            )

        return x, y

    def path_cb(self, msg: Path):
        if self.path_saved:
            return
        if not msg.poses:
            return

        self.pending_path_points = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        ]

        # Si la corrida ya comenzo, guarda el path inmediatamente.
        if self.start_flag:
            self._write_pending_path_once()

    def _write_pending_path_once(self):
        if self.path_saved:
            return
        if not self.pending_path_points:
            return

        self.get_logger().info(
            f"Guardando path ideal con {len(self.pending_path_points)} puntos en CSV."
        )

        for idx, (path_x, path_y) in enumerate(self.pending_path_points):
            row = {
                "row_type": "planned_path",
                "timestamp_s": None,
                "elapsed_s": None,
                "v_real": None,
                "w_real": None,
                "steering_angle_rad": None,
                "brake": None,
                "x": None,
                "y": None,
                "distance_m": None,
                "heading_error_rad": None,
                "kappa": None,
                "path_index": idx,
                "path_x": path_x,
                "path_y": path_y,
            }
            self._writer.writerow(row)

        self._csv_file.flush()
        self.path_saved = True

    # ─── Cierre ──────────────────────────────────────────────────────────

    def close(self):
        if self._closed:
            return

        # Si la corrida termina y aun no se guardo el path, intenta guardarlo al cierre.
        self._write_pending_path_once()

        self._csv_file.flush()
        self._csv_file.close()
        self._closed = True
        self.get_logger().info("CSV cerrado correctamente.")

def main():
    rclpy.init()
    node = MapMetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()