import os
import csv
import math
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import TwistStamped, Pose 
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock


FIELDNAMES = [
    "row_type",      # sample | planned_path
    "timestamp_s",
    "elapsed_s",
    "v_real",        # velocidad lineal real  (/diffdrive_controller/cmd_vel)
    "w_real",        # velocidad angular real
    "brake",         # cantidad de flancos de subida de freno desde la ultima fila
    "x",             # posición X del robot (Real)
    "y",             # posición Y del robot (Real)
    "distance_m",    # distancia acumulada en X-Y hasta esta muestra
    "heading_error_rad", # error de orientación respecto al camino ideal (nuevo campo)
    "cross_track_error_m", # error de posición lateral respecto al camino ideal (nuevo campo)
    "controller_error", # error combinado que el controlador intenta minimizar (nuevo campo)
    "path_index",    # indice del punto en el path ideal
    "path_x",        # X del path ideal
    "path_y",        # Y del path ideal
]

class MapMetricsLogger(Node):

    def __init__(self):
        super().__init__('map_metrics_logger')

        self.declare_parameter('path_topic', '/planned_path')
        self.path_topic = str(self.get_parameter('path_topic').value)

        # ─── Estado ──────────────────────────────────────────────────────
        self.start_time = None
        self.last_clock = None

        self.total_distance = 0.0
        self.brake_active = False
        self.pending_brake_events = 0  # flancos de subida acumulados desde la ultima fila
        self.start_flag = False

        self.latest_x = None
        self.latest_y = None
        self.prev_row_x = None
        self.prev_row_y = None
        self.v_ref = 0.0
        self.w_ref = 0.0

        self.heading_error = 0.0
        self.cross_track_error = 0.0
        self.controller_error = 0.0

        self.path_saved = False
        self.pending_path_points = []

        self._closed = False

        # ─── Archivo CSV ─────────────────────────────────────────────────
        data_dir = os.path.abspath("/home/santy-estrada/mrad_ws_2601_delta/src/delta_measure/data")
        self.get_logger().info(f"Directorio de datos: {data_dir}")
        os.makedirs(data_dir, exist_ok=True)

        existing = os.listdir(data_dir)
        pattern = re.compile(r"^(\d+)_map_metrics_run\.csv$")
        max_n = 0
        for name in existing:
            match = pattern.match(name)
            self.get_logger().debug(f"Archivo encontrado: {name} → match={bool(match)}")
            if match:
                max_n = max(max_n, int(match.group(1)))
        next_n = max_n + 1

        output_path = os.path.join(data_dir, f"{next_n}_map_metrics_run.csv")
        self._csv_file = open(output_path, "w", newline="", encoding="utf-8")
        self._writer   = csv.DictWriter(self._csv_file, fieldnames=FIELDNAMES)
        self._writer.writeheader()
        self._csv_file.flush()

        self.get_logger().info(f"Map Metrics Logger iniciado → {output_path}")

        # ─── Subscripciones ──────────────────────────────────────────────
        self.create_subscription(Bool,         '/start',                         self.start_cb, 10)
        self.create_subscription(Bool,         '/brake_active',                  self.brake_cb, 10)
        self.create_subscription(TwistStamped, '/cmd_vel_ttc_gap',               self.ref_cb,   10)
        self.create_subscription(TwistStamped, '/diffdrive_controller/cmd_vel',  self.cmd_cb,   10)
        self.create_subscription(Float64,      '/heading_error',             self.heading_error_cb, 10)  # Error de orientación
        self.create_subscription(Float64,      '/cross_track_error',         self.cross_track_error_cb, 10)  # Error de posición lateral
        self.create_subscription(Float64,      '/delta', self.delta_cb, 10)  # Error combinado que el controlador intenta minimizar
        self.create_subscription(Path,         self.path_topic,               self.path_cb, 10)

        # <-- Swapped /odom for the new /ground_truth_pose bridge topic -->
        self.create_subscription(Pose,         '/ground_truth_pose',             self.pose_cb,  10)
        self.create_subscription(Clock,        '/clock',                         self.clock_cb, 10)
        self.create_subscription(Bool,         '/stop',                          self.stop_cb,  10)

        self.get_logger().info(f"Escuchando path ideal en: {self.path_topic}")

    # ─── Callbacks ───────────────────────────────────────────────────────

    def heading_error_cb(self, msg: Float64):
        self.heading_error = msg.data

    def cross_track_error_cb(self, msg: Float64):
        self.cross_track_error = msg.data

    def delta_cb(self, msg: Float64):
        self.controller_error = msg.data

    def stop_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info("[map_metrics_logger] Señal /stop recibida — cerrando.")
            self.close()
            raise SystemExit   # sale del spin() limpiamente

    def start_cb(self, msg: Bool):
        if msg.data and not self.start_flag:
            self.start_flag = True
            # Arranca el cronometro exactamente con /start (o en el primer /clock posterior si aun no hay reloj).
            self.start_time = self.last_clock

            # Reinicia acumuladores para que cada corrida arranque limpia.
            self.total_distance = 0.0
            self.pending_brake_events = 0
            self.prev_row_x = None
            self.prev_row_y = None

            # Si ya hay path recibido, lo guarda una sola vez al iniciar la corrida.
            self._write_pending_path_once()

            if self.start_time is None:
                self.get_logger().info("Señal /start recibida; esperando /clock para fijar tiempo inicial.")
            else:
                self.get_logger().info(f"Señal /start recibida → comenzando medición en t={self.start_time:.6f}s.")
            
    def brake_cb(self, msg: Bool):
        if msg.data and not self.brake_active:
            self.pending_brake_events += 1
        self.brake_active = msg.data
        
    def ref_cb(self, msg: TwistStamped):
        self.v_ref = msg.twist.linear.x       # ← TwistStamped necesita .twist.
        self.w_ref = msg.twist.angular.z

    def cmd_cb(self, msg: TwistStamped):
        """Una muestra → una fila en el CSV."""
        if not self.start_flag:
            return
        if self.start_time is None:
            return  # aun no hay reloj sincronizado luego de /start

        cmd_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Si el publisher no estampa, cae al ultimo /clock disponible.
        sample_time = self.last_clock if cmd_stamp == 0.0 and self.last_clock is not None else cmd_stamp
        if sample_time is None:
            return

        elapsed = sample_time - self.start_time
        if elapsed < 0.0:
            elapsed = 0.0

        v = msg.twist.linear.x
        w = msg.twist.angular.z

        if self.latest_x is None or self.latest_y is None:
            x = 0.0
            y = 0.0
            delta_distance = 0.0
        else:
            x = self.latest_x
            y = self.latest_y
            if self.prev_row_x is None or self.prev_row_y is None:
                delta_distance = 0.0
            else:
                delta_distance = math.hypot(x - self.prev_row_x, y - self.prev_row_y)

        # La distancia acumulada avanza exactamente con el salto entre puntos guardados en filas consecutivas.
        self.total_distance += delta_distance

        row = {
            "row_type": "sample",
            "timestamp_s": sample_time,
            "elapsed_s": elapsed,
            "v_real":      v,
            "w_real":      w,
            "brake":       self.pending_brake_events,
            "x":           x,
            "y":           y,
            "distance_m":  self.total_distance,
            "heading_error_rad": self.heading_error,
            "cross_track_error_m": self.cross_track_error,
            "controller_error": self.controller_error,
            "path_index": None,
            "path_x": None,
            "path_y": None,
        }

        # Consume eventos de freno en el instante de escritura de la fila.
        self.pending_brake_events = 0

        if self.latest_x is not None and self.latest_y is not None:
            self.prev_row_x = x
            self.prev_row_y = y

        self._writer.writerow(row)
        self._csv_file.flush()

    # <-- New callback handling exact Gazebo Pose -->
    def pose_cb(self, msg: Pose):
        self.latest_x = msg.position.x
        self.latest_y = msg.position.y

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
                "brake": None,
                "x": None,
                "y": None,
                "distance_m": None,
                "heading_error_rad": None,
                "cross_track_error_m": None,
                "controller_error": None,
                "path_index": idx,
                "path_x": path_x,
                "path_y": path_y,
            }
            self._writer.writerow(row)

        self._csv_file.flush()
        self.path_saved = True

    def clock_cb(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.last_clock = t

        if self.start_flag and self.start_time is None:
            self.start_time = t
            self.get_logger().info(f"Reloj sincronizado post-/start en t={self.start_time:.6f}s.")

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