import os
import csv
import math
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


FIELDNAMES = [
    "timestamp_s",
    "elapsed_s",
    "v_real",        # velocidad lineal real  (/diffdrive_controller/cmd_vel)
    "w_real",        # velocidad angular real
    "brake",         # 1 = freno activo, 0 = inactivo
    "x",             # posición X del robot (odometría)
    "y",             # posición Y del robot (odometría)
    "distance_m",    # distancia acumulada en X-Y hasta esta muestra
    
]


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger')

        # ─── Estado ──────────────────────────────────────────────────────
        self.start_time  = None
        self.last_clock  = None
        self.last_time   = None          # dt entre ticks de reloj

        self.total_distance = 0.0
        self.brake_active  = False
        self.brake_event   = False   # ← se pone True solo en el flanco subida
        self.start_flag     = False
        
        self.last_x = None
        self.last_y = None
        self.v_ref = 0.0
        self.w_ref = 0.0

        # ─── Archivo CSV ─────────────────────────────────────────────────
        data_dir = os.path.abspath("/home/santy-estrada/mrad_ws_2601_delta/src/delta_measure/data")
        self.get_logger().info(f"Directorio de datos: {data_dir}")
        os.makedirs(data_dir, exist_ok=True)

        existing = os.listdir(data_dir)
        pattern = re.compile(r"^(\d+)_metrics_run\.csv$")
        max_n = 0
        for name in existing:
            match = pattern.match(name)
            self.get_logger().debug(f"Archivo encontrado: {name} → match={bool(match)}")
            if match:
                max_n = max(max_n, int(match.group(1)))
        next_n = max_n + 1

        output_path = os.path.join(data_dir, f"{next_n}_metrics_run.csv")
        self._csv_file = open(output_path, "w", newline="", encoding="utf-8")
        self._writer   = csv.DictWriter(self._csv_file, fieldnames=FIELDNAMES)
        self._writer.writeheader()
        self._csv_file.flush()

        self.get_logger().info(f"Metrics Logger iniciado → {output_path}")

        # ─── Subscripciones ──────────────────────────────────────────────
        self.create_subscription(Bool,         '/start',                         self.start_cb, 10)
        self.create_subscription(Bool,         '/brake_active',                  self.brake_cb, 10)
        self.create_subscription(TwistStamped, '/cmd_vel_ttc_gap',               self.ref_cb,   10)
        self.create_subscription(TwistStamped, '/diffdrive_controller/cmd_vel',  self.cmd_cb,   10)
        self.create_subscription(Odometry,     '/diffdrive_controller/odom',     self.odom_cb,  10)
        self.create_subscription(Clock,        '/clock',                          self.clock_cb, 10)
        self.create_subscription(Bool, '/stop', self.stop_cb, 10)

    # ─── Callbacks ───────────────────────────────────────────────────────
    # En __init__

    # Callback
    def stop_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info("[metrics_logger] Señal /stop recibida — cerrando.")
            self.close()
            raise SystemExit   # sale del spin() limpiamente

    def start_cb(self, msg: Bool):
        if msg.data and not self.start_flag:
            self.start_flag = True
            self.get_logger().info("Señal /start recibida → comenzando medición.")
            
    def brake_cb(self, msg: Bool):
        if msg.data and not self.brake_active:
            self.brake_event = True    # flanco de subida detectado
        self.brake_active = msg.data
        
    def ref_cb(self, msg: TwistStamped):
        self.v_ref = msg.twist.linear.x       # ← TwistStamped necesita .twist.
        self.w_ref = msg.twist.angular.z

    def cmd_cb(self, msg: TwistStamped):
        """Una muestra → una fila en el CSV."""
        if not self.start_flag or self.last_clock is None:
            return 
        if self.start_time is None or self.last_clock is None:
            return                            # aún no hay reloj sincronizado

        v = msg.twist.linear.x
        w = msg.twist.angular.z

        row = {
            "timestamp_s": self.last_clock,
            "elapsed_s":   self.last_clock - self.start_time,
            "v_real":      v,
            "w_real":      w,
            "brake":       1 if self.brake_event else 0,   # ← solo el flanco
            "x":           self.last_x if self.last_x is not None else 0.0,
            "y":           self.last_y if self.last_y is not None else 0.0,
            "distance_m": self.total_distance,
        }
        self.brake_event = False   # ← consumido, vuelve a 0 en la siguiente fila
        self._writer.writerow(row)
        self._csv_file.flush()  # Flush after each write for data safety

    def odom_cb(self, msg: Odometry):
        """Integra distancia usando v * dt del reloj de simulación."""
        if not self.start_flag or self.last_clock is None:
            return 
        if self.last_time is None:
            return
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is not None:
            self.total_distance += math.hypot(x - self.last_x, y - self.last_y)

        self.last_x = x
        self.last_y = y
        
        

    def clock_cb(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9

        if self.start_time is None:
            self.start_time = t
            self.last_clock = t
            return

        self.last_time  = t - self.last_clock
        self.last_clock = t

    # ─── Cierre ──────────────────────────────────────────────────────────

    def close(self):
        self._csv_file.flush()
        self._csv_file.close()
        self.get_logger().info("CSV cerrado correctamente.")


def main():
    rclpy.init()
    node = MetricsLogger()
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