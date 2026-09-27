#!/usr/bin/env python3
"""
Замер задержки «вход → результат» и частоты публикации для ноды tram_backup_odometry.

Задержка = время прихода /result/velocity минус время прихода входного сообщения с тем же header.stamp
(колёса или контроллер). Меряется по стенным часам, поэтому работает при обычном `ros2 bag play`.

    python3 latency_check.py            # запускать до `ros2 bag play`, остановить Ctrl+C — итоговый отчёт
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor


def key(stamp):
    return (stamp.sec, stamp.nanosec)


class LatencyCheck(Node):
    def __init__(self):
        super().__init__('latency_check')
        self.arrival = {}                      # stamp входа → стенное время прихода
        self.lat = []                          # задержки, с
        self.n_vel = self.n_pos = 0
        self.t_first = self.t_last = None
        q = qos_profile_sensor_data
        for topic in ('/vehicle/front_bogie_velocity', '/vehicle/rear_bogie_velocity'):
            self.create_subscription(VelocitySensor, topic, self.on_input, q)
        self.create_subscription(DriverControllerCommand, '/vehicle/driver_position_cmd', self.on_input, q)
        self.create_subscription(VelocitySensor, '/result/velocity', self.on_result, q)
        self.create_subscription(Odometry, '/result/position', self.on_position, q)
        self.create_timer(10.0, self.report)

    def on_input(self, msg):
        self.arrival.setdefault(key(msg.header.stamp), time.monotonic())
        if len(self.arrival) > 5000:           # не копим память
            for k in list(self.arrival)[:2500]:
                del self.arrival[k]

    def on_result(self, msg):
        now = time.monotonic()
        self.n_vel += 1
        self.t_first = self.t_first or now
        self.t_last = now
        t_in = self.arrival.get(key(msg.header.stamp))
        if t_in is not None:
            self.lat.append(now - t_in)

    def on_position(self, msg):
        self.n_pos += 1

    def report(self):
        if not self.lat:
            self.get_logger().info('ещё нет пар вход/результат')
            return
        a = np.array(self.lat) * 1e3
        dur = max((self.t_last or 0) - (self.t_first or 0), 1e-9)
        self.get_logger().info(
            f'задержка, мс: медиана {np.median(a):.2f}, p99 {np.quantile(a, .99):.2f}, max {a.max():.2f} '
            f'(n={len(a)}) | частота /result/velocity {self.n_vel / dur:.1f} Гц, '
            f'/result/position {self.n_pos / dur:.1f} Гц')


def main():
    rclpy.init()
    node = LatencyCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
