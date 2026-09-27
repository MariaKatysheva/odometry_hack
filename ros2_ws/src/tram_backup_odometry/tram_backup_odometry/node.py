"""
Резервная одометрия трамвая: скорость и положение по колёсам тележек и ручке контроллера.

Вход (в основном контуре — только эти три топика):
    /vehicle/front_bogie_velocity   tram_vehicle_msgs/VelocitySensor          (км/ч, как в данных)
    /vehicle/rear_bogie_velocity    tram_vehicle_msgs/VelocitySensor          (км/ч)
    /vehicle/driver_position_cmd    tram_vehicle_msgs/DriverControllerCommand (−15…15)
GNSS (/sensing/gnss/{master,rover}/fix) — только начальная выставка: RTK-точки первых `gnss_window` секунд.
Без GNSS — относительная одометрия: x = пройденный путь, y = z = 0.

Выход (header.stamp = время входного сообщения):
    /result/velocity   tram_vehicle_msgs/VelocitySensor  — скорость, м/с
    /result/position   nav_msgs/Odometry                 — положение base_link в map (Pathgraph), ковариация из фильтра
Публикация — на каждое сообщение колёс (после оценки) и контроллера (прогноз по модели, фильтр не меняется).
"""
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

from .core.estimator import Estimator, Params
from .core.start import init_from_gnss
from .core.track import Track, read_csv


def _t(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _data_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('tram_backup_odometry')) / 'data'
    except Exception:                                   # запуск из исходников
        return Path(__file__).resolve().parent.parent / 'data'


class BackupOdometry(Node):
    def __init__(self):
        super().__init__('tram_backup_odometry')
        P = {  # параметры по умолчанию = выбранный вариант (IMM + признак 2 + исправление ложного слипа)
            'data_dir': '', 'frame_id': 'map', 'child_frame_id': 'base_link',
            'gnss_window': 20.0, 'gnss_max_dist': 30.0, 'pair_tol': 0.005, 'pair_wait': 0.05,
            'publish_on_cmd': True, 'sigma_cross': 1.0, 'sigma_z': 0.5,
            'sigma_v_scale': 1.534, 'kappa_sigma': 0.0062, 'stop_exit': 0.5, 'slip_rollback': True,
            'slip_no_neutral': True,
        }
        for k, v in P.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        data = Path(g('data_dir')) if g('data_dir') else _data_dir()
        self.tracks = {d: Track(d, data) for d in ('AB', 'BA')}
        params = Params(anchors=False, gate_v=1e9, gp_sigma='const', imm=True,
                        drive_table=read_csv(data / 'drive_gp_table.csv'),
                        slip_calib=json.load(open(data / 'slip_calib.json')),
                        sigma_v_scale=g('sigma_v_scale'), kappa_sigma=g('kappa_sigma'),
                        stop_exit=g('stop_exit'), slip_rollback=g('slip_rollback'),
                        slip_no_neutral=g('slip_no_neutral'))
        self.est = Estimator(self.tracks['AB'], params)
        self.frame, self.child = g('frame_id'), g('child_frame_id')
        self.gnss_window, self.gnss_max_dist = g('gnss_window'), g('gnss_max_dist')
        self.pair_tol, self.pair_wait = g('pair_tol'), g('pair_wait')
        self.publish_on_cmd = g('publish_on_cmd')
        self.sig_c, self.sig_z = g('sigma_cross'), g('sigma_z')

        self.started = False          # первая порция колёс получена, фильтр запущен
        self.aligned = None           # результат выставки по GNSS (dict) или None — относительный режим
        self.offset = 0.0             # s_абс − s_отн, применённый к оценщику
        self.s_rel_first = None       # относительный путь в момент первой RTK-точки
        self.fixes = []
        self.odo_t, self.odo_s = [], []  # путь по колёсам (относительный) — для выставки по окну GNSS
        self.pending = {}             # 'front'/'rear' → (stamp, км/ч, стенное время прихода)

        qin = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=100)
        self.create_subscription(VelocitySensor, '/vehicle/front_bogie_velocity', lambda m: self.on_bogie('front', m), qin)
        self.create_subscription(VelocitySensor, '/vehicle/rear_bogie_velocity', lambda m: self.on_bogie('rear', m), qin)
        self.create_subscription(DriverControllerCommand, '/vehicle/driver_position_cmd', self.on_cmd, qin)
        self.create_subscription(NavSatFix, '/sensing/gnss/master/fix', lambda m: self.on_fix('master', m), qin)
        self.create_subscription(NavSatFix, '/sensing/gnss/rover/fix', lambda m: self.on_fix('rover', m), qin)
        self.pub_v = self.create_publisher(VelocitySensor, '/result/velocity', 10)
        self.pub_p = self.create_publisher(Odometry, '/result/position', 10)
        self.create_timer(0.02, self.flush_stale)
        self.get_logger().info(f'данные: {data}; жду колёса и контроллер')

    # ---------------- колёса: пара передняя/задняя с одним stamp → один шаг оценщика
    def on_bogie(self, side, msg):
        t, z = _t(msg.header.stamp), float(msg.velocity)
        other = 'rear' if side == 'front' else 'front'
        if other in self.pending and abs(self.pending[other][0] - t) <= self.pair_tol:
            zo = self.pending.pop(other)[1]
            front, rear = (z, zo) if side == 'front' else (zo, z)
            self.step(t, front, rear, msg.header.stamp)
            return
        if side in self.pending:                        # прошлое сообщение этой тележки осталось без пары
            self.flush(side)
        self.pending[side] = (t, z, time.monotonic(), msg.header.stamp)

    def flush(self, side):
        t, z, _, stamp = self.pending.pop(side)
        nan = float('nan')
        self.step(t, z if side == 'front' else nan, z if side == 'rear' else nan, stamp)

    def flush_stale(self):                              # вторая тележка не пришла (пропуск) — считаем с одной
        now = time.monotonic()
        for side in sorted(self.pending, key=lambda k: self.pending[k][0]):
            if now - self.pending[side][2] > self.pair_wait:
                self.flush(side)

    def step(self, t, front, rear, stamp):
        if not self.started:
            self.est.init(t, 0.0)
            self.started = True
            self.get_logger().info('старт: относительная одометрия до выставки по GNSS')
        self.est.on_wheels(t, front, rear)
        if self.aligned is None or t <= self.aligned['stamp'] + self.gnss_window + 1.0:
            self.odo_t.append(t)
            self.odo_s.append(self.est.s - self.offset)
        v, xyz = self.est.output()
        self.publish(stamp, v, xyz)

    # ---------------- контроллер
    def on_cmd(self, msg):
        t = _t(msg.header.stamp)
        self.est.on_cmd(t, int(msg.position))
        if self.started and self.publish_on_cmd:
            v, xyz = self.est.peek(t)
            self.publish(msg.header.stamp, v, xyz)

    # ---------------- GNSS: только начальная выставка
    def on_fix(self, receiver, msg):
        t = _t(msg.header.stamp)
        if not self.started or int(msg.status.status) != 2:     # только RTK
            return
        if self.fixes and t > self.fixes[0][1] + self.gnss_window:
            return                                               # окно выставки закончилось — GNSS больше не нужен
        self.fixes.append((receiver, t, msg.latitude, msg.longitude, 2))
        last = self.fixes[-1][1]
        if self.aligned is not None and last - getattr(self, '_t_align', 0.0) < 1.0:
            return                                               # пересчёт не чаще раза в секунду
        self._t_align = last
        odo = lambda tt: float(np.interp(tt, self.odo_t, self.odo_s)) if self.odo_t else 0.0
        ini = init_from_gnss(self.fixes, self.tracks, self.gnss_window, self.gnss_max_dist, odo=odo)
        if ini is None:
            return
        if self.s_rel_first is None:                    # относительный путь в момент первой RTK-точки
            self.s_rel_first = odo(ini['stamp'])
        new_offset = ini['s0'] - self.s_rel_first
        changed = self.aligned is None or ini['direction'] != self.aligned['direction']
        self.est.track = self.tracks[ini['direction']]
        self.est.s += new_offset - self.offset
        self.offset, self.aligned = new_offset, ini
        if changed:
            self.get_logger().info(f"выставка по GNSS: направление {ini['direction']}, s0 = {ini['s0']:.1f} м "
                                   f"(до линии {ini['dist']:.1f} м)")

    # ---------------- выход
    def publish(self, stamp, v, xyz):
        sv, ss = self.est.sigma()
        if self.aligned is None:                        # относительная одометрия
            x, y, z, yaw = self.est.s - self.offset, 0.0, 0.0, 0.0
        else:
            (x, y, z), yaw = xyz, self.est.track.yaw(self.est.s)
        mv = VelocitySensor()
        mv.header.stamp, mv.header.frame_id = stamp, self.child
        mv.velocity = float(v)
        self.pub_v.publish(mv)
        mo = Odometry()
        mo.header.stamp, mo.header.frame_id, mo.child_frame_id = stamp, self.frame, self.child
        p = mo.pose.pose
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        c, s = math.cos(yaw), math.sin(yaw)             # вдоль пути — σ_s, поперёк — σ_cross
        cov = [0.0] * 36
        cov[0] = ss ** 2 * c * c + self.sig_c ** 2 * s * s
        cov[7] = ss ** 2 * s * s + self.sig_c ** 2 * c * c
        cov[1] = cov[6] = (ss ** 2 - self.sig_c ** 2) * s * c
        cov[14] = self.sig_z ** 2
        cov[21] = cov[28] = cov[35] = 1e3               # ориентацию не оцениваем (курс — по карте)
        mo.pose.covariance = cov
        mo.twist.twist.linear.x = float(v)
        tc = [0.0] * 36
        tc[0] = sv ** 2
        mo.twist.covariance = tc
        self.pub_p.publish(mo)


def main(args=None):
    rclpy.init(args=args)
    node = BackupOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
