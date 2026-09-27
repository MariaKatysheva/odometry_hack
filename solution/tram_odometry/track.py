"""
Карта пути одного направления: s (м вдоль рельсов, точка base_link) -> x, y, z в системе Pathgraph
(UTM 37N минус (300 000, 6 100 000)), уклон.

Нужны файлы в data/: route_map.csv (линии AB/BA с шагом 1 м), pathgraph_AB.json, pathgraph_BA.json.
Зависимости: только numpy (без pandas, scipy, pyproj) — чтобы работать в базовом образе ROS 2 Humble.
"""
import json
from pathlib import Path

import csv

import numpy as np

PG_SHIFT = 100_000.0          # route_map.csv: x в MGRS 37U -> Pathgraph: x + 100 000
DATA = Path(__file__).resolve().parent.parent / 'data'


def read_csv(path, where=None):
    """CSV → dict столбец → np.array (числа — float, остальное — str); where — фильтр строк (dict → bool)."""
    with open(path, newline='') as f:
        rows = [r for r in csv.DictReader(f) if where is None or where(r)]
    out = {}
    for k in (rows[0].keys() if rows else []):
        try:
            out[k] = np.array([float(r[k]) if r[k] != '' else np.nan for r in rows])
        except ValueError:
            out[k] = np.array([r[k] for r in rows])
    return out


class Track:
    def __init__(self, direction, data_dir=DATA, terminal=False):
        m = read_csv(Path(data_dir) / 'route_map.csv', where=lambda r: r['direction'] == direction)
        self.s = m['s']
        self.x = m['x'] + PG_SHIFT
        self.y = m['y']
        self.lat, self.lon = m['lat'], m['lon']
        self.g = m['grade']
        self.heading = np.unwrap(m['heading'])               # рад, курс линии (для ориентации в Odometry)
        z = m['alt'] - 3.0                                   # антенны на 3,0 м над рельсом (tf)
        pts = json.load(open(Path(data_dir) / f'pathgraph_{direction}.json'))['points']
        px = np.array([p['x'] for p in pts]); py = np.array([p['y'] for p in pts]); pz = np.array([p['z'] for p in pts])
        # пути конечной за концом линии (OSM, build_terminal.py): за s_entry — взвешенное среднее по веткам
        self.term = None
        f = Path(data_dir) / f'terminal_{direction}.json'
        if terminal and f.exists():
            J = json.load(open(f))
            self.term = (J['s_entry'], np.array(J['prob']), [np.array(b) for b in J['branches']])
        # высота рельса из Pathgraph, где он есть (ближайшая точка в пределах 2 м)
        self.z = z.copy()
        for k in range(0, len(self.x)):
            d2 = (px - self.x[k]) ** 2 + (py - self.y[k]) ** 2
            j = int(np.argmin(d2))
            if d2[j] < 4.0:
                self.z[k] = pz[j]

    def xyz(self, s):
        z = float(np.interp(s, self.s, self.z))
        if self.term is not None and s > self.term[0]:
            s0, p, B = self.term
            d = s - s0                                   # путь за точкой входа; ветки — с шагом 1 м
            pts = np.array([b[min(int(d), len(b) - 1)] for b in B])
            return float(p @ pts[:, 0]), float(p @ pts[:, 1]), z
        return float(np.interp(s, self.s, self.x)), float(np.interp(s, self.s, self.y)), z

    def yaw(self, s):
        return float(np.interp(s, self.s, self.heading))

    def grade(self, s):
        return float(np.interp(s, self.s, self.g))

    def locate(self, lat, lon, exact=False):
        """Ближайшая точка линии к (lat, lon): (s, расстояние, м). Без проекций — локально плоско.
        exact=True — проекция на отрезок между соседними точками (иначе s с шагом карты 1 м)."""
        k = np.cos(np.radians(lat))
        X, Y = (self.lon - lon) * 111_320 * k, (self.lat - lat) * 110_540
        d = np.hypot(X, Y)
        j = int(np.argmin(d))
        if not exact:
            return float(self.s[j]), float(d[j])
        j = min(max(j, 1), len(self.s) - 2)
        ax, ay, bx, by = X[j - 1], Y[j - 1], X[j + 1], Y[j + 1]
        dx, dy = bx - ax, by - ay
        u = float(np.clip(-(ax * dx + ay * dy) / (dx * dx + dy * dy + 1e-12), 0.0, 1.0))
        px, py = ax + u * dx, ay + u * dy
        return float(self.s[j - 1] + u * (self.s[j + 1] - self.s[j - 1])), float(np.hypot(px, py))
