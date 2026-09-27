"""
Чтение rosbag2 (.db3, sqlite3 + CDR) без ROS: колёса, контроллер, GNSS (для стартовой точки).

    wheels, cmd, fix = read_bag('data/30618_0e41eac3')

wheels: stamp, front, rear (км/ч; тележки с одинаковым stamp — в одной строке, пропуск — NaN)
cmd:    stamp, position (−15…15)
fix:    receiver, stamp, lat, lon, alt, status
Время — header.stamp сообщения, с. Зависимости: numpy, pandas.
"""
import sqlite3
import struct
from pathlib import Path

import pandas as pd


class _CDR:
    def __init__(self, buf):
        self.b, self.e, self.o = buf, '<' if buf[1] == 1 else '>', 4

    def _r(self, fmt, size):
        self.o += (-(self.o - 4)) % size
        v = struct.unpack_from(self.e + fmt, self.b, self.o)[0]
        self.o += size
        return v

    def stamp(self):
        sec, nsec = self._r('i', 4), self._r('I', 4)
        n = self._r('I', 4)
        self.o += n                                    # frame_id
        return sec + nsec * 1e-9


def _vel(b):
    c = _CDR(b); t = c.stamp(); return t, c._r('d', 8)


def _cmd(b):
    c = _CDR(b); t = c.stamp(); return t, c._r('b', 1)


def _fix(b):
    c = _CDR(b); t = c.stamp()
    status = c._r('b', 1); c._r('H', 2)
    return t, c._r('d', 8), c._r('d', 8), c._r('d', 8), status


def _topic(con, name, parser):
    row = con.execute('SELECT id FROM topics WHERE name=?', (name,)).fetchone()
    if row is None:
        return []
    out = []
    for (blob,) in con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (row[0],)):
        try:
            out.append(parser(blob))
        except Exception:
            pass                                       # битое сообщение пропускаем
    return out


def read_bag(bag_dir):
    db = next(Path(bag_dir).glob('*.db3'))
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    f = pd.DataFrame(_topic(con, '/vehicle/front_bogie_velocity', _vel), columns=['stamp', 'front'])
    r = pd.DataFrame(_topic(con, '/vehicle/rear_bogie_velocity', _vel), columns=['stamp', 'rear'])
    wheels = pd.merge(f.drop_duplicates('stamp'), r.drop_duplicates('stamp'), on='stamp', how='outer')
    wheels = wheels.sort_values('stamp').reset_index(drop=True)
    cmd = pd.DataFrame(_topic(con, '/vehicle/driver_position_cmd', _cmd), columns=['stamp', 'position'])
    cmd = cmd.drop_duplicates('stamp').sort_values('stamp').reset_index(drop=True)
    fixes = []
    for rc in ('master', 'rover'):
        for row in _topic(con, f'/sensing/gnss/{rc}/fix', _fix):
            fixes.append((rc,) + row)
    con.close()
    fix = pd.DataFrame(fixes, columns=['receiver', 'stamp', 'lat', 'lon', 'alt', 'status'])
    return wheels, cmd, fix
