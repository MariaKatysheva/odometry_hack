#!/usr/bin/env python3
"""
Запуск оценщика (IMM + признак 2) на одном рейсе в потоковом режиме — без ROS.

    python3 run_offline.py <папка рейса> [выход.csv]

Папка рейса — любая из двух:
    data/<рейс>/    исходный rosbag2 (.db3) — читается напрямую (tram_odometry/bag.py);
    export/<рейс>/  выгрузка export_all.py: wheels.csv, cmd.csv, gnss_fix.csv.
GNSS нужен только для стартовой точки; если его нет — относительная одометрия от старта.
Выход (строка на каждое сообщение колёс и контроллера): stamp, v (м/с), x, y, z (Pathgraph), s (м вдоль пути), sigma_v, sigma_s (1σ), direction, slip_both, stopped.
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tram_odometry.estimator import Estimator, Params          # noqa: E402
from tram_odometry.track import DATA, Track                    # noqa: E402
from tram_odometry.bag import read_bag                          # noqa: E402
from tram_odometry.start import init_from_gnss                   # noqa: E402

# калибровка неуверенности выхода по 18 обучающим рейсам (unc_check.py): 95 % ошибок в ±2σ на чистой езде
SIGMA_CALIB = dict(sigma_v_scale=1.534, kappa_sigma=0.0062)
# ложная тревога «обе тележки проскальзывают» (найдено на эталоне жюри 30618_88aea4d9, проверено check_fix.py):
# обе тележки ровно 0 полсекунды → стоим; слип не подтвердился → путь за эпизод по колёсам
SLIP_FIX = dict(stop_exit=0.5, slip_rollback=True,
                # при u = 0 модель ждёт выбег, но трамвай бывает тормозит тормозом вне топика → не входим (check_fix.py)
                slip_no_neutral=True)


def load(run_dir):
    """Колёса, контроллер, GNSS — из .db3 (data/) или из CSV (export/)."""
    if any(run_dir.glob('*.db3')):
        return read_bag(run_dir)
    fix = run_dir / 'gnss_fix.csv'
    return (pd.read_csv(run_dir / 'wheels.csv'), pd.read_csv(run_dir / 'cmd.csv'),
            pd.read_csv(fix) if fix.exists() else pd.DataFrame(columns=['receiver', 'stamp', 'lat', 'lon', 'status']))


def main():
    run_dir = Path(sys.argv[1])
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(f'estimate_{run_dir.name}.csv')
    tracks = {d: Track(d) for d in ('AB', 'BA')}
    params = Params(anchors=False, gate_v=1e9, gp_sigma='const', imm=True, **SIGMA_CALIB, **SLIP_FIX,
                    drive_table=pd.read_csv(DATA / 'drive_gp_table.csv'),
                    slip_calib=json.load(open(DATA / 'slip_calib.json')))
    w, c, fix = load(run_dir)
    w, c = w.sort_values('stamp'), c.sort_values('stamp')
    zw = w[['front', 'rear']].mean(axis=1).fillna(0).to_numpy() / 3.6          # путь по колёсам — для выставки
    sw = np.r_[0, np.cumsum(0.5 * (zw[1:] + zw[:-1]) * np.diff(w.stamp.to_numpy()))]
    odo = lambda tt: float(np.interp(tt, w.stamp.to_numpy(), sw))
    ini = init_from_gnss(list(fix[['receiver', 'stamp', 'lat', 'lon', 'status']].itertuples(index=False, name=None)),
                         tracks, odo=odo)
    if ini is None:                                            # без GNSS: относительная одометрия
        ini = dict(stamp=float(w.stamp.iloc[0]), direction='AB', s0=0.0, dist=np.nan, relative=True)
    est = Estimator(tracks[ini['direction']], params)
    est.init(ini['stamp'], ini['s0'])
    ev = pd.concat([w.assign(kind='w'), c.assign(kind='c')]).sort_values('stamp', kind='stable')
    rows = []
    for e in ev[ev.stamp > ini['stamp']].itertuples(index=False):
        if e.kind == 'c':                         # контроллер: выдача по прогнозу, фильтр не трогаем
            est.on_cmd(e.stamp, e.position)
            v, (x, y, z) = est.peek(e.stamp)
        else:
            est.on_wheels(e.stamp, e.front, e.rear)
            v, (x, y, z) = est.output()
        sv, ss = est.sigma()
        if ini.get('relative'):
            x, y, z = est.s - ini['s0'], 0.0, 0.0
        rows.append((e.stamp, v, x, y, z, est.s, sv, ss, ini['direction'], est.in_slip, est.stopped))
    R = pd.DataFrame(rows, columns=['stamp', 'v', 'x', 'y', 'z', 's', 'sigma_v', 'sigma_s',
                                    'direction', 'slip_both', 'stopped'])
    R.to_csv(out, index=False)
    print(f'старт: {ini}; сообщений: {len(R)}; результат: {out}')


if __name__ == '__main__':
    main()
