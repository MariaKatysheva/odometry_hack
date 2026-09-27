"""
Начальная выставка по GNSS (разрешено заданием): направление и путь s0 base_link в момент первой RTK-точки.

RTK-точки первых `window` секунд проецируются на обе линии; направление — линия, к которой ближе все точки окна
(не только первая: на конечной «Щукинская» конец AB и начало BA проходят в метрах друг от друга). Кроме того,
трамвай не может стоять в самом конце своей линии, и путь по ней не может убывать при движении.
Если точки дальше `max_dist` от обеих линий (депо, другой путь) — None: относительная одометрия.

Зависимости: только numpy.
"""
import numpy as np

ANT_X = {'master': -9.873, 'rover': 2.563}                 # tf антенн в base_link (вдоль вагона), м
RTK = 2                                                    # NavSatFix.status.status для RTK в данных


def init_from_gnss(fixes, tracks, window=20.0, max_dist=30.0, odo=None):
    """fixes: список (receiver, stamp, lat, lon, status); tracks: {'AB': Track, 'BA': Track};
    odo(t) — путь по колёсам к моменту t (м, относительный), если есть.
    s0 — путь base_link в момент первой RTK-точки: медиана по всем точкам окна (обоих приёмников)
    значений s_i − x_антенны − (odo(t_i) − odo(t_первой)). Первая точка после захвата RTK бывает неточной
    (на записи жюри — на 3 м), поэтому одной точке не доверяем.
    Возвращает dict(stamp, direction, s0, dist) или None."""
    g = sorted((f for f in fixes if f[4] == RTK), key=lambda f: f[1])
    if not g:
        return None
    t_first, rc_first = g[0][1], g[0][0]
    g = [f for f in g if f[1] <= t_first + window]
    one = [f for f in g if f[0] == rc_first]                # ход пути — по одному приёмнику
    best = None
    for d, tr in tracks.items():
        dist = np.array([tr.locate(f[2], f[3])[1] for f in g])
        s = np.array([tr.locate(f[2], f[3])[0] for f in one]) - ANT_X[rc_first]
        score = float(np.mean(dist))
        score += 1e3 * (tr.s[-1] - s[0] < 30.0)             # в конце линии — ехать некуда
        score += 1e3 * (s[-1] - s[0] < -5.0)                # движение против направления линии
        if best is None or score < best[0]:
            best = (score, d, float(s[0]), float(np.mean(dist)))
            if odo is not None:                                  # медиана по окну, приведённая к t_первой
                s_all = np.array([tr.locate(f[2], f[3], exact=True)[0] - ANT_X[f[0]] - (odo(f[1]) - odo(t_first))
                                  for f in g])
                best = (score, d, float(np.median(s_all)), float(np.mean(dist)))
    _, d, s0, dist = best
    if dist > max_dist:
        return None
    return dict(stamp=float(t_first), direction=d, s0=s0, dist=dist)
