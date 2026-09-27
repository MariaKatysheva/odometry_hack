"""
Резервная одометрия трамвая: оценка скорости и положения base_link по колёсам и контроллеру.

Потоковый интерфейс (как будет в ROS 2 ноде): сообщения подаются по одному в порядке времени,
будущее не используется.

    est = Estimator(track, params)
    est.init(t, s0)                              # старт: положение на рельсах (из GNSS первых секунд)
    est.on_cmd(t, position)                      # /vehicle/driver_position_cmd
    est.on_wheels(t, front_kmh, rear_kmh)        # /vehicle/*_bogie_velocity (NaN — сообщения нет)
    v, (x, y, z) = est.output()                  # /result/velocity, /result/position

Скорость: одномерный фильтр Калмана. Прогноз — по физике (контроллер, уклон с карты), поправка —
по тележкам; доверие к тележке снижается пропорционально расхождению тележек между собой (и больше
у той, что дальше от прогноза) — без порогов. Пропуск сообщения тележки — просто нет поправки.
"""
from dataclasses import dataclass

import numpy as np

G = 9.81


@dataclass
class Params:
    k: float = 1 / 3.6                    # масштаб колёс: км/ч датчика -> м/с (с учётом износа)
    phys: tuple = (0.0976, 6.0061, 0.0516, -0.1269, 0.0068)   # α, P/m, c0, β1, β2 (wheel_map.py)
    sigma_wheel: float = 0.05             # м/с — разброс показания тележки
    sigma_acc: float = 1.0                # м/с² — неточность физики (подобрано на train, tune_speed.py)
    step: float = 0.1                     # с — шаг прогноза
    gamma: float = 10.0                   # насколько сильно расхождение тележек снижает доверие (tune_speed.py)
    kappa_sigma: float = 0.015            # априорный разброс масштаба колёс (по рейсам: 0,984…1,016)
    q_s: float = 1e-3                     # м²/м — рост неуверенности положения на метр пути
    q_kappa: float = 1e-10                # 1/м — дрейф масштаба на метр пути
    sigma_v_scale: float = 1.0            # множитель σ скорости на выходе (калибровка: unc_check.py)
    gate: float = 9.0                     # χ²-ворота привязки к месту остановки (3σ)
    gate_v: float = 9.0                   # χ²-ворота «колёса против физики» для обеих тележек (3σ)
    anchors: bool = True                  # привязываться к местам остановок
    drive_table: object = None            # таблица GP (u, v, a_mean, a_std): вместо формулы привода и σ_a
    gp_sigma: str = 'table'               # шум процесса с GP: 'table' — σ_GP, 'const' — σ_a, 'add' — √(σ_a² + σ_GP²)
    slip_calib: object = None             # калибровка детектора «обе тележки» (calib_slip.py) или None
    ml_detector: object = None            # веса MLP (ml_detector.py): вход в режим по вероятности > 0,5
    lnn: object = None                    # LNN (lnn_detector.py): dict(net, mu, sd, seq) — вход в режим по p > 0,5
    lnn_every: int = 3                    # вызывать сеть каждые N сообщений
    lnn_r: object = None                  # LNN end-to-end (lnn_e2e.py): множители шума тележек R_i = σ_w²·exp(r_i)
    imm: bool = False                     # IMM: режимы норма / передняя врёт / задняя врёт / обе врут
    imm_stay: tuple = (0.995, 0.9, 0.9, 0.9)  # вероятность остаться в режиме за сообщение
    imm_slip_sigma: tuple = (0.5, 0.3)    # шум «врущей» тележки: σ = a + b·v, м/с
    imm_q_both: float = 3.0               # множитель шума процесса в режиме «обе врут»
    slip_exit_k: float = 3.0              # выход: колёса в пределах k·σ прогноза
    enter_scale: float = 1.0              # множитель порога входа в режим «обе» (к калиброванному 99,9 %)
    bogie_agree: float = 0.06             # м/с — тележки «согласны» (98 % чистой езды: |z_F − z_R| ≤ ~0,06)
    slip_t_max: float = 10.0              # с — принудительный выход (защита от «залипания»)
    mass_adapt: bool = False              # θ = m_ном/m_факт: постоянна на перегоне, скачок только на остановке
    mass_sigma: float = 0.10              #   скачок неопределённости θ при трогании после остановки (P_θ += σ²)
    mass_obs_sigma: float = 0.2           #   м/с² — шум наблюдения θ по ускорению колёс на тяге
    slip_no_neutral: bool = False         # не входить в режим «обе проскальзывают» при u = 0 (нейтраль: тормоз вне топика)
    enter_scale_brake: float = 1.0        # множитель порога входа при u ≤ 0
    slip_traction_only: bool = False     # вход в режим «обе проскальзывают» только на тяге (u > 0): при u ≤ 0
                                          #   торможение бывает другим тормозом, которого нет в топике контроллера
    stop_exit: object = None              # с — обе тележки ровно 0 столько подряд → стоим, выход из slip
    slip_rollback: bool = False           # выход не по согласию скоростей (таймаут, остановка) = ложная тревога:
                                          #   путь за эпизод пересчитывается по колёсам
    speed_source: str = 'filter'          # 'filter' — наш фильтр скорости; 'wheels' — скорость как у колёс
                                          # (имитация внешнего EKF команды; слой положения тот же)


def a_phys(p, u, v):
    al, P, c0, b1, b2 = p
    if u > 0:
        return min(al * u, P / max(v, 1.0)) - c0
    if u < 0:
        return b1 * min(-u, 7) + b2 * max(-u - 8, 0) - c0
    return -c0


class Estimator:
    def __init__(self, track, params=Params(), stops=None):
        self.track, self.p = track, params
        self.stops = stops                    # таблица мест остановок этого направления: s, std
        self.s = self.v = 0.0
        self.P = 1.0                          # неуверенность скорости (по колёсам, номинальный масштаб)
        self.kappa = 1.0                      # поправка масштаба колёс (износ)
        self.Ps = np.diag([1.0, params.kappa_sigma ** 2])   # ковариация [s, κ]
        self.t = None
        self.u = 0
        self.gp = None
        if params.drive_table is not None:
            T = params.drive_table
            if hasattr(T, 'groupby'):                        # pandas.DataFrame
                T = {k: T[k].to_numpy() for k in ('u', 'v', 'a_mean', 'a_std')}
            self.gp = {}                                     # dict столбец → массив (track.read_csv)
            for u in np.unique(T['u']):
                i = T['u'] == u
                self.gp[int(u)] = (T['v'][i], T['a_mean'][i], T['a_std'][i])
        self.stopped = False
        self.slip_both = False
        self.in_slip, self.slip_t0, self.hist = False, 0.0, []
        self.slip_s0, self.slip_wd, self.slip_last, self.zero_t0 = 0.0, 0.0, None, None
        self.n_false_slip = 0
        self.theta, self.P_theta, self.m_stopped = 1.0, 0.10 ** 2, True   # масса (отклик на ручку)
        self.emax, self.t_du, self.du_last, self.p_slip = [], 0.0, 0, 0.0
        self.lbuf, self.lz, self.n_msg = [], [], 0
        self.h_r, self.r_mult = None, (0.0, 0.0)
        self.mu = np.array([0.97, 0.01, 0.01, 0.01])      # вероятности режимов IMM
        self.vm = np.zeros(4); self.Pm = np.ones(4)        # скорость и дисперсия в каждом режиме
        self.n_anchor = 0

    def init(self, t, s0, v0=0.0):
        self.t, self.s, self.v = t, s0, v0

    def on_cmd(self, t, position):
        if int(position) != self.u:
            self.t_du, self.du_last = t, int(position) - self.u
        self.u = int(position)

    def peek(self, t, dt_max=0.5):
        """Выход на момент t между сообщениями колёс: прогноз по модели привода, состояние фильтра не меняется."""
        dt = float(np.clip(t - self.t, 0.0, dt_max)) if self.t is not None else 0.0
        a, _ = self._model_acc()
        v = max(self.v + a * dt, 0.0)
        s = self.s + self.kappa * 0.5 * (self.v + v) * dt
        return self.kappa * v, self.track.xyz(s)

    def on_wheels(self, t, front, rear):
        if self.t is None:
            return
        self._predict(t)
        self._update(t, front, rear)

    def _predict(self, t):
        dt = max(t - self.t, 0.0)
        self.t = t
        # прогноз по физике (контроллер, уклон в текущей точке) подшагами — и через пропуски
        n = max(1, int(np.ceil(dt / self.p.step)))
        h = dt / n
        for _ in range(n):
            if self.gp is not None:
                vg, am, asd = self.gp[int(np.clip(self.u, -15, 15))]
                a = self._th() * float(np.interp(self.v, vg, am)) - G * self.track.grade(self.s)
                sg = float(np.interp(self.v, vg, asd))
                sig = {'table': sg, 'const': self.p.sigma_acc,
                       'add': (self.p.sigma_acc ** 2 + sg ** 2) ** 0.5}[self.p.gp_sigma]
            else:
                a = a_phys(self.p.phys, self.u, self.v) - G * self.track.grade(self.s)
                sig = self.p.sigma_acc
            v0 = self.v
            self.v = max(self.v + a * h, 0.0)
            ds = 0.5 * (v0 + self.v) * h
            self.s += self.kappa * ds
            F = np.array([[1.0, ds], [0.0, 1.0]])
            self.Ps = F @ self.Ps @ F.T + np.diag([self.p.q_s * ds, self.p.q_kappa * ds])
            self.P += (sig * h) ** 2

    def _update(self, t, front, rear):
        # ---- режим «обе тележки проскальзывают» (калибровка по чистой езде, calib_slip.py)
        zs_all = [z * self.p.k for z in (front, rear) if z is not None and np.isfinite(z)]
        if self.p.mass_adapt:
            self._mass_step(t, zs_all)
        if self.p.slip_calib is not None and zs_all:
            cal = self.p.slip_calib
            zb = min(zs_all, key=lambda z: abs(z - self.v))          # тележка, ближайшая к прогнозу
            self.hist = (self.hist + [(t, zb)])[-(cal['window'] + 1):]
            if self.in_slip:
                zm = float(np.mean(zs_all))
                if self.slip_last is not None:                         # путь по колёсам за эпизод
                    self.slip_wd += 0.5 * (self.slip_last[1] + zm) * max(t - self.slip_last[0], 0.0)
                self.slip_last = (t, zm)
                self.zero_t0 = (self.zero_t0 or t) if all(z == 0 for z in zs_all) else None
                ok = abs(zb - self.v) < self.p.slip_exit_k * (self.P + self.p.sigma_wheel ** 2) ** 0.5
                stop = (self.p.stop_exit is not None and self.zero_t0 is not None
                        and t - self.zero_t0 >= self.p.stop_exit)
                if ok and not stop or t - self.slip_t0 > self.p.slip_t_max or stop:
                    if (not ok or stop) and self.p.slip_rollback:           # не подтвердился → верим колёсам
                        self.s = self.slip_s0 + self.kappa * self.slip_wd
                        self.n_false_slip += 1
                        ok = False
                    if not ok:                                         # принудительно: верим колёсам заново
                        self.v, self.P = zb, self.p.sigma_wheel ** 2
                    self.in_slip = False
            elif self.p.lnn is not None:
                self._lnn_step(t, front, rear)
                if (self.p_slip > self.p.lnn.get('thr', 0.5) and self.v > 1.0
                        and (len(zs_all) < 2 or abs(zs_all[0] - zs_all[1]) < self.p.bogie_agree)):
                    self.in_slip, self.slip_t0 = True, t
                    self.slip_s0, self.slip_wd, self.slip_last, self.zero_t0 = self.s, 0.0, (t, zb), None
            elif self.p.ml_detector is not None and self.v > 1.0 and len(self.hist) > cal['window']:
                t0, z0 = self.hist[0]
                aw = (zb - z0) / max(t - t0, 1e-3)
                a_m, s_m = self._model_acc()
                ea = (aw - a_m) / (s_m ** 2 + cal['sigma_wheel_acc'] ** 2) ** 0.5
                self.emax = [(tt, x) for tt, x in self.emax if t - tt < 1.0] + [(t, abs(ea))]
                f = dict(u=self.u, du=self.du_last if t - self.t_du < 2.0 else 0, t_since_du=min(t - self.t_du, 30),
                         v=self.v, bogie_diff=abs(zs_all[0] - zs_all[1]) if len(zs_all) == 2 else 0.0,
                         acc_best=aw, e_acc=ea, e_acc_max1s=max(x for _, x in self.emax),
                         resid_best=(zb - self.v) / (self.P + 0.05 ** 2) ** 0.5)
                self.p_slip = self._mlp(f)
                if self.p_slip > 0.5:
                    self.in_slip, self.slip_t0 = True, t
                    self.slip_s0, self.slip_wd, self.slip_last, self.zero_t0 = self.s, 0.0, (t, zb), None
            elif (self.p.ml_detector is None and self.v > 1.0 and len(self.hist) > cal['window']
                  and (len(zs_all) < 2 or abs(zs_all[0] - zs_all[1]) < self.p.bogie_agree)):
                # только если тележки согласны: иначе проскальзывание ловит взвешивание по расхождению
                t0, z0 = self.hist[0]
                aw = (zb - z0) / max(t - t0, 1e-3)
                a_m, s_m = self._model_acc()
                e = (aw - a_m) / (s_m ** 2 + cal['sigma_wheel_acc'] ** 2) ** 0.5
                thr = cal['enter'] * self.p.enter_scale * (self.p.enter_scale_brake if self.u <= 0 else 1.0)
                if (abs(e) > thr and (self.u > 0 or not self.p.slip_traction_only)
                        and not (self.p.slip_no_neutral and self.u == 0)):
                    self.in_slip, self.slip_t0 = True, t
                    self.slip_s0, self.slip_wd, self.slip_last, self.zero_t0 = self.s, 0.0, (t, zb), None
            if self.in_slip:
                return                                                 # колёсам не верим, едем по модели
        elif not zs_all:
            self.hist = []
        # ---- IMM: вместо жёсткого переключения — параллельные режимы с вероятностями
        if self.p.imm:
            self._imm_update(front, rear)
            stopped_imm = all(z == 0 for z in (front, rear) if z is not None and np.isfinite(z)) and \
                any(z is not None and np.isfinite(z) for z in (front, rear))
            if stopped_imm:
                self.v = 0.0; self.vm[:] = 0.0; self.v_post = 0.0
                if not self.stopped:
                    self._anchor()
            self.stopped = stopped_imm
            self.in_slip = False
            return
        # ---- LNN end-to-end: сеть сама задаёт доверие к каждой тележке (как при обучении в lnn_e2e.py)
        if self.p.lnn_r is not None:
            f, dtm = self._raw_feats(t, front, rear)
            import torch
            L = self.p.lnn_r
            x = torch.tensor(((np.array(f, np.float32) - L['mu']) / L['sd'])[None])
            if self.h_r is None:
                self.h_r = torch.zeros(1, L['net'].units)
            with torch.no_grad():
                r, self.h_r = L['net'].step(x, self.h_r, torch.tensor([[dtm]], dtype=torch.float32))
            r = r[0].numpy()
            for i, z in enumerate((front, rear)):
                if z is None or not np.isfinite(z):
                    continue
                R = self.p.sigma_wheel ** 2 * float(np.exp(r[i]))
                K = self.P / (self.P + R)
                self.v = max(self.v + K * (z * self.p.k - self.v), 0.0)
                self.P *= (1 - K)
            zs0 = [z for z in (front, rear) if z is not None and np.isfinite(z)]
            stopped = len(zs0) > 0 and all(z == 0 for z in zs0)
            if stopped:
                self.v = 0.0
            self.stopped = stopped
            return
        # поправка по тележкам. Доверие снижается по РАСХОЖДЕНИЮ ТЕЛЕЖЕК между собой: если они
        # согласны — верим им, даже если физика предсказала иное (значит, ошиблась физика);
        # если расходятся — меньше веса у той, что дальше от прогноза (юз, слип, сбой одной).
        zs = [z * self.p.k for z in (front, rear) if z is not None and np.isfinite(z)]
        if len(zs) == 2:
            d2 = (zs[0] - zs[1]) ** 2
            r = [abs(z - self.v) for z in zs]
            share = [ri / (r[0] + r[1] + 1e-9) for ri in r]
            Rs = [self.p.sigma_wheel ** 2 + self.p.gamma * d2 * sh for sh in share]
        else:
            Rs = [self.p.sigma_wheel ** 2] * len(zs)
        # обе тележки против физики: общая невязка с прогнозом больше, чем ожидается (χ² > gate_v) —
        # колёсам в целом верим меньше. Неуверенность прогноза P растёт, пока колёса игнорируются,
        # поэтому при долгом расхождении фильтр сам возвращается к колёсам (нет «залипания»).
        self.slip_both = False
        if zs:
            rc = float(np.mean(zs)) - self.v
            S = self.P + self.p.sigma_wheel ** 2
            if rc ** 2 > self.p.gate_v * S:
                Rs = [R + rc ** 2 for R in Rs]
                self.slip_both = True
        for z, R in zip(zs, Rs):
            K = self.P / (self.P + R)
            self.v = max(self.v + K * (z - self.v), 0.0)
            self.P *= (1 - K)
        if self.p.speed_source == 'wheels' and zs:
            self.v = float(np.mean(zs))
        # стоянка: все пришедшие тележки показывают ровно 0 (так ведут себя датчики при остановке)
        stopped = len(zs) > 0 and all(z == 0 for z in zs)
        if stopped:
            self.v = 0.0
            if not self.stopped:
                self._anchor()
        self.stopped = stopped

    def _imm_update(self, front, rear):
        """IMM на скорость. Режимы: 0 — норма; 1 — передняя врёт; 2 — задняя врёт; 3 — обе врут.
        Перед вызовом общий прогноз уже сделан: self.v, self.P сдвинуты моделью привода от апостериорных
        self.v_post, self.P_post. Шаги IMM: смешивание по матрице переходов → прогноз каждого режима
        (тот же сдвиг dv, свой шум процесса) → обновление своими шумами тележек → вероятности режимов
        по правдоподобию → взвешенная оценка."""
        n = 4
        st = np.array(self.p.imm_stay)
        Pi = np.full((n, n), 0.0)
        for i in range(n):
            Pi[i, :] = (1 - st[i]) / (n - 1)
            Pi[i, i] = st[i]
        if not hasattr(self, 'v_post'):
            self.v_post, self.P_post = self.v, self.P
            self.vm[:] = self.v; self.Pm[:] = self.P
        dv, dq = self.v - self.v_post, max(self.P - self.P_post, 0.0)
        c = self.mu @ Pi
        w = (Pi * self.mu[:, None]) / np.maximum(c[None, :], 1e-12)
        v0 = w.T @ self.vm
        P0 = np.array([w[:, k] @ (self.Pm + (self.vm - v0[k]) ** 2) for k in range(n)])
        qm = np.array([1.0, 1.0, 1.0, self.p.imm_q_both])
        v0, P0 = v0 + dv, P0 + dq * qm
        zs = [(i, z * self.p.k) for i, z in enumerate((front, rear)) if z is not None and np.isfinite(z)]
        a, b = self.p.imm_slip_sigma
        L = np.ones(n)
        for k in range(n):
            vk, Pk = v0[k], P0[k]
            for i, z in zs:
                bad = (k == 3) or (k == 1 and i == 0) or (k == 2 and i == 1)
                R = (a + b * max(vk, 0.0)) ** 2 if bad else self.p.sigma_wheel ** 2
                S = Pk + R
                r = z - vk
                L[k] *= np.exp(-0.5 * r * r / S) / np.sqrt(2 * np.pi * S)
                K = Pk / S
                vk, Pk = vk + K * r, (1 - K) * Pk
            v0[k], P0[k] = max(vk, 0.0), Pk
        mu = c * L
        self.mu = mu / mu.sum() if mu.sum() > 1e-300 else c
        self.vm, self.Pm = v0, P0
        self.v = float(self.mu @ self.vm)
        self.P = float(self.mu @ (self.Pm + (self.vm - self.v) ** 2))
        self.v_post, self.P_post = self.v, self.P

    def _raw_feats(self, t, front, rear):
        """Признаки из сырых сигналов, как в lnn_detector.run_feats: (признаки, Δt)."""
        zf = front / 3.6 if front is not None and np.isfinite(front) else None
        zr = rear / 3.6 if rear is not None and np.isfinite(rear) else None
        zf, zr = (zf if zf is not None else zr), (zr if zr is not None else zf)
        zf, zr = zf or 0.0, zr or 0.0
        zm = (zf + zr) / 2
        self.lz = [(tt, z) for tt, z in self.lz if t - tt <= 0.6] + [(t, zm)]
        t0, z0 = next(((tt, z) for tt, z in self.lz if t - tt <= 0.5), self.lz[0])
        a_w = (zm - z0) / max(t - t0, 1e-3)
        if self.gp is not None:
            vg, am = self.gp[int(np.clip(self.u, -15, 15))][:2]
            a_m = float(np.interp(zm, vg, am))
        else:
            a_m = a_phys(self.p.phys, self.u, zm)
        a_m -= G * self.track.grade(self.s)
        du = self.du_last if t - self.t_du < 2.0 else 0
        dt = (t - self._t_raw) if getattr(self, '_t_raw', None) is not None else 0.1
        self._t_raw = t
        return [self.u / 15, du / 15, zf / 15, zr / 15, zf - zr, a_w, a_m], dt

    def _lnn_step(self, t, front, rear):
        """Признаки как в lnn_detector.run_feats (сырые сигналы), окно seq сообщений, CfC."""
        import torch
        L = self.p.lnn
        zf = front / 3.6 if front is not None and np.isfinite(front) else None
        zr = rear / 3.6 if rear is not None and np.isfinite(rear) else None
        zf, zr = (zf if zf is not None else zr), (zr if zr is not None else zf)
        zf, zr = zf or 0.0, zr or 0.0
        zm = (zf + zr) / 2
        self.lz = [(tt, z) for tt, z in self.lz if t - tt <= 0.6] + [(t, zm)]
        t0, z0 = next(((tt, z) for tt, z in self.lz if t - tt <= 0.5), self.lz[0])
        a_w = (zm - z0) / max(t - t0, 1e-3)
        vg, am = self.gp[int(np.clip(self.u, -15, 15))][:2] if self.gp is not None else (None, None)
        a_m = (float(np.interp(zm, vg, am)) if vg is not None else a_phys(self.p.phys, self.u, zm)) \
            - G * self.track.grade(self.s)
        du = self.du_last if t - self.t_du < 2.0 else 0
        dt = (t - self.lbuf[-1][0]) if self.lbuf else 0.1
        f = [self.u / 15, du / 15, zf / 15, zr / 15, zf - zr, a_w, a_m]
        self.lbuf = (self.lbuf + [(t, f, dt)])[-L['seq']:]
        self.n_msg += 1
        if len(self.lbuf) == L['seq'] and self.n_msg % self.p.lnn_every == 0:
            x = (np.array([b[1] for b in self.lbuf], np.float32) - L['mu']) / L['sd']
            ts = np.array([[b[2]] for b in self.lbuf], np.float32)
            with torch.no_grad():
                self.p_slip = float(torch.sigmoid(L['net'](torch.tensor(x[None]), torch.tensor(ts[None]))))

    def _mlp(self, f):
        """Прямой проход MLP на numpy (веса из ml_detector.json): вероятность проскальзывания обеих."""
        m = self.p.ml_detector
        x = (np.array([f[k] for k in m['features']]) - m['mu']) / m['sd']
        h = np.maximum(0, x @ np.array(m['W'][0]) + np.array(m['b'][0]))
        z = float(h @ np.array(m['W'][1]).ravel() + m['b'][1][0])
        return 1.0 / (1.0 + np.exp(-z))

    def _th(self):
        return self.theta if (self.p.mass_adapt and self.u != 0) else 1.0

    def _mass_step(self, t, zs_all):
        """θ = m_ном/m_факт: Q_θ = 0 на перегоне; при трогании после стоянки P_θ += σ² (масса сменилась
        на остановке); наблюдение — ускорение колёс на тяге, когда тележки согласны и слипа нет."""
        stopped = bool(zs_all) and all(z == 0 for z in zs_all)
        if self.m_stopped and zs_all and not stopped:           # трогание: возможен скачок массы
            self.P_theta += self.p.mass_sigma ** 2
        if zs_all:
            self.m_stopped = stopped
        if (self.u <= 0 or len(zs_all) < 2 or abs(zs_all[0] - zs_all[1]) > 0.05 or self.in_slip
                or self.v < 1.0 or len(self.hist) < 3 or self.mu[0] < 0.9):
            return
        (t0, z0), (t1, z1) = self.hist[0], self.hist[-1]
        if t1 - t0 < 0.2:
            return
        vg, am, _ = self.gp[int(np.clip(self.u, -15, 15))]
        a_tab = float(np.interp(self.v, vg, am))
        if a_tab < 0.3:
            return
        y = (z1 - z0) / (t1 - t0) + G * self.track.grade(self.s)   # наблюдаемое θ·a_tab
        S = a_tab ** 2 * self.P_theta + self.p.mass_obs_sigma ** 2
        K = self.P_theta * a_tab / S
        self.theta = float(np.clip(self.theta + K * (y - self.theta * a_tab), 0.6, 1.6))
        self.P_theta *= (1 - K * a_tab)

    def _model_acc(self):
        """Ускорение по модели привода с уклоном и его разброс (GP-таблица, если есть)."""
        grade = G * self.track.grade(self.s)
        if self.gp is not None:
            vg, am, asd = self.gp[int(np.clip(self.u, -15, 15))]
            return self._th() * float(np.interp(self.v, vg, am)) - grade, float(np.interp(self.v, vg, asd))
        return a_phys(self.p.phys, self.u, self.v) - grade, self.p.sigma_acc

    def _anchor(self):
        """Привязка к ближайшему (по χ²) известному месту остановки; уточняет и путь, и масштаб."""
        if not self.p.anchors or self.stops is None or len(self.stops) == 0:
            return
        y = self.stops.s.to_numpy() - self.s
        S = self.Ps[0, 0] + self.stops['std'].to_numpy() ** 2
        chi2 = y ** 2 / S
        i = int(np.argmin(chi2))
        if chi2[i] > self.p.gate:
            return                                        # встали не на известном месте
        K = self.Ps[:, 0] / S[i]
        self.s += K[0] * y[i]
        self.kappa = float(np.clip(self.kappa + K[1] * y[i], 0.95, 1.05))
        self.Ps = self.Ps - np.outer(K, self.Ps[0, :])
        self.n_anchor += 1

    def output(self):
        return self.kappa * self.v, self.track.xyz(self.s)

    def sigma(self):
        """Неуверенность выхода, 1σ: скорость (м/с) и положение вдоль пути (м)."""
        return self.p.sigma_v_scale * float(np.sqrt(self.P)), float(np.sqrt(self.Ps[0, 0]))
