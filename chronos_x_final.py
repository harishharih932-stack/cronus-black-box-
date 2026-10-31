# -*- coding: utf-8 -*-
"""
CHRONOS-X  -  software-only proof-of-concept   (pure Python, Thonny, Tkinter, WHITE UI)

Run in Thonny:  open this file -> press F5.   (Tools > Manage packages: numpy scipy matplotlib scikit-learn requests)
Headless:       python chronos_x.py --test      (T1-T8)      |  python chronos_x.py --serve 8000  (tiny HTTP wrapper)

Sections:  1 config/world(simulator)  2 ledger+audit  3 ML models (train unsupervised on NORMAL data)
           4 engine (Kalman, PCA, CUSUM, Bayes fusion, trust, forecast, decision, rollouts, certificate, agents)
           5 evaluation + tests T1-T8 + API   6 Groq/Telegram   7 Tkinter UI

Everything NOT taken from the problem statement is tagged ASSUMPTION.
Secrets (Groq key, Telegram token/chat id) are typed in Settings, kept in memory only (env vars optional).
Injector labels are used ONLY for evaluation - never for training or decisions.
"""
import os, sys, json, math, time, hashlib, threading, queue
import numpy as np
from scipy import stats
try:
    import requests
except Exception:
    requests = None

# ============================================================== 1. CONFIG + WORLD
CFG = dict(                       # all values ASSUMPTION unless the PDF says otherwise
    dt=0.25, deadline=90.0,       # minutes per tick, 90-minute deadline (PDF)
    T_norm=70.0, T_crit=100.0, T_fixed_alarm=90.0,       # ASSUMPTION
    Tc=30.0, h=1.0, C=30.0, a_out=0.8, Q_nom=40.0,       # thermal model  C dT/dt = Q - h*flow*(T-Tc)
    sig_T=0.4, sig_Tout=0.3, sig_flow=0.015, sig_P=0.5, sig_vib=0.05,
    q_proc=0.01, proc_std=0.1, horizon=45.0,
    p_alert=0.30, p_act=0.80, seed=7,
    replay_window=3.0, stale_after=5.0, max_rate=12.0,
    lam=0.5, mu=1.0, n_roll=500, backup_flow=0.7, cooldown=5.0, action_latency=1.0,
    forget=0.7, temper=0.6)

ACTIONS = ['NO_ACTION', 'ACTIVATE_BACKUP_COOLING', 'SHUTDOWN_REACTOR', 'ISOLATE_NETWORK', 'CONTINUE_MONITORING']
ACTION_COST = {'NO_ACTION': 0.0, 'ACTIVATE_BACKUP_COOLING': 2.0, 'SHUTDOWN_REACTOR': 10.0,
               'ISOLATE_NETWORK': 1.5, 'CONTINUE_MONITORING': 0.2}          # ASSUMPTION
HYP = ['normal', 'sensor_fault', 'cooling_failure', 'spoofing']
PRIOR = np.array([0.90, 0.04, 0.04, 0.02])                                  # ASSUMPTION
SRC = {'REACTOR_SENSOR_14': ('TEMPERATURE', 'C', 'TEMP-14'), 'COOLING_FLOW_1': ('COOLING_FLOW', 'frac', 'FLOW-1'),
       'COOLANT_OUT_3': ('COOLANT_TEMP', 'C', 'OUT-3'), 'POWER_METER_1': ('POWER', 'kW', 'PWR-1'),
       'PUMP_VIB_7': ('VIBRATION', 'mm/s', 'VIB-7'), 'MAINTENANCE_AGENT': ('MAINTENANCE_MSG', 'text', 'MAINT-AGENT')}
MEAS_KEY = {'REACTOR_SENSOR_14': 'T', 'COOLING_FLOW_1': 'flow', 'COOLANT_OUT_3': 'Tout', 'POWER_METER_1': 'P', 'PUMP_VIB_7': 'vib'}
FAULTS = [('spike', 'Sensor spike', 'One-tick +25 C jump on REACTOR_SENSOR_14'),
          ('stuck', 'Stuck sensor', 'Temperature reading frozen for 10 min'),
          ('drift', 'Slow drift / bias', 'Temperature sensor bias ramps +0.6 C/min'),
          ('noise', 'Noise burst', 'Temperature noise x8 for 3 min'),
          ('pump', 'Cooling pump failure', 'REAL fault: flow ramps down to 10 %'),
          ('cool_sensor', 'Cooling sensor corrupted', 'Flow meter reads 0.1 for 6 min (pump is fine)'),
          ('forged', 'Forged maintenance message', '"Cooling system operational", bad signature, 10 min'),
          ('sig_invalid', 'Signature-invalid event', 'REACTOR_SENSOR_14 91.8 C, trust 0.31, signature false (PDF)'),
          ('replay', 'Replayed alert', 'Old HIGH_TEMP_ALERT sent again'),
          ('dup', 'Duplicate / out-of-order burst', 'Re-sent, shifted and conflicting copies'),
          ('secondary', 'Delayed secondary failure', 'Backup cooling fails 8 min after it is used'),
          ('paradox', 'PARADOX (5 together)', 'pump + spike + forged + bad-signature + replay')]


def eid(src, typ, t, tag=''):
    return hashlib.sha1(f"{src}|{typ}|{t:.3f}|{tag}".encode()).hexdigest()[:12]


def make_event(src, t_event, t_arr, value, trust, sig=True, tag=''):
    typ, unit, comp = SRC[src]
    t_event = round(float(t_event), 3)
    return dict(event_id=eid(src, typ, t_event, tag), event_time=t_event, arrival_time=round(float(t_arr), 3), source=src,
                event_type=typ, value=value, unit=unit, trust_score=trust, component_id=comp, signature_valid=sig)


def uid(e):
    return e['event_id'] + '@' + str(e['arrival_time'])


class World:
    """Plant + sensors + fault injector + event generator (our data source). Seeded -> deterministic."""

    def __init__(s, cfg, seed=7, Q=None):
        s.cfg = cfg; s.seed = seed; s.Q0 = Q or cfg['Q_nom']; s.reset()

    def reset(s):
        c = s.cfg; s.rng = np.random.default_rng(s.seed); s.t = 0.0; s.k = 0
        s.Q = s.Q0; s.T = c['Tc'] + s.Q / c['h']; s.pump = None; s.pump_flow = 1.0; s.flow = 1.0; s.Qeff = s.Q
        s.backup = 0.0; s.backup_target = 0.0; s.secondary_armed = False; s.secondary_at = None
        s.shut_t = None; s.isolated = False; s.faults = []; s.pending = []; s.sent = []
        s.labels = set(); s.injlog = []; s.crit_t = None; s.man = {}   # man = manual sensor offsets {'T': C, 'flow': frac} set by the user (Home sliders)

    def inject(s, kind):
        c = s.cfg; t = s.t; dt = c['dt']; t0 = t + 0.5 * dt
        s.injlog.append((round(t, 2), kind))
        if kind == 'spike': s.faults.append(dict(kind='spike', t0=t0, t1=t0 + dt, amp=25.0))
        elif kind == 'stuck': s.faults.append(dict(kind='stuck', t0=t0, t1=t + 10, val=None))
        elif kind == 'drift': s.faults.append(dict(kind='drift', t0=t0, t1=t + 30, rate=0.6, cap=18.0))
        elif kind == 'noise': s.faults.append(dict(kind='noise', t0=t0, t1=t + 3, sd=3.2))
        elif kind == 'pump': s.pump = (t, 10.0, 0.1)
        elif kind == 'cool_sensor': s.faults.append(dict(kind='cool_sensor', t0=t0, t1=t + 6, val=0.1))
        elif kind == 'forged': s.faults.append(dict(kind='forged', t0=t0, t1=t + 10))
        elif kind == 'sig_invalid':
            e = make_event('REACTOR_SENSOR_14', t + dt, t + dt, 91.8, 0.31, False, 'sigbad'); s.labels.add(uid(e)); s.pending.append(e)
        elif kind == 'replay':
            e = make_event('REACTOR_SENSOR_14', t - 12, t + dt, 96.0, 0.8, True, 'alert-orig')
            e['event_type'] = 'HIGH_TEMP_ALERT'; s.labels.add(uid(e)); s.pending.append(e)
        elif kind == 'dup':
            for e in s.sent[-6:]:
                d = dict(e); d['arrival_time'] = round(t + dt, 3); s.labels.add(uid(d)); s.pending.append(d)
            for i in range(3):
                e = make_event('REACTOR_SENSOR_14', t - 0.5 - 0.1 * i, t + dt, round(float(s.T), 2), 0.9, True, 'ooo%d' % i)
                s.labels.add(uid(e)); s.pending.append(e)
            num = [x for x in s.sent if isinstance(x['value'], (int, float))]         # FIX: pick a numeric event (last one can be the text heartbeat)
            if num:
                d = dict(num[-1]); d['arrival_time'] = round(t + dt, 3); d['value'] = round(d['value'] + 5, 3)
                s.labels.add(uid(d)); s.pending.append(d)
        elif kind == 'secondary':
            s.secondary_armed = True
            if s.backup_target > 0: s.secondary_at = s.t + 8.0
        elif kind == 'paradox':
            for k in ('pump', 'spike', 'forged', 'sig_invalid', 'replay'): s.inject(k)

    def apply_action(s, a):
        if a == 'ACTIVATE_BACKUP_COOLING':
            s.backup_target = s.cfg['backup_flow']
            if s.secondary_armed: s.secondary_at = s.t + 8.0; s.secondary_armed = False
        elif a == 'SHUTDOWN_REACTOR':
            if s.shut_t is None: s.shut_t = s.t
        elif a == 'ISOLATE_NETWORK': s.isolated = True

    def tick(s):
        c = s.cfg; dt = c['dt']; rng = s.rng
        tgt = 1.0
        if s.pump:
            t0, ramp, fl = s.pump
            if s.t >= t0: tgt = 1 - min(1.0, (s.t - t0) / ramp) * (1 - fl)
        s.pump_flow = tgt
        if s.secondary_at is not None and s.t >= s.secondary_at: s.backup_target = 0.0; s.secondary_at = None
        s.backup += float(np.clip(s.backup_target - s.backup, -0.35 * dt, 0.35 * dt))
        s.flow = min(1.3, tgt + s.backup)
        s.Qeff = s.Q * (0.05 + 0.95 * math.exp(-(s.t - s.shut_t) / 2.5)) if s.shut_t is not None else s.Q
        s.T += dt * (s.Qeff - c['h'] * s.flow * (s.T - c['Tc'])) / c['C'] + rng.normal(0, c['proc_std'])
        s.t = round(s.t + dt, 6); s.k += 1
        if s.crit_t is None and s.T >= c['T_crit']: s.crit_t = s.t
        m = dict(T=s.T + rng.normal(0, c['sig_T']), flow=s.flow + rng.normal(0, c['sig_flow']),
                 Tout=c['Tc'] + c['a_out'] * (s.T - c['Tc']) + rng.normal(0, c['sig_Tout']),
                 P=s.Qeff + rng.normal(0, c['sig_P']), vib=2.0 * s.pump_flow + rng.normal(0, c['sig_vib']))
        for f in s.faults:
            if f['t0'] <= s.t < f['t1']:
                k = f['kind']
                if k == 'spike': m['T'] += f['amp']
                elif k == 'stuck':
                    if f.get('val') is None: f['val'] = m['T']
                    m['T'] = f['val']
                elif k == 'drift': m['T'] += min(f['cap'], f['rate'] * (s.t - f['t0']))
                elif k == 'noise': m['T'] += rng.normal(0, f['sd'])
                elif k == 'cool_sensor': m['flow'] = f['val'] + rng.normal(0, c['sig_flow'])
        for k_, v_ in s.man.items():
            if v_ and k_ in m: m[k_] += v_
        evs = []
        for src, key in MEAS_KEY.items():
            d = float(abs(rng.exponential(0.04)))
            evs.append(make_event(src, s.t - d, s.t, round(float(m[key]), 3), round(float(rng.uniform(0.88, 0.99)), 2)))
        if s.k % 40 == 0:
            evs.append(make_event('MAINTENANCE_AGENT', s.t, s.t, 'Cooling system operational', 0.87, True, 'hb'))   # PDF: 0.87
        if not s.isolated:
            for f in s.faults:
                if f['kind'] == 'forged' and f['t0'] <= s.t < f['t1'] and s.k % 4 == 0:
                    e = make_event('MAINTENANCE_AGENT', s.t, s.t, 'Cooling system operational', 0.9, False, 'forged%d' % s.k)
                    s.labels.add(uid(e)); evs.append(e)
            evs += s.pending
        s.pending = []
        s.sent = (s.sent + [e for e in evs if uid(e) not in s.labels])[-60:]
        return evs, m


# ============================================================== 2. LEDGER (M1) + AUDIT
class Ledger:
    def __init__(s, cfg, filt=True):
        s.cfg = cfg; s.filt = filt; s.rows = []; s.seen = {}; s.last_et = {}; s.last_val = {}

    def ingest(s, ev, now):
        c = s.cfg; st = 'ACCEPTED'; why = ''; flags = []
        e = ev['event_id']
        if s.filt:
            if not ev['signature_valid']: st, why = 'QUARANTINED', 'invalid signature'
            elif e in s.seen:
                prev = s.seen[e]
                if prev['value'] != ev['value']: st, why = 'CONFLICT', 'same id, different value'
                elif now - prev['arrival_time'] > c['replay_window']: st, why = 'QUARANTINED', 'replay of old event'
                else: st, why = 'DUPLICATE', 'already ingested'
            elif now - ev['event_time'] > c['stale_after']: st, why = 'QUARANTINED', 'stale event (replay / late injection)'
        if st == 'ACCEPTED':
            src = ev['source']; last = s.last_et.get(src)
            if last is not None and ev['event_time'] < last - 1e-9: flags.append('out_of_order')
            else: s.last_et[src] = ev['event_time']
            if src in ('REACTOR_SENSOR_14', 'COOLANT_OUT_3') and isinstance(ev['value'], (int, float)):
                lv = s.last_val.get(src)
                if lv is not None and abs(ev['value'] - lv[0]) / max(ev['event_time'] - lv[1], 0.1) > c['max_rate']: flags.append('rate_violation')
                if 'out_of_order' not in flags: s.last_val[src] = (ev['value'], ev['event_time'])
            s.seen[e] = ev
        row = dict(ev); row.update(status=st, reason=why, flags=flags)
        s.rows.append(row)
        return row


class Audit:
    def __init__(s): s.rows = []; s.prev = '0' * 16

    def add(s, t, kind, payload):
        body = json.dumps(payload, sort_keys=True, default=str)
        h = hashlib.sha256((s.prev + str(len(s.rows)) + kind + body).encode()).hexdigest()[:16]
        s.rows.append(dict(idx=len(s.rows), t=round(float(t), 2), kind=kind, payload=payload, prev=s.prev, hash=h)); s.prev = h

    def verify(s):
        prev = '0' * 16
        for r in s.rows:
            h = hashlib.sha256((prev + str(r['idx']) + r['kind'] + json.dumps(r['payload'], sort_keys=True, default=str)).encode()).hexdigest()[:16]
            if r['prev'] != prev or r['hash'] != h: return False, r['idx']
            prev = h
        return True, None


def defense_metrics(rows, labels):
    TP = FP = FN = 0
    for r in rows:
        lab = uid(r) in labels
        flagged = r['status'] != 'ACCEPTED' or bool(r['flags'])
        if lab and flagged: TP += 1
        elif lab: FN += 1
        elif flagged and r['status'] != 'ACCEPTED': FP += 1
    P = TP / (TP + FP) if TP + FP else None; R = TP / (TP + FN) if TP + FN else None
    return dict(TP=TP, FP=FP, FN=FN, precision=P, recall=R)


# ============================================================== 3. ML MODELS (M-core)
class KF:
    """Scalar Kalman filter on the thermal model, 2 measurements: T and coolant-outlet temperature."""
    def __init__(s, c): s.c = c; s.x = None; s.P = 1.0

    def init(s, x): s.x = float(x); s.P = 1.0

    def predict(s, Qm):                      # nominal model (commanded flow = 1.0) -> real cooling loss appears as innovation
        c = s.c; dt = c['dt']; a = 1 - dt * c['h'] / c['C']; b = dt * (Qm + c['h'] * c['Tc']) / c['C']
        s.x = a * s.x + b; s.P = a * a * s.P + c['q_proc']

    def innov(s, rows, zs):
        c = s.c; H = np.array([[1.0], [c['a_out']]])[rows]
        R = np.diag(np.array([c['sig_T'] ** 2, c['sig_Tout'] ** 2])[rows])
        v = np.array(zs, float) - H[:, 0] * s.x; S = H @ H.T * s.P + R
        return v, S, H, float(v @ np.linalg.solve(S, v))          # NIS = v' S^-1 v

    def update(s, rows, zs):
        v, S, H, nis = s.innov(rows, zs)
        K = (s.P * H.T) @ np.linalg.inv(S)
        s.x = float(s.x + (K @ v)[0]); s.P = float((1 - (K @ H)[0, 0]) * s.P)
        return v, S, nis


def train_gen(cfg):
    """Unsupervised training on simulated NORMAL data only. Generator -> (progress, message, models|None)."""
    c = cfg; a = c['a_out']; X = []; nis = []; peaks = []; cs = []; ew = []; runs = 8
    for r in range(runs):
        w = World(c, seed=1000 + r, Q=c['Q_nom'] * (0.9 + 0.2 * r / (runs - 1))); kf = KF(c); up = dn = pk = 0.0; e = 0.0
        for k in range(360):
            ev, m = w.tick(); X.append([m['T'], m['flow'], m['Tout'], m['P']])
            z = [m['T'], m['Tout'] - (1 - a) * c['Tc']]
            if kf.x is None: kf.init(m['T']); continue
            kf.predict(m['P']); v, S, H, n = kf.innov([0, 1], z)
            if k > 20: nis.append(n); u = v[0] / math.sqrt(S[0, 0]); up = max(0, up + u - .5); dn = max(0, dn - u - .5); pk = max(pk, up, dn)
            kf.update([0, 1], z); cc = m['T'] - z[1] / a; cs.append(cc)
        peaks.append(pk)
        yield (r + 1) / (runs + 3), 'Simulating NORMAL run %d/%d (seeded)' % (r + 1, runs), None
    X = np.array(X); mu = X.mean(0); sd = X.std(0) + 1e-9; Z = (X - mu) / sd
    U, S_, Vt = np.linalg.svd(Z, full_matrices=False); lam = S_ ** 2 / (len(Z) - 1); k = 2; Pm = Vt[:k].T
    sc = Z @ Pm; T2 = np.sum(sc ** 2 / lam[:k], 1); E = Z - sc @ Pm.T; SPE = np.sum(E ** 2, 1)
    yield (runs + 1) / (runs + 3), 'Fitting PCA (Hotelling T2 + SPE limits)', None
    c_sd = float(np.std(cs)); e = 0.0; ew = []
    for v_ in cs: e = 0.9 * e + 0.1 * v_ / c_sd; ew.append(abs(e))
    m = dict(mu=mu, sd=sd, Pm=Pm, lam=lam, k=k, t2_lim=float(np.quantile(T2, 0.997) * 1.3), spe_lim=float(np.quantile(SPE, 0.997) * 1.5),
             c_sd=c_sd, nis_lim={1: float(stats.chi2.ppf(0.999, 1)), 2: float(max(stats.chi2.ppf(0.999, 2), 1.15 * np.quantile(nis, 0.999)))},
             cusum_h=float(max(5.0, 1.25 * max(peaks))), ewma_lim=float(max(0.8, 1.25 * max(ew))), explained=(lam[:k].sum() / lam.sum()))
    yield (runs + 2) / (runs + 3), 'Fitting Isolation Forest (ablation only)', None
    try:
        from sklearn.ensemble import IsolationForest
        m['iforest'] = IsolationForest(n_estimators=100, contamination=0.002, random_state=0).fit(X)
    except Exception:
        m['iforest'] = None
    m['summary'] = ("Learned from %d NORMAL samples (labels never used):\n  PCA keeps %d comps (%.0f%% var)  Hotelling T2 limit = %.1f   SPE (Q) limit = %.1f\n"
                    "  Kalman NIS limit (2 dof) = %.1f   (1 dof) = %.1f\n  CUSUM h = %.1f (k=0.5)   EWMA(consistency) limit = %.2f   consistency sd = %.2f C"
                    % (len(X), k, 100 * m['explained'], m['t2_lim'], m['spe_lim'], m['nis_lim'][2], m['nis_lim'][1], m['cusum_h'], m['ewma_lim'], c_sd))
    yield 1.0, 'Training done', m


def train_models(cfg):
    for _, _, m in train_gen(cfg):
        if m: return m


# likelihood table P(feature=1 | hypothesis), order = HYP.   ASSUMPTION (expert prior, naive-Bayes independence)
TABLE = {'spike_inc': [0.005, 0.85, 0.02, 0.30], 'flat': [0.002, 0.85, 0.01, 0.05], 'drift_c': [0.005, 0.80, 0.01, 0.20],
         'persist_ok': [0.01, 0.03, 0.85, 0.05], 'flow_low': [0.01, 0.25, 0.90, 0.10], 'vib_low': [0.01, 0.05, 0.90, 0.05],
         'flow_vs_vib': [0.01, 0.85, 0.05, 0.10], 'quarantined': [0.03, 0.05, 0.05, 0.90], 'maint_conflict': [0.01, 0.20, 0.35, 0.70],
         't2': [0.01, 0.30, 0.90, 0.30], 'spe': [0.01, 0.85, 0.25, 0.40], 'rising': [0.005, 0.02, 0.85, 0.05]}
FEAT_TEXT = {'spike_inc': 'single huge Kalman innovation, T vs outlet-temp inconsistent', 'flat': 'sensor flat-lined (zero noise)',
             'drift_c': 'slow drift between T and outlet temp', 'persist_ok': 'persistent innovation, both temps agree',
             'flow_low': 'flow meter low', 'vib_low': 'pump vibration low', 'flow_vs_vib': 'flow meter disagrees with pump vibration',
             'quarantined': 'quarantined / conflicting events', 'maint_conflict': 'maintenance says OK but data says otherwise',
             't2': 'Hotelling T2 above limit', 'spe': 'SPE (Q) above limit', 'rising': 'both temperatures really rising'}


def forecast(c, x, P, f, Qh, H=None):
    """Propagate Kalman posterior forward (closed form, first-order plant). -> P(T crosses critical within H), time-to-critical."""
    H = c['horizon'] if H is None else H
    f = max(f, 0.02); tau = c['C'] / (c['h'] * f); Tss = c['Tc'] + Qh / (c['h'] * f)
    ts = np.arange(0.0, max(H, 0.5) + 1e-9, 0.5); n = ts / c['dt']
    mean = Tss + (x - Tss) * np.exp(-ts / tau); a = 1 - c['dt'] / tau; q = c['proc_std'] ** 2
    var = P * a ** (2 * n) + q * (1 - a ** (2 * n)) / (1 - a * a)
    pc = float(np.max(stats.norm.sf((c['T_crit'] - mean) / np.sqrt(var + 1e-9))))
    idx = np.where(mean >= c['T_crit'])[0]
    return pc, (float(ts[idx[0]]) if len(idx) else None)


def rollouts(c, x, P, f, Qh, rem, n, seed, horizon=None):
    """M3: Monte-Carlo rollouts per action with COMMON RANDOM NUMBERS (same noise for every action)."""
    dt = c['dt']; H = rem if horizon is None else min(rem, horizon); steps = max(1, int(H / dt)); rng = np.random.default_rng(seed)
    eps = rng.normal(0, c['proc_std'], (n, steps)); u0 = rng.normal(size=n); uf = rng.normal(size=n); uq = rng.normal(size=n)
    bf = rng.uniform(size=n); bt = rng.uniform(4, 25, size=n); out = {}
    for a in ACTIONS:
        T = x + math.sqrt(max(P, 1e-6)) * u0; fl = np.clip(f * (1 + 0.05 * uf), 0.0, 1.3); Q = Qh + uq; Tmax = T.copy()
        for i in range(steps):
            tt = i * dt; bl = 0.0; sf = 1.0
            if a == 'ACTIVATE_BACKUP_COOLING' and tt >= c['action_latency']:
                bl = np.where((bf < 0.15) & (tt >= bt), 0.0, min(1.0, (tt - c['action_latency']) / 2.0) * c['backup_flow'])   # 15 % backup failure ASSUMPTION
            if a == 'SHUTDOWN_REACTOR' and tt >= c['action_latency']: sf = 0.05 + 0.95 * math.exp(-(tt - c['action_latency']) / 2.5)
            T = T + dt * (Q * sf - c['h'] * (fl + bl) * (T - c['Tc'])) / c['C'] + eps[:, i]
            Tmax = np.maximum(Tmax, T)
        viol = Tmax >= c['T_crit']; L = 100.0 * viol + np.maximum(0.0, Tmax - c['T_fixed_alarm'])
        k = max(1, int(math.ceil(0.05 * n))); cvar = float(np.sort(L)[-k:].mean()); mu_ = float(L.mean()); se = 1.96 * float(L.std()) / math.sqrt(n)
        out[a] = dict(mean=mu_, cvar=cvar, viol=float(viol.mean()), lo=mu_ - se, hi=mu_ + se, Tmax=Tmax, worst=int(np.argmax(Tmax)))
    return out


# ============================================================== 4. ENGINE
def plain_msg(mode, st):
    T = st['T_est']; p = st['p_crit']; tt = st['ttc']
    ttc = 'about %.0f min' % tt if tt is not None else 'not within the forecast window'
    if mode == 'SHUTDOWN':
        return ("[RED] REACTOR AUTO-SHUTDOWN\nWhat happened: cooling water flow has really dropped and the reactor is heating up (estimate %.1f C).\n"
                "Why: flow meter, pump vibration and heat balance agree; chance of reaching 100 C = %d%% (%s).\n"
                "What to do: keep away from the reactor area, do NOT restart. Call the technician." % (T, round(100 * p), ttc))
    if mode == 'SENSOR_FAULT':
        return ("[AMBER] SENSOR PROBLEM (not a real heat problem)\nWhat happened: one temperature reading looks wrong, but cooling flow and the other sensors are normal.\n"
                "Why: the reading disagrees with physics and with the outlet-temperature sensor. Plant keeps running on the estimated value (%.1f C).\n"
                "What to do: no shutdown needed. Call the technician to check the sensor." % T)
    if mode == 'ALERT':
        return ("[AMBER] COOLING WARNING\nWhat happened: cooling looks weaker than normal; chance of overheating = %d%% (%s).\n"
                "Why: cooling evidence is abnormal but not yet fully confirmed.\nWhat to do: stay alert, check pump and valves only if safe, call the technician. Supervisor approval may be needed."
                % (round(100 * p), ttc))
    if mode == 'ATTACK_CONTAINED':
        return ("[AMBER] FAKE / REPLAYED MESSAGES BLOCKED\nWhat happened: some messages had bad signatures or were old copies.\n"
                "Why: they were quarantined and have no effect on decisions. Real emergency path stays active.\nWhat to do: tell the IT/security person.")
    if mode == 'ABSTAIN':
        return ("[AMBER] NOT SURE - NEED MORE EVIDENCE\nWhat happened: signals conflict and the cause cannot be identified.\n"
                "Why: system abstains instead of guessing.\nWhat to do: check the flow meter and pump locally and report what you see.")
    return "[GREEN] Back to normal."


class Engine:
    def __init__(s, cfg, models, sw=None):
        s.cfg = cfg; s.m = models
        s.sw = dict(cross=True, adv=True, causal=True, auto_backup=False, compromise=None)
        if sw: s.sw.update(sw)
        s.m3_every = 8; s.n_roll = cfg['n_roll']; s.reset()

    def reset(s):
        c = s.cfg
        s.ledger = Ledger(c, s.sw['adv']); s.audit = Audit(); s.kf = KF(c); s.k = 0; s.t = 0.0
        s.up = s.dn = 0.0; s.cus_alarm = False; s.ewc = 0.0; s.L = np.zeros(4); s.post = PRIOR.copy()
        s.f_ew = 1.0; s.v_ew = 2.0; s.Qh = c['Q_nom']; s.last = dict(T=70.0, flow=1.0, Tout=62.0, P=c['Q_nom'], vib=2.0)
        s.Th = []; s.Toh = []; s.Tmh = []; s.ser = []
        s.trust = {src: dict(trust=.9, org=.9, rel=.95, sig=1.0, tim=1.0, cons=1.0, plaus=1.0) for src in SRC}
        s.q_times = []; s.maint_t = -99.0; s.mode = 'NORMAL'; s.shutdown = False; s.backup_t = None; s.backup_used = False
        s.last_act_t = -99.0; s.hist = []; s.sim = None; s.cert = None; s.opt = None; s.agents = []; s.causes = []; s.spur = None
        s.pcrit = 0.0; s.ttc = None; s.feats = {}; s.info = {}; s.n_src = 0; s.f_est = 1.0; s.auto_ok = False; s.rising = False
        s.pca = dict(t2=0, spe=0, t2_flag=False, spe_flag=False, suspect=None); s.contrib = None; s.last_msg = {}; s.new_rows = []
        s.maint_ok = False; s.attack_t = -99.0; s.dec_k = -99; s.sT = 0.0; s.rise_n = 0; s.sf_until = -99.0

    # ---------- helpers
    def feasible(s, a, now):
        c = s.cfg; rem = c['deadline'] - now
        if s.shutdown and a not in ('NO_ACTION', 'CONTINUE_MONITORING'): return False, 'reactor already shut down (mutual exclusion)'
        if a == 'ACTIVATE_BACKUP_COOLING' and s.backup_used: return False, 'backup coolant already used (resource limit)'
        if a in ('ACTIVATE_BACKUP_COOLING', 'ISOLATE_NETWORK') and now - s.last_act_t < c['cooldown']:
            return False, 'cooldown active (%.1f min left)' % (c['cooldown'] - (now - s.last_act_t))
        if a == 'SHUTDOWN_REACTOR' and rem < c['action_latency']: return False, 'deadline: not enough time to complete action'
        return True, ''

    def needs_approval(s, a):
        if a == 'SHUTDOWN_REACTOR': return not s.auto_ok
        if a == 'ACTIVATE_BACKUP_COOLING': return not s.sw['auto_backup']
        return a == 'ISOLATE_NETWORK'

    def external_action(s, a, now):
        if a == 'ACTIVATE_BACKUP_COOLING': s.backup_t = now; s.backup_used = True
        if a == 'SHUTDOWN_REACTOR': s.shutdown = True; s.mode = 'SHUTDOWN'
        if a != 'CONTINUE_MONITORING' and a != 'NO_ACTION': s.last_act_t = now
        s.audit.add(now, 'ACTION', dict(action=a))

    def replay_hash(s):
        h = hashlib.sha256()
        for r in s.ledger.rows: h.update(f"{r['event_id']}|{r['arrival_time']}|{r['status']}".encode())
        for x in s.hist: h.update(f"{x['k']}|{x['mode']}|{x['pcrit']:.4f}|{x['That']:.4f}".encode())
        return h.hexdigest()[:16]

    # ---------- main per-tick step
    def step(self, events, now):
        s = self; c = s.cfg; m = s.m; sw = s.sw; a = c['a_out']; dt = c['dt']; Tc = c['Tc']
        s.k += 1; s.t = now; s.ledger.filt = sw['adv']; out = dict(actions=[], msgs=[])
        # ---- M1 ledger
        acc = {}; last_row = {}
        for ev in events:
            r = s.ledger.ingest(ev, now); last_row[ev['source']] = r
            if r['status'] == 'ACCEPTED':
                if 'out_of_order' not in r['flags']: acc[ev['source']] = r
            elif r['status'] in ('QUARANTINED', 'CONFLICT'):
                s.q_times.append(now); s.audit.add(now, 'LEDGER_' + r['status'], dict(id=ev['event_id'], src=ev['source'], why=r['reason']))
        s.q_times = [x for x in s.q_times if now - x <= 5.0]; s.new_rows = [last_row[k] for k in last_row]

        def val(src):
            r = acc.get(src)
            try: return None if r is None else float(r['value'])
            except Exception: return None
        mT, mf, mTo, mP, mv = [val(x) for x in ('REACTOR_SENSOR_14', 'COOLING_FLOW_1', 'COOLANT_OUT_3', 'POWER_METER_1', 'PUMP_VIB_7')]
        rm = acc.get('MAINTENANCE_AGENT')
        if rm is not None and 'operational' in str(rm['value']).lower(): s.maint_t = now
        # ---- Kalman (nominal model) + outlier gating
        Qm = mP if mP is not None else s.Qh; s.Qh = 0.5 * s.Qh + 0.5 * Qm
        if s.kf.x is None: s.kf.init(mT if mT is not None else c['T_norm'])
        else: s.kf.predict(Qm)
        rows_i = []; zs = []
        if mT is not None: rows_i.append(0); zs.append(mT)
        if mTo is not None: rows_i.append(1); zs.append(mTo - (1 - a) * Tc)
        info = dict(nis=0.0, c=0.0, dropped=None, u=None)
        if rows_i:
            v, S, H, nis = s.kf.innov(rows_i, zs); info['nis'] = nis; lim = m['nis_lim'][len(rows_i)]; use = list(range(len(rows_i)))
            if len(rows_i) == 2:
                cc = (zs[0] - zs[1] / a) / m['c_sd']; info['c'] = cc
                if nis > lim and abs(cc) > 4.0:
                    bad = int(np.argmax(np.abs(v) / np.sqrt(np.diag(S)))); info['dropped'] = ['T', 'Tout'][bad]; use = [1 - bad]
            elif nis > lim: info['dropped'] = 'unverified'; use = []
            if use:
                ri = [rows_i[i] for i in use]; v2, S2, _ = s.kf.update(ri, [zs[i] for i in use])
                if 0 in ri: j = ri.index(0); info['u'] = float(v2[j] / math.sqrt(S2[j, j]))
        # ---- CUSUM (persistent innovation) + EWMA (slow T-vs-Tout drift)
        if info['u'] is not None:
            u = info['u']; s.up = max(0.0, s.up + u - .5); s.dn = max(0.0, s.dn - u - .5); hh = m['cusum_h']
            if s.up > hh or s.dn > hh: s.cus_alarm = True
            elif s.up < hh / 4 and s.dn < hh / 4: s.cus_alarm = False
        if len(rows_i) == 2: s.ewc = 0.9 * s.ewc + 0.1 * info['c']
        drift_c = abs(s.ewc) > m['ewma_lim']
        # ---- PCA: Hotelling T2 + SPE + contribution plot
        for key, v_ in (('T', mT), ('flow', mf), ('Tout', mTo), ('P', mP), ('vib', mv)):
            if v_ is not None: s.last[key] = v_
        x = np.array([s.last['T'], s.last['flow'], s.last['Tout'], s.last['P']]); z = (x - m['mu']) / m['sd']
        sc = z @ m['Pm']; t2 = float(np.sum(sc ** 2 / m['lam'][:m['k']])); res = z - sc @ m['Pm'].T; spe = float(res @ res)
        ct2 = z * (m['Pm'] @ (sc / m['lam'][:m['k']])); cspe = res ** 2
        s.pca = dict(t2=t2, spe=spe, t2_flag=t2 > m['t2_lim'], spe_flag=spe > m['spe_lim'])
        names = ['REACTOR_SENSOR_14', 'COOLING_FLOW_1', 'COOLANT_OUT_3', 'POWER_METER_1']
        bi = int(np.argmax(cspe)) if s.pca['spe_flag'] else (int(np.argmax(np.abs(ct2))) if s.pca['t2_flag'] else None)
        s.pca['suspect'] = None if bi is None else names[bi]; s.contrib = dict(T2=ct2, SPE=cspe, names=names)
        # ---- trends / flat-line / flows
        if mT is not None: s.Tmh = (s.Tmh + [mT])[-10:]
        s.Th = (s.Th + [s.kf.x])[-12:]
        if mTo is not None: s.Toh = (s.Toh + [(mTo - (1 - a) * Tc) / a])[-12:]
        sl = lambda h: float(np.polyfit(np.arange(len(h)) * dt, h, 1)[0]) if len(h) >= 12 else 0.0
        sT = sl(s.Th); sTo = sl(s.Toh); s.sT = sT
        flat = len(s.Tmh) >= 10 and float(np.std(s.Tmh)) < 0.06
        s.rise_n = (s.rise_n + 1) if (sT > 0.35 and sTo > 0.35) else 0   # Stage0: noise guard (thresholds 0.35, ASSUMPTION)
        s.rising = s.rise_n >= 4                                           # must hold 4 ticks in a row (ASSUMPTION)
        if mf is not None: s.f_ew = 0.6 * s.f_ew + 0.4 * mf
        if mv is not None: s.v_ew = 0.6 * s.v_ew + 0.4 * mv
        b_ass = 0.0 if s.backup_t is None else min(c['backup_flow'], 0.35 * (now - s.backup_t))
        exp_flow = s.v_ew / 2 + b_ass
        r_pred = (s.Qh - c['h'] * s.f_ew * (s.kf.x - Tc)) / c['C']
        hb_pred = r_pred > 0.3; hb_obs = (sT > 0.2) and (r_pred - sT < 0.4); src_C = hb_pred and hb_obs   # heat-balance check
        agree = abs(s.f_ew - exp_flow) < 0.25
        s.f_est = max(0.0, s.f_ew if (not sw['cross'] or agree or src_C) else exp_flow)
        # ---- trust per source (maths): reliability x signature x timeliness x consistency x plausibility
        def cons_for(src):
            base = math.exp(-0.5 * (max(0.0, abs(info['c']) - 2.5) / 1.5) ** 2)
            if src == 'REACTOR_SENSOR_14': return 0.02 if info['dropped'] == 'T' else (1.0 if info['dropped'] == 'Tout' else base)
            if src == 'COOLANT_OUT_3': return 0.02 if info['dropped'] == 'Tout' else (1.0 if info['dropped'] == 'T' else base)
            if not sw['cross'] and src in ('COOLING_FLOW_1', 'PUMP_VIB_7'): return 1.0
            if src == 'COOLING_FLOW_1': return 1.0 if (agree or src_C or not hb_pred) else 0.1
            if src == 'PUMP_VIB_7': return 1.0 if (s.v_ew > 1.7 or s.f_ew < 0.85 or agree) else 0.3
            if src == 'MAINTENANCE_AGENT': return 0.1 if (s.f_ew < 0.85 or s.rising) else 1.0
            return 1.0
        for src in SRC:
            r = acc.get(src) or last_row.get(src)
            if r is None: continue
            t_ = s.trust[src]; ok = 1.0 if (r['status'] == 'ACCEPTED' and 'rate_violation' not in r['flags']) else 0.0
            t_['rel'] = 0.97 * t_['rel'] + 0.03 * ok; t_['sig'] = 1.0 if r['signature_valid'] else 0.1
            t_['tim'] = math.exp(-max(0.0, r['arrival_time'] - r['event_time']) / 2.0); t_['cons'] = cons_for(src)
            t_['plaus'] = 0.2 if (src == 'REACTOR_SENSOR_14' and flat) else (0.3 if 'rate_violation' in r['flags'] else 1.0)
            t_['org'] = r['trust_score']
            t_['trust'] = r['trust_score'] if not sw['adv'] else t_['rel'] * t_['sig'] * t_['tim'] * t_['cons'] * t_['plaus']
        trusted = lambda src: s.trust[src]['trust'] >= 0.5
        s.maint_ok = (now - s.maint_t <= 12.0) and trusted('MAINTENANCE_AGENT')
        A = s.f_ew < 0.85 and trusted('COOLING_FLOW_1'); B = s.v_ew < 1.7 and trusted('PUMP_VIB_7')
        s.n_src = (int(A) + int(B) + int(src_C)) if sw['cross'] else int(A)
        # ---- M2 Bayesian fusion over {normal, sensor_fault, cooling_failure, spoofing}
        F = dict(spike_inc=bool(info['dropped']), flat=flat, drift_c=drift_c, persist_ok=s.cus_alarm and not drift_c,
                 flow_low=s.f_ew < 0.85, vib_low=s.v_ew < 1.7, flow_vs_vib=bool(sw['cross'] and not agree and not src_C),
                 quarantined=len(s.q_times) > 0, maint_conflict=bool(s.maint_ok and s.f_ew < 0.85),
                 t2=s.pca['t2_flag'], spe=s.pca['spe_flag'], rising=s.rising)
        ll = np.zeros(4)
        for f, p in TABLE.items():
            p = np.array(p); ll += np.log(p) if F[f] else np.log(1 - p)
        s.L = c['forget'] * s.L + ll; lp = np.log(PRIOR) + c['temper'] * s.L; lp -= lp.max(); pp = np.exp(lp); s.post = pp / pp.sum()
        s.feats = F; s.info = info
        if info['dropped'] in ('T', 'Tout'): s.sf_until = now + 1.0          # Kalman gate rejected a measurement -> sensor-fault signature, hold 1 min (ASSUMPTION)
        # ---- forecast (time-to-critical)
        s.pcrit, s.ttc = forecast(c, s.kf.x, s.kf.P, s.f_est, s.Qh)
        ih = dict(zip(HYP, s.post)); old = s.mode
        # ---- decision policy
        veto = bool(s.maint_ok and sw['cross'])
        s.auto_ok = (not s.shutdown) and ((s.rising and s.n_src >= 2 and s.pcrit >= c['p_act']) if sw['cross'] else (s.pcrit >= c['p_act'] and s.n_src >= 1))
        if s.shutdown: mode = 'SHUTDOWN'
        elif s.auto_ok and not veto:
            mode = 'SHUTDOWN'
            d = s.run_decision(now, 'SHUTDOWN_REACTOR')
            if any(ch['status'] == 'FAIL' and ch['method'] == 'rule' for ch in d['cert']['checks']): mode = 'ABSTAIN'
            else: s.external_action('SHUTDOWN_REACTOR', now); out['actions'].append('SHUTDOWN_REACTOR')
        elif (ih['sensor_fault'] >= 0.5 and not (s.pcrit >= c['p_alert'] and s.n_src >= 2)) or (now < s.sf_until and s.pcrit < c['p_alert']): mode = 'SENSOR_FAULT'
        elif s.pcrit >= c['p_alert'] or ih['cooling_failure'] >= 0.4: mode = 'ABSTAIN' if (s.post.max() < 0.5 or veto) else 'ALERT'
        elif ih['spoofing'] >= 0.5: mode = 'ATTACK_CONTAINED'
        else: mode = 'NORMAL'
        s.mode = mode
        if mode in ('ALERT', 'ABSTAIN') and (old not in ('ALERT', 'ABSTAIN') or s.k - s.dec_k >= s.m3_every): s.run_decision(now)
        elif mode == 'NORMAL' and s.pcrit > 0.05 and s.k - s.dec_k >= s.m3_every: s.run_decision(now)
        if mode in ('ALERT', 'ABSTAIN') and sw['auto_backup'] and not s.backup_used and s.opt and s.opt['best'] == 'ACTIVATE_BACKUP_COOLING':
            s.external_action('ACTIVATE_BACKUP_COOLING', now); out['actions'].append('ACTIVATE_BACKUP_COOLING')
        # ---- M2 ranked causes + spurious-correlation rejection
        s.ser.append((s.kf.x, s.last['vib'], s.last['flow'])); s.ser = s.ser[-40:]; s.build_causes()
        # ---- alerts + audit
        st = dict(T_est=s.kf.x, p_crit=s.pcrit, ttc=s.ttc)
        if mode != old:
            s.audit.add(now, 'MODE', dict(frm=old, to=mode, p_crit=round(s.pcrit, 3), post=[round(float(q), 3) for q in s.post]))
            if mode != 'NORMAL' and now - s.last_msg.get(mode, -99) > 5.0: s.last_msg[mode] = now; out['msgs'].append((mode, plain_msg(mode, st)))
        if len(s.q_times) >= 2 and now - s.attack_t > 10.0 and mode != 'ATTACK_CONTAINED':
            s.attack_t = now; out['msgs'].append(('ATTACK_CONTAINED', plain_msg('ATTACK_CONTAINED', st)))
        s.hist.append(dict(k=s.k, t=now, Tm=mT, Tout=mTo, That=s.kf.x, fm=mf, vib=mv, f_est=s.f_est, pcrit=s.pcrit, ttc=s.ttc, mode=mode,
                           post=[float(q) for q in s.post], nis=info['nis'], c=info['c'], dropped=info['dropped'], n_src=s.n_src))
        return out

    # ---------- M2
    def build_causes(s):
        post = s.post; order = np.argsort(-post); ev = {h: [] for h in HYP}
        for src, hs in (('REACTOR_SENSOR_14', ['sensor_fault']), ('COOLING_FLOW_1', ['cooling_failure']), ('PUMP_VIB_7', ['cooling_failure'])):
            for r in reversed(s.ledger.rows[-40:]):
                if r['source'] == src and r['status'] == 'ACCEPTED':
                    for h in hs: ev[h].append(r['event_id'])
                    break
        ev['spoofing'] = [r['event_id'] for r in s.ledger.rows[-200:] if r['status'] in ('QUARANTINED', 'CONFLICT')][-3:]
        txt = {'normal': 'No fault - normal operation', 'sensor_fault': 'Sensor fault (%s)' % (s.pca['suspect'] or 'REACTOR_SENSOR_14'),
               'cooling_failure': 'REAL cooling failure (flow loss)', 'spoofing': 'Spoofed / forged / replayed messages'}
        top = float(post.max()); cs = []
        for i, h in enumerate(order):
            p = float(post[h]); lab = 'supported' if (p >= 0.6 and (i > 0 or p - float(post[order[1]]) >= 0.2)) else 'hypothesis (not identifiable)'
            cs.append(dict(rank=i + 1, cause=txt[HYP[h]], hyp=HYP[h], p=p, label=lab, evidence=ev[HYP[h]]))
        s.spur = None
        if len(s.ser) >= 30:
            T = np.array([q[0] for q in s.ser]); V = np.array([q[1] for q in s.ser]); Fl = np.array([q[2] for q in s.ser])
            cor = lambda x, y: 0.0 if x.std() < 1e-9 or y.std() < 1e-9 else float(np.corrcoef(x, y)[0, 1])
            rxy, rxz, ryz = cor(V, T), cor(V, Fl), cor(T, Fl); pr = (rxy - rxz * ryz) / math.sqrt(max((1 - rxz ** 2) * (1 - ryz ** 2), 1e-6))
            s.spur = dict(r=rxy, partial=pr, confounder='COOLING_FLOW_1', spurious=abs(rxy) > 0.7 and (abs(pr) < 0.35 or rxz ** 2 > 0.97))
            if s.spur['spurious'] and not s.sw['causal']:
                cs.insert(0, dict(rank=0, cause='Pump-vibration drop CAUSES temperature rise (correlation only)', hyp='spurious', p=min(0.99, abs(rxy)),
                                  label='SPURIOUS (accepted because causal model is OFF)', evidence=[]))
                for i, q in enumerate(cs): q['rank'] = i + 1
        s.causes = cs

    def alt_histories(s):
        order = np.argsort(-s.post)[:2]; out = []
        story = {'normal': 'Plant is healthy; any odd reading is noise.',
                 'sensor_fault': 'REACTOR_SENSOR_14 gives wrong values; the real reactor is near %.1f C, cooling is fine; keep running on the estimate.' % s.kf.x,
                 'cooling_failure': 'Cooling flow is really lost; reactor heads to critical (time-to-critical: %s) unless backup / shutdown.' % ('%.0f min' % s.ttc if s.ttc is not None else 'not in window'),
                 'spoofing': 'Messages are forged or replayed; physical plant is unaffected; keep the real-emergency path open.'}
        for h in order: out.append(dict(history=HYP[h], prob=round(float(s.post[h]), 3), story=story[HYP[h]], weak_evidence=bool(s.post.max() < 0.8)))
        return out

    # ---------- M3 + M5 + M7 + M4
    def run_decision(s, now, action=None):
        c = s.cfg; rem = max(0.0, c['deadline'] - now); seed = int(c['seed']) * 100003 + s.k; s.dec_k = s.k
        sim = rollouts(c, s.kf.x, s.kf.P, s.f_est, s.Qh, rem, s.n_roll, seed)
        short = rollouts(c, s.kf.x, s.kf.P, s.f_est, s.Qh, rem, min(s.n_roll, 150), seed, horizon=5.0)
        table = []
        for a in ACTIONS:
            ok, why = s.feasible(a, now); r = sim[a]
            table.append(dict(action=a, mean=r['mean'], cvar=r['cvar'], viol=r['viol'], lo=r['lo'], hi=r['hi'], cost=ACTION_COST[a],
                              J=r['mean'] + c['lam'] * r['cvar'] + c['mu'] * ACTION_COST[a], feasible=ok, reason=why, approval=s.needs_approval(a)))
        feas = [r for r in table if r['feasible']]; best = min(feas, key=lambda r: r['J'])['action'] if feas else 'CONTINUE_MONITORING'
        gf = [(short[a]['mean'] + c['lam'] * short[a]['cvar'] + c['mu'] * ACTION_COST[a], a) for a in ACTIONS if s.feasible(a, now)[0]]
        greedy = min(gf)[1] if gf else 'NO_ACTION'; J = {r['action']: r['J'] for r in table}
        s.sim = sim; s.opt = dict(best=best, greedy=greedy, table=table, seed=seed, rem=rem,
                                  baselines=dict(optimized=J[best], no_action=J['NO_ACTION'], greedy=J[greedy]))
        s.cert = s.certificate(action or best, now); s.agents = s.make_agents(now)
        s.audit.add(now, 'DECISION', dict(best=best, greedy=greedy, cert=s.cert['overall'], abstain=s.cert['abstain']))
        return dict(best=best, cert=s.cert, table=table)

    def certificate(s, action, now):
        c = s.cfg; checks = []; opt = s.opt; row = {r['action']: r for r in opt['table']}
        def add(cid, claim, st, method, detail): checks.append(dict(id=cid, claim=claim, status=st, method=method, detail=detail))
        bad = [r['event_id'] for r in s.ledger.rows[-30:] if r['status'] == 'ACCEPTED' and not r['signature_valid']]
        add('EVIDENCE_SIGNED', 'all evidence used carries a valid signature', 'FAIL' if bad else 'PASS', 'rule', 'unsigned accepted events: %d' % len(bad))
        need = 2 if action in ('SHUTDOWN_REACTOR', 'ACTIVATE_BACKUP_COOLING') else 0
        st = 'PASS' if s.n_src >= need else ('FAIL' if action == 'SHUTDOWN_REACTOR' and not s.shutdown else 'UNKNOWN')
        add('INDEPENDENT_SOURCES', '>=%d independent cooling-evidence sources' % need, st, 'rule', 'abnormal sources = %d' % s.n_src)
        ok, why = s.feasible(action, now) if not (action == 'SHUTDOWN_REACTOR' and s.shutdown) else (True, '')
        add('CONSTRAINTS', 'resources / cooldown / mutual exclusion / approval', 'PASS' if ok else 'FAIL', 'rule', why or ('approval required' if s.needs_approval(action) else 'ok'))
        ttc_ok = s.ttc is None or s.ttc > c['action_latency']
        add('DEADLINE_MARGIN', 'action completes before critical and before 90-min deadline', 'PASS' if ttc_ok else 'FAIL', 'rule',
            'time-to-critical = %s min, action latency = %.1f' % ('%.1f' % s.ttc if s.ttc is not None else 'none', c['action_latency']))
        pc_rem = forecast(c, s.kf.x, s.kf.P, s.f_est, s.Qh, H=max(opt['rem'], 0.5))[0]; gap = abs(pc_rem - row['NO_ACTION']['viol'])
        add('FORECAST_CALIBRATION', 'Kalman forecast agrees with Monte-Carlo', 'PASS' if gap <= 0.25 else 'UNKNOWN', 'empirical', 'closed-form %.2f vs rollouts %.2f' % (pc_rem, row['NO_ACTION']['viol']))
        if action in ('NO_ACTION', 'CONTINUE_MONITORING'): st, dt_ = 'PASS', 'monitoring only'
        else:
            a_, n_ = row[action], row['NO_ACTION']
            st = 'PASS' if a_['hi'] < n_['lo'] else ('UNKNOWN' if a_['lo'] <= n_['hi'] else 'FAIL'); dt_ = 'loss %.1f [%.1f,%.1f] vs no-action %.1f [%.1f,%.1f]' % (a_['mean'], a_['lo'], a_['hi'], n_['mean'], n_['lo'], n_['hi'])
        add('ACTION_BENEFIT', 'action lowers loss with non-overlapping 95% intervals', st, 'empirical', dt_)
        add('IDENTIFIABILITY', 'top cause is identifiable (posterior >= 0.6)', 'PASS' if s.post.max() >= 0.6 else 'UNKNOWN', 'empirical', 'max posterior %.2f' % s.post.max())
        sts = [x['status'] for x in checks]; overall = 'FAIL' if 'FAIL' in sts else ('UNKNOWN' if 'UNKNOWN' in sts else 'PASS')
        w = s.sim['NO_ACTION']; ce = dict(action='NO_ACTION', violation_rate=round(row['NO_ACTION']['viol'], 3),
                                         worst_rollout=dict(index=w['worst'], Tmax=round(float(w['Tmax'][w['worst']]), 1), seed=opt['seed']))
        for r in opt['table']:
            if not r['feasible']: ce = dict(action=r['action'], rejected_because=r['reason']); break
        return dict(action=action, overall=overall, abstain=overall == 'UNKNOWN', checks=checks, counterexample=ce,
                    note='"rule" = deterministic check; "empirical" = Monte-Carlo estimate. Empirical checks are NOT proofs.')

    def make_agents(s, now):
        post = s.post; top = int(np.argmax(post)); ev = [r['event_id'] for r in s.ledger.rows[-5:]]; best = s.opt['best']
        ok, why = s.feasible(best, now); q = len(s.q_times) > 0
        truth = dict(Investigator=HYP[top], Predictor=round(s.pcrit, 2), Optimizer=best, Safety=bool(ok), Red=q)
        ag = [dict(agent='Investigator', claim='cause=' + HYP[top], evidence_ids=ev[:3], confidence=round(float(post[top]), 3), _v=HYP[top]),
              dict(agent='Predictor', claim='P_crit(%dmin)=%.2f ttc=%s' % (s.cfg['horizon'], s.pcrit, s.ttc), evidence_ids=ev[:2], confidence=round(max(s.pcrit, 1 - s.pcrit), 3), _v=round(s.pcrit, 2)),
              dict(agent='Optimizer', claim='best action=' + best, evidence_ids=[], confidence=round(min(0.99, 0.5 + abs(s.opt['baselines']['no_action'] - s.opt['baselines']['optimized']) / 100), 3), _v=best),
              dict(agent='Safety', claim=best + (' permitted' if ok else ' BLOCKED: ' + why), evidence_ids=[], confidence=0.95, _v=bool(ok)),
              dict(agent='Red-team', claim='spoofing ' + ('suspected' if q else 'not seen'), evidence_ids=[r['event_id'] for r in s.ledger.rows[-100:] if r['status'] != 'ACCEPTED'][-3:], confidence=round(float(post[3]), 3), _v=q)]
        bad = s.sw.get('compromise')
        lies = {'Investigator': ('cause=normal', 'normal'), 'Predictor': ('P_crit=0.00 ttc=None (all safe)', 0.0), 'Optimizer': ('best action=NO_ACTION', 'NO_ACTION'),
                'Safety': ('all clear, no check needed', not bool(ok)), 'Red-team': ('no attack ever', not q)}
        for a in ag:
            if a['agent'] == bad: a['claim'], a['_v'] = lies[bad]; a['confidence'] = 0.99
            key = {'Red-team': 'Red'}.get(a['agent'], a['agent']); t = truth[key]
            valid = abs(a['_v'] - t) <= 0.1 if a['agent'] == 'Predictor' else a['_v'] == t
            a['validated'] = bool(valid); a['status'] = 'OK (matches validated evidence)' if valid else 'OVERRULED by validated evidence'
            a['llm_weight'] = 0
        return ag

    def state_json(s):
        top = s.causes[0] if s.causes else {}
        return dict(time_min=round(s.t, 2), mode=s.mode, shutdown=s.shutdown, temperature_estimate_C=round(s.kf.x or 0, 1),
                    last_measured_T=round(s.last['T'], 1), flow_ewma=round(s.f_ew, 2), pump_vibration_ewma=round(s.v_ew, 2),
                    p_critical=round(s.pcrit, 3), time_to_critical_min=s.ttc, posterior={h: round(float(p), 3) for h, p in zip(HYP, s.post)},
                    ranked_causes=[dict(cause=q['cause'], p=round(q['p'], 3), label=q['label']) for q in s.causes[:4]],
                    trust_per_source={k: dict(our_trust=round(v['trust'], 2), organizer_trust=v['org'], signature=v['sig'], consistency=round(v['cons'], 2)) for k, v in s.trust.items()},
                    independent_cooling_sources_abnormal=s.n_src, quarantined_events_last_5min=len(s.q_times), temperature_really_rising=s.rising,
                    active_evidence=[FEAT_TEXT[k] for k, v in s.feats.items() if v], switches=dict(cross_check=s.sw['cross'], adversarial_filter=s.sw['adv'], causal_model=s.sw['causal']),
                    last_decision=(s.opt or {}).get('best'), certificate=(s.cert or {}).get('overall'))


# ============================================================== 5. EVALUATION + TESTS + API
SCENARIOS = {'spike': [(0, 'spike')], 'stuck': [(0, 'stuck')], 'drift': [(0, 'drift')], 'noise': [(0, 'noise')], 'pump': [(0, 'pump')],
             'cool_sensor': [(0, 'cool_sensor')], 'forged': [(0, 'forged')], 'sig_invalid': [(0, 'sig_invalid')], 'replay': [(0, 'replay')],
             'dup': [(0, 'dup')], 'secondary': [(0, 'pump'), (0, 'secondary'), (2, 'op_backup')], 'paradox': [(0, 'paradox')]}


def run_scenario(models, cfg, kind, inject_t=20.0, sw=None, seed=7, n_roll=150, m3_every=40, engine=True):
    w = World(cfg, seed); e = Engine(cfg, models, sw) if engine else None
    if e: e.n_roll = n_roll; e.m3_every = m3_every
    plan = SCENARIOS.get(kind, []); K = int(cfg['deadline'] / cfg['dt']); ik = int(inject_t / cfg['dt']); stream = []; Xs = []
    res = dict(kind=kind, first_alert=None, shutdown_t=None, fixed_alarm=None, fixed_real=None, modes=set(), wrong=False)
    for k in range(K):
        for off, what in plan:
            if k == ik + int(off / cfg['dt']):
                if what == 'op_backup':
                    if e: w.apply_action('ACTIVATE_BACKUP_COOLING'); e.external_action('ACTIVATE_BACKUP_COOLING', w.t)
                else: w.inject(what)
        ev, m = w.tick(); stream.append(ev); Xs.append([m['T'], m['flow'], m['Tout'], m['P']])
        if res['fixed_alarm'] is None and m['T'] > cfg['T_fixed_alarm']: res['fixed_alarm'] = w.t
        if res['fixed_real'] is None and m['T'] > cfg['T_fixed_alarm'] and w.T > cfg['T_fixed_alarm'] - 2.0: res['fixed_real'] = w.t   # alarm justified by the TRUE temperature (not a spike)
        if e:
            out = e.step(ev, w.t)
            for a in out['actions']: w.apply_action(a)
            res['modes'].add(e.mode)
            if res['first_alert'] is None and e.mode in ('ALERT', 'ABSTAIN', 'SHUTDOWN'): res['first_alert'] = w.t
            if res['shutdown_t'] is None and e.shutdown: res['shutdown_t'] = w.t
    res.update(crit_true=w.crit_t, max_T=max([w.T], default=0), stream=stream, X=np.array(Xs), world=w, engine=e)
    return res


def evaluate(models, cfg, kinds=None, progress=None, seed=7):
    rows = []; briers = []; kinds = kinds or list(SCENARIOS)
    for i, kind in enumerate(kinds):
        r = run_scenario(models, cfg, kind, seed=seed)
        cf = run_scenario(models, cfg, kind, seed=seed, engine=False)   # counterfactual: nothing done
        # the counterfactual world must not receive the operator backup action: re-run without 'op_backup' is the same as plan (engine=False skips it)
        ccf = cf['world'].crit_t; e = r['engine']; H = cfg['horizon']; bs = []
        for h in e.hist:
            if h['mode'] == 'SHUTDOWN' and r['shutdown_t'] is not None and h['t'] > r['shutdown_t']: break
            o = 1.0 if (ccf is not None and h['t'] <= ccf <= h['t'] + H) else 0.0; bs.append((h['pcrit'] - o) ** 2)
        briers += bs
        if0 = None; ifp = 0
        if models.get('iforest') is not None:
            pr = models['iforest'].predict(r['X']); ik = int(20.0 / cfg['dt'])
            ifp = int(np.sum(pr[:ik] == -1)); idx = np.where(pr[ik:] == -1)[0]; if0 = (ik + idx[0] + 1) * cfg['dt'] if len(idx) else None
        dm = defense_metrics(e.ledger.rows, r['world'].labels); real = ccf is not None
        rows.append(dict(kind=kind, chronos=r['first_alert'], shutdown=r['shutdown_t'], base=cf['fixed_real'], crit_cf=ccf, crit_true=r['crit_true'],
                         lead_c=(ccf - r['first_alert']) if (real and r['first_alert'] is not None) else None,
                         lead_b=(ccf - cf['fixed_real']) if (real and cf['fixed_real'] is not None and cf['fixed_real'] < ccf) else None,
                         fa_c=(not real) and (r['first_alert'] is not None), fa_b=(not real) and (r['fixed_alarm'] is not None), iforest=if0, if_fp=ifp,
                         tp=dm['TP'], fp=dm['FP'], fn=dm['FN'], real=real))
        if progress: progress((i + 1) / len(kinds), kind)
    tp = sum(x['tp'] for x in rows); fp = sum(x['fp'] for x in rows); fn = sum(x['fn'] for x in rows)
    summ = dict(brier=float(np.mean(briers)) if briers else None, precision=tp / (tp + fp) if tp + fp else None, recall=tp / (tp + fn) if tp + fn else None,
                fa_chronos=sum(x['fa_c'] for x in rows), fa_base=sum(x['fa_b'] for x in rows))
    return rows, summ


def replay_stream(models, cfg, stream, sw=None):
    e = Engine(cfg, models, sw); e.m3_every = 40; e.n_roll = 150
    for k, ev in enumerate(stream): e.step(ev, round((k + 1) * cfg['dt'], 6))
    return e


def run_all_tests(models, cfg, log=print):
    res = []
    def rec(name, ok, detail): res.append((name, bool(ok), detail)); log(('PASS ' if ok else 'FAIL ') + name + ' - ' + detail)
    sp = run_scenario(models, cfg, 'spike'); e = sp['engine']
    rec('T1 spike -> sensor fault, no shutdown', sp['shutdown_t'] is None and 'SENSOR_FAULT' in sp['modes'] and sp['world'].T < 85,
        'modes=%s, fixed-threshold alarm at %s (false alarm), CHRONOS no shutdown' % (sorted(sp['modes']), sp['fixed_alarm']))
    pu = run_scenario(models, cfg, 'pump'); cf = run_scenario(models, cfg, 'pump', engine=False)
    ok = pu['shutdown_t'] is not None and (cf['world'].crit_t is not None) and pu['world'].crit_t is None
    lead = (cf['world'].crit_t - pu['first_alert']) if (ok and pu['first_alert']) else None
    leadb = (cf['world'].crit_t - cf['fixed_alarm']) if cf['fixed_alarm'] else None
    rec('T2 real pump failure -> early auto-shutdown', ok and (cf['fixed_alarm'] is None or pu['shutdown_t'] < cf['fixed_alarm']),
        'alert t=%s, shutdown t=%s, critical (no action) t=%s -> lead %s min vs fixed-threshold lead %s min; with action max T never reached 100 C' % (pu['first_alert'], pu['shutdown_t'], cf['world'].crit_t, None if lead is None else round(lead, 1), None if leadb is None else round(leadb, 1)))
    e2 = replay_stream(models, cfg, pu['stream']); e3 = replay_stream(models, cfg, pu['stream']); h0 = replay_stream(models, cfg, pu['stream']).replay_hash()
    rec('T3 deterministic replay hash', e2.replay_hash() == e3.replay_hash() == h0 and e2.shutdown, 'replay hash %s identical on 3 replays' % h0)
    fg = run_scenario(models, cfg, 'forged'); sg = run_scenario(models, cfg, 'sig_invalid'); q1 = sum(1 for r in fg['engine'].ledger.rows if r['status'] == 'QUARANTINED' and r['source'] == 'MAINTENANCE_AGENT')
    q2 = sum(1 for r in sg['engine'].ledger.rows if r['status'] == 'QUARANTINED' and r['source'] == 'REACTOR_SENSOR_14')
    rec('T4 forged maintenance + bad-signature event quarantined', q1 >= 3 and q2 >= 1 and fg['shutdown_t'] is None and sg['shutdown_t'] is None, 'forged quarantined=%d, sig-invalid quarantined=%d, no shutdown' % (q1, q2))
    cs = run_scenario(models, cfg, 'cool_sensor'); cs_off = run_scenario(models, cfg, 'cool_sensor', sw=dict(cross=False))
    rec('T5 corrupted cooling sensor -> no shutdown, no alert (cross-check ON)', cs['shutdown_t'] is None and cs['first_alert'] is None and cs_off['first_alert'] is not None,
        'cross-check ON: shutdown=%s alert=%s | OFF (ablation): false alert at t=%s (shutdown=%s, blocked by the safety rule)' % (cs['shutdown_t'], cs['first_alert'], cs_off['first_alert'], cs_off['shutdown_t']))
    du = run_scenario(models, cfg, 'dup'); st = [r['status'] for r in du['engine'].ledger.rows]
    rec('T6 duplicate / out-of-order burst handled', st.count('DUPLICATE') >= 5 and st.count('CONFLICT') >= 1 and du['shutdown_t'] is None,
        'DUPLICATE=%d CONFLICT=%d, no shutdown' % (st.count('DUPLICATE'), st.count('CONFLICT')))
    pa = run_scenario(models, cfg, 'paradox'); dm = defense_metrics(pa['engine'].ledger.rows, pa['world'].labels)
    rec('T7 paradox: attacks contained, real emergency still shut down', pa['shutdown_t'] is not None and pa['world'].crit_t is None and (dm['recall'] or 0) >= 0.8,
        'shutdown t=%s, attack recall=%s precision=%s' % (pa['shutdown_t'], dm['recall'], dm['precision']))
    e = pu['engine']; d = e.run_decision(pu['world'].t, 'ACTIVATE_BACKUP_COOLING'); c1 = d['cert']; e.backup_used = True; d2 = e.run_decision(pu['world'].t)
    infeasible = [r for r in d2['table'] if not r['feasible']]; sts = {x['status'] for x in c1['checks']}
    e.post = np.array([0.35, 0.3, 0.3, 0.05]); c3 = e.certificate('NO_ACTION', pu['world'].t)
    rec('T8 safety certificate (PASS/FAIL/UNKNOWN, counterexample, abstain)', sts <= {'PASS', 'FAIL', 'UNKNOWN'} and 'counterexample' in c1 and infeasible and c3['abstain'],
        'statuses=%s; infeasible action rejected: %s; abstain when UNKNOWN=%s' % (sorted(sts), infeasible[0]['reason'] if infeasible else None, c3['abstain']))
    log('%d/%d tests passed' % (sum(1 for r in res if r[1]), len(res)))
    return res


class API:
    """The 6 PDF endpoints as plain Python functions (sessions kept in memory)."""
    def __init__(s, models, cfg): s.models = models; s.cfg = cfg; s.sess = {}

    def new_scenario(s, kind='pump'):
        r = run_scenario(s.models, s.cfg, kind); sid = '%s-%d' % (kind, len(s.sess) + 1); s.sess[sid] = r; return dict(scenario_id=sid)

    def post_events(s, sid, events):                                         # POST /events
        r = s.sess.setdefault(sid, dict(engine=Engine(s.cfg, s.models), stream=[])); e = r['engine']
        now = max([ev['arrival_time'] for ev in events] + [e.t + s.cfg['dt']]); n0 = len(e.ledger.rows); e.step(events, now); r['stream'].append(events)
        return [dict(event_id=x['event_id'], status=x['status'], reason=x['reason']) for x in e.ledger.rows[n0:]]

    def reconstruct(s, sid):                                                 # POST /scenarios/{id}/reconstruct
        r = s.sess[sid]; e = replay_stream(s.models, s.cfg, r['stream']) if r.get('stream') else r['engine']
        return dict(replay_hash=e.replay_hash(), original_hash=r['engine'].replay_hash() if r.get('stream') else None, alternative_histories=e.alt_histories())

    def causes(s, sid): return s.sess[sid]['engine'].causes                  # GET /scenarios/{id}/causes

    def simulate(s, sid):                                                    # POST /scenarios/{id}/simulate
        e = s.sess[sid]['engine']; d = e.run_decision(e.t); return d['table']

    def decide(s, sid):                                                      # POST /scenarios/{id}/decide
        e = s.sess[sid]['engine']; d = e.run_decision(e.t); return dict(action=d['best'], mode=e.mode, certificate=d['cert'], agents=[{k: v for k, v in a.items() if k != '_v'} for a in e.agents])

    def audit(s, sid):                                                       # GET /scenarios/{id}/audit
        e = s.sess[sid]['engine']; ok, bad = e.audit.verify(); return dict(chain_valid=ok, first_bad=bad, entries=e.audit.rows)


# ============================================================== 5b. HOME DEMO (M1-M8 live proof; every text comes from real engine values)
JD_NAMES = ['M1 Record', 'M2 Find Cause', 'M3 Predict', 'M4 Team Check', 'M5 Best Action', 'M6 Fake Data Defense', 'M7 Safety Check', 'M8 Show & Alert']
JD_WHAT = ['Saves every reading, removes duplicates, replays', 'Two possible causes, ranked by support', 'Tests each action 100+ times', 'Five AI roles check each other',
           'Picks the safest action under limits', 'Catches fake, delayed and replayed data', 'PASS, FAIL or UNKNOWN; abstains if unsure', 'Shows everything live (screen, dashboard, Telegram)']   # = the project slide text
LCD_TXT = {'NORMAL': 'SYSTEM NORMAL', 'SENSOR_FAULT': 'SENSOR FAULT', 'ALERT': 'COOLING ALERT', 'ABSTAIN': 'NOT SURE - NEED MORE EVIDENCE', 'ATTACK_CONTAINED': 'FAKE MESSAGES BLOCKED', 'SHUTDOWN': 'FACTORY SHUT DOWN DUE TO ERROR'}
JD_PACE = 5.0   # seconds of pause after each module so judges can read it (ASSUMPTION; the values are computed BEFORE the pause)


def jd_scripted(models, cfg, seed, engine_on, post, pace=0.0, man=None, paradox=True):
    """t=20 spike, t=35 paradox (or only the pump failure when paradox=False). engine_on=False -> same world, nobody acts (counterfactual)."""
    w = World(cfg, seed); w.man = dict(man or {}); e = Engine(cfg, models, None) if engine_on else None
    if e: e.n_roll = 150; e.m3_every = 40
    K = int(cfg['deadline'] / cfg['dt']); k1 = int(20.0 / cfg['dt']); k2 = int(35.0 / cfg['dt'])
    r = dict(stream=[], fixed=[], alerts=[], k_sd=None, shutdown_t=None, t=[], Tm=[], Tt=[]); pf = pa = False
    for k in range(K):
        if k == k1:
            w.inject('spike')
            if engine_on: post('stage', 't=20 min: SPIKE injected (sensor fault, no shutdown expected)'); time.sleep(pace * 0.5)
        if k == k2:
            w.inject('paradox' if paradox else 'pump')
            if engine_on: post('stage', 't=35 min: REAL pump failure + forged maintenance + bad signature + replay'); time.sleep(pace * 0.5)
        ev, m = w.tick(); r['stream'].append(ev); r['t'].append(w.t); r['Tm'].append(float(m['T'])); r['Tt'].append(float(w.T))
        f = m['T'] > cfg['T_fixed_alarm']
        if f and not pf: r['fixed'].append(w.t)
        pf = f
        if e:
            out = e.step(ev, w.t)
            for a in out['actions']: w.apply_action(a)
            al = e.mode in ('ALERT', 'ABSTAIN', 'SHUTDOWN')
            if al and not pa: r['alerts'].append(w.t)
            pa = al
            if r['shutdown_t'] is None and e.shutdown: r['shutdown_t'] = w.t; r['k_sd'] = k
    return w, e, r


def judge_worker(models, cfg, post, n_tabs, tg_ok, man=None, seed=7, pace=JD_PACE):
    cfg = dict(cfg); G, A = 'green', 'amber'; f1 = lambda v: 'n/a' if v is None else '%.1f' % v; f2 = lambda v: 'n/a' if v is None else '%.2f' % v
    res = dict(scenario='t=20 min sensor spike; t=35 min real pump failure + forged maintenance + bad signature + replay', manual_sensor_offsets=dict(man or {}), rows=[])
    def row(i, conds, txt):
        p_, n_ = sum(1 for c_ in conds if c_), len(conds); res['rows'].append(dict(module=JD_NAMES[i], checks_passed=p_, checks_total=n_, proof=txt.replace('\n', ' | ')))
        post('row', i, G if p_ == n_ else A, txt, p_, n_); time.sleep(pace)
    w, e, r = jd_scripted(models, cfg, seed, True, post, pace, man); rows = e.ledger.rows; cnt = lambda s_: sum(1 for x in rows if x['status'] == s_)
    post('stage', 'same plant again, but nobody acts (to measure the lead time) ...')
    w0, _, r0 = jd_scripted(models, cfg, seed, False, post, 0.0, man, paradox=False); crit = w0.crit_t          # no-action counterfactual (physics of the paradox = the pump failure)
    ca = next((x for x in r['alerts'] if x >= 35.0), None); fx = next((x for x in r0['fixed'] if x >= 35.0), None)
    lead = (crit - ca) if (crit is not None and ca is not None) else None; leadf = (crit - fx) if (crit is not None and fx is not None) else None
    fa_c = sum(1 for x in r['alerts'] if x < 35.0); fa_f = sum(1 for x in r0['fixed'] if x < 35.0); H = e.hist; nan = float('nan')
    post('chart', dict(t=r['t'], Tm=r['Tm'], Tt=r['Tt'], Tt0=r0['Tt'], th=[h['t'] for h in H], That=[nan if h['That'] is None else float(h['That']) for h in H],
                       marks=dict(alert=ca, shutdown=r['shutdown_t'], crit0=crit, fixed_real=fx, fixed_false=[x for x in r0['fixed'] if x < 35.0])))
    biased = abs(float((man or {}).get('T', 0.0))) >= 5.0       # a big manual sensor offset makes the fixed alarm ring falsely -> its lead time is meaningless
    leadf_txt = 'not meaningful (offset makes it ring falsely)' if biased else '%s min' % f1(leadf)
    post('num', 'lead', 'CHRONOS %s min | fixed 90 C alarm %s' % (f1(lead), leadf_txt)); post('num', 'false', 'CHRONOS %d | fixed 90 C alarm %d' % (fa_c, fa_f))
    if biased: fxl = 'A fixed %.0f C alarm is not comparable here: the %+.0f C sensor offset makes it ring falsely (%d false alarms), so its lead time is not meaningful.\n' % (cfg['T_fixed_alarm'], float(man['T']), fa_f)
    else: fxl = 'A fixed %.0f C alarm rings at t=%s min = %s min before.\n' % (cfg['T_fixed_alarm'], f1(fx), f1(leadf))
    why = ('If nobody acts, the reactor reaches %.0f C at t=%s min (same simulated plant, run twice).\nCHRONOS warns at t=%s min = %s min before.\n' % (cfg['T_crit'], f1(crit), f1(ca), f1(lead))
           + fxl + '(Simulation with ASSUMED plant numbers, not a real plant.)')
    post('why', why); res['lead_time'] = dict(no_action_critical_t=crit, chrono_alert_t=ca, fixed_alarm_t=fx, chrono_lead_min=lead, fixed_lead_min=None if biased else leadf, fixed_biased=biased,
                                            shutdown_t=r['shutdown_t'], false_alarms_chronos=fa_c, false_alarms_fixed=fa_f, explanation=why.replace('\n', ' '))
    post('stage', 'filling the 8 modules from the real engine ...')
    e2 = replay_stream(models, cfg, r['stream']); h1, h2 = e.replay_hash(), e2.replay_hash()                      # M1
    row(0, [h1 == h2, cnt('DUPLICATE') + cnt('QUARANTINED') > 0], 'ACC %d  DUP %d\nQUAR %d  CONF %d\nreplay %s = %s' % (cnt('ACCEPTED'), cnt('DUPLICATE'), cnt('QUARANTINED'), cnt('CONFLICT'), h1[:8], h2[:8]))
    ks = r['k_sd'] if r['k_sd'] is not None else len(r['stream']) - 1                                           # decision snapshot just before the shutdown tick
    sn = replay_stream(models, cfg, r['stream'][:ks]); sn.n_roll = cfg['n_roll']; sn.sw['compromise'] = 'Predictor'
    post('comp', True); sn.run_decision(round(ks * cfg['dt'], 6))
    cs = sn.causes[:2]; sp = next((x for x in (sn.spur, e.spur) if x and x.get('spurious')), None)                 # M2
    row(1, [len(cs) == 2, all(q.get('label') for q in cs)], '\n'.join('%s %.2f (%s)' % (q['cause'], q['p'], q['label']) for q in cs) + ('\nspurious correlation rejected (partial r = %.2f)' % sp['partial'] if sp else ''))
    tb = {x['action']: x for x in sn.opt['table']}; bx = tb[sn.opt['best']]                                      # M3
    row(2, [sn.n_roll >= 100, len(tb) == len(ACTIONS), bx['lo'] <= bx['hi']], '%d rollouts x %d actions\nbest %s: loss [%.1f, %.1f]\nno action: [%.1f, %.1f]' % (sn.n_roll, len(tb), sn.opt['best'], bx['lo'], bx['hi'], tb['NO_ACTION']['lo'], tb['NO_ACTION']['hi']))
    ov = [a for a in sn.agents if a['status'].startswith('OVERRULED')]                                          # M4
    row(3, [len(sn.agents) == 5, bool(ov), all(a['llm_weight'] == 0 for a in sn.agents)], ('%s OVERRULED\n(lied: "%s")' % (ov[0]['agent'], ov[0]['claim']) if ov else 'no agent overruled') + '\nLLM weight = 0'); post('comp', False)
    b = sn.opt['baselines']; de = e.run_decision(w.t); rej = next((x for x in de['table'] if not x['feasible']), None)  # M5 (rejected action taken from the final state)
    row(4, [b['optimized'] <= b['no_action'], b['optimized'] <= b['greedy']], 'J best %.1f\nno-action %.1f\ngreedy %.1f' % (b['optimized'], b['no_action'], b['greedy']) + ('\nrejected %s:\n%s' % (rej['action'], rej['reason']) if rej else ''))
    dm = defense_metrics(rows, w.labels); P, R = dm['precision'], dm['recall']                                   # M6 (labels = evaluation only)
    row(5, [P is not None and P >= 0.9, R is not None and R >= 0.9, r['shutdown_t'] is not None], 'precision %s  recall %s\n(TP %d FP %d FN %d)\nreal emergency kept:\nshutdown at t=%s' % (f2(P), f2(R), dm['TP'], dm['FP'], dm['FN'], f1(r['shutdown_t'])))   # 0.9 = ASSUMPTION
    c = sn.cert; nr = sum(1 for x in c['checks'] if x['method'] == 'rule'); ne = len(c['checks']) - nr           # M7
    row(6, [c['overall'] in ('PASS', 'FAIL', 'UNKNOWN'), c['overall'] != 'UNKNOWN' or c['abstain'], bool(c['counterexample'])], '%s\n%d rule + %d empirical checks\ncounterexample: %s\nabstain: %s' % (c['overall'], nr, ne, 'yes' if c['counterexample'] else 'no', 'yes' if c['abstain'] else 'no'))
    lcd = LCD_TXT.get(e.mode, e.mode); post('final', dict(lcd=lcd, shutdown_t=r['shutdown_t']))                  # M8
    post('stage', 'calling the API endpoints ...')
    api = API(models, cfg); out = []; sid = [None]
    def step_(name, fn, chk):
        try: out.append((name, bool(chk(fn()))))
        except Exception: out.append((name, False))
    step_('new_scenario', lambda: api.new_scenario('pump'), lambda x: sid.__setitem__(0, x['scenario_id']) or bool(x['scenario_id']))
    if sid[0]:
        step_('events', lambda: api.post_events(sid[0], []), lambda x: isinstance(x, list))
        step_('reconstruct', lambda: api.reconstruct(sid[0]), lambda x: x['replay_hash'] == x['original_hash'])
        step_('causes', lambda: api.causes(sid[0]), lambda x: len(x) >= 1)
        step_('simulate', lambda: api.simulate(sid[0]), lambda x: len(x) == 5)
        step_('decide', lambda: api.decide(sid[0]), lambda x: bool(x['action']))
        step_('audit', lambda: api.audit(sid[0]), lambda x: x['chain_valid'] is True)
    nok = sum(1 for _, o in out if o)
    row(7, [nok == 7, bool(lcd), n_tabs >= 1], 'screen: "%s"\n%d tabs, API %d/7 OK%s\nTelegram: %s' % (lcd, n_tabs, nok, '' if nok == 7 else ' (failed: ' + ','.join(n for n, o in out if not o) + ')', 'configured' if tg_ok else 'not configured'))
    post('summary', res); post('stage', 'running tests T1-T8 ...'); tr = run_all_tests(models, dict(cfg), log=lambda x: None)
    post('tests', sum(1 for x in tr if x[1]), len(tr), [x[0].split(' ')[0] for x in tr if not x[1]], [(x[0], x[1], x[2]) for x in tr])


REPORT_Q = ('Write a short report for non-expert judges in simple English (about 14 lines). Start with one line: what happened in the demo. Then one short sentence per module M1 to M8 '
            '(say what it did and the proof number from STATE_JSON). Then explain in 2 sentences how the lead time is calculated (the same plant is run twice: once with CHRONOS, once with no action). '
            'End with one honest line: this is a simulation with assumed plant numbers.')


def template_report(res):
    L = ['CHRONOS-X demo report (plain template, no AI used)', 'Scenario: ' + res.get('scenario', '-'), '']
    for r, w in zip(res.get('rows', []), JD_WHAT): L.append('%s - %d of %d checks passed. %s.\n    Proof: %s' % (r['module'], r['checks_passed'], r['checks_total'], w, r['proof']))
    lt = res.get('lead_time')
    if lt: L += ['', 'Lead time: ' + lt['explanation']]
    if res.get('tests_passed'): L.append('Self-tests passed: ' + res['tests_passed'])
    return '\n'.join(L)


def serve_http(port, models, cfg):
    from http.server import BaseHTTPRequestHandler, HTTPServer
    api = API(models, cfg)

    class H(BaseHTTPRequestHandler):
        def _send(s, obj, code=200):
            b = json.dumps(obj, default=str).encode(); s.send_response(code); s.send_header('Content-Type', 'application/json'); s.send_header('Content-Length', str(len(b))); s.end_headers(); s.wfile.write(b)

        def _route(s, method):
            p = s.path.strip('/').split('/'); n = int(s.headers.get('Content-Length', 0) or 0); body = json.loads(s.rfile.read(n) or b'{}') if n else {}
            try:
                if method == 'POST' and p == ['events']: return s._send(api.post_events(body.get('scenario_id', 'live'), body.get('events', [])))
                if method == 'POST' and p == ['scenarios']: return s._send(api.new_scenario(body.get('kind', 'pump')))
                if len(p) == 3 and p[0] == 'scenarios':
                    f = {('POST', 'reconstruct'): api.reconstruct, ('GET', 'causes'): api.causes, ('POST', 'simulate'): api.simulate,
                         ('POST', 'decide'): api.decide, ('GET', 'audit'): api.audit}.get((method, p[2]))
                    if f: return s._send(f(p[1]))
                s._send(dict(error='not found'), 404)
            except Exception as ex: s._send(dict(error=str(ex)), 400)

        def do_GET(s): s._route('GET')
        def do_POST(s): s._route('POST')
    print('CHRONOS-X API on http://127.0.0.1:%d' % port); HTTPServer(('127.0.0.1', port), H).serve_forever()


# ============================================================== 6. GROQ + TELEGRAM (keys only in memory)
GROQ_MODELS = ['allam-2-7b', 'openai/gpt-oss-120b', 'openai/gpt-oss-20b', 'qwen/qwen3-32b']   # verify / refresh in Settings


def _scrub(txt, *secrets):
    txt = str(txt)
    for x in secrets:
        if x: txt = txt.replace(x, '***')
    return txt


def groq_chat(key, model, question, state):
    sysmsg = ("You are the explanation layer of CHRONOS-X. Reply in simple, clear English for a non-expert, short sentences, about 6 lines unless the question asks for more. Use ONLY facts from STATE_JSON. "
              "If something is not in STATE_JSON say 'not available'. You have ZERO decision authority; never tell people to ignore alarms.")
    r = requests.post('https://api.groq.com/openai/v1/chat/completions', headers={'Authorization': 'Bearer ' + key, 'Content-Type': 'application/json'},
                      json=dict(model=model, temperature=0.2, max_tokens=800, messages=[dict(role='system', content=sysmsg),
                                dict(role='user', content='STATE_JSON:\n' + json.dumps(state) + '\n\nQUESTION: ' + question)]), timeout=25)
    r.raise_for_status(); return r.json()['choices'][0]['message']['content'].strip()


def groq_models(key):
    r = requests.get('https://api.groq.com/openai/v1/models', headers={'Authorization': 'Bearer ' + key}, timeout=15); r.raise_for_status()
    return sorted(x['id'] for x in r.json()['data'])


def telegram_send(token, chat, text):
    r = requests.post('https://api.telegram.org/bot%s/sendMessage' % token, json=dict(chat_id=chat, text=text), timeout=15); r.raise_for_status(); return True


def tg_chunks(text, limit=3800):
    """Telegram allows 4096 characters per message -> cut the report on line boundaries."""
    out, cur = [], ''
    for line in str(text).split('\n'):
        while len(line) > limit:
            if cur: out.append(cur); cur = ''
            out.append(line[:limit]); line = line[limit:]
        if cur and len(cur) + len(line) + 1 > limit: out.append(cur); cur = line
        else: cur = (cur + '\n' + line) if cur else line
    if cur: out.append(cur)
    return out


def telegram_send_long(token, chat, text, limit=3800):
    parts = tg_chunks(text, limit)
    for i, part in enumerate(parts):
        telegram_send(token, chat, ('(%d/%d)\n' % (i + 1, len(parts)) if len(parts) > 1 else '') + part)
        if i < len(parts) - 1: time.sleep(0.6)
    return len(parts)


def build_full_report(res, rep_text='', suite=None, alerts=None, seed=7):
    """One plain-text report (lead time + M1-M8 proof + T1-T8 + evaluation suite + live alerts + explanation) for Telegram. Every number comes from the real runs."""
    f1 = lambda v: 'n/a' if v is None else '%.1f' % v; f2 = lambda v: 'n/a' if v is None else '%.2f' % v
    L = ['CHRONOS-X - FULL REPORT', time.strftime('%Y-%m-%d %H:%M'), 'SIMULATION with ASSUMED plant numbers (seed %s). Not a real plant, not a certified controller.' % seed, '']
    lt = res.get('lead_time')
    if lt:
        L += ['LEAD TIME', ' Scenario: ' + res.get('scenario', '-'), ' Critical (100 C) if nobody acts: t=%s min' % f1(lt['no_action_critical_t']),
              ' CHRONOS first alert: t=%s min -> lead %s min' % (f1(lt['chrono_alert_t']), f1(lt['chrono_lead_min']))]
        if lt.get('shutdown_t') is not None and lt.get('no_action_critical_t') is not None:
            L.append(' CHRONOS shutdown: t=%s min -> safety margin after shutdown %s min' % (f1(lt['shutdown_t']), f1(lt['no_action_critical_t'] - lt['shutdown_t'])))
        if lt.get('fixed_biased'): L.append(' Fixed 90 C alarm: not comparable (the sensor offset makes it ring falsely)')
        else: L.append(' Fixed 90 C alarm: t=%s min -> lead %s min' % (f1(lt['fixed_alarm_t']), f1(lt['fixed_lead_min'])))
        L += [' False alarms before the failure: CHRONOS %d, fixed 90 C alarm %d' % (lt['false_alarms_chronos'], lt['false_alarms_fixed']),
              ' Method: the same plant and seed are run twice (CHRONOS acting / nobody acting). Lead = critical time minus alert time.', '']
    else: L += ['Home demo not run yet (press "Run 60-second demo" on the Home tab for the lead time and the M1-M8 proof).', '']
    if res.get('rows'):
        L.append('MODULES M1-M8')
        for r in res['rows']: L.append(' %s: %d/%d checks - %s' % (r['module'], r['checks_passed'], r['checks_total'], r['proof']))
        L.append('')
    tests = res.get('tests')
    if tests:
        L.append('SELF-TESTS T1-T8: %s passed' % res.get('tests_passed', '%d/%d' % (sum(1 for x in tests if x[1]), len(tests))))
        for name, ok, detail in tests: L.append(' %s %s' % ('PASS' if ok else 'FAIL', name) + ('' if ok else ' - ' + str(detail)[:160]))
        L.append('')
    if suite:
        rows, summ = suite; real = [r for r in rows if r['real']]
        lc = [r['lead_c'] for r in real if r['lead_c'] is not None]; lb = [r['lead_b'] for r in real if r['lead_b'] is not None]
        L += ['EVALUATION SUITE (all scenarios)', ' Mean lead time (real failures): CHRONOS %s min vs fixed 90 C alarm %s min' % (f2(float(np.mean(lc)) if lc else None), f2(float(np.mean(lb)) if lb else None)),
              ' False alarms (benign scenarios): CHRONOS %d vs fixed alarm %d' % (summ['fa_chronos'], summ['fa_base']),
              ' Brier score (P_crit) %s | attack defense precision %s recall %s' % (f2(summ['brier']), f2(summ['precision']), f2(summ['recall']))]
        for r in real: L.append('  %s: alert %s, shutdown %s, critical %s, lead %s (fixed-alarm lead %s)' % (r['kind'], f1(r['chronos']), f1(r['shutdown']), f1(r['crit_cf']), f1(r['lead_c']), f1(r['lead_b'])))
        L.append('')
    if alerts:
        L.append('LIVE ALERTS (last 8)')
        for t_, m_ in alerts[-8:]: L.append(' t=%.1f %s' % (t_, str(m_).split('\n')[0]))
        L.append('')
    if rep_text: L += ['EXPLANATION', rep_text.strip(), '']
    L.append('Honest limits: simulator only; plant numbers are assumptions; the shutdown happens after the alert, so the real safety margin is shorter than the lead time.')
    return '\n'.join(L)


def local_answer(q, st):
    ql = q.lower(); top = st['ranked_causes'][0] if st['ranked_causes'] else {}
    if 'trust' in ql or 'bharosa' in ql:
        return 'Trust (maths, 0-1):\n' + '\n'.join('  %s: %.2f (organizer %.2f)' % (k, v['our_trust'], v['organizer_trust']) for k, v in st['trust_per_source'].items())
    base = 'Mode: %s. Estimated T=%.1f C, P(critical)=%.0f%%, time-to-critical=%s. ' % (st['mode'], st['temperature_estimate_C'], 100 * st['p_critical'], st['time_to_critical_min'])
    why = 'Top cause: %s (p=%.2f, %s). ' % (top.get('cause'), top.get('p', 0), top.get('label'))
    ev = 'Evidence: ' + ('; '.join(st['active_evidence']) if st['active_evidence'] else 'nothing abnormal') + '. Independent cooling sources abnormal: %d.' % st['independent_cooling_sources_abnormal']
    return base + why + ev


# ============================================================== 7. TKINTER UI  (clear WHITE, full screen)
try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    HAVE_TK = True
except Exception:
    HAVE_TK = False

BG = '#FFFFFF'; BORDER = '#E5E7EB'; TXT = '#111827'; MUTED = '#6B7280'; BLUE = '#2563EB'; RED = '#DC2626'; AMBER = '#F59E0B'; GREEN = '#16A34A'; SOFT = '#F3F4F6'
MODE_COL = {'NORMAL': GREEN, 'SENSOR_FAULT': AMBER, 'ATTACK_CONTAINED': AMBER, 'ALERT': AMBER, 'ABSTAIN': AMBER, 'SHUTDOWN': RED}
F_N = ('Segoe UI', 10); F_B = ('Segoe UI', 10, 'bold'); F_H = ('Segoe UI', 13, 'bold'); F_BIG = ('Segoe UI', 20, 'bold'); F_M = ('Consolas', 9)


class App:
    def __init__(s, root):
        s.root = root; s.cfg = dict(CFG); s.models = None; s.world = None; s.engine = None; s.running = False; s.acc = 0.0
        s.stream = []; s.q = queue.Queue(); s.frame = 0; s.drawn_inj = 0; s.alerts = []; s.fs = True; s.suite_rows = None
        root.title('CHRONOS-X'); root.configure(bg=BG)
        try: root.attributes('-fullscreen', True)
        except Exception:
            try: root.state('zoomed')
            except Exception: pass
        root.bind('<F11>', lambda e: s.toggle_fs()); root.bind('<Escape>', lambda e: s.toggle_fs(False))
        s.style(); s.header(); s.tabs(); s.training_overlay(); s.root.after(100, s.train_step_init)

    # ---------- infrastructure
    def toggle_fs(s, v=None):
        s.fs = (not s.fs) if v is None else v
        try: s.root.attributes('-fullscreen', s.fs)
        except Exception: pass

    def style(s):
        st = ttk.Style(); st.theme_use('clam')
        st.configure('.', background=BG, foreground=TXT, font=F_N); st.configure('TFrame', background=BG); st.configure('TLabel', background=BG, foreground=TXT)
        st.configure('TNotebook', background=BG, borderwidth=0); st.configure('TNotebook.Tab', background=SOFT, foreground=MUTED, padding=(14, 7), font=F_B, borderwidth=0)
        st.map('TNotebook.Tab', background=[('selected', BG)], foreground=[('selected', BLUE)])
        st.configure('Treeview', background=BG, fieldbackground=BG, foreground=TXT, rowheight=24, bordercolor=BORDER, borderwidth=0)
        st.configure('Treeview.Heading', background=SOFT, foreground=TXT, font=F_B, relief='flat'); st.map('Treeview', background=[('selected', '#DBEAFE')], foreground=[('selected', TXT)])
        st.configure('TButton', background=BG, foreground=TXT, bordercolor=BORDER, padding=(10, 5), relief='flat'); st.map('TButton', background=[('active', SOFT)])
        st.configure('Accent.TButton', background=BLUE, foreground='white', bordercolor=BLUE, font=F_B); st.map('Accent.TButton', background=[('active', '#1D4ED8')])
        st.configure('Danger.TButton', background=RED, foreground='white', bordercolor=RED, font=F_B); st.map('Danger.TButton', background=[('active', '#B91C1C')])
        st.configure('TCheckbutton', background=BG); st.configure('TScale', background=BG); st.configure('TEntry', fieldbackground=BG, bordercolor=BORDER)
        st.configure('TCombobox', fieldbackground=BG, bordercolor=BORDER); st.configure('Horizontal.TProgressbar', background=BLUE, troughcolor=SOFT, bordercolor=BORDER)

    def card(s, parent, title=None):
        f = tk.Frame(parent, bg=BG, highlightbackground=BORDER, highlightthickness=1, bd=0)
        if title: tk.Label(f, text=title, bg=BG, fg=MUTED, font=F_B).pack(anchor='w', padx=10, pady=(8, 2))
        return f

    def tree(s, parent, cols, widths, height=8):
        fr = tk.Frame(parent, bg=BG); t = ttk.Treeview(fr, columns=cols, show='headings', height=height); sb = ttk.Scrollbar(fr, orient='vertical', command=t.yview)
        t.configure(yscrollcommand=sb.set)
        for c_, w in zip(cols, widths): t.heading(c_, text=c_); t.column(c_, width=w, anchor='w')
        t.pack(side='left', fill='both', expand=True); sb.pack(side='right', fill='y'); t.frame = fr; return t

    def fig(s, parent, nrows=1, ncols=1, **kw):
        f = Figure(facecolor='white', constrained_layout=True); axs = f.subplots(nrows, ncols, squeeze=False, **kw)
        cv = FigureCanvasTkAgg(f, master=parent); cv.get_tk_widget().configure(bg='white', highlightthickness=0)
        for ax in axs.ravel(): ax.set_facecolor('white'); ax.grid(True, color='#EEF0F3', lw=0.8); [sp.set_color('#D1D5DB') for sp in ax.spines.values()]; ax.tick_params(colors=MUTED, labelsize=8)
        return f, axs, cv

    def text(s, parent, h=10, mono=True):
        t = tk.Text(parent, bg=BG, fg=TXT, relief='flat', height=h, wrap='word', font=F_M if mono else F_N, highlightbackground=BORDER, highlightthickness=1, padx=8, pady=6)
        return t

    def put(s, t, content):
        t.configure(state='normal'); t.delete('1.0', 'end'); t.insert('end', content); t.configure(state='disabled')

    # ---------- header
    def header(s):
        h = tk.Frame(s.root, bg=BG); h.pack(fill='x', padx=14, pady=(10, 4))
        tk.Label(h, text='CHRONOS-X', bg=BG, fg=BLUE, font=('Segoe UI', 18, 'bold')).pack(side='left')
        tk.Label(h, text='  predict the catastrophe before it happens  (software proof-of-concept)', bg=BG, fg=MUTED, font=F_N).pack(side='left')
        s.pill = tk.Label(h, text='TRAINING', bg=AMBER, fg='white', font=F_B, padx=14, pady=4); s.pill.pack(side='right')
        ttk.Button(h, text='Full screen (F11)', command=s.toggle_fs).pack(side='right', padx=8)
        s.clock = tk.Label(h, text='t = 0.0 / 90 min', bg=BG, fg=TXT, font=F_B); s.clock.pack(side='right', padx=12)
        c = tk.Frame(s.root, bg=BG); c.pack(fill='x', padx=14, pady=(0, 6))
        s.b_run = ttk.Button(c, text='Start', style='Accent.TButton', command=s.toggle_run); s.b_run.pack(side='left')
        ttk.Button(c, text='Step', command=lambda: s.do_tick(True)).pack(side='left', padx=6); ttk.Button(c, text='Reset', command=s.reset).pack(side='left')
        tk.Label(c, text='   Speed (ticks/s)', bg=BG, fg=MUTED).pack(side='left'); s.speed = tk.DoubleVar(value=10)
        ttk.Scale(c, from_=1, to=60, variable=s.speed, length=200).pack(side='left', padx=6)
        tk.Label(c, text='   Seed', bg=BG, fg=MUTED).pack(side='left'); s.seed_v = tk.StringVar(value=str(s.cfg['seed'])); ttk.Entry(c, textvariable=s.seed_v, width=6).pack(side='left', padx=4)
        s.dev_lbl = tk.Label(c, text='', bg=BG, fg=MUTED); s.dev_lbl.pack(side='right')

    def tabs(s):
        s.nb = ttk.Notebook(s.root); s.nb.pack(fill='both', expand=True, padx=10, pady=(0, 10)); s.tab = {}
        for n in ('Home', 'Report', 'Live Monitor', 'Fault Injection', 'Metrics', 'Telegram', 'Hypotheses', 'Simulation', 'Decision', 'Agents', 'Timeline', 'Audit', 'Settings'):
            f = tk.Frame(s.nb, bg=BG); s.nb.add(f, text=n); s.tab[n] = f
        s.build_home(); s.build_report(); s.build_live(); s.build_faults(); s.build_hyp(); s.build_sim(); s.build_dec(); s.build_agents(); s.build_time(); s.build_audit(); s.build_metrics(); s.build_telegram(); s.build_settings()
        for n_ in ('Hypotheses', 'Simulation', 'Decision', 'Agents', 'Timeline', 'Audit'): s.nb.hide(s.tab[n_])   # these stay built (engine code uses them) but hidden; Live Monitor, Fault Injection, Metrics, Telegram are visible
        s.nb.select(s.tab['Home'])

    # ---------- training overlay
    def training_overlay(s):
        s.ov = tk.Frame(s.root, bg=BG); s.ov.place(relx=0, rely=0, relwidth=1, relheight=1)
        box = tk.Frame(s.ov, bg=BG, highlightbackground=BORDER, highlightthickness=1); box.place(relx=0.5, rely=0.45, anchor='center')
        tk.Label(box, text='CHRONOS-X  -  training on simulated NORMAL data', bg=BG, fg=TXT, font=F_H).pack(padx=40, pady=(24, 6))
        s.tr_msg = tk.Label(box, text='starting...', bg=BG, fg=MUTED, font=F_N); s.tr_msg.pack()
        s.tr_bar = ttk.Progressbar(box, length=460, maximum=1.0); s.tr_bar.pack(padx=40, pady=14)
        s.tr_lim = tk.Label(box, text='', bg=BG, fg=TXT, font=F_M, justify='left'); s.tr_lim.pack(padx=40, pady=(0, 24))

    def train_step_init(s): s.gen = train_gen(s.cfg); s.train_step()

    def train_step(s):
        try:
            p, msg, m = next(s.gen); s.tr_bar['value'] = p; s.tr_msg.config(text=msg)
            if m: s.models = m; s.tr_lim.config(text=m['summary']); s.root.after(900, s.ready)
            else: s.root.after(1, s.train_step)
        except StopIteration: pass

    def ready(s):
        s.ov.place_forget(); s.set_lim_text(); s.reset(); s.loop()

    def set_lim_text(s): s.put(s.lim_txt, s.models['summary'])

    # ---------- run control
    def reset(s):
        try: s.cfg['seed'] = int(s.seed_v.get())
        except Exception: pass
        s.running = False; s.b_run.config(text='Start'); s.pending_inj = None; s.world = World(s.cfg, s.cfg['seed']); sw = s.engine.sw if s.engine else None
        s.engine = Engine(s.cfg, s.models, sw); s.stream = []; s.drawn_inj = 0; s.alerts = []; s.acc = 0.0
        for ax in s.live_axs.ravel(): ax.clear()
        s.live_init(); s.refresh(True); s.put_chat('CHRONOS: ready. Start the run, inject a fault, then ask "why?" or "trust?".\n')

    def toggle_run(s):
        s.running = not s.running; s.b_run.config(text='Pause' if s.running else 'Start')

    def inject(s, kind):
        if not s.world: return
        s.world.inject(kind); s.engine.audit.add(s.world.t, 'OPERATOR_INJECT', dict(kind=kind)); s.status('Injected: ' + kind)
        if not s.running: s.toggle_run()

    def do_tick(s, manual=False):
        if not s.engine or s.world.t >= s.cfg['deadline'] - 1e-6: s.running = False; s.b_run.config(text='Start'); return
        pi = getattr(s, 'pending_inj', None)
        if pi and s.world.t >= pi[0] - 1e-9: s.pending_inj = None; s.inject(pi[1])      # 'Run injection' from Home fires at the chosen minute
        ev, m = s.world.tick(); out = s.engine.step(ev, s.world.t); s.stream.append(ev)
        for a in out['actions']: s.world.apply_action(a)
        for mode, msg in out['msgs']: s.notify(mode, msg)
        if manual: s.refresh(True)

    def loop(s):
        if s.running and s.engine:
            s.acc += s.speed.get() * 0.05; n = 0
            while s.acc >= 1 and n < 30: s.acc -= 1; s.do_tick(); n += 1
            if s.world.t >= s.cfg['deadline'] - 1e-6: s.running = False; s.b_run.config(text='Start')
        s.frame += 1; s.refresh(); s.poll(); s.root.after(50, s.loop)

    def status(s, t): s.dev_lbl.config(text=t)

    def poll(s):
        try:
            while True:
                kind, payload = s.q.get_nowait()
                if kind == 'chat': s.put_chat('CHRONOS: ' + payload + '\n')
                elif kind == 'log': s.log_line(payload)
                elif kind == 'models': s.cb_model['values'] = payload; s.log_line('Groq models refreshed (%d)' % len(payload))
                elif kind == 'tests': s.tests_txt.insert('end', payload + '\n'); s.tests_txt.see('end')
                elif kind == 'suite': s.show_suite(payload)
                elif kind == 'judge': s.judge_update(payload)
                elif kind == 'report': s.put(s.rep_txt, payload); s.rep_lbl.config(text='report ready')
                elif kind == 'tgstat': s.tg_lbl.config(text=payload[1], fg=GREEN if payload[0] == 'ok' else RED); s.log_line(payload[1])
        except queue.Empty: pass

    # ---------- notifications
    def notify(s, mode, msg):
        s.alerts.append((round(s.world.t, 1), msg)); s.log_line('[t=%.1f] %s' % (s.world.t, msg.split('\n')[0]))
        tok, chat = s.tg_tok.get().strip() or os.environ.get('TELEGRAM_BOT_TOKEN', ''), s.tg_chat.get().strip() or os.environ.get('TELEGRAM_CHAT_ID', '')
        if tok and chat and requests:
            def work():
                try: telegram_send(tok, chat, msg); s.q.put(('log', 'Telegram sent'))
                except Exception as ex: s.q.put(('log', 'Telegram failed (%s) - template text kept' % _scrub(ex, tok)[:90]))
            threading.Thread(target=work, daemon=True).start()

    def log_line(s, t): s.alert_txt.configure(state='normal'); s.alert_txt.insert('end', t + '\n'); s.alert_txt.see('end'); s.alert_txt.configure(state='disabled')

    # ================= HOME (tab 1): 8 donuts M1-M8, one chart, flow chart, sliders. Heavy work in a thread, results via s.q
    def build_home(s):
        t = s.tab['Home']; s.jd_busy = False; s.jd_result = {}; s.jd_state = [None] * 8; s.jd_final = ''; s.jd_grey = '#9CA3AF'
        st = ttk.Style(); st.configure('Big.TButton', background=BLUE, foreground='white', font=('Segoe UI', 14, 'bold'), padding=(30, 10)); st.map('Big.TButton', background=[('active', '#1D4ED8')])
        top = tk.Frame(t, bg=BG); top.pack(fill='x', padx=12, pady=(8, 2))
        s.jd_btn = ttk.Button(top, text='Run 60-second demo (optional)', style='Big.TButton', command=s.judge_start); s.jd_btn.pack(side='left')
        s.jd_stage = tk.Label(top, text='Press Run: spike at t=20 min, real failure + attacks at t=35 min', bg=BG, fg=MUTED, font=F_B); s.jd_stage.pack(side='left', padx=14)
        s.jd_comp = tk.Label(top, text='', bg=BG, fg=RED, font=F_B); s.jd_comp.pack(side='left')
        ctl = tk.Frame(top, bg=BG); ctl.pack(side='right'); s.sv_T = tk.DoubleVar(value=0.0); s.sv_F = tk.DoubleVar(value=0.0); s.sv_on = tk.BooleanVar(value=False)
        tk.Label(ctl, text='Temperature sensor offset (C)', bg=BG, fg=MUTED, font=F_N).pack(side='left')
        tk.Scale(ctl, from_=-20, to=20, resolution=1, orient='horizontal', variable=s.sv_T, length=150, bg=BG, highlightthickness=0, troughcolor=SOFT).pack(side='left', padx=(4, 12))
        ttk.Checkbutton(ctl, text='cooling sensor too (optional)', variable=s.sv_on).pack(side='left')
        tk.Scale(ctl, from_=-0.5, to=0.5, resolution=0.05, orient='horizontal', variable=s.sv_F, length=130, bg=BG, highlightthickness=0, troughcolor=SOFT).pack(side='left', padx=4)
        inj = tk.Frame(t, bg=BG); inj.pack(fill='x', padx=12, pady=(2, 0))
        tk.Label(inj, text='Live injection:', bg=BG, fg=MUTED, font=F_B).pack(side='left')
        s.inj_var = tk.StringVar(value=dict((k, ti) for k, ti, _ in FAULTS)['pump'])
        ttk.Combobox(inj, textvariable=s.inj_var, values=[ti for _, ti, _ in FAULTS], width=34, state='readonly').pack(side='left', padx=6)
        tk.Label(inj, text='at t =', bg=BG, fg=MUTED).pack(side='left'); s.inj_t = tk.StringVar(value='20'); ttk.Entry(inj, textvariable=s.inj_t, width=5).pack(side='left', padx=4); tk.Label(inj, text='min', bg=BG, fg=MUTED).pack(side='left')
        ttk.Button(inj, text='Run injection', style='Danger.TButton', command=s.run_injection).pack(side='left', padx=10)
        tk.Label(inj, text='starts the live run and injects the chosen fault at that minute - watch it on the Live Monitor tab (more faults: Fault Injection tab)', bg=BG, fg=MUTED, font=('Segoe UI', 8)).pack(side='left')
        g = tk.Frame(t, bg=BG); g.pack(fill='x', padx=12, pady=4); s.jd_cv = []; s.jd_pl = []
        for i, nm in enumerate(JD_NAMES):
            c = s.card(g); c.grid(row=0, column=i, sticky='nsew', padx=3); g.columnconfigure(i, weight=1, uniform='g')
            cv = tk.Canvas(c, width=100, height=100, bg=BG, highlightthickness=0); cv.pack(pady=(6, 0)); s.jd_cv.append(cv); s.donut(cv, 0, 0, s.jd_grey)
            tk.Label(c, text=nm, bg=BG, fg=TXT, font=F_B).pack(); tk.Label(c, text=JD_WHAT[i], bg=BG, fg=MUTED, font=('Segoe UI', 8), wraplength=150, justify='center').pack()
            l = tk.Label(c, text='-', bg=BG, fg=TXT, font=('Segoe UI', 8, 'bold'), wraplength=150, justify='center', height=7); l.pack(pady=(4, 6)); s.jd_pl.append(l)
        mid = tk.Frame(t, bg=BG); mid.pack(fill='both', expand=True, padx=12, pady=4)
        cc = s.card(mid, 'TEMPERATURE  -  what would happen with NO action  -  every event on one timeline'); cc.pack(side='left', fill='both', expand=True)
        s.hf, s.hax, s.hcv = s.fig(cc, 2, 1, sharex=True, gridspec_kw=dict(height_ratios=[3, 1])); s.hcv.get_tk_widget().pack(fill='both', expand=True, padx=4, pady=4); s.draw_chart(None)
        rt = tk.Frame(mid, bg=BG, width=390); rt.pack(side='right', fill='y', padx=(8, 0)); rt.pack_propagate(False)
        c = s.card(rt, 'WHERE DOES THE LEAD TIME COME FROM?'); c.pack(fill='x'); s.jd_why = tk.Label(c, text='Run the demo.', bg=BG, fg=TXT, font=F_N, wraplength=360, justify='left'); s.jd_why.pack(anchor='w', padx=10, pady=(2, 8))
        s.jd_num = {}
        for key, ttl in (('lead', 'LEAD TIME before the reactor becomes critical'), ('false', 'FALSE ALARMS (sensor spike at t=20)'), ('tests', 'TESTS PASSED')):
            c = s.card(rt, ttl); c.pack(fill='x', pady=(6, 0)); l = tk.Label(c, text='-', bg=BG, fg=BLUE, font=('Segoe UI', 15, 'bold'), wraplength=360, justify='left'); l.pack(anchor='w', padx=10, pady=(0, 6)); s.jd_num[key] = l
        s.jd_flow = tk.Canvas(t, height=96, bg=BG, highlightthickness=1, highlightbackground=BORDER); s.jd_flow.pack(fill='x', padx=12, pady=(2, 8)); s.jd_flow.bind('<Configure>', lambda e: s.draw_flow()); s.draw_flow()

    def run_injection(s):
        if not s.world or not s.models: return
        kind = next((k for k, ti, _ in FAULTS if ti == s.inj_var.get()), 'pump')
        try: at = max(0.0, float(s.inj_t.get()))
        except Exception: at = 20.0
        if s.world.t >= s.cfg['deadline'] - 1e-6: s.reset()
        s.pending_inj = (at, kind); s.nb.select(s.tab['Live Monitor'])
        if not s.running: s.toggle_run()
        s.status('Injection armed: %s at t=%.1f min' % (kind, at))

    def donut(s, cv, p, n, col):
        cv.delete('all'); cv.create_oval(12, 12, 88, 88, outline='#E5E7EB', width=12)
        if n and p: cv.create_arc(12, 12, 88, 88, start=90, extent=-359.9 * p / n, style='arc', outline=col, width=12)
        cv.create_text(50, 50, text=('%d/%d' % (p, n)) if n else '-', font=('Segoe UI', 14, 'bold'), fill=TXT)

    def draw_flow(s):
        c = s.jd_flow; c.delete('all'); W = max(c.winfo_width(), 900); n = 9; gap = 24; bw = (W - 16 - gap * (n - 1)) / n; y0, y1 = 10, 62
        steps = [('Sensors\n(temp, flow,\nvibration, power)', None), ('M6 Fake data\nfilter', 5), ('M1 Record', 0), ('M2 Find\nCause', 1), ('M3 Predict', 2), ('M4 Team\nCheck', 3), ('M5 Best\nAction', 4), ('M7 Safety\nCheck', 6), ('M8 Action\n+ Alert', 7)]
        for i, (txt, k) in enumerate(steps):
            x0 = 8 + i * (bw + gap); col = BLUE if k is None else {'green': GREEN, 'amber': AMBER}.get(s.jd_state[k], s.jd_grey)
            c.create_rectangle(x0, y0, x0 + bw, y1, fill=col, outline=''); c.create_text(x0 + bw / 2, (y0 + y1) / 2, text=txt, fill='white', font=('Segoe UI', 8, 'bold'), justify='center')
            if i < n - 1: c.create_text(x0 + bw + gap / 2, (y0 + y1) / 2, text='\u2192', fill=MUTED, font=('Segoe UI', 14))
        c.create_text(W / 2, 80, text=s.jd_final or 'final output appears here after the run', fill=RED if 'SHUT' in s.jd_final else TXT, font=F_B)

    def draw_chart(s, d):
        a1, a2 = s.hax.ravel(); a1.clear(); a2.clear(); cf = s.cfg
        for ax in (a1, a2): ax.set_facecolor('white'); ax.grid(True, color='#EEF0F3', lw=0.8); ax.tick_params(colors=MUTED, labelsize=8); [sp.set_color('#D1D5DB') for sp in ax.spines.values()]
        a1.set_xlim(0, cf['deadline']); a1.set_ylim(55, 140); a2.set_ylim(0, 3.5); a2.set_yticks([]); a2.set_xlabel('virtual minutes', fontsize=8, color=MUTED); a1.set_ylabel('reactor temperature (C)', fontsize=8, color=MUTED)
        a1.axhline(cf['T_crit'], color=RED, ls='--', lw=1); a1.text(1, cf['T_crit'] + 1.5, 'critical %.0f C (ASSUMPTION)' % cf['T_crit'], color=RED, fontsize=8)
        a1.axhline(cf['T_fixed_alarm'], color='#9CA3AF', ls=':', lw=1); a1.text(1, cf['T_fixed_alarm'] + 1.5, 'fixed alarm %.0f C' % cf['T_fixed_alarm'], color=MUTED, fontsize=8)
        if d:
            a1.plot(d['t'], d['Tm'], '.', color='#CBD5E1', ms=2, label='sensor reading'); a1.plot(d['th'], d['That'], color=BLUE, lw=2, label='CHRONOS estimate')
            a1.plot(d['t'], d['Tt'], color=GREEN, lw=1.6, label='real temperature (CHRONOS acts)'); a1.plot(d['t'], d['Tt0'], color=RED, lw=1.4, ls='--', label='real temperature if NO action')
            a1.legend(loc='upper left', fontsize=8, frameon=False); mk = d['marks']
            it = [(20.0, 'sensor spike', AMBER), (35.0, 'real failure + attacks', RED)] + [(x, 'fixed alarm FALSE %.1f' % x, MUTED) for x in mk['fixed_false']]
            for key, lab, col in (('alert', 'CHRONOS alert %.1f', BLUE), ('shutdown', 'CHRONOS shutdown %.1f', GREEN), ('fixed_real', 'fixed alarm %.1f', MUTED), ('crit0', 'critical if no action %.1f', RED)):
                if mk.get(key) is not None: it.append((mk[key], lab % mk[key], col))
            it.sort(key=lambda z: z[0])
            for j, (x, lab, col) in enumerate(it):
                y = 0.3 + (j % 4) * 0.75; a1.axvline(x, color=col, lw=0.8, alpha=0.5); a2.plot([x, x], [0, y], color=col, lw=1.5)
                a2.text(x + (-0.5 if x > 62 else 0.5), y, lab, fontsize=7, color=col, va='bottom', ha='right' if x > 62 else 'left')
        s.hcv.draw_idle()

    def judge_start(s):
        if not s.models or s.jd_busy: return
        s.jd_busy = True; s.jd_btn.config(state='disabled'); s.jd_state = [None] * 8; s.jd_final = ''; s.jd_result = {}; s.draw_flow()
        for i, cv in enumerate(s.jd_cv): s.donut(cv, 0, 0, s.jd_grey); s.jd_pl[i].config(text='-', fg=MUTED)
        for l in s.jd_num.values(): l.config(text='...')
        s.draw_chart(None); s.jd_why.config(text='running ...')
        tg = bool((s.tg_tok.get().strip() or os.environ.get('TELEGRAM_BOT_TOKEN', '')) and (s.tg_chat.get().strip() or os.environ.get('TELEGRAM_CHAT_ID', '')))
        man = {'T': float(s.sv_T.get())}
        if s.sv_on.get(): man['flow'] = float(s.sv_F.get())
        n_tabs = len([x for x in s.nb.tabs() if s.nb.tab(x, 'state') == 'normal'])
        models, cfg, q = s.models, dict(s.cfg), s.q; post = lambda *a: q.put(('judge', a))
        def work():
            try: judge_worker(models, cfg, post, n_tabs, tg, man)
            except Exception as ex: post('err', '%s: %s' % (type(ex).__name__, str(ex)[:100]))
            post('end')
        threading.Thread(target=work, daemon=True).start()

    def judge_update(s, p):
        k = p[0]
        if k == 'row': i = p[1]; s.jd_state[i] = p[2]; s.donut(s.jd_cv[i], p[4], p[5], GREEN if p[2] == 'green' else AMBER); s.jd_pl[i].config(text=p[3], fg=TXT); s.draw_flow()
        elif k == 'stage': s.jd_stage.config(text=p[1])
        elif k == 'comp': s.jd_comp.config(text='Predictor agent COMPROMISED (lies with 0.99 confidence)' if p[1] else '')
        elif k == 'chart': s.draw_chart(p[1])
        elif k == 'num': s.jd_num[p[1]].config(text=p[2])
        elif k == 'why': s.jd_why.config(text=p[1])
        elif k == 'final': s.jd_final = 'Output on the screen: "%s"%s' % (p[1]['lcd'], '   (shutdown at t=%.1f min)' % p[1]['shutdown_t'] if p[1]['shutdown_t'] is not None else ''); s.draw_flow()
        elif k == 'summary': s.jd_result = p[1]
        elif k == 'tests': s.jd_result['tests'] = list(p[4]) if len(p) > 4 else []; s.jd_num['tests'].config(text='%d / %d%s' % (p[1], p[2], '' if p[1] == p[2] else '   (failed: ' + ', '.join(p[3]) + ')')); s.jd_result['tests_passed'] = '%d/%d' % (p[1], p[2])
        elif k == 'err': s.jd_stage.config(text='ERROR: ' + p[1])
        elif k == 'end':
            s.jd_busy = False; s.jd_btn.config(state='normal')
            if not s.jd_stage.cget('text').startswith('ERROR'): s.jd_stage.config(text='done - change the sensor offset and press Run again')
            if s.tg_auto.get() and s.jd_result.get('rows'): s.tg_send_report()

    # ================= TELEGRAM (tab): the WHOLE report goes to Telegram (split in 3800-character messages)
    def build_telegram(s):
        t = s.tab['Telegram']; top = tk.Frame(t, bg=BG); top.pack(fill='x', padx=14, pady=10)
        ttk.Button(top, text='Send FULL report to Telegram', style='Big.TButton', command=s.tg_send_report).pack(side='left')
        ttk.Button(top, text='Preview report', command=s.tg_preview).pack(side='left', padx=8)
        s.tg_auto = tk.BooleanVar(value=False); ttk.Checkbutton(top, text='send automatically when the demo finishes', variable=s.tg_auto).pack(side='left', padx=8)
        s.tg_lbl = tk.Label(top, text='Bot token + chat id are typed in Settings. Run the Home demo (and Metrics suite) first for a complete report.', bg=BG, fg=MUTED, font=F_N); s.tg_lbl.pack(side='left', padx=14)
        s.tg_txt = s.text(t, 20); s.tg_txt.configure(font=('Consolas', 10)); s.tg_txt.pack(fill='both', expand=True, padx=14, pady=(0, 12))

    def tg_report_text(s):
        try: rep = s.rep_txt.get('1.0', 'end').strip()
        except Exception: rep = ''
        if rep.startswith('Run the demo'): rep = ''
        return build_full_report(dict(s.jd_result), rep, s.suite_rows, list(s.alerts), s.cfg['seed'])

    def tg_preview(s): s.put(s.tg_txt, s.tg_report_text()); s.tg_lbl.config(text='preview only - not sent yet', fg=MUTED)

    def tg_send_report(s):
        tok = s.tg_tok.get().strip() or os.environ.get('TELEGRAM_BOT_TOKEN', ''); chat = s.tg_chat.get().strip() or os.environ.get('TELEGRAM_CHAT_ID', '')
        txt = s.tg_report_text(); s.put(s.tg_txt, txt)
        if not (tok and chat and requests): s.tg_lbl.config(text='Telegram not set: type bot token + chat id in Settings (and pip install requests).', fg=RED); return
        s.tg_lbl.config(text='sending ...', fg=MUTED); q = s.q
        def work():
            try: n = telegram_send_long(tok, chat, txt); q.put(('tgstat', ('ok', 'report sent to Telegram (%d message%s)' % (n, '' if n == 1 else 's'))))
            except Exception as ex: q.put(('tgstat', ('err', 'Telegram failed: ' + _scrub(ex, tok, chat)[:120])))
        threading.Thread(target=work, daemon=True).start()

    # ================= REPORT (tab 2): AI explains the Home result in simple English
    def build_report(s):
        t = s.tab['Report']; top = tk.Frame(t, bg=BG); top.pack(fill='x', padx=14, pady=10)
        ttk.Button(top, text='Generate report', style='Big.TButton', command=s.report_start).pack(side='left')
        s.rep_lbl = tk.Label(top, text='Run the demo on Home first. A Groq key (Settings) is optional: without it a plain template report is shown.', bg=BG, fg=MUTED, font=F_N); s.rep_lbl.pack(side='left', padx=14)
        s.rep_txt = s.text(t, 20, False); s.rep_txt.configure(font=('Segoe UI', 12)); s.rep_txt.pack(fill='both', expand=True, padx=14, pady=(0, 12))

    def report_start(s):
        res = dict(s.jd_result)
        if not res.get('rows'): s.put(s.rep_txt, 'Run the demo on the Home tab first (wait until it says "done").'); return
        key = s.groq_key.get().strip() or os.environ.get('GROQ_API_KEY', ''); model = s.groq_model.get(); q = s.q; s.rep_lbl.config(text='writing report ...')
        def work():
            try: txt = groq_chat(key, model, REPORT_Q, res) if (key and requests) else template_report(res)
            except Exception as ex: txt = '(Groq failed: %s - plain template report below)\n\n' % _scrub(ex, key)[:80] + template_report(res)
            q.put(('report', txt))
        threading.Thread(target=work, daemon=True).start()

    # ================= LIVE MONITOR
    def build_live(s):
        t = s.tab['Live Monitor']; left = s.card(t); left.pack(side='left', fill='both', expand=True, padx=(8, 4), pady=8)
        s.live_fig, s.live_axs, cv = s.fig(left, 3, 1, sharex=True); s.live_cv = cv; cv.get_tk_widget().pack(fill='both', expand=True, padx=4, pady=4)
        right = tk.Frame(t, bg=BG, width=420); right.pack(side='right', fill='y', padx=(4, 8), pady=8); right.pack_propagate(False)
        c1 = s.card(right, 'STATUS'); c1.pack(fill='x', pady=(0, 6))
        s.big = tk.Label(c1, text='-', bg=BG, fg=GREEN, font=F_BIG); s.big.pack(anchor='w', padx=12)
        s.kpi = tk.Label(c1, text='', bg=BG, fg=TXT, font=F_N, justify='left'); s.kpi.pack(anchor='w', padx=12, pady=(0, 8))
        c2 = s.card(right, 'VIRTUAL DEVICE  (mirrors the future ESP32-S3 screen)'); c2.pack(fill='x', pady=6)
        s.dev = tk.Canvas(c2, width=396, height=130, bg=BG, highlightthickness=0); s.dev.pack(padx=6, pady=4)
        c3 = s.card(right, 'TRUST PER SOURCE  (maths: reliability x signature x timeliness x consistency x plausibility)'); c3.pack(fill='x', pady=6)
        s.trust_cv = tk.Canvas(c3, width=396, height=150, bg=BG, highlightthickness=0); s.trust_cv.pack(padx=6, pady=4)
        c4 = s.card(right, 'ASK CHRONOS  (Groq explains - zero decision weight)'); c4.pack(fill='both', expand=True, pady=(6, 0))
        s.chat = s.text(c4, 8, False); s.chat.pack(fill='both', expand=True, padx=8, pady=2); s.chat.configure(state='disabled')
        r = tk.Frame(c4, bg=BG); r.pack(fill='x', padx=8, pady=6); s.chat_in = ttk.Entry(r); s.chat_in.pack(side='left', fill='x', expand=True)
        s.chat_in.bind('<Return>', lambda e: s.chat_send()); ttk.Button(r, text='Send', style='Accent.TButton', command=s.chat_send).pack(side='left', padx=(6, 0))

    def live_init(s):
        a1, a2, a3 = s.live_axs.ravel(); T = s.cfg
        for ax in (a1, a2, a3): ax.set_facecolor('white'); ax.grid(True, color='#EEF0F3', lw=0.8); ax.tick_params(colors=MUTED, labelsize=8); [sp.set_color('#D1D5DB') for sp in ax.spines.values()]
        a1.set_title('Reactor temperature  (raw sensor vs Kalman estimate)', loc='left', fontsize=10, color=TXT); a1.set_ylim(55, 135); a1.set_xlim(0, T['deadline'])
        a1.axhline(T['T_crit'], color=RED, ls='--', lw=1); a1.axhline(T['T_fixed_alarm'], color='#9CA3AF', ls=':', lw=1); a1.text(1, T['T_crit'] + 1.5, 'critical 100 C (ASSUMPTION)', color=RED, fontsize=8)
        a1.text(1, T['T_fixed_alarm'] + 1.5, 'fixed-threshold alarm 90 C', color='#6B7280', fontsize=8)
        s.l_Tm, = a1.plot([], [], '.', color='#9CA3AF', ms=3, label='raw sensor'); s.l_Th, = a1.plot([], [], color=BLUE, lw=2, label='Kalman estimate'); a1.legend(loc='upper left', fontsize=8, frameon=False)
        a2.set_title('Cooling flow  (flow meter vs pump vibration / 2)', loc='left', fontsize=10, color=TXT); a2.set_ylim(0, 1.4)
        s.l_f, = a2.plot([], [], color=BLUE, lw=1.6, label='flow meter'); s.l_v, = a2.plot([], [], color='#9CA3AF', lw=1.2, label='vibration/2'); s.l_fe, = a2.plot([], [], color=GREEN, lw=1, ls='--', label='flow used'); a2.legend(loc='lower left', fontsize=8, frameon=False)
        a3.set_title('P(critical within %d min)  and  time-to-critical' % T['horizon'], loc='left', fontsize=10, color=TXT); a3.set_ylim(0, 1.05); a3.set_xlabel('virtual minutes', fontsize=8, color=MUTED)
        a3.axhline(T['p_alert'], color=AMBER, ls=':', lw=1); a3.axhline(T['p_act'], color=RED, ls=':', lw=1); s.l_p, = a3.plot([], [], color=BLUE, lw=2); s.ttc_txt = a3.text(0.99, 0.9, '', transform=a3.transAxes, ha='right', fontsize=10, color=TXT)
        s.inj_lines = []

    def upd_live(s):
        H = s.engine.hist
        if not H: return
        t = [h['t'] for h in H]; nan = float('nan'); f = lambda k: [nan if h[k] is None else h[k] for h in H]
        s.l_Tm.set_data(t, f('Tm')); s.l_Th.set_data(t, f('That')); s.l_f.set_data(t, f('fm')); s.l_v.set_data(t, [nan if h['vib'] is None else h['vib'] / 2 for h in H])
        s.l_fe.set_data(t, f('f_est')); s.l_p.set_data(t, [h['pcrit'] for h in H]); e = s.engine
        s.ttc_txt.set_text('time-to-critical: ' + ('%.0f min' % e.ttc if e.ttc is not None else 'not in window') + '   P=%.2f' % e.pcrit)
        for (tt, kind) in s.world.injlog[s.drawn_inj:]:
            for ax in s.live_axs.ravel(): ax.axvline(tt, color=AMBER, lw=1, alpha=.8)
            s.live_axs[0, 0].text(tt + .3, 57, kind, fontsize=7, color=AMBER, rotation=90, va='bottom')
        s.drawn_inj = len(s.world.injlog); s.live_cv.draw_idle()

    def upd_side(s):
        e = s.engine; col = MODE_COL.get(e.mode, GREEN); s.big.config(text=e.mode.replace('_', ' '), fg=col); s.pill.config(text=e.mode.replace('_', ' '), bg=col)
        top = e.causes[0] if e.causes else None
        s.kpi.config(text='Kalman estimate: %.1f C     P(critical): %.0f%%\nTime-to-critical: %s\nTop cause: %s\nIndependent cooling sources abnormal: %d     quarantined (5 min): %d'
                     % (e.kf.x or 0, 100 * e.pcrit, ('%.0f min' % e.ttc) if e.ttc is not None else 'not in window', (top['cause'] + ' (%.2f)' % top['p']) if top else '-', e.n_src, len(e.q_times)))
        s.clock.config(text='t = %.1f / %d min' % (s.world.t, s.cfg['deadline']))
        d = s.dev; d.delete('all'); d.create_oval(14, 20, 94, 100, fill=col, outline=''); lines = {'NORMAL': ('SYSTEM NORMAL', 'T=%.1f C  P(crit)=%.2f' % (e.kf.x or 0, e.pcrit)),
              'SENSOR_FAULT': ('SENSOR FAULT', 'running on estimate %.1f C' % (e.kf.x or 0)), 'ALERT': ('COOLING ALERT', 'P(crit)=%.2f - call technician' % e.pcrit),
              'ABSTAIN': ('NOT SURE', 'need more evidence'), 'ATTACK_CONTAINED': ('FAKE MESSAGES BLOCKED', 'plant unaffected'), 'SHUTDOWN': ('FACTORY SHUT DOWN', 'DUE TO ERROR')}[e.mode]
        if e.mode == 'SHUTDOWN': d.create_text(112, 30, text='FACTORY SHUT DOWN\nDUE TO ERROR', anchor='nw', fill=RED, font=('Segoe UI', 15, 'bold')); d.create_text(112, 90, text='do NOT restart - call technician', anchor='nw', fill=MUTED, font=F_N)
        else: d.create_text(112, 30, text=lines[0], anchor='nw', fill=TXT, font=('Segoe UI', 15, 'bold')); d.create_text(112, 70, text=lines[1], anchor='nw', fill=MUTED, font=F_N)
        c = s.trust_cv; c.delete('all'); y = 6
        for src, v in e.trust.items():
            c.create_text(0, y + 9, text=src, anchor='w', fill=TXT, font=('Segoe UI', 8)); c.create_rectangle(150, y + 3, 330, y + 15, fill=SOFT, outline='')
            tc = GREEN if v['trust'] >= .7 else (AMBER if v['trust'] >= .4 else RED); c.create_rectangle(150, y + 3, 150 + 180 * max(0.01, min(1, v['trust'])), y + 15, fill=tc, outline='')
            c.create_text(336, y + 9, text='%.2f' % v['trust'], anchor='w', fill=TXT, font=('Segoe UI', 8, 'bold')); c.create_text(372, y + 9, text='(%.2f)' % v['org'], anchor='w', fill=MUTED, font=('Segoe UI', 7)); y += 24
        c.create_text(0, y + 2, text='bar = our maths trust, ( ) = organizer trust_score kept alongside', anchor='w', fill=MUTED, font=('Segoe UI', 7))

    def put_chat(s, t): s.chat.configure(state='normal'); s.chat.insert('end', t); s.chat.see('end'); s.chat.configure(state='disabled')

    def chat_send(s):
        q = s.chat_in.get().strip()
        if not q or not s.engine: return
        s.chat_in.delete(0, 'end'); s.put_chat('You: ' + q + '\n'); st = s.engine.state_json(); key = s.groq_key.get().strip() or os.environ.get('GROQ_API_KEY', ''); model = s.groq_model.get()
        if key and requests:
            def work():
                try: s.q.put(('chat', groq_chat(key, model, q, st)))
                except Exception as ex: s.q.put(('chat', '(Groq failed: %s) template answer -> %s' % (_scrub(ex, key)[:80], local_answer(q, st))))
            threading.Thread(target=work, daemon=True).start()
        else: s.put_chat('CHRONOS: ' + local_answer(q, st) + '\n')

    # ================= FAULT INJECTION
    def build_faults(s):
        t = s.tab['Fault Injection']; grid = tk.Frame(t, bg=BG); grid.pack(side='left', fill='both', expand=True, padx=8, pady=8)
        for i, (k, ti, ds) in enumerate(FAULTS):
            c = s.card(grid); c.grid(row=i // 3, column=i % 3, sticky='nsew', padx=6, pady=6); grid.columnconfigure(i % 3, weight=1); grid.rowconfigure(i // 3, weight=1)
            tk.Label(c, text=ti, bg=BG, fg=RED if k in ('pump', 'paradox') else TXT, font=F_H).pack(anchor='w', padx=12, pady=(12, 2))
            tk.Label(c, text=ds, bg=BG, fg=MUTED, wraplength=300, justify='left').pack(anchor='w', padx=12)
            ttk.Button(c, text='Inject now', style='Danger.TButton' if k in ('pump', 'paradox') else 'Accent.TButton', command=lambda k=k: s.inject(k)).pack(anchor='w', padx=12, pady=12)
        side = tk.Frame(t, bg=BG, width=360); side.pack(side='right', fill='y', padx=8, pady=8); side.pack_propagate(False)
        c = s.card(side, 'ABLATION SWITCHES  (live effect on the decision)'); c.pack(fill='x')
        s.v_cross = tk.BooleanVar(value=True); s.v_adv = tk.BooleanVar(value=True); s.v_cau = tk.BooleanVar(value=True); s.v_ab = tk.BooleanVar(value=False)
        for v, txt, key in ((s.v_cross, 'Cross-check ON  (flow meter vs vibration vs heat balance)', 'cross'), (s.v_adv, 'Adversarial filter ON  (signature, replay, trust maths)', 'adv'),
                            (s.v_cau, 'Causal model ON  (rejects spurious correlation)', 'causal'), (s.v_ab, 'Auto-approve backup cooling', 'auto_backup')):
            ttk.Checkbutton(c, text=txt, variable=v, command=lambda v=v, key=key: s.set_sw(key, v.get())).pack(anchor='w', padx=12, pady=6)
        tk.Label(c, text='Try: inject "Cooling sensor corrupted" with cross-check OFF -> FALSE ALERT (the safety rule still blocks the shutdown).\nTry PARADOX with adversarial filter OFF -> forged "all OK" message vetoes the shutdown.', bg=BG, fg=MUTED, wraplength=320, justify='left').pack(padx=12, pady=(2, 12))

    def set_sw(s, k, v):
        if s.engine: s.engine.sw[k] = v

    # ================= HYPOTHESES
    def build_hyp(s):
        t = s.tab['Hypotheses']; l = s.card(t, 'POSTERIOR (Bayesian fusion)'); l.pack(side='left', fill='both', expand=True, padx=(8, 4), pady=8)
        s.hyp_fig, ax, cv = s.fig(l); s.hyp_ax = ax[0, 0]; s.hyp_cv = cv; cv.get_tk_widget().pack(fill='both', expand=True, padx=4, pady=4)
        r = tk.Frame(t, bg=BG); r.pack(side='right', fill='both', expand=True, padx=(4, 8), pady=8)
        c = s.card(r, 'RANKED CAUSES with evidence links (M2)'); c.pack(fill='both', expand=True)
        s.cause_tree = s.tree(c, ('rank', 'cause', 'prob', 'label', 'evidence ids'), (40, 260, 55, 190, 220), 7); s.cause_tree.frame.pack(fill='both', expand=True, padx=8, pady=6)
        c2 = s.card(r, 'EVIDENCE / SPURIOUS CORRELATION / PCA CONTRIBUTION'); c2.pack(fill='both', expand=True, pady=(8, 0)); s.hyp_txt = s.text(c2, 12); s.hyp_txt.pack(fill='both', expand=True, padx=8, pady=6)

    def upd_hyp(s):
        e = s.engine; ax = s.hyp_ax; ax.clear(); ax.set_facecolor('white'); cols = [GREEN, AMBER, RED, '#7C3AED']
        ax.barh(HYP[::-1], [float(p) for p in e.post][::-1], color=cols[::-1]); ax.set_xlim(0, 1); ax.set_title('P(hypothesis | evidence)', loc='left', fontsize=10)
        for i, p in enumerate(e.post[::-1]): ax.text(min(float(p) + .01, .9), i, '%.2f' % p, va='center', fontsize=9)
        s.hyp_cv.draw_idle(); tr = s.cause_tree; tr.delete(*tr.get_children())
        for q in e.causes: tr.insert('', 'end', values=(q['rank'], q['cause'], '%.2f' % q['p'], q['label'], ', '.join(q['evidence'][:3])))
        L = ['Active evidence (features that fired this tick):'] + ['  + ' + FEAT_TEXT[k] for k, v in e.feats.items() if v] if e.feats else []
        L += ['', 'Kalman: NIS=%.1f  consistency c=%.1f sd  dropped=%s' % (e.info.get('nis', 0), e.info.get('c', 0), e.info.get('dropped')),
              'PCA: T2=%.1f (limit %.1f)  SPE=%.1f (limit %.1f)  suspect sensor: %s' % (e.pca['t2'], e.m['t2_lim'], e.pca['spe'], e.m['spe_lim'], e.pca['suspect'])]
        if e.contrib is not None: L.append('SPE contribution plot: ' + ', '.join('%s=%.1f' % (n.split('_')[0], v) for n, v in zip(e.contrib['names'], e.contrib['SPE'])))
        sp = e.spur
        if sp: L += ['', 'Correlation check  vibration <-> temperature: r=%.2f, partial r given COOLING_FLOW = %.2f' % (sp['r'], sp['partial']),
                     ('  -> SPURIOUS: hidden confounder is cooling flow.  ' + ('REJECTED by causal model.' if e.sw['causal'] else 'ACCEPTED (causal model OFF)!')) if sp['spurious'] else '  -> no spurious link']
        s.put(s.hyp_txt, '\n'.join(L))

    # ================= SIMULATION
    def build_sim(s):
        t = s.tab['Simulation']; top = tk.Frame(t, bg=BG); top.pack(fill='x', padx=8, pady=(8, 0))
        ttk.Button(top, text='Run 500 rollouts per action now', style='Accent.TButton', command=s.run_dec_now).pack(side='left')
        tk.Label(top, text='  common random numbers, J = E[L] + lambda*CVaR95(L) + mu*Cost,  lambda=%.1f mu=%.1f' % (s.cfg['lam'], s.cfg['mu']), bg=BG, fg=MUTED).pack(side='left')
        c = s.card(t, 'MAX-TEMPERATURE DISTRIBUTION PER ACTION'); c.pack(fill='both', expand=True, padx=8, pady=8)
        s.sim_fig, s.sim_axs, s.sim_cv = s.fig(c, 1, 5, sharey=True); s.sim_cv.get_tk_widget().pack(fill='both', expand=True, padx=4, pady=4)
        c2 = s.card(t, 'LOSS TABLE  (mean, tail = CVaR95, deadline-violation rate, 95% interval, J, feasibility)'); c2.pack(fill='x', padx=8, pady=(0, 8))
        s.sim_tree = s.tree(c2, ('action', 'mean loss', 'tail loss', 'violation', '95% CI', 'cost', 'J', 'feasible / reason', 'approval'), (200, 80, 80, 80, 130, 50, 70, 330, 80), 7); s.sim_tree.frame.pack(fill='x', padx=8, pady=6)
        s.base_lbl = tk.Label(t, text='', bg=BG, fg=TXT, font=F_B); s.base_lbl.pack(anchor='w', padx=12, pady=(0, 8))

    def run_dec_now(s):
        if s.engine and s.world: s.engine.run_decision(s.world.t); s.upd_sim(); s.upd_dec(); s.upd_agents()

    def upd_sim(s):
        e = s.engine
        if not e.sim: return
        for ax, a in zip(s.sim_axs.ravel(), ACTIONS):
            ax.clear(); ax.set_facecolor('white'); ax.hist(e.sim[a]['Tmax'], bins=24, color=BLUE if a != e.opt['best'] else GREEN, alpha=.85); ax.axvline(s.cfg['T_crit'], color=RED, ls='--', lw=1)
            ax.set_title(a.replace('_', '\n'), fontsize=8); ax.tick_params(labelsize=7, colors=MUTED)
        s.sim_cv.draw_idle(); tr = s.sim_tree; tr.delete(*tr.get_children())
        for r in e.opt['table']: tr.insert('', 'end', values=(r['action'] + (' *' if r['action'] == e.opt['best'] else ''), '%.1f' % r['mean'], '%.1f' % r['cvar'], '%.1f%%' % (100 * r['viol']), '[%.1f, %.1f]' % (r['lo'], r['hi']), r['cost'], '%.1f' % r['J'], 'yes' if r['feasible'] else 'REJECTED: ' + r['reason'], 'required' if r['approval'] else 'auto'))
        b = e.opt['baselines']; s.base_lbl.config(text='Optimized J=%.1f   vs   no-action J=%.1f   vs   greedy(5-min myopic: %s) J=%.1f      * = recommended' % (b['optimized'], b['no_action'], e.opt['greedy'], b['greedy']))

    # ================= DECISION
    def build_dec(s):
        t = s.tab['Decision']; top = s.card(t); top.pack(fill='x', padx=8, pady=8)
        s.dec_big = tk.Label(top, text='-', bg=BG, fg=TXT, font=F_BIG); s.dec_big.pack(anchor='w', padx=12, pady=(10, 0)); s.dec_sub = tk.Label(top, text='', bg=BG, fg=MUTED, justify='left'); s.dec_sub.pack(anchor='w', padx=12)
        r = tk.Frame(top, bg=BG); r.pack(anchor='w', padx=12, pady=10)
        ttk.Button(r, text='Decide now (rollouts + certificate)', style='Accent.TButton', command=s.run_dec_now).pack(side='left'); ttk.Button(r, text='Approve & execute recommended action', command=s.approve).pack(side='left', padx=8)
        c = s.card(t, 'SAFETY CERTIFICATE (M7)  -  rule = deterministic, empirical = Monte-Carlo (not proof)'); c.pack(fill='both', expand=True, padx=8, pady=(0, 8)); s.cert_txt = s.text(c, 20); s.cert_txt.pack(fill='both', expand=True, padx=8, pady=8)

    def approve(s):
        e = s.engine
        if not e.opt: s.run_dec_now()
        a = e.opt['best']; ok, why = e.feasible(a, s.world.t)
        if not ok: messagebox.showinfo('Rejected', why); return
        s.world.apply_action(a); e.external_action(a, s.world.t); s.status('Executed (approved): ' + a)

    def upd_dec(s):
        e = s.engine; o = e.opt
        if o:
            ab = e.cert['abstain'] if e.cert else False
            s.dec_big.config(text=('ABSTAIN - request evidence' if ab else o['best'].replace('_', ' ')), fg=AMBER if ab else (RED if o['best'] == 'SHUTDOWN_REACTOR' else BLUE))
            r = [x for x in o['table'] if x['action'] == o['best']][0]
            s.dec_sub.config(text='Mode: %s   |   approval flag: %s   |   certificate: %s   |   executed so far: %s' % (e.mode, 'REQUIRED' if r['approval'] else 'not required (auto)', e.cert['overall'], 'SHUTDOWN' if e.shutdown else ('backup cooling' if e.backup_used else 'none')))
            s.put(s.cert_txt, json.dumps(e.cert, indent=2, default=str))
        else: s.dec_big.config(text=e.mode.replace('_', ' ')); s.dec_sub.config(text='No decision computed yet - runs automatically when an alert starts, or press "Decide now".')

    # ================= AGENTS
    def build_agents(s):
        t = s.tab['Agents']; top = tk.Frame(t, bg=BG); top.pack(fill='x', padx=8, pady=8)
        tk.Label(top, text='Compromise one agent (it will lie with 0.99 confidence):', bg=BG, fg=MUTED).pack(side='left'); s.cb_comp = ttk.Combobox(top, values=['none', 'Investigator', 'Predictor', 'Optimizer', 'Safety', 'Red-team'], state='readonly', width=16)
        s.cb_comp.set('none'); s.cb_comp.pack(side='left', padx=8); s.cb_comp.bind('<<ComboboxSelected>>', lambda e: (s.set_sw('compromise', None if s.cb_comp.get() == 'none' else s.cb_comp.get()), s.run_dec_now()))
        tk.Label(top, text='LLM text weight in any decision = 0', bg=BG, fg=RED, font=F_B).pack(side='right')
        c = s.card(t, 'AGENT DISAGREEMENT TABLE  (JSON claims {agent, claim, evidence_ids, confidence}; validated against ledger evidence)'); c.pack(fill='both', expand=True, padx=8, pady=(0, 8))
        s.ag_tree = s.tree(c, ('agent', 'claim', 'confidence', 'evidence_ids', 'status'), (110, 420, 80, 280, 260), 8); s.ag_tree.frame.pack(fill='both', expand=True, padx=8, pady=6)
        s.ag_txt = s.text(c, 8); s.ag_txt.pack(fill='x', padx=8, pady=(0, 8))
        s.ag_tree.tag_configure('bad', foreground=RED)

    def upd_agents(s):
        e = s.engine; tr = s.ag_tree; tr.delete(*tr.get_children())
        for a in e.agents: tr.insert('', 'end', values=(a['agent'], a['claim'], a['confidence'], ', '.join(a['evidence_ids']), a['status']), tags=('' if a['validated'] else 'bad',))
        s.put(s.ag_txt, json.dumps([{k: v for k, v in a.items() if k != '_v'} for a in e.agents[:2]], indent=1, default=str) if e.agents else 'Agents run when a decision is computed.')

    # ================= TIMELINE
    def build_time(s):
        t = s.tab['Timeline']; top = s.card(t, 'TIME TRAVEL  (drag to inspect any past tick - state, posterior, decision)'); top.pack(fill='x', padx=8, pady=8)
        s.sl = tk.Scale(top, from_=0, to=1, orient='horizontal', bg=BG, highlightthickness=0, troughcolor=SOFT, command=lambda v: s.upd_time(), length=900); s.sl.pack(fill='x', padx=12, pady=8)
        s.follow = tk.BooleanVar(value=True); ttk.Checkbutton(top, text='follow live', variable=s.follow).pack(anchor='w', padx=12, pady=(0, 8))
        r = tk.Frame(t, bg=BG); r.pack(fill='both', expand=True, padx=8, pady=(0, 8))
        c1 = s.card(r, 'SNAPSHOT'); c1.pack(side='left', fill='both', expand=True, padx=(0, 4)); s.snap = s.text(c1, 14); s.snap.pack(fill='both', expand=True, padx=8, pady=8)
        c2 = s.card(r, 'TWO ALTERNATIVE HISTORIES (shown when evidence is weak)'); c2.pack(side='left', fill='both', expand=True, padx=(4, 0)); s.alt = s.text(c2, 14, False); s.alt.pack(fill='both', expand=True, padx=8, pady=8)

    def upd_time(s):
        e = s.engine; n = len(e.hist)
        if not n: return
        s.sl.configure(to=max(1, n - 1))
        if s.follow.get(): s.sl.set(n - 1)
        h = e.hist[min(int(s.sl.get()), n - 1)]
        s.put(s.snap, 'tick %d   t = %.2f min\nmode: %s\nraw T: %s   outlet: %s   Kalman estimate: %.2f C\nflow meter: %s   flow used: %.2f   vibration: %s\nP(critical): %.3f   time-to-critical: %s\nNIS=%.1f  consistency=%.1f  dropped sensor: %s\nindependent abnormal sources: %d\nposterior: %s'
                      % (h['k'], h['t'], h['mode'], '%.1f' % h['Tm'] if h['Tm'] is not None else '-', '%.1f' % h['Tout'] if h['Tout'] is not None else '-', h['That'], '%.2f' % h['fm'] if h['fm'] is not None else '-', h['f_est'],
                         '%.2f' % h['vib'] if h['vib'] is not None else '-', h['pcrit'], h['ttc'], h['nis'], h['c'], h['dropped'], h['n_src'], ', '.join('%s=%.2f' % (a, b) for a, b in zip(HYP, h['post']))))
        s.put(s.alt, '\n\n'.join('History %d: %s  (p=%.2f)%s\n%s' % (i + 1, a['history'], a['prob'], '  [weak evidence]' if a['weak_evidence'] else '', a['story']) for i, a in enumerate(e.alt_histories())))

    # ================= AUDIT
    def build_audit(s):
        t = s.tab['Audit']; top = tk.Frame(t, bg=BG); top.pack(fill='x', padx=8, pady=8)
        ttk.Button(top, text='Verify hash chain', style='Accent.TButton', command=s.verify_chain).pack(side='left'); ttk.Button(top, text='Deterministic replay', command=s.do_replay).pack(side='left', padx=8)
        s.aud_lbl = tk.Label(top, text='', bg=BG, fg=TXT, font=F_B); s.aud_lbl.pack(side='left', padx=12)
        r = tk.Frame(t, bg=BG); r.pack(fill='both', expand=True, padx=8, pady=(0, 8))
        c1 = s.card(r, 'HASH-CHAINED AUDIT LOG'); c1.pack(side='left', fill='both', expand=True, padx=(0, 4)); s.aud_tree = s.tree(c1, ('#', 't', 'kind', 'hash', 'payload'), (40, 55, 150, 130, 400), 18); s.aud_tree.frame.pack(fill='both', expand=True, padx=8, pady=6)
        c2 = s.card(r, 'EVENT LEDGER (M1): ACCEPTED / DUPLICATE / QUARANTINED / CONFLICT'); c2.pack(side='left', fill='both', expand=True, padx=(4, 0))
        s.led_tree = s.tree(c2, ('event_time', 'arrival', 'source', 'value', 'status', 'reason', 'org.trust'), (70, 70, 150, 130, 100, 190, 60), 18); s.led_tree.frame.pack(fill='both', expand=True, padx=8, pady=6)
        for k, col in (('QUARANTINED', RED), ('CONFLICT', AMBER), ('DUPLICATE', MUTED)): s.led_tree.tag_configure(k, foreground=col)

    def verify_chain(s):
        ok, bad = s.engine.audit.verify(); s.aud_lbl.config(text='chain VALID (%d entries)' % len(s.engine.audit.rows) if ok else 'chain BROKEN at entry %s' % bad, fg=GREEN if ok else RED)

    def do_replay(s):
        e2 = replay_stream(s.models, s.cfg, s.stream, s.engine.sw); a, b = s.engine.replay_hash(), e2.replay_hash()
        s.aud_lbl.config(text='live hash %s   replay hash %s   -> %s' % (a, b, 'IDENTICAL' if a == b else 'DIFFERENT (settings changed mid-run?)'), fg=GREEN if a == b else AMBER)

    def upd_audit(s):
        e = s.engine; tr = s.aud_tree; tr.delete(*tr.get_children())
        for r in e.audit.rows[-300:]: tr.insert('', 'end', values=(r['idx'], r['t'], r['kind'], r['hash'], json.dumps(r['payload'], default=str)[:120]))
        tr.yview_moveto(1); tl = s.led_tree; tl.delete(*tl.get_children())
        for r in e.ledger.rows[-300:]:
            if r['status'] != 'ACCEPTED' or r['source'] in ('MAINTENANCE_AGENT', 'REACTOR_SENSOR_14'): tl.insert('', 'end', values=(r['event_time'], r['arrival_time'], r['source'], str(r['value'])[:26], r['status'] + (' ' + ','.join(r['flags']) if r['flags'] else ''), r['reason'], r['trust_score']), tags=(r['status'],))
        tl.yview_moveto(1)

    # ================= METRICS
    def build_metrics(s):
        t = s.tab['Metrics']; top = tk.Frame(t, bg=BG); top.pack(fill='x', padx=8, pady=8)
        ttk.Button(top, text='Run evaluation suite (all 12 scenarios)', style='Accent.TButton', command=s.run_suite).pack(side='left'); ttk.Button(top, text='Run all tests T1-T8', command=s.run_tests).pack(side='left', padx=8)
        s.suite_bar = ttk.Progressbar(top, length=240, maximum=1.0); s.suite_bar.pack(side='left', padx=8); s.suite_lbl = tk.Label(top, text='', bg=BG, fg=MUTED); s.suite_lbl.pack(side='left')
        c = s.card(t, 'LEAD TIME vs FIXED-THRESHOLD ALARM (90 C), false alarms, Isolation-Forest ablation, attack defense (M6)'); c.pack(fill='both', expand=True, padx=8, pady=(0, 8))
        cols = ('scenario', 'CHRONOS alert t', 'shutdown t', 'fixed alarm t', 'critical t (no action)', 'lead CHRONOS', 'lead fixed', 'false alarm C/F', 'IsoForest 1st flag', 'IF false flags', 'attack TP/FP/FN')
        s.met_tree = s.tree(c, cols, (110, 100, 80, 90, 120, 90, 80, 95, 110, 90, 110), 12); s.met_tree.frame.pack(fill='both', expand=True, padx=8, pady=6)
        s.sum_lbl = tk.Label(c, text='', bg=BG, fg=TXT, font=F_B, justify='left'); s.sum_lbl.pack(anchor='w', padx=12, pady=(0, 6))
        s.tests_txt = s.text(c, 9); s.tests_txt.pack(fill='x', padx=8, pady=(0, 8))

    def run_suite(s):
        s.suite_lbl.config(text='running...')
        def work():
            rows, summ = evaluate(s.models, s.cfg, progress=lambda p, k: s.q.put(('log', 'suite: ' + k)) or None)
            s.q.put(('suite', (rows, summ)))
        threading.Thread(target=work, daemon=True).start()

    def show_suite(s, p):
        s.suite_rows = p; rows, summ = p; tr = s.met_tree; tr.delete(*tr.get_children()); f = lambda x: '-' if x is None else '%.1f' % x
        for r in rows: tr.insert('', 'end', values=(r['kind'], f(r['chronos']), f(r['shutdown']), f(r['base']), f(r['crit_cf']), f(r['lead_c']), f(r['lead_b']), '%s / %s' % ('Y' if r['fa_c'] else '-', 'Y' if r['fa_b'] else '-'), f(r['iforest']), r['if_fp'], '%d/%d/%d' % (r['tp'], r['fp'], r['fn'])))
        real = [r for r in rows if r['real']]; lc = [r['lead_c'] for r in real if r['lead_c'] is not None]; lb = [r['lead_b'] for r in real if r['lead_b'] is not None]
        fm = lambda x: 'n/a' if x is None else '%.2f' % x
        s.sum_lbl.config(text='Mean lead time (real failures): CHRONOS %s min  vs  fixed threshold %s min (missed %d of %d)\nFalse alarms (benign scenarios): CHRONOS %d  vs  fixed threshold %d     Brier score (P_crit): %s\nAttack-defense precision %s  recall %s'
                         % (fm(np.mean(lc) if lc else None), fm(np.mean(lb) if lb else None), len(real) - len(lb), len(real), summ['fa_chronos'], summ['fa_base'], fm(summ['brier']), fm(summ['precision']), fm(summ['recall'])))
        s.suite_bar['value'] = 1.0; s.suite_lbl.config(text='done')

    def run_tests(s):
        s.tests_txt.delete('1.0', 'end'); s.tests_txt.insert('end', 'running T1-T8 (takes ~1 min)...\n')
        threading.Thread(target=lambda: run_all_tests(s.models, s.cfg, log=lambda t: s.q.put(('tests', t))), daemon=True).start()

    # ================= SETTINGS
    def build_settings(s):
        t = s.tab['Settings']; l = tk.Frame(t, bg=BG); l.pack(side='left', fill='both', expand=True, padx=8, pady=8)
        c = s.card(l, 'GROQ  (key stays in memory only)'); c.pack(fill='x', pady=(0, 8)); s.groq_key = tk.StringVar(); s.groq_model = tk.StringVar(value=GROQ_MODELS[1])
        r = tk.Frame(c, bg=BG); r.pack(fill='x', padx=12, pady=4); tk.Label(r, text='API key', bg=BG, width=12, anchor='w').pack(side='left'); ttk.Entry(r, textvariable=s.groq_key, show='*', width=48).pack(side='left')
        r = tk.Frame(c, bg=BG); r.pack(fill='x', padx=12, pady=4); tk.Label(r, text='Model', bg=BG, width=12, anchor='w').pack(side='left'); s.cb_model = ttk.Combobox(r, textvariable=s.groq_model, values=GROQ_MODELS, width=36); s.cb_model.pack(side='left')
        ttk.Button(r, text='Refresh live model list', command=s.refresh_models).pack(side='left', padx=8)
        tk.Label(c, text='Verify IDs at console.groq.com/docs/models. Endpoint: https://api.groq.com/openai/v1/chat/completions (OpenAI-compatible). If Groq fails -> template text.', bg=BG, fg=MUTED, wraplength=640, justify='left').pack(anchor='w', padx=12, pady=(2, 10))
        c = s.card(l, 'TELEGRAM'); c.pack(fill='x', pady=8); s.tg_tok = tk.StringVar(); s.tg_chat = tk.StringVar()
        r = tk.Frame(c, bg=BG); r.pack(fill='x', padx=12, pady=4); tk.Label(r, text='Bot token', bg=BG, width=12, anchor='w').pack(side='left'); ttk.Entry(r, textvariable=s.tg_tok, show='*', width=48).pack(side='left')
        r = tk.Frame(c, bg=BG); r.pack(fill='x', padx=12, pady=4); tk.Label(r, text='Chat id', bg=BG, width=12, anchor='w').pack(side='left'); ttk.Entry(r, textvariable=s.tg_chat, width=24).pack(side='left'); ttk.Button(r, text='Test', style='Accent.TButton', command=s.test_tg).pack(side='left', padx=8)
        ttk.Button(r, text='Test Groq', command=s.test_groq).pack(side='left')
        c = s.card(l, 'DECISION POLICY  (applies live)'); c.pack(fill='x', pady=8)
        for key, lo, hi, nm in (('p_alert', 0.05, 0.9, 'p_alert  (alert level)'), ('p_act', 0.3, 0.99, 'p_act  (auto-shutdown level)'), ('horizon', 10, 80, 'forecast horizon (min)'), ('lam', 0.0, 2.0, 'lambda (CVaR weight)'), ('mu', 0.0, 3.0, 'mu (cost weight)')):
            r = tk.Frame(c, bg=BG); r.pack(fill='x', padx=12, pady=3); tk.Label(r, text=nm, bg=BG, width=26, anchor='w').pack(side='left'); v = tk.DoubleVar(value=s.cfg[key]); lb = tk.Label(r, text='%.2f' % s.cfg[key], bg=BG, width=6)
            ttk.Scale(r, from_=lo, to=hi, variable=v, length=260, command=lambda x, k=key, v=v, lb=lb: (s.cfg.__setitem__(k, round(float(v.get()), 2)), lb.config(text='%.2f' % s.cfg[k]))).pack(side='left'); lb.pack(side='left')
        r = tk.Frame(t, bg=BG); r.pack(side='right', fill='both', expand=True, padx=8, pady=8)
        c = s.card(r, 'LEARNED LIMITS (unsupervised, from NORMAL data)'); c.pack(fill='x'); s.lim_txt = s.text(c, 7); s.lim_txt.pack(fill='x', padx=8, pady=8)
        c = s.card(r, 'ALERT LOG  (plain-language, also sent to Telegram when configured)'); c.pack(fill='both', expand=True, pady=(8, 0)); s.alert_txt = s.text(c, 14, False); s.alert_txt.pack(fill='both', expand=True, padx=8, pady=8)

    def refresh_models(s):
        key = s.groq_key.get().strip() or os.environ.get('GROQ_API_KEY', '')
        if not key: s.log_line('Enter Groq key first'); return
        def work():
            try: s.q.put(('models', groq_models(key)))
            except Exception as ex: s.q.put(('log', 'Model refresh failed: ' + _scrub(ex, key)[:100]))
        threading.Thread(target=work, daemon=True).start()

    def test_groq(s):
        key = s.groq_key.get().strip() or os.environ.get('GROQ_API_KEY', ''); st = s.engine.state_json()
        def work():
            try: s.q.put(('log', 'Groq OK: ' + groq_chat(key, s.groq_model.get(), 'one line status?', st)[:120]))
            except Exception as ex: s.q.put(('log', 'Groq test failed: ' + _scrub(ex, key)[:100]))
        threading.Thread(target=work, daemon=True).start()

    def test_tg(s):
        tok = s.tg_tok.get().strip() or os.environ.get('TELEGRAM_BOT_TOKEN', ''); chat = s.tg_chat.get().strip() or os.environ.get('TELEGRAM_CHAT_ID', '')
        def work():
            try: telegram_send(tok, chat, 'CHRONOS-X test message: Telegram link works.'); s.q.put(('log', 'Telegram test sent'))
            except Exception as ex: s.q.put(('log', 'Telegram test failed: ' + _scrub(ex, tok)[:100]))
        threading.Thread(target=work, daemon=True).start()

    # ---------- periodic refresh (only the visible tab is redrawn)
    def refresh(s, force=False):
        if not s.engine or not s.world: return
        s.upd_side()
        if not (force or s.frame % 3 == 0): return
        n = s.nb.tab(s.nb.select(), 'text')
        if n == 'Live Monitor': s.upd_live()
        elif n == 'Hypotheses': s.upd_hyp()
        elif n == 'Simulation': s.upd_sim()
        elif n == 'Decision': s.upd_dec()
        elif n == 'Agents': s.upd_agents()
        elif n == 'Timeline': s.upd_time()
        elif n == 'Audit' and s.frame % 15 == 0: s.upd_audit()


def main():
    if '--test' in sys.argv:
        m = train_models(CFG); print(m['summary']); r = run_all_tests(m, dict(CFG)); sys.exit(0 if all(x[1] for x in r) else 1)
    if '--serve' in sys.argv:
        i = sys.argv.index('--serve'); port = int(sys.argv[i + 1]) if len(sys.argv) > i + 1 else 8000; serve_http(port, train_models(CFG), dict(CFG)); return
    if not HAVE_TK: print('tkinter / matplotlib missing. Thonny users: Tools > Manage packages.'); return
    root = tk.Tk(); App(root); root.mainloop()


if __name__ == '__main__':
    main()
