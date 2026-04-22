#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ADXL355 Web Monitor
- Flask + Gunicorn で動作
- 10Hz のピーク値を SSE (/stream_ui) で配信
- SQLite に 1秒/1分集約を保存
"""

import time, threading, json, sqlite3, queue, os, math, collections, atexit
import urllib.request, urllib.error
from flask import Flask, Response, send_from_directory, request, jsonify

# ==== 震度3以上で Webhook (Tweet) ====
# URL は秘匿。systemd の Environment= か環境変数 TWEET_WEBHOOK_URL で注入する。
TWEET_WEBHOOK_URL = os.environ.get("TWEET_WEBHOOK_URL", "").strip()
TWEET_MIN_LEVEL = int(os.environ.get("TWEET_MIN_LEVEL", "3"))

def _shindo_to_level(sh):
    """震度文字列 → 整数レベル（5弱→5, 5強→5, ... 不明→0）"""
    if not sh: return 0
    try:
        return int(sh[0])
    except (ValueError, IndexError):
        return 0

def _post_tweet_async(shindo, max_gal, max_h, max_v, duration, start_ts):
    if not TWEET_WEBHOOK_URL:
        return
    def _do():
        try:
            tstr = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_ts))
            text = (f"【地震検知】{tstr} 推定震度 {shindo} "
                    f"最大 {max_gal:.1f} gal (H {max_h:.1f} / V {max_v:.1f}) "
                    f"継続 {duration:.1f}秒 #自宅震度計")
            payload = json.dumps({
                "text": text,
                "shindo": shindo,
                "max_gal": round(max_gal, 1),
                "max_h": round(max_h, 1),
                "max_v": round(max_v, 1),
                "duration_sec": round(duration, 1),
                "start_ts": int(start_ts),
                "time": tstr,
            }).encode("utf-8")
            req = urllib.request.Request(
                TWEET_WEBHOOK_URL, data=payload,
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=10) as r:
                print(f"[tweet webhook] {r.status} shindo={shindo}")
        except Exception as e:
            print(f"[tweet webhook ERROR] {e}")
    threading.Thread(target=_do, daemon=True).start()

class BiquadLP:
    """2次バターワース ローパスフィルタ（fc=5Hz, fs=125Hz）
    強震計の反エイリアス / 高周ノイズ除去。地震帯域 0.1〜5Hz はとおす（建築物・地盤共振帯域）。"""
    __slots__ = ('x1','x2','y1','y2')
    def __init__(self):
        self.x1=self.x2=self.y1=self.y2=0.0
    def reset(self):
        self.x1=self.x2=self.y1=self.y2=0.0
    def process(self, x):
        # fc=5Hz, fs=125Hz, Butterworth Q=1/√2
        # b=[0.01335, 0.02670, 0.01335]  a=[1.0, -1.6479, 0.7010]
        y = (0.01335*x + 0.02670*self.x1 + 0.01335*self.x2
             + 1.6479*self.y1 - 0.7010*self.y2)
        self.x2=self.x1; self.x1=x
        self.y2=self.y1; self.y1=y
        return y

# --- SPI import ---
import spidev
try:
    import gpiod
except ModuleNotFoundError:
    gpiod = None
try:
    import RPi.GPIO as GPIO
except ModuleNotFoundError:
    GPIO = None

# ==== Flask ====
app = Flask(__name__, static_folder="static", static_url_path="/static")

# ==== ADXL355 定義 ====
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED_HZ = 1_000_000
DRDY_PIN = int(os.getenv("ADXL355_DRDY_PIN", "22"))
USE_DRDY = os.getenv("ADXL355_USE_DRDY", "1").lower() not in ("0", "false", "no", "off")
DRDY_TIMEOUT_MS = int(os.getenv("ADXL355_DRDY_TIMEOUT_MS", "200"))

REG_DEVID_AD = 0x00
REG_DEVID_MST = 0x01
REG_PARTID = 0x02
REG_REVID = 0x03
REG_XDATA3 = 0x08
REG_YDATA3 = 0x0B
REG_ZDATA3 = 0x0E
REG_RANGE = 0x2C
REG_FILTER = 0x28      # bits[6:4]=HPF_CORNER, bits[3:0]=ODR_LPF
REG_POWER_CTL = 0x2D
REG_SELF_TEST = 0x2E   # ★絶対に 0 にすること（旧コードはここを誤って FILTER と勘違いしていた）
REG_RESET = 0x2F

RANGE_BITS = 0x01  # ±4g
LSB_PER_G = 128000.0  # ADXL355 ±4g
SCALE_GAL = 980.665 / LSB_PER_G
DB_PATH = os.path.join(os.path.dirname(__file__), "mpu.db")

# ==== SPI 初期化 ====
spi = None
_spi_ok = False
_drdy_ready = False
_drdy_backend = None
_drdy_request = None

def _cleanup_gpio():
    global _drdy_request
    if _drdy_request is not None:
        try:
            _drdy_request.release()
        except Exception:
            pass
        _drdy_request = None
    if GPIO is not None:
        try:
            GPIO.cleanup()
        except Exception:
            pass

atexit.register(_cleanup_gpio)

def _ensure_spi():
    global spi, _spi_ok
    if not _spi_ok:
        spi = spidev.SpiDev()
        spi.open(SPI_BUS, SPI_DEVICE)
        spi.max_speed_hz = SPI_SPEED_HZ
        spi.mode = 0b00
        _spi_ok = True

def _spi_read_reg(reg):
    _ensure_spi()
    rx = spi.xfer2([(reg << 1) | 0x01, 0x00])
    return rx[1]

def _spi_write_reg(reg, value):
    _ensure_spi()
    spi.xfer2([(reg << 1) & 0xFE, value & 0xFF])

def _read_axis_raw(reg_base):
    _ensure_spi()
    rx = spi.xfer2([(reg_base << 1) | 0x01, 0x00, 0x00, 0x00])
    b3, b2, b1 = rx[1], rx[2], rx[3]
    raw = (b3 << 12) | (b2 << 4) | (b1 >> 4)
    if raw & (1 << 19):
        raw -= 1 << 20
    return raw

def read_xyz_gal():
    x = _read_axis_raw(REG_XDATA3) * SCALE_GAL
    y = _read_axis_raw(REG_YDATA3) * SCALE_GAL
    z = _read_axis_raw(REG_ZDATA3) * SCALE_GAL
    return x, y, z

def _spi_write_verify(reg, value, expected=None, retries=5, delay=0.01):
    """書込→読戻しでベリファイし、失敗時はリトライ。expected省略時はvalueと一致チェック。"""
    if expected is None:
        expected = value & 0xFF
    last = None
    for i in range(retries):
        _spi_write_reg(reg, value)
        time.sleep(delay)
        rb = _spi_read_reg(reg)
        last = rb
        if rb == expected:
            return rb, i + 1
    return last, retries  # 最後の値と試行回数

def adxl_init():
    _ensure_spi()
    devid = _spi_read_reg(REG_DEVID_AD)
    devid_mst = _spi_read_reg(REG_DEVID_MST)
    partid = _spi_read_reg(REG_PARTID)
    if not (devid == 0xAD and devid_mst == 0x1D and partid == 0xED):
        raise RuntimeError(
            f"ADXL355 not found: DEVID_AD=0x{devid:02X} DEVID_MST=0x{devid_mst:02X} PARTID=0x{partid:02X}"
        )
    # ハードリセット（0x52 を RESET レジスタへ）→ ADXL355 内部の再起動完了を十分待つ
    _spi_write_reg(REG_RESET, 0x52)
    time.sleep(0.1)  # データシート的には数ms で良いが、サービス起動直後の不安定対策で長めに

    # 順序は: standby → 設定 → measurement
    pc_rb, pc_try = _spi_write_verify(REG_POWER_CTL, 0x01)  # standby
    rg_rb, rg_try = _spi_write_verify(REG_RANGE, RANGE_BITS)
    st_rb, st_try = _spi_write_verify(REG_SELF_TEST, 0x00)
    # FILTER (0x28): HPF=000(無効), ODR_LPF=0101(=125Hz, LPF=ODR/4=31.25Hz)
    f_rb, f_try = _spi_write_verify(REG_FILTER, 0x05)
    # measurement モードへ。POWER_CTL はリセット直後に書込が落ちることがあるのでリトライ必須
    m_rb, m_try = _spi_write_verify(REG_POWER_CTL, 0x00)
    time.sleep(0.05)

    print(f"ADXL355初期化完了: FILTER=0x{f_rb:02X}(試{f_try}) SELF_TEST=0x{st_rb:02X}(試{st_try}) "
          f"RANGE=0x{rg_rb:02X}(試{rg_try}) POWER_CTL=0x{m_rb:02X}(試{m_try}) ODR=125Hz LPF=31Hz RANGE=±4g")

    # 全てのレジスタが期待値か検証。NG なら例外を投げて systemd に再起動させる
    failed = []
    if f_rb != 0x05: failed.append(f"FILTER=0x{f_rb:02X}≠0x05")
    if st_rb != 0x00: failed.append(f"SELF_TEST=0x{st_rb:02X}≠0x00")
    if rg_rb != RANGE_BITS: failed.append(f"RANGE=0x{rg_rb:02X}≠0x{RANGE_BITS:02X}")
    if m_rb != 0x00: failed.append(f"POWER_CTL=0x{m_rb:02X}≠0x00")
    if failed:
        raise RuntimeError("ADXL355レジスタ書込検証失敗: " + ", ".join(failed))

    # アナログ部生存確認: TEMP と XYZ を読み、両方ゼロなら故障扱い
    time.sleep(0.05)
    t_hi = _spi_read_reg(0x06); t_lo = _spi_read_reg(0x07)
    temp_raw = (t_hi << 8) | t_lo
    nonzero = 0
    for _ in range(5):
        time.sleep(0.02)
        x, y, z = read_xyz_gal()
        if x != 0 or y != 0 or z != 0:
            nonzero += 1
    print(f"ADXL355アナログ部確認: TEMP=0x{temp_raw:04X}({temp_raw}) 非ゼロサンプル={nonzero}/5")
    if temp_raw == 0 and nonzero == 0:
        raise RuntimeError(
            "ADXL355アナログ部応答なし (TEMP=0 かつ XYZ全0)。電源/配線確認またはセンサー故障の可能性。"
        )

def setup_drdy():
    global _drdy_ready, _drdy_backend, _drdy_request
    if not USE_DRDY:
        print("DRDY無効: タイマー同期でサンプリング")
        _drdy_ready = False
        _drdy_backend = None
        return
    if gpiod is not None:
        try:
            chip = gpiod.Chip('/dev/gpiochip0')
            _drdy_request = chip.request_lines(
                consumer='adxl355-drdy',
                config={DRDY_PIN: gpiod.LineSettings(edge_detection=gpiod.line.Edge.RISING)}
            )
            _drdy_ready = True
            _drdy_backend = "gpiod"
            print(f"DRDY有効(gpiod): GPIO{DRDY_PIN} 立ち上がり同期 @ 125Hz")
            return
        except Exception as e:
            print(f"DRDY gpiod初期化失敗 ({e})。RPi.GPIOへフォールバック")
    if GPIO is None:
        print("DRDY未使用: 利用可能なGPIOライブラリがないためタイマー同期へフォールバック")
        _drdy_ready = False
        _drdy_backend = None
        return
    try:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(DRDY_PIN, GPIO.IN)
        _drdy_ready = True
        _drdy_backend = "rpi_gpio"
        print(f"DRDY有効(RPi.GPIO): GPIO{DRDY_PIN} 立ち上がり同期")
    except Exception as e:
        print(f"DRDY初期化失敗 ({e})。タイマー同期へフォールバック")
        _drdy_ready = False
        _drdy_backend = None

def wait_next_sample(next_t, dt):
    global _drdy_ready, _drdy_backend
    if _drdy_ready:
        try:
            if _drdy_backend == "gpiod" and _drdy_request is not None:
                if _drdy_request.wait_edge_events(timeout=DRDY_TIMEOUT_MS / 1000.0):
                    _drdy_request.read_edge_events()
                    return next_t + dt
            elif _drdy_backend == "rpi_gpio":
                edge = GPIO.wait_for_edge(DRDY_PIN, GPIO.RISING, timeout=DRDY_TIMEOUT_MS)
                if edge is not None:
                    return next_t + dt
        except RuntimeError as e:
            print(f"DRDY待機エラー ({e})。タイマー同期へフォールバック")
            _drdy_ready = False
            _drdy_backend = None
    sleep_rem = next_t - time.monotonic()
    if sleep_rem > 0:
        time.sleep(sleep_rem)
    return next_t + dt

def calibrate(n=1000, delay=0.005, warmup=50, settle_window=50, settle_threshold=15.0):
    # ウォームアップ: センサー安定化のため破棄
    for _ in range(warmup):
        read_xyz_gal()
        time.sleep(delay)
    # 定常状態確認: Z軸stddevが閾値以下になるまで待機（最大10秒）
    deadline = time.time() + 10.0
    while time.time() < deadline:
        buf = []
        for _ in range(settle_window):
            _, _, z = read_xyz_gal()
            buf.append(z)
            time.sleep(delay)
        mean_z = sum(buf) / settle_window
        stddev = (sum((v - mean_z) ** 2 for v in buf) / settle_window) ** 0.5
        if stddev < settle_threshold:
            break
        print(f"キャリブレーション待機中... stddev={stddev:.1f} LSB")
    # 本計測: 1000サンプル平均
    sx = sy = sz = 0
    for _ in range(n):
        x, y, z = read_xyz_gal()
        sx += x
        sy += y
        sz += z
        time.sleep(delay)
    return sx/n, sy/n, sz/n

# ==== DB 初期化 ====
def db_init():
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute("""
      CREATE TABLE IF NOT EXISTS sec (
        ts INTEGER PRIMARY KEY,
        ax_mean REAL, ay_mean REAL, az_mean REAL,
        ax_peak REAL, ay_peak REAL, az_peak REAL,
        h_absmax REAL, v_absmax REAL
      )
    """)
    c.execute("""
      CREATE TABLE IF NOT EXISTS min (
        ts INTEGER PRIMARY KEY,
        ax_mean REAL, ay_mean REAL, az_mean REAL,
        ax_peak REAL, ay_peak REAL, az_peak REAL,
        h_absmax REAL, v_absmax REAL
      )
    """)
    c.execute("""
      CREATE TABLE IF NOT EXISTS events (
        id           INTEGER PRIMARY KEY AUTOINCREMENT,
        start_ts     INTEGER,
        end_ts       INTEGER,
        max_gal      REAL,
        shindo       TEXT,
        duration_sec REAL,
        max_h        REAL,
        max_v        REAL
      )
    """)
    # 既存DBに列が無い場合の追加（マイグレーション）
    c.execute("PRAGMA table_info(events)")
    cols = {row[1] for row in c.fetchall()}
    if "max_h" not in cols:
        c.execute("ALTER TABLE events ADD COLUMN max_h REAL")
    if "max_v" not in cols:
        c.execute("ALTER TABLE events ADD COLUMN max_v REAL")
    c.execute("PRAGMA journal_mode=WAL")
    c.execute("PRAGMA synchronous=NORMAL")
    conn.commit(); conn.close()

def _shindo_str(gal):
    if gal>=400: return '7'
    if gal>=315: return '6強'
    if gal>=250: return '6弱'
    if gal>=140: return '5強'
    if gal>=80:  return '5弱'
    if gal>=25:  return '4'
    if gal>=8:   return '3'
    if gal>=2.5: return '2'
    if gal>=0.8: return '1'
    return '0'

def _iva_to_shindo(iva):
    """JMA 計測震度 Iva → 震度階級（切り捨て式）"""
    if iva>=6.5: return '7'
    if iva>=6.0: return '6強'
    if iva>=5.5: return '6弱'
    if iva>=5.0: return '5強'
    if iva>=4.5: return '5弱'
    if iva>=3.5: return '4'
    if iva>=2.5: return '3'
    if iva>=1.5: return '2'
    if iva>=0.5: return '1'
    return '0'

def _save_event(cur, conn, start_ts, end_ts, max_gal, shindo, duration, max_h=None, max_v=None):
    cur.execute("""
      INSERT INTO events (start_ts, end_ts, max_gal, shindo, duration_sec, max_h, max_v)
      VALUES (?,?,?,?,?,?,?)
    """, (start_ts, end_ts, round(max_gal,2), shindo, round(duration,1),
          None if max_h is None else round(max_h,2),
          None if max_v is None else round(max_v,2)))
    conn.commit()

# ==== SSE （UI向け 10Hz ピーク）====
subs_ui = set()
lock_ui = threading.Lock()

# ==== 手動キャリブレーション要求フラグ ====
recal_request = threading.Event()
recal_status = {"state": "idle", "ts": 0}  # idle | requested | running | done | error

def ui_broadcast(payload_json):
    with lock_ui:
        dead=[]
        for q in subs_ui:
            try: q.put_nowait(payload_json)
            except queue.Full:
                try: q.get_nowait(); q.put_nowait(payload_json)
                except Exception: dead.append(q)
        for q in dead: subs_ui.discard(q)

def ui_stream():
    q = queue.Queue(maxsize=200)
    with lock_ui: subs_ui.add(q)
    try:
        yield "event: ping\ndata: {}\n\n"
        while True:
            data = q.get()
            yield f"data: {data}\n\n"
    except GeneratorExit:
        pass
    finally:
        with lock_ui: subs_ui.discard(q)

# ==== 収集スレッド ====
def producer_loop():
    time.sleep(1.0)  # 起動直後のSPI安定待ち
    adxl_init()
    setup_drdy()
    print("水平に置いてください（3秒後にキャリブレーション開始）")
    time.sleep(3)
    ax0, ay0, az0 = calibrate()
    print("キャリブレーション完了。配信開始。")

    dt = 0.008  # 125Hzサンプリング

    # LP IIR フィルタ（高周ノイズ除去 / 反エイリアス）
    # HPFは使わず、起動時オフセット減算（ax0/ay0/az0）と自動再キャリブでドリフトを吸収させる
    lp_x = BiquadLP(); lp_y = BiquadLP(); lp_z = BiquadLP()

    # 表示用移動平均（HP 後のノイズ低減）
    FILTER_LEN = 5
    _filt_x = collections.deque([0.0] * FILTER_LEN, maxlen=FILTER_LEN)
    _filt_y = collections.deque([0.0] * FILTER_LEN, maxlen=FILTER_LEN)
    _filt_z = collections.deque([0.0] * FILTER_LEN, maxlen=FILTER_LEN)

    # STA/LTA 地震検知
    STA_LEN     = 625    # 5秒 @ 125Hz
    LTA_LEN     = 7500   # 60秒 @ 125Hz
    STALTA_ON   = 4.0    # トリガー閾値（誤検知低減のため厳しめ）
    STALTA_OFF  = 1.8    # デトリガー閾値
    QUAKE_MIN_DUR = 5.0  # 地震確定の最小継続秒数
    sta_buf = collections.deque(maxlen=STA_LEN)
    lta_buf = collections.deque(maxlen=LTA_LEN)
    sta_sum = lta_sum = 0.0
    quake_on = False; quake_start_t = 0.0; quake_max_gal = 0.0
    quake_max_h = 0.0; quake_max_v = 0.0
    quake_c_hist = []  # 地震中の c_raw 全サンプル（Iva計算用）

    # 自動再キャリブレーション: 300秒EMAで持続的オフセットを検出
    # EMA時定数を長くすることで長い地震（1〜2分）との誤判定を防ぐ
    # calibrate()内のstddevチェックが揺れ継続中の実行を防ぐ二重安全
    RECAL_TAU       = 300.0  # EMA時定数（秒）= 5分。長い地震でも誤作動しにくい
    RECAL_THRESHOLD = 5.0    # gal: 持続平均がこれを超えたら再キャリブレーション
    RECAL_COOLDOWN  = 600.0  # 秒: 再キャリブレーション後10分クールダウン
    ema_alpha = dt / RECAL_TAU
    ema_x = ema_y = ema_z = 0.0
    recal_cooldown_rem = RECAL_TAU  # 起動直後はEMAが収束するまで待つ

    cur_sec = None
    s_cnt = 0
    s_sum_x = s_sum_y = s_sum_z = 0.0
    s_peak_x = s_peak_y = s_peak_z = 0.0
    s_h_absmax = s_v_absmax = 0.0

    cur_min = None
    m_cnt = 0
    m_sum_x = m_sum_y = m_sum_z = 0.0
    m_peak_x = m_peak_y = m_peak_z = 0.0
    m_h_absmax = m_v_absmax = 0.0

    # UI用 100msバケツ
    last_bin = None
    bin_peak_x = bin_peak_y = bin_peak_z = 0.0
    bin_abs_x = bin_abs_y = bin_abs_z = 0.0
    bin_h_abs = bin_v_abs = bin_c_abs = 0.0

    # 計測震度簡易版: 直近3秒の c を保持し、
    # |c| > a の累計時間 ≥ 0.3秒 となる最大 a を探す
    IVA_WINDOW_SEC = 3.0
    IVA_THRESH_SEC = 0.3
    IVA_MIN_SAMPLES = int(IVA_THRESH_SEC / 0.008)  # 125Hzにさ3サンプル以上
    c_hist = collections.deque(maxlen=int(IVA_WINDOW_SEC / 0.008) + 4)

    conn = sqlite3.connect(DB_PATH, check_same_thread=False)
    cur = conn.cursor()

    next_t = time.monotonic() + dt
    while True:
        next_t = wait_next_sample(next_t, dt)
        try:
            xg, yg, zg = read_xyz_gal()
            raw_ax = xg - ax0
            raw_ay = yg - ay0
            raw_az = zg - az0
        except (OSError, RuntimeError):
            time.sleep(0.01); continue

        # LP IIR フィルタ（高周ノイズ除去）→ 移動平均（表示用さらにスムージング）
        lp_ax = lp_x.process(raw_ax)
        lp_ay = lp_y.process(raw_ay)
        lp_az = lp_z.process(raw_az)
        _filt_x.append(lp_ax); _filt_y.append(lp_ay); _filt_z.append(lp_az)
        ax = sum(_filt_x) / FILTER_LEN
        ay = sum(_filt_y) / FILTER_LEN
        az = sum(_filt_z) / FILTER_LEN

        # ---- 自動再キャリブレーション判定 ----
        ema_x = ema_alpha * ax + (1 - ema_alpha) * ema_x
        ema_y = ema_alpha * ay + (1 - ema_alpha) * ema_y
        ema_z = ema_alpha * az + (1 - ema_alpha) * ema_z
        recal_cooldown_rem = max(0.0, recal_cooldown_rem - dt)
        if recal_cooldown_rem == 0.0 and (abs(ema_x) > RECAL_THRESHOLD or abs(ema_y) > RECAL_THRESHOLD):
            print(f"オフセットドリフト検出（EMA x={ema_x:.1f} y={ema_y:.1f} gal）。自動再キャリブレーション開始...")
            ax0, ay0, az0 = calibrate()
            lp_x.reset(); lp_y.reset(); lp_z.reset()
            for buf in (_filt_x, _filt_y, _filt_z):
                buf.clear(); buf.extend([0.0] * FILTER_LEN)
            sta_buf.clear(); lta_buf.clear()
            sta_sum = lta_sum = 0.0
            quake_on = False; quake_start_t = 0.0; quake_max_gal = 0.0
            quake_max_h = 0.0; quake_max_v = 0.0
            ema_x = ema_y = ema_z = 0.0
            recal_cooldown_rem = RECAL_COOLDOWN
            print("自動再キャリブレーション完了。")

        # ---- 手動再キャリブレーション要求 ----
        if recal_request.is_set():
            recal_request.clear()
            recal_status["state"] = "running"; recal_status["ts"] = time.time()
            print("手動再キャリブレーション要求を受信。実行中...")
            try:
                ui_broadcast(json.dumps({"type":"recal_start","t":time.time()}))
                ax0, ay0, az0 = calibrate()
                lp_x.reset(); lp_y.reset(); lp_z.reset()
                for buf in (_filt_x, _filt_y, _filt_z):
                    buf.clear(); buf.extend([0.0] * FILTER_LEN)
                sta_buf.clear(); lta_buf.clear()
                sta_sum = lta_sum = 0.0
                quake_on = False; quake_start_t = 0.0; quake_max_gal = 0.0
                quake_max_h = 0.0; quake_max_v = 0.0
                ema_x = ema_y = ema_z = 0.0
                recal_cooldown_rem = RECAL_COOLDOWN
                recal_status["state"] = "done"; recal_status["ts"] = time.time()
                print("手動再キャリブレーション完了。")
                ui_broadcast(json.dumps({"type":"recal_done","t":time.time()}))
            except Exception as e:
                recal_status["state"] = "error"; recal_status["ts"] = time.time()
                print(f"手動再キャリブレーション失敗: {e}")
                ui_broadcast(json.dumps({"type":"recal_error","t":time.time(),"err":str(e)}))

        h = math.hypot(ax, ay)
        v = abs(az)
        now = time.time(); now_s = int(now)

        # ---- STA/LTA 地震検知 ----
        c_sta = math.hypot(math.hypot(lp_ax, lp_ay), abs(lp_az))
        if len(sta_buf) == STA_LEN: sta_sum -= sta_buf[0]
        sta_buf.append(c_sta); sta_sum += c_sta
        if len(lta_buf) == LTA_LEN: lta_sum -= lta_buf[0]
        lta_buf.append(c_sta); lta_sum += c_sta
        if len(lta_buf) == LTA_LEN and lta_sum > 0.0:
            ratio = (sta_sum / STA_LEN) / (lta_sum / LTA_LEN)
            # ピーク値追跡用: LP前の生加速度（瞬時値）。震度算出は本来高周波も含むため。
            c_raw = math.hypot(math.hypot(raw_ax, raw_ay), abs(raw_az))
            h_raw = math.hypot(raw_ax, raw_ay)
            v_raw = abs(raw_az)
            if not quake_on and ratio >= STALTA_ON:
                quake_on = True; quake_start_t = now; quake_max_gal = c_raw
                quake_max_h = h_raw; quake_max_v = v_raw
                quake_c_hist = [c_raw]
                print(f"[地震検知] STA/LTA={ratio:.2f}")
                ui_broadcast(json.dumps({"type":"quake_start","t":now,"ratio":round(ratio,2)}))
            elif quake_on:
                if c_raw > quake_max_gal: quake_max_gal = c_raw
                if h_raw > quake_max_h: quake_max_h = h_raw
                if v_raw > quake_max_v: quake_max_v = v_raw
                quake_c_hist.append(c_raw)
                if ratio < STALTA_OFF:
                    dur = now - quake_start_t
                    if dur >= QUAKE_MIN_DUR:
                        # JMA計測震度 Iva簡易版: 地震中の c_raw をソートし、
                        # 上から 0.3秒分サンプル目の値（= 0.3秒以上継続した最大加速度）
                        sorted_c = sorted(quake_c_hist, reverse=True)
                        idx = min(IVA_MIN_SAMPLES - 1, len(sorted_c) - 1)
                        a_iva_q = sorted_c[idx] if sorted_c else 0.0
                        iva_q = (2.0 * math.log10(a_iva_q) + 0.94) if a_iva_q > 0.0 else -1.0
                        sh = _iva_to_shindo(iva_q)
                        _save_event(cur, conn, int(quake_start_t), int(now), quake_max_gal, sh, dur,
                                    max_h=quake_max_h, max_v=quake_max_v)
                        print(f"[地震終了] 最大{quake_max_gal:.1f}gal H{quake_max_h:.1f} V{quake_max_v:.1f} a_iva={a_iva_q:.2f} Iva={iva_q:.2f} 推定震度{sh} 継続{dur:.1f}秒")
                        ui_broadcast(json.dumps({"type":"quake_end","t":now,
                            "max_gal":round(quake_max_gal,1),"shindo":sh,"duration":round(dur,1),
                            "max_h":round(quake_max_h,1),"max_v":round(quake_max_v,1),
                            "iva":round(iva_q,2)}))
                        if _shindo_to_level(sh) >= TWEET_MIN_LEVEL:
                            _post_tweet_async(sh, quake_max_gal, quake_max_h, quake_max_v,
                                              dur, quake_start_t)
                    else:
                        print(f"[誤検知破棄] {dur:.1f}秒（最小{QUAKE_MIN_DUR}秒未満）")
                        ui_broadcast(json.dumps({"type":"quake_cancel","t":now,
                            "duration":round(dur,1)}))
                    quake_on=False; quake_start_t=0.0; quake_max_gal=0.0
                    quake_max_h=0.0; quake_max_v=0.0
                    quake_c_hist = []

        # ---- 10Hzピーク更新 ----
        pbin = int(now*10)
        ax_abs, ay_abs, az_abs = abs(ax), abs(ay), abs(az)
        c = math.hypot(h, v)
        # ライブ波形は raw（LP前）の瞬時値も比較対象に入れて、震度4以上の高周ピークを取りこぼさない
        h_raw_inst = math.hypot(raw_ax, raw_ay)
        v_raw_inst = abs(raw_az)
        c_raw_inst = math.hypot(h_raw_inst, v_raw_inst)
        c_hist.append(c_raw_inst)
        if last_bin is None:
            last_bin = pbin
            bin_peak_x, bin_abs_x = raw_ax, abs(raw_ax)
            bin_peak_y, bin_abs_y = raw_ay, abs(raw_ay)
            bin_peak_z, bin_abs_z = raw_az, abs(raw_az)
            bin_h_abs, bin_v_abs, bin_c_abs = h_raw_inst, v_raw_inst, c_raw_inst
        elif pbin == last_bin:
            rax_abs, ray_abs, raz_abs = abs(raw_ax), abs(raw_ay), abs(raw_az)
            if rax_abs > bin_abs_x: bin_abs_x, bin_peak_x = rax_abs, raw_ax
            if ray_abs > bin_abs_y: bin_abs_y, bin_peak_y = ray_abs, raw_ay
            if raz_abs > bin_abs_z: bin_abs_z, bin_peak_z = raz_abs, raw_az
            if h_raw_inst > bin_h_abs: bin_h_abs = h_raw_inst
            if v_raw_inst > bin_v_abs: bin_v_abs = v_raw_inst
            if c_raw_inst > bin_c_abs: bin_c_abs = c_raw_inst
        else:
            # 計測震度簡易版: ソートして上から IVA_MIN_SAMPLES 番目の値が
            # 「|c|>a が累計 0.3秒以上となる最大 a」
            if len(c_hist) >= IVA_MIN_SAMPLES:
                sorted_c = sorted(c_hist, reverse=True)
                a_iva = sorted_c[IVA_MIN_SAMPLES - 1]
            else:
                a_iva = 0.0
            iva = (2.0 * math.log10(a_iva) + 0.94) if a_iva > 0.0 else -1.0
            ui_broadcast(json.dumps({
                "t": last_bin/10.0,
                "ax": bin_peak_x, "ay": bin_peak_y, "az": bin_peak_z,
                "h": bin_h_abs, "v": bin_v_abs, "c": bin_c_abs,
                "iva": round(iva, 2), "shindo": _iva_to_shindo(iva), "a_iva": round(a_iva, 3)
            }))
            last_bin = pbin
            bin_peak_x, bin_abs_x = raw_ax, abs(raw_ax)
            bin_peak_y, bin_abs_y = raw_ay, abs(raw_ay)
            bin_peak_z, bin_abs_z = raw_az, abs(raw_az)
            bin_h_abs, bin_v_abs, bin_c_abs = h_raw_inst, v_raw_inst, c_raw_inst

        # ---- 1秒ごとにDB保存 ----
        if cur_sec is None: cur_sec = now_s
        if now_s != cur_sec:
            if s_cnt > 0:
                ax_mean = s_sum_x/s_cnt; ay_mean = s_sum_y/s_cnt; az_mean = s_sum_z/s_cnt
                cur.execute("""
                  INSERT OR REPLACE INTO sec
                    (ts, ax_mean, ay_mean, az_mean, ax_peak, ay_peak, az_peak, h_absmax, v_absmax)
                  VALUES (?,?,?,?,?,?,?,?,?)
                """,(cur_sec, ax_mean, ay_mean, az_mean, s_peak_x, s_peak_y, s_peak_z, s_h_absmax, s_v_absmax))
                conn.commit()
            cur_sec = now_s
            s_cnt=0; s_sum_x=s_sum_y=s_sum_z=0.0
            s_peak_x=s_peak_y=s_peak_z=0.0
            s_h_absmax=s_v_absmax=0.0
        s_cnt+=1
        s_sum_x+=ax; s_sum_y+=ay; s_sum_z+=az
        if abs(raw_ax)>abs(s_peak_x): s_peak_x=raw_ax
        if abs(raw_ay)>abs(s_peak_y): s_peak_y=raw_ay
        if abs(raw_az)>abs(s_peak_z): s_peak_z=raw_az
        # h/v ピークは raw（LP前）瞬時値で取り、震度判定と一致させる
        s_h_absmax=max(s_h_absmax,h_raw_inst)
        s_v_absmax=max(s_v_absmax,v_raw_inst)

        # ---- 1分ごとにDB保存 ----
        now_m = now_s // 60
        if cur_min is None: cur_min = now_m
        if now_m != cur_min:
            if m_cnt > 0:
                cur.execute("""
                  INSERT OR REPLACE INTO min
                    (ts, ax_mean, ay_mean, az_mean, ax_peak, ay_peak, az_peak, h_absmax, v_absmax)
                  VALUES (?,?,?,?,?,?,?,?,?)
                """,(cur_min*60, m_sum_x/m_cnt, m_sum_y/m_cnt, m_sum_z/m_cnt,
                     m_peak_x, m_peak_y, m_peak_z, m_h_absmax, m_v_absmax))
                conn.commit()
            cur_min = now_m
            m_cnt=0; m_sum_x=m_sum_y=m_sum_z=0.0
            m_peak_x=m_peak_y=m_peak_z=0.0; m_h_absmax=m_v_absmax=0.0
        m_cnt+=1
        m_sum_x+=ax; m_sum_y+=ay; m_sum_z+=az
        if abs(raw_ax)>abs(m_peak_x): m_peak_x=raw_ax
        if abs(raw_ay)>abs(m_peak_y): m_peak_y=raw_ay
        if abs(raw_az)>abs(m_peak_z): m_peak_z=raw_az
        m_h_absmax=max(m_h_absmax,h_raw_inst); m_v_absmax=max(m_v_absmax,v_raw_inst)

        

# ==== 履歴 ====
def parse_window(s):
    s=s.lower().strip()
    if s.endswith("m"): return int(float(s[:-1])*60)
    if s.endswith("h"): return int(float(s[:-1])*3600)
    if s.endswith("d"): return int(float(s[:-1])*86400)
    if s.endswith("w"): return int(float(s[:-1])*7*86400)
    if s.endswith("mo"):return int(float(s[:-2])*30*86400)
    if s.endswith("y"): return int(float(s[:-1])*365*86400)
    return 300

def fetch_history(sec, interval=0, start_ts=None, end_ts=None):
    """sec: 期間秒（start_ts/end_ts 指定時は無視）, interval: 集計秒（0=自動）"""
    if start_ts is not None and end_ts is not None:
        start = int(start_ts); now = int(end_ts)
        sec = max(1, now - start)
    else:
        now=int(time.time()); start=now-sec
    conn=sqlite3.connect(DB_PATH); c=conn.cursor()
    # ソーステーブル選択: interval>=60 or 自動で長期 → min、それ以外 → sec
    use_min = (interval >= 60) if interval > 0 else (sec > 259200)
    src_iv = 60 if use_min else 1
    if use_min:
        c.execute("SELECT ts,ax_mean,ay_mean,az_mean,h_absmax,v_absmax FROM min WHERE ts>=? AND ts<=? ORDER BY ts",(start,now))
    else:
        c.execute("SELECT ts,ax_mean,ay_mean,az_mean,h_absmax,v_absmax FROM sec WHERE ts>=? AND ts<=? ORDER BY ts",(start,now))
    rows=c.fetchall(); conn.close()

    agg = max(interval, src_iv)  # 集計間隔（秒）

    if agg <= src_iv:
        out=[]
        for ts,ax,ay,az,h,v in rows:
            ax=ax or 0; ay=ay or 0; az=az or 0
            h=h or math.hypot(ax,ay); v=v or abs(az)
            out.append({"t":ts*1000,"ax":ax,"ay":ay,"az":az,"h":h,"v":v,"c":math.hypot(h,v)})
        return out

    # bin 平均
    bins={}
    for ts,ax,ay,az,h,v in rows:
        b=(ts//agg)*agg
        if b not in bins: bins[b]={"ax":[],"ay":[],"az":[],"h":[],"v":[]}
        bins[b]["ax"].append(ax or 0); bins[b]["ay"].append(ay or 0); bins[b]["az"].append(az or 0)
        bins[b]["h"].append(h or 0);   bins[b]["v"].append(v or 0)
    out=[]
    for b in sorted(bins.keys()):
        d=bins[b]; n=len(d["ax"])
        ax=sum(d["ax"])/n; ay=sum(d["ay"])/n; az=sum(d["az"])/n
        h=sum(d["h"])/n;   v=sum(d["v"])/n
        out.append({"t":b*1000,"ax":ax,"ay":ay,"az":az,"h":h,"v":v,"c":math.hypot(h,v)})
    return out

# ==== Flask ルート ====
@app.route("/")
def index():
    resp = send_from_directory("static","index.html")
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["Expires"] = "0"
    return resp

@app.route("/stream_ui")
def stream_ui():
    headers={"Content-Type":"text/event-stream","Cache-Control":"no-cache",
             "Connection":"keep-alive","X-Accel-Buffering":"no"}
    return Response(ui_stream(), headers=headers)

@app.route("/history")
def history():
    # カスタム範囲指定（epoch ms）
    start_ms = request.args.get("start")
    end_ms = request.args.get("end")
    try: iv=int(request.args.get("interval","0"))
    except: iv=0
    if start_ms and end_ms:
        try:
            st = int(start_ms) // 1000
            ed = int(end_ms) // 1000
            return jsonify(fetch_history(0, iv, start_ts=st, end_ts=ed))
        except Exception:
            return jsonify({"error":"bad start/end"}),400
    w=request.args.get("window","5m")
    try: sec=parse_window(w)
    except: return jsonify({"error":"bad window"}),400
    return jsonify(fetch_history(sec, iv))

@app.route("/events")
def get_events():
    start_ms = request.args.get("start")
    end_ms = request.args.get("end")
    try:
        limit = int(request.args.get("limit", "20"))
    except:
        limit = 20
    limit = max(1, min(limit, 500))
    conn=sqlite3.connect(DB_PATH); c=conn.cursor()
    if start_ms and end_ms:
        try:
            st = int(start_ms) // 1000
            ed = int(end_ms) // 1000
            c.execute("SELECT id,start_ts,end_ts,max_gal,shindo,duration_sec,max_h,max_v "
                      "FROM events WHERE start_ts>=? AND start_ts<=? "
                      "ORDER BY start_ts DESC LIMIT ?", (st, ed, limit))
        except Exception:
            conn.close()
            return jsonify({"error":"bad start/end"}),400
    else:
        c.execute("SELECT id,start_ts,end_ts,max_gal,shindo,duration_sec,max_h,max_v "
                  "FROM events ORDER BY start_ts DESC LIMIT ?", (limit,))
    rows=c.fetchall(); conn.close()
    return jsonify([{"id":r[0],"start":r[1]*1000,"end":r[2]*1000,
                     "max_gal":r[3],"shindo":r[4],"duration":r[5],
                     "max_h":r[6],"max_v":r[7]} for r in rows])

@app.route("/recalibrate", methods=["POST"])
def recalibrate_api():
    if recal_status["state"] == "running":
        return jsonify({"ok":False,"state":"running","msg":"既に実行中です"}), 409
    recal_status["state"] = "requested"; recal_status["ts"] = time.time()
    recal_request.set()
    return jsonify({"ok":True,"state":"requested"})

@app.route("/recal_status")
def recal_status_api():
    return jsonify(recal_status)

# ==== Gunicorn でも起動時にスレッドを一度だけ立ち上げる ====
db_init()

def daily_recal_scheduler():
    """毎日 DAILY_RECAL_HOUR 時に再キャリブレーション要求を発火する。
    揺れ検知中は producer_loop 側がキューを処理するタイミングで安全に実行される。
    """
    hour = int(os.environ.get("DAILY_RECAL_HOUR", "4"))   # 既定 4:00 (静かな時間)
    minute = int(os.environ.get("DAILY_RECAL_MINUTE", "0"))
    while True:
        now = time.time()
        lt = time.localtime(now)
        target = time.mktime((lt.tm_year, lt.tm_mon, lt.tm_mday,
                              hour, minute, 0, 0, 0, lt.tm_isdst))
        if target <= now:
            target += 86400
        sleep_sec = target - now
        time.sleep(sleep_sec)
        print(f"[daily recal] {time.strftime('%Y-%m-%d %H:%M:%S')} 定期再キャリブレーション要求")
        recal_request.set()

if not getattr(app, "_thread_started", False):
    threading.Thread(target=producer_loop, daemon=True).start()
    threading.Thread(target=daily_recal_scheduler, daemon=True).start()
    app._thread_started = True

# ==== 開発用（python3 app.py で直起動）====
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000, threaded=True)

