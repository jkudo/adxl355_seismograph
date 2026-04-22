# ADXL355 Seismograph

Raspberry Pi + ADXL355（高精度 3 軸加速度センサー）で構築する、簡易**地震モニター／簡易震度計**です。
SPI 経由で 125 Hz サンプリングし、Web ブラウザでリアルタイムに波形・推定震度・履歴を確認できます。

![Python](https://img.shields.io/badge/python-3.11+-blue)
![Flask](https://img.shields.io/badge/flask-3.x-lightgrey)
![License](https://img.shields.io/badge/license-MIT-green)

## 主な機能

- **125 Hz リアルタイム計測**: ADXL355 の DRDY ピン（GPIO22）で立ち上がり同期
- **STA/LTA 法による地震自動検知**: 5 秒短期平均 / 60 秒長期平均で長周期地震にも反応
- **JMA 簡易震度推定**: 計測震度 Iva と gal 値の二系統で算出
- **Web UI（Chart.js）**:
  - 合成加速度 √(X²+Y²+Z²) のメイングラフ
  - 横揺れ H = √(X²+Y²) と縦揺れ V = |Z| 別表示（H/V 比で揺れの種類を判定）
  - 震度参照ライン（震度 2 〜 7）
  - ドラッグで過去範囲をズーム表示
  - 期間選択（1 分〜1 年）／集計粒度切替
  - 過去地震履歴をサイドバーに 50 件表示、クリックで詳細モーダル
  - 24 時間表示・サーバー送信イベント（SSE）で 10 Hz 更新
- **自動 / 手動キャリブレーション**:
  - 起動時 1000 サンプル平均で初期オフセット確定
  - EMA（5 分時定数）で長期ドリフト検出 → 自動再キャリブレーション
  - ブラウザから手動再キャリブレーションボタン
- **SQLite (WAL) 永続化**: 1 秒ピーク・1 分ピーク・地震イベントを記録
- **systemd サービス化**: 起動自動化、クラッシュ時自動再起動
- **自己回復**: センサー初期化失敗時に `RuntimeError` を投げ systemd が再起動

## ハードウェア

| 部品 | 接続 |
|---|---|
| ADXL355 (Analog Devices) | SPI Bus 0, Device 0 |
| ADXL355 DRDY | GPIO22（立ち上がりエッジ） |
| Raspberry Pi | 4B / 5（Bookworm 64bit 想定） |

### 配線

| ADXL355 | Pi 物理ピン | 機能 |
|---|---|---|
| VCC | 1 (3.3V) | 電源 |
| GND | 6 | GND |
| MISO | 21 | SPI0 MISO |
| MOSI | 19 | SPI0 MOSI |
| SCLK | 23 | SPI0 SCLK |
| CS | 24 | SPI0 CE0 |
| DRDY | 15 (GPIO22) | データ準備完了割込み |

`raspi-config` で **SPI を有効化**してください。

## ソフトウェア構成

| ファイル | 役割 |
|---|---|
| `app.py` | Flask + 計測スレッド + STA/LTA + SQLite + SSE |
| `static/index.html` | フロント (Chart.js / SSE クライアント) |
| `diag_pins.py` | センサー診断ツール（SPI ID / レジスタ書込 / アナログ部生存確認） |
| `check_adxl355_spi.py` | 最小限の SPI 通信検証 |
| `mpu.service` | systemd ユニット |
| `requirements.txt` | Python 依存関係 |

## セットアップ

### 1. システムパッケージ

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip libgpiod2 python3-libgpiod
sudo raspi-config nonint do_spi 0  # SPI を有効化
```

### 2. Python 環境

```bash
cd /home/pi
git clone https://github.com/jkudo/adxl355_seismograph.git web
cd web
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
```

### 3. 動作確認

```bash
# センサー診断（SPI 通信・レジスタ書込・アナログ部生存）
python3 diag_pins.py

# 単体起動
python3 app.py
```

ブラウザで `http://<Pi の IP>:8000/` を開く。

### 4. systemd サービス化

```bash
sudo cp mpu.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now mpu.service
sudo systemctl status mpu.service
journalctl -u mpu.service -f
```

## 主要パラメータ（app.py）

| 定数 | 値 | 意味 |
|---|---|---|
| `RANGE_BITS` | `0x01` | 計測レンジ ±4 g |
| ODR | 125 Hz | サンプリング周波数 |
| LPF | 31 Hz | ローパスフィルタ |
| `STA_LEN` | 625 (5 秒) | 短期平均ウィンドウ |
| `LTA_LEN` | 7500 (60 秒) | 長期平均ウィンドウ |
| `STALTA_ON` | 2.5 | 地震検知トリガー閾値 |
| `STALTA_OFF` | 1.3 | デトリガー閾値 |
| `QUAKE_MIN_DUR` | 3.0 秒 | 地震確定の最小継続時間 |
| `RECAL_TAU` | 300 秒 | 自動再キャリブレーション EMA 時定数 |
| `RECAL_THRESHOLD` | 5.0 gal | 自動再キャリブ閾値 |

## 震度換算（簡易）

| gal (合成最大) | 推定震度 |
|---|---|
| ≥ 400 | 7 |
| ≥ 315 | 6 強 |
| ≥ 250 | 6 弱 |
| ≥ 140 | 5 強 |
| ≥ 80 | 5 弱 |
| ≥ 25 | 4 |
| ≥ 8 | 3 |
| ≥ 2.5 | 2 |
| ≥ 0.8 | 1 |

※ 単一観測点・簡易換算のため気象庁発表値とは厳密には一致しません。

## HTTP エンドポイント

| メソッド | パス | 用途 |
|---|---|---|
| GET | `/` | UI (index.html) |
| GET | `/stream_ui` | SSE ライブストリーム |
| GET | `/history?window=5m&interval=0` | 履歴データ JSON |
| GET | `/history?start=ms&end=ms&interval=N` | 範囲指定履歴 |
| GET | `/events` | 過去の地震イベント一覧 |
| POST | `/recalibrate` | 手動再キャリブレーション要求 |
| GET | `/recal_status` | 再キャリブ状態確認 |

## 運用 Tips

- **接続が増えるとハングする場合**: `mpu.service` の `--threads` を増やす（既定 64）。SSE 接続 1 本につき 1 スレッド占有するため。
- **数値が出ない（全 0）**: `python3 diag_pins.py` でセンサー生存確認。Pi 電源を一度抜いて入れ直すと多くの場合復活します。
- **長周期地震を確実に拾いたい**: `STALTA_ON` を 2.0 まで下げると感度向上（誤検知も増える）。
- **デバッグ表示**: ブラウザコンソールで `localStorage.dbg='1'` してリロード。

## ライセンス

MIT

## 謝辞

- Analog Devices ADXL355 データシート
- 気象庁 計測震度算出方法
- Chart.js / chartjs-adapter-date-fns
