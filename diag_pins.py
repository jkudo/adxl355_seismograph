import spidev, time
spi = spidev.SpiDev(); spi.open(0,0); spi.max_speed_hz=1_000_000; spi.mode=0
def w(r,v): spi.xfer2([(r<<1)&0xFE, v&0xFF])
def r(reg): return spi.xfer2([(reg<<1)|1, 0])[1]

print('=== ADXL355 ピン/通信診断 ===\n')

# 1. SPIデジタル通信
print('[1] SPIデジタル通信 (DEVID読み取り)')
d_ad, d_mst, partid, revid = r(0x00), r(0x01), r(0x02), r(0x03)
print(f'  DEVID_AD=0x{d_ad:02X} (期待0xAD) {"OK" if d_ad==0xAD else "NG"}')
print(f'  DEVID_MST=0x{d_mst:02X} (期待0x1D) {"OK" if d_mst==0x1D else "NG"}')
print(f'  PARTID=0x{partid:02X} (期待0xED) {"OK" if partid==0xED else "NG"}')
print(f'  REVID=0x{revid:02X}')
print(f'  → MOSI/MISO/SCLK/CS/3.3V/GND の最低限の接続 {"OK" if d_ad==0xAD else "NG"}\n')

# 2. ハードリセット後の状態
w(0x2F, 0x52); time.sleep(0.1)
print('[2] ハードリセット後のレジスタ初期値')
print(f'  STATUS(0x04)=0x{r(0x04):02X}')
print(f'  POWER_CTL(0x2D)=0x{r(0x2D):02X} (リセット直後は 0x01=standby が正常)')
print(f'  RANGE(0x2C)=0x{r(0x2C):02X} (リセット直後は 0x81 が正常)')
print(f'  FILTER(0x28)=0x{r(0x28):02X}')
print(f'  SELF_TEST(0x2E)=0x{r(0x2E):02X}\n')

# 3. レジスタ書込ストレステスト
print('[3] POWER_CTL書込×5回ストレステスト (0x00=measurement に切替)')
ok=0
for i in range(5):
    w(0x2D, 0x00); time.sleep(0.02)
    rb = r(0x2D)
    res = "OK" if rb==0x00 else "NG"
    if rb==0: ok+=1
    print(f'  attempt {i+1}: readback=0x{rb:02X} {res}')
print(f'  成功率: {ok}/5\n')

# 4. アナログ部の生存確認
print('[4] アナログ部生存確認 (TEMP & XYZ)')
# 確実にmeasurementモードへ
for _ in range(5):
    w(0x2D, 0x00); time.sleep(0.02)
    if r(0x2D)==0: break
time.sleep(0.2)
t = (r(0x06)<<8) | r(0x07)
print(f'  TEMP raw=0x{t:04X} ({t}) ※室温で 1500-2500 程度が正常、0 はアナログ電源異常')
nz = 0
for i in range(5):
    time.sleep(0.05)
    rx_x = spi.xfer2([(0x08<<1)|1,0,0,0])
    rx_y = spi.xfer2([(0x0B<<1)|1,0,0,0])
    rx_z = spi.xfer2([(0x0E<<1)|1,0,0,0])
    if any(rx_x[1:]) or any(rx_y[1:]) or any(rx_z[1:]): nz+=1
    print(f'  X bytes={rx_x[1:]}  Y bytes={rx_y[1:]}  Z bytes={rx_z[1:]}')
print(f'  非ゼロデータ: {nz}/5')

if t == 0 and nz == 0:
    print('\n=== 判定: アナログ部死亡 ===')
    print('  → 3.3V電源 / GND の接触不良、またはセンサー本体の故障')
elif d_ad != 0xAD:
    print('\n=== 判定: SPI通信不良 ===')
    print('  → MOSI/MISO/SCLK/CS の配線を確認')
else:
    print('\n=== 判定: 復活！正常動作 ===')

spi.close()
