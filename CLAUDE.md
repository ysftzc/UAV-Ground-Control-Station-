# UAV Ground Control Station — Claude Code Master Briefing
> Bu dosyayı tamamen oku. Bu proje hakkında her şey burada.

---

## 1. PROJENİN AMACI VE HİKAYESİ

Bu proje, savunma sanayii (ASELSAN, Roketsan, Baykar, TAI) iş başvurusu için geliştirilmekte olan
portfolyo projesidir. Yusuf Tuzcu (EEE mezunu, IHA-1 UAV Pilot Lisanslı) tarafından geliştirilmektedir.

Projenin temel fikri: STM32 kartını fiziksel olarak eğdiğinde, gerçek IMU sensör verisi MAVLink
protokolüyle PX4 SITL'e beslenir ve Gazebo'daki sanal drone aynı hareketi yapar. Bu hardware-in-the-loop
demo videosu Baykar/ASELSAN mülakatında gösterilecek.

Bu mimari, Baykar TB2 ve TAI Aksungur gibi gerçek savunma sistemlerinin kullandığı
stack'in aynısıdır: FreeRTOS + CAN Bus + MAVLink + GCS.

---

## 2. TAM SİSTEM MİMARİSİ (4 KATMAN)

```
┌─────────────────────────────────────────────────────────────┐
│              KATMAN 1: STM32F103 — Donanım                  │
│                                                             │
│  [IMU_TASK]     [BARO_TASK]    [MAG_TASK]    [WDG_TASK]    │
│  MPU6050        BMP180         HMC5883L      Heap+LED       │
│  50ms/I2C       500ms/I2C      100ms/I2C     1000ms         │
│       │              │              │                        │
│       └──────────────┴──────────────┘                       │
│                      │ xQueueSend → xIMUQueue/xBaroQueue    │
│              [CAN_TX_TASK]  ← xSemaphoreTake(xCANMutex)    │
│         CAN Bus 500kbps / ID: 0x101–0x10F                   │
└──────────────────────┬──────────────────────────────────────┘
                       │ MAVLink encoding → UART (CP2102 USB)
┌──────────────────────▼──────────────────────────────────────┐
│              KATMAN 2: PX4 SITL — Uçuş Kontrolü            │
│                                                             │
│  EKF2 Sensör Füzyonu (IMU+Baro+Mag) → Durum tahmini        │
│  Uçuş modları: Offboard / Mission / Stabilize               │
│  Mixer/PWM → Motor sürücü çıkışı                            │
│  Gazebo physics simulation → sanal drone uçuşu              │
└──────────────────────┬──────────────────────────────────────┘
                       │ uXRCE-DDS bridge
┌──────────────────────▼──────────────────────────────────────┐
│              KATMAN 3: ROS2 Jazzy — Yazılım                 │
│                                                             │
│  px4_ros2 interface → topic bridge                          │
│  Nav2 → waypoint planlama ve takip                          │
│  Kalman node → sensör füzyonu + anomali tespiti             │
│  GCS bridge → WebSocket üzerinden dashboard'a veri          │
└──────────────────────┬──────────────────────────────────────┘
                       │ WebSocket + CSV log
┌──────────────────────▼──────────────────────────────────────┐
│              KATMAN 4: Python GCS Dashboard                  │
│                                                             │
│  Sol panel: Attitude HUD (pitch/roll/yaw), telemetri        │
│  Orta üst: Harita + waypoint takibi + drone ikonu           │
│  Orta alt: İrtifa/hız/ivme canlı sparkline grafikleri       │
│  Alt: CAN Bus frame monitörü (canlı hex dump)               │
│  Sağ panel: FreeRTOS task monitörü (RUN/BLK/CPU%)          │
│             Batarya, RF link kalitesi                        │
│             Alarm geçmişi (NORMAL/DİKKAT/ALARM)            │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. DONANIM BİLGİLERİ

### STM32F103C8T6 Blue Pill
- Flash: 128KB, RAM: 20KB
- Dahili CAN kontrolcüsü (ekstra chip gerekmez)
- Dahili LED: PC13 (AKTİF LOW — GPIO_PIN_RESET = yanar)
- Programlama: OpenOCD + klon ST-Link V2

### Pin Haritası
```
PA9  (USART1_TX) → CP2102 RXD (MAVLink çıkışı)
PA10 (USART1_RX) → CP2102 TXD
PA11 (CAN_RX)    → SN65HVD230 RX
PA12 (CAN_TX)    → SN65HVD230 TX
PB6  (I2C1_SCL)  → GY-87 SCL
PB7  (I2C1_SDA)  → GY-87 SDA
PC13 (GPIO_OUT)  → Dahili LED (aktif LOW)
```

### Sensörler — GY-87 10DOF Modülü
- MPU6050: 3 eksen ivme + 3 eksen jiroskop (I2C addr: 0x68)
- BMP180: Barometre + sıcaklık (I2C addr: 0x77)
- HMC5883L: 3 eksen manyetometre/pusula (I2C addr: 0x1E)
- Durum: Sipariş verildi, henüz gelmedi

### CAN Bus Transceiver — SN65HVD230
- 3.3V ile çalışır (STM32 ile direk uyumlu)
- Bağlantı: A11→RX, A12→TX, 3.3V→VCC, GND→GND
- 120Ω sonlandırma dirençleri her iki uca bağlanacak

### Programlayıcı — Klon ST-Link V2
- Flash komutu (BOOT0=1 yapıp reset sonrası):
```bash
cd /home/yusuf/stm32_ws/uav_gcs_node/Debug
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "adapter speed 100" -c "program uav_gcs_node.elf verify reset exit"
```
- Flash sonrası BOOT0=0, reset bas

---

## 4. YAZILIM MİMARİSİ — STM32 TARAFI

### FreeRTOS Task Yapısı
```c
// Task öncelikleri (yüksek = önce çalışır)
IMU_TASK    → öncelik 4, 50ms,   MPU6050 I2C → xIMUQueue
BARO_TASK   → öncelik 3, 500ms,  BMP180  I2C → xBaroQueue
MAG_TASK    → öncelik 3, 100ms,  HMC5883 I2C → xMagQueue
CAN_TX_TASK → öncelik 3, event,  Queue'dan al → CAN Bus (Mutex)
WDG_TASK    → öncelik 1, 1000ms, Heap monitör + PC13 LED toggle
```

### Queue ve Mutex Yapısı
```c
xIMUQueue  = xQueueCreate(10, sizeof(IMU_Data_t));   // 10 elemanlık
xBaroQueue = xQueueCreate(5,  sizeof(Baro_Data_t));  // 5 elemanlık
xMagQueue  = xQueueCreate(5,  sizeof(Mag_Data_t));   // 5 elemanlık
xCANMutex  = xSemaphoreCreateMutex();                // CAN Bus koruması
```

### Veri Struct'ları
```c
typedef struct {
    float accel_x, accel_y, accel_z;  // g cinsinden
    float gyro_x, gyro_y, gyro_z;     // derece/saniye
} IMU_Data_t;

typedef struct {
    float pressure;     // hPa
    float temperature;  // Celsius
    float altitude;     // metre
} Baro_Data_t;

typedef struct {
    float mag_x, mag_y, mag_z;  // mikro Tesla
} Mag_Data_t;
```

### CAN Bus Mesaj Formatı
```
ID: 0x101 | DLC: 8 | [ax_H][ax_L][ay_H][ay_L][az_H][az_L][00][00]
ID: 0x102 | DLC: 4 | [pressure_H][pressure_L][temp_H][temp_L]
ID: 0x103 | DLC: 6 | [mx_H][mx_L][my_H][my_L][mz_H][mz_L]

Encoding: float → int16 (×100 çarpanı ile, örn: 9.81g → 981)
```

---

## 5. MEVCUT KOD DURUMU

### Tamamlanan ✅
- FreeRTOS task iskeleti (freertos.c) — derleniyor, flash edildi
- CAN Bus loopback modu aktif ve çalışıyor
- CAN filter + HAL_CAN_Start + HAL_CAN_AddTxMessage implementasyonu
- SN65HVD230 fiziksel bağlantısı yapıldı
- OpenOCD ile başarılı flash (Verified OK)
- GitHub reposu aktif: https://github.com/ysftzc/UAV-Ground-Control-Station-

### Devam Eden 🔄
- Sahte IMU verisi kullanılıyor (accel_z = 9.81f hardcoded)
- PC13 LED toggle yazıldı ama test edilmedi
- GY-87 sensör bekleniyor

### Sıradaki Adımlar 📋
1. GY-87 gelince MPU6050 I2C sürücüsü yaz
2. BMP180 I2C sürücüsü yaz
3. HMC5883L I2C sürücüsü yaz
4. Sahte veriyi gerçek sensör verisiyle değiştir
5. UART üzerinden debug çıktısı ekle (CP2102 ile test)
6. MAVLink frame encoding yaz
7. Python tarafında pymavlink ile veri al
8. PX4 SITL kur ve ilk sanal uçuş yap
9. STM32 MAVLink → PX4 SITL entegrasyonu
10. ROS2 katmanı (px4_ros2, Nav2, Kalman node)
11. Python GCS dashboard (rich kütüphanesi)
12. Demo video: fiziksel STM32 hareketi → Gazebo drone hareketi

---

## 6. ÖNEMLİ NOTLAR VE TUZAKLAR

### CubeMX Problemi
CubeMX her "Generate Code" yaptığında freertos.c'ye otomatik olarak
`defaultTaskHandle`, `StartDefaultTask` ve ikinci bir `MX_FREERTOS_Init`
ekliyor. Bu bizim yazdığımız fonksiyonla çakışıyor.

ÇÖZÜM: CubeMX'te FreeRTOS → Tasks sekmesinde defaultTask'ı SİL.
Her generate sonrası freertos.c'yi kontrol et.

### PC13 LED Aktif LOW
Blue Pill'de dahili LED PC13'te, aktif LOW mantıkla çalışır:
- GPIO_PIN_RESET → LED YANAR
- GPIO_PIN_SET   → LED SÖNER
HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13) doğru kullanım.

### ST-Link Bağlantısı
Klon ST-Link V2'de BOOT0=1 yapılmadan flash çalışmıyor.
Flash sırası: BOOT0=1 → reset → openocd komutu → BOOT0=0 → reset

### I2C Pull-up
GY-87 modülünde dahili pull-up var (GY-521 serisi).
Ayrıca 4.7kΩ direnç bağlamak gerekmeyebilir ama güvenlik için eklenebilir.

### Workspace Yolu
/home/yusuf/stm32_ws/uav_gcs_node/

---

## 7. GELİŞTİRİCİ PROFİLİ

**Yusuf Tuzcu**
- Elektrik Elektronik Mühendisliği, Eskişehir Osmangazi Üniversitesi
- IHA-1 UAV Pilot Lisansı (SHGM, 2026)
- TUBITAK 2209-B projesi: Otonom sera hasat robotu (ROS2+Gazebo+YOLO, %79.2 başarı)
- Forvia Faurecia stajı: PLC programlama, endüstriyel otomasyon
- Hedef: ASELSAN, Roketsan, Baykar, TAI

**GitHub:** https://github.com/ysftzc
**Repo:** https://github.com/ysftzc/UAV-Ground-Control-Station-

---

## 8. PYTHON GCS DASHBOARD HEDEFİ

Terminal tabanlı, askeri HMI + SCADA karışımı görünüm:
- Koyu arka plan (#0a0c0f), yeşil (#00e676) ana renk
- `rich` kütüphanesi ile canlı panel güncellemesi
- WebSocket üzerinden ROS2'den veri alır
- Bölümler:
  * Üst bar: Sistem adı, drone ID, mod, uçuş süresi, bağlantı durumu
  * Sol panel: Attitude (pitch/roll/yaw barları), telemetri tablosu
  * Orta üst: ASCII harita + waypoint rotası
  * Orta alt: Sparkline grafikler (irtifa, hız, ivme)
  * Alt: CAN Bus hex dump (canlı)
  * Sağ panel: FreeRTOS task monitörü, batarya, alarm log

---

## 9. KOMUTLAR REFERANSI

```bash
# Flash
cd /home/yusuf/stm32_ws/uav_gcs_node/Debug
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "adapter speed 100" -c "program uav_gcs_node.elf verify reset exit"

# UART monitor (CP2102 bağlıyken)
sudo minicom -D /dev/ttyUSB0 -b 115200

# Git push
cd /home/yusuf/stm32_ws/uav_gcs_node
git add .
git commit -m "feat: ..."
git push

# PX4 SITL (kurulunca)
cd ~/PX4-Autopilot
make px4_sitl gazebo

# ROS2 (kurulunca)
source /opt/ros/jazzy/setup.bash
```
