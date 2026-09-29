ARGOS Robot Control Platform
============================

Wi-Fi ile calistirma:
1. wifi_secrets.example.h dosyasini wifi_secrets.h olarak kopyala.
2. Ag bilgilerini ve fallback AP parolasini yerel secrets dosyasinda ayarla.
3. ESP32-C3'e esp32_c3_argos_bridge sketch'ini yukle.
4. Bilgisayari ESP ile ayni aga veya yapilandirdigin ARGOS fallback agina bagla.
5. run_argos_control_panel.bat dosyasina cift tikla.
6. Panel ARGOS koprusunu otomatik olarak bulur ve baglanir.

USB/COM ile servis:
1. Panelde Link Mode alanini COM yap.
2. STLink Virtual COM Port'u sec ve CONNECT'e bas.

Komutlar:
- SCAN I2C    : s
- MPU READ    : m
- MPU STREAM  : M
- CAL GYRO    : c
- TOF ASSIGN  : T
- TOF CHECK   : t
- TOF STREAM  : D
- LEFT TOF    : L
- RIGHT TOF   : R
- DRIVE       : P left right, ornek P 25 -25
- STOP        : x
- FWD 25      : f
- BACK 25     : b
- TURN LEFT   : <
- TURN RIGHT  : >
- MOTOR STAT  : E
- GPIO TEST   : g
- RUN ALL     : a
- MENU        : h

Baglantilar:
- I2C1 SCL PB8, SDA PB9
- MPU6050 INT PB10, AD0 GND, adres 0x68
- VL53L0X Left XSHUT PA8, INT PC7, atanmis adres 0x30
- VL53L0X Right XSHUT PA9, INT PA10, atanmis adres 0x31
- Tum sensorler 3.3V ve ortak GND
- ToF geometri: sensorler arasi 70 mm, sol -15 derece, sag +15 derece,
  ikisi de yaklasik 5 derece yukari bakiyor
- ToF stream periyodu yaklasik 500 ms'dir; iki sensor arasi okumada 60 ms
  bosluk birakilir.
- Left motor BTS7960 RPWM PC6, LPWM PC8
- Right motor BTS7960 RPWM PA0, LPWM PA1
- Left encoder A PC4, B PC5
- Right encoder A PC2, B PC3
- OLED VCC 3V3, GND GND, SCL D12/PA6, SDA D11/PA7
- ESP RX Nucleo D1/PA2, ESP TX Nucleo D0/PA3, UART 115200 8N1
- ESP fallback Wi-Fi SSID ARGOS; parola yerel wifi_secrets.h dosyasinda ayarlanir.
- Varsayilan fallback adresi 192.168.4.1, TCP portu 3333'tur.
- D0/D1 ESP kullanimi icin Nucleo SB13/SB14 OFF, SB62/SB63 ON olmalidir.

Not:
Wi-Fi baglanmiyorsa bilgisayarin ARGOS aginda oldugunu ve ESP'nin acik
oldugunu kontrol et. Seri port listelenmiyorsa Windows Aygit Yoneticisi > Baglanti Noktalari
(COM ve LPT) altinda STLink Virtual COM Port gorunmeli.

pyserial eksikse:
pip install pyserial
