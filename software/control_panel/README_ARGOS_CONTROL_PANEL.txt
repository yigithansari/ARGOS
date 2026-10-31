ARGOS Robot Control Platform
============================

Wi-Fi ile calistirma:
Normal kullanim: wifi_secrets.h ile ayarlanan hotspot'u ac; ESP STA adresi
DHCP ile degisebilir. Panel AUTO kesfi UDP 3334 kullanir. Asagidaki ARGOS
SSID ve 192.168.4.1 adimlari fallback AP icindir.
1. ESP32-C3'e ESP32/esp32_c3_argos_bridge sketch'ini yukle.
2. Bilgisayari hotspot agina veya yerel secrets dosyanda parolasini belirledigin ARGOS fallback agina bagla.
3. run_argos_control_panel.bat dosyasina cift tikla.
4. Panel 192.168.4.1:3333 adresine otomatik baglanir.

USB/COM ile servis:
1. Panelde Link Mode alanini COM yap.
2. STLink Virtual COM Port'u sec ve CONNECT'e bas.
Not: SB13/SB14 OFF olan mevcut ESP-UART baglantisinda ST-LINK VCP
STM32 USART2'ye bagli degildir; portun gorunmesi UART iletisimini kanitlamaz.

2026-10-31 yayin kopyasi:
- STM32 WIRELESS UPDATE icin yalniz ARGOS_OTA.bin secilir.
- Dosya secicide firmware/argos_f446re/Debug/ klasorune git; binary dosyalar
  bu kaynak agacina eklenmez, yerel derlemeden elde edilir.
- Multipart aktarim ve OTA vektor denetimi aktiftir.
- 25/35/45 RPM tek kosu taramasi yapildi; tekrarlanabilir PI kabul ve zemin
  ayari tamamlanmadi. Son durum icin docs/08_phase_02_motion_control.md.

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
- WHEEL RPM   : V left_rpm right_rpm, ornek V 35 35 (kapali cevrim, max 55)
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
- ESP fallback Wi-Fi SSID ARGOS; parola yerel wifi_secrets.h dosyasinda ayarlanir, TCP 192.168.4.1:3333
- D0/D1 ESP kullanimi icin Nucleo SB13/SB14 OFF, SB62/SB63 ON olmalidir.

Not:
Wi-Fi baglanmiyorsa bilgisayarin ARGOS aginda oldugunu ve ESP'nin acik
oldugunu kontrol et. Seri port listelenmiyorsa Windows Aygit Yoneticisi > Baglanti Noktalari
(COM ve LPT) altinda STLink Virtual COM Port gorunmeli.

pyserial eksikse:
pip install pyserial

Phase 02 hiz kontrolu:
- Panelde Drive Mode alanindan PWM veya RPM secilir.
- Ilk RPM testi tekerler havadayken 25 RPM ile yapilmalidir.
- RPM modunda hedef, olculen teker RPM'i, PI integral katkisi ve uygulanan PWM
  motor telemetri alaninda gorunur.
- STOP komutu cikisi hemen sifirlar; motor ve teker mekanigi bir miktar serbest
  yuvarlanabilir.

STM32 kablosuz guncelleme:
- Ilk kurulumda ESP sketch'i USB ile, Debug/ARGOS_BOOTSTRAP.bin ise ST-LINK ile
  STM32 adres 0x08000000'a bir kez yuklenir.
- Sonraki STM32 uygulama guncellemelerinde panelde STM32 WIRELESS UPDATE secilir
  ve Debug/ARGOS_OTA.bin dosyasi gonderilir.
- Panel dosyayi ESP'nin 3335 numarali guncelleme servisine yollar; ESP dosyayi
  tamamen onbellege alip CRC32 dogrulamadan STM32 flash'ini silmez.
- Guncelleme boyunca motorlar STOP durumunda tutulur. Islem kesilirse bootloader
  kurtarma modunda kalir ve ayni dosya tekrar gonderilebilir.
- ARGOS_OTA.bin 0x08008000 icin baglanmistir; ilk kurulumda 0x08000000'a tek
  basina yazilmaz.
