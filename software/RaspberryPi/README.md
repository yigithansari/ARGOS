# ARGOS Raspberry Pi

Pi, STM32 UART trafiğini masaüstü paneline taşır. Sürüş, motor STOP, RPM
hedefi, enkoder, sensör ve kalibrasyon mevcut panelle aynı akışı kullanır.
Panel canlı görünümü ve keşif kararlarını yürütür. **EXPORT / REPLAY** ile
JSONL kaydı Pi'ye gider. Pi nokta haritasını hesaplar ve SVG ile JSON
özetini panele döndürür. Dosyalar PC'de kaydın yanında tutulur.

## Bağlantı

Pi ve robot kapalıyken ESP'nin UART bağlantısını ayır. Pi'yi kendi güç
kaynağından besle. Üç bağlantı yeterli.

| Raspberry Pi 5 | STM32 |
| --- | --- |
| Fiziksel pin 8, GPIO14 TX | PA3 RX, eski ESP GPIO21 hattı |
| Fiziksel pin 10, GPIO15 RX | PA2 TX, eski ESP GPIO20 hattı |
| Fiziksel pin 6, GND | Ortak GND |

UART 3,3 V ve 115200 8N1 kullanır. Pi'nin daha önce doğrulanan
`dtoverlay=uart0-pi5` ayarı ile aygıt `/dev/ttyAMA0`. Bu UART üzerinde
oturum açma konsolu veya başka bir seri program çalışıyorsa kapat.

## SSH ile kurulum ve çalıştırma

Pi ve Windows'u aynı Wi-Fi ağına bağla. Pi'de SSH kapalıysa
`sudo raspi-config` içinden **Interface Options → SSH** seç. Pi'nin adresi
için `hostname -I`, kullanıcı adı için `whoami` çalıştır. Aşağıdaki
`kullanici@PI_IP` yerlerini bu değerlerle değiştir.

Windows PowerShell'de, bu projenin kök klasöründen bir kez çalıştır.
İlk SSH bağlantısında Pi'nin anahtarını doğrula. Bu bağlantı masaüstü
panelinin daha sonra sunucu kimliğini denetleyebilmesini sağlar.

```powershell
ssh kullanici@PI_IP "mkdir -p ~/argos/control_panel ~/argos/RaspberryPi"
scp software/control_panel/argos_control_panel.py software/control_panel/argos_exploration.py software/control_panel/argos_odometry.py software/control_panel/argos_acceptance.py software/control_panel/argos_motion_calibration.json kullanici@PI_IP:argos/control_panel/
scp software/RaspberryPi/bridge.py software/RaspberryPi/desktop.py software/RaspberryPi/install-pi.sh kullanici@PI_IP:argos/RaspberryPi/
ssh -t kullanici@PI_IP "sed -i 's/\r$//' ~/argos/RaspberryPi/install-pi.sh && bash ~/argos/RaspberryPi/install-pi.sh"
ssh -t kullanici@PI_IP "sudo reboot"
```

Kurulum köprüyü Pi açılışında çalışacak şekilde ayarlar ve Pi masaüstüne
**ARGOS** kısayolunu ekler. Kurulum gerekiyorsa sudo parolasını sorar.

### Pi Connect

[Raspberry Pi Connect](https://connect.raspberrypi.com/) üzerinden **Screen
sharing** aç ve Pi masaüstündeki **ARGOS** simgesine çift tıkla. İlk
açılışta çalıştırma onayı gelirse **Execute** seç. Bu kullanımda ayrıca
SSH penceresi gerekmez. Ekran paylaşımı için Pi'de açık bir grafik oturumu
gerekir. [Pi Connect belgesi](https://www.raspberrypi.com/documentation/services/connect.html)

### Windows'tan tek tık

Windows OpenSSH ile Pi'ye bir kez anahtar tanıt. Kullanılabilir bir
`id_ed25519.pub` dosyan varsa anahtar üretme adımını geç. Parolalı SSH
anahtarı kullanırsan Windows `ssh-agent` ile oturum başına anahtarı yükle.

```powershell
ssh-keygen -t ed25519
Get-Content "$env:USERPROFILE\.ssh\id_ed25519.pub" | ssh kullanici@PI_IP 'umask 077; mkdir -p ~/.ssh; sed s/\\r$// >> ~/.ssh/authorized_keys'
ssh -o BatchMode=yes kullanici@PI_IP "echo ARGOS-SSH-OK"
powershell -File software/RaspberryPi/new-windows-shortcut.ps1 -Pi kullanici@PI_IP
```

Windows masaüstündeki **ARGOS Pi** kısayolu paneli ve şifreli bağlantıyı
birlikte açar. Alternatif olarak paneli doğrudan çalıştırabilirsin.
Windows'ta `python` farklı bir kuruluma işaret ediyorsa kısayol komutuna
`-Python "C:\...\python.exe"` ile Tkinter bulunan Python yolunu ver.
Değişen IP adresleri için erişilebiliyorsa `raspberrypi.local` gibi
Pi'nin sabit ana bilgisayar adını kullan. SSH anahtarını aynı adla tanıt.

```powershell
python software/RaspberryPi/desktop.py --ssh kullanici@PI_IP
```

Bir seferde tek panel robotu kontrol eder. Diğer panele geçmeden veya Pi
Connect ekran paylaşımından çıkmadan önce **STOP** ve **DISCONNECT**
kullan. Pi Connect ekranı kapansa bile Pi'deki uygulama açık kalabilir.
Kontrol bağlantısı kesildiğinde köprü STOP gönderir. UART erişilemezse
STM32'nin kendi watchdog'u geçerlidir. OTA gerektiğinde ESP köprüsüne dön.

## Mesafe, dönüş ve PI

- **DISTANCE / TURN** üzerinden `F` / `B` için **30–150 mm**, `L` / `R`
  için **15–90°** gir. Örneğin `F` ve `100` ile 100 mm ileri komutu verilir.
  Bu ekran STM32'nin mevcut sınırlı hareket komutunu kullanır. Phase 02'nin
  1 metre testi ayrı saha testidir.
- **Drive → RPM** ile tekerlerin ayrı RPM hedeflerini ver. **Phase 02
  Calibration** menüsünde mevcut kalibrasyon ve test akışı kullanılabilir.
  PI kazançları STM32 firmware'inde tanımlıdır.
- **EXPLORE / MAP** ekranından mevcut keşif akışını veya manuel kaydı kullan.
  **EXPORT / REPLAY** ile Pi haritayı işler. Sonuç mevcut ToF ve ultrasonik
  sensörlerle üretilen 2B nokta haritasıdır.

LiDAR eklendiğinde yeni ölçüm kaynağı Pi'deki harita işlemine bağlanabilir.
Sensör seçimi, montaj ve konum eşlemesi sonraki çalışmada ele alınacak.
