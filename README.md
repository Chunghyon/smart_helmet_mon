# Connected Node Recovery

The 4 MB OTA partition table includes a dedicated `state` NVS partition for Wi-SUN nodes that have produced a live packet. Cell, fire, fall, and sensor status values are not persisted. Mapping changes are batched and saved every 10 seconds. On boot, previously connected cells start as `COM`, the gateway restores their key/IPv6 mappings, and each cell is status-checked after the Wi-SUN link is confirmed. A later live packet replaces `COM` with the current status.

The first deployment of a build using this layout must flash the partition table (or perform a full flash). An application-only OTA update does not install the new `state` partition:

```bash
./tools/idf-wrapper -p /dev/ttyUSB0 flash
```

# Breath Detection — ESP-IDF (VS Code, Arduino as a Component)

이 프로젝트는 **ESP-IDF + Arduino-as-Component** 구조로 변환된 버전입니다.
VS Code의 **Espressif IDF** 확장을 사용해 바로 열고 빌드/플래시할 수 있습니다.

## 준비물
- VS Code + **Espressif IDF** 확장(툴체인 설치 포함)
- ESP-IDF v5.x 권장

## 열기 / 설정
1) VS Code에서 이 폴더를 엽니다.
2) 명령 팔레트(⇧⌘P / Ctrl+Shift+P) → **ESP-IDF: Set Espressif Device Target** → `esp32` (혹은 `esp32c3`, `esp32s3` 등)
3) 터미널에서 한 번 다음을 실행하여 Arduino 컴포넌트를 내려받습니다.
   ```bash
  idf.py add-dependency "espressif/arduino-esp32@^3.3.2"
   ```
   (이미 `main/idf_component.yml`에 선언되어 있어 첫 빌드 시 자동으로 받아옵니다.)

### Ubuntu → macOS로 복사한 경우
- `.vscode/settings.json`의 Ubuntu 절대경로(` /home/... `)는 제거되어 있습니다.
- 시리얼 포트는 macOS 형식으로 설정해야 합니다. 현재 기본값은 `/dev/cu.usbserial-0001` 입니다.
- 실제 포트 확인:
  ```bash
  ls /dev/cu.*
  ```
  확인된 포트를 VS Code 설정의 `idf.port`로 바꾸세요.
- `tools/idf-wrapper`는 아래 순서로 ESP-IDF를 자동 탐색합니다.
  1. `IDF_PATH_OVERRIDE`
  2. `IDF_PATH`
  3. `$HOME/esp/esp-idf`, `$HOME/esp-idf`, `$HOME/.espressif/frameworks/esp-idf`

## 빌드/플래시/모니터
```bash
idf.py set-target esp32   # 보드에 맞게 변경 (esp32 / esp32c3 / esp32s3 등)
idf.py build
idf.py -p <PORT> flash monitor
```
- 기본 시리얼 속도: **115200** (`sdkconfig.defaults`에서 변경 가능)

## BLE GATT 설정 메뉴
펌웨어는 Nordic UART Service(NUS) 형식의 BLE GATT serial bridge를 제공합니다.
BLE 앱에서 장치명 `BRD_<routerNumber>`로 연결한 뒤 RX characteristic에 다음 메뉴 명령을 보냅니다.

- Service: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- RX(write): `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
- TX(notify): `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
- `/` 메뉴 진입 후 `w` 선택
- `sta` 선택: 업링크 Wi-Fi SSID, 비밀번호, IP 설정 입력
- `ap` 선택: 장치가 제공하는 AP SSID와 비밀번호 입력

AP 비밀번호는 ESP32 SoftAP 제한에 따라 8자 이상이어야 하며, 입력한 AP SSID/비밀번호는 NVS에 저장되어 재부팅 후에도 유지됩니다.

## 외부 라이브러리(Arduino)
스케치에서 사용한 라이브러리:
- Adafruit MLX90614 (헤더: `Adafruit_MLX90614.h`)
- arduinoFFT (헤더: `arduinoFFT.h`)

ESP-IDF의 "Managed Components"에는 Arduino 코어가 제공되지만,
**일반 Arduino 라이브러리(Adafruit MLX90614, arduinoFFT)**는 자동으로 포함되지 않습니다.
다음 두 가지 방식 중 하나를 사용하세요.

### 방법 A) components/ 폴더에 벤더링
1. `components/Adafruit_MLX90614` 폴더를 만들고, 해당 라이브러리 소스(.h/.cpp) 파일을 넣습니다.
2. `components/Adafruit_MLX90614/CMakeLists.txt`에 다음과 같이 등록합니다.
   ```cmake
   idf_component_register(SRCS "Adafruit_MLX90614.cpp"
                          INCLUDE_DIRS "."
                          REQUIRES arduino)
   ```
3. `components/arduinoFFT`도 동일한 방식으로 추가합니다.

### 방법 B) Arduino-ESP32 코어 내부 라이브러리로 추가
- Arduino 코어에 라이브러리를 추가해 빌드에 포함되도록 할 수 있습니다.
- 단, ESP-IDF + Arduino-as-Component 구조에서는 **방법 A**를 권장합니다.

## 코드 구성
- `main/arduino_sketch.cpp`: 기존 Arduino `.ino` 코드를 C++로 변환해 포함했습니다.
  - 상단에 `#include "Arduino.h"`를 추가했습니다.
  - 기존 `setup()`/`loop()`는 그대로 유지됩니다.
- `main/app_main.cpp`: ESP-IDF의 엔트리 → `initArduino()` 후 `setup()`/`loop()`를 호출합니다.
- `main/idf_component.yml`: `espressif/arduino-esp32` 의존성 선언(자동 다운로드).
- `sdkconfig.defaults`: 시리얼 속도/BT/Wi-Fi 등의 기본 설정.

## 보드별 주의
- `BluetoothSerial.h`는 **ESP32/ESP32-S3의 Bluetooth Classic** 환경에서 사용합니다.
  - ESP32-C3는 Bluetooth Classic 미지원(LE만) → C3 사용 시 해당 코드 분기/비활성 필요합니다.
- `esp_wifi.h`/Wi-Fi 기능은 ESP-IDF 설정에서 활성화되어야 합니다(기본 활성).
- 핀맵은 기존 코드의 정의를 따릅니다. 보드가 바뀌면 핀 정의 확인이 필요합니다.

## 트러블슈팅
- 처음 빌드에서 Arduino 컴포넌트 다운로드가 안 될 경우:
  ```bash
  idf.py add-dependency "espressif/arduino-esp32@^3.3.2"
  ```
- 외부 라이브러리(Adafruit MLX90614 / arduinoFFT)가 없어서 컴파일 오류가 나면
  위의 **방법 A**로 `components/`에 추가하세요.

- macOS에서 Monitor 창에 키 입력이 안 먹는 것처럼 보일 때:
  - VS Code 터미널 포커스가 Monitor 터미널에 있는지 확인
  - 한글 입력기 대신 영문(ABC) 입력기로 전환 후 입력
  - 기본 Monitor 대신 `ESP-IDF: Monitor (raw)` 작업 사용
  - 이 프로젝트 메뉴 입력은 단일 문자(`d`, `n`, `i`, `h`, `s`, `q`, `/`, `?`) 기준으로 동작

