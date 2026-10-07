/*
 * Wi-SUN Border Router Gateway
 *
 * 역할: Wi-SUN Border Router 모듈(UART 연결)로부터 수신한 데이터를
 *       Wi-Fi 기반 Web UI/WebSocket으로 모니터링하는 게이트웨이.
 *
 * Wi-SUN Nodes → BR Module → UART → ESP32 (this) → Wi-Fi(Web UI + WebSocket)
 */

#include "Arduino.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include "driver/i2c_master.h"
#include "esp_wifi.h"
#include <SPIFFS.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <Update.h>
#include "esp_http_server.h"
#include "esp_app_desc.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

void setup();
void loop();
String sendCommand(const char* cmd);
void startWebServer();
static void serviceWebServerHealth(unsigned long now);
void notifyWebClients(const char* srcAddr, const char* payload, const char* rawLine, const char* key, const char* statusText, const char* code);
void updateOledStatus();
void registerWatchdogForMainTask();
void serviceWatchdog(bool allowYield = false);
static bool ensureATCommandMode();
static void upsertKeyIp(const char* key, const char* ipv6, bool markLive = true);
static String statusToCode(const char* statusText);
static bool loadRuntimeState();
static bool saveRuntimeState();
static void saveWiSUNConnectionState();
void saveCheckIntervalConfig();

// ============================================================================
//  Wi-SUN UART pins  (ESP32 <-> Wi-SUN BR module)
// ============================================================================
#define RX_PIN     17
#define TX_PIN     16
#define TXON_PIN   12
#define RESETN_PIN 19
#define WAKEUP_PIN 5

// LED pins
#define RED_LED_PIN   26
#define GREEN_LED_PIN 27

// Multicast group (used when replying to field nodes)
#define MULTICAST_ADDRESS_DEFAULT "<ff15::810a:64d1>"
char MULTICAST_ADDRESS[40] = MULTICAST_ADDRESS_DEFAULT;

char globalIPv6Address[40] = "";
char localIPv6Address[40]  = "";

HardwareSerial WiSUNSerial(2);

// ============================================================================
//  Configuration  (NVS)
// ============================================================================
char dong[11]     = "";
char ho[11]       = "";
int  sensorNumber = 1;
char routerNumber[16] = "1";

bool wiSunInitialized = false;
bool isWiSUNConnected = false;
bool isWiSUNSettingUp = false;
bool g_wisunRuntimeReady = false;
volatile bool g_wisun_abort = false;

char wifiMode[5]      = "sta";  // "sta" or "ap"
char wifiStaSSID[33]  = "";
char wifiStaPASS[65]  = "";
char wifiStaticIP[16] = "";
char wifiNetmask[16]  = "255.255.255.0";
char wifiGateway[16]  = "";
char wifiApSSID[33]   = "BRD-GW-AP";
char wifiApPASS[65]   = "12345678";
bool wifiConfigured   = false;

static bool g_dataLoggingEnabled = true;
static bool g_menuActive = false;

char wiSunChrate[16]   = "50kbps";

char wiSunLineBuffer[256];
int  wiSunLineIndex = 0;

unsigned long lastBlinkTime    = 0;
int           blinkState       = 0;
const unsigned long blinkMillisInterval = 500;

Preferences preferences;

static volatile uint8_t g_lastStaDiscReason = 0;
static volatile bool g_wifiOledUpdatePending = false;

static bool hasValidStaIp() {
    if (WiFi.status() != WL_CONNECTED) return false;
    uint32_t ip = static_cast<uint32_t>(WiFi.localIP());
    return ip != 0 && ip != 0xFFFFFFFFUL;
}

static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
        g_lastStaDiscReason = info.wifi_sta_disconnected.reason;
    }
    if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED ||
        event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED ||
        event == ARDUINO_EVENT_WIFI_STA_GOT_IP ||
        event == ARDUINO_EVENT_WIFI_AP_START ||
        event == ARDUINO_EVENT_WIFI_AP_STOP) {
        g_wifiOledUpdatePending = true;
    }
}

// ============================================================================
//  Web UI + WebSocket monitor
// ============================================================================
static httpd_handle_t g_httpServer = nullptr;
static unsigned long g_nextWebRetryMs = 0;
static unsigned long g_lastWebHealthCheckMs = 0;
static unsigned long g_webClientPressureSinceMs = 0;
static int g_lastWebClientCount = -1;
static volatile bool g_webRecoveryRequested = false;
static bool g_webRecoveryActive = false;
static unsigned int g_webStartFailureCount = 0;
static SemaphoreHandle_t g_wsSendMutex = nullptr;

struct AptRow {
    int dong;
    int startHo;
    int endHo;
    int sensorCount;
};

static AptRow g_aptRows[64];
static int g_aptRowCount = 0;
static char g_aptName[64] = "Wi-SUN";
static char g_aptBrIPv6[64] = "";
static char g_aptComPort[32] = "";

struct CellState {
    char key[24];
    char code[8];
    char status[48];
    unsigned long updatedMs;
    unsigned long lastWebPushMs;
    bool awaitingStatusCheck;
    unsigned long statusCheckDeadlineMs;
};

struct FireState {
    char key[24];
    uint8_t regionMask;
};

struct FallState {
    char key[24];
    bool active;
};

static CellState g_cellStates[256];
static int g_cellStateCount = 0;
static FireState g_fireStates[256];
static int g_fireStateCount = 0;
static FallState g_fallStates[256];
static int g_fallStateCount = 0;

extern const char _binary_APT_setup_start[] asm("_binary_APT_setup_start");
extern const char _binary_APT_setup_end[] asm("_binary_APT_setup_end");

static const char* DEFAULT_APT_SETUP =
    "501,101,1101,1\n"
    "501,102,1102,1\n"
    "502,101,1201,1\n"
    "502,102,1202,1\n"
    "503,201,1301,1\n"
    "504,101,2401,1\n"
    "504,102,2402,1\n"
    "Life Monitor Platform(LMP) by JJSystem\n"
    "2020:abcd::212:4b00:2de1:fc9d\n";

struct NodeInfo {
        char ipv6[40];
        unsigned long lastSeenMs;
};

struct KeyIpInfo {
    char key[24];
    char ipv6[40];
    unsigned long lastSeenMs;
    bool everSeen;
};

static NodeInfo g_nodes[32];
static int g_nodeCount = 0;
static KeyIpInfo g_keyIps[256];
static int g_keyIpCount = 0;
static uint32_t g_wsSeq = 0;
static bool g_packetStreamEnabled = false;
static unsigned long g_lastPacketStreamPushMs = 0;
static bool g_keyIpDirty = false;
static unsigned long g_lastKeyIpSaveMs = 0;
static const unsigned long KEYIP_SAVE_INTERVAL_MS = 10000;
static bool g_runtimeStateDirty = false;
static unsigned long g_lastRuntimeStateSaveMs = 0;
static const unsigned long RUNTIME_STATE_SAVE_INTERVAL_MS = 10000;
static unsigned long g_scheduledRestartMs = 0;
static const char* DEFAULT_IP_MAPPING_TABLE =
    "{\n"
    "  \"503_701_1\": {\n"
    "    \"dong_ho\": \"503_701_1\",\n"
    "    \"ipv6\": \"2020:abcd::212:4b00:2de2:9d\",\n"
    "    \"last_communication\": \"2025-07-16 12:36:29\"\n"
    "  },\n"
    "  \"501_101_1\": {\n"
    "    \"dong_ho\": \"501_101_1\",\n"
    "    \"ipv6\": \"2020:abcd::212:4b00:2de1:fcaf\",\n"
    "    \"last_communication\": \"2025-06-06 19:10:01\"\n"
    "  },\n"
    "  \"502_302_1\": {\n"
    "    \"dong_ho\": \"502_302_1\",\n"
    "    \"ipv6\": \"2020:abcd::212:4b00:2de2:9d\",\n"
    "    \"last_communication\": \"2025-06-07 12:42:56\"\n"
    "  },\n"
    "  \"504_2402_1\": {\n"
    "    \"dong_ho\": \"504_2402_1\",\n"
    "    \"ipv6\": \"2020:abcd::212:4b00:2de2:93c\",\n"
    "    \"last_communication\": \"2025-06-17 19:34:05\"\n"
    "  },\n"
    "  \"501_701_1\": {\n"
    "    \"dong_ho\": \"501_701_1\",\n"
    "    \"ipv6\": \"2020:abcd::212:4b00:2de2:9d9\",\n"
    "    \"last_communication\": \"2025-10-15 15:01:55\"\n"
    "  },\n"
    "  \"501_1101_1\": {\n"
    "    \"dong_ho\": \"501_1101_1\",\n"
    "    \"ipv6\": \"2020:abcd::212:4b00:2de1:fcaa\",\n"
    "    \"last_communication\": \"2025-10-15 18:41:28\"\n"
    "  },\n"
    "  \"504_2401_1\": {\n"
    "    \"dong_ho\": \"504_2401_1\",\n"
    "    \"ipv6\": \"2020:abcd::212:4b00:2de2:93c\",\n"
    "    \"last_communication\": \"2025-10-16 09:02:01\"\n"
    "  },\n"
    "  \"501_102_1\": {\n"
    "    \"dong_ho\": \"501_102_1\",\n"
    "    \"ipv6\": \"2020:abcd::212:4b00:2de2:9d9\",\n"
    "    \"last_communication\": \"2026-03-08 11:03:11\"\n"
    "  }\n"
    "}\n";

static unsigned long g_checkIntervalSec = 1800;
static const unsigned long STATUS_CHECK_RESPONSE_TIMEOUT_MS = 3000;
static unsigned long g_nextCheckCycleMs = 0;
static bool g_autoCheckInProgress = false;
static int g_autoCheckIndex = 0;
static unsigned long g_lastCheckSentMs = 0;
static int g_lastCheckCommandCellIndex = -1;

static const char* WEB_INDEX_HTML = R"HTML(
<!doctype html>
<html lang="en">
<head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>Life Monitor Platform(LMP) by JJSystem</title>
    <style>
        :root { --bg:#c7c7c7; --cell:#000; --fg:#d8ebff; --yellow:#ecec00; --line:#8c8c8c; }
        * { box-sizing: border-box; }
        body { margin: 0; font-family: "Noto Sans", "Segoe UI", sans-serif; background: var(--bg); color: #101010; display:flex; justify-content:center; }
        .wrap { width: min(1920px, 100vw); height: min(1024px, 100vh); margin: 0 auto; padding: 8px; overflow: hidden; position: relative; }
        .top { display:flex; justify-content:space-between; align-items:center; margin-bottom:4px; }
        .title { font-size: 18px; font-weight: 700; }
        .meta { font-size: 11px; }
        .mainArea { display:flex; gap:8px; height: calc(100% - 78px); }
        .mainArea.packet-hidden .logPane { display:none; }
        .boardWrap { flex: 1 1 auto; min-width: 0; overflow: auto; border: 1px solid var(--line); background: #d0d0d0; padding: 6px; }
        .logPane { width: min(38vw, 560px); min-width: 360px; display:flex; flex-direction:column; border:1px solid var(--line); background:#fff; }
        .logTitle { font-size: 12px; font-weight: 700; padding: 6px 8px; border-bottom:1px solid #ddd; background:#f3f3f3; display:flex; justify-content:space-between; align-items:center; }
        .logCloseBtn { border:1px solid #bbb; background:#fff; font-size:11px; height:22px; padding:0 8px; cursor:pointer; }
        .board { display: grid; gap: 2px; --cw: 56px; --ch: 38px; --lh: 20px; }
        .cell {
            width: var(--cw); height: var(--ch); background: var(--cell); color: var(--fg);
            border: 1px solid #3f3f3f; display:flex; flex-direction:column;
            align-items:center; justify-content:center; font-size: 10px; line-height: 1.0;
        }
        .cell .ho { font-weight: 700; color: #95c9ff; }
        .cell .code { color: #fff; }
        .cell.mov { background: var(--yellow); color: #000; }
        .cell.fire { background: #d32020; color: #fff; }
        .cell.nob { background: #f08a00; color: #111; }
        .cell.alr { background: #8f1c1c; color: #fff; }
        .cell.stb { background: #2e5d9f; color: #fff; }
        .cell.bpm { background: #1f9d3a; color: #fff; }
        .cell.nhp { background: #8f969e; color: #fff; }
        .cell.comSeen { background: #666; color: #fff; }
        .cell.fallA { background: #d32020; color: #fff; }
        .cell.fallB { background: #ecec00; color: #111; }
        .cell.bpm .ho, .cell.nhp .ho, .cell.fire .ho, .cell.alr .ho, .cell.stb .ho { color:#fff; }
        .cell.fallA .ho { color:#fff; }
        .cell.fallB .ho { color:#111; }
        .cell.brd { background: #2a2a2a; color: #fff; }
        .dongLabel { height: var(--lh); display:flex; align-items:center; justify-content:center; background:#efefef; border:1px solid #aaa; font-weight:700; font-size: 11px; }
        .empty { width: var(--cw); height: var(--ch); }
        .log { flex:1 1 auto; overflow: auto; padding: 6px; font-family: ui-monospace, monospace; font-size: 11px; }
        .statusBar { display:flex; gap:8px; font-size: 11px; margin: 4px 0; }
        .line:last-child { border-bottom: 0; }
        .controls { position:absolute; left:10px; bottom:10px; display:flex; gap:8px; align-items:center; z-index:2; white-space:nowrap; max-width:calc(100% - 20px); overflow-x:auto; }
        .controls button, .controls select { height: 28px; font-size: 12px; }
        #btnDataReset { background:#14532d; color:#fff; border:1px solid #86efac; font-weight:700; flex:0 0 auto; }
        .controls input[type="file"] { font-size: 11px; max-width: 220px; }
        .otaVer { font-size: 12px; font-weight: 700; min-width: 72px; }
        .otaFileInfo { font-size: 11px; max-width: 320px; overflow: hidden; text-overflow: ellipsis; }
        .otaMsg { font-size: 11px; max-width: 260px; overflow: hidden; text-overflow: ellipsis; }
        .otaModalOverlay {
            position: fixed;
            inset: 0;
            z-index: 120;
            display: none;
            align-items: center;
            justify-content: center;
            background: rgba(0, 0, 0, 0.45);
        }
        .otaModalOverlay.active { display: flex; }
        .otaModal {
            width: min(92vw, 560px);
            background: #f8f8f8;
            border: 2px solid #3a3a3a;
            box-shadow: 0 10px 26px rgba(0, 0, 0, 0.35);
            padding: 18px 18px 16px;
        }
        .otaModalTitle { font-size: 22px; font-weight: 800; margin-bottom: 8px; }
        .otaModalStatus { font-size: 14px; margin-bottom: 12px; min-height: 20px; }
        .otaProgressFrame {
            width: 100%;
            height: 30px;
            border: 1px solid #353535;
            background: #d0d0d0;
            overflow: hidden;
        }
        .otaProgressBar {
            width: 0%;
            height: 100%;
            background: linear-gradient(90deg, #0b67d1, #17a0ff);
            transition: width 0.16s linear;
        }
        .otaProgressText {
            margin-top: 8px;
            font-size: 18px;
            font-weight: 800;
            letter-spacing: 0.3px;
        }
        .ctxMenu { position: fixed; z-index: 99; background: #fff; border: 1px solid #666; box-shadow: 0 2px 10px rgba(0,0,0,0.2); display:none; }
        .ctxMenu button { display:block; width: 140px; border: 0; background:#fff; text-align:left; padding:8px 10px; font-size:12px; }
        .ctxMenu button:hover { background:#efefef; }
        .warningStack {
            position: fixed;
            top: 14px;
            right: 14px;
            z-index: 132;
            display: flex;
            flex-direction: column;
            gap: 10px;
            width: min(94vw, 720px);
            pointer-events: none;
        }
        .warningRow {
            display: flex;
            gap: 10px;
            justify-content: flex-end;
            align-items: stretch;
        }
        .warningCard {
            position: relative;
            flex: 1 1 0;
            min-width: 220px;
            background: #fff7e6;
            border: 2px solid #8a1b1b;
            box-shadow: 0 10px 26px rgba(0, 0, 0, 0.35);
            padding: 14px 42px 14px 14px;
            pointer-events: auto;
        }
        .warningCard.fire { background: #ffecec; border-color: #b00000; }
        .warningCard.nobreath { background: #f5e6e6; border-color: #8f1c1c; }
        .warningTitle { font-size: 22px; font-weight: 900; color: #8a1b1b; margin-bottom: 8px; }
        .warningCard.fire .warningTitle { color: #b00000; }
        .warningCard.nobreath .warningTitle { color: #8f1c1c; }
        .warningBody { font-size: 15px; font-weight: 700; color: #2a2a2a; line-height: 1.45; }
        .warningClose {
            position: absolute;
            top: 8px;
            right: 8px;
            width: 26px;
            height: 26px;
            border: 1px solid rgba(0, 0, 0, 0.35);
            background: rgba(255, 255, 255, 0.85);
            color: #111;
            font-size: 16px;
            font-weight: 900;
            line-height: 1;
            cursor: pointer;
        }
        .warningClose:hover { background: #fff; }
        @media (max-width: 560px) {
            .warningStack { left: 8px; right: 8px; top: 8px; width: auto; }
            .warningRow { flex-direction: column; }
            .warningCard { min-width: 0; }
        }
    </style>
</head>
<body>
    <div class="wrap">
        <div class="top">
            <div id="title" class="title">Life Monitor Platform(LMP) by JJSystem</div>
            <div id="wsState" class="meta">WS: connecting...</div>
        </div>
        <div class="statusBar">
            <div id="wisunState">Wi-SUN: -</div>
            <div id="wifiState">Wi-Fi: -</div>
            <div id="nodes">Nodes: -</div>
            <div id="checkInt">Check: 30m</div>
        </div>
        <div class="mainArea">
            <div class="boardWrap"><div id="board" class="board"></div></div>
            <div class="logPane">
                <div class="logTitle">Packet Display <button id="btnPacketClose" class="logCloseBtn">Close</button></div>
                <div class="log" id="log"></div>
            </div>
        </div>
        <div class="controls">
            <button id="btnDataReset">초기화</button>
            <span id="fwVerLabel" class="otaFileInfo">파일 이름: -</span>
            <input id="otaFile" type="file" accept=".bin,application/octet-stream" />
            <span id="otaFileInfo" class="otaVer">버전: -</span>
            <button id="btnOtaUpdate">Update</button>
            <span id="otaMsg" class="otaMsg"></span>
            <select id="intervalSel"></select>
            <button id="btnPacketToggle">Hide Window</button>
            <button id="btnCheckInterval">Check Interval</button>
            <button id="btnFireClear">Warning Clear</button>
                    </div>
        <div id="otaModal" class="otaModalOverlay" aria-hidden="true">
            <div class="otaModal" role="dialog" aria-modal="true" aria-label="Firmware Update Progress">
                <div class="otaModalTitle">Firmware Update</div>
                <div id="otaModalStatus" class="otaModalStatus">업로드 준비 중...</div>
                <div class="otaProgressFrame"><div id="otaProgressBar" class="otaProgressBar"></div></div>
                <div id="otaProgressText" class="otaProgressText">0%</div>
            </div>
        </div>
        <div id="ctxMenu" class="ctxMenu">
            <button id="ctxStatusCheck">Status Check</button>
            <button id="ctxNetworkCheck">Network Check</button>
        </div>
        <div id="warningStack" class="warningStack" aria-live="assertive" aria-label="Warning Notifications"></div>
    </div>
    <script>
        const logEl = document.getElementById('log');
        const nodesEl = document.getElementById('nodes');
        const wsStateEl = document.getElementById('wsState');
        const wisunStateEl = document.getElementById('wisunState');
        const wifiStateEl = document.getElementById('wifiState');
        const titleEl = document.getElementById('title');
        const boardEl = document.getElementById('board');
        const mainAreaEl = document.querySelector('.mainArea');
        const logPaneEl = document.querySelector('.logPane');
        const checkIntEl = document.getElementById('checkInt');
        const fwVerLabel = document.getElementById('fwVerLabel');
        const otaFileEl = document.getElementById('otaFile');
        const otaFileInfoEl = document.getElementById('otaFileInfo');
        const btnOtaUpdate = document.getElementById('btnOtaUpdate');
        const otaMsgEl = document.getElementById('otaMsg');
        const intervalSel = document.getElementById('intervalSel');
        const btnPacketToggle = document.getElementById('btnPacketToggle');
        const btnPacketClose = document.getElementById('btnPacketClose');
        const btnCheckInterval = document.getElementById('btnCheckInterval');
        const btnFireClear = document.getElementById('btnFireClear');
        const otaModalEl = document.getElementById('otaModal');
        const otaModalStatusEl = document.getElementById('otaModalStatus');
        const otaProgressBarEl = document.getElementById('otaProgressBar');
        const otaProgressTextEl = document.getElementById('otaProgressText');
        const ctxMenu = document.getElementById('ctxMenu');
        const ctxStatusCheck = document.getElementById('ctxStatusCheck');
        const ctxNetworkCheck = document.getElementById('ctxNetworkCheck');
        const warningStackEl = document.getElementById('warningStack');

        let aptRows = [];
        let cellMap = new Map();
        let maxFloor = 1;
        let dongGroups = [];
        let currentCtxKey = '';
        let packetPaneVisible = true;
        let fallBlinkOn = false;
        const warningActiveKeys = { fire: new Set(), fall: new Set(), nobreath: new Set() };
        const warningOrder = new Map();
        const dismissedWarnings = new Set();
        let nextWarningOrder = 1;
        const WARNING_TYPES = Object.keys(warningActiveKeys);
        const WARNING_META = {
            fire:     { title: 'Fire Detected!!',    ariaLabel: 'Fire Detection Warning',      body: (key) => keyToDongHoLabel(key) + ' Fire Detected!!' },
            fall:     { title: 'FALL DETECTED',      ariaLabel: 'Fall Detection Warning',      body: (key) => keyToDongHoLabel(key) + ' 낙상 발생' },
            nobreath: { title: 'NO BREATH DETECTED', ariaLabel: 'No Breath Detection Warning', body: (key) => keyToDongHoLabel(key) + ' 호흡 미감지' },
        };

        function keyToDongHoLabel(key) {
            if (!key) return '';
            const p = String(key).split('_');
            if (p.length < 2) return key;
            return `${p[0]}동 ${p[1]}호`;
        }

        function warningDismissId(type, key) {
            return type + ':' + key;
        }

        function markWarningActive(type, key) {
            if (!key || dismissedWarnings.has(warningDismissId(type, key))) return;
            warningActiveKeys[type].add(key);
            if (!warningOrder.has(key)) warningOrder.set(key, nextWarningOrder++);
        }

        function clearWarningType(type, key) {
            warningActiveKeys[type].delete(key);
            dismissedWarnings.delete(warningDismissId(type, key));
            if (!WARNING_TYPES.some(t => warningActiveKeys[t].has(key))) warningOrder.delete(key);
        }

        function dismissWarning(type, key) {
            dismissedWarnings.add(warningDismissId(type, key));
            updateWarningPopups();
        }

        function clearAllWarningPopups() {
            WARNING_TYPES.forEach(t => warningActiveKeys[t].clear());
            warningOrder.clear();
            dismissedWarnings.clear();
            updateWarningPopups();
        }

        function createWarningCard(type, key) {
            const meta = WARNING_META[type];
            const card = document.createElement('div');
            card.className = 'warningCard ' + type;
            card.setAttribute('role', 'alertdialog');
            card.setAttribute('aria-label', meta.ariaLabel);

            const close = document.createElement('button');
            close.className = 'warningClose';
            close.type = 'button';
            close.textContent = 'x';
            close.title = 'Close popup';
            close.onclick = () => dismissWarning(type, key);

            const title = document.createElement('div');
            title.className = 'warningTitle';
            title.textContent = meta.title;

            const body = document.createElement('div');
            body.className = 'warningBody';
            body.textContent = meta.body(key);

            card.appendChild(close);
            card.appendChild(title);
            card.appendChild(body);
            return card;
        }

        function updateWarningPopups() {
            warningStackEl.innerHTML = '';
            const activeKeys = Array.from(warningOrder.keys())
                .filter(k => WARNING_TYPES.some(t => warningActiveKeys[t].has(k) && !dismissedWarnings.has(warningDismissId(t, k))))
                .sort((a, b) => warningOrder.get(a) - warningOrder.get(b))
                .slice(0, 5);

            for (const key of activeKeys) {
                const row = document.createElement('div');
                row.className = 'warningRow';
                for (const t of WARNING_TYPES) {
                    if (warningActiveKeys[t].has(key) && !dismissedWarnings.has(warningDismissId(t, key))) {
                        row.appendChild(createWarningCard(t, key));
                    }
                }
                warningStackEl.appendChild(row);
            }
        }

        function refreshFallBlinkClasses() {
            for (const key of warningActiveKeys.fall) {
                const cell = cellMap.get(key);
                if (!cell) continue;
                cell.classList.remove('fallA', 'fallB');
                cell.classList.add(fallBlinkOn ? 'fallA' : 'fallB');
            }
        }

        function syncPacketStreamState() {
            fetch('/api/packet_stream?enable=' + (packetPaneVisible ? '1' : '0')).catch(() => {});
        }

        function setPacketWindowVisible(v) {
            packetPaneVisible = !!v;
            if (packetPaneVisible) {
                mainAreaEl.classList.remove('packet-hidden');
                btnPacketToggle.textContent = 'Hide Window';
            } else {
                mainAreaEl.classList.add('packet-hidden');
                btnPacketToggle.textContent = 'Show Window';
            }
            localStorage.setItem('packetWindowVisible', packetPaneVisible ? '1' : '0');
            syncPacketStreamState();
            setTimeout(() => buildBoard(), 20);
        }

        function addLog(line) {
            if (!packetPaneVisible) return;
            const div = document.createElement('div');
            div.className = 'line';
            div.textContent = line;
            logEl.appendChild(div);
            while (logEl.childElementCount > 300) logEl.removeChild(logEl.firstChild);
            logEl.scrollTop = logEl.scrollHeight;
        }

        function codeFromStatus(statusText) {
            const s = (statusText || '').trim();
            const sl = s.toLowerCase();
            if (s.endsWith('_fire_detected')) return 'FIR';
            if (s === 'Fall detected') return 'FAL';
            if (s === 'Fall cleared') return 'NHP';
            if (s.endsWith('_normal')) return 'NHP';
            if (s.startsWith('Detected BPM:')) {
                const m = s.match(/Detected BPM:\s*(\d+)/);
                if (m) return String(m[1]).padStart(3, '0').slice(0, 3);
                return '---';
            }
            if (sl.includes('no bpm detect')) return 'NPM';
            if (s === 'No breath detected...') return 'NOB';
            if (sl.includes('no valid breath') && sl.includes('detect')) return 'ALR';
            if (s === 'No valid breaths detected') return 'ALR';
            if (s === 'No human presence detected!' || s === 'Network OK') return 'NHP';
            if (s.includes('Object Moving') || s.includes('Movement detected') || s.includes('Fast Movement')) return 'MOV';
            if (s.includes('Signal stabilized') || s === 'measuring') return 'STB';
            return 'COM';
        }

        function intervalLabelFromSec(sec) {
            const totalMin = Math.max(30, Math.floor(sec / 60));
            const h = Math.floor(totalMin / 60);
            const m = totalMin % 60;
            if (h === 0) return `${m}m`;
            if (m === 0) return `${h}h`;
            return `${h}h ${m}m`;
        }

        function initIntervalOptions(selectedSec) {
            intervalSel.innerHTML = '';
            for (let sec = 1800; sec <= 86400; sec += 1800) {
                const opt = document.createElement('option');
                opt.value = String(sec);
                opt.textContent = intervalLabelFromSec(sec);
                if (sec === selectedSec) opt.selected = true;
                intervalSel.appendChild(opt);
            }
        }

        function buildBoard() {
            if (!aptRows.length) return;
            const columns = aptRows.length;
            const rows = maxFloor + 1; // floors + dong label row
            const bw = boardEl.parentElement.clientWidth;
            const bh = boardEl.parentElement.clientHeight;
            const cw = Math.max(26, Math.min(56, Math.floor((bw - ((columns - 1) * 2)) / columns)));
            const ch = Math.max(16, Math.min(38, Math.floor((bh - ((rows - 1) * 2)) / rows)));
            const lh = Math.max(14, Math.min(20, Math.floor(ch * 0.7)));

            boardEl.style.setProperty('--cw', cw + 'px');
            boardEl.style.setProperty('--ch', ch + 'px');
            boardEl.style.setProperty('--lh', lh + 'px');
            boardEl.style.gridTemplateColumns = `repeat(${columns}, ${cw}px)`;
            boardEl.innerHTML = '';
            cellMap = new Map();

            const dongIndex = new Map();
            aptRows.forEach((r, idx) => {
                if (!dongIndex.has(r.dong)) dongIndex.set(r.dong, []);
                dongIndex.get(r.dong).push(idx);
            });
            dongGroups = Array.from(dongIndex.entries());

            for (let floor = maxFloor; floor >= 1; floor--) {
                for (let c = 0; c < columns; c++) {
                    const r = aptRows[c];
                    const unit = r.startHo % 100;
                    const ho = floor * 100 + unit;
                    const inRange = ho >= r.startHo && ho <= r.endHo;
                    if (!inRange) {
                        const empty = document.createElement('div');
                        empty.className = 'empty';
                        boardEl.appendChild(empty);
                        continue;
                    }
                    const key = `${r.dong}_${ho}_1`;
                    const cell = document.createElement('div');
                    cell.className = 'cell';
                    cell.innerHTML = `<div class="ho">${ho}</div><div class="code"></div>`;
                    cell.addEventListener('contextmenu', (e) => {
                        e.preventDefault();
                        currentCtxKey = key;
                        ctxMenu.style.left = e.clientX + 'px';
                        ctxMenu.style.top = e.clientY + 'px';
                        ctxMenu.style.display = 'block';
                    });
                    boardEl.appendChild(cell);
                    cellMap.set(key, cell);
                }
            }

            for (let c = 0; c < columns; c++) {
                const label = document.createElement('div');
                label.className = 'dongLabel';
                label.textContent = aptRows[c].dong + ' 동';
                boardEl.appendChild(label);
            }

            refreshFallBlinkClasses();
        }

        function applyStatus(msg) {
            wisunStateEl.textContent = 'Wi-SUN: ' + (msg.wisunConnected ? 'Connected' : 'Not Connected');
            wifiStateEl.textContent = 'Wi-Fi: ' + (msg.wifiConnected ? ('Connected (' + (msg.ip || '-') + ')') : 'Not Connected');
            if (Number.isInteger(msg.nodeCount)) {
                nodesEl.textContent = 'Nodes: ' + msg.nodeCount;
            } else if (Array.isArray(msg.nodes)) {
                nodesEl.textContent = 'Nodes: ' + msg.nodes.length;
            }
            if (msg.checkIntervalSec) {
                checkIntEl.textContent = 'Check: ' + intervalLabelFromSec(msg.checkIntervalSec);
                if (!intervalSel.options.length) initIntervalOptions(msg.checkIntervalSec);
            }

            if (msg.key) applyCellUpdate(msg.key, msg.code, msg.statusText, msg.raw || '', msg.everSeen);
        }

        function applyCellUpdate(key, code, statusText, eventText, everSeen) {
            if (!key || !cellMap.has(key)) return;
            const cell = cellMap.get(key);
            const finalCode = (code && code.length) ? code : codeFromStatus(statusText || '');
            const normalizedStatus = String(statusText || '').trim().toLowerCase();
            const normalizedEvent = String(eventText || '').trim().toLowerCase();
            cell.querySelector('.code').textContent = finalCode || '';
            cell.classList.remove('mov', 'fire', 'nob', 'alr', 'stb', 'bpm', 'nhp', 'comSeen', 'fallA', 'fallB');
            if (finalCode === 'MOV') cell.classList.add('mov');
            if (finalCode === 'FIR') cell.classList.add('fire');
            if (finalCode === 'NOB') cell.classList.add('nob');
            if (finalCode === 'NPM') cell.classList.add('nob');
            if (finalCode === 'ALR') cell.classList.add('alr');
            if (finalCode === 'STB') cell.classList.add('stb');
            if (finalCode === 'NHP') cell.classList.add('nhp');
            if (finalCode === 'COM' && everSeen === true) cell.classList.add('comSeen');
            if (/^\d{3}$/.test(finalCode)) cell.classList.add('bpm');

            if (normalizedEvent === 'warning_clear') {
                clearWarningType('fall', key);
                clearWarningType('fire', key);
                clearWarningType('nobreath', key);
                updateWarningPopups();
                return;
            }

            const fireDetectedStatus = normalizedStatus.endsWith('_fire_detected');
            const fireNormalStatus = normalizedStatus.endsWith('_normal');
            if (finalCode === 'FIR' || (fireDetectedStatus && finalCode !== 'NHP')) {
                markWarningActive('fire', key);
            } else if (fireNormalStatus) {
                clearWarningType('fire', key);
            }

            // Fall popup lifetime is driven only by explicit fall events.
            if (normalizedStatus === 'fall detected') {
                markWarningActive('fall', key);
            } else if (normalizedStatus === 'fall cleared') {
                clearWarningType('fall', key);
            }

            if (finalCode === 'FAL') {
                markWarningActive('fall', key);
            }

            // No-breath popup: raised on "No valid breaths detected" (code ALR),
            // cleared as soon as ANY one of movement / stabilized / breath (BPM) /
            // no-human-presence is reported — not only once breathing resumes.
            if (finalCode === 'ALR') {
                markWarningActive('nobreath', key);
            } else if (finalCode === 'MOV' || finalCode === 'STB' || finalCode === 'NHP' || /^\d{3}$/.test(finalCode)) {
                clearWarningType('nobreath', key);
            }

            if (finalCode === 'FAL' || warningActiveKeys.fall.has(key)) {
                cell.classList.add(fallBlinkOn ? 'fallA' : 'fallB');
            }
            updateWarningPopups();
        }

        function syncCellsSnapshot() {
            return fetch('/api/cells')
                .then(r => r.json())
                .then(d => {
                    if (!d || !Array.isArray(d.cells)) return;
                    d.cells.forEach(c => applyCellUpdate(c.key, c.code, c.statusText, '', c.everSeen));
                })
                .catch(() => {});
        }

        function hideContextMenu() {
            ctxMenu.style.display = 'none';
        }

        function setOtaMessage(msg) {
            otaMsgEl.textContent = msg || '';
        }

        function showOtaModal(statusText) {
            otaModalEl.classList.add('active');
            otaModalEl.setAttribute('aria-hidden', 'false');
            otaModalStatusEl.textContent = statusText || '업로드 준비 중...';
        }

        function hideOtaModal() {
            otaModalEl.classList.remove('active');
            otaModalEl.setAttribute('aria-hidden', 'true');
        }

        function updateOtaProgress(percent, statusText) {
            const p = Math.max(0, Math.min(100, Math.round(Number(percent) || 0)));
            otaProgressBarEl.style.width = p + '%';
            otaProgressTextEl.textContent = p + '%';
            if (statusText) otaModalStatusEl.textContent = statusText;
        }

        function formatFileDateTime(ms) {
            if (!ms || Number.isNaN(ms)) return '-';
            const d = new Date(ms);
            const pad2 = n => String(n).padStart(2, '0');
            const y = d.getFullYear();
            const mo = pad2(d.getMonth() + 1);
            const da = pad2(d.getDate());
            const hh = pad2(d.getHours());
            const mi = pad2(d.getMinutes());
            const ss = pad2(d.getSeconds());
            return `${y}-${mo}-${da} ${hh}:${mi}:${ss}`;
        }

        function getLastUploadedMeta() {
            try {
                const raw = localStorage.getItem('otaLastUploadedMeta');
                if (!raw) return null;
                const parsed = JSON.parse(raw);
                if (!parsed || !parsed.name || !parsed.lastModified) return null;
                return parsed;
            } catch {
                return null;
            }
        }

        function saveLastUploadedMeta(file) {
            if (!file || !file.name) return;
            const meta = {
                name: file.name,
                lastModified: Number(file.lastModified || 0)
            };
            localStorage.setItem('otaLastUploadedMeta', JSON.stringify(meta));
        }

        function updateOtaFileInfo() {
            const file = otaFileEl.files && otaFileEl.files[0];
            if (file) {
                const dt = formatFileDateTime(file.lastModified);
                fwVerLabel.textContent = '파일 이름: ' + file.name + ' (' + dt + ')';
                return;
            }
            const last = getLastUploadedMeta();
            if (last) {
                const dt = formatFileDateTime(last.lastModified);
                fwVerLabel.textContent = '파일 이름: ' + last.name + ' (' + dt + ')';
                return;
            }
            fwVerLabel.textContent = '파일 이름: -';
        }

        document.addEventListener('click', hideContextMenu);

        btnFireClear.onclick = () => {
            fetch('/api/fire_clear').then(r => r.json()).then(() => {
                clearAllWarningPopups();
                addLog('[WEB] Warning clear sent');
            }).catch(() => addLog('[WEB] Warning clear failed'));
        };
        const btnDataReset = document.getElementById('btnDataReset');
        btnDataReset.onclick = () => {
            fetch('/api/data_reset').then(r => r.json()).then(() => {
                clearAllWarningPopups();
                logEl.textContent = '';
                cellMap.forEach((cell) => {
                    const code = cell.querySelector('.code');
                    if (code) code.textContent = '';
                    cell.className = 'cell';
                });
                nodesEl.textContent = 'Nodes: 0';
                addLog('[WEB] data reset, collecting from scratch');
            }).catch(() => addLog('[WEB] data reset failed'));
        };

        btnPacketToggle.onclick = () => setPacketWindowVisible(!packetPaneVisible);
        btnPacketClose.onclick = () => setPacketWindowVisible(false);

        btnCheckInterval.onclick = () => {
            const sec = Number(intervalSel.value || 1800);
            fetch('/api/check_interval?sec=' + sec).then(r => r.json()).then(() => {
                checkIntEl.textContent = 'Check: ' + intervalLabelFromSec(sec);
                addLog('[WEB] Check interval set: ' + intervalLabelFromSec(sec));
            }).catch(() => addLog('[WEB] Check interval update failed'));
        };

        otaFileEl.onchange = () => {
            updateOtaFileInfo();
            setOtaMessage('');
        };

        ctxStatusCheck.onclick = () => {
            hideContextMenu();
            if (!currentCtxKey) return;
            fetch('/api/status_check?key=' + encodeURIComponent(currentCtxKey))
                .then(r => r.json())
                .then(j => {
                    if (!j || !j.ok) throw new Error((j && j.msg) || 'status check failed');
                    addLog('[WEB] Status check sent: ' + currentCtxKey);
                })
                .catch(() => addLog('[WEB] Status check failed: ' + currentCtxKey));
        };

        ctxNetworkCheck.onclick = () => {
            hideContextMenu();
            if (!currentCtxKey) return;
            fetch('/api/network_check?key=' + encodeURIComponent(currentCtxKey))
                .then(r => r.json())
                .then(j => {
                    if (!j || !j.ok) throw new Error((j && j.msg) || 'network check failed');
                    addLog('[WEB] Network check sent: ' + currentCtxKey);
                })
                .catch(() => addLog('[WEB] Network check failed: ' + currentCtxKey));
        };

        btnOtaUpdate.onclick = async () => {
            const file = otaFileEl.files && otaFileEl.files[0];
            if (!file) {
                setOtaMessage('Select a .bin file first');
                return;
            }
            btnOtaUpdate.disabled = true;
            setOtaMessage('Uploading...');
            showOtaModal('펌웨어 업로드 중...');
            updateOtaProgress(0, '펌웨어 업로드 중...');
            try {
                const result = await new Promise((resolve, reject) => {
                    const xhr = new XMLHttpRequest();
                    xhr.open('POST', '/api/ota_update', true);
                    xhr.setRequestHeader('Content-Type', 'application/octet-stream');
                    xhr.setRequestHeader('X-Filename', encodeURIComponent(file.name));

                    xhr.upload.onprogress = (evt) => {
                        if (evt.lengthComputable && evt.total > 0) {
                            const percent = (evt.loaded / evt.total) * 100;
                            updateOtaProgress(percent, '펌웨어 업로드 중...');
                        } else {
                            otaModalStatusEl.textContent = '업로드 중...';
                        }
                    };

                    xhr.onerror = () => reject(new Error('network error'));
                    xhr.onabort = () => reject(new Error('upload aborted'));
                    xhr.onload = () => {
                        let parsed = null;
                        try {
                            parsed = JSON.parse(xhr.responseText || '{}');
                        } catch {
                            parsed = null;
                        }
                        if (xhr.status >= 200 && xhr.status < 300) {
                            resolve(parsed || {});
                        } else {
                            reject(new Error((parsed && parsed.msg) ? parsed.msg : ('http ' + xhr.status)));
                        }
                    };

                    xhr.send(file);
                });

                const j = result || {};
                setOtaMessage(j && j.msg ? j.msg : 'Update response received');
                if (j && j.ok) {
                    updateOtaProgress(100, '업로드 완료, 재부팅 준비 중...');
                    saveLastUploadedMeta(file);
                    updateOtaFileInfo();
                    setTimeout(() => hideOtaModal(), 1400);
                } else {
                    otaModalStatusEl.textContent = (j && j.msg) ? j.msg : '업데이트 실패';
                }
            } catch {
                setOtaMessage('Update failed');
                otaModalStatusEl.textContent = '업데이트 실패';
            }
            btnOtaUpdate.disabled = false;
        };

        fetch('/api/layout').then(r => r.json()).then(cfg => {
            titleEl.textContent = cfg.aptName || 'Life Monitor Platform(LMP) by JJSystem';
            aptRows = cfg.rows || [];
            maxFloor = cfg.maxFloor || 1;
            buildBoard();
            syncCellsSnapshot();
            return fetch('/api/status').then(r => r.json()).then(applyStatus).catch(() => {});
        }).then(() => {
            return fetch('/api/fw_version').then(r => r.json()).then(v => {
                otaFileInfoEl.textContent = '버전: ' + ((v && v.version) ? v.version : '-');
            }).catch(() => {
                otaFileInfoEl.textContent = '버전: -';
            });
        }).catch(() => {});
        updateOtaFileInfo();
        initIntervalOptions(1800);
        setPacketWindowVisible(localStorage.getItem('packetWindowVisible') === '1');
        setInterval(() => {
            syncCellsSnapshot();
        }, 5000);

        window.addEventListener('resize', () => buildBoard());

        let ws = null;
        let wsRetryMs = 1000;
        let wsRetryTimer = null;
        let wsConnectWatchdog = null;
        let wsLastMessageMs = 0;

        function scheduleWsReconnect() {
            if (wsRetryTimer) return;
            wsStateEl.textContent = 'WS: reconnecting...';
            wsRetryTimer = setTimeout(() => {
                wsRetryTimer = null;
                connectWs();
            }, wsRetryMs);
            wsRetryMs = Math.min(wsRetryMs * 2, 10000);
        }

        function connectWs() {
            if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) return;
            const proto = (location.protocol === 'https:') ? 'wss://' : 'ws://';
            let socket;
            try {
                socket = new WebSocket(proto + location.host + '/ws');
                ws = socket;
            } catch {
                scheduleWsReconnect();
                return;
            }

            if (wsConnectWatchdog) {
                clearTimeout(wsConnectWatchdog);
                wsConnectWatchdog = null;
            }
            wsConnectWatchdog = setTimeout(() => {
                if (ws === socket && socket.readyState === WebSocket.CONNECTING) {
                    wsStateEl.textContent = 'WS: timeout';
                    try { socket.close(); } catch {}
                    scheduleWsReconnect();
                }
            }, 4000);

            socket.onopen = () => {
                if (ws !== socket) return;
                if (wsConnectWatchdog) {
                    clearTimeout(wsConnectWatchdog);
                    wsConnectWatchdog = null;
                }
                wsStateEl.textContent = 'WS: connected';
                wsRetryMs = 1000;
                wsLastMessageMs = Date.now();
                syncCellsSnapshot();
            };
            socket.onclose = () => {
                if (ws !== socket) return;
                if (wsConnectWatchdog) {
                    clearTimeout(wsConnectWatchdog);
                    wsConnectWatchdog = null;
                }
                ws = null;
                wsStateEl.textContent = 'WS: disconnected';
                scheduleWsReconnect();
            };
            socket.onerror = () => {
                if (ws !== socket) return;
                if (wsConnectWatchdog) {
                    clearTimeout(wsConnectWatchdog);
                    wsConnectWatchdog = null;
                }
                wsStateEl.textContent = 'WS: error';
                try { socket.close(); } catch {}
            };
            socket.onmessage = (evt) => {
                if (ws !== socket) return;
                wsLastMessageMs = Date.now();
                if (evt.data === 'pong') return;
                try {
                    const msg = JSON.parse(evt.data);
                    applyStatus(msg);
                    if (msg.raw && msg.raw.length) addLog(msg.raw);
                } catch {
                    addLog(evt.data);
                }
            };
        }

        connectWs();
        setInterval(() => {
            if (!ws || ws.readyState !== WebSocket.OPEN) return;
            if (Date.now() - wsLastMessageMs > 25000) {
                wsStateEl.textContent = 'WS: stale';
                try { ws.close(); } catch {}
                return;
            }
            try { ws.send('ping'); } catch { try { ws.close(); } catch {} }
        }, 10000);
        setInterval(() => {
            fallBlinkOn = !fallBlinkOn;
            if (warningActiveKeys.fall.size) refreshFallBlinkClasses();
        }, 500);
        setInterval(() => {
            if (!ws || ws.readyState !== WebSocket.OPEN) {
                fetch('/api/status').then(r => r.json()).then(applyStatus).catch(() => {});
            }
        }, 3000);
    </script>
</body>
</html>
)HTML";

static String jsonEscape(const char* s) {
        String out;
        if (!s) return "";
        while (*s) {
                char c = *s++;
                if (c == '"' || c == '\\') { out += '\\'; out += c; }
                else if (c == '\n') out += "\\n";
                else if (c == '\r') out += "\\r";
                else out += c;
        }
        return out;
}

static esp_err_t sendWebResponse(httpd_req_t* req, const char* type, const char* body, size_t bodyLen, const char* tag) {
    httpd_resp_set_type(req, type);
    unsigned long startedMs = millis();
    esp_err_t result = httpd_resp_send(req, body, bodyLen);
    unsigned long elapsedMs = millis() - startedMs;
    if (elapsedMs >= 2000) {
        g_webRecoveryRequested = true;
        Serial.printf("[WEB] slow HTTP response tag=%s uri=%s elapsed=%lums\n",
                      tag ? tag : "?", req->uri, elapsedMs);
    }
    if (result != ESP_OK) {
        Serial.printf("[WEB] HTTP send failed tag=%s uri=%s fd=%d bytes=%u err=%s free=%u largest=%u\n",
                              tag ? tag : "?", req->uri, httpd_req_to_sockfd(req),
                  (unsigned)bodyLen, esp_err_to_name(result), (unsigned)esp_get_free_heap_size(),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    }
    return result;
}

static void trackNode(const char* srcAddr) {
        if (!srcAddr || !strlen(srcAddr)) return;
        for (int i = 0; i < g_nodeCount; i++) {
                if (strcmp(g_nodes[i].ipv6, srcAddr) == 0) {
                        g_nodes[i].lastSeenMs = millis();
                        return;
                }
        }
        if (g_nodeCount < (int)(sizeof(g_nodes) / sizeof(g_nodes[0]))) {
                strncpy(g_nodes[g_nodeCount].ipv6, srcAddr, sizeof(g_nodes[g_nodeCount].ipv6) - 1);
                g_nodes[g_nodeCount].ipv6[sizeof(g_nodes[g_nodeCount].ipv6) - 1] = '\0';
                g_nodes[g_nodeCount].lastSeenMs = millis();
                g_nodeCount++;
        }
}

static String buildWsJson(const char* srcAddr, const char* payload, const char* rawLine, bool includeNodes) {
        String s = "{";
        s += "\"seq\":" + String(++g_wsSeq);
        s += ",\"wisunConnected\":" + String(isWiSUNConnected ? "true" : "false");
        s += ",\"wifiConnected\":" + String(hasValidStaIp() ? "true" : "false");
    s += ",\"checkIntervalSec\":" + String(g_checkIntervalSec);
        s += ",\"ip\":\"" + jsonEscape(WiFi.localIP().toString().c_str()) + "\"";
        s += ",\"src\":\"" + jsonEscape(srcAddr ? srcAddr : "") + "\"";
        s += ",\"payload\":\"" + jsonEscape(payload ? payload : "") + "\"";
        s += ",\"raw\":\"" + jsonEscape(rawLine ? rawLine : "") + "\"";
        s += ",\"nodeCount\":" + String(g_nodeCount);
        if (includeNodes) {
            s += ",\"nodes\":[";
            for (int i = 0; i < g_nodeCount; i++) {
                if (i) s += ",";
                s += "\"" + jsonEscape(g_nodes[i].ipv6) + "\"";
            }
            s += "]";
        }
        s += "}";
        return s;
}

static String trimPacketForWeb(const char* text) {
    if (!text || !*text) return "";
    const char* tail = strrchr(text, ':');
    if (tail && *(tail + 1)) {
        tail++;
        while (*tail == ' ') tail++;
        if (*tail) return String(tail);
    }
    return String(text);
}

static int findKeyIp(const char* key) {
    for (int i = 0; i < g_keyIpCount; i++) {
        if (strcmp(g_keyIps[i].key, key) == 0) return i;
    }
    return -1;
}

static int skipWhitespace(const String& text, int pos) {
    while (pos < (int)text.length()) {
        char c = text[pos];
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            pos++;
        } else {
            break;
        }
    }
    return pos;
}

static bool parseIpMappingTableText(const String& text) {
    int loaded = 0;
    int pos = 0;
    while (pos < (int)text.length()) {
        int k1 = text.indexOf('"', pos);
        if (k1 < 0) break;
        int k2 = text.indexOf('"', k1 + 1);
        if (k2 < 0) break;

        String key = text.substring(k1 + 1, k2);
        int colon = text.indexOf(':', k2 + 1);
        if (colon < 0) break;

        int objStart = skipWhitespace(text, colon + 1);
        if (objStart >= (int)text.length() || text[objStart] != '{') {
            pos = k2 + 1;
            continue;
        }

        int objEnd = text.indexOf('}', objStart + 1);
        if (objEnd < 0) break;

        String obj = text.substring(objStart, objEnd + 1);
        int ipv6Tag = obj.indexOf("\"ipv6\"");
        if (ipv6Tag >= 0) {
            int ipv6Colon = obj.indexOf(':', ipv6Tag);
            if (ipv6Colon >= 0) {
                int v1 = obj.indexOf('"', ipv6Colon + 1);
                int v2 = (v1 >= 0) ? obj.indexOf('"', v1 + 1) : -1;
                if (v1 >= 0 && v2 > v1) {
                    String ipv6 = obj.substring(v1 + 1, v2);
                    if (key.length() > 0 && ipv6.length() > 0) {
                        upsertKeyIp(key.c_str(), ipv6.c_str(), false);
                        loaded++;
                    }
                }
            }
        }

        pos = objEnd + 1;
    }
    return loaded > 0;
}

static bool ensureSpiffsMounted() {
    static int8_t s_spiffsState = -1;  // -1 unknown, 0 unavailable, 1 mounted

    if (s_spiffsState == 1) return true;
    if (s_spiffsState == 0) return false;

    const esp_partition_t* spiffsPart = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_DATA_SPIFFS,
        nullptr);
    if (!spiffsPart) {
        Serial.println("[SPIFFS] Partition not found; using embedded/default data only.");
        s_spiffsState = 0;
        return false;
    }

    if (!SPIFFS.begin(false)) {
        Serial.println("[SPIFFS] Mount failed; using embedded/default data only.");
        s_spiffsState = 0;
        return false;
    }

    s_spiffsState = 1;
    return true;
}

static bool saveIpMappingTable() {
    if (!ensureSpiffsMounted()) return false;

    File f = SPIFFS.open("/ip_mapping.table", "w");
    if (!f) return false;

    f.println("{");
    for (int i = 0; i < g_keyIpCount; i++) {
        String ts = String((unsigned long)(g_keyIps[i].lastSeenMs / 1000));
        f.print("  \"");
        f.print(g_keyIps[i].key);
        f.println("\": {");
        f.print("    \"dong_ho\": \"");
        f.print(g_keyIps[i].key);
        f.println("\",");
        f.print("    \"ipv6\": \"");
        f.print(g_keyIps[i].ipv6);
        f.println("\",");
        f.print("    \"last_communication\": \"");
        f.print(ts);
        f.println("\"");
        f.print("  }");
        if (i + 1 < g_keyIpCount) f.println(",");
        else f.println();
    }
    f.println("}");
    f.close();

    g_keyIpDirty = false;
    g_lastKeyIpSaveMs = millis();
    return true;
}

static void loadIpMappingTable() {
    bool loaded = false;

    if (ensureSpiffsMounted()) {
        File f = SPIFFS.open("/ip_mapping.table", "r");
        if (f) {
            String text = f.readString();
            f.close();
            loaded = parseIpMappingTableText(text);
            if (loaded) Serial.printf("[IPMAP] Loaded %d entries from /ip_mapping.table\n", g_keyIpCount);
        }
    }

    if (!loaded) {
        loaded = parseIpMappingTableText(String(DEFAULT_IP_MAPPING_TABLE));
        if (loaded) {
            Serial.printf("[IPMAP] Loaded %d default entries\n", g_keyIpCount);
            saveIpMappingTable();
        } else {
            Serial.println("[IPMAP] No mapping loaded");
        }
    }

    g_keyIpDirty = false;
    g_lastKeyIpSaveMs = millis();
}

static void upsertKeyIp(const char* key, const char* ipv6, bool markLive) {
    if (!key || !*key || !ipv6 || !*ipv6) return;
    int i = findKeyIp(key);
    bool changed = false;
    bool isNewEntry = false;
    if (i < 0) {
        if (g_keyIpCount >= (int)(sizeof(g_keyIps) / sizeof(g_keyIps[0]))) return;
        i = g_keyIpCount++;
        strncpy(g_keyIps[i].key, key, sizeof(g_keyIps[i].key) - 1);
        g_keyIps[i].key[sizeof(g_keyIps[i].key) - 1] = '\0';
        changed = true;
        isNewEntry = true;
    }
    if (strcmp(g_keyIps[i].ipv6, ipv6) != 0) changed = true;
    strncpy(g_keyIps[i].ipv6, ipv6, sizeof(g_keyIps[i].ipv6) - 1);
    g_keyIps[i].ipv6[sizeof(g_keyIps[i].ipv6) - 1] = '\0';
    if (markLive) {
        g_keyIps[i].lastSeenMs = millis();
        if (!g_keyIps[i].everSeen) changed = true;
        g_keyIps[i].everSeen = true;
    } else if (isNewEntry) {
        g_keyIps[i].lastSeenMs = 0; // bulk-loaded, not actually seen live yet
        g_keyIps[i].everSeen = false;
    }
    // existing entry + markLive==false: leave lastSeenMs untouched (never regress a real timestamp)
    if (changed) {
        g_keyIpDirty = true;
        g_runtimeStateDirty = true;
    }
}

static int findCellStateIndexByKey(const char* key) {
    if (!key || !*key) return -1;
    for (int i = 0; i < g_cellStateCount; i++) {
        if (strcmp(g_cellStates[i].key, key) == 0) return i;
    }
    return -1;
}

static bool forceCellComAndNotify(int idx, const char* reasonTag) {
    if (idx < 0 || idx >= g_cellStateCount) return false;
    if (strcmp(g_cellStates[idx].code, "COM") == 0) return false;

    strncpy(g_cellStates[idx].code, "COM", sizeof(g_cellStates[idx].code) - 1);
    g_cellStates[idx].code[sizeof(g_cellStates[idx].code) - 1] = '\0';
    String eventText = String(g_cellStates[idx].key) + "," + (reasonTag ? reasonTag : "timeout");
    notifyWebClients("", g_cellStates[idx].status, eventText.c_str(), g_cellStates[idx].key, g_cellStates[idx].status, g_cellStates[idx].code);
    return true;
}

static int findOrCreateCellStateForCheck(const char* key) {
    int ci = findCellStateIndexByKey(key);
    if (ci >= 0) return ci;
    if (g_cellStateCount >= (int)(sizeof(g_cellStates) / sizeof(g_cellStates[0]))) return -1;

    ci = g_cellStateCount++;
    strncpy(g_cellStates[ci].key, key, sizeof(g_cellStates[ci].key) - 1);
    g_cellStates[ci].key[sizeof(g_cellStates[ci].key) - 1] = '\0';
    g_cellStates[ci].status[0] = '\0';
    g_cellStates[ci].code[0] = '\0';
    g_cellStates[ci].updatedMs = millis();
    g_cellStates[ci].lastWebPushMs = 0;
    g_cellStates[ci].awaitingStatusCheck = false;
    g_cellStates[ci].statusCheckDeadlineMs = 0;
    return ci;
}

static bool sendCheckCommandForKey(const char* key, bool networkCheck) {
    if (!key || !*key) return false;
    int ci = findOrCreateCellStateForCheck(key);
    if (ci < 0) return false;

    auto failCheck = [&](const char* reason) {
        g_cellStates[ci].awaitingStatusCheck = false;
        g_cellStates[ci].statusCheckDeadlineMs = 0;
        forceCellComAndNotify(ci, reason);
        Serial.printf("[CHECK] failed -> key=%s reason=%s\n", key, reason);
        return false;
    };

    if (!g_wisunRuntimeReady) return failCheck("wisun_not_ready");
    const char* dst = g_aptBrIPv6;
    int i = findKeyIp(key);
    if (i >= 0 && strlen(g_keyIps[i].ipv6) > 0) dst = g_keyIps[i].ipv6;
    if (!dst || !*dst) return failCheck("missing_destination");
    if (!ensureATCommandMode()) return failCheck("at_mode_failed");

    char cmd[128];
    snprintf(cmd, sizeof(cmd), "udps %s %s\r\n", dst, networkCheck ? "_chknet" : "_chksts");
    g_lastCheckCommandCellIndex = ci;
    size_t written = WiSUNSerial.print(cmd);
    WiSUNSerial.flush();
    if (written != strlen(cmd)) {
        g_lastCheckCommandCellIndex = -1;
        return failCheck("uart_write_failed");
    }

    g_cellStates[ci].awaitingStatusCheck = true;
    const unsigned long responseTimeoutMs = STATUS_CHECK_RESPONSE_TIMEOUT_MS;
    g_cellStates[ci].statusCheckDeadlineMs = millis() + responseTimeoutMs;

    Serial.printf("[CHECK] sent -> type=%s key=%s dst=%s timeout=%lums\n",
                  networkCheck ? "network" : "status", key, dst, responseTimeoutMs);
    return true;
}

static void clearAllWarnings() {
    for (int i = 0; i < g_fireStateCount; i++) g_fireStates[i].regionMask = 0;
    for (int i = 0; i < g_fallStateCount; i++) g_fallStates[i].active = false;

    for (int i = 0; i < g_cellStateCount; i++) {
        String newCode = statusToCode(g_cellStates[i].status);
        // Manual warning clear must release alarm latches immediately.
        if (newCode == "FIR" || newCode == "FAL") newCode = "NHP";

        if (strcmp(g_cellStates[i].code, newCode.c_str()) != 0) {
            strncpy(g_cellStates[i].code, newCode.c_str(), sizeof(g_cellStates[i].code) - 1);
            g_cellStates[i].code[sizeof(g_cellStates[i].code) - 1] = '\0';
            notifyWebClients("", g_cellStates[i].status, "warning_clear", g_cellStates[i].key, g_cellStates[i].status, g_cellStates[i].code);
        }
    }
    Serial.println("[WEB] Warnings cleared (fire + fall)");
}

static esp_err_t sendOkJson(httpd_req_t* req, bool ok, const char* msg) {
    String body = "{";
    body += "\"ok\":" + String(ok ? "true" : "false");
    body += ",\"msg\":\"" + jsonEscape(msg ? msg : "") + "\"";
    body += "}";
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body.c_str(), HTTPD_RESP_USE_STRLEN);
}

static const char* getFirmwareVersion() {
    const esp_app_desc_t* app = esp_app_get_description();
    if (!app || app->version[0] == '\0') return "unknown";
    return app->version;
}

static esp_err_t webFirmwareVersionHandler(httpd_req_t* req) {
    String body = "{";
    body += "\"version\":\"" + jsonEscape(getFirmwareVersion()) + "\"";
    body += "}";
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body.c_str(), HTTPD_RESP_USE_STRLEN);
}

static esp_err_t webOtaUpdateHandler(httpd_req_t* req) {
    const int total = req->content_len;
    if (total <= 0) {
        return sendOkJson(req, false, "empty firmware payload");
    }

    if (!Update.begin((size_t)total)) {
        return sendOkJson(req, false, Update.errorString());
    }

    uint8_t buf[1024];
    int remaining = total;
    while (remaining > 0) {
        int toRead = remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining;
        int got = httpd_req_recv(req, (char*)buf, toRead);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (got <= 0) {
            Update.abort();
            return sendOkJson(req, false, "ota recv failed");
        }
        size_t written = Update.write(buf, (size_t)got);
        if (written != (size_t)got) {
            Update.abort();
            return sendOkJson(req, false, Update.errorString());
        }
        remaining -= got;
    }

    if (!Update.end(true)) {
        return sendOkJson(req, false, Update.errorString());
    }

    g_scheduledRestartMs = millis() + 1200;
    return sendOkJson(req, true, "update uploaded, rebooting");
}

static esp_err_t webFireClearHandler(httpd_req_t* req) {
    clearAllWarnings();
    return sendOkJson(req, true, "warning cleared");
}

static char g_restartReply[96];
static char g_lastHelmetDst[40];

static void sendRestartTo(const char* dst) {
    if (!dst || !dst[0]) return;
    char addr[48];
    const char* src = dst;
    if (*src == '<') src++;
    strncpy(addr, src, sizeof(addr) - 1);
    addr[sizeof(addr) - 1] = '\0';
    char* gt = strchr(addr, '>');
    if (gt) *gt = '\0';
    char cmd[96];
    snprintf(cmd, sizeof(cmd), "udps %s RESTART_SENSOR\r\n", addr);
    String reply = sendCommand(cmd);
    strncpy(g_restartReply, reply.c_str(), sizeof(g_restartReply) - 1);
    g_restartReply[sizeof(g_restartReply) - 1] = '\0';
    Serial.printf("[RESET] dst=%s reply=%s\n", addr, g_restartReply);
}

static bool sendRestartSensor(const char* only) {
    g_restartReply[0] = '\0';
    if (!only || !only[0]) {
        strncpy(g_restartReply, "no selected helmet", sizeof(g_restartReply) - 1);
        return false;
    }
    if (!ensureATCommandMode()) {
        Serial.println("[RESET] at mode failed");
        strncpy(g_restartReply, "at mode failed", sizeof(g_restartReply) - 1);
        return false;
    }
    sendRestartTo(only);
    return strstr(g_restartReply, "fail") == NULL && strstr(g_restartReply, "Fail") == NULL;
}

static void urlDecode(char* s);

static esp_err_t handleDataReset(httpd_req_t *req) {
    char q[96] = "";
    char src[48] = "";
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        httpd_query_key_value(q, "src", src, sizeof(src));
        urlDecode(src);
    }
    if (!src[0]) strncpy(src, g_lastHelmetDst, sizeof(src) - 1);
    bool sent = sendRestartSensor(src);
    Serial.printf("[WEB] data reset src=%s\n", src);
    return sendOkJson(req, sent, g_restartReply[0] ? g_restartReply : "no reply");
}

static esp_err_t webCheckIntervalHandler(httpd_req_t* req) {
    char q[64] = "";
    char v[16] = "";
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "sec", v, sizeof(v)) == ESP_OK) {
        long sec = atol(v);
        if (sec >= 1800 && sec <= 86400 && (sec % 1800) == 0) {
            g_checkIntervalSec = (unsigned long)sec;
            g_nextCheckCycleMs = millis() + (g_checkIntervalSec * 1000UL);
            g_autoCheckInProgress = false;
            for (int i = 0; i < g_cellStateCount; i++) {
                if (g_cellStates[i].awaitingStatusCheck) {
                    g_cellStates[i].statusCheckDeadlineMs = g_nextCheckCycleMs;
                }
            }
            saveCheckIntervalConfig();
            Serial.printf("[WEB] check interval updated: %lus (saved)\n", g_checkIntervalSec);
            return sendOkJson(req, true, "interval updated");
        }
    }
    return sendOkJson(req, false, "invalid sec");
}

static esp_err_t webStatusCheckHandler(httpd_req_t* req) {
    char q[96] = "";
    char key[24] = "";
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "key", key, sizeof(key)) == ESP_OK) {
        bool ok = sendCheckCommandForKey(key, false);
        return sendOkJson(req, ok, ok ? "status check sent" : "status check failed");
    }
    return sendOkJson(req, false, "missing key");
}

static esp_err_t webNetworkCheckHandler(httpd_req_t* req) {
    char q[96] = "";
    char key[24] = "";
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "key", key, sizeof(key)) == ESP_OK) {
        bool ok = sendCheckCommandForKey(key, true);
        return sendOkJson(req, ok, ok ? "network check sent" : "network check failed");
    }
    return sendOkJson(req, false, "missing key");
}

static esp_err_t webPacketStreamHandler(httpd_req_t* req) {
    char q[32] = "";
    char v[8] = "";
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "enable", v, sizeof(v)) == ESP_OK) {
        g_packetStreamEnabled = (strcmp(v, "0") != 0);
        return sendOkJson(req, true, g_packetStreamEnabled ? "enabled" : "disabled");
    }
    return sendOkJson(req, false, "missing enable");
}

    static String statusToCode(const char* statusText) {
        String s = statusText ? String(statusText) : "";
        s.trim();
        String sl = s;
        sl.toLowerCase();

        if (sl.endsWith("_fire_detected")) return "FIR";
        if (sl == "fall detected") return "FAL";
        if (sl == "fall cleared") return "NHP";
        if (sl.endsWith("_normal")) return "NHP";

        if (s.startsWith("Detected BPM:")) {
            int p = s.indexOf(":");
            if (p != -1) {
                int bpm = s.substring(p + 1).toInt();
                char b[8]; snprintf(b, sizeof(b), "%03d", bpm);
                return String(b);
            }
            return "---";
        }
        if (sl.indexOf("no bpm detect") != -1) return "NPM";
        if (s == "No breath detected...") return "NOB";
        if (sl.indexOf("no valid breath") != -1 && sl.indexOf("detect") != -1) return "ALR";
        if (s == "No valid breaths detected") return "ALR";
        if (s == "No human presence detected!" || s == "Network OK") return "NHP";
        if (sl.indexOf("object moving") != -1 || sl.indexOf("movement detected") != -1 || sl.indexOf("fast movement") != -1) return "MOV";
        if (sl.indexOf("signal stabilized") != -1 || sl == "measuring") return "STB";
        return "COM";
    }

    static int findOrCreateFireState(const char* key) {
        for (int i = 0; i < g_fireStateCount; i++) {
            if (strcmp(g_fireStates[i].key, key) == 0) return i;
        }
        if (g_fireStateCount < (int)(sizeof(g_fireStates) / sizeof(g_fireStates[0]))) {
            strncpy(g_fireStates[g_fireStateCount].key, key, sizeof(g_fireStates[g_fireStateCount].key) - 1);
            g_fireStates[g_fireStateCount].key[sizeof(g_fireStates[g_fireStateCount].key) - 1] = '\0';
            g_fireStates[g_fireStateCount].regionMask = 0;
            return g_fireStateCount++;
        }
        return -1;
    }

    static int findOrCreateFallState(const char* key) {
        for (int i = 0; i < g_fallStateCount; i++) {
            if (strcmp(g_fallStates[i].key, key) == 0) return i;
        }
        if (g_fallStateCount < (int)(sizeof(g_fallStates) / sizeof(g_fallStates[0]))) {
            strncpy(g_fallStates[g_fallStateCount].key, key, sizeof(g_fallStates[g_fallStateCount].key) - 1);
            g_fallStates[g_fallStateCount].key[sizeof(g_fallStates[g_fallStateCount].key) - 1] = '\0';
            g_fallStates[g_fallStateCount].active = false;
            return g_fallStateCount++;
        }
        return -1;
    }

    struct RuntimeBlobHeader {
        uint32_t magic;
        uint16_t version;
        uint16_t count;
        uint16_t recordSize;
        uint16_t reserved;
        uint32_t crc;
    };

    struct PersistedKeyIp {
        char key[24];
        char ipv6[40];
        uint8_t everSeen;
    };

    static const uint32_t RUNTIME_STATE_MAGIC = 0x4C4D5053; // LMPS
    static const uint16_t RUNTIME_STATE_VERSION = 1;

    static uint32_t runtimeStateCrc(const uint8_t* data, size_t len) {
        uint32_t crc = 0xFFFFFFFFu;
        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int bit = 0; bit < 8; bit++) {
                crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
            }
        }
        return ~crc;
    }

    static bool validateRuntimeBlob(const uint8_t* blob, size_t blobLen, size_t recordSize, uint16_t maxCount) {
        if (!blob || blobLen < sizeof(RuntimeBlobHeader)) return false;
        const RuntimeBlobHeader* header = reinterpret_cast<const RuntimeBlobHeader*>(blob);
        if (header->magic != RUNTIME_STATE_MAGIC || header->version != RUNTIME_STATE_VERSION ||
            header->recordSize != recordSize || header->count > maxCount) return false;
        size_t recordsLen = (size_t)header->count * recordSize;
        if (blobLen != sizeof(RuntimeBlobHeader) + recordsLen) return false;
        return header->crc == runtimeStateCrc(blob + sizeof(RuntimeBlobHeader), recordsLen);
    }

    static bool loadRuntimeState() {
        Preferences statePrefs;
        if (!statePrefs.begin("runtime", false, "state")) {
            Serial.println("[STATE] Dedicated NVS partition unavailable");
            return false;
        }

        if (statePrefs.isKey("cells")) statePrefs.remove("cells");

        int restoredCount = 0;
        size_t keysLen = statePrefs.getBytesLength("keys");
        if (keysLen >= sizeof(RuntimeBlobHeader)) {
            uint8_t* blob = (uint8_t*)malloc(keysLen);
            if (blob && statePrefs.getBytes("keys", blob, keysLen) == keysLen &&
                validateRuntimeBlob(blob, keysLen, sizeof(PersistedKeyIp), 256)) {
                const RuntimeBlobHeader* header = reinterpret_cast<const RuntimeBlobHeader*>(blob);
                const PersistedKeyIp* records = reinterpret_cast<const PersistedKeyIp*>(blob + sizeof(RuntimeBlobHeader));
                for (uint16_t i = 0; i < header->count; i++) {
                    if (!records[i].everSeen) continue;
                    upsertKeyIp(records[i].key, records[i].ipv6, false);
                    int keyIndex = findKeyIp(records[i].key);
                    if (keyIndex >= 0) g_keyIps[keyIndex].everSeen = true;

                    int cellIndex = findOrCreateCellStateForCheck(records[i].key);
                    if (cellIndex >= 0) {
                        strncpy(g_cellStates[cellIndex].code, "COM", sizeof(g_cellStates[cellIndex].code) - 1);
                        g_cellStates[cellIndex].code[sizeof(g_cellStates[cellIndex].code) - 1] = '\0';
                        g_cellStates[cellIndex].status[0] = '\0';
                    }
                    restoredCount++;
                }
                Serial.printf("[STATE] Restored %d connected node mappings as COM\n", restoredCount);
            } else {
                Serial.println("[STATE] Ignoring invalid node-mapping blob");
            }
            free(blob);
        }

        statePrefs.end();
        g_keyIpDirty = false;
        g_runtimeStateDirty = false;
        g_lastRuntimeStateSaveMs = millis();
        return restoredCount > 0;
    }

    static bool saveRuntimeState() {
        Preferences statePrefs;
        if (!statePrefs.begin("runtime", false, "state")) {
            Serial.println("[STATE] Failed to open dedicated NVS partition");
            return false;
        }

        if (statePrefs.isKey("cells")) statePrefs.remove("cells");

        bool keysSaved = false;
        int connectedCount = 0;
        for (int i = 0; i < g_keyIpCount; i++) {
            if (g_keyIps[i].everSeen) connectedCount++;
        }
        size_t keysRecordsLen = (size_t)connectedCount * sizeof(PersistedKeyIp);
        size_t keysBlobLen = sizeof(RuntimeBlobHeader) + keysRecordsLen;
        uint8_t* keysBlob = (uint8_t*)calloc(1, keysBlobLen);
        if (keysBlob) {
            RuntimeBlobHeader* header = reinterpret_cast<RuntimeBlobHeader*>(keysBlob);
            PersistedKeyIp* records = reinterpret_cast<PersistedKeyIp*>(keysBlob + sizeof(RuntimeBlobHeader));
            header->magic = RUNTIME_STATE_MAGIC;
            header->version = RUNTIME_STATE_VERSION;
            header->count = (uint16_t)connectedCount;
            header->recordSize = sizeof(PersistedKeyIp);
            int recordIndex = 0;
            for (int i = 0; i < g_keyIpCount; i++) {
                if (!g_keyIps[i].everSeen) continue;
                strncpy(records[recordIndex].key, g_keyIps[i].key, sizeof(records[recordIndex].key) - 1);
                strncpy(records[recordIndex].ipv6, g_keyIps[i].ipv6, sizeof(records[recordIndex].ipv6) - 1);
                records[recordIndex].everSeen = 1;
                recordIndex++;
            }
            header->crc = runtimeStateCrc(keysBlob + sizeof(RuntimeBlobHeader), keysRecordsLen);
            keysSaved = statePrefs.putBytes("keys", keysBlob, keysBlobLen) == keysBlobLen;
            free(keysBlob);
        }

        statePrefs.end();
        if (!keysSaved) {
            Serial.println("[STATE] Connected node mapping save failed");
            return false;
        }

        g_keyIpDirty = false;
        g_runtimeStateDirty = false;
        g_lastRuntimeStateSaveMs = millis();
        Serial.printf("[STATE] Saved %d connected node mappings\n", connectedCount);
        return true;
    }

    static bool parseFallPacketStatus(const char* statusText, bool* outDetected, bool* outCleared) {
        if (!statusText || !outDetected || !outCleared) return false;
        String s = String(statusText);
        s.trim();
        String sl = s;
        sl.toLowerCase();

        if (sl == "fall detected") {
            *outDetected = true;
            *outCleared = false;
            return true;
        }
        if (sl == "fall cleared") {
            *outDetected = false;
            *outCleared = true;
            return true;
        }
        return false;
    }

    static bool parseFirePacketStatus(const char* statusText, uint8_t* outMask, bool* outDetected) {
        if (!statusText || !outMask || !outDetected) return false;
        String s = statusText;
        s.trim();
        String sl = s;
        sl.toLowerCase();

        bool isDetected = false;
        int suffixLen = 0;
        if (sl.endsWith("_fire_detected")) {
            isDetected = true;
            suffixLen = 14;
        } else if (sl.endsWith("_normal")) {
            isDetected = false;
            suffixLen = 7;
        } else {
            return false;
        }

        String body = s.substring(0, s.length() - suffixLen);
        body.trim();
        while (body.endsWith("_")) body.remove(body.length() - 1);

        uint8_t mask = 0;
        int i = 0;
        while (i < body.length()) {
            int r = body.indexOf('R', i);
            if (r < 0 || r + 1 >= body.length()) break;
            char d = body.charAt(r + 1);
            if (d >= '1' && d <= '6') {
                mask |= (uint8_t)(1u << (d - '1'));
            }
            i = r + 2;
        }
        if (mask == 0) return false;

        *outMask = mask;
        *outDetected = isDetected;
        return true;
    }

    static bool upsertCellState(const char* key, const char* statusText) {
        if (!key || !strlen(key)) return false;
        String code = statusToCode(statusText);

        uint8_t fireMask = 0;
        bool fireDetected = false;
        bool isFirePacket = parseFirePacketStatus(statusText, &fireMask, &fireDetected);
        if (isFirePacket) {
            int fi = findOrCreateFireState(key);
            if (fi >= 0) {
                if (fireDetected) g_fireStates[fi].regionMask |= fireMask;
                else g_fireStates[fi].regionMask &= (uint8_t)(~fireMask);
                code = (g_fireStates[fi].regionMask != 0) ? "FIR" : "NHP";
            }
        }

        bool fallDetected = false;
        bool fallCleared = false;
        bool isFallPacket = parseFallPacketStatus(statusText, &fallDetected, &fallCleared);
        int fallIndex = findOrCreateFallState(key);
        if (isFallPacket && fallIndex >= 0) {
            if (fallDetected) g_fallStates[fallIndex].active = true;
            if (fallCleared) g_fallStates[fallIndex].active = false;
        }

        if (fallIndex >= 0 && g_fallStates[fallIndex].active) {
            code = "FAL";
        }

        for (int i = 0; i < g_cellStateCount; i++) {
            if (strcmp(g_cellStates[i].key, key) == 0) {
                bool changed = strcmp(g_cellStates[i].code, code.c_str()) != 0;
                strncpy(g_cellStates[i].status, statusText ? statusText : "", sizeof(g_cellStates[i].status) - 1);
                g_cellStates[i].status[sizeof(g_cellStates[i].status) - 1] = '\0';
                strncpy(g_cellStates[i].code, code.c_str(), sizeof(g_cellStates[i].code) - 1);
                g_cellStates[i].code[sizeof(g_cellStates[i].code) - 1] = '\0';
                g_cellStates[i].updatedMs = millis();
                g_cellStates[i].awaitingStatusCheck = false;
                g_cellStates[i].statusCheckDeadlineMs = 0;
                return changed;
            }
        }
        if (g_cellStateCount < (int)(sizeof(g_cellStates) / sizeof(g_cellStates[0]))) {
            strncpy(g_cellStates[g_cellStateCount].key, key, sizeof(g_cellStates[g_cellStateCount].key) - 1);
            g_cellStates[g_cellStateCount].key[sizeof(g_cellStates[g_cellStateCount].key) - 1] = '\0';
            strncpy(g_cellStates[g_cellStateCount].status, statusText ? statusText : "", sizeof(g_cellStates[g_cellStateCount].status) - 1);
            g_cellStates[g_cellStateCount].status[sizeof(g_cellStates[g_cellStateCount].status) - 1] = '\0';
            strncpy(g_cellStates[g_cellStateCount].code, code.c_str(), sizeof(g_cellStates[g_cellStateCount].code) - 1);
            g_cellStates[g_cellStateCount].code[sizeof(g_cellStates[g_cellStateCount].code) - 1] = '\0';
            g_cellStates[g_cellStateCount].updatedMs = millis();
            g_cellStates[g_cellStateCount].lastWebPushMs = 0;
            g_cellStates[g_cellStateCount].awaitingStatusCheck = false;
            g_cellStates[g_cellStateCount].statusCheckDeadlineMs = 0;
            g_cellStateCount++;
            return true;
        }
        return false;
    }

    static const char* getCellCodeByKey(const char* key) {
        if (!key) return "";
        for (int i = 0; i < g_cellStateCount; i++) {
            if (strcmp(g_cellStates[i].key, key) == 0) {
                return g_cellStates[i].code;
            }
        }
        return "";
    }

    static bool parseKeyStatusPacket(const char* payload, char* outSrc, size_t outSrcLen, char* outKey, size_t outKeyLen, char* outStatus, size_t outStatusLen) {
        if (!payload) return false;
        outSrc[0] = '\0'; outKey[0] = '\0'; outStatus[0] = '\0';

        const char* s0 = strstr(payload, "udpr<");
        if (s0) {
            s0 += 5;
            const char* e0 = strchr(s0, '>');
            if (e0) {
                int n = (int)(e0 - s0);
                if (n > 0 && n < (int)outSrcLen) { strncpy(outSrc, s0, n); outSrc[n] = '\0'; }
            }
        }

        const char* comma = strchr(payload, ',');
        if (!comma) return false;

        const char* kstart = comma;
        while (kstart > payload && *(kstart - 1) != ':' && *(kstart - 1) != '>' && *(kstart - 1) != ' ') kstart--;
        int kn = (int)(comma - kstart);
        if (kn <= 0 || kn >= (int)outKeyLen) return false;
        strncpy(outKey, kstart, kn);
        outKey[kn] = '\0';

        int d, h, sn;
        if (sscanf(outKey, "%d_%d_%d", &d, &h, &sn) != 3) return false;
        if (strncmp(outKey, "103_", 4) == 0) {
            char mapped[24];
            snprintf(mapped, sizeof(mapped), "501_%d_%d", h, sn);
            strncpy(outKey, mapped, outKeyLen - 1);
            outKey[outKeyLen - 1] = '\0';
        }

        const char* st = comma + 1;
        while (*st == ' ') st++;
        strncpy(outStatus, st, outStatusLen - 1);
        outStatus[outStatusLen - 1] = '\0';
        return true;
    }

    static bool parseAptSetupText(const String& text) {
        g_aptRowCount = 0;
        int pos = 0;
        while (pos < (int)text.length()) {
            int nl = text.indexOf('\n', pos);
            if (nl < 0) nl = text.length();
            String line = text.substring(pos, nl);
            line.trim();
            pos = nl + 1;
            if (!line.length()) continue;

            int d, s, e, c;
            if (sscanf(line.c_str(), "%d,%d,%d,%d", &d, &s, &e, &c) == 4) {
                if (g_aptRowCount < (int)(sizeof(g_aptRows) / sizeof(g_aptRows[0]))) {
                    g_aptRows[g_aptRowCount++] = {d, s, e, c};
                }
                continue;
            }
            if (line.indexOf(':') != -1) {
                strncpy(g_aptBrIPv6, line.c_str(), sizeof(g_aptBrIPv6) - 1);
                g_aptBrIPv6[sizeof(g_aptBrIPv6) - 1] = '\0';
                continue;
            }
            if (line.startsWith("/dev/") || line.startsWith("COM")) {
                strncpy(g_aptComPort, line.c_str(), sizeof(g_aptComPort) - 1);
                g_aptComPort[sizeof(g_aptComPort) - 1] = '\0';
                continue;
            }
            strncpy(g_aptName, line.c_str(), sizeof(g_aptName) - 1);
            g_aptName[sizeof(g_aptName) - 1] = '\0';
        }

        return g_aptRowCount > 0;
    }

    static bool loadAptSetup() {
        String text;

        // Use build-embedded APT.setup first so deployment does not depend on SPIFFS partition.
        const char* aptStart = &_binary_APT_setup_start[0];
        const char* aptEnd = &_binary_APT_setup_end[0];
        if (aptEnd > aptStart) {
            text = String(aptStart).substring(0, aptEnd - aptStart);
            if (parseAptSetupText(text)) {
                Serial.println("[APT] Loaded embedded APT.setup");
                return true;
            }
        }

        if (ensureSpiffsMounted()) {
            File f = SPIFFS.open("/APT.setup", "r");
            if (f) {
                text = f.readString();
                f.close();
                if (parseAptSetupText(text)) {
                    Serial.println("[APT] Loaded /APT.setup from SPIFFS");
                    return true;
                }
            }
        }

        text = String(DEFAULT_APT_SETUP);
        if (parseAptSetupText(text)) {
            Serial.println("[APT] Loaded built-in default layout");
            return true;
        }

        return false;
    }


#define HELMET_DIR_MAX 1024
#define HELMET_RAM_MAX 32
#define HELMET_CONNECTED_MS 20000UL

struct HelmetId {
    char ipv6[40];
    char alias[24];
    char phone[16];
    uint32_t lastSeenMs;
};
struct HelmetLive {
    char ipv6[40];
    char motion[12];
    char fall[8];
    char pulse[12];
    char body[16];
    char amb[16];
    char rh[12];
    char voc[12];
    char co[12];
    char nh3[12];
    char no2[12];
    uint32_t updatedMs;
};
static HelmetId* g_helmetDir = nullptr;
static int g_helmetDirCount = 0;
static int g_helmetDirStored = 0;
static HelmetLive* g_helmetLive = nullptr;
static int g_helmetLiveCount = 0;
static char g_selectedHelmet[40];
static bool g_helmetDirDirty = false;

static int helmetDirFileCount(void) {
    if (!ensureSpiffsMounted()) return 0;
    File f = SPIFFS.open("/helmet_dir.csv", "r");
    if (!f) return 0;
    int n = 0;
    while (f.available() && n < HELMET_DIR_MAX) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length()) n++;
    }
    f.close();
    return n;
}

static bool helmetDirFileLookup(const char* ip, char* alias, size_t aliasN, char* phone, size_t phoneN) {
    if (alias && aliasN) alias[0] = '\0';
    if (phone && phoneN) phone[0] = '\0';
    if (!ip || !ensureSpiffsMounted()) return false;
    File f = SPIFFS.open("/helmet_dir.csv", "r");
    if (!f) return false;
    bool found = false;
    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        int c1 = line.indexOf(',');
        if (c1 < 1) continue;
        if (line.substring(0, c1) != ip) continue;
        int c2 = line.indexOf(',', c1 + 1);
        String a = (c2 > c1) ? line.substring(c1 + 1, c2) : line.substring(c1 + 1);
        String ph = (c2 > c1) ? line.substring(c2 + 1) : "";
        if (alias && aliasN) strncpy(alias, a.c_str(), aliasN - 1);
        if (phone && phoneN) strncpy(phone, ph.c_str(), phoneN - 1);
        found = true;
        break;
    }
    f.close();
    return found;
}

static void helmetDirFileUpsert(const char* ip, const char* alias, const char* phone) {
    if (!ip || !ip[0] || !ensureSpiffsMounted()) return;
    File in = SPIFFS.open("/helmet_dir.csv", "r");
    File out = SPIFFS.open("/helmet_dir.tmp", "w");
    if (!out) {
        if (in) in.close();
        return;
    }
    bool replaced = false;
    int n = 0;
    if (in) {
        while (in.available() && n < HELMET_DIR_MAX) {
            String line = in.readStringUntil('\n');
            line.trim();
            if (!line.length()) continue;
            int c1 = line.indexOf(',');
            if (c1 > 0 && line.substring(0, c1) == ip) {
                out.printf("%s,%s,%s\n", ip, alias ? alias : "", phone ? phone : "");
                replaced = true;
            } else {
                out.println(line);
            }
            n++;
        }
        in.close();
    }
    if (!replaced && n < HELMET_DIR_MAX) {
        out.printf("%s,%s,%s\n", ip, alias ? alias : "", phone ? phone : "");
        n++;
    }
    out.close();
    SPIFFS.remove("/helmet_dir.csv");
    SPIFFS.rename("/helmet_dir.tmp", "/helmet_dir.csv");
    g_helmetDirStored = n;
}

static void helmetDirFileDelete(const char* ip) {
    if (!ip || !ip[0] || !ensureSpiffsMounted()) return;
    File in = SPIFFS.open("/helmet_dir.csv", "r");
    File out = SPIFFS.open("/helmet_dir.tmp", "w");
    if (!out) {
        if (in) in.close();
        return;
    }
    int n = 0;
    if (in) {
        while (in.available()) {
            String line = in.readStringUntil('\n');
            line.trim();
            if (!line.length()) continue;
            int c1 = line.indexOf(',');
            if (c1 > 0 && line.substring(0, c1) == ip) continue;
            out.println(line);
            n++;
        }
        in.close();
    }
    out.close();
    SPIFFS.remove("/helmet_dir.csv");
    SPIFFS.rename("/helmet_dir.tmp", "/helmet_dir.csv");
    g_helmetDirStored = n;
}

static void helmetRamEnsure(void) {
    if (!g_helmetDir) g_helmetDir = (HelmetId*)calloc(HELMET_RAM_MAX, sizeof(HelmetId));
    if (!g_helmetLive) g_helmetLive = (HelmetLive*)calloc(HELMET_RAM_MAX, sizeof(HelmetLive));
}

static int helmetDirFind(const char* ip) {
    if (!g_helmetDir || !ip) return -1;
    for (int i = 0; i < g_helmetDirCount; i++) {
        if (strcmp(g_helmetDir[i].ipv6, ip) == 0) return i;
    }
    return -1;
}

static void helmetDirSave(void) {
    if (!g_helmetDirDirty) return;
    for (int i = 0; i < g_helmetDirCount; i++) {
        helmetDirFileUpsert(g_helmetDir[i].ipv6, g_helmetDir[i].alias, g_helmetDir[i].phone);
    }
    g_helmetDirDirty = false;
}

static void helmetDirLoad(void) {
    g_helmetDirStored = helmetDirFileCount();
    Serial.printf("[HELMET] stored %d, ram cap %d\n", g_helmetDirStored, HELMET_RAM_MAX);
}

static int helmetDirNote(const char* ip) {
    if (!ip || !ip[0]) return -1;
    helmetRamEnsure();
    if (!g_helmetDir) return -1;
    int i = helmetDirFind(ip);
    if (i < 0) {
        if (g_helmetDirCount >= HELMET_RAM_MAX) return -1;
        i = g_helmetDirCount++;
        memset(&g_helmetDir[i], 0, sizeof(g_helmetDir[i]));
        strncpy(g_helmetDir[i].ipv6, ip, sizeof(g_helmetDir[i].ipv6) - 1);
        if (!helmetDirFileLookup(ip, g_helmetDir[i].alias, sizeof(g_helmetDir[i].alias), g_helmetDir[i].phone, sizeof(g_helmetDir[i].phone))) {
            helmetDirFileUpsert(ip, "", "");
        }
    } else if (!g_helmetDir[i].alias[0] && !g_helmetDir[i].phone[0]) {
        helmetDirFileLookup(ip, g_helmetDir[i].alias, sizeof(g_helmetDir[i].alias), g_helmetDir[i].phone, sizeof(g_helmetDir[i].phone));
    }
    g_helmetDir[i].lastSeenMs = millis();
    if (!g_selectedHelmet[0]) strncpy(g_selectedHelmet, ip, sizeof(g_selectedHelmet) - 1);
    strncpy(g_lastHelmetDst, ip, sizeof(g_lastHelmetDst) - 1);
    return i;
}

static int helmetLiveFind(const char* ip) {
    helmetRamEnsure();
    if (!g_helmetLive) return -1;
    for (int i = 0; i < g_helmetLiveCount; i++) {
        if (strcmp(g_helmetLive[i].ipv6, ip) == 0) return i;
    }
    if (g_helmetLiveCount >= HELMET_RAM_MAX) return 0;
    int i = g_helmetLiveCount++;
    memset(&g_helmetLive[i], 0, sizeof(g_helmetLive[i]));
    strncpy(g_helmetLive[i].ipv6, ip, sizeof(g_helmetLive[i].ipv6) - 1);
    return i;
}

struct HelmetReport {
    char src[48];
    char motion[16];
    char fall[16];
    char pulse[16];
    char body[24];
    char amb[24];
    char rh[24];
    char voc[24];
    char co[24];
    char nh3[24];
    char no2[24];
    unsigned long updatedMs;
};
static HelmetReport g_helmet = {};

static void helmetTake(const char* text, const char* key, char* dst, size_t n) {
    const char* p = strstr(text, key);
    if (!p) return;
    p += strlen(key);
    size_t i = 0;
    while (*p && *p != ',' && *p != '\r' && *p != '\n' && i + 1 < n) dst[i++] = *p++;
    dst[i] = '\0';
    if (i) g_helmet.updatedMs = millis();
}

static void ingestHelmetText(const char* src, const char* payload) {
    if (!payload || !payload[0]) return;
    const char* body = payload;
    const char* tag = strstr(payload, "udpr<");
    if (tag) {
        const char* ip = tag + 5;
        const char* end = strchr(ip, '>');
        if (end && end - ip < (int)sizeof(g_helmet.src)) {
            int n = (int)(end - ip);
            strncpy(g_helmet.src, ip, n);
            g_helmet.src[n] = '\0';
            strncpy(g_lastHelmetDst, g_helmet.src, sizeof(g_lastHelmetDst) - 1);
            helmetDirNote(g_helmet.src);
            body = end + 1;
            if (*body == ':') body++;
        }
    } else if (src && src[0]) {
        strncpy(g_helmet.src, src, sizeof(g_helmet.src) - 1);
        helmetDirNote(g_helmet.src);
    }
    helmetTake(body, "motion:", g_helmet.motion, sizeof(g_helmet.motion));
    {
        int li = helmetLiveFind(g_helmet.src);
        if (li >= 0) {
            strncpy(g_helmetLive[li].ipv6, g_helmet.src, sizeof(g_helmetLive[li].ipv6) - 1);
            g_helmetLive[li].updatedMs = millis();
        }
    }
    helmetTake(body, "fall:", g_helmet.fall, sizeof(g_helmet.fall));
    helmetTake(body, "pulse:", g_helmet.pulse, sizeof(g_helmet.pulse));
    helmetTake(body, "body:", g_helmet.body, sizeof(g_helmet.body));
    helmetTake(body, "amb:", g_helmet.amb, sizeof(g_helmet.amb));
    helmetTake(body, "rh:", g_helmet.rh, sizeof(g_helmet.rh));
    helmetTake(body, "voc:", g_helmet.voc, sizeof(g_helmet.voc));
    helmetTake(body, "co:", g_helmet.co, sizeof(g_helmet.co));
    helmetTake(body, "nh3:", g_helmet.nh3, sizeof(g_helmet.nh3));
    helmetTake(body, "no2:", g_helmet.no2, sizeof(g_helmet.no2));
    {
        int li = helmetLiveFind(g_helmet.src);
        if (li >= 0 && g_helmet.src[0]) {
            strncpy(g_helmetLive[li].motion, g_helmet.motion, sizeof(g_helmetLive[li].motion) - 1);
            strncpy(g_helmetLive[li].fall, g_helmet.fall, sizeof(g_helmetLive[li].fall) - 1);
            strncpy(g_helmetLive[li].pulse, g_helmet.pulse, sizeof(g_helmetLive[li].pulse) - 1);
            strncpy(g_helmetLive[li].body, g_helmet.body, sizeof(g_helmetLive[li].body) - 1);
            strncpy(g_helmetLive[li].amb, g_helmet.amb, sizeof(g_helmetLive[li].amb) - 1);
            strncpy(g_helmetLive[li].rh, g_helmet.rh, sizeof(g_helmetLive[li].rh) - 1);
            strncpy(g_helmetLive[li].voc, g_helmet.voc, sizeof(g_helmetLive[li].voc) - 1);
            strncpy(g_helmetLive[li].co, g_helmet.co, sizeof(g_helmetLive[li].co) - 1);
            strncpy(g_helmetLive[li].nh3, g_helmet.nh3, sizeof(g_helmetLive[li].nh3) - 1);
            strncpy(g_helmetLive[li].no2, g_helmet.no2, sizeof(g_helmetLive[li].no2) - 1);
            g_helmetLive[li].updatedMs = millis();
        }
    }
}

static const char HELMET_HTML[] =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Smart Helmet</title><style>"
    "body{font-family:sans-serif;background:#101418;color:#e8eef2;margin:0}"
    "header{padding:16px 20px;background:#182028}"
    "h1{margin:0;font-size:22px}"
    ".row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-top:10px}"
    "select,input{background:#101820;color:#e8eef2;border:1px solid #345;border-radius:8px;padding:8px;min-width:180px}"
    "button{border-radius:8px;padding:8px 14px;font-weight:700;cursor:pointer}"
    "#btnDataReset{background:#166534;color:#fff;border:1px solid #86efac}"
    "#save{background:#1d4ed8;color:#fff;border:1px solid #93c5fd}"
"#del{background:#7f1d1d;color:#fff;border:1px solid #fca5a5}"
    ".meta{color:#9ab;font-size:13px;margin-top:8px}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:12px;padding:16px}"
    ".card{background:#1c2630;border-radius:10px;padding:14px}"
    ".k{color:#8aa;font-size:12px;text-transform:uppercase}.v{font-size:28px;margin-top:6px}"
    ".bad{color:#f66}.warn{color:#fc6}.ok{color:#6d6}"
    "</style></head><body><header><h1>Smart Helmet</h1>"
    "<div class='row'><select id='sel'></select>"
    "<input id='alias' placeholder='별명'>"
    "<input id='phone' placeholder='전화번호'>"
    "<button id='save' type='button'>저장</button>"
    "<button id='del' type='button'>삭제</button>"
    "<button id='btnDataReset' type='button'>초기화</button></div>"
    "<div class='meta' id='meta'>waiting</div></header><div class='grid' id='grid'></div>"
    "<script>"
    "const keys=['motion','fall','pulse','body','amb','rh','voc','co','nh3','no2'];"
    "let selected='';"
    "function cls(k,v){if(k=='fall'&&v=='yes')return 'bad';if(k=='pulse'&&(v=='rising'||v=='falling'))return 'warn';return 'ok';}"
    "function paint(d){var cur=(window._items||[]).find(it=>it.ipv6==(d.src||selected))||{};var who=(cur.alias||'')+(cur.phone?(' '+cur.phone):'');document.getElementById('meta').textContent=(who?who+' / ':'')+(d.src||'-')+(d.has_data?('  '+d.age_s+'s ago'):'');"
    "document.getElementById('grid').innerHTML=keys.map(k=>'<div class=\"card\"><div class=\"k\">'+k+'</div><div class=\"v '+cls(k,d[k]||'')+'\">'+(d[k]||'-')+'</div></div>').join('');}"
    "function loadOne(){var u=selected?('/api/helmet?src='+encodeURIComponent(selected)):'/api/helmet';fetch(u).then(r=>r.json()).then(paint).catch(()=>{});}"
    "function fillMeta(){const cur=(window._items||[]).find(it=>it.ipv6==selected)||{};document.getElementById('alias').value=cur.alias||'';document.getElementById('phone').value=cur.phone||'';}"
    "function loadList(){fetch('/api/helmets').then(r=>r.json()).then(j=>{"
    "window._items=j.items||[];const sel=document.getElementById('sel');const prev=selected||sel.value;"
    "sel.innerHTML='';window._items.forEach(it=>{const o=document.createElement('option');o.value=it.ipv6;o.textContent=(it.alias?it.alias+' / ':'')+it.ipv6;sel.appendChild(o);});"
    "if(prev && Array.from(sel.options).some(o=>o.value==prev)) sel.value=prev; else if(sel.options.length) sel.value=sel.options[0].value;"
    "const changed=selected!==sel.value;selected=sel.value;"
    "const typing=document.activeElement&&(document.activeElement.id=='alias'||document.activeElement.id=='phone');"
    "if(changed||!typing)fillMeta();loadOne();"
    "}).catch(()=>{});}"
    "document.getElementById('sel').onchange=function(){selected=this.value;fillMeta();loadOne();};"
    "function saveMeta(){if(!selected)return;fetch('/api/helmet_meta?src='+encodeURIComponent(selected)+'&alias='+encodeURIComponent(document.getElementById('alias').value)+'&phone='+encodeURIComponent(document.getElementById('phone').value)).then(r=>r.json()).then(j=>{document.getElementById('meta').textContent=(j&&j.msg)?j.msg:'saved';});}"
    "document.getElementById('save').onclick=saveMeta;"
    "document.getElementById('del').onclick=function(){if(!selected)return;fetch('/api/helmet_delete?src='+encodeURIComponent(selected)).then(()=>{selected='';loadList();});};"
    "document.getElementById('btnDataReset').onclick=function(){if(!selected){document.getElementById('meta').textContent='no selected helmet';return;}"
    "fetch('/api/data_reset?src='+encodeURIComponent(selected)).then(r=>r.json()).then(j=>{document.getElementById('meta').textContent=(j&&j.msg)?j.msg:'no reply';}).catch(()=>{document.getElementById('meta').textContent='reset failed';});};"
    "loadList();setInterval(loadList,2000);</script></body></html>";

static HelmetLive* helmetLiveBySrc(const char* src) {
    if (!g_helmetLive || !src || !src[0]) return nullptr;
    for (int i = 0; i < g_helmetLiveCount; i++) {
        if (strcmp(g_helmetLive[i].ipv6, src) == 0) return &g_helmetLive[i];
    }
    return nullptr;
}


static int hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void urlDecode(char* s) {
    if (!s) return;
    char* w = s;
    for (char* r = s; *r; ) {
        if (*r == '%' && hexVal(r[1]) >= 0 && hexVal(r[2]) >= 0) {
            *w++ = (char)((hexVal(r[1]) << 4) | hexVal(r[2]));
            r += 3;
        } else if (*r == '+') {
            *w++ = ' ';
            r++;
        } else {
            *w++ = *r++;
        }
    }
    *w = '\0';
}

static bool helmetListHas(const String& body, const char* ip) {
    String needle = String("\"ipv6\":\"") + ip + "\"";
    return body.indexOf(needle) >= 0;
}

static esp_err_t webHelmetsHandler(httpd_req_t* req) {
    String body = "{\"max\":1024,\"count\":" + String(g_helmetDirStored) + ",\"items\":[";
    bool first = true;
    if (ensureSpiffsMounted()) {
        File f = SPIFFS.open("/helmet_dir.csv", "r");
        if (f) {
            while (f.available()) {
                String line = f.readStringUntil('\n');
                line.trim();
                int c1 = line.indexOf(',');
                if (c1 < 1) continue;
                String ip = line.substring(0, c1);
                int c2 = line.indexOf(',', c1 + 1);
                String alias = (c2 > c1) ? line.substring(c1 + 1, c2) : line.substring(c1 + 1);
                String phone = (c2 > c1) ? line.substring(c2 + 1) : "";
                if (!first) body += ",";
                first = false;
                body += "{\"ipv6\":\"" + jsonEscape(ip.c_str()) + "\"";
                body += ",\"alias\":\"" + jsonEscape(alias.c_str()) + "\"";
                body += ",\"phone\":\"" + jsonEscape(phone.c_str()) + "\"}";
            }
            f.close();
        }
    }
    if (g_helmetDir) {
        for (int i = 0; i < g_helmetDirCount; i++) {
            if (!g_helmetDir[i].ipv6[0] || helmetListHas(body, g_helmetDir[i].ipv6)) continue;
            if (!first) body += ",";
            first = false;
            body += "{\"ipv6\":\"" + jsonEscape(g_helmetDir[i].ipv6) + "\"";
            body += ",\"alias\":\"" + jsonEscape(g_helmetDir[i].alias) + "\"";
            body += ",\"phone\":\"" + jsonEscape(g_helmetDir[i].phone) + "\"}";
        }
    }
    if (g_helmet.src[0] && !helmetListHas(body, g_helmet.src)) {
        if (!first) body += ",";
        body += "{\"ipv6\":\"" + jsonEscape(g_helmet.src) + "\",\"alias\":\"\",\"phone\":\"\"}";
    }
    body += "]}";
    return sendWebResponse(req, "application/json", body.c_str(), body.length(), "helmets");
}

static esp_err_t webHelmetMetaHandler(httpd_req_t* req) {
    char q[160] = "";
    char src[48] = "";
    char alias[24] = "";
    char phone[16] = "";
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        httpd_query_key_value(q, "src", src, sizeof(src));
        httpd_query_key_value(q, "alias", alias, sizeof(alias));
        httpd_query_key_value(q, "phone", phone, sizeof(phone));
        urlDecode(src);
        urlDecode(alias);
        urlDecode(phone);
    }
    int i = helmetDirFind(src);
    if (i < 0) helmetDirNote(src);
    i = helmetDirFind(src);
    if (i < 0) {
        Serial.printf("[HELMET] save fail src=%s reason=unknown\n", src);
        return sendOkJson(req, false, "unknown helmet");
    }
    strncpy(g_helmetDir[i].alias, alias, sizeof(g_helmetDir[i].alias) - 1);
    g_helmetDir[i].alias[sizeof(g_helmetDir[i].alias) - 1] = '\0';
    strncpy(g_helmetDir[i].phone, phone, sizeof(g_helmetDir[i].phone) - 1);
    g_helmetDir[i].phone[sizeof(g_helmetDir[i].phone) - 1] = '\0';
    g_helmetDirDirty = true;
    helmetDirSave();
    strncpy(g_selectedHelmet, src, sizeof(g_selectedHelmet) - 1);
    Serial.printf("[HELMET] save ok src=%s alias=%s phone=%s\n", src, alias, phone);
    return sendOkJson(req, true, "saved");
}

static esp_err_t webHelmetDeleteHandler(httpd_req_t* req) {
    char q[96] = "";
    char src[48] = "";
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        httpd_query_key_value(q, "src", src, sizeof(src));
        urlDecode(src);
    }
    int i = helmetDirFind(src);
    if (i < 0) helmetDirNote(src);
    i = helmetDirFind(src);
    if (i < 0) {
        Serial.printf("[HELMET] delete fail src=%s reason=unknown\n", src);
        return sendOkJson(req, false, "unknown helmet");
    }
    helmetDirFileDelete(src);
    Serial.printf("[HELMET] delete ok src=%s\n", src);
    for (int j = i; j + 1 < g_helmetDirCount; j++) g_helmetDir[j] = g_helmetDir[j + 1];
    g_helmetDirCount--;
    memset(&g_helmetDir[g_helmetDirCount], 0, sizeof(g_helmetDir[0]));
    if (g_helmetLive) {
        for (int j = 0; j < g_helmetLiveCount; j++) {
            if (strcmp(g_helmetLive[j].ipv6, src) == 0) {
                memset(&g_helmetLive[j], 0, sizeof(g_helmetLive[j]));
            }
        }
    }
    g_helmetDirDirty = true;
    helmetDirSave();
    if (strcmp(g_selectedHelmet, src) == 0) g_selectedHelmet[0] = '\0';
    return sendOkJson(req, true, "deleted");
}

static esp_err_t webHelmetApiHandler(httpd_req_t* req) {
    char q[96] = "";
    char src[48] = "";
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        httpd_query_key_value(q, "src", src, sizeof(src));
        urlDecode(src);
    }
    if (!src[0]) strncpy(src, g_helmet.src[0] ? g_helmet.src : g_selectedHelmet, sizeof(src) - 1);
    if (src[0]) strncpy(g_selectedHelmet, src, sizeof(g_selectedHelmet) - 1);
    HelmetLive* live = helmetLiveBySrc(src);
    if (!live && g_helmet.src[0] && strcmp(src, g_helmet.src) == 0) {
        int li = helmetLiveFind(g_helmet.src);
        live = helmetLiveBySrc(src);
        (void)li;
    }
    unsigned long age = (live && live->updatedMs) ? (millis() - live->updatedMs) / 1000UL : 0;
    String body = "{";
    body += "\"src\":\"" + jsonEscape(src) + "\"";
    body += ",\"motion\":\"" + jsonEscape(live ? live->motion : "") + "\"";
    body += ",\"fall\":\"" + jsonEscape(live ? live->fall : "") + "\"";
    body += ",\"pulse\":\"" + jsonEscape(live ? live->pulse : "") + "\"";
    body += ",\"body\":\"" + jsonEscape(live ? live->body : "") + "\"";
    body += ",\"amb\":\"" + jsonEscape(live ? live->amb : "") + "\"";
    body += ",\"rh\":\"" + jsonEscape(live ? live->rh : "") + "\"";
    body += ",\"voc\":\"" + jsonEscape(live ? live->voc : "") + "\"";
    body += ",\"co\":\"" + jsonEscape(live ? live->co : "") + "\"";
    body += ",\"nh3\":\"" + jsonEscape(live ? live->nh3 : "") + "\"";
    body += ",\"no2\":\"" + jsonEscape(live ? live->no2 : "") + "\"";
    body += ",\"age_s\":" + String(live ? age : 0);
    body += ",\"has_data\":" + String(live && live->updatedMs ? "true" : "false");
    body += "}";
    return sendWebResponse(req, "application/json", body.c_str(), body.length(), "helmet");
}

static esp_err_t webIndexHandler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, HELMET_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t webLegacyHandler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, WEB_INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t webStatusHandler(httpd_req_t* req) {
        String body = buildWsJson("", "", "", true);
    return sendWebResponse(req, "application/json", body.c_str(), body.length(), "status");
}

static String buildCellsJson() {
    unsigned long now = millis();
    String s = "{";
    s += "\"cells\":[";
    for (int i = 0; i < g_cellStateCount; i++) {
        const char* codeOut = g_cellStates[i].code;
        bool statusTimedOut = g_cellStates[i].awaitingStatusCheck &&
                              (long)(now - g_cellStates[i].statusCheckDeadlineMs) >= 0;
        if (statusTimedOut) {
            codeOut = "COM";
        }
        if (i) s += ",";
        s += "{";
        s += "\"key\":\"" + jsonEscape(g_cellStates[i].key) + "\"";
        s += ",\"code\":\"" + jsonEscape(codeOut) + "\"";
        int keyIndex = findKeyIp(g_cellStates[i].key);
        s += ",\"everSeen\":" + String((keyIndex >= 0 && g_keyIps[keyIndex].everSeen) ? "true" : "false");
        s += "}";
    }
    s += "]}";
    return s;
}

static esp_err_t webCellsHandler(httpd_req_t* req) {
    String body = buildCellsJson();
    return sendWebResponse(req, "application/json", body.c_str(), body.length(), "cells");
}

static esp_err_t webLayoutHandler(httpd_req_t* req) {
    int maxFloor = 1;
    String body = "{";
    body += "\"aptName\":\"" + jsonEscape(g_aptName) + "\"";
    body += ",\"rows\":[";
    for (int i = 0; i < g_aptRowCount; i++) {
        if (i) body += ",";
        body += "{";
        body += "\"dong\":" + String(g_aptRows[i].dong);
        body += ",\"startHo\":" + String(g_aptRows[i].startHo);
        body += ",\"endHo\":" + String(g_aptRows[i].endHo);
        body += ",\"sensorCount\":" + String(g_aptRows[i].sensorCount);
        body += "}";
        int ef = g_aptRows[i].endHo / 100;
        if (ef > maxFloor) maxFloor = ef;
    }
    body += "]";
    body += ",\"maxFloor\":" + String(maxFloor);
    body += "}";
    return sendWebResponse(req, "application/json", body.c_str(), body.length(), "layout");
}

static esp_err_t webSocketHandler(httpd_req_t* req) {
    if (req->method == HTTP_GET) {
        timeval sendTimeout = { .tv_sec = 0, .tv_usec = 250000 };
        setsockopt(httpd_req_to_sockfd(req), SOL_SOCKET, SO_SNDTIMEO,
               &sendTimeout, sizeof(sendTimeout));
        return ESP_OK;
    }
        httpd_ws_frame_t frame = {};
        frame.type = HTTPD_WS_TYPE_TEXT;
        esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
        if (ret != ESP_OK) return ret;
        if (frame.len > 0) {
                uint8_t* buf = (uint8_t*)malloc(frame.len + 1);
                if (!buf) return ESP_ERR_NO_MEM;
                frame.payload = buf;
                ret = httpd_ws_recv_frame(req, &frame, frame.len);
            if (ret == ESP_OK) {
                buf[frame.len] = '\0';
                if (frame.type == HTTPD_WS_TYPE_TEXT && strcmp((char*)buf, "ping") == 0) {
                    static const char pong[] = "pong";
                    httpd_ws_frame_t reply = {};
                    reply.type = HTTPD_WS_TYPE_TEXT;
                    reply.payload = (uint8_t*)pong;
                    reply.len = sizeof(pong) - 1;
                    if (g_wsSendMutex && xSemaphoreTake(g_wsSendMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
                        ret = httpd_ws_send_frame(req, &reply);
                        xSemaphoreGive(g_wsSendMutex);
                    } else {
                        ret = ESP_ERR_TIMEOUT;
                    }
                }
            }
            free(buf);
            if (ret != ESP_OK) return ret;
        }
        return ESP_OK;
}

void notifyWebClients(const char* srcAddr, const char* payload, const char* rawLine, const char* key, const char* statusText, const char* code) {
        if (!g_httpServer) return;

    String json = buildWsJson(srcAddr, payload, rawLine, false);
    if (json.endsWith("}")) json.remove(json.length() - 1);
    json += ",\"key\":\"" + jsonEscape(key ? key : "") + "\"";
    json += ",\"statusText\":\"" + jsonEscape(statusText ? statusText : "") + "\"";
    json += ",\"code\":\"" + jsonEscape(code ? code : "") + "\"";
    int keyIndex = findKeyIp(key);
    json += ",\"everSeen\":" + String((keyIndex >= 0 && g_keyIps[keyIndex].everSeen) ? "true" : "false");
    json += "}";
        httpd_ws_frame_t wsFrame = {};
        wsFrame.type = HTTPD_WS_TYPE_TEXT;
        wsFrame.payload = (uint8_t*)json.c_str();
        wsFrame.len = json.length();

        int fds[16] = {0};
        size_t count = sizeof(fds) / sizeof(fds[0]);
        if (httpd_get_client_list(g_httpServer, &count, fds) != ESP_OK) return;

        static int wsSendErrorCount = 0;
        for (size_t i = 0; i < count; i++) {
            serviceWatchdog();
            if (httpd_ws_get_fd_info(g_httpServer, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
                esp_err_t ret = ESP_ERR_TIMEOUT;
                if (g_wsSendMutex && xSemaphoreTake(g_wsSendMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
                    ret = httpd_ws_send_frame_async(g_httpServer, fds[i], &wsFrame);
                    xSemaphoreGive(g_wsSendMutex);
                }
                if (ret != ESP_OK) {
                    wsSendErrorCount++;
                    Serial.printf("[WEB] ws_send_frame_async failed (fd=%d, err=%d), closing socket. Total errors: %d\n", fds[i], (int)ret, wsSendErrorCount);
                    httpd_sess_trigger_close(g_httpServer, fds[i]);
                    // Keep service alive: close bad socket, but do not stop server in data path.
                    if (wsSendErrorCount >= 10) {
                        Serial.println("[WEB] Frequent WebSocket send errors; keeping server alive and dropping bad sockets.");
                        wsSendErrorCount = 0;
                    }
                }
            }
        }
}

    static void serviceWebServerHealth(unsigned long now) {
        static const unsigned long WEB_HEALTH_CHECK_INTERVAL_MS = 5000;
        static const unsigned long WEB_CLIENT_PRESSURE_TIMEOUT_MS = 30000;
        static const size_t WEB_CLIENT_PRESSURE_LIMIT = 11;

        if (!g_httpServer) return;
        if (g_webRecoveryRequested) {
            Serial.println("[WEB] restarting HTTP server after slow response");
            g_webRecoveryActive = true;
            esp_err_t stopResult = httpd_stop(g_httpServer);
            if (stopResult == ESP_OK) {
                g_httpServer = nullptr;
                g_nextWebRetryMs = now + 1000;
                g_lastWebClientCount = -1;
                g_webRecoveryRequested = false;
            } else {
                Serial.printf("[WEB] HTTP server stop failed: %s\n", esp_err_to_name(stopResult));
            }
            return;
        }
        if ((now - g_lastWebHealthCheckMs) < WEB_HEALTH_CHECK_INTERVAL_MS) return;
        g_lastWebHealthCheckMs = now;

        int fds[16] = {0};
        size_t count = sizeof(fds) / sizeof(fds[0]);
        esp_err_t result = httpd_get_client_list(g_httpServer, &count, fds);
        if (result != ESP_OK) {
            Serial.printf("[WEB] health client list failed: %s\n", esp_err_to_name(result));
            return;
        }

        if ((int)count != g_lastWebClientCount) {
            g_lastWebClientCount = (int)count;
            Serial.printf("[WEB] active clients=%u\n", (unsigned)count);
        }

        if (count < WEB_CLIENT_PRESSURE_LIMIT) {
            g_webClientPressureSinceMs = 0;
            return;
        }

        if (g_webClientPressureSinceMs == 0) {
            g_webClientPressureSinceMs = now;
            Serial.printf("[WEB] client pressure detected: %u clients\n", (unsigned)count);
            return;
        }

        if ((now - g_webClientPressureSinceMs) < WEB_CLIENT_PRESSURE_TIMEOUT_MS) return;

        Serial.printf("[WEB] restarting HTTP server after %u clients stayed active for %lums\n",
                  (unsigned)count, now - g_webClientPressureSinceMs);
        g_webRecoveryActive = true;
        esp_err_t stopResult = httpd_stop(g_httpServer);
        if (stopResult == ESP_OK) {
            g_httpServer = nullptr;
            g_nextWebRetryMs = now + 1000;
            g_lastWebClientCount = -1;
        } else {
            Serial.printf("[WEB] HTTP server stop failed: %s\n", esp_err_to_name(stopResult));
        }
        g_webClientPressureSinceMs = 0;
    }

void startWebServer() {
        if (g_httpServer) return;

        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.server_port = 80;
        config.max_uri_handlers = 20;
    // LWIP_MAX_SOCKETS=16; httpd uses 3 internal, leaving room for 13 sessions.
    // Keep one slot available so new UI requests can still reach the server.
    config.max_open_sockets = 12;
    config.backlog_conn = 8;
    config.lru_purge_enable = true;
    config.send_wait_timeout = 10;

        esp_err_t startResult = httpd_start(&g_httpServer, &config);
        if (startResult != ESP_OK) {
            g_httpServer = nullptr;
            if (g_webRecoveryActive && ++g_webStartFailureCount >= 3) {
                Serial.printf("[WEB] HTTP recovery failed %u times; restarting device\n",
                              g_webStartFailureCount);
                delay(100);
                ESP.restart();
            }
            Serial.println("[WEB] Failed to start HTTP server. Will retry in 5s.");
            g_nextWebRetryMs = millis() + 5000;
                return;
        }
        g_webRecoveryActive = false;
        g_webStartFailureCount = 0;
        g_lastWebHealthCheckMs = millis();
        g_webClientPressureSinceMs = 0;
        g_lastWebClientCount = -1;

        httpd_uri_t uIndex = {
                .uri = "/",
                .method = HTTP_GET,
                .handler = webIndexHandler,
                .user_ctx = nullptr
        };
        httpd_uri_t uStatus = {
                .uri = "/api/status",
                .method = HTTP_GET,
                .handler = webStatusHandler,
                .user_ctx = nullptr
        };
        httpd_uri_t uCells = {
            .uri = "/api/cells",
            .method = HTTP_GET,
            .handler = webCellsHandler,
            .user_ctx = nullptr
        };
        httpd_uri_t uLayout = {
            .uri = "/api/layout",
            .method = HTTP_GET,
            .handler = webLayoutHandler,
            .user_ctx = nullptr
        };
        httpd_uri_t uWs = {
                .uri = "/ws",
                .method = HTTP_GET,
                .handler = webSocketHandler,
                .user_ctx = nullptr,
                .is_websocket = true,
                .handle_ws_control_frames = true,
                .supported_subprotocol = nullptr
        };
        httpd_uri_t uFireClear = {
            .uri = "/api/fire_clear",
            .method = HTTP_GET,
            .handler = webFireClearHandler,
            .user_ctx = nullptr
        };
        httpd_uri_t uDataReset = {
            .uri = "/api/data_reset",
            .method = HTTP_GET,
            .handler = handleDataReset,
            .user_ctx = nullptr
        };
        httpd_uri_t uHelmets = {
            .uri = "/api/helmets",
            .method = HTTP_GET,
            .handler = webHelmetsHandler,
            .user_ctx = nullptr
        };
        httpd_uri_t uHelmetMeta = {
            .uri = "/api/helmet_meta",
            .method = HTTP_GET,
            .handler = webHelmetMetaHandler,
            .user_ctx = nullptr
        };
        httpd_uri_t uHelmetDelete = {
            .uri = "/api/helmet_delete",
            .method = HTTP_GET,
            .handler = webHelmetDeleteHandler,
            .user_ctx = nullptr
        };
        httpd_uri_t uCheckInterval = {
            .uri = "/api/check_interval",
            .method = HTTP_GET,
            .handler = webCheckIntervalHandler,
            .user_ctx = nullptr
        };
        httpd_uri_t uStatusCheck = {
            .uri = "/api/status_check",
            .method = HTTP_GET,
            .handler = webStatusCheckHandler,
            .user_ctx = nullptr
        };
        httpd_uri_t uNetworkCheck = {
            .uri = "/api/network_check",
            .method = HTTP_GET,
            .handler = webNetworkCheckHandler,
            .user_ctx = nullptr
        };
        httpd_uri_t uPacketStream = {
            .uri = "/api/packet_stream",
            .method = HTTP_GET,
            .handler = webPacketStreamHandler,
            .user_ctx = nullptr
        };
        httpd_uri_t uFwVersion = {
            .uri = "/api/fw_version",
            .method = HTTP_GET,
            .handler = webFirmwareVersionHandler,
            .user_ctx = nullptr
        };
        httpd_uri_t uOtaUpdate = {
            .uri = "/api/ota_update",
            .method = HTTP_POST,
            .handler = webOtaUpdateHandler,
            .user_ctx = nullptr
        };

        httpd_uri_t uHelmet = {
            .uri = "/api/helmet",
            .method = HTTP_GET,
            .handler = webHelmetApiHandler,
            .user_ctx = nullptr
        };
        httpd_uri_t uLegacy = {
            .uri = "/legacy",
            .method = HTTP_GET,
            .handler = webLegacyHandler,
            .user_ctx = nullptr
        };
        httpd_register_uri_handler(g_httpServer, &uIndex);
        httpd_register_uri_handler(g_httpServer, &uHelmet);
        httpd_register_uri_handler(g_httpServer, &uLegacy);
        httpd_register_uri_handler(g_httpServer, &uStatus);
        httpd_register_uri_handler(g_httpServer, &uCells);
        httpd_register_uri_handler(g_httpServer, &uLayout);
        httpd_register_uri_handler(g_httpServer, &uWs);
        httpd_register_uri_handler(g_httpServer, &uFireClear);
        httpd_register_uri_handler(g_httpServer, &uDataReset);
        httpd_register_uri_handler(g_httpServer, &uHelmets);
        httpd_register_uri_handler(g_httpServer, &uHelmetMeta);
        httpd_register_uri_handler(g_httpServer, &uHelmetDelete);
        httpd_register_uri_handler(g_httpServer, &uCheckInterval);
        httpd_register_uri_handler(g_httpServer, &uStatusCheck);
        httpd_register_uri_handler(g_httpServer, &uNetworkCheck);
        httpd_register_uri_handler(g_httpServer, &uPacketStream);
        httpd_register_uri_handler(g_httpServer, &uFwVersion);
        httpd_register_uri_handler(g_httpServer, &uOtaUpdate);

        g_nextWebRetryMs = 0;
        IPAddress staIp = WiFi.localIP();
        IPAddress apIp = WiFi.softAPIP();
        if (hasValidStaIp()) {
            Serial.printf("[WEB] UI ready (STA): http://%s/\n", staIp.toString().c_str());
        }
        if (static_cast<uint32_t>(apIp) != 0) {
            Serial.printf("[WEB] UI ready (AP):  http://%s/\n", apIp.toString().c_str());
        }
}

// ============================================================================
//  Watchdog
// ============================================================================
void registerWatchdogForMainTask() {
    // Intentionally no-op: avoid forcing task WDT registration from app code.
}
inline void serviceWatchdog(bool allowYield) {
    if (allowYield) taskYIELD();
}

// ============================================================================
//  LED
// ============================================================================
void setLEDColor(const char* color) {
    if      (strcmp(color, "red")    == 0) { digitalWrite(RED_LED_PIN, HIGH); digitalWrite(GREEN_LED_PIN, LOW);  }
    else if (strcmp(color, "green")  == 0) { digitalWrite(RED_LED_PIN, LOW);  digitalWrite(GREEN_LED_PIN, HIGH); }
    else if (strcmp(color, "yellow") == 0) { digitalWrite(RED_LED_PIN, HIGH); digitalWrite(GREEN_LED_PIN, HIGH); }
    else                                   { digitalWrite(RED_LED_PIN, LOW);  digitalWrite(GREEN_LED_PIN, LOW);  }
}
void blinkGreenTwice() {
    for (int i = 0; i < 2; i++) { setLEDColor("green"); delay(200); setLEDColor("off"); delay(200); }
    setLEDColor("green");
}

// ============================================================================
//  OLED Display (SSD1306 128x64, I2C) — 4-line status
// ============================================================================
#define OLED_SDA_PIN  21
#define OLED_SCL_PIN  22
#define OLED_I2C_ADDR 0x3C
#define OLED_WIDTH    128
#define OLED_HEIGHT   64
#define OLED_PAGES    (OLED_HEIGHT / 8)

static uint8_t oledBuffer[OLED_WIDTH * OLED_PAGES];
static bool    oledReady = false;
static i2c_master_bus_handle_t oledI2cBus = NULL;
static i2c_master_dev_handle_t oledI2cDev = NULL;

// Standard 5x7 font, columns for ASCII 0x20-0x7E (space..~)
static const uint8_t oledFont5x7[95][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x56,0x20,0x50}, {0x00,0x08,0x07,0x03,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14}, {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78}, {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78}, {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C}, {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00}, {0x08,0x04,0x08,0x10,0x08},
};

// A full 33-byte SSD1306 I2C transaction at 400kHz takes well under 1ms
// (33 bytes * 9 bits/byte * 2.5us/bit ~= 0.74ms plus start/stop overhead).
// 20ms gives a >20x margin for a healthy bus/slave while still bounding
// worst-case blocking if the panel is disconnected or the bus is stuck.
#define OLED_I2C_TIMEOUT_MS 20

static esp_err_t oledCmd(uint8_t c) {
    if (!oledI2cDev) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = { 0x00, c }; // control byte: command
    return i2c_master_transmit(oledI2cDev, buf, sizeof(buf), OLED_I2C_TIMEOUT_MS);
}

static esp_err_t oledData(const uint8_t* data, size_t len) {
    if (!oledI2cDev) return ESP_ERR_INVALID_STATE;
    const size_t CHUNK = 32;
    uint8_t buf[CHUNK + 1];
    for (size_t i = 0; i < len; i += CHUNK) {
        size_t n = (len - i > CHUNK) ? CHUNK : (len - i);
        buf[0] = 0x40; // control byte: data
        memcpy(buf + 1, data + i, n);
        esp_err_t err = i2c_master_transmit(oledI2cDev, buf, n + 1, OLED_I2C_TIMEOUT_MS);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static bool oledDisplay();

void initOled() {
    i2c_master_bus_config_t busCfg = {};
    busCfg.i2c_port = I2C_NUM_0;
    busCfg.sda_io_num = (gpio_num_t)OLED_SDA_PIN;
    busCfg.scl_io_num = (gpio_num_t)OLED_SCL_PIN;
    busCfg.clk_source = I2C_CLK_SRC_DEFAULT;
    busCfg.glitch_ignore_cnt = 7;
    busCfg.flags.enable_internal_pullup = true;
    if (i2c_new_master_bus(&busCfg, &oledI2cBus) != ESP_OK) {
        Serial.println("[OLED] I2C bus init failed");
        return;
    }

    i2c_device_config_t devCfg = {};
    devCfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    devCfg.device_address = OLED_I2C_ADDR;
    devCfg.scl_speed_hz = 400000;
    if (i2c_master_bus_add_device(oledI2cBus, &devCfg, &oledI2cDev) != ESP_OK) {
        Serial.println("[OLED] I2C device add failed");
        return;
    }

    delay(50); // let the I2C bus/pull-ups settle before the first transactions

    static const uint8_t initCmds[] = {
        0xAE,             // display off
        0xD5, 0x80,       // clock divide ratio / osc freq
        0xA8, 0x3F,       // multiplex ratio = 64
        0xD3, 0x00,       // display offset = 0
        0x40,             // start line = 0
        0x8D, 0x14,       // charge pump enable
        0x20, 0x00,       // memory addressing mode = horizontal
        0xA0,             // segment remap (reversed -> 180 deg rotation)
        0xC0,             // COM output scan direction (normal -> 180 deg rotation)
        0xDA, 0x12,       // COM pins hardware config
        0x81, 0x7F,       // contrast
        0xD9, 0xF1,       // pre-charge period
        0xDB, 0x40,       // VCOMH deselect level
        0xA4,             // resume RAM content display
        0xA6,             // normal (non-inverted) display
    };
    for (size_t i = 0; i < sizeof(initCmds); i++) oledCmd(initCmds[i]);

    // Clear GDDRAM (its power-up contents are undefined) before turning the
    // panel on, so the very first frame isn't a stale/garbled one — since
    // updateOledStatus() only redraws on change, a corrupted first frame
    // would otherwise never get overwritten.
    memset(oledBuffer, 0, sizeof(oledBuffer));
    oledDisplay();
    delay(100); // let the charge pump settle before enabling the panel

    oledCmd(0xAF); // display on
    oledReady = true;
}

static void oledDrawChar(int x, int page, char ch) {
    if (ch < 0x20 || ch > 0x7E) ch = ' ';
    const uint8_t* glyph = oledFont5x7[ch - 0x20];
    for (int col = 0; col < 6; col++) {
        int bx = x + col;
        if (bx < 0 || bx >= OLED_WIDTH || page < 0 || page >= OLED_PAGES) continue;
        oledBuffer[page * OLED_WIDTH + bx] = (col < 5) ? glyph[col] : 0x00; // 1px gap
    }
}

static void oledDrawString(int x, int page, const char* s) {
    int cx = x;
    while (*s && cx < OLED_WIDTH) { oledDrawChar(cx, page, *s++); cx += 6; }
}

static void oledSetPixel(int x, int y, bool on) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    int page = y / 8;
    uint8_t mask = (uint8_t)(1 << (y % 8));
    int idx = page * OLED_WIDTH + x;
    if (on) oledBuffer[idx] |= mask;
    else    oledBuffer[idx] &= (uint8_t)~mask;
}

// Draws a char scaled by scaleNum/scaleDen (e.g. 3/2 = 150%) via nearest-neighbor
// pixel mapping back onto the 5x7 font, since the font has no larger bitmaps.
static void oledDrawCharScaled(int x, int y, char ch, int scaleNum, int scaleDen) {
    if (ch < 0x20 || ch > 0x7E) ch = ' ';
    const uint8_t* glyph = oledFont5x7[ch - 0x20];
    int w = (5 * scaleNum) / scaleDen;
    int h = (7 * scaleNum) / scaleDen;
    for (int dy = 0; dy < h; dy++) {
        int srcRow = (dy * scaleDen) / scaleNum;
        for (int dx = 0; dx < w; dx++) {
            int srcCol = (dx * scaleDen) / scaleNum;
            bool on = glyph[srcCol] & (1 << srcRow);
            oledSetPixel(x + dx, y + dy, on);
        }
    }
}

// Returns the x position right after the drawn text, so callers can chain
// another string (e.g. a different scale) immediately after it.
static int oledDrawStringScaled(int x, int y, const char* s, int scaleNum, int scaleDen) {
    int charW = (5 * scaleNum) / scaleDen;
    int gap   = (1 * scaleNum) / scaleDen; if (gap < 1) gap = 1;
    int cx = x;
    while (*s && cx < OLED_WIDTH) { oledDrawCharScaled(cx, y, *s++, scaleNum, scaleDen); cx += charW + gap; }
    return cx;
}

// Many SSD1306 128x64 panels have 132 columns of driver RAM but only 128 are
// wired to the glass. Flipping the segment remap for the 180-degree rotation
// moves those extra columns to the opposite side, shifting the visible image
// — compensate by offsetting the column start address. 2px is the commonly
// reported correction for this panel family.
#define OLED_COL_OFFSET 2

static bool oledDisplay() {
    for (int page = 0; page < OLED_PAGES; page++) {
        if (oledCmd(0xB0 + page) != ESP_OK) return false;                          // page address
        if (oledCmd(0x00 | (OLED_COL_OFFSET & 0x0F)) != ESP_OK) return false;       // lower column start address
        if (oledCmd(0x10 | ((OLED_COL_OFFSET >> 4) & 0x0F)) != ESP_OK) return false; // higher column start address
        if (oledData(&oledBuffer[page * OLED_WIDTH], OLED_WIDTH) != ESP_OK) return false;
    }
    return true;
}

// Counts dong/ho keys that have actually received a live Wi-SUN packet
// within the last few minutes (see upsertKeyIp's markLive parameter) —
// distinct from g_keyIpCount, which also includes entries bulk-loaded from
// the persisted/default IP-mapping table at boot and were never really seen.
static int countLiveWisunNodes() {
    const unsigned long LIVE_WINDOW_MS = 5UL * 60UL * 1000UL; // 5 minutes
    unsigned long now = millis();
    int count = 0;
    for (int i = 0; i < g_keyIpCount; i++) {
        if (g_keyIps[i].lastSeenMs != 0 && (now - g_keyIps[i].lastSeenMs) <= LIVE_WINDOW_MS) {
            count++;
        }
    }
    return count;
}

// Redraws the OLED only when the rendered status text actually changes,
// so the panel is not rewritten over I2C on every loop() tick.
void updateOledStatus() {
    if (!oledReady) return;

    char line2[32], line3[32], line4[32];

    bool wifiStaConnected = hasValidStaIp();
    wifi_mode_t wifiRadioMode = WiFi.getMode();
    bool wifiApActive = (wifiRadioMode == WIFI_AP || wifiRadioMode == WIFI_AP_STA);

    if (wifiStaConnected) {
        snprintf(line2, sizeof(line2), "WiFi: %s", WiFi.localIP().toString().c_str());
    } else if (wifiApActive) {
        snprintf(line2, sizeof(line2), "WiFi: AP %s", WiFi.softAPIP().toString().c_str());
    } else {
        snprintf(line2, sizeof(line2), "WiFi: Not connected");
    }

    if (wiSunInitialized && isWiSUNConnected) {
        snprintf(line3, sizeof(line3), "WiSUN: %d connected", countLiveWisunNodes());
    } else if (wiSunInitialized) {
        snprintf(line3, sizeof(line3), "WiSUN: Initialized");
    } else {
        snprintf(line3, sizeof(line3), "WiSUN: Not connected");
    }

    bool working = (wifiStaConnected || wifiApActive) && wiSunInitialized && isWiSUNConnected;
    snprintf(line4, sizeof(line4), "Status: %s", working ? "Working" : "Not working");

    static char prevLine2[32] = "";
    static char prevLine3[32] = "";
    static char prevLine4[32] = "";
    static bool everDrawn = false;
    static bool oledLastDrawFailed = false;

    if (everDrawn && strcmp(line2, prevLine2) == 0 && strcmp(line3, prevLine3) == 0 &&
        strcmp(line4, prevLine4) == 0) {
        return; // nothing changed — skip the I2C rewrite
    }

    memset(oledBuffer, 0, sizeof(oledBuffer));
    // "LMP Server" at 150%, "v1.0" smaller (100%) and baseline-aligned with it
    // (150%-scale glyphs are 10px tall vs 7px at 100%, hence the +3 y offset).
    int titleEndX = oledDrawStringScaled(0, 0, "LMP Server ", 3, 2);
    oledDrawStringScaled(titleEndX, 3, "v1.0", 1, 1);
    oledDrawString(0, 2, line2);
    oledDrawString(0, 4, line3);
    oledDrawString(0, 6, line4);

    if (!oledDisplay()) {
        if (!oledLastDrawFailed) {
            Serial.println("[OLED] redraw failed (I2C error) - will retry on next change");
            oledLastDrawFailed = true;
        }
        return; // leave prevLine*/everDrawn untouched so a retry is forced
    }
    if (oledLastDrawFailed) {
        Serial.println("[OLED] redraw recovered");
        oledLastDrawFailed = false;
    }

    strncpy(prevLine2, line2, sizeof(prevLine2)); prevLine2[sizeof(prevLine2)-1] = '\0';
    strncpy(prevLine3, line3, sizeof(prevLine3)); prevLine3[sizeof(prevLine3)-1] = '\0';
    strncpy(prevLine4, line4, sizeof(prevLine4)); prevLine4[sizeof(prevLine4)-1] = '\0';
    everDrawn = true;
}

// ============================================================================
//  BLE Serial Bridge  (Nordic UART Service)
// ============================================================================
class BLESerialBridge : public Stream {
public:
    BLESerialBridge()
        : started(false), clientConnected(false), needReAdvertise(false),
          server(nullptr), txCharacteristic(nullptr), rxCharacteristic(nullptr),
          rxHead(0), rxTail(0), rxCount(0),
          serverCallbacks(this), rxCallbacks(this) {}

    bool begin(const char* deviceName) {
        end();
        BLEDevice::init(deviceName);
        server = BLEDevice::createServer();
        if (!server) return false;
        server->setCallbacks(&serverCallbacks);
        BLEService* svc = server->createService(kUartServiceUuid);
        if (!svc) { end(); return false; }
        txCharacteristic = svc->createCharacteristic(kTxCharUuid, BLECharacteristic::PROPERTY_NOTIFY);
        rxCharacteristic = svc->createCharacteristic(kRxCharUuid,
            BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
        if (!txCharacteristic || !rxCharacteristic) { end(); return false; }
        txCharacteristic->addDescriptor(new BLE2902());
        rxCharacteristic->setCallbacks(&rxCallbacks);
        svc->start();
        BLEAdvertising* adv = BLEDevice::getAdvertising();
        if (!adv) { end(); return false; }
        adv->addServiceUUID(kUartServiceUuid);
        adv->setScanResponse(true);
        adv->setMinPreferred(0x06);
        adv->setMinPreferred(0x12);
        BLEDevice::startAdvertising();
        clearRxBuffer();
        started = true; clientConnected = false; needReAdvertise = false;
        return true;
    }
    void check() {
        if (!started || !needReAdvertise) return;
        needReAdvertise = false;
        BLEAdvertising* adv = BLEDevice::getAdvertising();
        if (!adv) return;
        adv->setScanResponse(true);
        BLEDevice::startAdvertising();
        Serial.println("[BLE] Re-advertising started");
    }
    void end() {
        if (started) BLEDevice::deinit(false);
        started = false; clientConnected = false; needReAdvertise = false;
        server = nullptr; txCharacteristic = nullptr; rxCharacteristic = nullptr;
        clearRxBuffer();
    }
    int    available() override { return (int)rxCount; }
    int    peek()      override { return rxCount ? rxBuffer[rxTail] : -1; }
    int    read()      override {
        if (!rxCount) return -1;
        uint8_t v = rxBuffer[rxTail]; rxTail = (rxTail + 1) % kBufSz; rxCount--; return v;
    }
    void   flush()  override {}
    size_t write(uint8_t v) override { return write(&v, 1); }
    size_t write(const uint8_t* buf, size_t sz) override {
        if (!started || !clientConnected || !txCharacteristic || !buf) return sz;
        for (size_t off = 0; off < sz; ) {
            size_t chunk = sz - off; if (chunk > kChunk) chunk = kChunk;
            txCharacteristic->setValue(buf + off, chunk);
            txCharacteristic->notify();
            off += chunk; delay(2);
        }
        return sz;
    }
    using Print::write;

private:
    static constexpr const char* kUartServiceUuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
    static constexpr const char* kTxCharUuid      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
    static constexpr const char* kRxCharUuid      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
    static constexpr size_t kBufSz = 1024;
    static constexpr size_t kChunk = 20;

    class SC : public BLEServerCallbacks {
    public: explicit SC(BLESerialBridge* p):parent(p){}
        void onConnect(BLEServer*) override {
            parent->clientConnected = true;
            parent->needReAdvertise = false;
            BLEDevice::stopAdvertising();
            Serial.println("[BLE] Client connected");
        }
        void onDisconnect(BLEServer*) override {
            parent->clientConnected = false;
            parent->needReAdvertise = true;
            Serial.println("[BLE] Client disconnected");
        }
    private: BLESerialBridge* parent;
    };
    class RC : public BLECharacteristicCallbacks {
    public: explicit RC(BLESerialBridge* p):parent(p){}
        void onWrite(BLECharacteristic* c) override {
            String v = c->getValue(); for (size_t i=0;i<v.length();i++) parent->pushByte((uint8_t)v.charAt(i));
        }
    private: BLESerialBridge* parent;
    };

    void clearRxBuffer() { rxHead = rxTail = rxCount = 0; }
    void pushByte(uint8_t v) {
        if (rxCount >= kBufSz) { rxTail = (rxTail+1)%kBufSz; rxCount--; }
        rxBuffer[rxHead] = v; rxHead = (rxHead+1)%kBufSz; rxCount++;
    }

    bool started, clientConnected;
    volatile bool needReAdvertise;
    BLEServer* server;
    BLECharacteristic *txCharacteristic, *rxCharacteristic;
    uint8_t rxBuffer[kBufSz];
    size_t rxHead, rxTail, rxCount;
    SC serverCallbacks; RC rxCallbacks;
};

BLESerialBridge SerialBT;
bool bluetoothStarted = false;

// ============================================================================
//  UART0 direct-read  (no event-queue driver)
// ============================================================================
static int s_uart0_peeked = -1;
static bool uart0RxAvailable() {
    if (s_uart0_peeked >= 0) return true;
    size_t len = 0;
    return (uart_get_buffered_data_len(UART_NUM_0, &len) == ESP_OK && len > 0);
}
static int uart0RxRead() {
    if (s_uart0_peeked >= 0) { int v = s_uart0_peeked; s_uart0_peeked = -1; return v; }
    uint8_t b; return (uart_read_bytes(UART_NUM_0, &b, 1, 0) == 1) ? b : -1;
}
static int uart0RxPeek() {
    if (s_uart0_peeked >= 0) return s_uart0_peeked;
    uint8_t b;
    if (uart_read_bytes(UART_NUM_0, &b, 1, 0) == 1) { s_uart0_peeked = b; return b; }
    return -1;
}

// ============================================================================
//  Unified input helpers  (UART0 + BLE)
// ============================================================================
bool inputAvailable() { return uart0RxAvailable() || (bluetoothStarted && SerialBT.available()); }
char readInput() {
    if (uart0RxAvailable())                       { int b = uart0RxRead(); return b >= 0 ? (char)b : 0; }
    if (bluetoothStarted && SerialBT.available())   return SerialBT.read();
    return 0;
}
char peekInput() {
    if (uart0RxAvailable())                       { int b = uart0RxPeek(); return b >= 0 ? (char)b : 0; }
    if (bluetoothStarted && SerialBT.available())   return SerialBT.peek();
    return 0;
}
void flushInput() {
    while (uart0RxAvailable()) uart0RxRead();
    if (bluetoothStarted) while (SerialBT.available()) SerialBT.read();
}

// ============================================================================
//  Text prompt input helper
// ============================================================================
void readSerialInput(const char* prompt, const char* def, char* out, size_t maxLen) {
    Serial.printf("%s [%s]: ", prompt, def);
    SerialBT.printf("%s [%s]: ", prompt, def);
    Serial.flush();
    char buf[65] = ""; int idx = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 30000) {
        serviceWatchdog();
        if (!inputAvailable()) { delay(10); continue; }
        char c = readInput();
        if (c == '\n' || c == '\r') {
            Serial.println();
            if (idx == 0 && strlen(def) > 0) { strncpy(out, def,  maxLen-1); }
            else                              { buf[idx]='\0'; strncpy(out, buf, maxLen-1); }
            out[maxLen-1] = '\0'; return;
        }
        if (idx < 64) {
            buf[idx++] = c;
            Serial.print(c);
            SerialBT.print(c);
        }
    }
    Serial.println("\nTimeout.");
    strncpy(out, def, maxLen-1); out[maxLen-1] = '\0';
}

// ============================================================================
//  Wi-SUN helpers
// ============================================================================
static bool abortableDelay(unsigned long ms) {
    unsigned long t0 = millis();
    while (millis() - t0 < ms) {
        serviceWatchdog();
        if (inputAvailable() && peekInput() == '/') {
            readInput();
            unsigned long t1 = millis();
            while (!inputAvailable() && millis() - t1 < 80) delay(1);
            if (!inputAvailable() || peekInput() == '\r' || peekInput() == '\n') {
                flushInput();
                g_wisun_abort = true;
                Serial.println("[WiSUN] Abort requested by user input '/'.");
                return true;
            }
        }
        delay(10);
    }
    return g_wisun_abort;
}

static bool wisunParamHasRate(const String& paramResp, const char* rate) {
    String p = paramResp;
    p.toLowerCase();
    String r = String(rate ? rate : "");
    r.toLowerCase();
    if (r == "50" || r == "50k" || r == "50kb") r = "50kbps";
    if (r == "150" || r == "150k" || r == "150kb") r = "150kbps";
    if (r == "50kbps") {
        return p.indexOf("rate=50kbps") != -1 || p.indexOf("rate=50k") != -1;
    }
    if (r == "150kbps") {
        return p.indexOf("rate=150kbps") != -1 || p.indexOf("rate=150k") != -1;
    }
    String needle = String("rate=") + r;
    return p.indexOf(needle) != -1;
}

static bool applyWiSUNPhyProfileForRate(const char* rate) {
    String r = String(rate ? rate : "");
    r.toLowerCase();
    if (r == "50" || r == "50k" || r == "50kb") r = "50kbps";
    if (r == "150" || r == "150k" || r == "150kb") r = "150kbps";

    if (r == "50kbps") {
        sendCommand("param set class 1\r\n");
        if (abortableDelay(250)) return false;
        sendCommand("param set base_channel 902200\r\n");
        if (abortableDelay(250)) return false;
        sendCommand("param set channel_space 200\r\n");
        if (abortableDelay(250)) return false;
        sendCommand("param set channel_number 129\r\n");
        if (abortableDelay(250)) return false;
        return true;
    }

    if (r == "150kbps") {
        sendCommand("param set class 2\r\n");
        if (abortableDelay(250)) return false;
        sendCommand("param set base_channel 902400\r\n");
        if (abortableDelay(250)) return false;
        sendCommand("param set channel_space 400\r\n");
        if (abortableDelay(250)) return false;
        sendCommand("param set channel_number 64\r\n");
        if (abortableDelay(250)) return false;
        return true;
    }

    return true;
}

static bool setWiSUNChrateVerified(const char* rate) {
    auto verifyNow = [&](const char* stage) {
        String p = sendCommand("param\r\n");
        bool ok = wisunParamHasRate(p, rate);
        Serial.printf("[WiSUN] %s rate verify: %s (target=%s)\n", stage, ok ? "OK" : "FAIL", rate);
        return ok;
    };

    if (!applyWiSUNPhyProfileForRate(rate)) {
        Serial.printf("[WiSUN] WARNING: PHY profile apply interrupted (target=%s)\n", rate);
        return false;
    }

    String rateToken = String(rate ? rate : "");
    rateToken.toLowerCase();
    if (rateToken == "50kbps") rateToken = "50k";
    else if (rateToken == "150kbps") rateToken = "150k";

    {
        char cmd[48];
        snprintf(cmd, sizeof(cmd), "chrate %s\r\n", rateToken.c_str());
        sendCommand(cmd);
        if (abortableDelay(300)) return false;
        if (verifyNow("chrate")) return true;
    }

    {
        char cmd[48];
        snprintf(cmd, sizeof(cmd), "param set rate %s\r\n", rateToken.c_str());
        sendCommand(cmd);
        if (abortableDelay(300)) return false;
        if (verifyNow("param set rate")) return true;
    }

    {
        char cmd[48];
        snprintf(cmd, sizeof(cmd), "param set chrate %s\r\n", rate);
        sendCommand(cmd);
        if (abortableDelay(300)) return false;
        if (verifyNow("param set chrate")) return true;
    }

    Serial.printf("[WiSUN] WARNING: rate apply not verified (target=%s)\n", rate);
    return false;
}

String sendCommand(const char* cmd) {
    Serial.printf("[WiSUN CMD] %s", cmd);
    WiSUNSerial.print(cmd);
    char resp[512] = ""; int ri = 0;
    unsigned long t0 = millis(), lastRx = t0;
    while (millis() - t0 < 3000) {
        serviceWatchdog();
        while (WiSUNSerial.available() && ri < (int)sizeof(resp)-1) {
            resp[ri++] = WiSUNSerial.read(); Serial.print(resp[ri-1]); lastRx = millis();
        }
        if (ri > 0 && millis() - lastRx > 300) break;
        delay(10);
    }
    resp[ri] = '\0';
    return String(resp);
}

bool isFstatJoined(const String& r) {
    return r.indexOf("net state:5") != -1 || r.indexOf("net state: 5") != -1 ||
           r.indexOf("state:5")     != -1 || r.indexOf("Operational") != -1;
}

static bool isATModeResponse(const String& r) {
    return r.indexOf("net state:") != -1 ||
           r.indexOf("[Network state]") != -1 ||
           r.indexOf("Border router") != -1;
}

static bool isLikelyTransparentTraffic(const String& r) {
    return r.indexOf("udpr<") != -1 ||
           r.indexOf(":501_") != -1 ||
           r.indexOf("Detected BPM") != -1 ||
           r.indexOf("Movement Detected") != -1 ||
           r.indexOf("Signal stabilized") != -1;
}

static bool sendPlusPlusEscape() {
    // Many Wi-SUN firmwares require guard time and no CR/LF for "+++" escape.
    while (WiSUNSerial.available()) (void)WiSUNSerial.read();
    delay(1100);

    Serial.println("[WiSUN CMD] +++ (guarded)");
    WiSUNSerial.print("+++");
    WiSUNSerial.flush();

    delay(1100);
    String probe = sendCommand("fstat\r\n");
    return isATModeResponse(probe);
}

static bool ensureATCommandMode() {
    // Probe first: if command mode is active, avoid toggling with '+++'.
    String probe = sendCommand("fstat\r\n");
    if (isATModeResponse(probe)) return true;

    // Retry once before attempting any risky mode-toggle escape.
    delay(250);
    probe = sendCommand("fstat\r\n");
    if (isATModeResponse(probe)) return true;

    // Guard against accidental AT->transparent toggle on noisy links.
    // Only attempt '+++' if we can see transparent payload-like traffic.
    if (!isLikelyTransparentTraffic(probe)) return false;

    // Not responding as AT mode: try to switch from transparent mode.
    if (sendPlusPlusEscape()) return true;

    // Some firmware echoes minimally; probe again after guard delay.
    delay(300);
    probe = sendCommand("fstat\r\n");
    if (isATModeResponse(probe)) return true;

    // Last retry for flaky transitions.
    if (sendPlusPlusEscape()) return true;
    delay(300);
    probe = sendCommand("fstat\r\n");
    return isATModeResponse(probe);
}

static bool forceATModeByHardwareReset() {
    Serial.println("[AT] Forcing command mode via hardware reset...");

    while (WiSUNSerial.available()) (void)WiSUNSerial.read();
    digitalWrite(TXON_PIN, LOW);
    digitalWrite(WAKEUP_PIN, HIGH);
    digitalWrite(RESETN_PIN, HIGH);
    delay(50);

    digitalWrite(RESETN_PIN, LOW);
    delay(120);
    digitalWrite(RESETN_PIN, HIGH);
    delay(600);
    digitalWrite(TXON_PIN, HIGH);
    delay(250);

    while (WiSUNSerial.available()) (void)WiSUNSerial.read();
    String probe = sendCommand("fstat\r\n");
    return isATModeResponse(probe);
}

// ============================================================================
//  Received Wi-SUN data dispatcher
// ============================================================================
void processReceivedWiSUNData(const char* srcAddr, const char* payload) {
    serviceWatchdog();
    ingestHelmetText(srcAddr, payload);
    if (g_dataLoggingEnabled && !g_menuActive) {
        SerialBT.printf("[RX] <%s> %s\n", srcAddr, payload);
    }

    char srcResolved[40] = "";
    if (srcAddr && strlen(srcAddr) > 0) {
        strncpy(srcResolved, srcAddr, sizeof(srcResolved) - 1);
        srcResolved[sizeof(srcResolved) - 1] = '\0';
    } else {
        const char* p = strstr(payload, "udpr<");
        if (p) {
            p += 5;
            const char* e = strchr(p, '>');
            if (e) {
                int n = (int)(e - p);
                if (n > 0 && n < (int)sizeof(srcResolved)) {
                    strncpy(srcResolved, p, n);
                    srcResolved[n] = '\0';
                }
            }
        }
    }

    char pktSrc[40] = "";
    char pktKey[24] = "";
    char pktStatus[160] = "";
    if (parseKeyStatusPacket(payload, pktSrc, sizeof(pktSrc), pktKey, sizeof(pktKey), pktStatus, sizeof(pktStatus))) {
        serviceWatchdog();
        bool stateChanged = upsertCellState(pktKey, pktStatus);
        const char* latchedCode = getCellCodeByKey(pktKey);
        const char* effectiveSrc = nullptr;
        if (strlen(pktSrc) > 0) {
            effectiveSrc = pktSrc;
        } else if (strlen(srcResolved) > 0) {
            // Some packets arrive as "<src> key,status" without udpr<...> in payload.
            // Use the parsed line prefix source so this key is still treated as live.
            effectiveSrc = srcResolved;
        }

        if (effectiveSrc && *effectiveSrc) {
            upsertKeyIp(pktKey, effectiveSrc);
            trackNode(effectiveSrc);
        }
        String compact = String(pktKey) + "," + String(pktStatus);
        bool allowStreamPush = false;
        if (g_packetStreamEnabled) {
            unsigned long now = millis();
            if (now - g_lastPacketStreamPushMs >= 800) {
                g_lastPacketStreamPushMs = now;
                allowStreamPush = true;
            }
        }
        if (stateChanged || allowStreamPush) {
            const char* rawForWeb = g_packetStreamEnabled ? compact.c_str() : "";
            notifyWebClients(effectiveSrc ? effectiveSrc : "", compact.c_str(), rawForWeb, pktKey, pktStatus, latchedCode);
        }
        return;
    }

    if (strlen(srcResolved) > 0) trackNode(srcResolved);
    String compact = trimPacketForWeb(payload);
    if (g_packetStreamEnabled) {
        unsigned long now = millis();
        if (now - g_lastPacketStreamPushMs < 1200) return;
        g_lastPacketStreamPushMs = now;
        notifyWebClients(srcResolved, compact.c_str(), compact.c_str(), "", "", "");
    }
}

void processLine(const char* line) {
    serviceWatchdog();

    if (g_dataLoggingEnabled && !g_menuActive) {
        Serial.printf("[WiSUN LINE] '%s'\n", line);
    }

    char src[40] = "";
    const char* payload = line;

    if (line[0] == '<') {
        const char* ep = strchr(line + 1, '>');
        if (ep) {
            int len = (int)(ep - (line + 1));
            if (len < (int)sizeof(src)-1) { strncpy(src, line+1, len); src[len]='\0'; }
            payload = ep + 1;
            while (*payload == ' ') payload++;
        }
    }

    if (strcasecmp(payload, "send OK") == 0) {
        g_lastCheckCommandCellIndex = -1;
        return;
    }
    if (strcasecmp(payload, "send fail") == 0) {
        int failedCellIndex = g_lastCheckCommandCellIndex;
        g_lastCheckCommandCellIndex = -1;
        if (failedCellIndex >= 0 && failedCellIndex < g_cellStateCount) {
            g_cellStates[failedCellIndex].awaitingStatusCheck = false;
            g_cellStates[failedCellIndex].statusCheckDeadlineMs = 0;
            forceCellComAndNotify(failedCellIndex, "send_fail");
            Serial.printf("[CHECK] send fail -> key=%s\n", g_cellStates[failedCellIndex].key);
        }
        return;
    }

    if (payload[0] == '_') {
        const char* cmd = payload + 1;
        if (strcasecmp(cmd, "chknet") == 0) {
            char reply[160];
            const char* rn = (strlen(routerNumber) > 0) ? routerNumber : "1";
            if (strlen(src) > 0)
                snprintf(reply, sizeof(reply), "udpr<%s>:BRD_%s,Network OK\r\n", src, rn);
            else
                snprintf(reply, sizeof(reply), "udpr%s:BRD_%s,Network OK\r\n", MULTICAST_ADDRESS, rn);
            WiSUNSerial.print(reply);
            WiSUNSerial.flush();
            Serial.println("[WiSUN TX] Network OK reply sent.");
        }
    } else if (strlen(payload) > 0) {
        processReceivedWiSUNData(src, payload);
    }
}

void checkWiSUNUnsolicited() {
    static unsigned long lastCharTime = 0;
    while (WiSUNSerial.available()) {
        serviceWatchdog();
        char c = WiSUNSerial.read(); lastCharTime = millis();
        if (c == '\n') {
            if (wiSunLineIndex > 0) {
                wiSunLineBuffer[wiSunLineIndex] = '\0';
                processLine(wiSunLineBuffer);
                wiSunLineIndex = 0;
            }
        } else if (c == '\b' || c == 0x7F) {
            if (wiSunLineIndex > 0) wiSunLineIndex--;
        } else if (c != '\r') {
            if (wiSunLineIndex < (int)sizeof(wiSunLineBuffer)-1)
                wiSunLineBuffer[wiSunLineIndex++] = c;
        }
    }
    if (wiSunLineIndex > 0 && millis() - lastCharTime >= 2000) {
        wiSunLineBuffer[wiSunLineIndex] = '\0';
        processLine(wiSunLineBuffer);
        wiSunLineIndex = 0;
    }
}

// ============================================================================
//  Wi-SUN initialization
// ============================================================================
void initializeWiSUN() {
    g_wisun_abort = false;
    g_wisunRuntimeReady = false;
    isWiSUNConnected = false;
    flushInput();
    delay(30);
    flushInput();
    Serial.println("[WiSUN] Press '/' to abort.");

    auto reinstallUart2 = []() {
        uart_driver_delete(UART_NUM_2);
        uart_driver_install(UART_NUM_2, 512, 0, 0, NULL, 0);
        uart_config_t cfg = { .baud_rate=115200, .data_bits=UART_DATA_8_BITS,
            .parity=UART_PARITY_DISABLE, .stop_bits=UART_STOP_BITS_1,
            .flow_ctrl=UART_HW_FLOWCTRL_DISABLE, .source_clk=UART_SCLK_DEFAULT };
        uart_param_config(UART_NUM_2, &cfg);
        uart_set_pin(UART_NUM_2, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        Serial.println("[UART2] Reinstalled (no event queue).");
    };

    auto hwReset = [&]() -> bool {
        digitalWrite(TXON_PIN, LOW); digitalWrite(RESETN_PIN, HIGH); digitalWrite(WAKEUP_PIN, HIGH);
        if (abortableDelay(50))  return false;
        digitalWrite(RESETN_PIN, LOW);
        if (abortableDelay(100)) return false;
        digitalWrite(RESETN_PIN, HIGH);
        if (abortableDelay(500)) return false;
        digitalWrite(TXON_PIN, HIGH);
        return true;
    };

    // Preserve a live module session when only the ESP32 restarted. After a full
    // power cycle, also give the module time to auto-join with its saved params
    // before interrupting it with resets or configuration writes.
    if (wiSunInitialized && strlen(globalIPv6Address) > 0) {
        Serial.println("[WiSUN] Trying non-reset session resume...");
        isWiSUNSettingUp = true;
        WiSUNSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
        reinstallUart2();

        if (!ensureATCommandMode()) {
            Serial.println("[WiSUN] AT mode not confirmed yet; waiting for module auto-recovery.");
        }

        unsigned long resumeStartedMs = millis();
        while (millis() - resumeStartedMs < 20000) {
            if (g_wisun_abort) goto wisun_aborted;
            String state = sendCommand("fstat\r\n");
            if (isFstatJoined(state)) {
                isWiSUNConnected = true;
                g_wisunRuntimeReady = true;
                isWiSUNSettingUp = false;
                g_nextCheckCycleMs = millis() + (g_checkIntervalSec * 1000UL);
                setLEDColor("green");
                Serial.println("[WiSUN] Existing/saved module session resumed without reset.");
                notifyWebClients("WISUN", "Connected (session resumed)", "", "", "Wi-SUN Connected", "5");
                saveWiSUNConnectionState();
                blinkGreenTwice();
                return;
            }
            checkWiSUNUnsolicited();
            if (abortableDelay(750)) goto wisun_aborted;
        }
        Serial.println("[WiSUN] Non-reset resume not ready; trying controlled recovery.");
    }

    // ---- Quick reconnect ----
    if (wiSunInitialized && strlen(globalIPv6Address) > 0) {
        Serial.println("[WiSUN] Quick reconnect attempt...");
        isWiSUNSettingUp = true;
        WiSUNSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
        reinstallUart2();
        if (!hwReset()) goto wisun_aborted;
        setWiSUNChrateVerified(wiSunChrate);       if (abortableDelay(200)) goto wisun_aborted;
        sendCommand("param save\r\n");           if (abortableDelay(500)) goto wisun_aborted;
        sendCommand("svrst\r\n");
        if (abortableDelay(3000)) goto wisun_aborted;
        {
            unsigned long t0 = millis();
            while (millis() - t0 < 60000) {
                if (g_wisun_abort) goto wisun_aborted;
                if (millis() - lastBlinkTime > blinkMillisInterval) {
                    lastBlinkTime = millis(); blinkState = !blinkState;
                    setLEDColor(blinkState ? "red" : "yellow");
                }
                checkWiSUNUnsolicited();
                if (isFstatJoined(sendCommand("fstat\r\n"))) {
                    if (!isWiSUNConnected) {
                        Serial.println("[WISUN] Connected (net state 5)!");
                        notifyWebClients("WISUN", "Connected (net state 5)", "", "", "Wi-SUN Connected", "5");
                    }
                    isWiSUNConnected = true;
                    break;
                }
                if (abortableDelay(500)) goto wisun_aborted;
            }
        }
        if (isWiSUNConnected) {
            setLEDColor("green"); isWiSUNSettingUp = false;
            g_wisunRuntimeReady = true;
            g_nextCheckCycleMs = millis() + (g_checkIntervalSec * 1000UL);
            Serial.println("[WiSUN] Quick reconnect OK — AT Command Mode.");
            saveWiSUNConnectionState();
            blinkGreenTwice(); return;
        }
        Serial.println("[WiSUN] Quick reconnect failed — full init.");
        wiSunInitialized = false; isWiSUNConnected = false;
    }

    // ---- Full initialization ----
    isWiSUNSettingUp = true;
    WiSUNSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    reinstallUart2();
    if (!hwReset()) goto wisun_aborted;

    sendCommand("version\r\n");                     if (abortableDelay(500)) goto wisun_aborted;
    sendCommand("role\r\n");                        if (abortableDelay(500)) goto wisun_aborted;
    sendCommand("param set domain 1\r\n");          if (abortableDelay(500)) goto wisun_aborted;
    sendCommand("param set class 1\r\n");           if (abortableDelay(500)) goto wisun_aborted;
    sendCommand("param set base_channel 902200\r\n"); if (abortableDelay(500)) goto wisun_aborted;
    sendCommand("param set channel_space 200\r\n"); if (abortableDelay(500)) goto wisun_aborted;
    sendCommand("param set channel_number 129\r\n"); if (abortableDelay(500)) goto wisun_aborted;
    sendCommand("param set tx_power 20\r\n");       if (abortableDelay(500)) goto wisun_aborted;
    sendCommand("param set cca_threshold -83\r\n"); if (abortableDelay(500)) goto wisun_aborted;
    setWiSUNChrateVerified(wiSunChrate);           if (abortableDelay(200)) goto wisun_aborted;
    {
        String r = sendCommand("udpopts\r\n"); if (abortableDelay(500)) goto wisun_aborted;
        if (r.indexOf(MULTICAST_ADDRESS) == -1) {
            char c[64]; snprintf(c, sizeof(c), "udpopts set multicast %s\r\n", MULTICAST_ADDRESS);
            sendCommand(c); if (abortableDelay(500)) goto wisun_aborted;
        }
    }
    sendCommand("udpopts set disp_source on\r\n");  if (abortableDelay(500)) goto wisun_aborted;
    sendCommand("udpopts set udp_port 1234\r\n");   if (abortableDelay(500)) goto wisun_aborted;
    sendCommand("param save\r\n");                  if (abortableDelay(500)) goto wisun_aborted;
    sendCommand("svrst\r\n");                       if (abortableDelay(5000)) goto wisun_aborted;

    Serial.println("[WiSUN] Waiting for network join (max 180 s)...");
    {
        unsigned long t0 = millis();
        while (millis() - t0 < 180000) {
            if (g_wisun_abort) goto wisun_aborted;
            if (millis() - lastBlinkTime > blinkMillisInterval) {
                lastBlinkTime = millis(); blinkState = !blinkState;
                setLEDColor(blinkState ? "red" : "yellow");
            }
            checkWiSUNUnsolicited(); serviceWatchdog();
            if (isFstatJoined(sendCommand("fstat\r\n"))) { isWiSUNConnected = true; break; }
            if (abortableDelay(500)) goto wisun_aborted;
        }
    }

    setLEDColor(isWiSUNConnected ? "green" : "red"); delay(1000);
    if (!isWiSUNConnected) {
        Serial.println("[WISUN] WARNING: Join failed — continuing.");
        notifyWebClients("WISUN", "Disconnected", "", "", "Wi-SUN Disconnected", "0");
    }

    {
        String r = sendCommand("param\r\n"); delay(500);
        auto parseAddr = [&](const char* tag, char* out, size_t outLen, const char* fb) {
            int ti = r.indexOf(tag);
            if (ti != -1) { int s = r.lastIndexOf("<", ti); int e = r.indexOf(">", ti);
                if (s != -1 && e != -1) { r.substring(s+1, e).toCharArray(out, outLen); return; }
            }
            strncpy(out, fb, outLen-1); out[outLen-1]='\0';
        };
        parseAddr("(global)", globalIPv6Address, sizeof(globalIPv6Address), "2020:abcd::1");
        parseAddr("(local)",  localIPv6Address,  sizeof(localIPv6Address),  "fe80::1");
        Serial.printf("[WiSUN] Global: %s  Local: %s\n", globalIPv6Address, localIPv6Address);
    }

    wiSunInitialized = true; isWiSUNSettingUp = false;
    g_wisunRuntimeReady = true;
    g_nextCheckCycleMs = millis() + (g_checkIntervalSec * 1000UL);
    Serial.println("[WiSUN] AT Command Mode — ready.");
    saveWiSUNConnectionState();
    blinkGreenTwice(); return;

wisun_aborted:
    Serial.println("[WiSUN] Aborted."); isWiSUNSettingUp = false; isWiSUNConnected = false;
    g_wisunRuntimeReady = false;
    saveWiSUNConnectionState();
    setLEDColor("yellow");
}

// ============================================================================
//  NVS
// ============================================================================
void loadConfig() {
    preferences.begin("gw-config", true);
    preferences.getString("dong",       dong,              sizeof(dong));
    preferences.getString("ho",         ho,                sizeof(ho));
    sensorNumber      = preferences.getInt("sensorNum",  1);
    preferences.getString("routerNo",   routerNumber,      sizeof(routerNumber));
    wiSunInitialized  = preferences.getBool("wisunInit", false);
    (void)preferences.getBool("wisunConn", false);
    // Persisted connection is only a boot hint. Actual connectivity must be
    // confirmed from the module after every ESP32 or full power restart.
    isWiSUNConnected = false;
    preferences.getString("globalIPv6", globalIPv6Address, sizeof(globalIPv6Address));
    preferences.getString("localIPv6",  localIPv6Address,  sizeof(localIPv6Address));

    wifiConfigured = preferences.getBool("wifiCfg", false);
    preferences.getString("wifiMode", wifiMode, sizeof(wifiMode));
    preferences.getString("wifiSSID", wifiStaSSID, sizeof(wifiStaSSID));
    preferences.getString("wifiPASS", wifiStaPASS, sizeof(wifiStaPASS));
    preferences.getString("wifiIP", wifiStaticIP, sizeof(wifiStaticIP));
    preferences.getString("wifiNM", wifiNetmask, sizeof(wifiNetmask));
    preferences.getString("wifiGW", wifiGateway, sizeof(wifiGateway));
    preferences.getString("apSSID", wifiApSSID, sizeof(wifiApSSID));
    preferences.getString("apPASS", wifiApPASS, sizeof(wifiApPASS));

    preferences.getString("wisunChrate", wiSunChrate, sizeof(wiSunChrate));
    preferences.getString("multicast", MULTICAST_ADDRESS, sizeof(MULTICAST_ADDRESS));
    g_checkIntervalSec = preferences.getULong("checkIntSec", 1800);
    preferences.end();

    if (strlen(routerNumber) == 0) {
        strncpy(routerNumber, "1", sizeof(routerNumber) - 1);
        routerNumber[sizeof(routerNumber) - 1] = '\0';
    }

    if (strlen(wifiApSSID) == 0) {
        strncpy(wifiApSSID, "BRD-GW-AP", sizeof(wifiApSSID) - 1);
        wifiApSSID[sizeof(wifiApSSID) - 1] = '\0';
    }
    if (strlen(wifiApPASS) < 8) {
        strncpy(wifiApPASS, "12345678", sizeof(wifiApPASS) - 1);
        wifiApPASS[sizeof(wifiApPASS) - 1] = '\0';
    }

    if (strlen(MULTICAST_ADDRESS) == 0) {
        strncpy(MULTICAST_ADDRESS, MULTICAST_ADDRESS_DEFAULT, sizeof(MULTICAST_ADDRESS) - 1);
        MULTICAST_ADDRESS[sizeof(MULTICAST_ADDRESS) - 1] = '\0';
    }

    if (g_checkIntervalSec < 1800 || g_checkIntervalSec > 86400 || (g_checkIntervalSec % 1800) != 0) {
        g_checkIntervalSec = 1800;
    }

    // Normalize persisted Wi-SUN chrate value (fix typos like 50kps/50kpbs)
    {
        String s = String(wiSunChrate);
        s.trim();
        s.toLowerCase();
        s.replace(" ", "");
        if (s == "50" || s == "50k" || s == "50kb" || s == "50kps" || s == "50kpbs") {
            strncpy(wiSunChrate, "50kbps", sizeof(wiSunChrate) - 1);
            wiSunChrate[sizeof(wiSunChrate) - 1] = '\0';
        } else if (s == "150" || s == "150k" || s == "150kb" || s == "150kps" || s == "150kpbs") {
            strncpy(wiSunChrate, "150kbps", sizeof(wiSunChrate) - 1);
            wiSunChrate[sizeof(wiSunChrate) - 1] = '\0';
        } else if (s != "50kbps" && s != "150kbps") {
            strncpy(wiSunChrate, "50kbps", sizeof(wiSunChrate) - 1);
            wiSunChrate[sizeof(wiSunChrate) - 1] = '\0';
        }
    }

    Serial.printf("[NVS] router=%s dong=%s ho=%s sensor=%d wisun=%s wifiCfg=%s\n",
        routerNumber, dong, ho, sensorNumber, wiSunInitialized ? "Y" : "N", wifiConfigured ? "Y" : "N");
}
void saveDongHoConfig() {
    preferences.begin("gw-config", false);
    preferences.putString("dong",    dong);
    preferences.putString("ho",      ho);
    preferences.putInt   ("sensorNum", sensorNumber);
    preferences.putString("routerNo", routerNumber);
    preferences.end();
}

void saveWiFiConfig() {
    preferences.begin("gw-config", false);
    preferences.putBool("wifiCfg", wifiConfigured);
    preferences.putString("wifiMode", wifiMode);
    preferences.putString("wifiSSID", wifiStaSSID);
    preferences.putString("wifiPASS", wifiStaPASS);
    preferences.putString("wifiIP", wifiStaticIP);
    preferences.putString("wifiNM", wifiNetmask);
    preferences.putString("wifiGW", wifiGateway);
    preferences.putString("apSSID", wifiApSSID);
    preferences.putString("apPASS", wifiApPASS);
    preferences.end();
}

void saveWiSUNConfig() {
    preferences.begin("gw-config", false);
    preferences.putString("wisunChrate", wiSunChrate);
    preferences.putString("multicast", MULTICAST_ADDRESS);
    preferences.end();
}

static void saveWiSUNConnectionState() {
    preferences.begin("gw-config", false);
    preferences.putBool("wisunInit", wiSunInitialized);
    preferences.putBool("wisunConn", isWiSUNConnected);
    preferences.putString("globalIPv6", globalIPv6Address);
    preferences.putString("localIPv6", localIPv6Address);
    preferences.putString("wisunChrate", wiSunChrate);
    preferences.end();
}

void saveCheckIntervalConfig() {
    preferences.begin("gw-config", false);
    preferences.putULong("checkIntSec", (uint32_t)g_checkIntervalSec);
    preferences.end();
}

bool tryStaConnect(const char* ssid, const char* pass, bool useStaticIP = false) {
    if (!ssid || strlen(ssid) == 0) return false;
    Serial.printf("[WIFI] Connecting STA: %s\n", ssid);
    SerialBT.println("[WIFI] STA connect...");

    // Manual retries own the connection state. Auto reconnect would start a
    // competing connection between attempts and make the next scan fail.
    WiFi.setAutoReconnect(false);

    // Switch to STA first — this cleanly stops AP and its DHCP server.
    static bool staReady = false;
    if (!staReady) {
        WiFi.mode(WIFI_STA);            // stops AP, enables STA in one call
        delay(200);
        WiFi.disconnect(false);         // clear stale STA connection, keep Wi-Fi running
        delay(200);
        WiFi.setSleep(false);
        staReady = true;
    } else {
        WiFi.disconnect(false);         // lightweight disconnect, keep STA mode
        delay(150);
    }

    if (useStaticIP && strlen(wifiStaticIP) > 0 && strlen(wifiGateway) > 0) {
        IPAddress ip, gw, nm;
        if (ip.fromString(wifiStaticIP) && gw.fromString(wifiGateway) && nm.fromString(wifiNetmask)) {
            Serial.printf("[WIFI] Static IP=%s GW=%s NM=%s\n", wifiStaticIP, wifiGateway, wifiNetmask);
            WiFi.config(ip, gw, nm);
        }
    } else {
        Serial.println("[WIFI] Using DHCP");
        const IPAddress dhcpAddress(0, 0, 0, 0);
        WiFi.config(dhcpAddress, dhcpAddress, dhcpAddress);
    }

    // Pre-scan using default IDF parameters (NULL config) to avoid
    // BLE coexistence warning about non-default active scan time.
    bool hasBssid = false;
    if (esp_wifi_scan_start(nullptr, true) == ESP_OK) {
        uint16_t apTotal = 0;
        if (esp_wifi_scan_get_ap_num(&apTotal) == ESP_OK && apTotal > 0) {
            wifi_ap_record_t* aps = (wifi_ap_record_t*)malloc(sizeof(wifi_ap_record_t) * apTotal);
            if (aps) {
                uint16_t apCount = apTotal;
                if (esp_wifi_scan_get_ap_records(&apCount, aps) == ESP_OK) {
                    for (uint16_t i = 0; i < apCount; i++) {
                        if (strcmp((const char*)aps[i].ssid, ssid) == 0) {
                            hasBssid = true;
                            Serial.printf("[WIFI] Found '%s' on ch%d RSSI=%d\n", ssid, (int)aps[i].primary, (int)aps[i].rssi);
                            break;
                        }
                    }
                    if (!hasBssid) {
                        Serial.printf("[WIFI] '%s' not found in %u APs\n", ssid, (unsigned)apCount);
                    }
                }
                free(aps);
            } else {
                Serial.printf("[WIFI] AP scan buffer alloc failed (%u APs)\n", (unsigned)apTotal);
            }
        }
    }

    g_lastStaDiscReason = 0;
    WiFi.begin(ssid, pass);
    // Allow retries for transient AUTH_EXPIRE/handshake failures within this
    // attempt; disable it again before the outer loop starts a new attempt.
    WiFi.setAutoReconnect(true);

    unsigned long t0 = millis();
    while (!hasValidStaIp() && millis() - t0 < 18000) {
        serviceWatchdog();
        delay(300);
        Serial.print('.');
    }
    Serial.println();
    wl_status_t finalStatus = WiFi.status();
    bool ok = hasValidStaIp();
    if (!ok) {
        Serial.printf("[WIFI] STA connect failed. status=%d reason=%u\n", (int)finalStatus, (unsigned)g_lastStaDiscReason);
        WiFi.setAutoReconnect(false);
        WiFi.disconnect(false);
        delay(300);
    }
    if (!ok) staReady = false;          // next call will do full reinit
    return ok;
}

bool applyWiFiSettings() {
    if (strcmp(wifiMode, "ap") == 0 || strlen(wifiStaSSID) == 0) {
        WiFi.disconnect(true);
        WiFi.mode(WIFI_AP);
        bool ok = WiFi.softAP(wifiApSSID, wifiApPASS);
        if (ok) {
            Serial.printf("[WIFI] SoftAP started. SSID=%s IP=%s\n", wifiApSSID, WiFi.softAPIP().toString().c_str());
            SerialBT.printf("[WIFI] AP IP=%s\n", WiFi.softAPIP().toString().c_str());
        } else {
            Serial.println("[WIFI] SoftAP start failed");
        }
        return ok;
    }

    bool useStatic = strlen(wifiStaticIP) > 0 && strlen(wifiGateway) > 0;
    bool ok = false;
    for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.printf("[WIFI] STA attempt %d/3\n", attempt);
        ok = tryStaConnect(wifiStaSSID, wifiStaPASS, useStatic);
        if (ok) break;
        delay(500);
    }
    if (ok) {
        WiFi.setAutoReconnect(true);
        Serial.printf("[WIFI] STA connected. IP=%s\n", WiFi.localIP().toString().c_str());
        SerialBT.printf("[WIFI] STA IP=%s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("[WIFI] STA connect failed after retries");
        SerialBT.println("[WIFI] STA connect fail");

        WiFi.disconnect(false);
        WiFi.mode(WIFI_AP_STA);
        bool apOk = WiFi.softAP(wifiApSSID, wifiApPASS);
        if (apOk) {
            Serial.printf("[WIFI] Fallback AP started. SSID=%s IP=%s\n", wifiApSSID, WiFi.softAPIP().toString().c_str());
            SerialBT.printf("[WIFI] AP IP=%s\n", WiFi.softAPIP().toString().c_str());
        }
        WiFi.setAutoReconnect(true);
        WiFi.begin(wifiStaSSID, wifiStaPASS);
        Serial.println("[WIFI] STA background reconnect enabled while fallback AP remains available");
        return apOk;
    }
    return ok;
}

// ============================================================================
//  BLE device name
// ============================================================================
void updateBluetoothDeviceName() {
    char name[50];
    snprintf(name, sizeof(name), "BRD_%s", strlen(routerNumber) > 0 ? routerNumber : "1");
    if (bluetoothStarted) { SerialBT.end(); delay(50); bluetoothStarted = false; }
    if (SerialBT.begin(name)) { bluetoothStarted = true; Serial.printf("[BLE] %s\n", name); }
    else Serial.println("[BLE] Failed.");
}

void inputWiFiSettings() {
    char tmp[65];
    readSerialInput("WiFi mode (sta/ap)", wifiMode, tmp, sizeof(tmp));
    if (strlen(tmp) > 0) {
        if (strcasecmp(tmp, "ap") == 0) strncpy(wifiMode, "ap", sizeof(wifiMode));
        else strncpy(wifiMode, "sta", sizeof(wifiMode));
    }

    if (strcmp(wifiMode, "ap") == 0) {
        readSerialInput("AP SSID", wifiApSSID, tmp, sizeof(tmp));
        if (strlen(tmp) > 0) strncpy(wifiApSSID, tmp, sizeof(wifiApSSID) - 1);
        wifiApSSID[sizeof(wifiApSSID) - 1] = '\0';

        while (true) {
            readSerialInput("AP password (min 8 chars)", wifiApPASS, tmp, sizeof(tmp));
            if (strlen(tmp) >= 8) {
                strncpy(wifiApPASS, tmp, sizeof(wifiApPASS) - 1);
                wifiApPASS[sizeof(wifiApPASS) - 1] = '\0';
                break;
            }
            Serial.println("[WIFI] AP password must be at least 8 characters.");
            SerialBT.println("[WIFI] AP password must be at least 8 characters.");
        }
    } else {
        readSerialInput("WiFi SSID", wifiStaSSID, tmp, sizeof(wifiStaSSID));
        strncpy(wifiStaSSID, tmp, sizeof(wifiStaSSID));

        readSerialInput("WiFi Password", wifiStaPASS, tmp, sizeof(wifiStaPASS));
        strncpy(wifiStaPASS, tmp, sizeof(wifiStaPASS));

        readSerialInput("Static IP (dhcp/0=DHCP)", wifiStaticIP, tmp, sizeof(wifiStaticIP));
        // Allow user to explicitly clear static IP for DHCP
        if (strcasecmp(tmp, "dhcp") == 0 || strcmp(tmp, "0") == 0 || strcasecmp(tmp, "none") == 0) {
            tmp[0] = '\0';
        }
        strncpy(wifiStaticIP, tmp, sizeof(wifiStaticIP));

        if (strlen(wifiStaticIP) > 0) {
            readSerialInput("Netmask", wifiNetmask, tmp, sizeof(wifiNetmask));
            strncpy(wifiNetmask, tmp, sizeof(wifiNetmask));

            readSerialInput("Gateway (ex: 192.168.1.1)", wifiGateway, tmp, sizeof(wifiGateway));
            strncpy(wifiGateway, tmp, sizeof(wifiGateway));
        } else {
            wifiStaticIP[0] = '\0';
            wifiGateway[0] = '\0';
            strncpy(wifiNetmask, "255.255.255.0", sizeof(wifiNetmask));
            Serial.println("[WIFI] DHCP mode selected");
        }
    }

    wifiConfigured = true;
    saveWiFiConfig();
    bool ok = applyWiFiSettings();
    Serial.printf("[WIFI] Apply result: %s\n", ok ? "OK" : "FAIL");
}

void quickScanAndConnectWiFi() {
    Serial.println("[WIFI] Scanning nearby APs...");
    SerialBT.println("[WIFI] Scanning APs...");

    wifi_mode_t prevMode;
    esp_wifi_get_mode(&prevMode);

    auto restoreAfterScanAbort = [&]() {
        if (prevMode == WIFI_MODE_AP || prevMode == WIFI_MODE_APSTA) {
            WiFi.mode(prevMode == WIFI_MODE_AP ? WIFI_AP : WIFI_AP_STA);
            delay(120);
            (void)WiFi.softAP(wifiApSSID, wifiApPASS);
        }
        if (strcmp(wifiMode, "sta") == 0 && strlen(wifiStaSSID) > 0) {
            WiFi.setAutoReconnect(true);
            WiFi.begin(wifiStaSSID, wifiStaPASS);
        }
    };

    // Pause fallback STA reconnects while the synchronous scan runs.
    // Otherwise a failed boot connection can keep the Wi-Fi driver busy and
    // prevent this menu action from completing reliably.
    bool resumeStaReconnect = strcmp(wifiMode, "sta") == 0 && strlen(wifiStaSSID) > 0;
    if (resumeStaReconnect) {
        WiFi.setAutoReconnect(false);
        WiFi.disconnect(false);
        delay(200);
    }

    // Force a clean STA-only scan phase to avoid APSTA reconnect contention.
    esp_wifi_scan_stop();
    WiFi.mode(WIFI_STA);
    delay(250);
    WiFi.disconnect(false);
    delay(150);

    uint16_t apCount = 0;
    esp_err_t scanErr = esp_wifi_scan_start(nullptr, true);
    esp_err_t countErr = scanErr == ESP_OK ? esp_wifi_scan_get_ap_num(&apCount) : scanErr;
    if (scanErr != ESP_OK || countErr != ESP_OK || apCount == 0) {
        Serial.printf("[WIFI] Scan failed: scan=%s count=%s APs=%u\n",
                      esp_err_to_name(scanErr), esp_err_to_name(countErr), apCount);
        SerialBT.printf("[WIFI] Scan failed: scan=%s count=%s APs=%u\n",
                        esp_err_to_name(scanErr), esp_err_to_name(countErr), apCount);
        restoreAfterScanAbort();
        return;
    }

    wifi_ap_record_t* aps = (wifi_ap_record_t*)malloc(sizeof(wifi_ap_record_t) * apCount);
    if (!aps) {
        Serial.println("[WIFI] Scan buffer alloc failed.");
        SerialBT.println("[WIFI] Scan alloc failed.");
        restoreAfterScanAbort();
        return;
    }

    uint16_t fetched = apCount;
    if (esp_wifi_scan_get_ap_records(&fetched, aps) != ESP_OK || fetched == 0) {
        Serial.println("[WIFI] Scan fetch failed.");
        SerialBT.println("[WIFI] Scan fetch failed.");
        free(aps);
        restoreAfterScanAbort();
        return;
    }

    // Sort by RSSI descending and show top 10.
    for (uint16_t i = 0; i + 1 < fetched; i++) {
        for (uint16_t j = i + 1; j < fetched; j++) {
            if (aps[j].rssi > aps[i].rssi) {
                wifi_ap_record_t t = aps[i];
                aps[i] = aps[j];
                aps[j] = t;
            }
        }
    }

    uint16_t top = fetched > 10 ? 10 : fetched;
    Serial.printf("[WIFI] Top %u APs:\n", (unsigned)top);
    Serial.println("  -----------------------------------------------");
    Serial.println("   #  SSID                           CH   RSSI  ENC");
    Serial.println("  -----------------------------------------------");
    for (uint16_t i = 0; i < top; i++) {
        const char* enc;
        switch (aps[i].authmode) {
            case WIFI_AUTH_OPEN:            enc = "OPEN"; break;
            case WIFI_AUTH_WEP:             enc = "WEP";  break;
            case WIFI_AUTH_WPA_PSK:         enc = "WPA";  break;
            case WIFI_AUTH_WPA2_PSK:        enc = "WPA2"; break;
            case WIFI_AUTH_WPA_WPA2_PSK:    enc = "WPA/2"; break;
            case WIFI_AUTH_WPA3_PSK:        enc = "WPA3"; break;
            case WIFI_AUTH_WPA2_WPA3_PSK:   enc = "WPA2/3"; break;
            default:                        enc = "?";    break;
        }
        Serial.printf("  %2u) %-30s  %2d  %4d  %s\n",
                      (unsigned)(i + 1), (const char*)aps[i].ssid, (int)aps[i].primary, (int)aps[i].rssi, enc);
    }
    Serial.println("  -----------------------------------------------");

    char selBuf[16] = "0";
    readSerialInput("Select AP number (0=cancel)", "0", selBuf, sizeof(selBuf));
    int sel = atoi(selBuf);
    if (sel <= 0 || sel > (int)top) {
        Serial.println("[WIFI] AP selection cancelled.");
        SerialBT.println("[WIFI] AP selection cancelled.");
        free(aps);
        restoreAfterScanAbort();
        return;
    }

    wifi_ap_record_t picked = aps[sel - 1];
    free(aps);

    char prevWifiMode[8];
    char prevSSID[33];
    char prevPASS[65];
    char prevIP[16];
    char prevNM[16];
    char prevGW[16];
    bool prevConfigured = wifiConfigured;
    strncpy(prevWifiMode, wifiMode, sizeof(prevWifiMode));
    strncpy(prevSSID, wifiStaSSID, sizeof(prevSSID));
    strncpy(prevPASS, wifiStaPASS, sizeof(prevPASS));
    strncpy(prevIP, wifiStaticIP, sizeof(prevIP));
    strncpy(prevNM, wifiNetmask, sizeof(prevNM));
    strncpy(prevGW, wifiGateway, sizeof(prevGW));

    char ssidBuf[33] = "";
    strncpy(ssidBuf, (const char*)picked.ssid, sizeof(ssidBuf) - 1);
    ssidBuf[sizeof(ssidBuf) - 1] = '\0';

    char passBuf[65] = "";
    if (picked.authmode != WIFI_AUTH_OPEN) {
        readSerialInput("WiFi Password", "", passBuf, sizeof(passBuf));
    }

    strncpy(wifiMode, "sta", sizeof(wifiMode) - 1);
    wifiMode[sizeof(wifiMode) - 1] = '\0';
    strncpy(wifiStaSSID, ssidBuf, sizeof(wifiStaSSID) - 1);
    wifiStaSSID[sizeof(wifiStaSSID) - 1] = '\0';
    strncpy(wifiStaPASS, passBuf, sizeof(wifiStaPASS) - 1);
    wifiStaPASS[sizeof(wifiStaPASS) - 1] = '\0';

    // Force DHCP for quick-select flow.
    wifiStaticIP[0] = '\0';
    wifiGateway[0] = '\0';
    strncpy(wifiNetmask, "255.255.255.0", sizeof(wifiNetmask) - 1);
    wifiNetmask[sizeof(wifiNetmask) - 1] = '\0';

    wifiConfigured = true;
    applyWiFiSettings();
    bool staOk = hasValidStaIp();
    if (staOk) {
        saveWiFiConfig();
        Serial.println("[WIFI] Quick connect saved.");
        SerialBT.println("[WIFI] Quick connect saved.");
    } else {
        strncpy(wifiMode, prevWifiMode, sizeof(wifiMode));
        strncpy(wifiStaSSID, prevSSID, sizeof(wifiStaSSID));
        strncpy(wifiStaPASS, prevPASS, sizeof(wifiStaPASS));
        strncpy(wifiStaticIP, prevIP, sizeof(wifiStaticIP));
        strncpy(wifiNetmask, prevNM, sizeof(wifiNetmask));
        strncpy(wifiGateway, prevGW, sizeof(wifiGateway));
        wifiConfigured = prevConfigured;
        Serial.println("[WIFI] Quick connect not saved (STA failed).");
        SerialBT.println("[WIFI] Quick connect not saved.");
    }
    Serial.printf("[WIFI] Quick connect STA: %s\n", staOk ? "OK" : "FAIL");
    SerialBT.printf("[WIFI] Quick connect STA: %s\n", staOk ? "OK" : "FAIL");
}

void inputWiSUNParamSettings() {
    char tmp[32] = "";
    readSerialInput("Wi-SUN chrate (ex: 50kbps/150kbps)", wiSunChrate, tmp, sizeof(tmp));
    if (strlen(tmp) > 0) {
        String s = String(tmp);
        s.trim();
        s.toLowerCase();
        s.replace(" ", "");

        if (s == "50" || s == "50k" || s == "50kb" || s == "50kps" || s == "50kpbs") s = "50kbps";
        else if (s == "150" || s == "150k" || s == "150kb" || s == "150kps" || s == "150kpbs") s = "150kbps";

        if (s == "50kbps" || s == "150kbps") {
            strncpy(wiSunChrate, s.c_str(), sizeof(wiSunChrate) - 1);
            wiSunChrate[sizeof(wiSunChrate) - 1] = '\0';
        } else {
            Serial.println("[WiSUN] Invalid chrate. Use 50kbps or 150kbps.");
            SerialBT.println("[WiSUN] Invalid chrate. Use 50kbps or 150kbps.");
            Serial.printf("[WiSUN] Keeping previous chrate=%s\n", wiSunChrate);
            SerialBT.printf("[WiSUN] Keep=%s\n", wiSunChrate);
            return;
        }
    }

    char mtmp[40] = "";
    readSerialInput("Multicast address (ex: ff15::810a:64d1)", MULTICAST_ADDRESS, mtmp, sizeof(mtmp));
    if (strlen(mtmp) > 0) {
        String s = String(mtmp);
        s.trim();
        if (!s.startsWith("<")) s = "<" + s;
        if (!s.endsWith(">"))   s = s + ">";
        if (s.length() < sizeof(MULTICAST_ADDRESS)) {
            strncpy(MULTICAST_ADDRESS, s.c_str(), sizeof(MULTICAST_ADDRESS) - 1);
            MULTICAST_ADDRESS[sizeof(MULTICAST_ADDRESS) - 1] = '\0';
        } else {
            Serial.println("[WiSUN] Multicast address too long — keeping previous value.");
            SerialBT.println("[WiSUN] Multicast too long — kept previous.");
        }
    }

    saveWiSUNConfig();
    Serial.printf("[WiSUN] chrate=%s multicast=%s (saved). Run 'i' to apply.\n", wiSunChrate, MULTICAST_ADDRESS);
    SerialBT.printf("[WiSUN] chrate=%s multicast=%s saved. run i\n", wiSunChrate, MULTICAST_ADDRESS);
}

// ============================================================================
//  Menu
// ============================================================================
void inputDongHoSensor() {
    char tmp[65];
    readSerialInput("dong",         dong,               tmp, sizeof(dong));   strncpy(dong, tmp, sizeof(dong));
    readSerialInput("ho",           ho,                 tmp, sizeof(ho));     strncpy(ho,   tmp, sizeof(ho));
    char defn[8]; snprintf(defn, sizeof(defn), "%d", sensorNumber);
    readSerialInput("sensorNumber", defn,               tmp, sizeof(tmp));
    int n = atoi(tmp); if (n >= 1 && n <= 999) sensorNumber = n;
    saveDongHoConfig(); updateBluetoothDeviceName(); blinkGreenTwice();
    Serial.printf("[CONFIG] dong=%s ho=%s sensor=%d\n", dong, ho, sensorNumber);
}

void inputRouterNumber() {
    char tmp[32] = "";
    readSerialInput("routerNumber", routerNumber, tmp, sizeof(tmp));
    if (strlen(tmp) > 0) {
        size_t j = 0;
        for (size_t i = 0; i < strlen(tmp) && j < sizeof(routerNumber) - 1; i++) {
            if (tmp[i] >= '0' && tmp[i] <= '9') routerNumber[j++] = tmp[i];
        }
        routerNumber[j] = '\0';
    }
    if (strlen(routerNumber) == 0) strncpy(routerNumber, "1", sizeof(routerNumber) - 1);
    saveDongHoConfig();
    updateBluetoothDeviceName();
    blinkGreenTwice();
    Serial.printf("[ROUTER] BRD_%s\n", routerNumber);
    SerialBT.printf("[ROUTER] BRD_%s\n", routerNumber);
}

void printMenu() {
    const char* m =
        "\n====== Wi-SUN BR Gateway ======\n"
        "  ?  - Status\n"
        "  /  - Show menu\n"
        "  l  - Data logging ON/OFF\n"
        "  r  - Router number (BRD_routerNumber)\n"
        "  w  - WiFi setup (uplink STA or AP SSID/PASS)\n"
        "  q  - Quick AP scan/connect (DHCP)\n"
        "  p  - Wi-SUN parameter setup (chrate/multicast)\n"
        "  m  - Monitor incoming Wi-SUN text\n"
        "  b  - Show BR IPv6 address\n"
        "  i  - Initialize Wi-SUN module\n"
        "  a  - AT command mode (direct Wi-SUN comms)\n"
        "  n  - Show connected Wi-SUN nodes (conlist)\n"
        "  s  - Start gateway\n"
        "================================\n";
    Serial.print(m);
    SerialBT.print(m);
}

void printStatus() {
    Serial.printf("router=%s  dong=%s  ho=%s  sensor=%d\n", routerNumber, dong, ho, sensorNumber);
    Serial.printf("Wi-SUN: %s\n", wiSunInitialized && isWiSUNConnected ? "Connected" :
                                  wiSunInitialized                     ? "Init(not connected)" : "Not init");
    if (wiSunInitialized) Serial.printf("  GloabalIPv6: %s\n", globalIPv6Address);
    Serial.printf("WiFi : mode=%s cfg=%s status=%s\n", wifiMode, wifiConfigured ? "Y" : "N",
                  hasValidStaIp() ? "Connected" : "Not connected");
    if (hasValidStaIp()) Serial.printf("  WiFi IP: %s\n", WiFi.localIP().toString().c_str());
    if (WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA) {
        Serial.printf("  AP: %s IP=%s\n", wifiApSSID, WiFi.softAPIP().toString().c_str());
    }
    Serial.printf("Nodes: %d\n", g_nodeCount);
    Serial.printf("WiSUN: chrate=%s\n", wiSunChrate);
    Serial.printf("WiSUN: multicast=%s\n", MULTICAST_ADDRESS);
    Serial.printf("Data logging: %s\n", g_dataLoggingEnabled ? "ON" : "OFF");

    SerialBT.printf("router=%s  dong=%s  ho=%s  sensor=%d\n", routerNumber, dong, ho, sensorNumber);
    SerialBT.printf("Wi-SUN: %s\n", wiSunInitialized && isWiSUNConnected ? "Connected" :
                                   wiSunInitialized                     ? "Init(not connected)" : "Not init");
    if (wiSunInitialized) SerialBT.printf("  GlobalIPv6: %s\n", globalIPv6Address);
    SerialBT.printf("WiFi : mode=%s cfg=%s status=%s\n", wifiMode, wifiConfigured ? "Y" : "N",
                    hasValidStaIp() ? "Connected" : "Not connected");
    if (hasValidStaIp()) SerialBT.printf("  WiFi IP: %s\n", WiFi.localIP().toString().c_str());
    if (WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA) {
        SerialBT.printf("  AP: %s IP=%s\n", wifiApSSID, WiFi.softAPIP().toString().c_str());
    }
    SerialBT.printf("Nodes: %d\n", g_nodeCount);
    SerialBT.printf("WiSUN: chrate=%s\n", wiSunChrate);
    SerialBT.printf("WiSUN: multicast=%s\n", MULTICAST_ADDRESS);
    SerialBT.printf("Data logging: %s\n", g_dataLoggingEnabled ? "ON" : "OFF");
}

void printBorderRouterAddress() {
    Serial.println("[BR] Border Router Address Info");
    SerialBT.println("[BR] Address Info");

    if (!wiSunInitialized) {
        Serial.println("[BR] Wi-SUN not initialized. Run 'n' first.");
        SerialBT.println("[BR] Not initialized. Use 'n'.");
        return;
    }

    Serial.printf("[BR] Global IPv6: %s\n", strlen(globalIPv6Address) ? globalIPv6Address : "(empty)");
    Serial.printf("[BR] Local  IPv6: %s\n", strlen(localIPv6Address)  ? localIPv6Address  : "(empty)");
    Serial.printf("[BR] Link state : %s\n", isWiSUNConnected ? "Connected" : "Init(not connected)");

    SerialBT.printf("[BR] Global: %s\n", strlen(globalIPv6Address) ? globalIPv6Address : "(empty)");
    SerialBT.printf("[BR] Local : %s\n", strlen(localIPv6Address)  ? localIPv6Address  : "(empty)");
}

void printConnectedNodes() {
    Serial.println("[BR] Connected Wi-SUN nodes (conlist)");
    SerialBT.println("[BR] Connected nodes");
    if (!wiSunInitialized) {
        Serial.println("[BR] Wi-SUN not initialized. Run 'n' init first.");
        SerialBT.println("[BR] Not initialized. Use 'n'.");
        return;
    }
    String r = sendCommand("conlist\r\n");
    if (r.length() == 0) {
        Serial.println("(no response from BR)");
        SerialBT.println("(no response)");
        return;
    }
    Serial.println(r.c_str());
    SerialBT.println(r.c_str());
}

void monitorWiSUNTextMode() {
    Serial.println("[MON] Wi-SUN text monitor mode. '/' to exit.");
    SerialBT.println("[MON] Wi-SUN text monitor. '/' exit.");

    if (!wiSunInitialized) {
        Serial.println("[MON] Wi-SUN not initialized yet. You can still watch raw UART text.");
        SerialBT.println("[MON] Wi-SUN not initialized.");
    }

    char line[256] = "";
    int idx = 0;
    unsigned long lastCharTime = 0;

    while (true) {
        serviceWatchdog();

        if (inputAvailable() && peekInput() == '/') {
            readInput();
            flushInput();
            Serial.println("\n[MON] Exit.");
            SerialBT.println("[MON] Exit");
            return;
        }

        while (WiSUNSerial.available()) {
            char c = WiSUNSerial.read();
            lastCharTime = millis();

            if (c == '\n') {
                if (idx > 0) {
                    line[idx] = '\0';
                    Serial.printf("[WiSUN RAW] %s\n", line);
                    SerialBT.printf("[RAW] %s\n", line);
                    idx = 0;
                }
            } else if (c != '\r') {
                if (idx < (int)sizeof(line) - 1) {
                    line[idx++] = c;
                }
            }
        }

        if (idx > 0 && millis() - lastCharTime >= 1500) {
            line[idx] = '\0';
            Serial.printf("[WiSUN RAW] %s\n", line);
            SerialBT.printf("[RAW] %s\n", line);
            idx = 0;
        }

        delay(10);
    }
}

void atCommandMode() {
    Serial.println("[AT MODE] Entering direct Wi-SUN command mode...");
    SerialBT.println("[AT MODE] Started");

    // Always ensure UART2 is open before entering AT mode.
    // In manual-init flow, wiSunInitialized may come from NVS while UART2 is not opened yet.
    WiSUNSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(100);
    Serial.printf("[AT] UART2 ready: RX=%d, TX=%d, Speed=115200\n", RX_PIN, TX_PIN);

    if (wiSunInitialized) {
        Serial.println("[AT] Keeping current Wi-SUN session (no reset).");
        if (ensureATCommandMode()) {
            Serial.println("[AT] Command mode ready.");
        } else {
            Serial.println("[AT] WARNING: Could not confirm command mode.");
            if (forceATModeByHardwareReset()) {
                Serial.println("[AT] Command mode recovered by hardware reset.");
                isWiSUNConnected = false;
                g_wisunRuntimeReady = false;
            } else {
                Serial.println("[AT] Recovery failed. Try 'i' (manual init) then re-enter AT mode.");
            }
        }
    }
    
    Serial.println("[AT MODE] Ready. Enter commands. Type '/' to exit.");
    while (true) {
        checkWiSUNUnsolicited(); serviceWatchdog();
        
        String input = "";
        while (true) {
            serviceWatchdog();
            checkWiSUNUnsolicited();
            if (!inputAvailable()) { delay(50); continue; }
            
            char c = readInput();
            
            // Exit on '/'
            if (c == '/') {
                Serial.println("\n[AT MODE] Exiting...");
                return;
            }
            
            // Enter key - send command
            if (c == '\r' || c == '\n') {
                Serial.println(); // newline after input
                if (input.length() == 0) break; // Empty line, just continue

                // Helper: send <ipv6> <text>  ->  udps <ipv6> <text>
                String cmdInput = input;
                String cmd;
                if (cmdInput.startsWith("send ")) {
                    int sp = cmdInput.indexOf(' ', 5);
                    if (sp > 5 && sp < (int)cmdInput.length() - 1) {
                        String dst = cmdInput.substring(5, sp);
                        String msg = cmdInput.substring(sp + 1);
                        cmd = "udps " + dst + " " + msg + "\r\n";
                        Serial.printf("[AT] helper: send -> %s\n", cmd.c_str());
                    } else {
                        Serial.println("[AT] usage: send <ipv6> <text>");
                        break;
                    }
                } else {
                    // Raw AT command passthrough
                    cmd = cmdInput + "\r\n";
                }

                // Send command to Wi-SUN module with CR+LF
                Serial.printf("[AT] TX: '%s' (", input.c_str());
                for (size_t i = 0; i < cmd.length(); i++) {
                    Serial.printf("%02X", (unsigned char)cmd[i]);
                    if (i < cmd.length() - 1) Serial.print(" ");
                }
                Serial.println(")");
                
                size_t written = WiSUNSerial.write((const uint8_t*)cmd.c_str(), cmd.length());
                WiSUNSerial.flush();
                Serial.printf("[AT] Written %d bytes to UART2\n", written);
                
                // Wait for response
                char resp[512] = ""; int ri = 0;
                unsigned long t0 = millis(), lastRx = t0;
                unsigned long waitMs = 5000;
                if (input.equalsIgnoreCase("conlist")) waitMs = 8000;
                while (millis() - t0 < waitMs) {
                    serviceWatchdog();
                    while (WiSUNSerial.available() && ri < (int)sizeof(resp)-1) {
                        resp[ri++] = WiSUNSerial.read();
                        Serial.print(resp[ri-1]);
                        lastRx = millis();
                    }
                    if (ri > 0 && millis() - lastRx > 300) break;
                    delay(10);
                }
                resp[ri] = '\0';
                if (ri == 0) {
                    Serial.println("\n[AT] (no response)");
                    Serial.println("[AT] Hint: module may be in Transparent mode. Try '+++' then retry.");
                }
                else Serial.println();
                break; // Back to prompt
            } else if (c >= 32 && c < 127) { // Printable characters
                Serial.print(c); // Echo character
                input += c;
            }
        }
    }
}

void runMenu() {
    g_menuActive = true;
    printMenu();
    while (true) {
        checkWiSUNUnsolicited(); serviceWatchdog();
        if (g_wifiOledUpdatePending) {
            g_wifiOledUpdatePending = false;
            updateOledStatus();
        }
        if (!inputAvailable()) { delay(100); continue; }
        char c = tolower(readInput());
        if (c == '\r' || c == '\n') continue;
        while (uart0RxAvailable()) uart0RxRead();
        if (bluetoothStarted) while (SerialBT.available()) SerialBT.read();
        if      (c == '?') { printStatus(); }
        else if (c == '/') { printMenu(); }
        else if (c == 'l') {
            g_dataLoggingEnabled = !g_dataLoggingEnabled;
            Serial.printf("[LOG] Data logging %s\n", g_dataLoggingEnabled ? "ON" : "OFF");
            SerialBT.printf("[LOG] Data logging %s\n", g_dataLoggingEnabled ? "ON" : "OFF");
            printMenu();
        }
        else if (c == 'r') { inputRouterNumber(); printMenu(); }
        else if (c == 'w') { inputWiFiSettings(); printMenu(); }
        else if (c == 'q') { quickScanAndConnectWiFi(); printMenu(); }
        else if (c == 'p') { inputWiSUNParamSettings(); printMenu(); }
        else if (c == 'm') { monitorWiSUNTextMode(); printMenu(); }
        else if (c == 'b') { printBorderRouterAddress(); }
        else if (c == 'n') { printConnectedNodes(); printMenu(); }
        else if (c == 'i') {
            Serial.println("[INIT] Initializing Wi-SUN module...");
            initializeWiSUN();
            Serial.println("[INIT] Wi-SUN module initialization complete.");
            if (wiSunInitialized && isWiSUNConnected) {
                Serial.println("[INIT] Entering AT command wait mode automatically...");
                atCommandMode();
            }
            printMenu();
        }
        else if (c == 'a') {
            atCommandMode();
            printMenu();
        }
        else if (c == 's') {
            Serial.println("[GW] Running. 'q' or '/' = menu.");
            SerialBT.println("[GW] Running. 'q' or '/' = menu.");
            g_menuActive = false;
            return;
        }
        else Serial.println("[MENU] Unknown cmd — '?' or '/' for help.");
    }
}

// ============================================================================
//  setup()
// ============================================================================
void setup() {
    Serial.begin(115200);

    // Reinstall UART0 in AEC mode (no event queue)
    uart_driver_delete(UART_NUM_0);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    {
        uart_config_t cfg = { .baud_rate=115200, .data_bits=UART_DATA_8_BITS,
            .parity=UART_PARITY_DISABLE, .stop_bits=UART_STOP_BITS_1,
            .flow_ctrl=UART_HW_FLOWCTRL_DISABLE, .source_clk=UART_SCLK_DEFAULT };
        uart_param_config(UART_NUM_0, &cfg);
        uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    delay(2000);

    esp_log_level_set("BT_HCI", ESP_LOG_ERROR);
    esp_log_level_set("BT_BTM", ESP_LOG_ERROR);
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);
    esp_log_level_set("httpd_uri", ESP_LOG_ERROR);
    esp_log_level_set("httpd_ws", ESP_LOG_ERROR);

    WiFi.onEvent(onWiFiEvent);
    g_wsSendMutex = xSemaphoreCreateMutex();

    Serial.println("============================");
    Serial.println("  Wi-SUN BR Gateway Firmware");
    Serial.println("============================");

    loadAptSetup();
    loadIpMappingTable();

    loadConfig();
    loadRuntimeState();
    helmetDirLoad();
    updateBluetoothDeviceName();

    initOled();
    updateOledStatus();

    pinMode(RED_LED_PIN,   OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(TXON_PIN,      OUTPUT);
    pinMode(RESETN_PIN,    OUTPUT);
    pinMode(WAKEUP_PIN,    OUTPUT);

    // Requested LED policy:
    // 1) Power-on: red
    // 2) Wi-Fi connected: yellow
    // 3) Wi-SUN net state 5: green (handled inside initializeWiSUN)
    setLEDColor("red");

    // Always bring up Wi-Fi (AP fallback when STA settings are empty or STA retries fail) for Web UI.
    bool wifiOk = applyWiFiSettings();
    if (wifiOk && hasValidStaIp()) {
        setLEDColor("yellow");
    } else {
        setLEDColor("red");
    }
    startWebServer();

    // Reflect WiFi's real state right away — initializeWiSUN() below can block
    // for a long time (quick-reconnect up to ~60s, full init up to ~180s), and
    // loop() (where updateOledStatus() normally runs) isn't reached until
    // setup() returns, so without this the OLED would stay on its pre-WiFi
    // boot screen for that entire stretch even though WiFi already connected.
    updateOledStatus();

    Serial.println("[AUTO] Starting Wi-SUN auto init on boot...");
    initializeWiSUN();
    if (isWiSUNConnected) {
        Serial.println("[AUTO] Wi-SUN joined (net state 5) - LED green.");
    } else {
        Serial.println("[AUTO] Wi-SUN not joined. Use menu 'i' to retry init.");
    }

    updateOledStatus(); // reflect Wi-SUN's final boot state without waiting for the first loop() tick
}

// ============================================================================
//  loop()  — gateway operation
// ============================================================================
void loop() {
    SerialBT.check();
    checkWiSUNUnsolicited();
    serviceWatchdog();

    unsigned long now = millis();

    serviceWebServerHealth(now);

    if (!g_httpServer && (g_nextWebRetryMs != 0) && (long)(now - g_nextWebRetryMs) >= 0) {
        startWebServer();
    }

    if (g_runtimeStateDirty && (now - g_lastRuntimeStateSaveMs >= RUNTIME_STATE_SAVE_INTERVAL_MS)) {
        saveRuntimeState();
    }

    if (g_scheduledRestartMs != 0 && (long)(now - g_scheduledRestartMs) >= 0) {
        if (g_runtimeStateDirty) saveRuntimeState();
        Serial.println("[OTA] Restarting after successful update upload");
        delay(150);
        ESP.restart();
    }

    bool statusCheckPending = false;
    // If a sensor stops responding, downgrade to COM so UI does not keep stale last status forever.
    for (int i = 0; i < g_cellStateCount; i++) {
        bool statusTimedOut = g_cellStates[i].awaitingStatusCheck &&
                              (long)(now - g_cellStates[i].statusCheckDeadlineMs) >= 0;
        if (statusTimedOut) {
            g_cellStates[i].awaitingStatusCheck = false;
            g_cellStates[i].statusCheckDeadlineMs = 0;
            if (g_lastCheckCommandCellIndex == i) g_lastCheckCommandCellIndex = -1;
            Serial.printf("[CHECK] timeout -> key=%s\n", g_cellStates[i].key);
            forceCellComAndNotify(i, "status_check_timeout");
            continue;
        }
        if (g_cellStates[i].awaitingStatusCheck) statusCheckPending = true;
    }

    if (g_wisunRuntimeReady && isWiSUNConnected && g_keyIpCount > 0) {
        if (!g_autoCheckInProgress && (long)(now - g_nextCheckCycleMs) >= 0) {
            g_autoCheckInProgress = true;
            g_autoCheckIndex = 0;
            g_lastCheckSentMs = 0;
        }

        if (g_autoCheckInProgress && !statusCheckPending && (now - g_lastCheckSentMs >= 140)) {
            const unsigned long checkIntervalMs = g_checkIntervalSec * 1000UL;
            while (g_autoCheckIndex < g_keyIpCount) {
                const KeyIpInfo& keyInfo = g_keyIps[g_autoCheckIndex];
                bool recentlySeen = keyInfo.lastSeenMs != 0 &&
                                    (now - keyInfo.lastSeenMs) < checkIntervalMs;
                if (keyInfo.everSeen && !recentlySeen) break;
                g_autoCheckIndex++;
            }
            if (g_autoCheckIndex < g_keyIpCount) {
                sendCheckCommandForKey(g_keyIps[g_autoCheckIndex].key, false);
                g_lastCheckSentMs = now;
                g_autoCheckIndex++;
            } else {
                g_autoCheckInProgress = false;
                g_nextCheckCycleMs = now + (g_checkIntervalSec * 1000UL);
                Serial.printf("[CHECK] cycle complete; next cycle in %lus\n", g_checkIntervalSec);
            }
        }
    }

    // '/' returns to the menu. Inside the menu, 'q' starts a Wi-Fi scan.
    if (inputAvailable()) {
        char menuKey = tolower(peekInput());
        if (menuKey == '/') {
            readInput();
            setLEDColor("yellow");
            runMenu();
        }
    }

    // LED: blink when Wi-SUN not connected, steady green when OK
    if (!isWiSUNSettingUp && !wiSunInitialized) {
        if (now - lastBlinkTime > blinkMillisInterval) {
            lastBlinkTime = now; blinkState = !blinkState;
            setLEDColor(blinkState ? "red" : "yellow");
        }
    } else if (isWiSUNConnected) {
        setLEDColor("green");
    }

    // Wi-Fi events are reflected immediately; other displayed state is checked once per second.
    static unsigned long lastOledStatusCheckMs = 0;
    if (g_wifiOledUpdatePending || now - lastOledStatusCheckMs >= 1000) {
        g_wifiOledUpdatePending = false;
        lastOledStatusCheckMs = now;
        updateOledStatus();
    }

    delay(100);
}
