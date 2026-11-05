// M5StickS3 heading sensor for the rover.
//
// Reads the BMI270 gyro/accelerometer, integrates heading (yaw about the gravity vector, so the
// stick can be mounted at any angle as long as it stays put), and streams it over UDP to whoever
// subscribed. Protocol on UDP port 8773:
//   Mac -> stick  "sub"   stream to me for the next 3 s (resend every second)
//                 "zero"  set the current heading to 0
//   stick -> Mac  {"seq":..,"yaw":deg,"rate":deg/s,"still":0|1,"cal":0|1,"bumps":n,"bat":%,"up":ms}
//                 50 per second. yaw is unwrapped (keeps counting past 360), + = turned left.
// The gyro bias is re-learnt whenever the stick is still, so heading drift stays small.
// Button A (the big front button) also zeroes the heading. Holding button B (the side button) for
// 2 s turns the stick off, and so does 10 minutes without the Mac asking for data; the power
// button turns it back on. The screen dims while nobody is subscribed.
#include <M5Unified.h>
#include <cmath>
#include <cstring>
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "secrets.h"

static constexpr int PORT = 8773;
static constexpr int SEND_MS = 20;
static constexpr int64_t SUB_US = 3000000;
static constexpr float STILL_DPS = 1.5f;      // gyro below this (after bias) ...
static constexpr float STILL_G = 0.04f;       // ... and |accel| within this of 1 g ...
static constexpr int64_t STILL_US = 400000;   // ... for this long = standing still
static constexpr float BUMP_G = 0.6f;         // jolt that counts as a bump
static constexpr uint32_t OFF_HOLD_MS = 2000;  // hold button B this long to turn off
static constexpr int64_t IDLE_OFF_US = 600000000;  // off after 10 min with no "sub" from the Mac

struct State {
  float yaw = 0, rate = 0;
  bool still = false, cal = false, zero = false;
  uint32_t bumps = 0;
  int bat = -1;
};
static State st;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool wifi_up = false;
static volatile int64_t last_sub = 0;          // when the Mac last asked for data
static volatile bool going_off = false;
static char ip_text[20] = "no wifi";

static float dot(const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// Everything on the I2C bus (IMU, power chip, buttons) runs in this one task.
static void imu_task(void*) {
  float bias[3] = {0, 0, 0}, grav[3] = {0, 0, 1}, sum[3] = {0, 0, 0};
  int sum_n = 0;
  bool cal = false, still = false;
  float yaw = 0;
  uint32_t bumps = 0;
  int64_t last = esp_timer_get_time(), calm_since = last, last_bump = 0, last_bat = 0;
  int bat = M5.Power.getBatteryLevel();
  bool on_usb = true;                                   // until the first reading says otherwise
  int64_t last_vbus = 0;
  for (;;) {
    M5.update();
    if (M5.BtnA.wasPressed()) yaw = 0;
    // Off only on battery: on USB the power chip can't cut power, and a failed power-off leaves the
    // stick stuck until it's unplugged.
    if (esp_timer_get_time() - last_vbus > 1000000) {
      on_usb = M5.Power.getVBUSVoltage() > 4000;
      last_vbus = esp_timer_get_time();
    }
    if (!on_usb && (M5.BtnB.pressedFor(OFF_HOLD_MS) || esp_timer_get_time() - last_sub > IDLE_OFF_US)) {
      going_off = true;                                 // the screen says so, then power off
      vTaskDelay(pdMS_TO_TICKS(1200));
      M5.Power.powerOff();
    }
    if (on_usb) last_sub = esp_timer_get_time();        // the idle timer starts when unplugged
    if (!M5.Imu.update()) {
      vTaskDelay(1);
      continue;
    }
    auto d = M5.Imu.getImuData();
    int64_t now = esp_timer_get_time();
    float dt = (now - last) / 1e6f;
    last = now;
    if (dt > 0.1f) dt = 0.1f;
    float a[3] = {d.accel.x, d.accel.y, d.accel.z};
    float g[3] = {d.gyro.x - bias[0], d.gyro.y - bias[1], d.gyro.z - bias[2]};
    float amag = sqrtf(dot(a, a));
    float k = dt / (0.5f + dt);                        // ~0.5 s low-pass for gravity
    for (int i = 0; i < 3; i++) grav[i] += k * (a[i] - grav[i]);
    float gmag = sqrtf(dot(grav, grav));
    float jolt[3] = {a[0] - grav[0], a[1] - grav[1], a[2] - grav[2]};
    if (sqrtf(dot(jolt, jolt)) > BUMP_G && now - last_bump > 300000) {
      bumps++;
      last_bump = now;
    }
    bool calm = sqrtf(dot(g, g)) < (cal ? STILL_DPS : 6.0f) && fabsf(amag - 1.0f) < STILL_G;
    if (!calm) calm_since = now;
    still = now - calm_since > STILL_US;
    if (still) {                                        // learn the bias while standing still
      float raw[3] = {d.gyro.x, d.gyro.y, d.gyro.z};
      if (!cal) {
        for (int i = 0; i < 3; i++) sum[i] += raw[i];
        if (++sum_n >= 200) {
          for (int i = 0; i < 3; i++) bias[i] = sum[i] / sum_n;
          cal = true;
        }
      } else {
        for (int i = 0; i < 3; i++) bias[i] += 0.005f * (raw[i] - bias[i]);
      }
    } else if (!cal) {
      sum[0] = sum[1] = sum[2] = 0;
      sum_n = 0;
    }
    // Yaw rate = rotation about "up" (the accelerometer reads +1 g pointing up).
    float rate = (cal && gmag > 0.5f) ? dot(g, grav) / gmag : 0.0f;
    if (still) rate = 0;
    yaw += rate * dt;
    if (now - last_bat > 5000000) {
      bat = M5.Power.getBatteryLevel();
      last_bat = now;
    }
    portENTER_CRITICAL(&mux);
    if (st.zero) {
      yaw = 0;
      st.zero = false;
    }
    st.yaw = yaw;
    st.rate = rate;
    st.still = still;
    st.cal = cal;
    st.bumps = bumps;
    st.bat = bat;
    portEXIT_CRITICAL(&mux);
    vTaskDelay(1);
  }
}

static State snapshot() {
  portENTER_CRITICAL(&mux);
  State s = st;
  portEXIT_CRITICAL(&mux);
  return s;
}

static void on_wifi(void*, esp_event_base_t base, int32_t id, void* data) {
  if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) {
    wifi_up = false;
    esp_wifi_connect();
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    auto* e = (ip_event_got_ip_t*)data;
    snprintf(ip_text, sizeof ip_text, IPSTR, IP2STR(&e->ip_info.ip));
    wifi_up = true;
  }
}

static void wifi_start() {
  if (nvs_flash_init() != ESP_OK) {
    nvs_flash_erase();
    nvs_flash_init();
  }
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_t* netif = esp_netif_create_default_wifi_sta();
  esp_netif_set_hostname(netif, "rover-imu");
  wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&init);
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, nullptr);
  esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi, nullptr);
  wifi_config_t cfg = {};
  strncpy((char*)cfg.sta.ssid, WIFI_SSID, sizeof cfg.sta.ssid);
  strncpy((char*)cfg.sta.password, WIFI_PASS, sizeof cfg.sta.password);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &cfg);
  esp_wifi_start();
  esp_wifi_set_ps(WIFI_PS_NONE);                       // lowest latency
  mdns_init();
  mdns_hostname_set("rover-imu");
}

static void net_task(void*) {
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  sockaddr_in local = {};
  local.sin_family = AF_INET;
  local.sin_port = htons(PORT);
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  bind(sock, (sockaddr*)&local, sizeof local);
  timeval tv = {0, SEND_MS * 1000};
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
  sockaddr_in sub = {};
  int64_t sub_until = 0, next_send = 0;
  uint32_t seq = 0;
  char buf[160];
  for (;;) {
    sockaddr_in from = {};
    socklen_t flen = sizeof from;
    int n = recvfrom(sock, buf, sizeof buf - 1, 0, (sockaddr*)&from, &flen);
    int64_t now = esp_timer_get_time();
    if (n > 0) {
      buf[n] = 0;
      if (!strncmp(buf, "sub", 3)) {
        sub = from;
        sub_until = now + SUB_US;
        last_sub = now;
      } else if (!strncmp(buf, "zero", 4)) {
        portENTER_CRITICAL(&mux);
        st.zero = true;
        portEXIT_CRITICAL(&mux);
      }
    }
    if (now >= next_send && now < sub_until) {
      next_send = now + SEND_MS * 1000;
      State s = snapshot();
      int len = snprintf(buf, sizeof buf,
                         "{\"seq\":%lu,\"yaw\":%.2f,\"rate\":%.2f,\"still\":%d,\"cal\":%d,\"bumps\":%lu,"
                         "\"bat\":%d,\"up\":%lld}",
                         (unsigned long)seq++, s.yaw, s.rate, s.still, s.cal, (unsigned long)s.bumps, s.bat,
                         (long long)(now / 1000));
      sendto(sock, buf, len, 0, (sockaddr*)&sub, sizeof sub);
    }
  }
}

static M5Canvas lcd(&M5.Display);                    // drawn off-screen, pushed whole: no flicker

static void draw(const State& s) {
  int w = lcd.width(), cx = w / 2, cy = 120, r = 52;
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextDatum(top_center);
  lcd.setTextColor(TFT_WHITE);
  lcd.setFont(&fonts::Font2);
  lcd.drawString("ROVER HEADING", cx, 6);
  lcd.setTextColor(wifi_up ? TFT_GREEN : TFT_ORANGE);
  lcd.drawString(ip_text, cx, 26);
  lcd.drawCircle(cx, cy, r, TFT_DARKGREY);
  float h = s.yaw * (float)M_PI / 180.0f;               // + = left, so draw counter-clockwise
  int tx = cx - (int)(r * sinf(h)), ty = cy - (int)(r * cosf(h));
  lcd.drawWideLine(cx, cy, tx, ty, 4, s.cal ? TFT_CYAN : TFT_DARKGREY);
  lcd.fillCircle(tx, ty, 5, s.cal ? TFT_CYAN : TFT_DARKGREY);
  float wrapped = fmodf(s.yaw + 540.0f, 360.0f) - 180.0f;
  lcd.setTextColor(TFT_WHITE);
  lcd.setFont(&fonts::Font4);
  char t[16];
  snprintf(t, sizeof t, "%+.0f", wrapped);
  lcd.drawString(t, cx, cy + r + 10);
  lcd.setFont(&fonts::Font2);
  lcd.setTextColor(s.cal ? (s.still ? TFT_GREEN : TFT_YELLOW) : TFT_ORANGE);
  lcd.drawString(!s.cal ? "keep still..." : s.still ? "still" : "moving", cx, cy + r + 40);
  lcd.setTextColor(TFT_LIGHTGREY);
  snprintf(t, sizeof t, "bat %d%%", s.bat);
  lcd.drawString(t, cx, lcd.height() - 20);
  lcd.pushSprite(0, 0);
}

extern "C" void app_main() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(0);
  M5.Display.setBrightness(80);
  lcd.setColorDepth(16);
  lcd.createSprite(M5.Display.width(), M5.Display.height());
  wifi_start();
  xTaskCreatePinnedToCore(imu_task, "imu", 6144, nullptr, 10, nullptr, 1);
  xTaskCreatePinnedToCore(net_task, "net", 4096, nullptr, 5, nullptr, 0);
  char shown[64] = "";
  int brightness = 80;
  for (;;) {
    if (going_off) {
      lcd.fillScreen(TFT_BLACK);
      lcd.setTextDatum(middle_center);
      lcd.setTextColor(TFT_WHITE);
      lcd.setFont(&fonts::Font4);
      lcd.drawString("Off", lcd.width() / 2, lcd.height() / 2);
      lcd.pushSprite(0, 0);
      M5.Display.setBrightness(80);
      vTaskDelay(portMAX_DELAY);
    }
    int want = esp_timer_get_time() - last_sub < SUB_US ? 80 : 15;   // dim while unused
    if (want != brightness) M5.Display.setBrightness(brightness = want);
    State s = snapshot();
    char now[64];                                       // redraw only when what's shown changes
    snprintf(now, sizeof now, "%.0f %d %d %d %d %s", s.yaw, s.cal, s.still, s.bat, (int)wifi_up, ip_text);
    if (strcmp(now, shown)) {
      strcpy(shown, now);
      draw(s);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
