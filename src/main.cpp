/**
 * ============================================================
 *  SMART PARKING SYSTEM - AUTO GATE WITH RFID & SLOT MONITOR
 *  [ VERSI IoT + FUZZY LOGIC — ESP32 DevKit C v4 ]
 * ============================================================
 *  Author  : Mohd. Ikhsan Sadillah
 *  Platform: ESP32 DevKit C v4
 *
 *  FITUR IoT:
 *  ──────────
 *  1. MQTT (HiveMQ Cloud) — publish status slot & gate event
 *  2. Firebase Realtime DB — simpan data slot, summary, log RFID
 *  3. Telegram Bot — notifikasi otomatis ke chat pribadi
 *  4. Website Monitoring — baca data real-time dari Firebase
 *
 *  FUZZY LOGIC:
 *  ────────────
 *  1. FUZZY DETEKSI SLOT
 *     Input  : jarak (cm) dari HC-SR04
 *     Output : status KOSONG / RAGU / TERISI
 *
 *  2. FUZZY PRIORITAS SLOT
 *     Input  : derajat keanggotaan "kosong" setiap slot
 *     Output : skor prioritas 0.0–1.0 per slot
 *
 *  LIBRARY YANG DIPERLUKAN:
 *  ────────────────────────
 *  1. miguelbalboa/MFRC522              → RFID
 *  2. madhephaestus/ESP32Servo          → Servo SG90
 *  3. knolleary/PubSubClient            → MQTT
 *  4. mobizt/Firebase Arduino Client    → Firebase RTDB
 *  5. bblanchon/ArduinoJson             → JSON handling
 *
 *  KOMPONEN & PIN:
 *  ───────────────
 *  RFID MFRC522:
 *    SDA→5  SCK→18  MOSI→23  MISO→19  RST→4
 *
 *  SERVO SG90:     PWM→13  (5V eksternal + GND bersama)
 *  BUZZER AKTIF:   +→2     -→GND
 *
 *  HC-SR04 (Slot 2 & Slot 4):
 *    Slot 2: TRIG→14  ECHO→35
 *    Slot 4: TRIG→16  ECHO→17
 *
 *  LED (GPIO ESP32, seri R 220Ω ke GND):
 *    GPIO 25 → LED Hijau Slot 2
 *    GPIO 33 → LED Merah Slot 2
 *    GPIO 21 → LED Hijau Slot 4
 *    GPIO 22 → LED Merah Slot 4
 * ============================================================
 */

#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Firebase_ESP_Client.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>

// Firebase helper addons
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// ──────────────────────────────────────────────
//  PIN DEFINITIONS
// ──────────────────────────────────────────────
#define RFID_SS_PIN 5
#define RFID_RST_PIN 4

#define SERVO_PIN 13
#define BUZZER_PIN 2

// HC-SR04 — hanya Slot 2 & Slot 4
#define TRIG_SLOT2 14
#define ECHO_SLOT2 26

#define TRIG_SLOT4 16
#define ECHO_SLOT4 17

// LED — langsung ke GPIO ESP32
#define LED_GREEN_SLOT2 25
#define LED_RED_SLOT2 33
#define LED_GREEN_SLOT4 21
#define LED_RED_SLOT4 22

// ──────────────────────────────────────────────
//  KONFIGURASI KONEKSI — GANTI DENGAN DATA ANDA
// ──────────────────────────────────────────────

// WiFi
#define WIFI_SSID "AIRFY"
#define WIFI_PASSWORD "rumahbiru"

// HiveMQ Public MQTT
#define MQTT_BROKER "broker.hivemq.com"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "esp32-smartparking-2026"

// Firebase Realtime Database
#define FIREBASE_API_KEY "AIzaSyB1-nAnEnvGfs0WY5FUKGvvCwJJg5qEE_U"
#define FIREBASE_DB_URL "https://smart-parking-24a54-default-rtdb.asia-southeast1.firebasedatabase.app/"

// Telegram Bot
#define TELEGRAM_TOKEN "8769885445:AAEpcxxk8GXgyLjIRnqJ9mZVlFI5X7ke4AM"
#define TELEGRAM_CHAT_ID "970487432"

// MQTT Topics
#define TOPIC_SLOT_STATUS "parkir/slot/status"
#define TOPIC_GATE_EVENT "parkir/gate/event"
#define TOPIC_SYSTEM "parkir/system/online"

// ──────────────────────────────────────────────
//  SLOT CONFIG
// ──────────────────────────────────────────────
#define TOTAL_SLOTS 2

const int SLOT_LABEL[] = {2, 4}; // Index 0→Slot 2, Index 1→Slot 4

// LED pin arrays — index 0 = Slot 2, index 1 = Slot 4
const int LED_GREEN_PINS[] = {LED_GREEN_SLOT2, LED_GREEN_SLOT4};
const int LED_RED_PINS[] = {LED_RED_SLOT2, LED_RED_SLOT4};

// HC-SR04 pin arrays
const int TRIG_PINS[] = {TRIG_SLOT2, TRIG_SLOT4};
const int ECHO_PINS[] = {ECHO_SLOT2, ECHO_SLOT4};

// ──────────────────────────────────────────────
//  CONSTANTS — UMUM
// ──────────────────────────────────────────────
#define SERVO_OPEN_ANGLE 90
#define SERVO_CLOSE_ANGLE 0
#define GATE_OPEN_DURATION 3000 // ms
#define SENSOR_SAMPLES 3

// ──────────────────────────────────────────────
//  CONSTANTS — FUZZY LOGIC
// ──────────────────────────────────────────────
/**
 * MEMBERSHIP FUNCTION — DETEKSI SLOT (input: jarak cm)
 *
 *  TERISI  ████▓▒░─────────────────────
 *  RAGU    ────░▒▓████▓▒░──────────────
 *  KOSONG  ───────────░▒▓████▓▒░───────
 *
 *  0       5   10   15   20   25   30+ (cm)
 */
#define FZ_TERISI_PEAK 5.0f
#define FZ_TERISI_ZERO 15.0f

#define FZ_RAGU_LO 8.0f
#define FZ_RAGU_PEAK_LO 12.0f
#define FZ_RAGU_PEAK_HI 18.0f
#define FZ_RAGU_HI 22.0f

#define FZ_KOSONG_PEAK 20.0f
#define FZ_KOSONG_ZERO 12.0f

#define FZ_DECIDE_OCCUPIED 0.5f
#define FZ_DECIDE_EMPTY 0.5f

// ──────────────────────────────────────────────
//  CONSTANTS — NETWORK TIMING
// ──────────────────────────────────────────────
#define MQTT_RECONNECT_INTERVAL 5000   // ms
#define TELEGRAM_COOLDOWN 5000         // ms

// ──────────────────────────────────────────────
//  RFID SEKARANG MENGGUNAKAN FIREBASE
// ──────────────────────────────────────────────

// ──────────────────────────────────────────────
//  OBJECT INSTANCES
// ──────────────────────────────────────────────
MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);
Servo gateServo;

// Network objects
WiFiClient espClient;
PubSubClient mqtt(espClient);
FirebaseData fbdo;
FirebaseAuth fbAuth;
FirebaseConfig fbConfig;

// ──────────────────────────────────────────────
//  STATE VARIABLES
// ──────────────────────────────────────────────
bool slotOccupied[TOTAL_SLOTS] = {false, false};
float slotMuTerisi[TOTAL_SLOTS] = {0.0f, 0.0f};
float slotMuRagu[TOTAL_SLOTS] = {0.0f, 0.0f};
float slotMuKosong[TOTAL_SLOTS] = {0.0f, 0.0f};
float slotPriority[TOTAL_SLOTS] = {0.0f, 0.0f};
long slotDistance[TOTAL_SLOTS] = {0, 0};
int recommendedSlot = -1;

String currentCardName = "";
String currentCardNIM = "";

bool gateOpen = false;
unsigned long gateOpenTime = 0;

// Network state
bool firebaseReady = false;
unsigned long lastMqttReconnect = 0;
unsigned long lastTelegramSend = 0;

// ──────────────────────────────────────────────
//  FUNCTION PROTOTYPES
// ──────────────────────────────────────────────
// Hardware
long measureDistance(int trigPin, int echoPin);
long measureDistanceAvg(int trigPin, int echoPin);
void updateSlotLED(int slot, bool occupied);
void updateAllLEDs();
int countAvailableSlots();
void openGate();
void closeGate();
bool isCardAllowed(String uid);
void beepSuccess();
void beepDenied();
void printParkingStatus();
String getUID();

// Fuzzy
float fuzzyMuTerisi(float dist);
float fuzzyMuRagu(float dist);
float fuzzyMuKosong(float dist);
bool fuzzifySlot(int slot, float dist);
void fuzzifyAllSlots();
int fuzzyBestSlot();
void printFuzzyDebug();

// Network
void connectWiFi();
void connectMQTT();
void setupFirebase();
unsigned long getEpochTime();
void mqttPublishSlotStatus(int slot);
void mqttPublishGateEvent(String uid, bool allowed, bool gateOpened);
void firebaseUpdateSlot(int slot);
void firebaseUpdateSummary();
void firebasePushLog(String uid, bool allowed, bool gateOpened);
void sendTelegram(String message);
void telegramNotifyStatusChange();
void telegramNotifyRFID(String uid, bool allowed, int available);

// ══════════════════════════════════════════════
//  FUZZY MEMBERSHIP FUNCTIONS
// ══════════════════════════════════════════════

/**
 * μ_TERISI(dist) — Fungsi keanggotaan "Slot Terisi"
 * Bentuk: trapezoid kiri (turun dari 1 ke 0)
 */
float fuzzyMuTerisi(float dist)
{
  if (dist <= 0)
    return 1.0f; // sensor error → anggap terisi (aman)
  if (dist <= FZ_TERISI_PEAK)
    return 1.0f;
  if (dist >= FZ_TERISI_ZERO)
    return 0.0f;
  return (FZ_TERISI_ZERO - dist) / (FZ_TERISI_ZERO - FZ_TERISI_PEAK);
}

/**
 * μ_RAGU(dist) — Fungsi keanggotaan "Zona Ragu"
 * Bentuk: trapezoid tengah (naik, plateau, turun)
 */
float fuzzyMuRagu(float dist)
{
  if (dist <= 0)
    return 0.0f;
  if (dist <= FZ_RAGU_LO)
    return 0.0f;
  if (dist <= FZ_RAGU_PEAK_LO)
    return (dist - FZ_RAGU_LO) / (FZ_RAGU_PEAK_LO - FZ_RAGU_LO);
  if (dist <= FZ_RAGU_PEAK_HI)
    return 1.0f;
  if (dist <= FZ_RAGU_HI)
    return (FZ_RAGU_HI - dist) / (FZ_RAGU_HI - FZ_RAGU_PEAK_HI);
  return 0.0f;
}

/**
 * μ_KOSONG(dist) — Fungsi keanggotaan "Slot Kosong"
 * Bentuk: trapezoid kanan (naik dari 0 ke 1)
 */
float fuzzyMuKosong(float dist)
{
  if (dist <= 0)
    return 0.0f; // sensor error
  if (dist <= FZ_KOSONG_ZERO)
    return 0.0f;
  if (dist >= FZ_KOSONG_PEAK)
    return 1.0f;
  return (dist - FZ_KOSONG_ZERO) / (FZ_KOSONG_PEAK - FZ_KOSONG_ZERO);
}

// ══════════════════════════════════════════════
//  FUZZY INFERENCE — SLOT DETECTION
// ══════════════════════════════════════════════

/**
 * Fuzzifikasi satu slot.
 * ATURAN DEFUZZIFIKASI (MAX-MIN):
 *   IF μ_TERISI > FZ_DECIDE_OCCUPIED → TERISI
 *   ELSE IF μ_KOSONG > FZ_DECIDE_EMPTY → KOSONG
 *   ELSE → RAGU → dianggap TERISI (untuk keamanan)
 */
bool fuzzifySlot(int slot, float dist)
{
  float muT = fuzzyMuTerisi(dist);
  float muR = fuzzyMuRagu(dist);
  float muK = fuzzyMuKosong(dist);

  slotMuTerisi[slot] = muT;
  slotMuRagu[slot] = muR;
  slotMuKosong[slot] = muK;

  if (muT > FZ_DECIDE_OCCUPIED)
    return true;
  if (muK > FZ_DECIDE_EMPTY)
    return false;
  return true; // RAGU → TERISI (konservatif)
}

/**
 * Fuzzifikasi semua slot + update LED + publish jika ada perubahan.
 */
void fuzzifyAllSlots()
{
  bool changed = false;

  for (int i = 0; i < TOTAL_SLOTS; i++)
  {
    long dist = measureDistanceAvg(TRIG_PINS[i], ECHO_PINS[i]);
    slotDistance[i] = dist; // simpan untuk network
    bool nowOccupied = fuzzifySlot(i, (float)dist);

    if (nowOccupied != slotOccupied[i])
    {
      slotOccupied[i] = nowOccupied;
      updateSlotLED(i, nowOccupied);
      changed = true;

      Serial.print("[FUZZY] Slot ");
      Serial.print(SLOT_LABEL[i]);
      Serial.print(" → jarak=");
      Serial.print(dist);
      Serial.print("cm  μT=");
      Serial.print(slotMuTerisi[i], 2);
      Serial.print("  μR=");
      Serial.print(slotMuRagu[i], 2);
      Serial.print("  μK=");
      Serial.print(slotMuKosong[i], 2);
      Serial.print("  →  ");
      Serial.println(nowOccupied ? "TERISI" : "KOSONG");

      // ── Publish perubahan slot ke MQTT & Firebase ──
      mqttPublishSlotStatus(i);
      firebaseUpdateSlot(i);
      
      // ── Notifikasi Telegram Spesifik Perubahan Slot ──
      if (!nowOccupied) {
        String msg = "🚗 <b>KENDARAAN KELUAR</b>\n\n";
        msg += "Slot <b>" + String(SLOT_LABEL[i]) + "</b> sekarang kosong dan tersedia.\n";
        msg += "Sisa Slot: " + String(countAvailableSlots()) + " dari " + String(TOTAL_SLOTS);
        sendTelegram(msg);
      } else {
        String msg = "🅿️ <b>KENDARAAN MASUK</b>\n\n";
        msg += "Slot <b>" + String(SLOT_LABEL[i]) + "</b> telah terisi.\n";
        msg += "Sisa Slot: " + String(countAvailableSlots()) + " dari " + String(TOTAL_SLOTS);
        sendTelegram(msg);
      }
    }
  }

  if (changed)
  {
    printParkingStatus();

    // ── Update summary ──
    firebaseUpdateSummary();
    // telegramNotifyStatusChange(); // Dinonaktifkan karena sudah ada notifikasi spesifik di atas
  }
}

// ══════════════════════════════════════════════
//  FUZZY INFERENCE — SLOT PRIORITY
// ══════════════════════════════════════════════

/**
 * Hitung skor prioritas & temukan slot terbaik.
 * RUMUS: priority[i] = (1.0 × μ_KOSONG[i]) - (0.5 × μ_RAGU[i])
 * Return: index slot terbaik (0-based), -1 jika parkir penuh.
 */
int fuzzyBestSlot()
{
  const float W_KOSONG = 1.0f;
  const float W_RAGU = 0.5f;

  float bestScore = -1.0f;
  int bestSlot = -1;

  for (int i = 0; i < TOTAL_SLOTS; i++)
  {
    if (slotOccupied[i])
    {
      slotPriority[i] = 0.0f;
      continue;
    }

    float score = (W_KOSONG * slotMuKosong[i]) - (W_RAGU * slotMuRagu[i]);
    if (score < 0.0f)
      score = 0.0f;
    if (score > 1.0f)
      score = 1.0f;

    slotPriority[i] = score;

    if (score > bestScore)
    {
      bestScore = score;
      bestSlot = i;
    }
  }

  return bestSlot;
}

/**
 * Cetak tabel debug fuzzy ke Serial Monitor.
 */
void printFuzzyDebug()
{
  Serial.println("\n┌──────────────────────────────────────────────────────┐");
  Serial.println("│            FUZZY LOGIC — DEBUG TABLE                 │");
  Serial.println("├────────┬──────────┬──────────┬──────────┬────────────┤");
  Serial.println("│  SLOT  │ μ_TERISI │  μ_RAGU  │ μ_KOSONG │  PRIORITAS │");
  Serial.println("├────────┼──────────┼──────────┼──────────┼────────────┤");

  for (int i = 0; i < TOTAL_SLOTS; i++)
  {
    bool isRec = (i == recommendedSlot);
    Serial.printf("│  %d %-3s │  %.4f  │  %.4f  │  %.4f  │   %.4f  %s│\n",
                  SLOT_LABEL[i],
                  isRec ? "◄" : "  ",
                  slotMuTerisi[i],
                  slotMuRagu[i],
                  slotMuKosong[i],
                  slotPriority[i],
                  isRec ? "★ " : "  ");
  }

  Serial.println("├────────┴──────────┴──────────┴──────────┴────────────┤");
  if (recommendedSlot >= 0)
  {
    Serial.printf("│  Rekomendasi: Slot %-2d (prioritas tertinggi)           │\n",
                  SLOT_LABEL[recommendedSlot]);
  }
  else
  {
    Serial.println("│  Tidak ada slot tersedia (parkir penuh)               │");
  }
  Serial.println("└──────────────────────────────────────────────────────┘\n");
}

// ══════════════════════════════════════════════
//  NETWORK — WiFi
// ══════════════════════════════════════════════

void connectWiFi()
{
  Serial.print("[WiFi] Menghubungkan ke ");
  Serial.print(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println();
    Serial.print("[WiFi] Terhubung! IP: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println();
    Serial.println("[WiFi] GAGAL terhubung! Sistem tetap berjalan offline.");
  }
}

// ══════════════════════════════════════════════
//  NETWORK — MQTT (HiveMQ Cloud)
// ══════════════════════════════════════════════

void connectMQTT()
{
  if (WiFi.status() != WL_CONNECTED)
    return;

  Serial.print("[MQTT] Menghubungkan ke HiveMQ... ");

  // Last Will Testament — otomatis publish "offline" jika ESP32 mati
  if (mqtt.connect(MQTT_CLIENT_ID, TOPIC_SYSTEM, 1, true, "{\"status\":\"offline\"}"))
  {
    Serial.println("Terhubung!");
    // Publish online status (retained)
    String onlineMsg = "{\"status\":\"online\",\"ip\":\"" + WiFi.localIP().toString() + "\"}";
    mqtt.publish(TOPIC_SYSTEM, onlineMsg.c_str(), true);
  }
  else
  {
    Serial.print("Gagal, rc=");
    Serial.println(mqtt.state());
  }
}

// ══════════════════════════════════════════════
//  NETWORK — Firebase Realtime Database
// ══════════════════════════════════════════════

void setupFirebase()
{
  fbConfig.api_key = FIREBASE_API_KEY;
  fbConfig.database_url = FIREBASE_DB_URL;

  // Anonymous authentication
  Serial.print("[Firebase] Anonymous sign-in... ");
  if (Firebase.signUp(&fbConfig, &fbAuth, "", ""))
  {
    Serial.println("Berhasil!");
    firebaseReady = true;
  }
  else
  {
    Serial.print("Gagal: ");
    Serial.println(fbConfig.signer.signupError.message.c_str());
  }

  fbConfig.token_status_callback = tokenStatusCallback;
  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(true);
}

// ══════════════════════════════════════════════
//  NETWORK — NTP Time
// ══════════════════════════════════════════════

unsigned long getEpochTime()
{
  time_t now;
  time(&now);
  return (unsigned long)now;
}

// ══════════════════════════════════════════════
//  MQTT — Publish Functions
// ══════════════════════════════════════════════

void mqttPublishSlotStatus(int slot)
{
  if (!mqtt.connected())
    return;

  JsonDocument doc;
  doc["slot"] = SLOT_LABEL[slot];
  doc["occupied"] = slotOccupied[slot];
  doc["distance"] = slotDistance[slot];
  doc["muTerisi"] = serialized(String(slotMuTerisi[slot], 2));
  doc["muRagu"] = serialized(String(slotMuRagu[slot], 2));
  doc["muKosong"] = serialized(String(slotMuKosong[slot], 2));
  doc["priority"] = serialized(String(slotPriority[slot], 2));
  doc["available"] = countAvailableSlots();
  doc["total"] = TOTAL_SLOTS;

  char buffer[300];
  serializeJson(doc, buffer);
  mqtt.publish(TOPIC_SLOT_STATUS, buffer);

  Serial.print("[MQTT] Slot ");
  Serial.print(SLOT_LABEL[slot]);
  Serial.println(" status published.");
}

void mqttPublishGateEvent(String uid, bool allowed, bool gateOpened)
{
  if (!mqtt.connected())
    return;

  JsonDocument doc;
  doc["uid"] = uid;
  doc["allowed"] = allowed;
  doc["gateOpened"] = gateOpened;
  doc["recommendedSlot"] = recommendedSlot >= 0 ? SLOT_LABEL[recommendedSlot] : -1;
  doc["timestamp"] = getEpochTime();

  char buffer[256];
  serializeJson(doc, buffer);
  mqtt.publish(TOPIC_GATE_EVENT, buffer);

  Serial.println("[MQTT] Gate event published.");
}

// ══════════════════════════════════════════════
//  Firebase — Update Functions
// ══════════════════════════════════════════════

void firebaseUpdateSlot(int slot)
{
  if (!Firebase.ready() || !firebaseReady)
    return;

  String path = "/slots/" + String(SLOT_LABEL[slot]);

  FirebaseJson json;
  json.set("occupied", slotOccupied[slot]);
  json.set("distance", (int)slotDistance[slot]);
  json.set("muTerisi", (double)slotMuTerisi[slot]);
  json.set("muRagu", (double)slotMuRagu[slot]);
  json.set("muKosong", (double)slotMuKosong[slot]);
  json.set("priority", (double)slotPriority[slot]);
  json.set("lastUpdate", (unsigned long)getEpochTime());

  if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json))
  {
    Serial.print("[Firebase] Slot ");
    Serial.print(SLOT_LABEL[slot]);
    Serial.println(" updated.");
  }
  else
  {
    Serial.print("[Firebase] Gagal update slot: ");
    Serial.println(fbdo.errorReason().c_str());
  }
}

void firebaseUpdateSummary()
{
  if (!Firebase.ready() || !firebaseReady)
    return;

  int available = countAvailableSlots();
  int occupied = TOTAL_SLOTS - available;

  FirebaseJson json;
  json.set("totalSlots", TOTAL_SLOTS);
  json.set("available", available);
  json.set("occupied", occupied);
  json.set("recommendedSlot", recommendedSlot >= 0 ? SLOT_LABEL[recommendedSlot] : -1);
  json.set("lastUpdate", (unsigned long)getEpochTime());

  if (Firebase.RTDB.setJSON(&fbdo, "/summary", &json))
  {
    Serial.println("[Firebase] Summary updated.");
  }
  else
  {
    Serial.print("[Firebase] Gagal update summary: ");
    Serial.println(fbdo.errorReason().c_str());
  }
}

void firebasePushLog(String uid, bool allowed, bool gateOpened)
{
  if (!Firebase.ready() || !firebaseReady)
    return;

  FirebaseJson json;
  json.set("uid", uid);
  json.set("allowed", allowed);
  json.set("gateOpened", gateOpened);
  json.set("recommendedSlot", recommendedSlot >= 0 ? SLOT_LABEL[recommendedSlot] : -1);
  json.set("timestamp", (unsigned long)getEpochTime());

  if (Firebase.RTDB.pushJSON(&fbdo, "/logs", &json))
  {
    Serial.println("[Firebase] Log pushed.");
  }
  else
  {
    Serial.print("[Firebase] Gagal push log: ");
    Serial.println(fbdo.errorReason().c_str());
  }
}

// ══════════════════════════════════════════════
//  Telegram — Notification Functions
// ══════════════════════════════════════════════

void sendTelegram(String message)
{
  if (WiFi.status() != WL_CONNECTED)
    return;

  // Cooldown agar tidak spam
  unsigned long now = millis();
  if (lastTelegramSend != 0 && (now - lastTelegramSend < TELEGRAM_COOLDOWN))
    return;
  lastTelegramSend = now;

  HTTPClient http;
  String url = "https://api.telegram.org/bot" + String(TELEGRAM_TOKEN) + "/sendMessage";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  // Gunakan ArduinoJson untuk escape karakter dengan benar
  JsonDocument doc;
  doc["chat_id"] = TELEGRAM_CHAT_ID;
  doc["text"] = message;
  doc["parse_mode"] = "HTML";

  String payload;
  serializeJson(doc, payload);

  int httpCode = http.POST(payload);
  if (httpCode > 0)
  {
    Serial.println("[Telegram] Notifikasi terkirim.");
  }
  else
  {
    Serial.print("[Telegram] Gagal: ");
    Serial.println(http.errorToString(httpCode));
  }
  http.end();
}

void telegramNotifyStatusChange()
{
  int available = countAvailableSlots();

  String msg = "🅿️ <b>Update Status Parkir</b>\n\n";

  for (int i = 0; i < TOTAL_SLOTS; i++)
  {
    msg += slotOccupied[i] ? "🔴" : "🟢";
    msg += " Slot " + String(SLOT_LABEL[i]) + ": ";
    msg += slotOccupied[i] ? "<b>TERISI</b>" : "TERSEDIA";
    msg += "\n";
  }

  msg += "\n📊 Tersedia: " + String(available) + "/" + String(TOTAL_SLOTS);

  if (available == 0)
  {
    msg += "\n\n⚠️ <b>PARKIR PENUH!</b>";
  }
  else if (recommendedSlot >= 0)
  {
    msg += "\n⭐ Rekomendasi: Slot " + String(SLOT_LABEL[recommendedSlot]);
  }

  sendTelegram(msg);
}

void telegramNotifyRFID(String uid, bool allowed, int available)
{
  String msg;

  if (allowed && available > 0)
  {
    msg = "✅ <b>AKSES DITERIMA</b>\n\n";
    msg += "🔑 UID: <code>" + uid + "</code>\n";
    if (currentCardName != "") msg += "👤 Nama: <b>" + currentCardName + "</b>\n";
    if (currentCardNIM != "") msg += "🆔 NIM: " + currentCardNIM + "\n";
    msg += "🚧 Gate: TERBUKA\n";
    if (recommendedSlot >= 0)
    {
      msg += "⭐ Rekomendasi: Slot " + String(SLOT_LABEL[recommendedSlot]) + "\n";
    }
    msg += "📊 Tersedia: " + String(available) + "/" + String(TOTAL_SLOTS);
  }
  else if (allowed && available == 0)
  {
    msg = "🚫 <b>Parkir Penuh</b>\n\n";
    msg += "🔑 UID: <code>" + uid + "</code>\n";
    if (currentCardName != "") msg += "👤 Nama: <b>" + currentCardName + "</b>\n";
    if (currentCardNIM != "") msg += "🆔 NIM: " + currentCardNIM + "\n";
    msg += "⚠️ Kartu valid tapi tidak ada slot tersedia!";
  }
  else
  {
    msg = "⛔ <b>Akses Ditolak</b>\n\n";
    msg += "🔑 UID: <code>" + uid + "</code>\n";
    msg += "⚠️ Kartu belum diaktifkan atau tidak terdaftar!";
  }

  sendTelegram(msg);
}

// ══════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════
void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("============================================================");
  Serial.println("   SMART PARKING SYSTEM + FUZZY + IoT - Initializing...     ");
  Serial.println("============================================================");

  // ── Buzzer ──
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(80);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("[OK] Buzzer siap.");

  // ── LED GPIO langsung ──
  for (int i = 0; i < TOTAL_SLOTS; i++)
  {
    pinMode(LED_GREEN_PINS[i], OUTPUT);
    pinMode(LED_RED_PINS[i], OUTPUT);
    digitalWrite(LED_GREEN_PINS[i], HIGH); // Awal: semua hijau (tersedia)
    digitalWrite(LED_RED_PINS[i], LOW);
  }
  Serial.println("[OK] LED GPIO siap (Slot 2: GPIO25/33 | Slot 4: GPIO21/22). Semua HIJAU.");

  // ── Ultrasonic ──
  for (int i = 0; i < TOTAL_SLOTS; i++)
  {
    pinMode(TRIG_PINS[i], OUTPUT);
    digitalWrite(TRIG_PINS[i], LOW);
    pinMode(ECHO_PINS[i], INPUT_PULLDOWN);
  }
  Serial.println("[OK] Sensor ultrasonik siap (Slot 2 & Slot 4).");

  // ── Servo ──
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  gateServo.setPeriodHertz(50);
  gateServo.attach(SERVO_PIN, 500, 2400);
  gateServo.write(SERVO_CLOSE_ANGLE);
  delay(500);
  Serial.println("[OK] Servo SG90 gate tertutup.");

  // ── RFID SPI ──
  SPI.begin();
  rfid.PCD_Init();
  delay(100);
  byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  if (version == 0x00 || version == 0xFF)
  {
    Serial.println("[ERROR] RFID MFRC522 tidak terdeteksi!");
  }
  else
  {
    Serial.print("[OK] RFID MFRC522 siap. Firmware: 0x");
    Serial.println(version, HEX);
  }

  // ── Fuzzy — bacaan awal (sebelum network, jadi tidak trigger notifikasi) ──
  Serial.println("[OK] Menginisialisasi fuzzy logic...");
  // Baca sensor tanpa network (network belum aktif, jadi publish functions return early)
  for (int i = 0; i < TOTAL_SLOTS; i++)
  {
    long dist = measureDistanceAvg(TRIG_PINS[i], ECHO_PINS[i]);
    slotDistance[i] = dist;
    slotOccupied[i] = fuzzifySlot(i, (float)dist);
    updateSlotLED(i, slotOccupied[i]);
  }
  recommendedSlot = fuzzyBestSlot();
  printFuzzyDebug();

  // ════════════════════════════════════════════
  //  NETWORK INITIALIZATION
  // ════════════════════════════════════════════
  Serial.println("\n──── NETWORK SETUP ────");

  // ── WiFi ──
  connectWiFi();

  if (WiFi.status() == WL_CONNECTED)
  {
    // ── NTP Time ──
    configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov"); // GMT+7 WIB
    Serial.print("[NTP] Sinkronisasi waktu");
    int ntpRetry = 0;
    while (time(nullptr) < 1700000000 && ntpRetry < 10)
    {
      delay(500);
      Serial.print(".");
      ntpRetry++;
    }
    Serial.println(" OK!");

    // ── MQTT ──
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    mqtt.setBufferSize(512);
    connectMQTT();

    // ── Firebase ──
    setupFirebase();

    // ── Kirim data awal ke Firebase ──
    for (int i = 0; i < TOTAL_SLOTS; i++)
    {
      firebaseUpdateSlot(i);
    }
    firebaseUpdateSummary();

    // ── Notifikasi startup ke Telegram ──
    int available = countAvailableSlots();
    String startMsg = "🟢 <b>Smart Parking System ONLINE</b>\n\n";
    startMsg += "📡 IP: " + WiFi.localIP().toString() + "\n";
    startMsg += "📊 Slot tersedia: " + String(available) + "/" + String(TOTAL_SLOTS) + "\n";

    for (int i = 0; i < TOTAL_SLOTS; i++)
    {
      startMsg += slotOccupied[i] ? "🔴" : "🟢";
      startMsg += " Slot " + String(SLOT_LABEL[i]) + ": ";
      startMsg += slotOccupied[i] ? "TERISI" : "TERSEDIA";
      startMsg += "\n";
    }

    sendTelegram(startMsg);
  }

  Serial.println("\n------------------------------------------------------------");
  Serial.println("  Sistem aktif. Tempelkan kartu RFID...                    ");
  Serial.println("------------------------------------------------------------\n");

  printParkingStatus();
}

// ══════════════════════════════════════════════
//  MAIN LOOP
// ══════════════════════════════════════════════
void loop()
{

  // ── 0. MQTT maintain connection ──
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!mqtt.connected())
    {
      unsigned long now = millis();
      if (now - lastMqttReconnect > MQTT_RECONNECT_INTERVAL)
      {
        lastMqttReconnect = now;
        connectMQTT();
      }
    }
    mqtt.loop();
  }

  // ── 1. Baca sensor + fuzzifikasi semua slot ──
  fuzzifyAllSlots();

  // ── 2. Update rekomendasi slot terbaik ──
  recommendedSlot = fuzzyBestSlot();

  // ── 3. Tutup gate otomatis ──
  if (gateOpen && (millis() - gateOpenTime >= GATE_OPEN_DURATION))
  {
    closeGate();
  }

  // ── 4. Baca RFID ──
  if (!rfid.PICC_IsNewCardPresent())
  {
    delay(50);
    return;
  }
  if (!rfid.PICC_ReadCardSerial())
  {
    delay(50);
    return;
  }

  String uid = getUID();
  Serial.print("\n[RFID] Kartu terdeteksi → UID: ");
  Serial.println(uid);

  if (isCardAllowed(uid))
  {
    int available = countAvailableSlots();
    if (available > 0)
    {
      Serial.println("[AKSES] Kartu DIIZINKAN — Gate membuka!");

      if (recommendedSlot >= 0)
      {
        Serial.print("[FUZZY] Rekomendasi slot terbaik → Slot ");
        Serial.print(SLOT_LABEL[recommendedSlot]);
        Serial.print("  (prioritas: ");
        Serial.print(slotPriority[recommendedSlot], 3);
        Serial.println(")");
      }

      printFuzzyDebug();
      beepSuccess();
      openGate();
      printParkingStatus();

      // ── Network: publish gate event ──
      mqttPublishGateEvent(uid, true, true);
      firebasePushLog(uid, true, true);
      telegramNotifyRFID(uid, true, available);
    }
    else
    {
      Serial.println("[AKSES] Kartu valid, tapi PARKIR PENUH!");
      beepDenied();

      // ── Network: publish event parkir penuh ──
      mqttPublishGateEvent(uid, true, false);
      firebasePushLog(uid, true, false);
      telegramNotifyRFID(uid, true, 0);
    }
  }
  else
  {
    Serial.println("[AKSES] Kartu TIDAK DIIZINKAN.");
    beepDenied();

    // ── Network: publish akses ditolak ──
    mqttPublishGateEvent(uid, false, false);
    firebasePushLog(uid, false, false);
    telegramNotifyRFID(uid, false, countAvailableSlots());
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  delay(300);
}

// ══════════════════════════════════════════════
//  FUNCTIONS — SENSOR & HARDWARE
// ══════════════════════════════════════════════

long measureDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0)
    return -1;
  return (long)(duration * 0.0343 / 2.0);
}

long measureDistanceAvg(int trigPin, int echoPin)
{
  long samples[SENSOR_SAMPLES];
  int valid = 0;

  for (int i = 0; i < SENSOR_SAMPLES; i++)
  {
    long d = measureDistance(trigPin, echoPin);
    if (d > 0)
      samples[valid++] = d;
    delay(10);
  }

  if (valid == 0)
    return -1;

  for (int i = 0; i < valid - 1; i++)
  {
    for (int j = i + 1; j < valid; j++)
    {
      if (samples[j] < samples[i])
      {
        long tmp = samples[i];
        samples[i] = samples[j];
        samples[j] = tmp;
      }
    }
  }
  return samples[valid / 2];
}

/**
 * Update LED satu slot.
 * TERISI  → Merah ON,  Hijau OFF
 * KOSONG  → Hijau ON,  Merah OFF
 */
void updateSlotLED(int slot, bool occupied)
{
  if (slot < 0 || slot >= TOTAL_SLOTS)
    return;
  if (occupied)
  {
    digitalWrite(LED_GREEN_PINS[slot], LOW);
    digitalWrite(LED_RED_PINS[slot], HIGH);
  }
  else
  {
    digitalWrite(LED_GREEN_PINS[slot], HIGH);
    digitalWrite(LED_RED_PINS[slot], LOW);
  }
}

void updateAllLEDs()
{
  for (int i = 0; i < TOTAL_SLOTS; i++)
    updateSlotLED(i, slotOccupied[i]);
}

int countAvailableSlots()
{
  int count = 0;
  for (int i = 0; i < TOTAL_SLOTS; i++)
  {
    if (!slotOccupied[i])
      count++;
  }
  return count;
}

void openGate()
{
  gateServo.write(SERVO_OPEN_ANGLE);
  gateOpen = true;
  gateOpenTime = millis();
  Serial.print("[GATE] Terbuka selama ");
  Serial.print(GATE_OPEN_DURATION / 1000);
  Serial.println(" detik.");
}

void closeGate()
{
  gateServo.write(SERVO_CLOSE_ANGLE);
  gateOpen = false;
  Serial.println("[GATE] Tertutup.");
}

bool isCardAllowed(String uid)
{
  uid.trim();
  
  if (!Firebase.ready() || !firebaseReady) {
    Serial.println("[Firebase] Offline, koneksi terputus. Gagal validasi RFID.");
    return false; 
  }

  // Hapus spasi pada UID (contoh: "A1 B2" menjadi "A1B2")
  String uidKey = uid;
  uidKey.replace(" ", "");

  String path = "/kartu_rfid/" + uidKey;

  Serial.print("[Firebase] Memeriksa kartu: ");
  Serial.println(uidKey);

  // Reset global variables
  currentCardName = "";
  currentCardNIM = "";

  // Coba ambil seluruh data JSON kartu
  if (Firebase.RTDB.getJSON(&fbdo, path)) {
    FirebaseJson json;
    FirebaseJsonData jsonData;
    json.setJsonData(fbdo.jsonString());
    
    // Update waktu_tap terakhir
    Firebase.RTDB.setDouble(&fbdo, path + "/waktu_tap", (double)getEpochTime());

    json.get(jsonData, "nama");
    if (jsonData.success) currentCardName = jsonData.stringValue;

    json.get(jsonData, "nim");
    if (jsonData.success) currentCardNIM = jsonData.stringValue;

    json.get(jsonData, "status");
    String status = jsonData.success ? jsonData.stringValue : "";

    if (status == "aktif" || status == "diizinkan") {
      return true;
    } else {
      Serial.print("[Firebase] Kartu ditolak. Status saat ini: ");
      Serial.println(status);
      return false;
    }
  } else {
    // Jika tidak ada di database, otomatis daftarkan sebagai "menunggu_input_manual"
    Serial.println("[Firebase] Kartu belum terdaftar! Mendaftarkan otomatis...");
    
    FirebaseJson newCard;
    newCard.set("nim", "");
    newCard.set("nama", "");
    newCard.set("status", "menunggu_input_manual");
    newCard.set("waktu_tap", (double)getEpochTime());
    
    if (Firebase.RTDB.setJSON(&fbdo, "/kartu_rfid/" + uidKey, &newCard)) {
      Serial.println("[Firebase] Kartu baru berhasil didaftarkan untuk review.");
    } else {
      Serial.print("[Firebase] Gagal mendaftarkan kartu: ");
      Serial.println(fbdo.errorReason());
    }
    
    return false; // Karena baru mendaftar, tetap tolak aksesnya
  }
}

void beepSuccess()
{
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(120);
    digitalWrite(BUZZER_PIN, LOW);
    delay(80);
  }
}

void beepDenied()
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(600);
  digitalWrite(BUZZER_PIN, LOW);
}

String getUID()
{
  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++)
  {
    if (rfid.uid.uidByte[i] < 0x10)
      uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
    if (i < rfid.uid.size - 1)
      uid += " ";
  }
  uid.toUpperCase();
  return uid;
}

void printParkingStatus()
{
  int available = countAvailableSlots();
  int occupied = TOTAL_SLOTS - available;

  Serial.println("\n┌──────────────────────────────────────────────────────┐");
  Serial.println("│             STATUS PARKIR SAAT INI                   │");
  Serial.println("├─────────┬───────────────────┬──────────┬─────────────┤");
  Serial.println("│  SLOT   │      STATUS       │  JARAK   │  FZ STATUS  │");
  Serial.println("├─────────┼───────────────────┼──────────┼─────────────┤");

  for (int i = 0; i < TOTAL_SLOTS; i++)
  {
    long dist = measureDistanceAvg(TRIG_PINS[i], ECHO_PINS[i]);

    const char *fzLabel;
    if (slotMuTerisi[i] >= slotMuKosong[i] && slotMuTerisi[i] >= slotMuRagu[i])
      fzLabel = "TERISI  ";
    else if (slotMuKosong[i] >= slotMuTerisi[i] && slotMuKosong[i] >= slotMuRagu[i])
      fzLabel = "KOSONG  ";
    else
      fzLabel = "RAGU    ";

    bool isRec = (i == recommendedSlot);
    Serial.printf("│  Slot %d │  %-17s│ %5ld cm  │ %-8s %s│\n",
                  SLOT_LABEL[i],
                  slotOccupied[i] ? "TERISI     " : "TERSEDIA   ",
                  dist > 0 ? dist : 0L,
                  fzLabel,
                  isRec ? "★" : " ");
  }

  Serial.println("├─────────┴───────────────────┴──────────┴─────────────┤");
  Serial.printf("│  Terisi : %-3d    Tersedia : %-3d                      │\n",
                occupied, available);

  if (available == 0)
  {
    Serial.println("│               PARKIR PENUH !                        │");
  }
  else if (recommendedSlot >= 0)
  {
    Serial.printf("│  ★ Slot Rekomendasi (Fuzzy): Slot %-2d                 │\n",
                  SLOT_LABEL[recommendedSlot]);
  }
  Serial.println("└──────────────────────────────────────────────────────┘\n");
}
