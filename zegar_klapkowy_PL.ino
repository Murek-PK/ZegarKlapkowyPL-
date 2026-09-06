#include <Arduino.h>
#include <WiFi.h>
#include "time.h"
#include "esp_sntp.h"
#include <ESP32Servo.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <ESPmDNS.h>
#include <WebServer.h>

// --- stepery ---

#define IN1m 14
#define IN2m 27
#define IN3m 26
#define IN4m 25

#define IN1g 33
#define IN2g 32
#define IN3g 4
#define IN4g 12

AccelStepper stepperH(AccelStepper::HALF4WIRE, IN1g, IN3g, IN2g, IN4g);
AccelStepper stepperM(AccelStepper::HALF4WIRE, IN1m, IN3m, IN2m, IN4m);

const float NORMAL_STEPPER_MAX_SPEED = 1000.0;
const float STEPPER_ACCELERATION = 5000.0;
const long STEPS_PER_FLAP = 512;
const int FLAP_COUNT = 12;
const unsigned long MOTION_GAP_MS = 500;
const char *MINUTE_FLAP_LABELS[FLAP_COUNT] = {
  "pusta", "pięć", "dziesięć", "kwadrans", "dwadzieścia", "dwadzieścia pięć",
  "w pół do", "dwadzieścia pięć", "dwadzieścia", "kwadrans", "dziesięć", "pięć"
};
const char *HOUR_FLAP_LABELS[FLAP_COUNT] = {
  "pierwsz", "drug", "trzeci", "czwart", "piąt", "szóst",
  "siódm", "ósm", "dziesiąt", "dziesiąt", "jedenst", "deunast"
};

// --- serwa ---

Servo serwoZA;
Servo serwoPO;
Servo serwo1_4min;
Servo serwoEnding;

const int PIN_serwoZA = 17;
const int PIN_serwoPO = 18;
const int PIN_serwo1_4min = 19;
const int PIN_serwoEnding = 23;

int posZA[2] = {0, 155};     // blank, ZA
int posPO[2] = {0, 155};     // blank, PO
int posEND[3] = {139, 77, 16};  // A, EJ, IEJ
int pos1_4MIN[5] = {0, 45, 85, 127, 170};  // 0, 1, 2, 3, 4

// --- czas ---

int curMin = 0;
int curHour = 0;
int curSecond = 0;
bool timeValid = false;
unsigned long lastTimeReadMs = 0;

const unsigned long TIME_REFRESH_INTERVAL_MS = 1000;

// --- wifi ---

const char *WIFI_SSID = "WPISZ_NAZWE_WIFI";
const char *WIFI_PASSWORD = "WPISZ_HASLO_WIFI";

const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const char *timeZone = "CET-1CEST,M3.5.0,M10.5.0/3";
const char *mdnsHostName = "zegarklapkowypl";

// --- web + stan ---

WebServer server(80);

const uint32_t EEPROM_MAGIC = 0x5A4B5031;  // ZKP1
const uint16_t EEPROM_VERSION = 1;
const int EEPROM_SIZE = 128;

struct PersistedState {
  uint32_t magic;
  uint16_t version;
  uint8_t calibrated;
  long hourStepperPosition;
  long minuteStepperPosition;
  uint8_t hourFlapIndex;
  uint8_t minuteFlapIndex;
};

PersistedState persistedState;

int runtimeHourFlapIndex = 0;
int runtimeMinuteFlapIndex = 0;
bool stateLoaded = false;
bool calibrationMode = true;
bool hourSavePending = false;
bool minuteSavePending = false;
bool persistDirty = false;
bool stepperWasMoving = false;
unsigned long lastMotionCompletedMs = 0;
long pendingHourStepperDelta = 0;
long pendingMinuteStepperDelta = 0;
int currentPosZA = 0;
int currentPosPO = 0;
int currentPosEND = 0;
int currentPos1_4MIN = 0;
int targetPosZA = 0;
int targetPosPO = 0;
int targetPosEND = 0;
int targetPos1_4MIN = 0;

void loadPersistedState();
void savePersistedState();
void restoreClockState();
void persistCurrentRuntimeState();
void setupWebServer();
void handleRoot();
void handleCalibrateManual();
void handleStepperMove();
void handleNotFound();
void redirectHome();
bool isValidStepperName(const String &stepperName);
void enterCalibrationMode();
bool ensureNoMotionInProgress();
void queueRelativeMove(AccelStepper &stepper, long deltaSteps);
void scheduleRelativeMove(AccelStepper &stepper, long deltaSteps);
void updateStepperPersistence();
void disableIdleStepperOutputs();
void updateMotionCompletionState();
void processPendingActions();
bool applyNextServoAction();
bool applyServoAction(int &currentPos, int targetPos, Servo &servo);
void updateClockFromTime();
void updateServoDisplay();
String buildHtmlPage();
int normalizeHourFlapIndex(int hour24);
int targetHourFlapIndexForTime(int hour24, int minute);
int targetMinuteFlapIndexForTime(int minute);
void assignCurrentMechanismToTime(int hour24, int minute);
void assignCurrentMechanismToFlaps(int hourFlapIndex, int minuteFlapIndex);

void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    timeValid = false;
    Serial.println("No time available (yet)");
    return;
  }

  timeValid = true;
  curHour = timeinfo.tm_hour;
  curMin = timeinfo.tm_min;
  curSecond = timeinfo.tm_sec;

  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void timeavailable(struct timeval *t) {
  (void)t;
  Serial.println("Got time adjustment from NTP!");
  printLocalTime();
}

void setup() {
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);
  loadPersistedState();

  Serial.printf("Connecting to %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  esp_sntp_servermode_dhcp(1);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin(mdnsHostName)) {
    MDNS.addService("http", "tcp", 80);
    Serial.print("mDNS address: http://");
    Serial.print(mdnsHostName);
    Serial.println(".local");
  } else {
    Serial.println("Could not start mDNS");
  }

  sntp_set_time_sync_notification_cb(timeavailable);
  configTzTime(timeZone, ntpServer1, ntpServer2);

  serwoZA.attach(PIN_serwoZA, 500, 2400);
  serwoPO.attach(PIN_serwoPO, 500, 2400);
  serwo1_4min.attach(PIN_serwo1_4min, 500, 2400);
  serwoEnding.attach(PIN_serwoEnding, 500, 2400);

  stepperH.setMaxSpeed(NORMAL_STEPPER_MAX_SPEED);
  stepperH.setAcceleration(STEPPER_ACCELERATION);
  stepperH.disableOutputs();

  stepperM.setMaxSpeed(NORMAL_STEPPER_MAX_SPEED);
  stepperM.setAcceleration(STEPPER_ACCELERATION);
  stepperM.disableOutputs();

  restoreClockState();
  setupWebServer();
  printLocalTime();
  updateServoDisplay();
  currentPosZA = targetPosZA;
  currentPosPO = targetPosPO;
  currentPosEND = targetPosEND;
  currentPos1_4MIN = targetPos1_4MIN;
  serwoZA.write(currentPosZA);
  serwoPO.write(currentPosPO);
  serwoEnding.write(currentPosEND);
  serwo1_4min.write(currentPos1_4MIN);
  lastMotionCompletedMs = millis();
}

void loop() {
  stepperH.run();
  stepperM.run();

  server.handleClient();
  updateStepperPersistence();
  updateMotionCompletionState();
  processPendingActions();
  disableIdleStepperOutputs();

  if (millis() - lastTimeReadMs >= TIME_REFRESH_INTERVAL_MS) {
    lastTimeReadMs = millis();
    printLocalTime();
    updateClockFromTime();
    updateServoDisplay();
  }
}

void loadPersistedState() {
  EEPROM.get(0, persistedState);

  if (persistedState.magic != EEPROM_MAGIC || persistedState.version != EEPROM_VERSION || persistedState.calibrated != 1) {
    persistedState.magic = EEPROM_MAGIC;
    persistedState.version = EEPROM_VERSION;
    persistedState.calibrated = 0;
    persistedState.hourStepperPosition = 0;
    persistedState.minuteStepperPosition = 0;
    persistedState.hourFlapIndex = 0;
    persistedState.minuteFlapIndex = 0;
    savePersistedState();
  }
}

void savePersistedState() {
  EEPROM.put(0, persistedState);
  EEPROM.commit();
}

void restoreClockState() {
  stepperH.setCurrentPosition(persistedState.hourStepperPosition);
  stepperH.moveTo(persistedState.hourStepperPosition);
  stepperM.setCurrentPosition(persistedState.minuteStepperPosition);
  stepperM.moveTo(persistedState.minuteStepperPosition);

  runtimeHourFlapIndex = persistedState.hourFlapIndex;
  runtimeMinuteFlapIndex = persistedState.minuteFlapIndex;
  stateLoaded = persistedState.calibrated == 1;
  calibrationMode = !stateLoaded;

  if (stateLoaded) {
    Serial.println("Restored calibrated clock state from EEPROM.");
  } else {
    Serial.println("No valid calibration found. Use the web page to align the clock.");
  }
}

void persistCurrentRuntimeState() {
  persistedState.magic = EEPROM_MAGIC;
  persistedState.version = EEPROM_VERSION;
  persistedState.calibrated = 1;
  persistedState.hourStepperPosition = stepperH.currentPosition();
  persistedState.minuteStepperPosition = stepperM.currentPosition();
  persistedState.hourFlapIndex = runtimeHourFlapIndex;
  persistedState.minuteFlapIndex = runtimeMinuteFlapIndex;
  savePersistedState();
  stateLoaded = true;
  calibrationMode = false;
  persistDirty = false;
}

void redirectHome() {
  server.sendHeader("Location", "/", true);
  server.send(303, "text/plain", "");
}

void handleRoot() {
  server.send(200, "text/html; charset=utf-8", buildHtmlPage());
}

void handleCalibrateManual() {
  if (!server.hasArg("hourFlap") || !server.hasArg("minuteFlap")) {
    server.send(400, "text/plain; charset=utf-8", "Missing flap selection.");
    return;
  }

  const int hourFlapIndex = server.arg("hourFlap").toInt();
  const int minuteFlapIndex = server.arg("minuteFlap").toInt();

  if (hourFlapIndex < 0 || hourFlapIndex >= FLAP_COUNT || minuteFlapIndex < 0 || minuteFlapIndex >= FLAP_COUNT) {
    server.send(400, "text/plain; charset=utf-8", "Invalid flap selection.");
    return;
  }

  if (!ensureNoMotionInProgress()) {
    return;
  }

  assignCurrentMechanismToFlaps(hourFlapIndex, minuteFlapIndex);
  redirectHome();
}

void handleStepperMove() {
  if (!server.hasArg("stepper") || !server.hasArg("steps")) {
    server.send(400, "text/plain; charset=utf-8", "Missing stepper or steps.");
    return;
  }

  if (!ensureNoMotionInProgress()) {
    return;
  }

  const String stepperName = server.arg("stepper");
  const long deltaSteps = server.arg("steps").toInt();

  if (!isValidStepperName(stepperName) || deltaSteps == 0) {
    server.send(400, "text/plain; charset=utf-8", "Invalid stepper or steps.");
    return;
  }

  enterCalibrationMode();

  if (stepperName == "hour") {
    queueRelativeMove(stepperH, deltaSteps);
  } else {
    queueRelativeMove(stepperM, deltaSteps);
  }

  redirectHome();
}

void handleNotFound() {
  server.send(404, "text/plain; charset=utf-8", "Not found.");
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/calibrate-manual", HTTP_POST, handleCalibrateManual);
  server.on("/stepper/move", HTTP_POST, handleStepperMove);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started on port 80.");
}

bool isValidStepperName(const String &stepperName) {
  return stepperName == "hour" || stepperName == "minute";
}

void enterCalibrationMode() {
  calibrationMode = true;
  stateLoaded = false;
}

bool ensureNoMotionInProgress() {
  if (stepperH.distanceToGo() != 0 || stepperM.distanceToGo() != 0 || pendingHourStepperDelta != 0 || pendingMinuteStepperDelta != 0) {
    server.send(409, "text/plain; charset=utf-8", "Motion is still in progress. Wait for it to finish.");
    return false;
  }
  return true;
}

void queueRelativeMove(AccelStepper &stepper, long deltaSteps) {
  stepper.enableOutputs();
  stepper.moveTo(stepper.targetPosition() + deltaSteps);
}

void scheduleRelativeMove(AccelStepper &stepper, long deltaSteps) {
  if (&stepper == &stepperH) {
    pendingHourStepperDelta += deltaSteps;
  } else if (&stepper == &stepperM) {
    pendingMinuteStepperDelta += deltaSteps;
  }
}

void updateStepperPersistence() {
  if (hourSavePending && pendingHourStepperDelta == 0 && stepperH.distanceToGo() == 0) {
    persistedState.hourStepperPosition = stepperH.currentPosition();
    persistedState.hourFlapIndex = runtimeHourFlapIndex;
    hourSavePending = false;
    persistDirty = true;
  }

  if (minuteSavePending && pendingMinuteStepperDelta == 0 && stepperM.distanceToGo() == 0) {
    persistedState.minuteStepperPosition = stepperM.currentPosition();
    persistedState.minuteFlapIndex = runtimeMinuteFlapIndex;
    minuteSavePending = false;
    persistDirty = true;
  }

  if (!hourSavePending && !minuteSavePending && stateLoaded && persistDirty) {
    persistedState.magic = EEPROM_MAGIC;
    persistedState.version = EEPROM_VERSION;
    persistedState.calibrated = 1;
    savePersistedState();
    persistDirty = false;
  }
}

void disableIdleStepperOutputs() {
  if (stepperH.distanceToGo() == 0) {
    stepperH.disableOutputs();
  }

  if (stepperM.distanceToGo() == 0) {
    stepperM.disableOutputs();
  }
}

void updateMotionCompletionState() {
  const bool stepperMoving = stepperH.distanceToGo() != 0 || stepperM.distanceToGo() != 0;
  if (stepperWasMoving && !stepperMoving) {
    lastMotionCompletedMs = millis();
  }
  stepperWasMoving = stepperMoving;
}

void processPendingActions() {
  if (stepperWasMoving) {
    return;
  }

  if (millis() - lastMotionCompletedMs < MOTION_GAP_MS) {
    return;
  }

  if (applyNextServoAction()) {
    lastMotionCompletedMs = millis();
    return;
  }

  if (pendingMinuteStepperDelta != 0) {
    const long delta = pendingMinuteStepperDelta;
    pendingMinuteStepperDelta = 0;
    queueRelativeMove(stepperM, delta);
    stepperWasMoving = true;
    return;
  }

  if (applyServoAction(currentPosPO, targetPosPO, serwoPO)) {
    lastMotionCompletedMs = millis();
    return;
  }

  if (pendingHourStepperDelta != 0) {
    const long delta = pendingHourStepperDelta;
    pendingHourStepperDelta = 0;
    queueRelativeMove(stepperH, delta);
    stepperWasMoving = true;
    return;
  }

  if (applyServoAction(currentPosEND, targetPosEND, serwoEnding)) {
    lastMotionCompletedMs = millis();
  }
}

bool applyNextServoAction() {
  if (applyServoAction(currentPos1_4MIN, targetPos1_4MIN, serwo1_4min)) {
    return true;
  }

  if (applyServoAction(currentPosZA, targetPosZA, serwoZA)) {
    return true;
  }

  return false;
}

bool applyServoAction(int &currentPos, int targetPos, Servo &servo) {
  if (currentPos == targetPos) {
    return false;
  }

  currentPos = targetPos;
  servo.write(currentPos);
  return true;
}

int normalizeHourFlapIndex(int hour24) {
  int hour12 = hour24 % 12;
  if (hour12 < 0) {
    hour12 += 12;
  }
  if (hour12 == 0) {
    hour12 = 12;
  }
  return hour12 - 1;
}

int targetHourFlapIndexForTime(int hour24, int minute) {
  int displayHour = hour24;
  if (minute >= 30) {
    displayHour = (hour24 + 1) % 24;
  }
  return normalizeHourFlapIndex(displayHour);
}

int targetMinuteFlapIndexForTime(int minute) {
  return minute / 5;
}

void assignCurrentMechanismToTime(int hour24, int minute) {
  runtimeHourFlapIndex = targetHourFlapIndexForTime(hour24, minute);
  runtimeMinuteFlapIndex = targetMinuteFlapIndexForTime(minute);
  persistCurrentRuntimeState();
  Serial.println("Calibration saved.");
}

void assignCurrentMechanismToFlaps(int hourFlapIndex, int minuteFlapIndex) {
  runtimeHourFlapIndex = hourFlapIndex;
  runtimeMinuteFlapIndex = minuteFlapIndex;
  persistCurrentRuntimeState();
  Serial.println("Manual flap calibration saved.");
}

void updateClockFromTime() {
  if (!timeValid || calibrationMode || !stateLoaded) {
    return;
  }

  if (stepperH.distanceToGo() != 0 || stepperM.distanceToGo() != 0) {
    return;
  }

  const int targetMinuteIndex = targetMinuteFlapIndexForTime(curMin);
  const int targetHourIndex = targetHourFlapIndexForTime(curHour, curMin);

  if (runtimeMinuteFlapIndex != targetMinuteIndex) {
    const int deltaFlaps = (targetMinuteIndex - runtimeMinuteFlapIndex + FLAP_COUNT) % FLAP_COUNT;
    scheduleRelativeMove(stepperM, deltaFlaps * STEPS_PER_FLAP);
    runtimeMinuteFlapIndex = targetMinuteIndex;
    minuteSavePending = true;
    return;
  }

  if (runtimeHourFlapIndex != targetHourIndex) {
    const int deltaFlaps = (targetHourIndex - runtimeHourFlapIndex + FLAP_COUNT) % FLAP_COUNT;
    scheduleRelativeMove(stepperH, -deltaFlaps * STEPS_PER_FLAP);
    runtimeHourFlapIndex = targetHourIndex;
    hourSavePending = true;
  }
}

void updateServoDisplay() {
  if (!timeValid) {
    return;
  }

  const int displayHour = (curMin >= 30) ? (curHour + 1) % 24 : curHour;

  targetPos1_4MIN = pos1_4MIN[curMin % 5];

  if (curMin < 5) {
    targetPosPO = posPO[0];
    targetPosZA = posZA[0];
    targetPosEND = posEND[0];
    return;
  }

  if (curMin < 30) {
    targetPosPO = posPO[1];
    targetPosZA = posZA[0];
    targetPosEND = (displayHour == 2 || displayHour == 14) ? posEND[2] : posEND[1];
    return;
  }

  if (curMin < 35) {
    targetPosPO = posPO[0];
    targetPosZA = posZA[0];
    targetPosEND = (displayHour == 2 || displayHour == 14) ? posEND[2] : posEND[1];
    return;
  }

  targetPosPO = posPO[0];
  targetPosZA = posZA[1];
  targetPosEND = posEND[0];
}

String buildHtmlPage() {
  String html;
  html.reserve(7000);

  html += "<!doctype html><html lang='pl'><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Zegar klapkowy</title><style>";
  html += "body{margin:0;padding:24px;background:#f4efe6;color:#1f2933;font-family:Arial,sans-serif;}";
  html += ".card{background:#fff;border-radius:16px;padding:20px;max-width:820px;margin:0 auto 16px;box-shadow:0 10px 30px rgba(0,0,0,.08);}";
  html += "h1,h2{margin-top:0;}p{line-height:1.5;}code{background:#eef2f7;padding:2px 6px;border-radius:6px;}";
  html += ".row{display:flex;gap:10px;flex-wrap:wrap;align-items:center;}button{padding:12px 16px;border:0;border-radius:10px;background:#1f6feb;color:#fff;cursor:pointer;font-size:15px;}";
  html += "button.secondary{background:#44566c;}button.warn{background:#b45309;}input{padding:10px;border:1px solid #cbd5e1;border-radius:8px;width:110px;}";
  html += "form{margin:8px 0;display:inline-block;}";
  html += "</style></head><body>";

  html += "<div class='card'><h1>Zegar klapkowy PL</h1>";
  html += "<p>Panel do ustawiania pozycji mechanizmu i kalibracji po restarcie.</p>";
  html += "<p><strong>WiFi IP:</strong> ";
  html += WiFi.localIP().toString();
  html += "</p><p><strong>Adres mDNS:</strong> http://";
  html += mdnsHostName;
  html += ".local";
  html += "</p><p><strong>Czas NTP:</strong> ";
  if (timeValid) {
    html += String(curHour) + ":" + (curMin < 10 ? "0" : "") + String(curMin) + ":" + (curSecond < 10 ? "0" : "") + String(curSecond);
  } else {
    html += "brak synchronizacji";
  }
  html += "</p><p><strong>Tryb:</strong> ";
  html += calibrationMode ? "kalibracja" : "normalna praca";
  html += "</p><p><strong>Stan zapisany:</strong> ";
  html += stateLoaded ? "tak" : "nie";
  html += "</p><p><strong>Pozycje stepperow:</strong> minuta <code>";
  html += String(stepperM.currentPosition());
  html += "</code>, godzina <code>";
  html += String(stepperH.currentPosition());
  html += "</code></p></div>";

  html += "<div class='card'><h2>Kalibracja</h2>";
  html += "<p>1. Ustaw mechanicznie klapki przyciskami ponizej. 2. Wybierz z list, jakie klapki faktycznie widzisz. 3. Kliknij jeden przycisk zapisu. Zegar zapisze kalibracje i wroci do aktualnego czasu z NTP.</p>";
  html += "<form action='/calibrate-manual' method='post' class='row'>";
  html += "<select name='minuteFlap'>";
  for (int i = 0; i < FLAP_COUNT; i++) {
    html += "<option value='";
    html += String(i);
    html += "'";
    if (i == runtimeMinuteFlapIndex) {
      html += " selected";
    }
    html += ">";
    html += MINUTE_FLAP_LABELS[i];
    html += "</option>";
  }
  html += "</select>";
  html += "<select name='hourFlap'>";
  for (int i = 0; i < FLAP_COUNT; i++) {
    html += "<option value='";
    html += String(i);
    html += "'";
    if (i == runtimeHourFlapIndex) {
      html += " selected";
    }
    html += ">";
    html += HOUR_FLAP_LABELS[i];
    html += "</option>";
  }
  html += "</select>";
  html += "<button class='secondary' type='submit'>Zapisz klapki i ustaw aktualny czas</button></form></div>";

  html += "<div class='card'><h2>Ruch o cala klapke</h2>";
  html += "<p>Te przyciski przesuwaja mechanizm do kalibracji. Nie zmieniaja jeszcze logiki zegara, dopoki nie zapiszesz kalibracji.</p>";
  html += "<p><strong>Minuty</strong></p><div class='row'>";
  html += "<form action='/stepper/move' method='post'><input type='hidden' name='stepper' value='minute'><input type='hidden' name='steps' value='-512'><button class='warn' type='submit'>-1 klapka</button></form>";
  html += "<form action='/stepper/move' method='post'><input type='hidden' name='stepper' value='minute'><input type='hidden' name='steps' value='512'><button class='warn' type='submit'>+1 klapka</button></form>";
  html += "</div><p><strong>Godziny</strong></p><div class='row'>";
  html += "<form action='/stepper/move' method='post'><input type='hidden' name='stepper' value='hour'><input type='hidden' name='steps' value='512'><button class='warn' type='submit'>-1 klapka</button></form>";
  html += "<form action='/stepper/move' method='post'><input type='hidden' name='stepper' value='hour'><input type='hidden' name='steps' value='-512'><button class='warn' type='submit'>+1 klapka</button></form>";
  html += "</div></div>";

  html += "<div class='card'><h2>Ruch precyzyjny</h2>";
  html += "<p>Do drobnego doregulowania pozycji mechanicznej steppera.</p>";
  html += "<p><strong>Minuty</strong></p><div class='row'>";
  html += "<form action='/stepper/move' method='post'><input type='hidden' name='stepper' value='minute'><input type='hidden' name='steps' value='-100'><button class='secondary' type='submit'>-100</button></form>";
  html += "<form action='/stepper/move' method='post'><input type='hidden' name='stepper' value='minute'><input type='hidden' name='steps' value='-10'><button class='secondary' type='submit'>-10</button></form>";
  html += "<form action='/stepper/move' method='post'><input type='hidden' name='stepper' value='minute'><input type='hidden' name='steps' value='10'><button class='secondary' type='submit'>+10</button></form>";
  html += "<form action='/stepper/move' method='post'><input type='hidden' name='stepper' value='minute'><input type='hidden' name='steps' value='100'><button class='secondary' type='submit'>+100</button></form>";
  html += "</div><p><strong>Godziny</strong></p><div class='row'>";
  html += "<form action='/stepper/move' method='post'><input type='hidden' name='stepper' value='hour'><input type='hidden' name='steps' value='100'><button class='secondary' type='submit'>-100</button></form>";
  html += "<form action='/stepper/move' method='post'><input type='hidden' name='stepper' value='hour'><input type='hidden' name='steps' value='10'><button class='secondary' type='submit'>-10</button></form>";
  html += "<form action='/stepper/move' method='post'><input type='hidden' name='stepper' value='hour'><input type='hidden' name='steps' value='-10'><button class='secondary' type='submit'>+10</button></form>";
  html += "<form action='/stepper/move' method='post'><input type='hidden' name='stepper' value='hour'><input type='hidden' name='steps' value='-100'><button class='secondary' type='submit'>+100</button></form>";
  html += "</div></div>";

  html += "<div class='card'><h2>Jak tego uzyc</h2>";
  html += "<p>Jesli zegar po restarcie nie wie, co aktualnie pokazuje, wejdz na te strone, ustaw klapki przyciskami, a potem kliknij zapis kalibracji. Od tej chwili pozycja i stan logiczny beda zapisane w EEPROM i po kolejnym restarcie zegar wznowi prace z poprawnego punktu.</p>";
  html += "</div>";

  html += "</body></html>";
  return html;
}
