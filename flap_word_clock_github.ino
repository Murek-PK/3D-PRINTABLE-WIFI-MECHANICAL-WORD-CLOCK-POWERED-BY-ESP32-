#include <WiFi.h>
#include "time.h"

#include <ESP32Servo.h>
Servo min1_4servo;
Servo toPastservo;

#include <AccelStepper.h>
#include <EEPROM.h>
#include <ESPmDNS.h>
#include <WebServer.h>

#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25

AccelStepper stepperH(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);

#define S2_IN1 33
#define S2_IN2 32
#define S2_IN3 21
#define S2_IN4 22

AccelStepper stepperM(AccelStepper::HALF4WIRE, S2_IN1, S2_IN3, S2_IN2, S2_IN4);

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

const char* hostName = "FlapWordClock";

const long STEPS_PER_FLAP = 512;
const int FLAP_COUNT = 12;
const long FULL_ROTATION_STEPS = STEPS_PER_FLAP * FLAP_COUNT;
const int EEPROM_SIZE = 64;
const uint32_t EEPROM_MAGIC = 0x46574331;  // FWC1
const unsigned long SERVO_STEP_INTERVAL_MS = 50;
const int SERVO_PIN_MINUTES = 18;
const int SERVO_PIN_TO_PAST = 19;

const int DEFAULT_MIN1_4_POS[5] = {140, 105, 72, 38, 0};
const int DEFAULT_TO_PAST_POS[3] = {0, 120, 180};
int min1_4pos[5] = {140, 105, 72, 38, 0};
int toPast[] = {0, 120, 180};
const char* HOUR_FLAPS[FLAP_COUNT] = {
  "ONE", "TWO", "THREE", "FOUR", "FIVE", "SIX",
  "SEVEN", "EIGHT", "NINE", "TEN", "ELEVEN", "TWELVE"
};
const char* MINUTE_FLAPS[FLAP_COUNT] = {
  "blank", "FIVE", "TEN", "QUARTER", "TWENTY", "TWENTY FIVE",
  "HALF", "TWENTY FIVE", "TWENTY", "QUARTER", "TEN", "FIVE"
};

enum ToPastMode {
  TO_PAST_MODE_PAST = 0,
  TO_PAST_MODE_TO = 1,
  TO_PAST_MODE_BLANK = 2
};

struct ClockDisplayState {
  int hourFlapIndex;
  int minuteFlapIndex;
  int minuteRemainder;
  int toPastMode;
};

struct PersistedState {
  uint32_t magic;
  long hourStepperPosition;
  long minuteStepperPosition;
  int minuteServoOneMinAngle;
  int toPastServoPastAngle;
  uint8_t isSummerTime;
};

PersistedState persistedState;
WebServer server(80);

unsigned long lastClockLogMs = 0;
long lastSavedHourPosition = 0;
long lastSavedMinutePosition = 0;
bool hourSavePending = false;
bool minuteSavePending = false;
int lastRenderedMinute = -1;
bool minServoAttached = false;
bool toPastServoAttached = false;
int minServoCurrentAngle = min1_4pos[0];
int minServoTargetAngle = min1_4pos[0];
int toPastServoCurrentAngle = toPast[TO_PAST_MODE_BLANK];
int toPastServoTargetAngle = toPast[TO_PAST_MODE_BLANK];
int minuteServoOneMinAngle = DEFAULT_MIN1_4_POS[1];
int toPastServoPastAngle = DEFAULT_TO_PAST_POS[TO_PAST_MODE_PAST];
unsigned long lastServoStepMs = 0;
unsigned long lastServoMovementFinishedMs = 0;

void loadPersistedState();
void savePersistedState();
void restoreStepperPositions();
void refreshServoCalibrationAngles();
void setupWebServer();
void handleRoot();
void handleStepperMove();
void handleStepperSetFlap();
void handleServoMove();
void handleServoSave();
void handleTimeModeSave();
void handleNotFound();
AccelStepper* getStepperByName(const String& stepperName);
const char* getStepperLabel(const String& stepperName);
const char* getServoLabel(const String& servoName);
long normalizeFlapIndex(long stepPosition);
String buildHtmlPage();
void queueStepperForward(AccelStepper& stepper, long stepCount);
void moveStepperToFlap(AccelStepper& stepper, int flapIndex);
void setCurrentFlap(AccelStepper& stepper, int flapIndex);
ClockDisplayState calculateDisplayState(const struct tm& timeinfo);
void applyDisplayState(const ClockDisplayState& state);
void renderCurrentTimeIfNeeded();
void showCurrentTimeNow();
const char* getToPastLabel(int mode);
void applyTimeConfig();
void previewServoCalibration(const String& servoName);
void updateServos();
void setServoTarget(Servo& servo, bool& isAttached, int pin, int& currentAngle, int& targetAngle, int newTargetAngle);
bool stepServoTowardTarget(Servo& servo, int& currentAngle, int targetAngle);
void enableStepperOutputs(AccelStepper& stepper);
void disableAllMotorOutputsIfIdle();
void updateStepperPersistence();
void logClockOncePerSecond();
void minDisplay(int min);
void toPastDisplay(int mode);

void setup() {
  Serial.begin(115200);
  delay(1000);

  EEPROM.begin(EEPROM_SIZE);
  loadPersistedState();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Laczenie z WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("WiFi polaczone, IP: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin(hostName)) {
    MDNS.addService("http", "tcp", 80);
    Serial.print("mDNS aktywne: http://");
    Serial.print(hostName);
    Serial.println(".local");
  } else {
    Serial.println("Nie udalo sie uruchomic mDNS");
  }

  applyTimeConfig();

  struct tm timeinfo;
  bool hasTime = false;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Nie udalo sie pobrac czasu");
  } else {
    hasTime = true;
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  }

  min1_4servo.setPeriodHertz(50);
  toPastservo.setPeriodHertz(50);
  lastServoMovementFinishedMs = millis();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(S2_IN1, OUTPUT);
  pinMode(S2_IN2, OUTPUT);
  pinMode(S2_IN3, OUTPUT);
  pinMode(S2_IN4, OUTPUT);

  stepperH.setMaxSpeed(200);
  stepperH.setAcceleration(300);
  stepperH.disableOutputs();

  stepperM.setMaxSpeed(200);
  stepperM.setAcceleration(300);
  stepperM.disableOutputs();

  restoreStepperPositions();
  setupWebServer();
  if (hasTime) {
    applyDisplayState(calculateDisplayState(timeinfo));
    lastRenderedMinute = timeinfo.tm_min;
  }
}

void loop() {
  stepperH.run();
  stepperM.run();
  updateServos();
  updateStepperPersistence();
  disableAllMotorOutputsIfIdle();
  server.handleClient();
  renderCurrentTimeIfNeeded();
  logClockOncePerSecond();
}

void loadPersistedState() {
  EEPROM.get(0, persistedState);

  if (persistedState.magic != EEPROM_MAGIC) {
    persistedState.magic = EEPROM_MAGIC;
    persistedState.hourStepperPosition = 0;
    persistedState.minuteStepperPosition = 0;
    persistedState.minuteServoOneMinAngle = DEFAULT_MIN1_4_POS[1];
    persistedState.toPastServoPastAngle = DEFAULT_TO_PAST_POS[TO_PAST_MODE_PAST];
    persistedState.isSummerTime = 1;
    savePersistedState();
  }

  if (persistedState.minuteServoOneMinAngle < 0 || persistedState.minuteServoOneMinAngle > 180) {
    persistedState.minuteServoOneMinAngle = DEFAULT_MIN1_4_POS[1];
  }

  if (persistedState.toPastServoPastAngle < 0 || persistedState.toPastServoPastAngle > 180) {
    persistedState.toPastServoPastAngle = DEFAULT_TO_PAST_POS[TO_PAST_MODE_PAST];
  }

  if (persistedState.isSummerTime > 1) {
    persistedState.isSummerTime = 1;
  }

  minuteServoOneMinAngle = persistedState.minuteServoOneMinAngle;
  toPastServoPastAngle = persistedState.toPastServoPastAngle;
  refreshServoCalibrationAngles();

  lastSavedHourPosition = persistedState.hourStepperPosition;
  lastSavedMinutePosition = persistedState.minuteStepperPosition;
}

void savePersistedState() {
  EEPROM.put(0, persistedState);
  EEPROM.commit();
}

void restoreStepperPositions() {
  stepperH.setCurrentPosition(persistedState.hourStepperPosition);
  stepperM.setCurrentPosition(persistedState.minuteStepperPosition);

  Serial.print("Przywrocono pozycje H: ");
  Serial.println(persistedState.hourStepperPosition);
  Serial.print("Przywrocono pozycje M: ");
  Serial.println(persistedState.minuteStepperPosition);
}

void refreshServoCalibrationAngles() {
  for (int i = 0; i < 5; i++) {
    min1_4pos[i] = constrain(
      minuteServoOneMinAngle + (DEFAULT_MIN1_4_POS[i] - DEFAULT_MIN1_4_POS[1]),
      0,
      180
    );
  }

  for (int i = 0; i < 3; i++) {
    toPast[i] = constrain(
      toPastServoPastAngle + (DEFAULT_TO_PAST_POS[i] - DEFAULT_TO_PAST_POS[TO_PAST_MODE_PAST]),
      0,
      180
    );
  }
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/stepper/move", HTTP_POST, handleStepperMove);
  server.on("/stepper/set-flap", HTTP_POST, handleStepperSetFlap);
  server.on("/servo/move", HTTP_POST, handleServoMove);
  server.on("/servo/save", HTTP_POST, handleServoSave);
  server.on("/time-mode/save", HTTP_POST, handleTimeModeSave);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("Serwer HTTP uruchomiony");
}

void handleRoot() {
  server.send(200, "text/html; charset=utf-8", buildHtmlPage());
}

void handleStepperMove() {
  String stepperName = server.arg("stepper");

  AccelStepper* stepper = getStepperByName(stepperName);
  if (stepper == nullptr) {
    server.send(400, "text/plain", "Unknown stepper");
    return;
  }

  long stepAmount = server.arg("steps").toInt();
  if (stepAmount <= 0) {
    server.send(400, "text/plain", "Invalid step count");
    return;
  }

  queueStepperForward(*stepper, stepAmount);
  server.sendHeader("Location", "/", true);
  server.send(303);
}

void handleStepperSetFlap() {
  String stepperName = server.arg("stepper");
  AccelStepper* stepper = getStepperByName(stepperName);
  if (stepper == nullptr) {
    server.send(400, "text/plain", "Unknown stepper");
    return;
  }

  int flapIndex = server.arg("flap").toInt();
  if (flapIndex < 0 || flapIndex >= FLAP_COUNT) {
    server.send(400, "text/plain", "Invalid flap index");
    return;
  }

  setCurrentFlap(*stepper, flapIndex);
  showCurrentTimeNow();
  server.sendHeader("Location", "/", true);
  server.send(303);
}

void handleServoMove() {
  String servoName = server.arg("servo");
  int delta = server.arg("delta").toInt();

  if (delta == 0) {
    server.send(400, "text/plain", "Invalid servo delta");
    return;
  }

  if (servoName == "minute_indicator") {
    minuteServoOneMinAngle = constrain(minuteServoOneMinAngle + delta, 0, 180);
  } else if (servoName == "to_past") {
    toPastServoPastAngle = constrain(toPastServoPastAngle + delta, 0, 180);
  } else {
    server.send(400, "text/plain", "Unknown servo");
    return;
  }

  refreshServoCalibrationAngles();
  previewServoCalibration(servoName);

  server.sendHeader("Location", "/", true);
  server.send(303);
}

void handleServoSave() {
  String servoName = server.arg("servo");

  if (servoName == "minute_indicator") {
    persistedState.minuteServoOneMinAngle = minuteServoOneMinAngle;
  } else if (servoName == "to_past") {
    persistedState.toPastServoPastAngle = toPastServoPastAngle;
  } else {
    server.send(400, "text/plain", "Unknown servo");
    return;
  }

  savePersistedState();
  showCurrentTimeNow();

  server.sendHeader("Location", "/", true);
  server.send(303);
}

void handleTimeModeSave() {
  String mode = server.arg("time_mode");

  if (mode == "summer") {
    persistedState.isSummerTime = 1;
  } else if (mode == "winter") {
    persistedState.isSummerTime = 0;
  } else {
    server.send(400, "text/plain", "Invalid time mode");
    return;
  }

  savePersistedState();
  applyTimeConfig();
  showCurrentTimeNow();

  server.sendHeader("Location", "/", true);
  server.send(303);
}

void handleNotFound() {
  server.send(404, "text/plain", "404");
}

AccelStepper* getStepperByName(const String& stepperName) {
  if (stepperName == "hour") {
    return &stepperH;
  }

  if (stepperName == "minute") {
    return &stepperM;
  }

  return nullptr;
}

const char* getStepperLabel(const String& stepperName) {
  if (stepperName == "hour") {
    return "Hour drum";
  }

  if (stepperName == "minute") {
    return "Minute drum";
  }

  return "Unknown";
}

const char* getServoLabel(const String& servoName) {
  if (servoName == "to_past") {
    return "PAST / TO servo";
  }

  if (servoName == "minute_indicator") {
    return "1-4 minute servo";
  }

  return "Unknown servo";
}

long normalizeFlapIndex(long stepPosition) {
  long flap = stepPosition / STEPS_PER_FLAP;
  flap %= FLAP_COUNT;

  if (flap < 0) {
    flap += FLAP_COUNT;
  }

  return flap;
}

String buildHtmlPage() {
  String html;
  html.reserve(12000);

  html += "<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Flap Word Clock - Calibration</title>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;background:#f3efe6;color:#1f2933;margin:0;padding:24px;}";
  html += "main{max-width:960px;margin:0 auto;}";
  html += "h1{margin-bottom:8px;}p{line-height:1.5;}";
  html += ".stack{display:flex;flex-direction:column;gap:20px;margin-top:24px;}";
  html += ".card{background:#fff;border-radius:16px;padding:20px;box-shadow:0 8px 30px rgba(0,0,0,0.08);}";
  html += ".status{background:#e9f5ec;border-radius:12px;padding:12px 14px;margin:16px 0;}";
  html += ".row{display:flex;gap:12px;flex-wrap:wrap;margin-top:16px;}";
  html += "button{border:none;border-radius:10px;padding:12px 16px;font-size:16px;cursor:pointer;background:#1f6f5f;color:#fff;}";
  html += "button.alt{background:#355c7d;}";
  html += "select{padding:10px 12px;font-size:16px;border-radius:10px;border:1px solid #c6d0da;}";
  html += "form{margin:0;}";
  html += ".meta{font-size:14px;color:#52606d;}";
  html += ".instructions{background:#fff8e8;border-radius:12px;padding:14px 16px;border:1px solid #ead9b1;}";
  html += "</style></head><body><main>";
  html += "<h1>Flap Word Clock</h1>";
  html += "<p>Calibration panel available at <strong>http://";
  html += hostName;
  html += ".local</strong>.</p>";
  html += "<div class='status'>";
  html += "Hour position: <strong>" + String(stepperH.currentPosition()) + "</strong> steps, flap <strong>" + String(normalizeFlapIndex(stepperH.currentPosition())) + "</strong><br>";
  html += "Minute position: <strong>" + String(stepperM.currentPosition()) + "</strong> steps, flap <strong>" + String(normalizeFlapIndex(stepperM.currentPosition())) + "</strong><br>";
  html += "Time mode: <strong>";
  html += persistedState.isSummerTime ? "Summer time" : "Winter time";
  html += "</strong>";
  html += "</div>";
  html += "<div class='instructions'>";
  html += "<strong>Calibration flow:</strong> move the drum until the visible flap is centered, then choose the text you currently see and confirm it.<br>";
  html += "Recommended starting point: set the hour drum to <strong>TWELVE</strong> and the minute drum to <strong>blank</strong>.<br>";
  html += "Use <strong>+100</strong> for coarse movement and <strong>+10</strong> / <strong>+1</strong> for fine adjustment.<br>";
  html += "For servo calibration, use <strong>-1</strong> and <strong>+1</strong> until <strong>PAST</strong> is fully visible and the <strong>1 min</strong> pointer is exactly aligned.<br>";
  html += "Confirming a visible flap also re-bases the internal step counter to a safe single-turn range.";
  html += "</div>";
  html += "<section class='card' style='margin-top:24px;'>";
  html += "<h2>Time mode</h2>";
  html += "<p class='meta'>Choose whether the clock should run in winter time or summer time.</p>";
  html += "<form method='post' action='/time-mode/save' class='row'>";
  html += "<select name='time_mode'>";
  html += "<option value='winter'";
  if (!persistedState.isSummerTime) {
    html += " selected";
  }
  html += ">Winter time</option>";
  html += "<option value='summer'";
  if (persistedState.isSummerTime) {
    html += " selected";
  }
  html += ">Summer time</option>";
  html += "</select>";
  html += "<button type='submit'>Save time mode</button>";
  html += "</form>";
  html += "</section>";
  html += "<div class='stack'>";

  String moduleNames[4] = {"minute_stepper", "to_past_servo", "hour_stepper", "minute_indicator_servo"};
  for (int i = 0; i < 4; i++) {
    String moduleName = moduleNames[i];
    bool isStepper = moduleName == "minute_stepper" || moduleName == "hour_stepper";
    String stepperName = moduleName == "minute_stepper" ? "minute" : "hour";
    AccelStepper* stepper = getStepperByName(stepperName);
    html += "<section class='card'>";
    html += "<h2>";
    if (moduleName == "minute_stepper") {
      html += getStepperLabel("minute");
    } else if (moduleName == "to_past_servo") {
      html += getServoLabel("to_past");
    } else if (moduleName == "hour_stepper") {
      html += getStepperLabel("hour");
    } else {
      html += getServoLabel("minute_indicator");
    }
    html += "</h2>";
    if (isStepper) {
      html += "<p class='meta'>Current flap: ";
      html += String(normalizeFlapIndex(stepper->currentPosition()));
      html += " (";
      html += stepperName == "hour" ? HOUR_FLAPS[normalizeFlapIndex(stepper->currentPosition())] : MINUTE_FLAPS[normalizeFlapIndex(stepper->currentPosition())];
      html += ") / 11<br>Target: ";
      html += String(stepper->targetPosition());
      html += "<br>Motion: ";
      html += stepper->distanceToGo() == 0 ? "idle" : "moving";
      html += "</p>";
      html += "<div class='row'>";
      html += "<form method='post' action='/stepper/move'>";
      html += "<input type='hidden' name='stepper' value='" + stepperName + "'>";
      html += "<input type='hidden' name='steps' value='100'>";
      html += "<button type='submit'>+100 steps</button></form>";
      html += "<form method='post' action='/stepper/move'>";
      html += "<input type='hidden' name='stepper' value='" + stepperName + "'>";
      html += "<input type='hidden' name='steps' value='10'>";
      html += "<button class='alt' type='submit'>+10 steps</button></form>";
      html += "<form method='post' action='/stepper/move'>";
      html += "<input type='hidden' name='stepper' value='" + stepperName + "'>";
      html += "<input type='hidden' name='steps' value='1'>";
      html += "<button class='alt' type='submit'>+1 step</button></form>";
      html += "</div>";
      html += "<div class='row'>";
      html += "<form method='post' action='/stepper/set-flap'>";
      html += "<input type='hidden' name='stepper' value='" + stepperName + "'>";
      html += "<select name='flap'>";

      for (int flap = 0; flap < FLAP_COUNT; flap++) {
        html += "<option value='" + String(flap) + "'";
        if (flap == normalizeFlapIndex(stepper->currentPosition())) {
          html += " selected";
        }
        html += ">Flap ";
        html += String(flap);
        html += " - ";
        html += stepperName == "hour" ? HOUR_FLAPS[flap] : MINUTE_FLAPS[flap];
        html += "</option>";
      }

      html += "</select>";
      html += "<button type='submit'>I can see this flap</button></form>";
      html += "</div>";
    } else {
      String servoName = moduleName == "to_past_servo" ? "to_past" : "minute_indicator";
      int calibrationAngle = servoName == "to_past" ? toPastServoPastAngle : minuteServoOneMinAngle;
      int savedAngle = servoName == "to_past" ? persistedState.toPastServoPastAngle : persistedState.minuteServoOneMinAngle;
      String calibrationTarget = servoName == "to_past" ? "PAST fully visible" : "pointer exactly at 1 min";

      html += "<p class='meta'>Calibration target: ";
      html += calibrationTarget;
      html += "<br>Reference angle: ";
      html += String(calibrationAngle);
      html += "<br>Saved angle: ";
      html += String(savedAngle);
      html += "</p>";
      html += "<div class='row'>";
      html += "<form method='post' action='/servo/move'>";
      html += "<input type='hidden' name='servo' value='" + servoName + "'>";
      html += "<input type='hidden' name='delta' value='-1'>";
      html += "<button class='alt' type='submit'>-1</button></form>";
      html += "<form method='post' action='/servo/move'>";
      html += "<input type='hidden' name='servo' value='" + servoName + "'>";
      html += "<input type='hidden' name='delta' value='1'>";
      html += "<button type='submit'>+1</button></form>";
      html += "<form method='post' action='/servo/save'>";
      html += "<input type='hidden' name='servo' value='" + servoName + "'>";
      html += "<button type='submit'>Save position</button></form>";
      html += "</div>";
    }
    html += "</section>";
  }

  html += "</div>";
  html += "<p class='meta'>The steppers only move forward. One full flap equals 512 half-steps, and the drum wraps after 12 flaps. Positions are saved to EEPROM after motion completes and whenever you confirm the visible flap.</p>";
  html += "</main></body></html>";
  return html;
}

void queueStepperForward(AccelStepper& stepper, long stepCount) {
  enableStepperOutputs(stepper);
  long target = stepper.targetPosition() + stepCount;
  stepper.moveTo(target);

  if (&stepper == &stepperH) {
    hourSavePending = true;
  } else if (&stepper == &stepperM) {
    minuteSavePending = true;
  }

  Serial.print("Nowy target: ");
  Serial.println(target);
}

void moveStepperToFlap(AccelStepper& stepper, int flapIndex) {
  long currentFlap = normalizeFlapIndex(stepper.targetPosition());
  long flapDelta = flapIndex - currentFlap;

  if (flapDelta < 0) {
    flapDelta += FLAP_COUNT;
  }

  queueStepperForward(stepper, flapDelta * STEPS_PER_FLAP);
}

void setCurrentFlap(AccelStepper& stepper, int flapIndex) {
  enableStepperOutputs(stepper);
  // Manual calibration re-bases the step counter into a single safe revolution.
  long newPosition = static_cast<long>(flapIndex) * STEPS_PER_FLAP;
  stepper.setCurrentPosition(newPosition);
  stepper.moveTo(newPosition);

  if (&stepper == &stepperH) {
    persistedState.hourStepperPosition = newPosition;
    lastSavedHourPosition = newPosition;
  } else if (&stepper == &stepperM) {
    persistedState.minuteStepperPosition = newPosition;
    lastSavedMinutePosition = newPosition;
  }

  savePersistedState();

  Serial.print("Skalibrowano klapke na pozycje: ");
  Serial.println(newPosition);
}

ClockDisplayState calculateDisplayState(const struct tm& timeinfo) {
  ClockDisplayState state;
  int roundedMinutes = timeinfo.tm_min / 5;
  int minuteRemainder = timeinfo.tm_min % 5;
  int displayHour24 = timeinfo.tm_hour;

  if (roundedMinutes >= 7) {
    displayHour24 = (displayHour24 + 1) % 24;
  }

  state.hourFlapIndex = ((displayHour24 + 11) % 12);
  state.minuteFlapIndex = roundedMinutes;
  state.minuteRemainder = minuteRemainder;

  if (roundedMinutes == 0) {
    state.toPastMode = TO_PAST_MODE_BLANK;
  } else if (roundedMinutes <= 6) {
    state.toPastMode = TO_PAST_MODE_PAST;
  } else {
    state.toPastMode = TO_PAST_MODE_TO;
  }

  return state;
}

void applyDisplayState(const ClockDisplayState& state) {
  moveStepperToFlap(stepperH, state.hourFlapIndex);
  moveStepperToFlap(stepperM, state.minuteFlapIndex);
  minDisplay(state.minuteRemainder);
  toPastDisplay(state.toPastMode);

  Serial.print("Ustawiono zegar: ");
  Serial.print(HOUR_FLAPS[state.hourFlapIndex]);
  Serial.print(" | ");
  Serial.print(MINUTE_FLAPS[state.minuteFlapIndex]);
  Serial.print(" | ");
  Serial.print(getToPastLabel(state.toPastMode));
  Serial.print(" | reszta minut: ");
  Serial.println(state.minuteRemainder);
}

void renderCurrentTimeIfNeeded() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return;
  }

  if (timeinfo.tm_min == lastRenderedMinute) {
    return;
  }

  lastRenderedMinute = timeinfo.tm_min;
  applyDisplayState(calculateDisplayState(timeinfo));
}

void showCurrentTimeNow() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return;
  }

  lastRenderedMinute = timeinfo.tm_min;
  applyDisplayState(calculateDisplayState(timeinfo));
}

const char* getToPastLabel(int mode) {
  if (mode == TO_PAST_MODE_PAST) {
    return "PAST";
  }

  if (mode == TO_PAST_MODE_TO) {
    return "TO";
  }

  return "blank";
}

void applyTimeConfig() {
  int dstOffset = persistedState.isSummerTime ? daylightOffset_sec : 0;
  configTime(gmtOffset_sec, dstOffset, ntpServer);
}

void previewServoCalibration(const String& servoName) {
  if (servoName == "to_past") {
    setServoTarget(
      toPastservo,
      toPastServoAttached,
      SERVO_PIN_TO_PAST,
      toPastServoCurrentAngle,
      toPastServoTargetAngle,
      toPast[TO_PAST_MODE_PAST]
    );
    return;
  }

  if (servoName == "minute_indicator") {
    setServoTarget(
      min1_4servo,
      minServoAttached,
      SERVO_PIN_MINUTES,
      minServoCurrentAngle,
      minServoTargetAngle,
      min1_4pos[1]
    );
  }
}

void updateServos() {
  if (!minServoAttached && !toPastServoAttached) {
    return;
  }

  bool minuteNeedsMove = minServoAttached && (minServoCurrentAngle != minServoTargetAngle);
  bool toPastNeedsMove = toPastServoAttached && (toPastServoCurrentAngle != toPastServoTargetAngle);

  if (!minuteNeedsMove && !toPastNeedsMove) {
    return;
  }

  if (millis() - lastServoStepMs < SERVO_STEP_INTERVAL_MS) {
    return;
  }

  lastServoStepMs = millis();

  bool moved = false;
  if (minuteNeedsMove) {
    moved |= stepServoTowardTarget(min1_4servo, minServoCurrentAngle, minServoTargetAngle);
  }

  if (toPastNeedsMove) {
    moved |= stepServoTowardTarget(toPastservo, toPastServoCurrentAngle, toPastServoTargetAngle);
  }

  if (moved && minServoCurrentAngle == minServoTargetAngle && toPastServoCurrentAngle == toPastServoTargetAngle) {
    lastServoMovementFinishedMs = millis();
  }
}

void setServoTarget(Servo& servo, bool& isAttached, int pin, int& currentAngle, int& targetAngle, int newTargetAngle) {
  if (!isAttached) {
    servo.attach(pin, 500, 2400);
    servo.write(currentAngle);
    isAttached = true;
  }

  targetAngle = newTargetAngle;

  if (currentAngle == targetAngle) {
    servo.write(currentAngle);
    lastServoMovementFinishedMs = millis();
    return;
  }

  lastServoStepMs = 0;
}

bool stepServoTowardTarget(Servo& servo, int& currentAngle, int targetAngle) {
  if (currentAngle == targetAngle) {
    return false;
  }

  if (currentAngle < targetAngle) {
    currentAngle++;
  } else {
    currentAngle--;
  }

  servo.write(currentAngle);
  return true;
}

void enableStepperOutputs(AccelStepper& stepper) {
  stepper.enableOutputs();
}

void disableAllMotorOutputsIfIdle() {
  if (stepperH.distanceToGo() == 0) {
    stepperH.disableOutputs();
  }

  if (stepperM.distanceToGo() == 0) {
    stepperM.disableOutputs();
  }
}

void updateStepperPersistence() {
  if (hourSavePending && stepperH.distanceToGo() == 0) {
    long currentPosition = stepperH.currentPosition();
    if (currentPosition != lastSavedHourPosition) {
      persistedState.hourStepperPosition = currentPosition;
      lastSavedHourPosition = currentPosition;
      savePersistedState();
      Serial.print("Zapisano pozycje H: ");
      Serial.println(currentPosition);
    }
    hourSavePending = false;
  }

  if (minuteSavePending && stepperM.distanceToGo() == 0) {
    long currentPosition = stepperM.currentPosition();
    if (currentPosition != lastSavedMinutePosition) {
      persistedState.minuteStepperPosition = currentPosition;
      lastSavedMinutePosition = currentPosition;
      savePersistedState();
      Serial.print("Zapisano pozycje M: ");
      Serial.println(currentPosition);
    }
    minuteSavePending = false;
  }
}

void logClockOncePerSecond() {
  if (millis() - lastClockLogMs < 1000) {
    return;
  }

  lastClockLogMs = millis();

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.println(&timeinfo, "%H:%M:%S");
  } else {
    Serial.println("Brak czasu");
  }
}

void minDisplay(int min) {
  int q = min % 5;

  setServoTarget(min1_4servo, minServoAttached, SERVO_PIN_MINUTES, minServoCurrentAngle, minServoTargetAngle, min1_4pos[q]);
  Serial.print("minut: ");
  Serial.print(min);
  Serial.print(" =modulo=> ");
  Serial.print(q);
  Serial.print(" =pozycja serwa=> ");
  Serial.print(min1_4pos[q]);
  Serial.println();
}

void toPastDisplay(int mode) {
  if (mode < 0 || mode > 2) {
    mode = TO_PAST_MODE_BLANK;
  }

  setServoTarget(toPastservo, toPastServoAttached, SERVO_PIN_TO_PAST, toPastServoCurrentAngle, toPastServoTargetAngle, toPast[mode]);
  Serial.print("to/past: ");
  Serial.println(getToPastLabel(mode));
}
