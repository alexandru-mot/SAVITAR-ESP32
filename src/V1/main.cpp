#include <Arduino.h>

#include <QTRSensors.h>
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

/*
SAVITAR V1 Hardware V1 Software
Worked on it during the RoboTEC 2026 
Took 2nd place at AC faculty RoboTEC LFR Classic Competition.

Features: 
- PD Tuning via Access Point Wireless
- Edge Detection Emergency Turn

MCU: WEMOS D1 Mini ESP32
Motor Driver: DVR8833
Wheels: Pololu D32
Motors: Micro N20 500RPM

by Alexandru Mot
*/

// =====================================================
// PIN DEFINITIONS
// =====================================================
// QTR-8RC sensor pins
const uint8_t SENSOR_PINS[] = {4, 16, 17, 18, 19, 14, 33, 23};
const uint8_t SENSOR_COUNT  = 8;

// BATT LEVEL
const uint8_t BATT_LEVEL_PIN = 36;

// DRV8833 motor driver pins
const uint8_t RIGHT_MOT_FWD = 27;  // AIN1
const uint8_t RIGHT_MOT_REV = 21;  // AIN2
const uint8_t LEFT_MOT_FWD  = 25;  // BIN1
const uint8_t LEFT_MOT_REV  = 22;  // BIN2



// =====================================================
// GLOBAL VARIABLES
// =====================================================

// BATT
float battVoltage = 0.0;
int battPercentage = 0;

// PREFS
struct LFRConfig {
    float kp;
    float ki;
    float kd;

    int maxSpeed;
    int minSpeed;
    int leftOffset;

    int straightZone;
    int curveZone;

    int straightBase;
    int curveBase;
    int sharpBase;

    int emergencyThreshold;
    int emergencySpin;
};
LFRConfig cfg;
Preferences prefs;

// LINE FOLLOWING
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];
float lastError     = 0;
float integral      = 0;
int   lineLostCount = 0;
int   lastPosition  = 3500;
const int LINE_LOST_THRESHOLD = 15;

// WEBSERVER
WebServer server(80);
bool robotRunning = false;

// ACCESS POINT CREDENTIALS
const char* AP_SSID = "SAVITAR_LFR";
const char* AP_PASS = "135798642";


// =====================================================
// PWM SETTINGS
// =====================================================
const int PWM_FREQ       = 5000;   // 5 kHz
const int PWM_RESOLUTION = 12;      // 0–255

// LEDC channels for ESP32 PWM
const uint8_t CH_RIGHT_FWD = 0;
const uint8_t CH_RIGHT_REV = 1;
const uint8_t CH_LEFT_FWD  = 2;
const uint8_t CH_LEFT_REV  = 3;


// =====================================================
// GET 2S LIPO VOLTAGE
// =====================================================
float getBatteryVoltage() {
    int rawADC = analogRead(BATT_LEVEL_PIN);
    float adcVoltage = (rawADC / 4095.0f) * 3.3f;
    float batteryVoltage = adcVoltage * ((100.0f + 33.0f) / 33.0f);
    return batteryVoltage;
}

int getBatteryPercentage() {
    float voltage = getBatteryVoltage();
    
    voltage = constrain(voltage, 6.0, 8.4);
    int intVoltage = voltage * 100;

    int percentage = map(intVoltage, 600, 840, 0, 100);
    return percentage;
}


// =====================================================
// CONFIG VALUES PREFERENCES.h
// =====================================================

void loadDefaults() {
  cfg.kp = 1.3;
  cfg.ki = 0.0;
  cfg.kd = 11.0;

  cfg.maxSpeed = 4000;
  cfg.minSpeed = -3000;
  cfg.leftOffset = 0;

  cfg.straightZone = 1000;
  cfg.curveZone = 2500;

  cfg.straightBase = 3500;
  cfg.curveBase = 2500;
  cfg.sharpBase = 500;

  cfg.emergencyThreshold = 50;
  cfg.emergencySpin = 4000;
}

void loadConfig() {
  prefs.begin("lfr", true);

  cfg.kp = prefs.getFloat("kp", cfg.kp);
  cfg.ki = prefs.getFloat("ki", cfg.ki);
  cfg.kd = prefs.getFloat("kd", cfg.kd);

  cfg.maxSpeed = prefs.getInt("maxSpd", cfg.maxSpeed);
  cfg.minSpeed = prefs.getInt("minSpd", cfg.minSpeed);
  cfg.leftOffset = prefs.getInt("leftOff", cfg.leftOffset);

  cfg.straightZone = prefs.getInt("strZone", cfg.straightZone);
  cfg.curveZone = prefs.getInt("curvZone", cfg.curveZone);

  cfg.straightBase = prefs.getInt("strBase", cfg.straightBase);
  cfg.curveBase = prefs.getInt("curvBase", cfg.curveBase);
  cfg.sharpBase = prefs.getInt("sharpBase", cfg.sharpBase);

  cfg.emergencyThreshold = prefs.getInt("emThr", cfg.emergencyThreshold);
  cfg.emergencySpin = prefs.getInt("emSpin", cfg.emergencySpin);

  prefs.end();
}

void saveConfig() {
  prefs.begin("lfr", false);

  prefs.putFloat("kp", cfg.kp);
  prefs.putFloat("ki", cfg.ki);
  prefs.putFloat("kd", cfg.kd);

  prefs.putInt("maxSpd", cfg.maxSpeed);
  prefs.putInt("minSpd", cfg.minSpeed);
  prefs.putInt("leftOff", cfg.leftOffset);

  prefs.putInt("strZone", cfg.straightZone);
  prefs.putInt("curvZone", cfg.curveZone);

  prefs.putInt("strBase", cfg.straightBase);
  prefs.putInt("curvBase", cfg.curveBase);
  prefs.putInt("sharpBase", cfg.sharpBase);

  prefs.putInt("emThr", cfg.emergencyThreshold);
  prefs.putInt("emSpin", cfg.emergencySpin);

  prefs.end();
}

// =====================================================
// MOTOR FUNCTIONS
// =====================================================
void setupMotorPWM() {
    // Configure LEDC channels then attach pins
    ledcSetup(CH_RIGHT_FWD, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_RIGHT_REV, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_LEFT_FWD,  PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_LEFT_REV,  PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(RIGHT_MOT_FWD, CH_RIGHT_FWD);
    ledcAttachPin(RIGHT_MOT_REV, CH_RIGHT_REV);
    ledcAttachPin(LEFT_MOT_FWD,  CH_LEFT_FWD);
    ledcAttachPin(LEFT_MOT_REV,  CH_LEFT_REV);
}

void driveRight(int speed) {
    speed = constrain(speed, -cfg.maxSpeed, cfg.maxSpeed);
    if (speed >= 0) {
        ledcWrite(CH_RIGHT_FWD, speed);
        ledcWrite(CH_RIGHT_REV, 0);
    } else {
        ledcWrite(CH_RIGHT_FWD, 0);
        ledcWrite(CH_RIGHT_REV, -speed);
    }
}

void driveLeft(int speed) {
    speed = constrain(speed, -cfg.maxSpeed, cfg.maxSpeed);
    if (speed >= 0) {
        ledcWrite(CH_LEFT_FWD, speed);
        ledcWrite(CH_LEFT_REV, 0);
    } else {
        ledcWrite(CH_LEFT_FWD, 0);
        ledcWrite(CH_LEFT_REV, -speed);
    }
}

void stopMotors() {
    ledcWrite(CH_RIGHT_FWD, 0);
    ledcWrite(CH_RIGHT_REV, 0);
    ledcWrite(CH_LEFT_FWD, 0);
    ledcWrite(CH_LEFT_REV, 0);
}


// =====================================================
// HTML PAGE
// =====================================================
String htmlPage() {
  String status = robotRunning ? "RUNNING" : "STOPPED";

  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>LFR Config</title>
  <style>
    body { font-family: Arial; margin: 20px; background: #f7f7f7; }
    h1 { margin-bottom: 5px; }
    .card { background: white; padding: 16px; border-radius: 12px; box-shadow: 0 2px 8px rgba(0,0,0,0.08); max-width: 700px; }
    .row { margin-bottom: 10px;}
    label { display: inline-block; width: 180px; font-weight: bold; }
    input { width: 140px; padding: 6px; }
    button {
      padding: 10px 16px;
      border: none;
      border-radius: 8px;
      cursor: pointer;
      font-size: 16px;
      width: 100%;
      box-sizing: border-box;
      margin-bottom: 8px;
    }
    .save { background: #2e86de; color: white; }
    .start { background: #27ae60; color: white; }
    .stop { background: #c0392b; color: white; }
    .defaults { background: #f39c12; color: white; }
    .status { font-size: 18px; margin-bottom: 14px; }
    .voltage-ok { color: #27ae60; }
    .voltage-low { color: #c0392b; }
    
    /* Start/Stop row - split 50/50 */
    .button-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      margin-bottom: 8px;
    }
    .button-row button {
      margin-bottom: 0;
    }
  </style>
</head>
<body>
  <div class="card">
    <h1>SAVITAR LFR Config Panel</h1>
    <div class="status"><b>Status:</b> %STATUS%</div>
)rawliteral";

    html += "<div class='status ";
    if (battVoltage < 6.5) {
        html += "voltage-low";
    } else {
        html += "voltage-ok";
    }
    html += "'><b>2S LiPo Voltage:</b> ";
    html += String(battVoltage, 2);
    html += "V (";
    html += String(battPercentage);
    html += "%)</div>";

    html += R"rawliteral(

    <form action="/update" method="post">
      <div class="row"><label>Kp</label><input name="kp" value="%KP%" onfocus="this.select()"></div>
      <div class="row"><label>Ki</label><input name="ki" value="%KI%" onfocus="this.select()"></div>
      <div class="row"><label>Kd</label><input name="kd" value="%KD%" onfocus="this.select()"></div>

      <div class="row"><label>Max Speed</label><input name="maxSpeed" value="%MAXSPEED%" onfocus="this.select()"></div>
      <div class="row"><label>Min Speed</label><input name="minSpeed" value="%MINSPEED%" onfocus="this.select()"></div>
      <div class="row"><label>Left Offset</label><input name="leftOffset" value="%LEFTOFFSET%" onfocus="this.select()"></div>

      <div class="row"><label>Straight Zone</label><input name="straightZone" value="%STRAIGHTZONE%" onfocus="this.select()"></div>
      <div class="row"><label>Curve Zone</label><input name="curveZone" value="%CURVEZONE%" onfocus="this.select()"></div>

      <div class="row"><label>Straight Base</label><input name="straightBase" value="%STRAIGHTBASE%" onfocus="this.select()"></div>
      <div class="row"><label>Curve Base</label><input name="curveBase" value="%CURVEBASE%" onfocus="this.select()"></div>
      <div class="row"><label>Sharp Base</label><input name="sharpBase" value="%SHARPBASE%" onfocus="this.select()"></div>

      <div class="row"><label>Emergency Threshold</label><input name="emergencyThreshold" value="%EMERGENCYTHRESHOLD%" onfocus="this.select()"></div>
      <div class="row"><label>Emergency Spin</label><input name="emergencySpin" value="%EMERGENCYSPIN%" onfocus="this.select()"></div>

      <button class="save" type="submit">Apply Values</button>
    </form>

    <form action="/save" method="post">
      <button class="save" type="submit">Save to Flash</button>
    </form>

    <div class="button-row">
      <form action="/start" method="post" style="margin: 0;">
        <button class="start" type="submit">Start</button>
      </form>

      <form action="/stop" method="post" style="margin: 0;">
        <button class="stop" type="submit">Stop</button>
      </form>
    </div>

    <form action="/defaults" method="post">
      <button class="defaults" type="submit">Load Defaults</button>
    </form>
  </div>
</body>
</html>
)rawliteral";

  html.replace("%STATUS%", status);
  html.replace("%KP%", String(cfg.kp, 4));
  html.replace("%KI%", String(cfg.ki, 4));
  html.replace("%KD%", String(cfg.kd, 4));

  html.replace("%MAXSPEED%", String(cfg.maxSpeed));
  html.replace("%MINSPEED%", String(cfg.minSpeed));
  html.replace("%LEFTOFFSET%", String(cfg.leftOffset));

  html.replace("%STRAIGHTZONE%", String(cfg.straightZone));
  html.replace("%CURVEZONE%", String(cfg.curveZone));

  html.replace("%STRAIGHTBASE%", String(cfg.straightBase));
  html.replace("%CURVEBASE%", String(cfg.curveBase));
  html.replace("%SHARPBASE%", String(cfg.sharpBase));

  html.replace("%EMERGENCYTHRESHOLD%", String(cfg.emergencyThreshold));
  html.replace("%EMERGENCYSPIN%", String(cfg.emergencySpin));

  return html;
}


float getArgFloat(const String& name, float currentValue) {
  if (server.hasArg(name)) return server.arg(name).toFloat();
  return currentValue;
}

int getArgInt(const String& name, int currentValue) {
  if (server.hasArg(name)) return server.arg(name).toInt();
  return currentValue;
}

void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleUpdate() {
  cfg.kp = getArgFloat("kp", cfg.kp);
  cfg.ki = getArgFloat("ki", cfg.ki);
  cfg.kd = getArgFloat("kd", cfg.kd);

  cfg.maxSpeed = getArgInt("maxSpeed", cfg.maxSpeed);
  cfg.minSpeed = getArgInt("minSpeed", cfg.minSpeed);
  cfg.leftOffset = getArgInt("leftOffset", cfg.leftOffset);

  cfg.straightZone = getArgInt("straightZone", cfg.straightZone);
  cfg.curveZone = getArgInt("curveZone", cfg.curveZone);

  cfg.straightBase = getArgInt("straightBase", cfg.straightBase);
  cfg.curveBase = getArgInt("curveBase", cfg.curveBase);
  cfg.sharpBase = getArgInt("sharpBase", cfg.sharpBase);

  cfg.emergencyThreshold = getArgInt("emergencyThreshold", cfg.emergencyThreshold);
  cfg.emergencySpin = getArgInt("emergencySpin", cfg.emergencySpin);

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSave() {
  saveConfig();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStart() {
  robotRunning = true;
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStop() {
  robotRunning = false;
  stopMotors();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleDefaults() {
  loadDefaults();
  robotRunning = false;
  stopMotors();
  server.sendHeader("Location", "/");
  server.send(303);
}

// ========================= Wi-Fi =============================
void startAccessPoint() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(ip);
}


// ===================== CALIBRATION ===========================
void calibrateSensors() {
    Serial.println("Calibrating... Move sensor over the line!");

    qtr.setTypeRC();
    qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);
    qtr.setEmitterPin(13);
    
    // Turn on built-in LED during calibration
    pinMode(2, OUTPUT);
    int currentLEDState = LOW;

    
    // Sweep sensors over the line for ~5 seconds
    for (int i = 0; i < 200; i++) {
        qtr.calibrate();
        currentLEDState = !currentLEDState;
        digitalWrite(2, currentLEDState);

        delay(5);
    }
    
    stopMotors();
    digitalWrite(2, LOW);
    
    // Print calibration results
    Serial.println("Calibration MIN values:");
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print("  ");
    }
    Serial.println();
    
    Serial.println("Calibration MAX values:");
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print("  ");
    }
    Serial.println();
    Serial.println("Calibration complete!");
    delay(1000);
    digitalWrite(2, HIGH);
}

// ===================== SETUP =================================
void setup() {
    Serial.begin(115200);
    Serial.println("Line Follower Starting...");

    analogReadResolution(12);
    pinMode(BATT_LEVEL_PIN, INPUT);
    
    setupMotorPWM();
    stopMotors();
    
    delay(2000);
    
    calibrateSensors();

    loadDefaults();
    loadConfig();

    startAccessPoint();

    server.on("/", HTTP_GET, handleRoot);
    server.on("/update", HTTP_POST, handleUpdate);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/start", HTTP_POST, handleStart);
    server.on("/stop", HTTP_POST, handleStop);
    server.on("/defaults", HTTP_POST, handleDefaults);
    
    server.begin();

    Serial.println("Web server started.");
    Serial.println("Connect to WiFi: SAVITAR_LFR");
    Serial.println("Open: http://192.168.4.1/");
}

// ===================== MAIN LOOP =============================
void loop() {
    server.handleClient();

    battVoltage = getBatteryVoltage();
    battPercentage = getBatteryPercentage();

    if (robotRunning) {
        uint16_t position = qtr.readLineBlack(sensorValues);
    
        // ---- Line lost detection ----
        bool allSensorsLow = true;
        for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
            if (sensorValues[i] > 100) {
                allSensorsLow = false;
                break;
            }
        }
        
        if (allSensorsLow) {
            lineLostCount++;
        } else {
            lineLostCount = 0;
            lastPosition = position;
        }
        
        // ---- LOST: Spin toward last known direction ----
        if (lineLostCount > LINE_LOST_THRESHOLD) {
            stopMotors();
            return;
        }

        // =========================================================
        // EMERGENCY TURN — line on outermost sensors (sharp angle)
        // =========================================================
        
        bool farLeft  = (sensorValues[0] > cfg.emergencyThreshold);
        bool farRight = (sensorValues[7] > cfg.emergencyThreshold);
        
        if (farLeft && !farRight) {
            // Line hit sensor 1 → hard spin LEFT
            driveLeft(-cfg.emergencySpin);
            driveRight(cfg.emergencySpin);
            lastError = -3500;
            lastPosition = position;
            Serial.println("⚠ EMERGENCY LEFT");
            return;
        }
        
        if (farRight && !farLeft) {
            // Line hit sensor 8 → hard spin RIGHT
            driveLeft(cfg.emergencySpin);
            driveRight(-cfg.emergencySpin);
            lastError = 3500;
            lastPosition = position;
            Serial.println("⚠ EMERGENCY RIGHT");
            return;
        }
        
        // ---- PID ----
        float error = (float)position - 3500.0;
        float absError = abs(error);
        
        integral += error;
        integral = constrain(integral, -10000, 10000);
        
        float derivative = error - lastError;
        float correction = (cfg.kp * error) + (cfg.ki * integral) + (cfg.kd * derivative);
        lastError = error;
        
        int leftSpeed, rightSpeed;
        
        // ---- ZONE CONTROL ----
        if (absError < cfg.straightZone) {
            // ══════ STRAIGHT ══════
            leftSpeed  = cfg.straightBase + (int)correction;
            rightSpeed = cfg.straightBase - (int)correction;
            
        } else if (absError < cfg.curveZone) {
            // ═══╗ GENTLE CURVE ╔═══
            leftSpeed  = cfg.curveBase + (int)correction;
            rightSpeed = cfg.curveBase - (int)correction;
            
        } else {
            // ╗ SHARP CORNER ╔
            // Boost correction + low base = aggressive turn
            float sharpCorrection = correction * 2.5;
            leftSpeed  = cfg.sharpBase + (int)sharpCorrection;
            rightSpeed = cfg.sharpBase - (int)sharpCorrection;
        }
        
        // Apply motor offset
        leftSpeed += cfg.leftOffset;
        
        // Constrain
        leftSpeed  = constrain(leftSpeed,  cfg.minSpeed, cfg.maxSpeed);
        rightSpeed = constrain(rightSpeed, cfg.minSpeed, cfg.maxSpeed);
        
        driveLeft(leftSpeed);
        driveRight(rightSpeed);
    } else {
        stopMotors();
    }
}