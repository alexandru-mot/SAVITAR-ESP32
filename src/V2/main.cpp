#include <Arduino.h>
#include <QTRSensors.h>
#include <Preferences.h>
#include <WiFi.h>
#include "WebServerManager.h"

/*
SAVITAR V1 Hardware V2 Software
Worked on it April-May 2026
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
const uint8_t SENSOR_PINS[] = {4, 16, 17, 18, 19, 14, 33, 23};
const uint8_t SENSOR_COUNT  = 8;
const uint8_t BATT_LEVEL_PIN = 36;
const uint8_t STATE_LED = 2;

const uint8_t RIGHT_MOT_FWD = 27;
const uint8_t RIGHT_MOT_REV = 21;
const uint8_t LEFT_MOT_FWD  = 25;
const uint8_t LEFT_MOT_REV  = 22;

// =====================================================
// GLOBAL VARIABLES
// =====================================================
float battVoltage = 0.0;
int battPercentage = 0;

LFRConfig cfg;
Preferences prefs;

QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];
float lastError     = 0;
float lastDerivative = 0;
float integral      = 0;
int   lineLostCount = 0;
int   lastPosition  = 3500;
const int LINE_LOST_THRESHOLD = 15;

WebServerManager webServer;

const char* AP_SSID = "SAVITAR_LFR";
const char* AP_PASS = "135798642";

// =====================================================
// PWM SETTINGS
// =====================================================
const int PWM_FREQ       = 5000;
const int PWM_RESOLUTION = 12;

const uint8_t CH_RIGHT_FWD = 0;
const uint8_t CH_RIGHT_REV = 1;
const uint8_t CH_LEFT_FWD  = 2;
const uint8_t CH_LEFT_REV  = 3;

// =====================================================
// BATTERY FUNCTIONS
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
// CONFIG FUNCTIONS
// =====================================================
void loadDefaults() {
    cfg.kp = 1.3;
    cfg.ki = 0.0;
    cfg.kd = 11.0;
    cfg.maxSpeed = 4000;
    cfg.minSpeed = -3000;
    cfg.straightBase = 3500;
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
    cfg.straightBase = prefs.getInt("strBase", cfg.straightBase);
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
    prefs.putInt("strBase", cfg.straightBase);
    prefs.putInt("emThr", cfg.emergencyThreshold);
    prefs.putInt("emSpin", cfg.emergencySpin);
    prefs.end();
}

// =====================================================
// MOTOR FUNCTIONS
// =====================================================
void setupMotorPWM() {
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
// CALIBRATION
// =====================================================
void calibrateSensors() {
    Serial.println("Calibrating... Move sensor over the line!");

    qtr.setTypeRC();
    qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);
    qtr.setEmitterPin(13);
    
    pinMode(STATE_LED, OUTPUT);
    int currentLEDState = LOW;

    for (int i = 0; i < 200; i++) {
        qtr.calibrate();
        currentLEDState = !currentLEDState;
        digitalWrite(STATE_LED, currentLEDState);
        delay(5);
    }
    
    stopMotors();
    digitalWrite(STATE_LED, LOW);
    
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
    digitalWrite(STATE_LED, HIGH);
}

// =====================================================
// WEBSERVER CALLBACKS
// =====================================================
void onStart() {
    Serial.println("Robot started from web");
}

void onStop() {
    Serial.println("Robot stopped from web");
    stopMotors();
}

void onConfigUpdate(LFRConfig newCfg) {
    cfg = newCfg;
    Serial.println("Config updated from web");
}

void onSaveConfig() {
    saveConfig();
    Serial.println("Config saved to flash");
}

void onLoadDefaults() {
    loadDefaults();
    stopMotors();
    Serial.println("Defaults loaded");
}

// =====================================================
// SETUP
// =====================================================
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

    // Setup webserver with callbacks
    webServer.begin(AP_SSID, AP_PASS);
    webServer.setConfig(cfg);
    webServer.onStart = onStart;
    webServer.onStop = onStop;
    webServer.onConfigUpdate = onConfigUpdate;
    webServer.onSaveConfig = onSaveConfig;
    webServer.onLoadDefaults = onLoadDefaults;

    Serial.println("Setup complete!");
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
    webServer.handleClient();

    battVoltage = getBatteryVoltage();
    battPercentage = getBatteryPercentage();
    
    // Update webserver with current values
    webServer.setBatteryVoltage(battVoltage);
    webServer.setBatteryPercentage(battPercentage);
    webServer.setRobotRunning(webServer.getRobotRunning());
    webServer.setConfig(cfg);

    if (webServer.getRobotRunning()) {
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

        // ---- LOST: Stop and wait ----
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
            driveLeft(-cfg.emergencySpin);
            driveRight(cfg.emergencySpin);
            lastError = -3500;
            lastPosition = position;
            Serial.println("EMERGENCY LEFT");
            return;
        }

        if (farRight && !farLeft) {
            driveLeft(cfg.emergencySpin);
            driveRight(-cfg.emergencySpin);
            lastError = 3500;
            lastPosition = position;
            Serial.println("EMERGENCY RIGHT");
            return;
        }

        // =========================================================
        // PURE PID — correction scales smoothly with error
        // =========================================================

        float error = (float)position - 3500.0;

        integral += error;
        integral = constrain(integral, -10000, 10000);

        float derivative = (error - lastError);
		derivative = (derivative * 0.5) + (lastDerivative * 0.5);  // Smooth it
		lastDerivative = derivative;

        float correction = (cfg.kp * error) + (cfg.ki * integral) + (cfg.kd * derivative);

        lastError = error;
		

        int leftSpeed  = cfg.straightBase + (int)correction;
        int rightSpeed = cfg.straightBase - (int)correction;

        // Apply motor offset
        // leftSpeed += cfg.leftOffset;

        // Constrain
        leftSpeed  = constrain(leftSpeed,  cfg.minSpeed, cfg.maxSpeed);
        rightSpeed = constrain(rightSpeed, cfg.minSpeed, cfg.maxSpeed);

        driveLeft(leftSpeed);
        driveRight(rightSpeed);
    } else {
        stopMotors();
    }
}