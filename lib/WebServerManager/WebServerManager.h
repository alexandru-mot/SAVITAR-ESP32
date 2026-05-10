#ifndef WEBSERVERMANAGER_H
#define WEBSERVERMANAGER_H

#include <Arduino.h>
#include <WebServer.h>

struct LFRConfig {
    float kp;
    float ki;
    float kd;
    int maxSpeed;
    int minSpeed;
    int straightBase;
    int emergencyThreshold;
    int emergencySpin;
};

class WebServerManager {
private:
    WebServer server;
    
    // Data from main loop
    float battVoltage = 0.0;
    int battPercentage = 0;
    bool robotRunning = false;
    LFRConfig cfg;
    
    // Handler functions
    void handleRoot();
    void handleUpdate();
    void handleSave();
    void handleStart();
    void handleStop();
    void handleDefaults();
    
    // HTML generation
    String generateHTML();
    
    // Utility
    float getArgFloat(const String& name, float currentValue);
    int getArgInt(const String& name, int currentValue);

public:
    WebServerManager();
    
    void begin(const char* ssid, const char* password);
    void handleClient();
    
    // Setters for data from main loop
    void setBatteryVoltage(float voltage);
    void setBatteryPercentage(int percentage);
    void setRobotRunning(bool running);
    void setConfig(LFRConfig config);
    
    // Getters for main loop
    bool getRobotRunning();
    LFRConfig getConfig();
    
    // Callback functions (to be linked to main loop)
    void (*onStart)() = nullptr;
    void (*onStop)() = nullptr;
    void (*onConfigUpdate)(LFRConfig) = nullptr;
    void (*onSaveConfig)() = nullptr;
    void (*onLoadDefaults)() = nullptr;
};

#endif