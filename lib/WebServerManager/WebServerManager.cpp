#include "WebServerManager.h"

WebServerManager::WebServerManager() : server(80) {}

void WebServerManager::begin(const char* ssid, const char* password) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    
    IPAddress ip = WiFi.softAPIP();
    Serial.print("AP IP: ");
    Serial.println(ip);
    
    // Bind handlers
    server.on("/", HTTP_GET, [this]() { handleRoot(); });
    server.on("/update", HTTP_POST, [this]() { handleUpdate(); });
    server.on("/save", HTTP_POST, [this]() { handleSave(); });
    server.on("/start", HTTP_POST, [this]() { handleStart(); });
    server.on("/stop", HTTP_POST, [this]() { handleStop(); });
    server.on("/defaults", HTTP_POST, [this]() { handleDefaults(); });
    
    server.begin();
    Serial.println("Web server started.");
    Serial.println("Connect to WiFi: " + String(ssid));
    Serial.println("Open: http://192.168.4.1/");
}

void WebServerManager::handleClient() {
    server.handleClient();
}

// ==================== SETTERS ====================

void WebServerManager::setBatteryVoltage(float voltage) {
    battVoltage = voltage;
}

void WebServerManager::setBatteryPercentage(int percentage) {
    battPercentage = percentage;
}

void WebServerManager::setRobotRunning(bool running) {
    robotRunning = running;
}

void WebServerManager::setConfig(LFRConfig config) {
    cfg = config;
}

// ==================== GETTERS ====================

bool WebServerManager::getRobotRunning() {
    return robotRunning;
}

LFRConfig WebServerManager::getConfig() {
    return cfg;
}

// ==================== HANDLERS ====================

void WebServerManager::handleRoot() {
    server.send(200, "text/html", generateHTML());
}

void WebServerManager::handleUpdate() {
    cfg.kp = getArgFloat("kp", cfg.kp);
    cfg.ki = getArgFloat("ki", cfg.ki);
    cfg.kd = getArgFloat("kd", cfg.kd);
    cfg.maxSpeed = getArgInt("maxSpeed", cfg.maxSpeed);
    cfg.minSpeed = getArgInt("minSpeed", cfg.minSpeed);
    cfg.straightBase = getArgInt("straightBase", cfg.straightBase);
    cfg.emergencyThreshold = getArgInt("emergencyThreshold", cfg.emergencyThreshold);
    cfg.emergencySpin = getArgInt("emergencySpin", cfg.emergencySpin);
    
    if (onConfigUpdate) {
        onConfigUpdate(cfg);
    }
    
    server.sendHeader("Location", "/");
    server.send(303);
}

void WebServerManager::handleSave() {
    if (onSaveConfig) {
        onSaveConfig();
    }
    server.sendHeader("Location", "/");
    server.send(303);
}

void WebServerManager::handleStart() {
    robotRunning = true;
    if (onStart) {
        onStart();
    }
    server.sendHeader("Location", "/");
    server.send(303);
}

void WebServerManager::handleStop() {
    robotRunning = false;
    if (onStop) {
        onStop();
    }
    server.sendHeader("Location", "/");
    server.send(303);
}

void WebServerManager::handleDefaults() {
    robotRunning = false;
    if (onLoadDefaults) {
        onLoadDefaults();
    }
    server.sendHeader("Location", "/");
    server.send(303);
}

// ==================== UTILITIES ====================

float WebServerManager::getArgFloat(const String& name, float currentValue) {
    if (server.hasArg(name)) {
        String value = server.arg(name);
        value.replace(",", "."); 
        return value.toFloat();
    }
    return currentValue;
}

int WebServerManager::getArgInt(const String& name, int currentValue) {
    if (server.hasArg(name)) return server.arg(name).toInt();
    return currentValue;
}

// ==================== HTML GENERATION ====================

String WebServerManager::generateHTML() {
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
    .row { margin-bottom: 16px; }
    label { display: block; font-weight: bold; margin-bottom: 6px; }
    input { 
      width: 100%; 
      padding: 12px; 
      font-size: 16px;
      border: 1px solid #ddd;
      border-radius: 6px;
      box-sizing: border-box;
    }
    input:focus { 
      outline: none;
      border-color: #2e86de;
      box-shadow: 0 0 4px rgba(46, 134, 222, 0.3);
    }
    button {
      padding: 12px 16px;
      border: none;
      border-radius: 8px;
      cursor: pointer;
      font-size: 16px;
      width: 100%;
      box-sizing: border-box;
      margin-bottom: 8px;
      font-weight: bold;
    }
    .save { background: #2e86de; color: white; }
    .start { background: #27ae60; color: white; }
    .stop { background: #c0392b; color: white; }
    .defaults { background: #f39c12; color: white; }
    .status { font-size: 18px; margin-bottom: 14px; font-weight: bold; }
    .voltage-ok { color: #27ae60; }
    .voltage-low { color: #c0392b; }
    
    .button-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      margin-bottom: 16px;
    }
    .button-row button {
      margin-bottom: 0;
      padding: 12px;
    }

    .dropdown-header {
      background: #f0f0f0;
      padding: 14px;
      border-radius: 6px;
      cursor: pointer;
      user-select: none;
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-weight: bold;
      border: 1px solid #ddd;
      margin-bottom: 12px;
    }
    .dropdown-header:hover {
      background: #e8e8e8;
    }
    .dropdown-header.active {
      background: #2e86de;
      color: white;
      border-color: #2e86de;
    }
    .dropdown-content {
      max-height: 0;
      overflow: hidden;
      transition: max-height 0.3s ease-out;
      background: #fafafa;
      border-left: 3px solid #2e86de;
      padding: 0 16px;
    }
    .dropdown-content.show {
      max-height: 500px;
      padding: 16px;
    }
    .arrow {
      transition: transform 0.3s;
      font-size: 20px;
    }
    .arrow.active {
      transform: rotate(180deg);
    }
  </style>
</head>
<body>
  <div class="card">
    <h1><strong>SAVITAR LFR</strong></h1>
    <div class="status"><b>Status:</b> )rawliteral";

    html += status;
    html += R"rawliteral(</div>)rawliteral";

    // Battery voltage section
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
    
    <div class="button-row">
      <form action="/start" method="post" style="margin: 0;">
        <button class="start" type="submit">Start</button>
      </form>

      <form action="/stop" method="post" style="margin: 0;">
        <button class="stop" type="submit">Stop</button>
      </form>
    </div>

    <form action="/update" method="post">
      <div class="row">
        <label>Kp</label>
        <input name="kp" value=")rawliteral";
    html += String(cfg.kp, 4);
    html += R"rawliteral(" onfocus="this.select()" inputmode="decimal">
      </div>

      <div class="row">
        <label>Ki</label>
        <input name="ki" value=")rawliteral";
    html += String(cfg.ki, 4);
    html += R"rawliteral(" onfocus="this.select()" inputmode="decimal">
      </div>
      
      <div class="row">
        <label>Kd</label>
        <input name="kd" value=")rawliteral";
    html += String(cfg.kd, 4);
    html += R"rawliteral(" onfocus="this.select()" inputmode="decimal">
      </div>

      <div class="row">
        <label>Straight Base Speed</label>
        <input name="straightBase" value=")rawliteral";
    html += String(cfg.straightBase);
    html += R"rawliteral(" onfocus="this.select()" inputmode="decimal">
      </div>

      <!-- Advanced Settings Dropdown -->
      <div class="dropdown-header" onclick="toggleDropdown(event)">
        <span>Advanced Settings</span>
      </div>

      <div class="dropdown-content" id="dropdownContent">
        <div class="row">
          <label>Max Speed</label>
          <input name="maxSpeed" value=")rawliteral";
    html += String(cfg.maxSpeed);
    html += R"rawliteral(" onfocus="this.select()" inputmode="decimal">
        </div>

        <div class="row">
          <label>Min Speed</label>
          <input name="minSpeed" value=")rawliteral";
    html += String(cfg.minSpeed);
    html += R"rawliteral(" onfocus="this.select()" inputmode="decimal">
        </div>

        <div class="row">
          <label>Emergency Threshold</label>
          <input name="emergencyThreshold" value=")rawliteral";
    html += String(cfg.emergencyThreshold);
    html += R"rawliteral(" onfocus="this.select()" inputmode="decimal">
        </div>

        <div class="row">
          <label>Emergency Spin Speed</label>
          <input name="emergencySpin" value=")rawliteral";
    html += String(cfg.emergencySpin);
    html += R"rawliteral(" onfocus="this.select()" inputmode="decimal">
        </div>
      </div>

      <button class="save" type="submit">Apply Changes</button>
    </form>

    <form action="/save" method="post">
      <button class="save" type="submit">Save to Flash</button>
    </form>

    <form action="/defaults" method="post">
      <button class="defaults" type="submit">Load Default Settings</button>
    </form>
  </div>

  <script>
    function toggleDropdown(event) {
      const header = event.currentTarget;
      const content = document.getElementById('dropdownContent');
      const arrow = header.querySelector('.arrow');
      
      header.classList.toggle('active');
      content.classList.toggle('show');
      arrow.classList.toggle('active');
    }
  </script>
</body>
</html>
)rawliteral";

    return html;
}