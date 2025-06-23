#include <BLEDevice.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Wi-Fi + MQTT
const char* ssid = "Axio Electronics";
const char* password = "axio2929";
const char* mqtt_server = "eu.thingsboard.cloud";
const char* access_token = "eZA4XwLwvhZLy5xvO7CS";

WiFiClient espClient;
PubSubClient client(espClient);

// BLE
BLEScan* pBLEScan = nullptr;
BLEAdvertisedDevice* targetDevice = nullptr;
BLEClient* pClient = nullptr;
static BLEAddress targetAddress("F4:BB:D7:12:72:EA"); // replace with actual MAC

bool wifiConnected = false;
bool mqttConnected = false;
bool bleConnected = false;
bool doConnect = false;
bool isScanning = false;

unsigned long lastScanTime = 0;
unsigned long scanInterval = 5000; // ms
unsigned long connectStartTime = 0;
const unsigned long CONNECT_TIMEOUT_MS = 5000;

// Buffers
String afePayload, tempPayload, imuPayload;
unsigned long lastMQTTFlush = 0;

// Connect to WiFi
void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi Connected!");
  wifiConnected = true;
}

// Connect to MQTT
void connectMQTT() {
  client.setServer(mqtt_server, 1883);
  while (!client.connected()) {
    Serial.print("Connecting to ThingsBoard...");
    if (client.connect("ESP32Client", access_token, NULL)) {
      Serial.println("✅ MQTT Connected!");
      mqttConnected = true;
    } else {
      Serial.print("❌ MQTT failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// BLE notification callback
static void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  String uuid = pChar->getUUID().toString().c_str();

  if (uuid.indexOf("19b11001") >= 0 && length >= 7) {
    uint16_t spo2 = pData[0] | (pData[1] << 8);
    uint16_t hr = pData[2] | (pData[3] << 8);
    uint8_t hr_conf = pData[4];
    uint16_t hrv_ms = pData[5] | (pData[6] << 8);
    afePayload = String("{\"spo2\":") + spo2 + ",\"hr\":" + hr + ",\"hr_conf\":" + hr_conf + ",\"hrv_ms\":" + hrv_ms + "}";
  }
  else if (uuid.indexOf("19b11002") >= 0 && length >= 11) {
    int16_t tempC = pData[0] | (pData[1] << 8);
    int16_t tempF = pData[2] | (pData[3] << 8);
    int16_t tempAvg = pData[4] | (pData[5] << 8);
    int16_t tempMin = pData[6] | (pData[7] << 8);
    int16_t tempMax = pData[8] | (pData[9] << 8);
    uint8_t tempVar = pData[10];
    tempPayload = String("{\"temp_c\":") + tempC + ",\"temp_f\":" + tempF + ",\"temp_avg\":" + tempAvg +
                  ",\"temp_min\":" + tempMin + ",\"temp_max\":" + tempMax + ",\"temp_var\":" + tempVar + "}";
  }
  else if (uuid.indexOf("19b11003") >= 0 && length >= 11) {
    uint32_t steps = pData[0] | (pData[1] << 8) | (pData[2] << 16) | (pData[3] << 24);
    uint8_t fall = pData[4];
    uint8_t activity = pData[5];
    uint8_t act_conf = pData[6];
    uint16_t accel = pData[7] | (pData[8] << 8);
    uint16_t gyro = pData[9] | (pData[10] << 8);

    const char* activityLabel;
    switch (activity) {
      case 0: activityLabel = "Idle"; break;
      case 1: activityLabel = "Walk"; break;
      case 2: activityLabel = "Run"; break;
      case 3: activityLabel = "Sitting"; break;
      default: activityLabel = "Unknown";
    }

    imuPayload = String("{\"steps\":") + steps + ",\"fall\":" + fall +
                 ",\"activity\":\"" + activityLabel + "\",\"act_conf\":" + act_conf +
                 ",\"accel_mg\":" + accel + ",\"gyro_mdps\":" + gyro + "}";
  }
}

// BLE Scan callback
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getAddress().equals(targetAddress)) {
      Serial.println("✅ Wristband found.");
      targetDevice = new BLEAdvertisedDevice(advertisedDevice);
      pBLEScan->stop();
      isScanning = false;
      doConnect = true;
    }
  }
};

// BLE connect
bool connectToDevice() {
  Serial.println("🔗 Connecting to wristband...");
  pClient = BLEDevice::createClient();
  if (!pClient->connect(targetDevice)) {
    Serial.println("❌ BLE connection failed.");
    pClient->disconnect();
    BLEDevice::deinit(true);
    return false;
  }

  Serial.println("✅ BLE connected. Waiting before discovery...");
  delay(500); // Let device settle

  auto* services = pClient->getServices();
  if (!services || services->empty()) {
    Serial.println("❌ No services found. Disconnecting...");
    pClient->disconnect();
    BLEDevice::deinit(true);
    return false;
  }

  for (auto& s : *services) {
    for (auto& c : *s.second->getCharacteristics()) {
      if (c.second->canNotify()) {
        c.second->registerForNotify(notifyCallback);
      }
    }
  }

  bleConnected = true;
  Serial.println("✅ Notification subscriptions successful.");
  return true;
}

// BLE scan trigger
void startScan() {
  Serial.println("🔍 Scanning for wristband...");
  BLEDevice::deinit(true);  // Reset BLE stack
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(10, false);
  isScanning = true;
  lastScanTime = millis();
}

void setup() {
  Serial.begin(115200);
  connectWiFi();
  connectMQTT();
  startScan();
}

void loop() {
  // MQTT
  if (!client.connected()) connectMQTT();
  client.loop();

  // BLE telemetry flush
  if (millis() - lastMQTTFlush > 3000) {
    if (afePayload.length()) { client.publish("v1/devices/me/telemetry", afePayload.c_str()); afePayload = ""; }
    if (tempPayload.length()) { client.publish("v1/devices/me/telemetry", tempPayload.c_str()); tempPayload = ""; }
    if (imuPayload.length()) { client.publish("v1/devices/me/telemetry", imuPayload.c_str()); imuPayload = ""; }
    lastMQTTFlush = millis();
  }

  // Retry scan if needed
  if (!isScanning && !doConnect && !bleConnected && millis() - lastScanTime > scanInterval) {
    startScan();
    scanInterval = min(scanInterval + 2000UL, 20000UL);  // Ensure both are unsigned long
  // Exponential backoff up to 20s
  }

  // Connect if flagged
  if (doConnect && targetDevice != nullptr && !bleConnected) {
    connectStartTime = millis();
    if (!connectToDevice()) {
      Serial.println("🔁 BLE connect failed. Will rescan.");
      doConnect = false;
      bleConnected = false;
      delay(1000);
      startScan();
    } else {
      doConnect = false;
    }
  }

  // Disconnect check
  if (pClient && bleConnected && !pClient->isConnected()) {
    Serial.println("⚠️ BLE disconnected!");
    bleConnected = false;
    BLEDevice::deinit(true);
    delay(1000);
    startScan();
  }

  // Connection timeout guard
  if (doConnect && millis() - connectStartTime > CONNECT_TIMEOUT_MS) {
    Serial.println("⏱️ BLE connect timeout.");
    doConnect = false;
    startScan();
  }

  delay(50);
}
