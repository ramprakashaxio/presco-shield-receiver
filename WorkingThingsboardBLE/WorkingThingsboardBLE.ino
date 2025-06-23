    #include <NimBLEDevice.h>
    #include "esp_log.h"
    #include <WiFi.h>
    #include <PubSubClient.h>
    #include <ArduinoJson.h>
    #include <Adafruit_NeoPixel.h>

    struct ConfigFrame {
      uint8_t Temp_Sensor;
      uint8_t IMU_Sensor;
      uint8_t AFE_Sensor;
      uint8_t MAX32664c_Sensor;
      uint8_t haptic_enable;
      uint8_t rgb_enable;
      uint8_t buzzer_enable;
      uint8_t mic_enable;
      uint8_t sos_enable;
      uint8_t max32664c_mode;
      uint16_t sampling_period_ms;
    };

    #define LED_PIN_WS2812 32   // GPIO13 for WS2812B
    #define NUM_LEDS 1          // Change to actual number of WS2812 LEDs
    Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN_WS2812, NEO_GRB + NEO_KHZ800);
    void blinkColor(uint32_t color, int duration = 1000) {
      for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, color);
      }
      strip.show();
      delay(duration);
      strip.clear();
      strip.show();
    }
    void blinkGreenMQTT()  { blinkColor(strip.Color(0, 255, 0)); }
    void blinkBlueBLEData()  { blinkColor(strip.Color(0, 0, 255)); }
    void blinkWhiteConnection() { blinkColor(strip.Color(255, 255, 255)); }
    // Buffers for telemetry
    String afePayload, tempPayload, imuPayload, hubPayload, batteryPayload, configPayload, statusPayload;


    ConfigFrame lastReceivedConfig;
    bool configPendingToSend = false;
    unsigned long lastBLEAttempt = 0;
    bool bleConnected = false;
    bool configReadyToSend = false;
    bool bleInitialized = false;


    const char* ssid = "NeuralBits-A320";
    const char* password = "nb@8369885895";
    const char* mqtt_server = "eu.thingsboard.cloud";
    const int mqtt_port = 1883;
    const char* access_token = "8bk0iuh7lrjyblqibfm8";

    WiFiClient espClient;
    PubSubClient client(espClient);
    
    // ——— Output Pin Definitions ———
    const int HAPTIC_PIN = 15;
    const int LED_PIN = 2;
    const int MIC_PIN = 4;
    const int BUZZER_PIN = 13;

    unsigned long lastMQTTFlush = 0;

    // Use correct constructor with address and address type
    static NimBLEAddress targetAddress("F4:BB:D7:12:72:EA", BLE_ADDR_RANDOM);

    // BLE UUIDs
    static NimBLEUUID svcUUID("19B11010-E8F2-537E-4F6C-D104768A1214");
    static NimBLEUUID chrAFE("19B11001-E8F2-537E-4F6C-D104768A1214");
    static NimBLEUUID chrTEMP("19B11002-E8F2-537E-4F6C-D104768A1214");
    static NimBLEUUID chrIMU("19B11003-E8F2-537E-4F6C-D104768A1214");
    static NimBLEUUID CHAR_HUB("19B11004-E8F2-537E-4F6C-D104768A1214");
    static NimBLEUUID CHAR_BATTERY("19B11005-E8F2-537E-4F6C-D104768A1214");
    static NimBLEUUID CHAR_CONFIG("19B11006-E8F2-537E-4F6C-D104768A1214");

    NimBLEClient*            pClient = nullptr;
    NimBLERemoteCharacteristic* pAFE = nullptr;
    NimBLERemoteCharacteristic* pTEMP = nullptr;
    NimBLERemoteCharacteristic* pIMU = nullptr;
    NimBLERemoteCharacteristic* pHUB = nullptr;
    NimBLERemoteCharacteristic* pBATTERY = nullptr;
    NimBLERemoteCharacteristic* pCONFIG = nullptr;


  void callback(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0';
    Serial.printf("\n📩 %s → %s\n", topic, payload);

        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, hubPayload);
        if (error) {
          Serial.print("❌ JSON Parse Error: ");
          Serial.println(error.c_str());
        }
        if (deserializeJson(doc, payload)) {
            Serial.println("❌ JSON parse failed");
            return;
        }

        const char* method = doc["method"];
        JsonVariant params = doc["params"];
        Serial.printf("🔧 method: %s\n", method);

        if (params.is<bool>()) {
            bool val = params.as<bool>();

            if (strcmp(method,"setTempSensor") == 0) {
                lastReceivedConfig.Temp_Sensor = val;
                Serial.printf(" ↪ setTempSensor → %s\n", val ? "ENABLED" : "DISABLED");
                configPendingToSend = true;
            } else if (strcmp(method,"setIMU") == 0) {
                lastReceivedConfig.IMU_Sensor = val;
                Serial.printf(" ↪ setIMU → %s\n", val ? "ENABLED" : "DISABLED");
                configPendingToSend = true;
            } else if (strcmp(method,"setAFE") == 0) {
                lastReceivedConfig.AFE_Sensor = val;
                Serial.printf(" ↪ setAFE → %s\n", val ? "ENABLED" : "DISABLED");
                configPendingToSend = true;
            } else if (strcmp(method,"setHub") == 0) {
                lastReceivedConfig.MAX32664c_Sensor = val;
                Serial.printf(" ↪ setHub → %s\n", val ? "ENABLED" : "DISABLED");
                configPendingToSend = true;
            } else if (strcmp(method,"setHapticMotor") == 0) {
                lastReceivedConfig.haptic_enable = val;
                digitalWrite(HAPTIC_PIN, val ? HIGH : LOW);
                configPendingToSend = true;
            } else if (strcmp(method,"setLED") == 0) {
                lastReceivedConfig.rgb_enable = val;
                digitalWrite(LED_PIN, val ? HIGH : LOW);
                configPendingToSend = true;
            } else if (strcmp(method,"setBuzzer") == 0) {
                lastReceivedConfig.buzzer_enable = val;
                digitalWrite(BUZZER_PIN, val ? HIGH : LOW);
                configPendingToSend = true;
            } else if (strcmp(method,"setMicrophone") == 0) {
                lastReceivedConfig.mic_enable = val;
                digitalWrite(MIC_PIN, val ? HIGH : LOW);
                configPendingToSend = true;
            } else {
                Serial.printf(" ⚠️ Unhandled bool RPC: %s\n", method);
            }
        } else if (params.is<const char*>()) {
            const char* txt = params.as<const char*>();

            if (strcmp(method,"setSamplingPeriod") == 0) {
                lastReceivedConfig.sampling_period_ms = atoi(txt);
                configPendingToSend = true;
            } else if (strcmp(method,"setHubMode") == 0) {
                lastReceivedConfig.max32664c_mode = atoi(txt);
                configPendingToSend = true;
            } else {
                Serial.printf(" ⚠️ Unhandled string RPC: %s\n", method);
            }
        } else {
            Serial.printf(" ⚠️ Unknown RPC param type for %s\n", method);
        }

    }






    class ClientCallbacks : public NimBLEClientCallbacks {
    public:
      void onConnect(NimBLEClient* pClient) override {
        Serial.println("✅ [Callback] Connected to server.");
      }

      void onDisconnect(NimBLEClient* pClient, int reason) override {
        Serial.printf("⚠️ [Callback] Disconnected. Reason: 0x%02X\n", reason);
      }

      void onConnectFail(NimBLEClient* pClient) {
        Serial.println("❌ [Callback] Failed to connect to server.");
      }
    };

    void notifyCallback(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t length, bool isNotify) {
        String uuid = chr->getUUID().toString().c_str();

        if (uuid.indexOf("19b11001") >= 0 && length >= 7) {
          uint16_t spo2 = data[0] | (data[1] << 8);
          uint16_t hr = data[2] | (data[3] << 8);
          uint8_t hr_conf = data[4];
          uint16_t hrv_ms = data[5] | (data[6] << 8);
          afePayload = String("{\"spo2\":") + spo2 + ",\"hr\":" + hr + ",\"hr_conf\":" + hr_conf + ",\"hrv_ms\":" + hrv_ms + "}";
        }
        else if (uuid.indexOf("19b11002") >= 0 && length >= 11) {
          int16_t tempC = data[0] | (data[1] << 8);
          int16_t tempF = data[2] | (data[3] << 8);
          int16_t tempAvg = data[4] | (data[5] << 8);
          int16_t tempMin = data[6] | (data[7] << 8);
          int16_t tempMax = data[8] | (data[9] << 8);
          uint8_t tempVar = data[10];
          tempPayload = String("{\"temp_c\":") + tempC + ",\"temp_f\":" + tempF + ",\"temp_avg\":" + tempAvg +
                        ",\"temp_min\":" + tempMin + ",\"temp_max\":" + tempMax + ",\"temp_var\":" + tempVar + "}";
        }
        else if (uuid.indexOf("19b11003") >= 0 && length >= 11) {
          uint32_t steps = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
          uint8_t fall = data[4];
          uint8_t activity = data[5];
          uint8_t act_conf = data[6];
          uint16_t accel = data[7] | (data[8] << 8);
          uint16_t gyro = data[9] | (data[10] << 8);

          const char* activityLabel = "Unknown";
          switch (activity) {
            case 0: activityLabel = "Idle"; break;
            case 1: activityLabel = "Walk"; break;
            case 2: activityLabel = "Run"; break;
            case 3: activityLabel = "Sitting"; break;
          }

          imuPayload = String("{\"steps\":") + steps + ",\"fall\":" + fall +
                      ",\"activity\":\"" + activityLabel + "\",\"act_conf\":" + act_conf +
                      ",\"accel_mg\":" + accel + ",\"gyro_mdps\":" + gyro + "}";
        }
        else if (uuid.indexOf("19b11004") >= 0 && length >= 20) {
            // old fields
            uint8_t  mode                = data[0];
            uint16_t hr                  = data[1]  | (data[2] << 8);
            uint16_t hr_conf             = data[3]  | (data[4] << 8);
            uint16_t spo2                = data[5]  | (data[6] << 8);
            uint16_t spo2_conf           = data[7]  | (data[8] << 8);
            uint8_t  activity_class      = data[9];
            uint16_t skin_detect         = data[10] | (data[11] << 8);

            // new fields
            uint16_t RTOR                = data[12] | (data[13] << 8);
            uint8_t  RTOR_conf           = data[14];
            uint8_t  spo2_valid_pct      = data[15];
            uint8_t  spo2_low_quality    = data[16];
            uint8_t  spo2_excessive_motion = data[17];
            uint8_t  spo2_low_pi         = data[18];
            uint8_t  spo2_state          = data[19];

            Serial.println("📡 HUB Frame (extended):");
            Serial.printf("  Mode: %u\n",               mode);
            Serial.printf("  HR: %u (%u%%)\n",           hr, hr_conf);
            Serial.printf("  SpO₂: %u (%u%%)\n",         spo2, spo2_conf);
            Serial.printf("  Activity: %u  Skin: %u\n",   activity_class, skin_detect);
            Serial.printf("  RTOR: %u (%u%%)\n",         RTOR, RTOR_conf);
            Serial.printf("  ValidPct: %u  LowQual: %u  Motion: %u  LowPI: %u  State: %u\n",
                          spo2_valid_pct, spo2_low_quality,
                          spo2_excessive_motion, spo2_low_pi, spo2_state);

            // if you want to build your JSON string:
            hubPayload = String("{") +
              "\"hub_mode\":" + mode +
              ",\"hub_hr\":" + hr +
              ",\"hub_hr_conf\":" + hr_conf +
              ",\"hub_spo2\":" + spo2 +
              ",\"hub_spo2_conf\":" + spo2_conf +
              ",\"hub_activity_class\":" + activity_class +
              ",\"hub_skin_detect\":" + skin_detect +
              ",\"hub_RTOR\":" + RTOR +
              ",\"hub_RTOR_conf\":" + RTOR_conf +
              ",\"hub_spo2_valid_pct\":" + spo2_valid_pct +
              ",\"hub_spo2_low_quality\":" + spo2_low_quality +
              ",\"hub_spo2_excessive_motion\":" + spo2_excessive_motion +
              ",\"hub_spo2_low_pi\":" + spo2_low_pi +
              ",\"hub_spo2_state\":" + spo2_state +
            "}";
        }


        

        else if (uuid.indexOf("19b11005") >= 0 && length >= 5) {
            uint16_t voltage_mv = data[0] | (data[1] << 8);
            uint8_t percentage = data[2];
            uint8_t status = data[3];
            uint8_t error = data[4];
            Serial.println("🔋 Battery Info:");
            Serial.printf("  Voltage: %u mV  Charge: %u%%  Status: 0x%02X  Error: %u\n",
                          voltage_mv, percentage, status, error);

            batteryPayload = String("{\"battery_voltage_mv\":") + voltage_mv + 
                 ",\"battery_percentage\":" + percentage + 
                 ",\"battery_status\":0x" + String(status, HEX) + 
                 ",\"battery_error\":" + error + "}";
        }
          


        else if (uuid.indexOf("19b11006") >= 0 && length >= 9) {
            uint16_t sample_period = data[0] | (data[1] << 8);
            uint8_t max_mode = data[2];
            uint8_t led_drive = data[3];
            uint8_t sensor_mask = data[4];
            uint8_t haptic = data[5];
            uint8_t rgb = data[6];
            uint8_t buzzer = data[7];
            uint8_t mic = data[8];
            Serial.println("⚙️ Config Info:");
            Serial.printf("  Period: %u ms  Mode: %u  LED: %u mA\n  Mask: 0x%02X  Haptic: %u  RGB: %u  Buzzer: %u  Mic: %u\n",
                          sample_period, max_mode, led_drive, sensor_mask, haptic, rgb, buzzer, mic);
            configPayload = String("{\"sample_period_ms\":") + sample_period + 
                ",\"max_mode\":" + max_mode + 
                ",\"led_drive_ma\":" + led_drive + 
                ",\"sensor_mask\":0x" + String(sensor_mask, HEX) + 
                ",\"haptic_enable\":" + haptic + 
                ",\"rgb_enable\":" + rgb + 
                ",\"buzzer_enable\":" + buzzer + 
                ",\"mic_enable\":" + mic + "}";
        }

        blinkBlueBLEData();

          
      }


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

    // ——— Connect to MQTT (ThingsBoard) ———
    void connectMQTT() {
      client.setServer(mqtt_server, mqtt_port);
      while (!client.connected()) {
        Serial.print("Connecting to ThingsBoard...");
        if (client.connect("ESP32Client", access_token, NULL)) {
          Serial.println("✅ MQTT Connected!");
          client.subscribe("v1/devices/me/rpc/request/+");
          Serial.println("🔔 Subscribed to RPC");
        } else {
          Serial.print("❌ MQTT failed, rc=");
          Serial.println(client.state());
          delay(2000);
        }
      }
    }

    bool connectToServer() {
      Serial.print("🔗 Connecting to ");
      Serial.println(targetAddress.toString().c_str());
      bleConnected = true;
      // Always clean previous client
      if (pClient) {
        if (pClient->isConnected()) {
          pClient->disconnect();
        }
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
      }

      pClient = NimBLEDevice::createClient();
      if (!pClient) {
        Serial.println("❌ Failed to create BLE client");
        bleConnected = false;
        return false;
      }

      // Register connection callbacks
      pClient->setClientCallbacks(new ClientCallbacks());

      if (!pClient->connect(targetAddress)) {
        Serial.println("❌ Connect failed");
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        return false;
      }

      NimBLERemoteService* svc = pClient->getService(svcUUID);
      if (!svc) {
        Serial.println("❌ Service not found");
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        return false;
      }

      // Get characteristics with full null checks
      pAFE    = svc->getCharacteristic(chrAFE);
      pTEMP   = svc->getCharacteristic(chrTEMP);
      pIMU    = svc->getCharacteristic(chrIMU);
      pHUB     = svc->getCharacteristic(CHAR_HUB);
      pBATTERY = svc->getCharacteristic(CHAR_BATTERY);
      pCONFIG  = svc->getCharacteristic(CHAR_CONFIG);
      void blinkWhiteConnection();
      if (!pAFE || !pTEMP || !pIMU) {
        Serial.println("❌ One or more characteristics not found");
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        return false;
      }

      if (pAFE->canNotify())                  pAFE->subscribe(true, notifyCallback);
      if (pTEMP->canNotify())                 pTEMP->subscribe(true, notifyCallback);
      if (pIMU->canNotify())                  pIMU->subscribe(true, notifyCallback);
      if (pHUB && pHUB->canNotify())          pHUB->subscribe(true, notifyCallback);
      if (pBATTERY && pBATTERY->canNotify())  pBATTERY->subscribe(true, notifyCallback);
      if (pCONFIG && pCONFIG->canNotify())    pCONFIG->subscribe(true, notifyCallback);

      Serial.println("🎉 Notifications enabled!");
      bleConnected = true;

      return true;
    }

    void sendConfigToBand(ConfigFrame cfg) {
        if (!pCONFIG || !pCONFIG->canWrite()) {
            Serial.println("❌ Cannot write to Config characteristic.");
            return;
        }

        uint8_t buf[12];
        buf[0] = cfg.Temp_Sensor;
        buf[1] = cfg.IMU_Sensor;
        buf[2] = cfg.AFE_Sensor;
        buf[3] = cfg.MAX32664c_Sensor;
        buf[4] = cfg.haptic_enable;
        buf[5] = cfg.rgb_enable;
        buf[6] = cfg.buzzer_enable;
        buf[7] = cfg.mic_enable;
        buf[8] = cfg.sos_enable;
        buf[9] = cfg.max32664c_mode;
        buf[10] = cfg.sampling_period_ms & 0xFF;
        buf[11] = (cfg.sampling_period_ms >> 8) & 0xFF;

        if (pCONFIG->writeValue(buf, sizeof(buf), false)) {
            Serial.println("✅ Sent config to wristband via BLE.");
        } else {
            Serial.println("❌ Failed to send config over BLE.");
        }
        Serial.println("📥 Final Config to Send via BLE:");
        Serial.printf("  Temp_Sensor: %d\n  IMU_Sensor: %d\n  AFE_Sensor: %d\n  MAX32664c_Sensor: %d\n",
                      cfg.Temp_Sensor, cfg.IMU_Sensor, cfg.AFE_Sensor, cfg.MAX32664c_Sensor);
        Serial.printf("  haptic_enable: %d\n  rgb_enable: %d\n  buzzer_enable: %d\n  mic_enable: %d\n",
                      cfg.haptic_enable, cfg.rgb_enable, cfg.buzzer_enable, cfg.mic_enable);
        Serial.printf("  sos_enable: %d\n  max32664c_mode: %d\n  sampling_period_ms: %d\n",
                      cfg.sos_enable, cfg.max32664c_mode, cfg.sampling_period_ms);

    }

    void ensureMQTTConnected() {
      if (!client.connected()) {
        Serial.print("🔄 Attempting MQTT reconnect...");
        if (client.connect("ESP32Client", access_token, NULL)) {
          Serial.println("✅ MQTT Reconnected!");
          client.subscribe("v1/devices/me/rpc/request/+");
        } else {
          Serial.print("❌ MQTT reconnect failed. rc=");
          Serial.println(client.state());
        }
      }
    }

    void setup() {
      Serial.begin(115200);
      strip.begin();
      strip.setBrightness(50);  // Optional: Set brightness (0–255)
      strip.show();             // Initialize all LEDs to 'off'
      pinMode(HAPTIC_PIN, OUTPUT);
      pinMode(LED_PIN, OUTPUT);
      pinMode(MIC_PIN, OUTPUT);
      pinMode(BUZZER_PIN, OUTPUT);

      digitalWrite(HAPTIC_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
      digitalWrite(MIC_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);

      connectWiFi();
      blinkWhiteConnection();  // Indicate WiFi connection
      client.setCallback(callback); // MQTT receive handler
      connectMQTT();
      blinkWhiteConnection();  // Indicate MQTT connected
      Serial.println("=== ESP32 BLE Client (NimBLE) ===");
      esp_log_level_set("*", ESP_LOG_DEBUG);
      NimBLEDevice::init("");
      NimBLEDevice::setSecurityAuth(false, false, false);
      NimBLEDevice::setPower(ESP_PWR_LVL_P9);

      if (!connectToServer()) {
        Serial.println("🔁 Initial BLE connection failed – retrying...");
      }


      
    }

 void loop() {
      client.loop();
      ensureMQTTConnected();

      // if (!client.connected()) {
      //   connectMQTT();
      // }

      // if (pClient && !pClient->isConnected()) {
      //   Serial.println("⚠️ Disconnected – attempting reconnect...");
      //   delay(3000);
      //   connectToServer();
      // } 
      // else if (pClient->isConnected() && configPendingToSend) {
      //   Serial.println("🚀 BLE connected. Sending stored config...");
      //   sendConfigToBand(lastReceivedConfig);
      //   configPendingToSend = false;
      // }
      // Try BLE connect every 10 seconds if not already connected
      if (!bleConnected && millis() - lastBLEAttempt > 10000) {
        Serial.println("🔁 Trying to connect to BLE device...");
        bleConnected = connectToServer();
        lastBLEAttempt = millis();
      }

      if (bleConnected && configPendingToSend) {
        Serial.println("🚀 BLE connected. Sending stored config...");
        sendConfigToBand(lastReceivedConfig);
        configPendingToSend = false;
      }
    // Update BLE and WiFi status (1 = connected, 0 = not connected)
    statusPayload = String("{\"wi-fi_connected\":") + (WiFi.status() == WL_CONNECTED ? 1 : 0) +
                    ",\"bluetooth_connected\":" + (bleConnected ? 1 : 0) + "}";

      if (millis() - lastMQTTFlush > 3000) {
        if (afePayload.length()) {
          client.publish("v1/devices/me/telemetry", afePayload.c_str());
          Serial.println("📤 Sent AFE Data:");
          Serial.println(afePayload);
          afePayload = "";
        }

        if (tempPayload.length()) {
          client.publish("v1/devices/me/telemetry", tempPayload.c_str());
          Serial.println("📤 Sent Temp Data:");
          Serial.println(tempPayload);
          tempPayload = "";
        }

        if (imuPayload.length()) {
          client.publish("v1/devices/me/telemetry", imuPayload.c_str());
          Serial.println("📤 Sent IMU Data:");
          Serial.println(imuPayload);
          imuPayload = "";
        }

        if (hubPayload.length()) {
          client.publish("v1/devices/me/telemetry", hubPayload.c_str());
          Serial.println("📤 Sent HUB Data:");
          Serial.println(hubPayload);
          hubPayload = "";
        }

        if (batteryPayload.length()) {
          client.publish("v1/devices/me/telemetry", batteryPayload.c_str());
          Serial.println("📤 Sent Battery Data:");
          Serial.println(batteryPayload);
          batteryPayload = "";
        }

        if (configPayload.length()) {
          client.publish("v1/devices/me/telemetry", configPayload.c_str());
          Serial.println("📤 Sent Config Data:");
          Serial.println(configPayload);
          configPayload = "";
        }

        if (statusPayload.length()) {
          client.publish("v1/devices/me/telemetry", statusPayload.c_str());
          Serial.println("📤 Sent Status Data:");
          Serial.println(statusPayload);
          statusPayload = "";
        }
        
        blinkGreenMQTT();  // ✅ Indicate data was sent to MQTT

        lastMQTTFlush = millis();
      }

      delay(100);
    }
