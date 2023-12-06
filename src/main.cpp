#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "EmonLib.h" // Energy Monitoring library to calculate RMS current from SCT-013-000 split current transformer
#include <WiFi.h> // Main library for WiFi connectivity, also used by AsyncWebServer.
#include "Husarnet.h" // IPV6 for ESP32 to enable peer-to-peer communication between devices inside a Husarnet network.
#include "ESPAsyncWebServer.h" // Make sure to include Husarnet before this.
#include <ESPmDNS.h> // Allows to resolve hostnames to IP addresses within a local network.
#include "AsyncElegantOTA.h" // Over the air updates for the ESP32.
#include "ESPConnect.h" // ESPConnect library for the web serve
struct SystemData {
	double current;
};

SystemData systemData;
TaskHandle_t wifiConnectionTaskHandle = nullptr;
TaskHandle_t ledBlinkerTaskHandle = nullptr;

//Event group for task synchronization
EventGroupHandle_t system_events;

#define WIFI_CONNECTED_BIT BIT0
#define VPN_CONNECTED_BIT BIT1
#define SERVER_CONNECTED_BIT BIT2

AsyncWebServer server(80);

enum BlinkRate : uint32_t {
    Slow = 2000,
    Medium = 1000,
    Fast = 300,
    Pulse = 100 // Pulse is a special value that will make the LED blink fast and then return to the previous blink rate.
};

// Tasks can send notifications here to change the blink rate of the LED in order to communicate the status of the boat.
void FastBlinkPulse(int pin);
void LedBlinkerTask(void* parameter) {

    constexpr uint8_t led_pin = GPIO_NUM_2; // Built-in LED pin for the ESP32 DevKit board.
    pinMode(led_pin, OUTPUT);
    uint32_t blink_rate = BlinkRate::Slow;
    uint32_t previous_blink_rate = blink_rate;

    auto FastBlinkPulse = [](int pin) {
        for (int i = 0; i < 4; i++) {
            digitalWrite(pin, HIGH); vTaskDelay(pdMS_TO_TICKS(50));
            digitalWrite(pin, LOW);  vTaskDelay(pdMS_TO_TICKS(50));
        }
    };

    while (true) {

        static uint32_t previous_blink_time = millis();
        if (millis() - previous_blink_time > blink_rate) {
            previous_blink_time = millis();
            digitalWrite(led_pin, !digitalRead(led_pin));
        }
           
        // Set blink rate to the value received from the notification
        static uint32_t received_value = BlinkRate::Slow;
        if (xTaskNotifyWait(0, 0, (uint32_t*)&received_value, 100)) {
            if (received_value == BlinkRate::Pulse) {
                FastBlinkPulse(led_pin);
            } else {
                blink_rate = received_value;
            }
        }
    }
}


void NetworkConnectionTask(void* parameter) {
    
    //Configure SSID and password for configuration portal
    ESPConnect.autoConnect("", "", 180000);

    /* 
    Begin connecting to previous WiFi
    or start autoConnect AP if unable to connect
    */
    if(ESPConnect.begin(&server)){
        Serial.println("Connected to WiFi");
        Serial.println("IPAddress: " + WiFi.localIP().toString());
        xEventGroupSetBits(system_events, WIFI_CONNECTED_BIT);
    } else {
        Serial.println("Failed to connect to WiFi");
    }


    server.on("/", HTTP_GET, [&](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Medidor de corrente");
    });

	server.on("/current", HTTP_GET, [&](AsyncWebServerRequest *request) {
		request->send(200, "text/plain", String(systemData.current));
	});

	//Setup mdns service
	if (!MDNS.begin("medidor")) {
		Serial.println("Error setting up MDNS responder!");
	} else {
		Serial.println("mDNS responder started");
	}

	AsyncElegantOTA.begin(&server);    // Start ElegantOTA
    server.begin();
    
    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            xEventGroupClearBits(system_events, WIFI_CONNECTED_BIT);
            
            xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Fast, eSetValueWithOverwrite);
            if(ESPConnect.begin(&server)){
                Serial.println("Connected to WiFi");
                Serial.println("IPAddress: "+WiFi.localIP().toString());
                xEventGroupSetBits(system_events, WIFI_CONNECTED_BIT);
                xTaskNotify(ledBlinkerTaskHandle, BlinkRate::Slow, eSetValueWithOverwrite);
            } else {
                Serial.println("Failed to connect to WiFi");
            }
        }

        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


/// @brief Calibrates a reading by using a linear equation obtained by comparing the readings with a multimeter.
/// @param input 
/// @param slope 
/// @param intercept 
/// @return Calibrated reading
float LinearCorrection(const float input_value, const float slope, const float intercept) {
    return slope * input_value + intercept;
}

/// @brief Measures the current and voltage of the freezer to get the power consumption.
void PowerMeasurerTask(void *parameter) {

    // Measure current using the SCT-013-000 split current transformer
    // The SCT-013-000 outputs a voltage that is proportional to the current flowing through the wire that passes through the center of the transformer.
    // The voltage is then converted to a current using the calibration value of the transformer. 
    // A trimmer potentiometer is used to correct the shape of the sine wave to be as close as possible to a perfect sine wave on the output signal.
    
    EnergyMonitor measurer; // Create an instance of the EnergyMonitor class
    constexpr uint8_t current_sensor_pin = GPIO_NUM_34; // VP GPIO pin used to read the voltage from the current sensor
    constexpr int number_turns_secondary_coil = 2000; // Number of turns in the secondary coil of the current sensor
    constexpr int burden_resistor_value = 22;
    constexpr float current_sensor_calibration = (float) number_turns_secondary_coil / burden_resistor_value;



    // Calibrate the current sensor by comparing the readings with a multimeter
    measurer.current(current_sensor_pin, current_sensor_calibration); // Sets input pin and calibration factor for current sensor
    while (true) {

        constexpr int number_loops = 1; // Number of times wire passes through the center of the current sensor
        constexpr int sample_weight = 4;
        
        static double filtered_current = 0.0f;
		double current = measurer.calcIrms(1480) / number_loops; // Calculate Irms only
        filtered_current = (sample_weight * filtered_current + current) / (sample_weight + 1);
		double calibrated_current = LinearCorrection(filtered_current, 1.00f, 0.0f);
		systemData.current = calibrated_current;

        static unsigned long previous_time = millis();
        if (millis() - previous_time > 1000) {
            previous_time = millis();
            Serial.printf("\nCurrent: %.2fA\t"
                          "Calibrated current: %.2fA\n", 
                          current, calibrated_current);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
	Serial.begin(115200);
	Serial.println("Starting HouseMeasurer");
	system_events = xEventGroupCreate();
    xTaskCreate(LedBlinkerTask, "LedBlinkerTask", 10000, NULL, 1, &ledBlinkerTaskHandle);
    xTaskCreate(NetworkConnectionTask, "NetworkConnectionTask", 10000, NULL, 1, &wifiConnectionTaskHandle);
	xTaskCreate(PowerMeasurerTask, "PowerMeasurerTask", 4096, NULL, 1, NULL);

}

void loop() {

}
