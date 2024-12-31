#include <Arduino.h>
#include <DHT.h>
#include <esp_mac.h>
#include <WiFi.h>
#include <PubSubClient.h> // MQTT library

//Constants
#define DHTPIN 10     // what pin we're connected to
#define DHTTYPE DHT22   // DHT22
#define ANEMOMETER_PIN 13 // The pin connected to the anemometer signal
volatile unsigned int pulseCount = 0; // Counter for pulses
float windSpeed = 0.0; // Calculated wind speed (m/s or other units depending on calibration)
unsigned long lastMillis = 0; // Track the last time we calculated wind speed
const float CALIBRATION_FACTOR = 0.5; // Adjust based on your anemometer (e.g., pulses per m/s)

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor for normal 16mhz Arduino
const char* ssid = "iot";
const char* password = "fe801240f3fffe7c";
const char* mqttServer = "xtomi.czella.net";
const int mqttPort = 64525;
const char* mqttUser = "meteo";
const char* mqttPassword = "Meteo123.."; // No password

WiFiClient espClient;
PubSubClient client(espClient);

//Variables
float hum;  //Stores humidity value
float temp; //Stores temperature value
unsigned long lastMqttPublishMillis = 0; // Track the last time we published MQTT data

// Resistance to degrees conversion table
struct WindDirection {
  float resistance;
  float degrees;
};

WindDirection windDirections[] = {
  {33000, 0},
  {6570, 22.5},
  {8200, 45},
  {891, 67.5},
  {1000, 90},
  {688, 112.5},
  {2200, 135},
  {1410, 157.5},
  {3900, 180},
  {3140, 202.5},
  {16000, 225},
  {14120, 247.5},
  {120000, 270},
  {42120, 292.5},
  {64900, 315},
  {21880, 337.5}
};

float getWindDirection(float resistance) {
  float closestDirection = 0;
  float smallestDifference = abs(resistance - windDirections[0].resistance);

  for (int i = 1; i < sizeof(windDirections) / sizeof(WindDirection); i++) {
    float difference = abs(resistance - windDirections[i].resistance);
    if (difference < smallestDifference) {
      smallestDifference = difference;
      closestDirection = windDirections[i].degrees;
    }
  }

  return closestDirection;
}

void countPulse() {
  pulseCount++; // Increment pulse counter for each interrupt
}

void connectToWiFi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nwifiok");
  Serial.print("IP:");
  Serial.println(WiFi.localIP());
}

void connectToMQTT() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed MQTT connection, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
}

const int sensorPin = 03; // ADC pin for wind vane sensor

void setup() 
{
  Serial.begin(9600);
  //Initialize the DHT sensor
  dht.begin();
  delay(5000);

  pinMode(ANEMOMETER_PIN, INPUT_PULLUP); // Set the pin as input with pullup resistor
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), countPulse, FALLING); // Interrupt on falling edge

  connectToWiFi();
  delay(10000);

  client.setServer(mqttServer, mqttPort);
  connectToMQTT();

  analogReadResolution(12); // Set ADC resolution to 12 bits
}

void loop() 
{ 
    float converted = 0.00;
    unsigned long currentMillis = millis();

   // Calculate wind speed every second (1000 ms)
    if (currentMillis - lastMillis >= 1000) {
      detachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN)); // Temporarily disable interrupt

      // Calculate wind speed in desired units
      windSpeed = (pulseCount / CALIBRATION_FACTOR);

      // Print the wind speed
      Serial.print("Wind Speed: ");
      Serial.print(windSpeed);
      Serial.println(" m/s");

      // Reset pulse count and timestamp
      pulseCount = 0;
      lastMillis = currentMillis;

      attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), countPulse, FALLING); // Re-enable interrupt
    }
    


    int adcValue = analogRead(sensorPin);
    float voltage = (adcValue / 4095.0) * 3.3; // Convert ADC value to voltage
    float resistance = (3300.0 / voltage) - 1000; // Convert voltage to resistance (assuming a 3.3V reference and 1k pull-down resistor)

    float windDirection = getWindDirection(resistance);
    Serial.print("Wind Direction: ");
    Serial.print(windDirection);
    Serial.println(" degrees");

    delay(1000); // Delay for 1 second
    // Read data and store it to variables hum and temp
    hum = dht.readHumidity();
    temp= dht.readTemperature();

    Serial.print("Celsius = ");
    Serial.print(temp);
    // Print degree symbol
    Serial.write(176); 
    Serial.println("C");

    // Kelvin
    // T(K) = T(°C) + 273.15          
    converted = temp + 273.15;
    Serial.print("Kelvin = ");
    Serial.print(converted);
    Serial.println(" K");

    // Print degree symbol
    Serial.write(176);    
    Serial.println("R");

    Serial.print("Humidity =");
    Serial.println(hum);

    // Publish to MQTT every 30 seconds
    if (currentMillis - lastMqttPublishMillis >= 30000) {
      if (!client.connected()) {
        connectToMQTT();
      }

      client.loop(); // Maintain MQTT connection

      char tempStr[8];
      char humStr[8];
      char windStr[8];

      dtostrf(temp, 6, 2, tempStr); // Convert temperature to string
      dtostrf(hum, 6, 2, humStr);   // Convert humidity to string
      dtostrf(windSpeed, 6, 2, windStr); // Convert wind speed to string

      client.publish("meteo/temperature", tempStr);
      client.publish("meteo/humidity", humStr);
      client.publish("meteo/windSpeed", windStr);
      client.publish("meteo/windDirection", String(windDirection).c_str());

      Serial.println("MQTT Data Published");

      lastMqttPublishMillis = currentMillis;
    }

    // 2000ms delay between reads
    delay(2000);

}