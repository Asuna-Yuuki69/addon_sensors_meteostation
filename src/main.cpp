#include <Arduino.h>
#include <DHT.h>
#include <esp_mac.h>
#include <WiFi.h>
#include <PubSubClient.h> // MQTT library
#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
#include "utilities.h"

XPowersPMU  PMU;

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

#define TINY_GSM_RX_BUFFER 1024
#define TINY_GSM_MODEM_SIM7080  // Define the modem you're using
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(Serial1, Serial);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif
//co to kurva je pls help ??
const char *register_info[] = {
    "Not registered, MT is not currently searching an operator to register to.The GPRS service is disabled, the UE is allowed to attach for GPRS if requested by the user.",
    "Registered, home network.",
    "Not registered, but MT is currently trying to attach or searching an operator to register to. The GPRS service is enabled, but an allowable PLMN is currently not available. The UE will start a GPRS attach as soon as an allowable PLMN is available.",
    "Registration denied, The GPRS service is disabled, the UE is not allowed to attach for GPRS if it is requested by the user.",
    "Unknown.",
    "Registered, roaming.",
};
//konec amogus zpravy 
enum {
    MODEM_CATM = 1,
    MODEM_NB_IOT,
    MODEM_CATM_NBIOT,
};

//gprs creds  type shiii
const char apn[] = "lpwa.vodafone.com";
const char gprsUser[] = "easy";
const char gprsPass[] = "connect";

//mqtt connect creds ifk 
const char server[]   = "xtomi.czella.net";
const int  port       = 64525;
char buffer[1024] = {0};
char username[] = "meteo";
char password[] = "Meteo123..";
char clientID[] = "meteoespdomecek";
int data_channel = 0;
const int randMax = 100; // Define a maximum random value
const int randMin = 0;   // Define a minimum random value

bool isConnect()
{
    modem.sendAT("+SMSTATE?");
    if (modem.waitResponse("+SMSTATE: ")) {
      String res = modem.stream.readStringUntil('\r');
      return res.toInt();

    }
    return false;
}
#define DHTPIN 10     // what pin we're connected to
#define DHTTYPE DHT22   // DHT22
#define ANEMOMETER_PIN 13 // The pin connected to the anemometer signal
volatile unsigned int pulseCount = 0; // Counter for pulses
float windSpeed = 0.0; // Calculated wind speed (m/s or other units depending on calibration)
unsigned long lastMillis = 0; // Track the last time we calculated wind speed
const float CALIBRATION_FACTOR = 0.5; // Adjust based on your anemometer (e.g., pulses per m/s)
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor for normal 16mhz Arduino


//const char* ssid = "iot";
//const char* password = "fe801240f3fffe7c";
//const char* mqttServer = "xtomi.czella.net";
//const int mqttPort = 64525;
//const char* mqttUser = "meteo";
//const char* mqttPassword = "Meteo123.."; // No password

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
 //vane caculation constants
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



const int sensorPin = 03; // ADC pin for wind vane sensor

void setup() 
{

  Serial.begin(115200);
  while (!Serial);

  delay(3000);
  Serial.println();
  //power chip and modem initialization
  if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
      Serial.println("power init failed :(");
      while (1) {
          delay(5000);
      }
  }

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED ) {
      PMU.disableDC3();

      delay(200);
  }

  PMU.setDC3Voltage(3000);
  PMU.enableDC3();



  PMU.disableTSPinMeasure();

  Serial1.begin(115200, SERIAL_8N1, BOARD_MODEM_RXD_PIN, BOARD_MODEM_TXD_PIN);
  pinMode(BOARD_MODEM_PWR_PIN, OUTPUT);
  pinMode(BOARD_MODEM_DTR_PIN, OUTPUT);
  pinMode(BOARD_MODEM_RI_PIN, INPUT);
  int retry = 0;
  while (!modem.testAT(1000)) {
      Serial.print(".");
      if (retry++ > 6) {
          // Pull down PWRKEY for more than 1 second according to manual requirements
          digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
          delay(100);
          digitalWrite(BOARD_MODEM_PWR_PIN, HIGH);
          delay(1000);
          digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
          retry = 0;
          Serial.println("Retry start modem .");
      }
  }
  Serial.println();
  Serial.print("Modem started!");

  String result ;
  

  if (modem.getSimStatus() != SIM_READY) {
      Serial.println("SIM Card is not insert!!!");
        return ;
  }
  modem.setNetworkMode(2);    //use automatic

  modem.setPreferredMode(MODEM_NB_IOT);

  uint8_t pre = modem.getPreferredMode();

  uint8_t mode = modem.getNetworkMode();

  Serial.printf("getNetworkMode:%u getPreferredMode:%u\n", mode, pre);

      SIM70xxRegStatus s;
    do {
        s = modem.getRegistrationStatus();
        if (s != REG_OK_HOME && s != REG_OK_ROAMING) {
            Serial.print(".");
            delay(1000);
        }

    } while (s != REG_OK_HOME && s != REG_OK_ROAMING) ;

    Serial.println();
    Serial.print("Network register info:");
    Serial.println(register_info[s]);

    bool res = modem.isGprsConnected();
    if (!res) {
        modem.sendAT("+CNACT=0,1");
        if (modem.waitResponse() != 1) {
            Serial.println("Activate network bearer Failed!");
            return;
        }
        // if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        //     return ;
        // }
    }

    Serial.print("GPRS status:");
    Serial.println(res ? "connected" : "not connected");


    /*********************************
    * step 6 : setup MQTT Client
    ***********************************/
 
    // If it is already connected, disconnect it first
    modem.sendAT("+SMDISC");
    modem.waitResponse();


    snprintf(buffer, 1024, "+SMCONF=\"URL\",\"%s\",%d", server, port);
    modem.sendAT(buffer);
    if (modem.waitResponse() != 1) {
        return;
    }
    snprintf(buffer, 1024, "+SMCONF=\"USERNAME\",\"%s\"", username);
    modem.sendAT(buffer);
    if (modem.waitResponse() != 1) {
        return;
    }

    snprintf(buffer, 1024, "+SMCONF=\"PASSWORD\",\"%s\"", password);
    modem.sendAT(buffer);
    if (modem.waitResponse() != 1) {
        return;
    }

    snprintf(buffer, 1024, "+SMCONF=\"CLIENTID\",\"%s\"", clientID);
    modem.sendAT(buffer);
    if (modem.waitResponse() != 1) {
        return;
    }
    int8_t ret;
    do {

        modem.sendAT("+SMCONN");
        ret = modem.waitResponse(30000);
        if (ret != 1) {
            Serial.println("Connect failed, retry connect ..."); delay(1000);
        }

    } while (ret != 1);

    Serial.println("MQTT Client connected!");

    // random seed data
    randomSeed(esp_random());

 
 
  //Initialize the DHT sensor
  dht.begin();
  delay(5000);

  pinMode(ANEMOMETER_PIN, INPUT_PULLUP); // Set the pin as input with pullup resistor
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), countPulse, FALLING); // Interrupt on falling edge

  analogReadResolution(12); // Set ADC resolution to 12 bits
}

void loop() 
{ 
    if (!isConnect()) {
        Serial.println("MQTT Client disconnect!"); delay(1000);
        return ;
    }
    Serial.println();

    // Create payload for temperature and humidity
    String payload = "temp,c=";
    int temp =  rand() % (randMax - randMin) + randMin;
    payload.concat(temp);
    payload.concat(",humidity,percent=");
    int humidity = rand() % 100; // Replace with actual humidity reading
    payload.concat(humidity);
    payload.concat("\r\n");

    // Publish payload
    snprintf(buffer, 1024, "+SMPUB=\"v1/%s/things/%s/data/%d\",%d,1,1", username, clientID, data_channel, payload.length());
    modem.sendAT(buffer);
    if (modem.waitResponse(">") == 1) {
        modem.stream.write(payload.c_str(), payload.length());
        Serial.print("Try publish payload: ");
        Serial.println(payload);

        if (modem.waitResponse(3000)) {
            Serial.println("Send Packet success!");
        } else {
            Serial.println("Send Packet failed!");
        }
    }

    delay(60000);

    float converted = 0.00;
    unsigned long currentMillis = millis();

    // Calculate wind speed every second (1000 ms)
    if (currentMillis - lastMillis >= 1000) {
        detachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN)); // Temporarily disable interrupt

        // Calculate wind speed in desired units
        windSpeed = (pulseCount / CALIBRATION_FACTOR);

        // Create payload for wind speed
        String windPayload = "wind_speed,m/s=";
        windPayload.concat(windSpeed);
        windPayload.concat("\r\n");

        // Publish wind speed payload
        snprintf(buffer, 1024, "+SMPUB=\"v1/%s/things/%s/data/%d\",%d,1,1", username, clientID, data_channel, windPayload.length());
        modem.sendAT(buffer);
        if (modem.waitResponse(">") == 1) {
            modem.stream.write(windPayload.c_str(), windPayload.length());
            Serial.print("Try publish wind payload: ");
            Serial.println(windPayload);

            if (modem.waitResponse(3000)) {
                Serial.println("Send Packet success!");
            } else {
                Serial.println("Send Packet failed!");
            }
        }

        // Reset pulse count and timestamp
        pulseCount = 0;
        lastMillis = currentMillis;
    }
}