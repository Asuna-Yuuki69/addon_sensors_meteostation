#include <Arduino.h>

// Resistance to degrees conversion table
struct WindDirection {
  float resistance;
  int degrees;
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

const int sensorPin = 34; // ADC pin for wind vane sensor

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Set ADC resolution to 12 bits
}

void loop() {
  int adcValue = analogRead(sensorPin);
  float voltage = (adcValue / 4095.0) * 3.3; // Convert ADC value to voltage
  float resistance = (3300.0 / voltage) - 1000; // Convert voltage to resistance (assuming a 3.3V reference and 1k pull-down resistor)

  int windDirection = getWindDirection(resistance);
  Serial.print("Wind Direction: ");
  Serial.print(windDirection);
  Serial.println(" degrees");

  delay(1000); // Delay for 1 second
}

int getWindDirection(float resistance) {
  int closestDirection = 0;
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