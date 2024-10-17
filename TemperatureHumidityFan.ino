#include <DHT.h>  // Include DHT library for temperature/humidity sensor

// Pin Definitions
#define DHTPIN 34       // Pin connected to DHT22 sensor
#define DHTTYPE DHT22   // Specify the type of DHT sensor (DHT11 or DHT22)
#define fanPin 21       // Fan connected to GPIO 21
#define statusLedPin 16 // Status LED connected to GPIO 16

// Create DHT object
DHT dht(DHTPIN, DHTTYPE);  

float temperature;  // Variable to store temperature reading

void setup() {
  Serial.begin(115200);          // Initialize serial communication
  pinMode(fanPin, OUTPUT);       // Set fan pin as output
  pinMode(statusLedPin, OUTPUT); // Set status LED pin as output

  // Start with fan and LED turned OFF
  digitalWrite(fanPin, LOW);
  digitalWrite(statusLedPin, LOW);
  
  dht.begin();  // Initialize DHT sensor

  Serial.println("System Initialized. Monitoring temperature and humidity...");
}

void loop() {
  // Read temperature and humidity from DHT22
  float humidity = dht.readHumidity();  // Read humidity percentage
  temperature = dht.readTemperature();  // Read temperature in Celsius

  // Check if the readings are valid
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    delay(2000);  // Retry after 2 seconds
    return;
  }

  // Display temperature and humidity on Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  // Control fan based on temperature threshold
  if (temperature > 30.0 {  // If temperature exceeds 30°C
    digitalWrite(fanPin, HIGH);       // Turn on the fan
    digitalWrite(statusLedPin, HIGH); // Turn on the status LED
    Serial.println("Fan ON. Status LED ON.");
  } else {
    digitalWrite(fanPin, LOW);        // Turn off the fan
    digitalWrite(statusLedPin, LOW);  // Turn off the status LED
    Serial.println("Fan OFF. Status LED OFF.");
  }

  // Wait before the next reading
  delay(2000);  // Read every 2 seconds
}
