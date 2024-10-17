# **Blended Project: Temperature and Humidity Monitoring with Temperature-Controlled Fan System for Agriculture Application**

## **Introduction**

This project combines **temperature and humidity monitoring** using a **DHT22 sensor** with a **temperature-controlled fan system**. The fan (or an LED as a substitute) is activated based on temperature readings, making this setup ideal for agricultural environments where managing temperature and humidity is crucial for plant growth and storage conditions.

---

## **Components Required**

- **ESP32 Development Board**
- **DHT22 Temperature and Humidity Sensor**
  - **Temperature Range**: 0°C to 50°C (±2°C accuracy)
  - **Humidity Range**: 20% to 90% (±5% accuracy)
  - **Sampling Rate**: Takes a new reading every 2 seconds
- **1 DC Fan** (or **1 LED** as a substitute)
- **1 Red Status LED**
- **Resistors** (220Ω for LEDs)
- **Breadboard** and **Jumper Wires**
- **USB Cable** (for programming and powering the ESP32)

---

## **Pinout and Connections**

### **1. DHT22 Sensor Connections:**

| **DHT22 Pin** | **ESP32 Pin** |
| ------------- | ------------- |
| OUT           | GPIO 34       |
| VCC           | 3.3V          |
| GND           | GND           |

> **Note**: Ensure that you correctly match the DHT22 sensor pins as they may vary depending on the sensor model.

### **2. Fan or LED (Substitute) Connections:**

| **Component**    | **ESP32 Pin** |
| ---------------- | ------------- |
| Fan Positive     | GPIO 21       |
| Fan Negative     | GND           |
| LED Anode        | GPIO 21 (via 220Ω resistor) |
| LED Cathode      | GND           |

### **3. Status LED Connections:**

| **LED Pin**    | **ESP32 Pin** |
| -------------- | ------------- |
| LED Anode      | GPIO 26 (via 220Ω resistor) |
| LED Cathode    | GND           |

### **4. ESP32 Connection:**

- Use a **USB cable** to connect the ESP32 to your computer for power and programming.

---

## **Setup Instructions**

### **Step 1: Install Required Libraries**

To read temperature and humidity data from the DHT22 sensor, you need to install the **DHT sensor library** by Adafruit.

1. Open **Arduino IDE**.
2. Navigate to **Sketch > Include Library > Manage Libraries**.
3. Search for **DHT sensor library** by Adafruit and install it.
4. Install the **Adafruit Unified Sensor library**, which is a dependency for the DHT library.

---

## **Blended Code: Temperature Monitoring with Fan Control**

```cpp
#include <DHT.h>  // Include DHT library for temperature/humidity sensor

// Pin Definitions
#define DHTPIN 34       // Pin connected to DHT22 sensor
#define DHTTYPE DHT22   // Specify the type of DHT sensor (DHT11 or DHT22)
#define fanPin 21       // Fan connected to GPIO 21
#define statusLedPin 26 // Status LED connected to GPIO 26

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
  if (temperature > 30.0) {  // If temperature exceeds 30°C
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
```

---

## **Code Explanation**

1. **Library Inclusions**:
   - The **`DHT.h`** library is used to interface with the DHT22 sensor for reading temperature and humidity data.
  
2. **Pin Definitions**:
   - **DHTPIN**: Pin where the DHT22 sensor is connected.
   - **fanPin**: Pin where the fan (or LED) is connected.
   - **statusLedPin**: Pin where the status LED is connected.

3. **Setup Function**:
   - **`Serial.begin(115200)`** starts serial communication to display temperature and humidity data on the Serial Monitor.
   - **`pinMode(fanPin, OUTPUT)`** and **`pinMode(statusLedPin, OUTPUT)`** initialize the fan and LED pins as outputs.
   - **`dht.begin()`** initializes the DHT22 sensor for temperature and humidity monitoring.

4. **Main Loop Function**:
   - **`dht.readHumidity()`** and **`dht.readTemperature()`** read the current humidity and temperature values.
   - If the temperature exceeds **30°C**, the fan (or LED) is turned on, and the status LED lights up.
   - If the temperature is below or equal to **30°C**, the fan (or LED) and status LED are turned off.
   - The loop repeats every **2 seconds** to update the readings.

---

## **How to Use**

1. **Connect the ESP32 to your computer** using a USB cable.
2. **Upload the code** to the ESP32 via the Arduino IDE.
3. **Open the Serial Monitor** (set the baud rate to **115200**) to view the temperature and humidity readings in real-time.
4. Observe the **fan** or **LED** activating when the temperature exceeds **30°C**, and turning off when it drops below the threshold.

---

## **Testing the System**

1. **Monitor the Temperature**: Use a heat source (such as a hairdryer) near the DHT22 sensor to increase the temperature.
2. **Observe Fan Behavior**: As the temperature crosses **30°C**, the fan (or substitute LED) should turn on, and the status LED should indicate fan activation.
3. **Monitor Humidity**: Humidity data is also displayed on the Serial Monitor for observation.

---

## **Conclusion**

This blended project demonstrates how to monitor **temperature and humidity** using the **DHT22 sensor** while automatically controlling a fan based on temperature thresholds. This is ideal for **agricultural applications** where maintaining optimal environmental conditions is critical. You can further extend this project by integrating data logging or connecting it to a cloud platform for remote monitoring and control.
