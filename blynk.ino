// TTGO T-Call pin definitions
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

#define BLYNK_PRINT Serial    
#define BLYNK_HEARTBEAT 30
#define TINY_GSM_MODEM_SIM800

#define PHOTOCELL_PIN 34
#define LED_BLUE  13

#include <TinyGsmClient.h>
#include <BlynkSimpleSIM800.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <Wire.h>
#include "utilities.h"

Adafruit_BME280 bme280; // use I2C interface

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Hardware Serial
#define SerialAT Serial1
TinyGsm modem(SerialAT);

const char apn[]  = "internet.eplus.de";
const char user[] = "eplus";
const char pass[] = "internet";

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
const char auth[] = "wqGsknA0-mx3G8CNnolifE8ELm5MUATD";

boolean ledState = LOW;

BlynkTimer timer; // Announcing the timer
BlynkTimer timerLed; // Announcing the timer
int sensorData = 0;
int stoerung = 0;
float temperature = 20;
float humidity = 50;
int temperaturUnter15 = 0;
int temperaturUnter10 = 0;

void myTimerLedEvent()
{
    digitalWrite(LED_BLUE, ledState);
    ledState = !ledState;
}

void myTimerEvent()
{
  temperature = bme280.readTemperature();
  humidity = bme280.readHumidity();
  SerialMon.print(F("Temperature = "));
  SerialMon.print(temperature);
  SerialMon.println(" *C");
  
  if (temperature < 15)
  {
    if (temperaturUnter15 == 0)
    {
      temperaturUnter15 = 1;
      Blynk.email("bbuehner@gmx.net", "Temperatur in Cappel-Neufeld unter 15 Grad Celsius gefallen", "Die Temperatur im Haus in Cappel-Neufeld ist unter 15 Grad Celsius gefallen");
    }
  }
  else
  {
    temperaturUnter15 = 0;
  }

  if (temperature < 10)
  {
    if (temperaturUnter10 == 0)
    {
      temperaturUnter10 = 1;
      Blynk.email("bbuehner@gmx.net", "Temperatur in Cappel-Neufeld unter 10 Grad Celsius gefallen", "Die Temperatur im Haus in Cappel-Neufeld ist unter 10 Grad Celsius gefallen");
    }
  }
  else
  {
    temperaturUnter10 = 0;
  }
  
  SerialMon.print(F("Humidity = "));
  SerialMon.print(humidity);
  SerialMon.println(" %");

  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);

  sensorData = analogRead(PHOTOCELL_PIN);
  SerialMon.print("sensorData: ");
  SerialMon.println(sensorData);
  if (sensorData > 2500)
  {
    if (stoerung == 0)
    {
      Blynk.email("bbuehner@gmx.net", "Störung der Heizung in Cappel-Neufeld", "Die Störungslampe an der Heizung leuchtet");
//      Blynk.notify("Störung der Heizung in Cappel-Neufeld");
    }
    stoerung = 1;
  }
  else
  {
    if (stoerung == 1)
    {
      Blynk.email("bbuehner@gmx.net", "Störung der Heizung in Cappel-Neufeld behoben", "Die Störungslampe an der Heizung leuchtet nicht mehr");
//      Blynk.notify("Störung der Heizung in Cappel-Neufeld behoben");
    }
    stoerung = 0;
  }
  Blynk.virtualWrite(V2, stoerung);
  Blynk.virtualWrite(V3, sensorData);
}

void setup()
{
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  // Keep power when running from battery
  Wire.begin(I2C_SDA, I2C_SCL);
  bool   isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  Serial.print(F("BME280 Sensor event test "));
  if (!bme280.begin(0x76)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1) delay(10);
  }
  Serial.println("OK");

  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, LOW);

  // Set-up modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);

  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  timerLed.setInterval(1000, myTimerLedEvent);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN
  modem.simUnlock("0103");

   SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork(240000L)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

  SerialMon.print(F("Connecting to APN: "));
  SerialMon.println(apn);
  modem.gprsConnect(apn, user, pass);
  SerialMon.println("Modem connected");

  Blynk.begin(auth, modem, apn, user, pass);
  // Setup a function to be called every 5 minutes
  timer.setInterval(1 * 60 * 1000, myTimerEvent);
  SerialMon.println("Blink begin");
}

void loop()
{
  Blynk.run();
  timer.run(); // running timer 
  timerLed.run();
}
