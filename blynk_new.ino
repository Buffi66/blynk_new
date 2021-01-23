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

#define uS_TO_S_FACTOR 1000000  //Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  1800        //Time ESP32 will go to sleep (in seconds)

#include <TinyGsmClient.h>
#include <BlynkSimpleSIM800.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <esp_task_wdt.h>

#include <Wire.h>
#include "utilities.h"

Adafruit_BME280 bme280; // use I2C interface

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Hardware Serial
#define SerialAT Serial1
TinyGsm modem(SerialAT);

//Alditalk
const char apn[]  = "internet.eplus.de";
const char user[] = "eplus";
const char pass[] = "internet";
//Lidl Connect
//const char apn[]  = "web.vodafone.de";
//const char user[] = "guest";
//const char pass[] = "guest";

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
const char auth[] = "wqGsknA0-mx3G8CNnolifE8ELm5MUATD";

int wdtTimeout = 10 * 60; // 10 Minuten Watchdog Timeout

boolean ledState = LOW;

int sensorData = 0;
int stoerung = 0;
float temperature = 20;
float humidity = 50;

RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : SerialMon.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : SerialMon.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : SerialMon.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : SerialMon.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : SerialMon.println("Wakeup caused by ULP program"); break;
    default : SerialMon.println("Wakeup was not caused by deep sleep."); break;
  }
}

void myTimerEvent()
{
  temperature = bme280.readTemperature();
  humidity = bme280.readHumidity();
  SerialMon.print(F("Temperature = "));
  SerialMon.print(temperature);
  SerialMon.println(" *C");
  
  SerialMon.print(F("Humidity = "));
  SerialMon.print(humidity);
  SerialMon.println(" %");

  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);

  sensorData = analogRead(PHOTOCELL_PIN);
  SerialMon.print("sensorData: ");
  SerialMon.println(sensorData);
  Blynk.virtualWrite(V10, bootCount);
  Blynk.virtualWrite(V3, sensorData);
}

void setup()
{
  ++bootCount;                    //Increment boot number and print it every reboot
  // Set console baud rate
  SerialMon.begin(115200);
  delay(1000);
  SerialMon.println("Boot number: " + String(bootCount));
  print_wakeup_reason();          //Print wakeup Reason
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  SerialMon.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
  SerialMon.print("Watchdog timeout = ");
  SerialMon.print(wdtTimeout/60);
  SerialMon.println(" minutes.");
  esp_task_wdt_init(wdtTimeout, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch


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
  SerialMon.println("Blink begin");
  myTimerEvent();
  
  //Go to sleep now
  SerialMon.println("Going to sleep now");
  delay(5000);
  esp_deep_sleep_start();
  SerialMon.println("This will never be printed");
}

void loop()
{
}
