/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <MKRWAN.h>
#include "secrets.h"
#include "ArduinoLowPower.h"
#include <Wire.h>
#include <Adafruit_SHT31.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
// num of attempts to join the lora network
#define MAX_ATTEMPTS 5

// waiting time between two join attempts in milliseconds
#define ATTEMPT_DELAY 30000

// waiting time before reset and new attempt in milliseconds
#define RESET_DELAY 600000

// sleep time between two measurements minutes
#define MEASUREMENT_TIMEOUT 2

// sleep time between tx and rx in seconds
#define RX_TIMEOUT 10

// minimum sending interval in minutes
#define MIN_SEND_INTERVAL 120

// temperature hysteresis
#define TEMP_HYSTERESIS 0.5

/*******************************************************************************
 * Pin Definitions
 ******************************************************************************/
#define PIN_RGB_R 8
#define PIN_RGB_G 10
#define PIN_RGB_B 7
#define PIN_RELAIS_1 3
#define PIN_RELAIS_2 2

/*******************************************************************************
 * Object Declarations
 ******************************************************************************/

LoRaModem modem;
Adafruit_SHT31 sht = Adafruit_SHT31();

/*******************************************************************************
 * Functions
 ******************************************************************************/

void (*resetFunc)(void) = 0;  // create a standard reset function

/**
 * \brief resets the board in RESET_DELAY ms time
 */
void delayReset(void) {
  Serial1.println("The board will be resetted soon");
  delay(RESET_DELAY);
  resetFunc();
}

/**
 * \brief lets inside led shine in a color
 * \param red brightness of red 0-255
 * \param green brightness of green 0-255
 * \param blue brightness of blue 0-255
 */
void setRgbLed(uint8_t red, uint8_t green, uint8_t blue) {
  red = map(red, 0, 255, 255, 0);
  green = map(green, 0, 255, 255, 0);
  blue = map(blue, 0, 255, 255, 0);
  analogWrite(PIN_RGB_R, red);
  analogWrite(PIN_RGB_G, green);
  analogWrite(PIN_RGB_B, blue);
}

/**
 * \brief converts data into sendable buffer
 * \param temp temperature value
 * \param humi humidity value
 * \param relais1 state of relais 1
 * \param relais2 state of relais2
 * \param buf address to buffer of min size 4
 */
void convertTxData(float temp, float humi, bool relais1, bool relais2, uint8_t* buf) {
  uint16_t _temp = round((temp + 20) * 100);
  uint8_t _humi = round(humi * 2);
  uint8_t _relais = 0;
  _relais |= relais1 ? 0x1 : 0x0;
  _relais |= relais2 ? 0x2 : 0x0;

  // write in buf
  buf[0] = _temp >> 8;
  buf[1] = _temp & 0xFF;
  buf[2] = _humi;
  buf[3] = _relais;
}

void setup() {
  // GPIO Initialization
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
  pinMode(PIN_RELAIS_1, OUTPUT);
  pinMode(PIN_RELAIS_2, OUTPUT);

  // Red LED
  setRgbLed(255, 0, 0);

  // Start Serial Connection
  Serial1.begin(115200);
  while (!Serial1)
    ;

  // make sure that there's enough time to flash
  delay(20000);
  Serial1.println("\n\nstarting...");

  // LoRa module initialization
  Serial1.print("starting modem ... ");
  if (!modem.begin(EU868)) {
    Serial1.println("Error");
    delayReset();
  };
  Serial1.println("OK");

  // Join procedure to the network server
  Serial1.print("join network ... ");
  uint8_t joinAttempts = 0;
  int connected = 0;
  do {
    connected = modem.joinOTAA(APP_EUI, APP_KEY);
    joinAttempts++;
    if (connected) {
      break;
    } else {
      delay(ATTEMPT_DELAY);
    }
  } while (joinAttempts <= MAX_ATTEMPTS);
  if (!connected) {
    Serial1.println("Error");
    delayReset();
  }
  Serial1.println("OK");

  // Sensor start
  Serial1.print("search sensor ... ");
  if (!sht.begin()) {
    Serial1.println("Error");
    delayReset();
  }
  Serial1.println("OK");

  // turn off LED
  setRgbLed(0, 0, 0);
}

void loop() {
  // variables
  static float temp = 0;
  static float lastTemp = 0;
  static float humi = 0;
  static uint8_t relais1 = LOW;
  static uint8_t relais2 = LOW;
  static uint16_t measurementsSinceLastTx = 0;
  static bool toSend = false;
  uint32_t now = millis();

  // Begin Measurement and sending routine
  setRgbLed(0, 255, 0);  // green LED

  // read the sensor values
  sht.readBoth(&temp, &humi);
  measurementsSinceLastTx++;
  Serial1.print("Temp: ");
  Serial1.print(temp, 2);
  Serial1.print(", Humi: ");
  Serial1.println(humi, 1);


  // check if there was a temperature change
  if (abs(temp - lastTemp) > TEMP_HYSTERESIS) {
    toSend = true;
    lastTemp = temp;
    Serial1.println("Sending due to large temperature change");
  }

  // check if there has been no message for a long time
  if (measurementsSinceLastTx >= MIN_SEND_INTERVAL / MEASUREMENT_TIMEOUT) {
    toSend = true;
    lastTemp = temp;
    Serial1.println("Sending due to time offset");
  }

  // send and receive if necessary
  if (toSend) {
    uint8_t txBuf[4];
    convertTxData(temp, humi, relais1, relais2, txBuf);

    modem.beginPacket();
    modem.write(txBuf, 4);
    toSend = modem.endPacket() ? false : true;

    if (!toSend) {
      Serial1.print("Sended data: ");
      Serial1.print(txBuf[0], HEX);
      Serial1.print(txBuf[1], HEX);
      Serial1.print(txBuf[2], HEX);
      Serial1.println(txBuf[3], HEX);
    } else {
      Serial1.println("Data could not be sent");
    }

    // check if there is downlonk data availible
    delay(RX_TIMEOUT);  // wait for RX Delays
    bool dataReceived = modem.available() > 0 ? true : false;

    if (dataReceived) {
      uint8_t rxBuf[30];
      uint8_t i = 0;
      uint8_t j = 0;
      Serial1.println("Data received!");

      while (modem.available()) {
        rxBuf[i++] = (uint8_t)modem.read();
      }

      Serial1.print("Received ");
      Serial1.print(i);
      Serial1.println(" Bytes:");

      for (j = 0; j < i; j++) {
        Serial1.print(rxBuf[j], HEX);
      }
      Serial1.print("\n");


      // the received data has the form 0xFF, 0xFE, 0xFD, 0b000000xy where x

      // search for the pattern
      for (j = 0; j < i - 3; j++) {
        if (rxBuf[j] == 0xFF && rxBuf[j + 1] == 0xFE && rxBuf[j + 2] == 0xFD) {
          relais1 = rxBuf[j + 3] & 0x1 ? HIGH : LOW;
          relais2 = rxBuf[j + 3] & 0x2 ? HIGH : LOW;

          // set the relais
          digitalWrite(PIN_RELAIS_1, relais1);
          digitalWrite(PIN_RELAIS_2, relais2);

          // read actual set variables
          relais1 = digitalRead(PIN_RELAIS_1);
          relais2 = digitalRead(PIN_RELAIS_2);

          Serial1.println("Relais newly set to:");
          Serial1.print(relais1);
          Serial1.println(relais2);

          toSend = true; // send update due to new setted relais

          break;
        }
      }
    }

    measurementsSinceLastTx = 0;
  }

  delay(2000);         // make sure one can see the led
  setRgbLed(0, 0, 0);  // dark LED
  LowPower.deepSleep(MEASUREMENT_TIMEOUT * 60000 - (millis() - now));
}