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
  Serial.println("The board will be resetted soon");
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

  // make sure that there's enough time to flash
  delay(20000); 

  // LoRa module initialization
  if (!modem.begin(EU868)) {
    delayReset();
  };

  // Join procedure to the network server
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
    delayReset();
  }

  // Sensor start
  if (!sht.begin()) {
    delayReset();
  }

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
  uint8_t txBuf[4];
  uint8_t rxBuf[1];

  // Begin Measurement and sending routine
  setRgbLed(0, 255, 0);  // green LED

  // read the sensor values
  sht.readBoth(&temp, &humi);
  measurementsSinceLastTx++;

  // check if there was a temperature change
  if (abs(temp - lastTemp) > TEMP_HYSTERESIS) {
    toSend = true;
    lastTemp = temp;
  }

  // check if there has been no message for a long time
  if (measurementsSinceLastTx >= MIN_SEND_INTERVAL / MEASUREMENT_TIMEOUT) {
    toSend = true;
    measurementsSinceLastTx = 0;
  }

  // send and receive if necessary
  if (toSend) {
    convertTxData(temp, humi, relais1, relais2, txBuf);

    modem.beginPacket();
    modem.write(txBuf, 4);
    toSend = modem.endPacket() ? false : true;

    // check if there is downlonk data availible
    bool dataReceived = modem.available();

    if (dataReceived) {
      // the received data has the form 0xFF, 0xFE, 0xFD, 0b000000xy where x

      // drop old/wrong rx packets
      while(modem.read() != 0xFF && modem.available())
      {
        ;
      }

      // check if the pattern matches
      if (modem.read() == 0xFE & modem.read() == 0xFD)
      {
        // read the actual data and assign it to the relais variables
        rxBuf[0] = (uint8_t)modem.read();
        relais1 = rxBuf[0] & 0x1 ? HIGH : LOW;
        relais2 = rxBuf[0] & 0x2 ? HIGH : LOW;

        // set the relais
        digitalWrite(PIN_RELAIS_1, relais1);
        digitalWrite(PIN_RELAIS_2, relais2);

        // read actual set variables
        relais1 = digitalRead(PIN_RELAIS_1);
        relais2 = digitalRead(PIN_RELAIS_2);
      }

      // drop all data from the rx buffer
      while (modem.available()) {
        modem.read();
      }
    }
  }

  delay(2000); // make sure one can see the led
  setRgbLed(0, 0, 0);          // dark LED
  LowPower.deepSleep(MEASUREMENT_TIMEOUT * 60000);
}