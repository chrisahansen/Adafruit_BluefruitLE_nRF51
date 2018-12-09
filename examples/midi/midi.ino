/*
 * Copyright 2018 Chris Hansen
 * TODO: license
 * 
 * This code goes with the design for the Modolin, a portable instrument
 * with similar fingering to a violin.
 * 
 * MPR121 pinouts correspond to the following fingers:
 * 
 * 0 = right index finger = G string
 * 1 = right middle finger = D string
 * 2 = right ring finger = A string
 * 3 = right pinky finger = E string
 * 4 = left index finger = first finger
 * 5 = left middle finger = second finger
 * 6 = left ring finger = third finger
 * 7 = left pinky finger = fourth finger
 * 8 = right thumb = sharp
 * 9 = left thumb = flat
 * 
 * --- Derived from the following code
 * Copyright 2017 Don Coleman: Arduino 101 Bluetooth MIDI Controller
 * Adafruit midi (TODO)
 */

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_MPR121.h"
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.7.0"

#define _BV(bit) (1 << (bit)) 

Adafruit_MPR121 cap = Adafruit_MPR121();

// TODO: These bits are not used with the current code. Remove?
// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;

//Buffer to hold 5 bytes of MIDI data. Note the timestamp is forced
uint8_t midiData[] = {0x80, 0x80, 0x00, 0x00, 0x00};

int midiChannel = 0;

int g2 = 55;

// increment of notes by [string][finger]
int fingerSteps[4][5] = {
  { 0, 2, 4, 5, 7 }, // G string
  { 7, 9, 11, 12, 14 }, // D string
  { 14, 16, 17, 19, 21 }, // A string
  { 21, 22, 24, 26, 28 }  // E string
};

int lastNote = -1; // none
int currNote = -1; // none

// This app was tested on iOS with the following apps:
//
// https://itunes.apple.com/us/app/midimittr/id925495245?mt=8
// https://itunes.apple.com/us/app/igrand-piano-free-for-ipad/id562914032?mt=8
//
// To test:
// - Run this sketch and open the Serial Monitor
// - Open the iGrand Piano Free app
// - Open the midimittr app on your phone and under Clients select "Adafruit Bluefruit LE"
// - When you see the 'Connected' label switch to the Routing panel
// - Set the Destination to 'iGrand Piano'
// - Switch to the iGrand Piano Free app and you should see notes playing one by one

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Adafruit_BLEMIDI midi(ble);

bool isConnected = false;
int current_note = 60;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// callback
void connected(void)
{
  isConnected = true;

  Serial.println(F(" CONNECTED!"));
  delay(1000);

}

void disconnected(void)
{
  Serial.println("disconnected");
  isConnected = false;
}

void BleMidiRX(uint16_t timestamp, uint8_t status, uint8_t byte1, uint8_t byte2)
{
  Serial.print("[MIDI ");
  Serial.print(timestamp);
  Serial.print(" ] ");

  Serial.print(status, HEX); Serial.print(" ");
  Serial.print(byte1 , HEX); Serial.print(" ");
  Serial.print(byte2 , HEX); Serial.print(" ");

  Serial.println();
}

void playNote(int note) {
  Serial.print("play "); Serial.println(currNote);
  midi.send(0x90, note, 0x64);
}

void releaseNote(uint8_t note) {
  Serial.print("release "); Serial.println(currNote);
  midi.send(0x80, note, 0x64);
}

void setupTouchSensor() {
  Serial.println("Looking for MPR121 Capacitive Touch Sensor"); 
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 found!");
}

void setupBluetooth()
{
  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  //ble.sendCommandCheckOK(F("AT+uartflow=off"));
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Set BLE callbacks */
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);

  // Set MIDI RX callback
  midi.setRxCallback(BleMidiRX);

  Serial.println(F("Enable MIDI: "));
  if ( ! midi.begin(true) )
  {
    error(F("Could not enable MIDI"));
  }

  ble.verbose(false);
  Serial.print(F("Waiting for a connection..."));
}


void setup(void)
{
//  while (!Serial);  // required for Flora & Micro
//  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit MIDI Example"));
  Serial.println(F("---------------------------------------"));

  setupBluetooth();
  setupTouchSensor();
}

void loop() {
  // interval for each scanning ~ 500ms (non blocking)
  ble.update(500);

  // bail if not connected
  if (! isConnected)
    return;

  // Get the currently touched pads
  currtouched = cap.touched();

  int string = -1;
  int finger = 0;
  int halfStep = 0;
  
  for (uint8_t i=0; i<12; i++) {
    if (currtouched & _BV(i)) {
      // Serial.print(i); Serial.println(" touched");
      
      if (i < 4) {
        string = i; // index from 0
      } else if (i < 8) {
        finger = i - 3; // index from 1 (0 is open string)
      } else if (i == 8) {
        halfStep = 1; // sharp
      } else if (i == 9) {
        halfStep = -1; // flat
      }
    }
  }

  currNote = (string == -1 ? -1 : g2 + fingerSteps[string][finger] + halfStep);

  if (lastNote != -1 && currNote != lastNote) {
    releaseNote(lastNote);
  }
  
  if (currNote != -1 && currNote != lastNote) {
    playNote(currNote);
  }

  // reset our state
  lasttouched = currtouched;
  lastNote = currNote;

  // Save power and avoid crashing, disconnecting BLE
  delay(20);
}
