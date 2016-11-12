#ifndef CONFIG_H
#define CONFIG_H

#define BAUD 9600
#define DEBUG true
#define RELAY_ON_VALUE LOW
#define RELAY_OFF_VALUE HIGH

// Electrodragon 2 relay ESP8266 board.
const byte onewireData = 5; // one-wire data
const byte relayHeat = 13;   // fridge compressor
const byte relayCool = 12;   // heating element

double coolSetpoint = 11;

#endif
