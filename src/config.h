#ifndef CONFIG_H
#define CONFIG_H

#define BAUD 9600
#define DEBUG true
#define RELAY_ON_VALUE HIGH

// Electrodragon 2 relay ESP8266 board.
const byte onewireData = D5; // one-wire data
const byte relayHeat = 13; // heating element
const byte relayCool = 12; // fridge compressor

double coolSetpoint = 11;

#endif
