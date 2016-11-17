#ifndef CONFIG_H
#define CONFIG_H

#define BAUD 9600
#define DEBUG true

// Electrodragon 2 relay ESP8266 board.
#define RELAY_ON_VALUE HIGH
const byte onewireData = 5; // one-wire data
const byte relayHeat = 13; // heating element
const byte relayCool = 12; // fridge compressor

// NodeMCU dev ESP8266 board.
// #define RELAY_ON_VALUE LOW
// const byte onewireData = D5; // one-wire data
// const byte relayHeat = D7; // heating element
// const byte relayCool = D6; // fridge compressor

double coolSetpoint = 25;

#endif
