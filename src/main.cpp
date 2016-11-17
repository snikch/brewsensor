#define uint8_t byte

#include "fridge.h"
#include "probe.h"
#include <OneWire.h>
#include "PID_v1.h"
#include "config.h"

double coolInput, coolOutput, coolKp, coolKi, coolKd;  // SP, PV, CO, tuning params for main PID
double heatInput, heatOutput, heatSetpoint, heatKp, heatKi, heatKd;  // SP, PV, CO tuning params for HEAT PID

bool RELAY_ON = RELAY_ON_VALUE;
bool RELAY_OFF = !RELAY_ON_VALUE;

byte programState;  // 6 bit-flag program state -- (mainPID manual/auto)(heatPID manual/auto)(temp C/F)(fermentation profile on/off)(data capture on/off)(file operations) = 0b000000
#define MAIN_PID_MODE 0b100000
#define HEAT_PID_MODE 0b010000
#define DISPLAY_UNIT  0b001000
#define TEMP_PROFILE  0b000100
#define DATA_LOGGING  0b000010
#define FILE_OPS      0b000001


OneWire onewire(onewireData);  // declare instance of the OneWire class to communicate with onewire sensors
// probe externTemp(&onewire), fridgeTemp(&onewire);
probe fridgeTemp(&onewire);

PID coolPID(&coolInput, &coolOutput, &coolSetpoint, 2, 5, 1, DIRECT);  // main PID instance for beer temp control (DIRECT: beer temperature ~ fridge(air) temperature)
PID heatPID(&heatInput, &heatOutput, &heatSetpoint, heatKp, heatKi, heatKd, DIRECT);   // create instance of PID class for cascading HEAT control (HEATing is a DIRECT process)

void setup() {
  pinMode(relayHeat, OUTPUT);  // configure relay pins and write default HIGH (relay open)
  digitalWrite(relayHeat, RELAY_OFF);
  pinMode(relayCool, OUTPUT);
  digitalWrite(relayCool, RELAY_OFF);
  #if DEBUG == true  //start serial at 9600 baud for debuging
    Serial.begin(BAUD);
  #endif

  // externTemp.init();
  fridgeTemp.init();

  // coolPID.SetTunings(coolKp, coolKi, coolKd);    // set tuning params
  coolPID.SetSampleTime(1000);       // (ms) matches sample rate (1 hz)
  coolPID.SetOutputLimits(0.3, 38);  // deg C (~32.5 - ~100 deg F)
  coolPID.SetMode(AUTOMATIC);  // set man/auto
  // if (programState & MAIN_PID_MODE) coolPID.SetMode(AUTOMATIC);  // set man/auto
  //   else coolPID.SetMode(MANUAL);
  coolPID.setOutputType(FILTERED);
  coolPID.setFilterConstant(10);
  coolPID.initHistory();

  heatPID.SetTunings(heatKp, heatKi, heatKd);
  heatPID.SetSampleTime(heatWindow);       // sampletime = time proportioning window length
  heatPID.SetOutputLimits(0, heatWindow);  // heatPID output = duty time per window
  heatPID.SetMode(AUTOMATIC);
  // if (programState & HEAT_PID_MODE) heatPID.SetMode(AUTOMATIC);
  //   else heatPID.SetMode(MANUAL);
  heatPID.initHistory();
}

void mainUpdate();  // update sensors, PID output, fridge state, write to log, run profiles
void loop() {
  mainUpdate();                            // subroutines manage their own timings, call every loop
}

double lastCool;

void mainUpdate() {                              // call all update subroutines
  probe::startConv();                            // start conversion for all sensors
  if (probe::isReady()) {                        // update sensors when conversion complete
    fridgeTemp.update();
    // externTemp.update();
    coolInput = fridgeTemp.getFilter();
    if (coolInput != lastCool) {
      Serial.print("Temp changed ");
      Serial.println(coolInput);
      lastCool = coolInput;
    }
  }
  coolPID.Compute();                             // update main PID
  updateFridge();                                // update fridge status
}
