#include "fridge.h"

byte fridgeState[2] = { IDLE, IDLE };      // [0] - current fridgeTemp state; [1] - fridgeTemp state t - 1 history
double peakEstimator = 30;    // to predict COOL overshoot; units of deg C per hour (always positive)
double peakEstimate = 0;      // to determine prediction error = (estimate - actual)
unsigned long startTime = 0;  // timing variables for enforcing min/max cycling times
unsigned long stopTime = 0;

void updateFridge() {        // maintain fridgeTemp at temperature set by mainPID -- COOLing with predictive differential, HEATing with time proportioned heatPID
  Serial.print("Checking fridge: ");
  Serial.print(fridgeState[0]);
  Serial.print(fridgeState[1]);
  Serial.println(coolOutput);
  switch (fridgeState[0]) {  // MAIN switch -- IDLE/peak detection, COOL, HEAT routines
    default:
    case IDLE:
      if (fridgeState[1] == IDLE) {   // only switch to HEAT/COOL if not waiting for COOL peak
        if ((fridgeTemp.getFilter() > coolOutput + fridgeIdleDiff) && ((unsigned long)((millis() - stopTime) / 1000) > coolMinOff)) {  // switch to COOL only if temp exceeds IDLE range and min off time met
          #if DEBUG == true
            Serial.println("Updating fridge state from IDLE IDLE to COOL: ");
          #endif
          updateFridgeState(COOL);    // update current fridgeTemp status and t - 1 history
          digitalWrite(relayCool, RELAY_ON);  // close relay 1; supply power to fridgeTemp compressor
          startTime = millis();       // record COOLing start time
        }
        else if ((fridgeTemp.getFilter() < coolOutput - fridgeIdleDiff) && ((unsigned long)((millis() - stopTime) / 1000) > heatMinOff)) {  // switch to HEAT only if temp below IDLE range and min off time met
          #if DEBUG == true
            Serial.println("Updating fridge state from IDLE IDLE to HEAT: ");
          #endif
          updateFridgeState(HEAT);
          if (programState & 0b010000) heatSetpoint = coolOutput;  // update heat PID setpoint if in automatic mode
          heatPID.Compute();      // compute new heat PID output, update timings to align PID and time proportioning routine
          startTime = millis();   // start new time proportioned window
        }
      }
      else if (fridgeState[1] == COOL) {  // do peak detect if waiting on COOL
        if (fridgeTemp.peakDetect()) {        // negative peak detected...
          #if DEBUG == true
            Serial.println("Fridge state from IDLE COOL peak estimate: ");
          #endif
          tuneEstimator(&peakEstimator, peakEstimate - fridgeTemp.getFilter());  // (error = estimate - actual) positive error requires larger estimator; negative:smaller
          fridgeState[1] = IDLE;          // stop peak detection until next COOL cycle completes
        }
        else {                                                               // no peak detected
          double offTime = (unsigned long)(millis() - stopTime) / 1000;      // IDLE time in seconds
          if (offTime < peakMaxWait) break;                                  // keep waiting for filter confirmed peak if too soon
          #if DEBUG == true
            Serial.println("Updating fridge state from IDLE COOL to tune non peak: ");
          #endif
          tuneEstimator(&peakEstimator, peakEstimate - fridgeTemp.getFilter());  // temp is drifting in the right direction, but too slowly; update estimator
          fridgeState[1] = IDLE;                                             // stop peak detection
        }
      }
      break;

    case COOL:  // run compressor until peak predictor lands on controller coolOutput
      { double runTime = (unsigned long)(millis() - startTime) / 1000;  // runtime in seconds
      if (runTime < coolMinOn) break;     // ensure minimum compressor runtime
      if (fridgeTemp.getFilter() < coolOutput - fridgeIdleDiff) {  // temp already below output - idle differential: most likely cause is change in setpoint or long minimum runtime
        #if DEBUG == true
          Serial.println("Updating fridge state from COOL to IDLE IDLE: ");
        #endif
        updateFridgeState(IDLE, IDLE);    // go IDLE, ignore peaks
        digitalWrite(relayCool, RELAY_OFF);       // open relay 1; power down fridgeTemp compressor
        stopTime = millis();              // record idle start
        break;
      }
      if ((fridgeTemp.getFilter() - (min(runTime, peakMaxTime) / 3600) * peakEstimator) < coolOutput - fridgeIdleDiff) {  // if estimated peak exceeds coolOutput - differential, set IDLE and wait for actual peak
        peakEstimate = fridgeTemp.getFilter() - (min(runTime, peakMaxTime) / 3600) * peakEstimator;   // record estimated peak prediction
        #if DEBUG == true
          Serial.println("Updating fridge state from COOL to IDLE: ");
        #endif
        updateFridgeState(IDLE);     // go IDLE, wait for peak
        digitalWrite(relayCool, RELAY_OFF);
        stopTime = millis();
      }
      if (runTime > coolMaxOn) {  // if compressor runTime exceeds max on time, skip peak detect, go IDLE
        #if DEBUG == true
          Serial.println("Updating fridge state from COOL to IDLE IDLE, max timeout ");
        #endif
        updateFridgeState(IDLE, IDLE);
        digitalWrite(relayCool, RELAY_OFF);
        stopTime = millis();
      }
      break; }

    case HEAT:  // run HEAT using time proportioning
      { double runTime = millis() - startTime;  // runtime in ms
      if ((runTime < heatOutput) && digitalRead(relayHeat)) digitalWrite(relayHeat, RELAY_ON);           // active duty; close relay, write only once
        else if ((runTime > heatOutput) && !digitalRead(relayHeat)) digitalWrite(relayHeat, RELAY_OFF);  // active duty completed; rest of window idle; write only once
      if (programState & 0b010000) heatSetpoint = coolOutput;
      if (heatPID.Compute()) {  // if heatPID computes (once per window), current window complete, start new
        startTime = millis();
      }
      if (fridgeTemp.getFilter() > coolOutput + fridgeIdleDiff) {  // temp exceeds setpoint, go to idle to decide if it is time to COOL
        #if DEBUG == true
          Serial.println("Updating fridge state from HEAT to IDLE IDLE, exceeded setpoint");
        #endif
        updateFridgeState(IDLE, IDLE);
        digitalWrite(relayHeat, RELAY_OFF);
        stopTime = millis();
      }
      break; }
  }
}

void tuneEstimator(double* estimator, double error) {  // tune fridgeTemp overshoot estimator
  if (abs(error) <= fridgePeakDiff) return;            // leave estimator unchanged if error falls within contstrained peak differential
  if (error > 0) *estimator *= constrain(1.2 + 0.03 * abs(error), 1.2, 1.5);                 // if positive error; increase estimator 20% - 50% relative to error
    else *estimator = max(0.05, *estimator / constrain(1.2 + 0.03 * abs(error), 1.2, 1.5));  // if negative error; decrease estimator 17% - 33% relative to error, constrain to non-zero value
  EEPROMWrite(38, peakEstimator, DOUBLE);              // update estimator value stored in EEPROM
}

void updateFridgeState(byte state) {  // update current fridgeTemp state
  fridgeState[1] = fridgeState[0];
  fridgeState[0] = state;
}

void updateFridgeState(byte state0, byte state1) {  // update current fridgeTemp state and history
  fridgeState[1] = state1;
  fridgeState[0] = state0;
}
