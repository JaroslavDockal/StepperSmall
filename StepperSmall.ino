#include <TM1638.h>
#include <AccelStepper.h>

// TM1638 inicializace
#define STB 11 // Strobe digital pin
#define CLK 12 // clock digital pin
#define DIO 13 // data digital pin
TM1638 tm(CLK, DIO, STB);

// Inicializace krokových motorů
#define STG_DIR_PIN 3
#define STG_STEP_PIN 2
#define PROP_DIR_PIN 5
#define PROP_STEP_PIN 4
AccelStepper stgStepper(AccelStepper::DRIVER, STG_STEP_PIN, STG_DIR_PIN);
AccelStepper propStepper(AccelStepper::DRIVER, PROP_STEP_PIN, PROP_DIR_PIN);

// Definice pinů IO
const int endSwitch1Pin = 6;
const int endSwitch2Pin = 7;
const int localControlPin = 8;
int refPin = A0;
int speedRefPin = A1;

// Definice parametrů krokového motoru a driveru
const float stepAngle = 1.8;
const int microSteps = 16;
const float stepsPerRevolution = (360 / stepAngle) * microSteps;
const float stepsPerDegree = stepsPerRevolution / 360;

//Další parametry
const float gearBoxRatio = 10.0;
const float normalModeSpeed = 2.5;
const float fastModeSpeed = 5.0;
const float reducedSpeed = 1.0;
float maxStepsPerSecond = gearBoxRatio * stepsPerDegree * reducedSpeed;

// Rychlost sériovky
const unsigned long Baudrate = 9600;

// Proměnné pro pozici motoru
long minPosition = 0;
long maxPosition = 0;
long middlePosition = 0;
long positionRange = 0; 
long currentPosition = 0;
long referencePosition = 0;
int dirDirection = 0;

// Proměnné pro sledování úhel
const float minAngle = -35.0;
const float maxAngle = 35.0;
float angleRange = maxAngle - minAngle;
float actualAngle = 0.0;
float referenceAngle = 0.0;

// Proměnné pro rychlost
const float minSpeed = 15.0;
float actualSpeed = 0.0;
float referenceSpeed = 0.0;
int stepSpeedReference = 0;

// Definice LED na TM1638
const int ccwLED = 1;
const int atReferenceLED = 2;
const int cwLED = 3;
const int digitalInputsLED = 4;
const int analogInputsLED = 5;
const int initLED = 7;
const int heartbeatLED = 8;

// Definice tlačítek na TM1638
const int decreaseBtnTm = 1;
const int increaseBtnTm = 2;
const int fastModeBtnTm = 3;
const int analogModeBtnTm = 4;
const int endSw1Tm = 6;
const int endSw2Tm = 7;
const int initBtnTm = 8;

// Proměnná určující control place
bool analogMode = false;
unsigned long currentMillis = 0;

// Stringy do displaye
const uint8_t calibration[] = {0x39, 0x77, 0x38, 0x80, 0x00, 0x00, 0x00, 0x00}; // CAL.
const uint8_t calibrationDone[] = {0x39, 0x77, 0x38, 0x80, 0x5E, 0x3F, 0x54, 0x79}; // CAL.DONE
const uint8_t callAbb[] = {0x39, 0x77, 0x38, 0x38, 0x00, 0x77, 0x7c, 0x7c}; // Call ABB

// Definice komunikačních promněných
float referenceAngleRemote = 0;
float referenceAngleFb = 0;
float referenceSpeedRemote = 0;
float referenceSpeedFb = 0;
int controlWord = 0;
int statusWord = 0;
bool commError = 1;

// Proměné ze vstupů
bool endSwitch1 = 0;
bool endSwitch2 = 0;
bool increaseBtn = 0;
bool decreaseBtn = 0;
bool initBtn = 0;
bool analogModeBtn = 0;
bool localControlSw = 0;
bool fastModeBtn = 0;
float mappedreference = 0;
float mappedSpeedReference = 0;

// Další proměnné
bool atReference = true;
bool runCwAllowed = 0;
bool runCcwAllowed = 0;

// NMEA related
unsigned long lastMessageTime = 0;
const unsigned long timeoutInterval = 2000; // 2 sekundy

// Debug level - čím vyšší, tím víc zpráv...
bool logLevel = 1;

//Status bits
bool emergencyOffActive, steeringReady, propulsionReady, ecrInCommand;
bool readyForRemote, localInCommand, bit6s, bit7s;
bool cruiseModeActive, maneuveringModeActive, bit10s, autoRunSteeringActive;
bool autoRunPropulsionActive, bit13s, fastModeActive, watchdogOut;

//Control bits
bool emergencyOff, steeringRunInhibit, propulsionRunInhibit, bit3c;
bool remoteInCommand, bit5c, bit6c, bit7c;
bool cruiseMode, maneuveringMode, bit10c, autoRunSteering;
bool autoRunPropulsion, bit13c, fastModeOn, watchdogIn;

void setup() {
  // Inicializace sériovek
  Serial.begin(Baudrate);
  Serial1.begin(Baudrate);
  debugPrint("Setup started.");

  // Inicializace TM1638 modulu
  tm.reset();
  tm.displaySetBrightness(static_cast<pulse_t>(7));

  // Zavolání funkce pro zobrazení speciálního textu na TM1638 displeji
  sendToDisplay(callAbb, sizeof(callAbb));
  delay(5000);

  // Inicializace pinů pro digitální vstupy
  pinMode(endSwitch1Pin, INPUT_PULLUP);
  pinMode(endSwitch2Pin, INPUT_PULLUP);
  pinMode(localControlPin, INPUT_PULLUP);

  // Výpis parametrů
  debugPrint("Defined step angle: " + String(stepAngle) + "°");
  debugPrint("Defined number of microsteps: 1/" + String(microSteps));
  debugPrint("Defined steps per revolution: " + String(stepsPerRevolution));
  debugPrint("Defined steps per angle: " + String(stepsPerDegree,2));

  // Definice maximální rychlosti a zrychlení
  stgStepper.setMaxSpeed(maxStepsPerSecond);
  stgStepper.setAcceleration(sqrt(maxStepsPerSecond));
  propStepper.setMaxSpeed(10000);
  debugPrint("Init speed set to: " + String(maxStepsPerSecond));
  debugPrint("Init acceleration set to: " + String(sqrt(maxStepsPerSecond)));

  // Inicializace pozice
  initializePosition();
  debugPrint("Setup finished.");
}

// Inicializace polohy motoru
void initializePosition() {
  debugPrint("Initializing position...");

  tm.writeLed(initLED, true);
  sendToDisplay(calibration, sizeof(calibration));
  delay(2000);

  // Move in the negative direction until end switch 1 is triggered
  stgStepper.setSpeed(-maxStepsPerSecond);
  while (!getButton(endSw1Tm)) {
    stgStepper.runSpeed();
    debugPrint("Moving to negative direction, current position: " + String(stgStepper.currentPosition()),2);
  }

  stgStepper.setCurrentPosition(0);
  debugPrint("Reached minimum position - defined as 0.");

  // Move in the positive direction until end switch 2 is triggered
  stgStepper.setSpeed(maxStepsPerSecond);
  while (!getButton(endSw2Tm)) {
    stgStepper.runSpeed();
    debugPrint("Moving to positive direction, current position: " + String(stgStepper.currentPosition()),2);
  }
  
  debugPrint("Reached maximum position: " + String(stgStepper.currentPosition()));

  minPosition = -stgStepper.currentPosition()/2;
  maxPosition = stgStepper.currentPosition()/2;
  stgStepper.setCurrentPosition(maxPosition);
  
  debugPrint("Minimum position: " + String(minPosition));
  debugPrint("Middle position defined as " + String(middlePosition));
  debugPrint("Maximum position: " + String(maxPosition));

  // Move to the middle position
  stgStepper.moveTo(middlePosition);
  debugPrint("Moving to middle position, distance to go: " + String(stgStepper.distanceToGo()));
  while (stgStepper.distanceToGo() != 0) {
    stgStepper.run();
    debugPrint("Moving to middle position, distance to go: " + String(stgStepper.distanceToGo()),2);
  }
  debugPrint("Reached middle position.");

  positionRange = maxPosition - minPosition;
  angleRange = maxAngle - minAngle;

  debugPrint("Position range: " + String(positionRange));
  debugPrint("Angle range: " + String(angleRange));
  debugPrint("Steps per degree: " + String(stepsPerDegree));

  sendToDisplay(calibrationDone, sizeof(calibrationDone));
  tm.writeLed(initLED, false);
  delay(1000);

  debugPrint("Position initialized.");
}

void loop() {
  debugPrint("-------------------- New cycle --------------------");

  currentMillis = millis();
  
  readInputs();

  // Kontrola příchodu nové zprávy
  if (Serial1.available()) {
    String message = Serial1.readStringUntil('\n');
    parseNMEA(message);
  }

  // Kontrola časového intervalu mezi zprávami
  if (millis() - lastMessageTime > timeoutInterval) {
    commError = true;
  }

  // Nastavení maximální rychlosti a akcelerace pro běžný provoz
  if (fastModeActive) { 
    maxStepsPerSecond = stepsPerDegree * fastModeSpeed * gearBoxRatio;
  } else { 
    maxStepsPerSecond = stepsPerDegree * normalModeSpeed * gearBoxRatio;
  }
  stgStepper.setSpeed(maxStepsPerSecond);
  stgStepper.setAcceleration(sqrt(maxStepsPerSecond));
  debugPrint("Max steps per second: " + String(maxStepsPerSecond) + (fastModeActive ? ", Fast Mode" : ", Normal Mode")); 

  // Aktualizace aktuální pozice
  currentPosition = stgStepper.currentPosition();
  actualAngle = currentPosition / stepsPerDegree;
  debugPrint("Current possition: " + String(currentPosition)); 
  debugPrint("Actual angle: " + String(actualAngle)); 

  // Kontrola tlačítka pro spuštění inicializační fáze (tlačítko 7 na TM1638 modulu)
  if (getButton(initBtnTm)) {
    debugPrint("Initialization button pressed.");
    initializePosition();
  }

  // Kontrola tlačítek pro změnu referenčního úhlu a režimu
  if (analogModeBtn) { // Tlačítko 4 na TM1638 modulu pro přepnutí mezi režimem
    analogMode = !analogMode;
    debugPrint("Analog mode toggled: " + String(analogMode ? "ON" : "OFF"));
    // Reset referenčního úhlu při přepnutí do digitálního režimu
    if (!analogMode) {
      referenceAngle = 0;
    }
  } 

  // Kontrola tlačítek pro změnu modu
  if (fastModeBtn) { // Tlačítko 4 na TM1638 modulu pro přepnutí mezi režimem
    fastModeActive = !fastModeActive;
    debugPrint("Fast mode " + String(analogMode ? "Enabled" : "Disabled"));
  }

  // Určení referenčního úhlu
  if (!analogMode) {  
    if (increaseBtn) { // Tlačítko 1 na TM1638 modulu pro zvýšení referenčního úhlu
      referenceAngle = (referenceAngle + 1.0 > maxAngle) ? maxAngle : referenceAngle + 1.0;
      debugPrint("Reference angle increased: " + String(referenceAngle),3);
    }
    if (decreaseBtn) { // Tlačítko 2 na TM1638 modulu pro snížení referenčního úhlu
      referenceAngle = (referenceAngle - 1.0 < minAngle) ? minAngle : referenceAngle - 1.0;
      debugPrint("Reference angle decreased: " + String(referenceAngle),3);
    }
    debugPrint("Reference angle from digital inputs: " + String(referenceAngle),2);
  } else { 
    referenceAngle = mappedreference;
    debugPrint("Reference angle from analog input: " + String(referenceAngle),2);
  }

  atReference = abs(referenceAngle - actualAngle) <= 0.5;
  referencePosition = referenceAngle * stepsPerDegree;
  debugPrint("Reference position: " + String(referencePosition));

  debugPrint(atReference ? "At reference position." : "Not at reference position - movement request.");
  // Pohyb motoru
  if (!atReference) {
    runCwAllowed = !endSwitch1 & !(currentPosition <= minPosition);
    runCcwAllowed = !endSwitch2 & !(currentPosition >= maxPosition);
    debugPrint("Run allowed status. CW: " + String(runCwAllowed ? "TRUE" : "FALSE") + ", CCW: " + String(runCcwAllowed ? "TRUE" : "FALSE"),2);
    if (referenceAngle < actualAngle & runCcwAllowed) {
      stgStepper.moveTo(referencePosition);
      dirDirection = 1;
      debugPrint("Moving CCW to position: " + String(referencePosition));
    } else if (referenceAngle > actualAngle & runCwAllowed) {
      stgStepper.moveTo(referencePosition);
      dirDirection = -1;
      debugPrint("Moving CW to position: " + String(referencePosition));
    }
    stgStepper.run();
  } else { 
    if (stgStepper.isRunning()){
      stgStepper.stop();
      debugPrint("Stepper stopped.");
    }
    dirDirection = 0;
  }  

  if (abs(mappedSpeedReference) > (minSpeed/2))  {
    if (abs(mappedSpeedReference) < minSpeed)  {
      referenceSpeed = minSpeed;
    } else { 
      referenceSpeed = mappedSpeedReference;
    }
  } else { 
    referenceSpeed = 0;
  }
  debugPrint("Used speed reference: " + String(referenceSpeed));

  stepSpeedReference = 3 * stepsPerRevolution * referenceSpeed /100;
  debugPrint("Used step speed reference: " + String(stepSpeedReference));

/*
  if (stepSpeedReference != 0) {
    propStepper.setSpeed(stepSpeedReference);
    propStepper.runSpeed();
    debugPrint("Propulsion stepper started.");
  } else { 
    propStepper.stop();
    debugPrint("Propulsion stepper stopped.");
  }
*/
  propStepper.setSpeed(9600);
  propStepper.runSpeed();

  // Aktualizace indikace
  updateLedIndication();
  updateDisplayIndication(referenceAngle, actualAngle, dirDirection);

  sendNMEA();
}

void readInputs () {
  endSwitch1 = (digitalRead(endSwitch1Pin) == LOW) | getButton(endSw1Tm);
  endSwitch2 = (digitalRead(endSwitch2Pin) == LOW) | getButton(endSw2Tm);
  increaseBtn = getButton(increaseBtnTm);
  decreaseBtn = getButton(decreaseBtnTm);
  initBtn = getButton(initBtnTm);
  analogModeBtn = getButton(analogModeBtnTm);
  localControlSw = digitalRead(localControlPin) == LOW;
  fastModeBtn = getButton(fastModeBtnTm);

  mappedreference = analogRead(refPin) * (angleRange / 1023.0) + minAngle;
  mappedSpeedReference = analogRead(speedRefPin) * (200.0 / 1023.0) - 100.0;
  debugPrint("Mapped analog angle reference: " + String(mappedreference,1)); 
  debugPrint("Mapped analog speed reference: " + String(mappedSpeedReference,0));

  referenceAngleFb = referenceAngle;
  referenceSpeedFb = referenceSpeed;
}

bool getButton (int button)  {
  return tm.getButton(static_cast<button_t>(button-1));
}

void parseNMEA(String message) {
  // Kontrola formátu zprávy
  if (message.startsWith("$ADPXA")) {
    debugPrint("Received NMEA: " + message);
    // Odstranění znaku '$' a rozdělení zprávy na části
    message.remove(0, 1);
    int checksumIndex = message.indexOf('*');
    if (checksumIndex == -1) return; // Kontrola existence checksumu

    // Extrahování částí zprávy
    String dataPart = message.substring(0, checksumIndex);
    String checksumPart = message.substring(checksumIndex + 1);

    // Výpočet a kontrola checksumu
    byte calculatedChecksum = 0;
    for (int i = 0; i < dataPart.length(); i++) {
      calculatedChecksum ^= dataPart[i];
    }
    byte receivedChecksum = strtol(checksumPart.c_str(), NULL, 16);
    if (calculatedChecksum != receivedChecksum) return;

    // Rozdělení datové části zprávy na jednotlivé položky
    int firstCommaIndex = dataPart.indexOf(',');
    int secondCommaIndex = dataPart.indexOf(',', firstCommaIndex + 1);
    int thirdCommaIndex = dataPart.indexOf(',', secondCommaIndex + 1);
    
    if (firstCommaIndex == -1 || secondCommaIndex == -1 || thirdCommaIndex == -1) return;

    String referenceAngleString = dataPart.substring(firstCommaIndex + 1, secondCommaIndex);
    String referenceSpeedString = dataPart.substring(secondCommaIndex + 1, thirdCommaIndex);
    String controlWordString = dataPart.substring(thirdCommaIndex + 1);

    // Konverze na příslušné typy
    referenceAngleRemote = referenceAngleString.toFloat();
    referenceSpeedRemote = referenceSpeedString.toFloat();
    controlWord = strtol(controlWordString.c_str(), NULL, 16);
    decompileControlWord();
    debugPrint("Control word: " + String(controlWord),2);

    // Aktualizace času poslední přijaté zprávy
    lastMessageTime = millis();
    commError = false;
  }
}

// Funkce pro odesílání NMEA zprávy
void sendNMEA() {
  compileStatusWord();
  debugPrint("Status word: " + String(statusWord),2);

  // Sestavení NMEA zprávy
  String nmeaMessage = "$ADPXB," + String(referenceAngleFb, 1) + "," + String(referenceSpeedFb, 1) + "," + String(actualAngle, 1) + "," + String(actualSpeed, 1) + "," + statusWord;

  // Výpočet checksumu
  int checksum = 0;
  for (int i = 1; i < nmeaMessage.length(); i++) {
    checksum ^= nmeaMessage[i];
  }

  // Přidání checksumu do zprávy
  nmeaMessage += "*" + String(checksum, HEX);

  Serial1.println(nmeaMessage);

  debugPrint("Sent NMEA: " + nmeaMessage);
}

void updateLedIndication() {
  debugPrint("Updating LED indication.",3);

  // Zobrazení směru otáčení na TM1638
  if (dirDirection == 1) {
    tm.writeLed(cwLED, true);
    tm.writeLed(ccwLED, false);
    debugPrint("CW LED on, CCW LED off.",4);
  } else if (dirDirection == -1){
    tm.writeLed(cwLED, false);
    tm.writeLed(ccwLED, true);
    debugPrint("CW LED off, CCW LED on.",4);
  } else {
    tm.writeLed(cwLED, false);
    tm.writeLed(ccwLED, false);
    debugPrint("CW LED off, CCW LED off.",4);
  }

  if (analogMode){
    tm.writeLed(analogInputsLED, true);
    tm.writeLed(digitalInputsLED, false);
    debugPrint("Analog mode LED on, Digital mode LED off.",4);
  } else {
    tm.writeLed(analogInputsLED, false);
    tm.writeLed(digitalInputsLED, true);
    debugPrint("Analog mode LED off, Digital mode LED on.",4);
  }

  tm.writeLed(atReferenceLED, atReference);
  debugPrint("At reference LED: " + String(atReference ? "ON" : "OFF"), 4);

  if (blink1s()) {
    tm.writeLed(heartbeatLED, true);
  } else {
    tm.writeLed(heartbeatLED, false);
  }

  debugPrint("LED indication updated.",3);
}

void updateDisplayIndication(float ref, float act, int dir) {
  debugPrint("Updating display indication.",3);
  byte display_data[8] = {0x00};

  // Define segment codes
  byte digit_codes[20] = {
      0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F,  // 0-9
      0x39, 0x77, 0x38, 0x7C, 0x40, 0x80, 0x00, 0x09, 0x24, 0x12   // Ten zbytek
  };

  // 1. segment - sign of ref
  if (ref < 0) {
    display_data[0] = digit_codes[14]; // Minus sign
  } else {
    display_data[0] = digit_codes[16]; // Space (no display)
  }

  // 2. segment - tens place of ref
  if (abs(ref) < 10) {
    display_data[1] = digit_codes[16]; // Space (no display)
  } else {
    display_data[1] = digit_codes[abs(int(ref / 10))];
  }

  // 3. segment - units place of ref + decimal point
  display_data[2] = digit_codes[abs(static_cast<int>(ref)) % 10];

  // 4. segment - dir
  int dirIndex = (currentMillis / 500) % 3;

  if (act < 0) {
    display_data[4] = digit_codes[14]; // Minus sign
  } else {
    display_data[4] = digit_codes[16]; // Space (no display)
  }

  // Dejme to na pozici 3 místo desetin reference
  if (dir == 1) {
    display_data[3] = digit_codes[17 + dirIndex]; // CW dir
  } else if (dir == -1) {
    display_data[3] = digit_codes[19 - dirIndex]; // CCW dir
  } else {
    display_data[4] = digit_codes[16]; // Space (no display)
  } 

  // 6. segment - tens place of act
  if (abs(act) < 10) {
    display_data[5] = digit_codes[16]; // Space (no display)
  } else {
    display_data[5] = digit_codes[abs(int(act / 10))];
  }

  // 7. segment - units place of act + decimal point
  display_data[6] = digit_codes[15]; // Decimal point
  display_data[6] += digit_codes[abs(static_cast<int>(act)) % 10];

  // 8. segment - decimal place of act
  display_data[7] = digit_codes[int(abs(act) * 10) % 10];

  // Send data to display
  sendToDisplay(display_data, sizeof(display_data));
  debugPrint("Display updated.", 3);
}

// Ok, už je to 4x, tak to jde do funkce...
void sendToDisplay(const uint8_t* message, uint8_t length) {
  debugPrint("Sending data to display.",3);
  for (uint8_t i = 0; i < length; i++) {
    tm.displayDig(7 - i, message[i]);
  }
  debugPrint("Data sent to display.",3);
}

void debugPrint (String message, int level) {
  if (logLevel >= level) {
     Serial.println(message);
  }   
}

void debugPrint (String message) {
     debugPrint(message, 1);
}

bool blink1s(){
  static unsigned long previousMillis = 0;
  static bool output = false;
  int interval = 1000;
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    output = !output; // Přepnout stav
  }
  return output;
}

void compileStatusWord() {
  statusWord = (emergencyOffActive ? 1 : 0) |
               (steeringReady ? 1 << 1 : 0) |
               (propulsionReady ? 1 << 2 : 0) |
               (ecrInCommand ? 1 << 3 : 0) |
               (readyForRemote ? 1 << 4 : 0) |
               (localInCommand ? 1 << 5 : 0) |
               (bit6s ? 1 << 6 : 0) |
               (bit7s ? 1 << 7 : 0) |
               (cruiseModeActive ? 1 << 8 : 0) |
               (maneuveringModeActive ? 1 << 9 : 0) |
               (bit10s ? 1 << 10 : 0) |
               (autoRunSteeringActive ? 1 << 11 : 0) |
               (autoRunPropulsionActive ? 1 << 12 : 0) |
               (bit13s ? 1 << 13 : 0) |
               (fastModeActive ? 1 << 14 : 0) |
               (watchdogOut ? 1 << 15 : 0);
}

void decompileControlWord() {
  emergencyOff = (controlWord & (1 << 0)) != 0;
  steeringRunInhibit = (controlWord & (1 << 1)) != 0;
  propulsionRunInhibit = (controlWord & (1 << 2)) != 0;
  bit3c = (controlWord & (1 << 3)) != 0;
  remoteInCommand = (controlWord & (1 << 4)) != 0;
  bit5c = (controlWord & (1 << 5)) != 0;
  bit6c = (controlWord & (1 << 6)) != 0;
  bit7c = (controlWord & (1 << 7)) != 0;
  cruiseMode = (controlWord & (1 << 8)) != 0;
  maneuveringMode = (controlWord & (1 << 9)) != 0;
  bit10c = (controlWord & (1 << 10)) != 0;
  autoRunSteering = (controlWord & (1 << 11)) != 0;
  autoRunPropulsion = (controlWord & (1 << 12)) != 0;
  bit13c = (controlWord & (1 << 13)) != 0;
  fastModeOn = (controlWord & (1 << 14)) != 0;
  watchdogIn = (controlWord & (1 << 15)) != 0;
}
