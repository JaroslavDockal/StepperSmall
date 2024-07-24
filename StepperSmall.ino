#include <TM1638.h>
#include <AccelStepper.h>

// Pin definice pro TM1638 modul
#define STB 8 // Strobe digital pin
#define CLK 9 // clock digital pin
#define DIO 10 // data digital pin

// TM1638 inicializace
TM1638 tm(CLK, DIO, STB);

// Define stepper pins
#define STEERING_DIR_PIN 3
#define STEERING_STEP_PIN 2
#define PROPULSION_DIR_PIN 7
#define PROPULSION_STEP_PIN 6

// Inicializace objektu AccelStepper pro krokový motor
AccelStepper SteeringStepper(AccelStepper::DRIVER, STEERING_STEP_PIN, STEERING_DIR_PIN);
AccelStepper PropulsionStepper(AccelStepper::DRIVER, PROPULSION_STEP_PIN, PROPULSION_DIR_PIN);

const unsigned long Baudrate = 9600;

// Definice počtu kroků na jednu otáčku
const int stepsPerRevolution = 51400;

// Definice pinů pro koncové spínače
const int endSwitch1Pin = 4;
const int endSwitch2Pin = 5;

// Definice pinu pro analog ref
int angleRefPin = A0;
int speedRefPin = A1;

// Proměnné pro sledování aktuální pozice motoru
long currentPosition = 0;
long minPosition = 0;
long maxPosition = 0;
long middlePosition = 0;
long referencePosition = 0;
long positionRange = 0; 
int rotationDirection = 0;

// Proměnné pro sledování úhlu
const float maxAngle = 35.0; 
const float minAngle = -35.0;
float referenceAngle = 0.0;
float actualAngle = 0.0;
bool atReference = true;
float angleRange = 0;

// Další proměnné
const float initSpeed = stepsPerRevolution * 100; // Rychlost 1 otáčka za sekundu
const float initAcceleration = stepsPerRevolution * 10; // Akcelerace pro inicializaci
const float maxDegreesPerSecond = 2.5;
float stepsPerDegree = stepsPerRevolution / 360.0;
float maxStepsPerSecond = stepsPerDegree * maxDegreesPerSecond;

// Stavy tlačítek na TM1638
const int ccwLED = 1;
const int atReferenceLED = 2;
const int cwLED = 3;
const int digitalInputsLED = 4;
const int analogInputsLED = 5;
const int initLED = 7;
const int heartbeatLED = 8;

// Proměnná určující control place
bool analogMode = false;
unsigned long currentMillis = 0;

// Stringy do displaye
const uint8_t calibration[] = {0x39, 0x77, 0x38, 0x80, 0x00, 0x00, 0x00, 0x00}; // CAL.
const uint8_t calibrationDone[] = {0x39, 0x77, 0x38, 0x80, 0x5E, 0x3F, 0x54, 0x79}; // CAL.DONE
const uint8_t callAbb[] = {0x39, 0x77, 0x38, 0x38, 0x00, 0x77, 0x7c, 0x7c}; // Call ABB

bool debugEnable = true;

// Proměnné pro sledování úhlu
float referenceSpeed = 0.0;
float actualSpeed = 0.0;

String statusWord = "1A2B";

void setup() {
  Serial.begin(Baudrate);  // USB serial communication for debugging
  Serial1.begin(Baudrate); // Hardware serial communication

  // Zavolání funkce pro zobrazení speciálního textu na TM1638 displeji
  sendToDisplay(callAbb, sizeof(callAbb));
  delay(5000);

  // Inicializace pinů jako vstupy pro koncové spínače
  pinMode(endSwitch1Pin, INPUT_PULLUP);
  pinMode(endSwitch2Pin, INPUT_PULLUP);

  // Temp for testing
  SteeringStepper.setMaxSpeed(initSpeed);
  SteeringStepper.setAcceleration(initAcceleration);

  // Inicializace pozice
  initializePosition();
}

// Inicializace polohy motoru
void initializePosition() {
  // Zapnutí LED při zahájení inicializace
  tm.writeLed(initLED, true);

  // Zobrazení "CAL." na TM1638 displeji
  sendToDisplay(calibration, sizeof(calibration));

  delay(2000);

  // Simulace pohybu do opačného směru, dokud není stisknuto tlačítko 5 (simulovaný koncový spínač 2)
  while (!tm.getButton(static_cast<button_t>(5))) {
    SteeringStepper.moveTo(SteeringStepper.currentPosition() - 100);
    SteeringStepper.run();
  }
  currentPosition = 0;
  minPosition = currentPosition;

  // Simulace pohybu do jednoho směru, dokud není stisknuto tlačítko 6 (simulovaný koncový spínač 1)
  while (!tm.getButton(static_cast<button_t>(6))) {
    SteeringStepper.moveTo(SteeringStepper.currentPosition() + 100);
    SteeringStepper.run();
  }
  maxPosition = currentPosition;
  middlePosition = maxPosition/2;

  // Najeď na pozici 0
  SteeringStepper.moveTo(middlePosition);
  while (SteeringStepper.distanceToGo() != 0) {
    SteeringStepper.run();
  }

  positionRange = maxPosition - minPosition;
  angleRange = maxAngle - minAngle;

  stepsPerDegree = positionRange/angleRange;
  maxStepsPerSecond = 2.5 * stepsPerDegree;

  sendToDisplay(calibrationDone, sizeof(calibrationDone));
  
  delay(1000);
  tm.writeLed(initLED, false); // Vypnutí LED po dokončení inicializace
}

void loop() {
  int endSwitch1Input = digitalRead(endSwitch1Pin);
  int endSwitch2Input = digitalRead(endSwitch2Pin);

  bool endSwitch1Active = (endSwitch1Input == LOW) | (tm.getButton(static_cast<button_t>(6)));
  bool endSwitch2Active = (endSwitch1Input == LOW) | (tm.getButton(static_cast<button_t>(5)));

  currentMillis = millis();
  
  float mappedAngleReference = analogRead(angleRefPin) * (70.0 / 1023.0) - 35.0;
  float mappedSpeedReference = analogRead(speedRefPin) * (290.0 / 1023.0) - 145.0; 

  bool delayInput = currentMillis % 200 >= 0 & currentMillis % 200 <= 10;

  // Nastavení maximální rychlosti a akcelerace pro běžný provoz
  //TODO s tímhle se tomu nechce - nevím co za hodnoty to používá. Nevidím dovnitř...
  //myStepper.setMaxSpeed(maxStepsPerSecond);
  //myStepper.setAcceleration(maxStepsPerSecond/10);

  // Aktualizace aktuální pozice
  currentPosition = SteeringStepper.currentPosition();
  actualAngle = currentPosition / stepsPerDegree;

  // Kontrola tlačítka pro spuštění inicializační fáze (tlačítko 7 na TM1638 modulu)
  if (tm.getButton(static_cast<button_t>(7))) {
    initializePosition();
  }

  // Kontrola tlačítek pro změnu referenčního úhlu a režimu
   if (tm.getButton(static_cast<button_t>(3)) & delayInput) { // Tlačítko 4 na TM1638 modulu pro přepnutí mezi režimem
    analogMode = !analogMode;
    // Reset referenčního úhlu při přepnutí do digitálního režimu
    if (!analogMode) {
      referenceAngle = 0;
    }
  } 

  // Určení referenčního úhlu
  if (!analogMode) {  
    if (tm.getButton(static_cast<button_t>(1)) & delayInput) { // Tlačítko 1 na TM1638 modulu pro zvýšení referenčního úhlu
      referenceAngle = (referenceAngle + 1.0 > maxAngle) ? maxAngle : referenceAngle + 1.0;
    }
    if (tm.getButton(static_cast<button_t>(0)) & delayInput) { // Tlačítko 2 na TM1638 modulu pro snížení referenčního úhlu
      referenceAngle = (referenceAngle - 1.0 < minAngle) ? minAngle : referenceAngle - 1.0;
    }
  } else { 
    referenceAngle = mappedAngleReference;
  }

  // Detekce, zda je aktuální úhel blízko referenčnímu úhlu
  atReference = abs(referenceAngle - actualAngle) <= 0.5;

  // TODO Tohle by mělo dát referenci pozice přímo stepperu až to bude fungovat
  referencePosition = ((referenceAngle - minAngle) * stepsPerDegree) - minPosition;

  // Pohyb motoru
  if (!atReference) {
    //TODO endswitche asi obráceně nebo něco - nechám na později
    //bool runCwAllowed = !endSwitch1Active & !(currentPosition <= minPosition);
    //bool runCcwAllowed = !endSwitch2Active & !(currentPosition >= maxPosition);
    bool runCwAllowed = true;
    bool runCcwAllowed = true;
    if (referenceAngle < actualAngle & runCcwAllowed) {
      SteeringStepper.moveTo(referencePosition);
      SteeringStepper.moveTo(SteeringStepper.currentPosition() + 100);
      rotationDirection = 1;
    } else if (referenceAngle > actualAngle & runCwAllowed) {
      //myStepper.moveTo(referencePosition);
      SteeringStepper.moveTo(SteeringStepper.currentPosition() - 100);
      rotationDirection = -1;
    }
    SteeringStepper.run();

  } else { 
    if (SteeringStepper.isRunning()){
      SteeringStepper.stop();
    }
    rotationDirection = 0;
  }  

  if (abs(mappedSpeedReference) > 7.5 ) {
    PropulsionStepper.setMaxSpeed((abs(mappedSpeedReference < 15.0)) ? 15.0 : mappedSpeedReference);
    PropulsionStepper.runSpeed();
  } else { 
    PropulsionStepper.stop();
  }

  // Aktualizace indikace
  updateLedIndication();
  updateDisplayIndication(referenceAngle,actualAngle,rotationDirection);
}

// Funkce pro aktualizaci stavu LED TM1638
void updateLedIndication() {

  // Zobrazení směru otáčení na TM1638
  if (rotationDirection == 1) {
    tm.writeLed(cwLED, true);
    tm.writeLed(ccwLED, false);
  } else if (rotationDirection == -1){
    tm.writeLed(cwLED, false);
    tm.writeLed(ccwLED, true);
  } else {
    tm.writeLed(cwLED, false);
    tm.writeLed(ccwLED, false);
  }

  if (analogMode){
    tm.writeLed(analogInputsLED, true);
    tm.writeLed(digitalInputsLED, false);
  } else {
    tm.writeLed(analogInputsLED, false);
    tm.writeLed(digitalInputsLED, true);
  }

  tm.writeLed(atReferenceLED, atReference);

  // Heartbeat
  if (blink1s()) {
        tm.writeLed(heartbeatLED, true);
    } else {
        tm.writeLed(heartbeatLED, false);
    }
}

// Funkce pro aktualizaci zobrazení na TM1638 displeji
void updateDisplayIndication(float angleRef, float angleAct, int rotation) {
  byte display_data[8] = {0x00};

  // Define segment codes
  byte digit_codes[20] = {
      0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F,  // 0-9
      0x39, 0x77, 0x38, 0x7C, 0x40, 0x80, 0x00, 0x09, 0x24, 0x12   // Ten zbytek
  };

  // 1. segment - sign of angleRef
  if (angleRef < 0) {
      display_data[0] = digit_codes[14]; // Minus sign
  } else {
      display_data[0] = digit_codes[16]; // Space (no display)
  }

  // 2. segment - tens place of angleRef
  if (abs(angleRef) < 10) {
      display_data[1] = digit_codes[16]; // Space (no display)
  } else {
      display_data[1] = digit_codes[abs(int(angleRef / 10))];
  }

  // 3. segment - units place of angleRef + decimal point
  //display_data[2] = digit_codes[15]; // Decimal point
  display_data[2] = digit_codes[abs(static_cast<int>(angleRef)) % 10];

  // 4. segment - decimal place of angleRef
  //display_data[3] = digit_codes[int(abs(angleRef) * 10) % 10];

  // 5. segment - rotation
  int rotationIndex = (currentMillis / 500) % 3;

  if (angleAct < 0) {
          display_data[4] = digit_codes[14]; // Minus sign
  } else {
          display_data[4] = digit_codes[16]; // Space (no display)
  }

  // Dejme to na pozici 3 místo desetin reference
  if (rotation == 1) {
    display_data[3] = digit_codes[17 + rotationIndex]; // CW rotation
  } else if (rotation == -1) {
    display_data[3] = digit_codes[19 - rotationIndex]; // CCW rotation
  } else {
    display_data[4] = digit_codes[16]; // Space (no display)
  } 

  // 6. segment - tens place of angleAct
  if (abs(angleAct) < 10) {
      display_data[5] = digit_codes[16]; // Space (no display)
  } else {
      display_data[5] = digit_codes[abs(int(angleAct / 10))];
  }

  // 7. segment - units place of angleAct + decimal point
  display_data[6] = digit_codes[15]; // Decimal point
  display_data[6] += digit_codes[abs(static_cast<int>(angleAct)) % 10];

  // 8. segment - decimal place of angleAct
  display_data[7] = digit_codes[int(abs(angleAct) * 10) % 10];

  // Send data to display
  sendToDisplay(display_data, sizeof(display_data));
}

// S parametrem intervalu by to asi haprovalo kdyby byly volány různé časy
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

// Ok, už je to 4x, tak to jde do funkce...
void sendToDisplay(const uint8_t* message, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        tm.displayDig(7 - i, message[i]);
    }
}

void debugPrint(String message) {
  if (debugEnable) {
    Serial.println(message);
  }
}