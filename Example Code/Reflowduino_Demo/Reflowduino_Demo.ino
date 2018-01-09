/*
 * Title: Reflowduino Demo
 * Author: Timothy Woo
 * Website: www.botletics.com
 * Last modified: 12/1/2017
 *
 * -----------------------------------------------------------------------------------------------
 * This is an example sketch for the Reflowduino reflow oven controller board. The default
 * settings in this code is for lead-free solder found in most solder paste, but the parameters
 * can be changed or added to suit your exact needs. The code implements temperature PID control
 * to follow the desired temperature profile and uses Bluetooth Low Energy (BLE) to communicate
 * the readings to the Reflowdiuno app. This code can also enable the Reflowduino to automatically
 * enter data points into Excel to graph the data in real time if connected to a computer. Simply
 * uncomment lines 262-268 to enable this feature!
 *
 * Order a Reflowduino at https://www.botletics.com/products/reflowduino
 * Full documentation and design resources at https://github.com/botletics/Reflowduino
 *
 * -----------------------------------------------------------------------------------------------
 * Credits: Special thanks to all those who have been an invaluable part of the DIY community,
 * like the author of the Arduino PID library and the developers at Adafruit!
 *
 * -----------------------------------------------------------------------------------------------
 * License: This code is released under the GNU General Public License v3.0
 * https://choosealicense.com/licenses/gpl-3.0/ and appropriate attribution must be
 * included in all redistributions of this code.
 *
 * -----------------------------------------------------------------------------------------------
 * Disclaimer: Dealing with mains voltages is dangerous and potentially life-threatening!
 * If you do not have adequate experience working with high voltages, please consult someone
 * with experience or avoid this project altogether. We shall not be liable for any damage that
 * might occur involving the use of the Reflowduino and all actions are taken at your own risk.
 */

#include <SoftwareSerial.h> // Library needed for Bluetooth communication
#include <Keyboard.h> // Only if you need the ATmega32u4 to act as a keyboard

// Libraries needed for using MAX31855 thermocouple interface
#include <SPI.h>
#include "Adafruit_MAX31855.h" // https://github.com/adafruit/Adafruit-MAX31855-library

// Library for PID control
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library

#include "pitches.h" // Includes the different notes for the buzzer

// Define pins
#define buzzer 5
#define relay 7
#define BT_RX 9
#define BT_TX 10
#define LED 13 // This LED is used to indicate if the reflow process is underway
#define MAX_CS 8 // MAX31855 chip select pin
#define BTN_P2 2
#define BTN_P3 3

// Initialize Bluetooth software serial
SoftwareSerial BT = SoftwareSerial(BT_TX,BT_RX); // Reflowduino (RX, TX), Bluetooth (TX, RX)

// Initialize thermocouple with hardware SPI
// Reflowduino uses hardware SPI to save digital pins
Adafruit_MAX31855 thermocouple(MAX_CS);

// Define if you want to enable the keyboard feature to type data into Excel
#define enableKeyboard false

// Define reflow temperature profile parameters (in *C)
// First define a subtraction constant to compensate for overshoot:
#define T_const 1 // From testing, overshoot was about 5-6*C

// Standard lead-free solder paste (melting point around 215*C)
//#define T_preheat 150
//#define T_soak 217
//#define T_reflow 249 - T_const

// "Low-temp" lead-free solder paste (melting point around 138*C)
//#define T_preheat 90
//#define T_soak 138
//#define T_reflow 165 - T_const

// Superior Solder 8013-85 Water Soluble
#define T_preheat 120
#define T_soak 150
#define soak_time 50*1000
#define T_reflow_a 180 - T_const
#define reflow_a_time 50*1000
#define T_reflow_b 220 - T_const
#define reflow_b_time 20*1000

// Test values to make sure your Reflowduino is actually working
//#define T_preheat 50
//#define T_soak 80
//#define T_reflow 100 - T_const

#define T_cool 40 // Safe temperature at which the board is "ready" (dinner bell sounds!)
#define preheat_rate 2 // Increase of 1-3 *C/s
#define soak_rate 0.7 // Increase of 0.5-1 *C/s
#define reflow_rate 2 // Increase of 1-3 *C/s
#define cool_rate -4 // Decrease of < 6 *C/s max to prevent thermal shock. Negative sign indicates decrease

// Define PID parameters. The gains depend on your particular setup
// but these values should be good enough to get you started
#define PID_sampleTime 1000 // 1000ms = 1s
// Preheat phase
#define Kp_preheat 150
#define Ki_preheat 0
#define Kd_preheat 100
// Soak phase
#define Kp_soak 200
#define Ki_soak 0.05
#define Kd_soak 300
// Reflow phase
#define Kp_reflow 300
#define Ki_reflow 0.05
#define Kd_reflow 350

// Bluetooth app settings. Define which characters belong to which functions
#define dataChar '*' // App is receiving data from Reflowduino
#define stopChar '!' // App is receiving command to stop reflow process (process finished!)
#define startReflow 'A' // Command from app to "activate" reflow process
#define stopReflow 'S' // Command from app to "stop" reflow process at any time
enum STATE{idle,preheat,soak,reflowA,reflowB,cool};
enum STATE currState = idle; // {idle,preheat,soak,reflowA,reflowB,cool};

double temperature, output, setPoint; // Input, output, set point
PID myPID(&temperature, &output, &setPoint, Kp_preheat, Ki_preheat, Kd_preheat, DIRECT);

// Buzzer settings
// This melody plays when the reflow temperature is reached,
// at which point you should open the door (for toaster ovens)
int openDoorTune[] = {
  NOTE_G6 // I found that NOTE_G6 catches my attention pretty well
};

// This melody plays at the very end when it's safe to take your PCB's!
int doneDealMelody[] = {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4};

// Note durations: 4 = quarter note, 8 = eighth note, etc.
int noteDurations[] = {4, 8, 8, 4, 4, 4, 4, 4};

// Logic flags
bool justStarted = true;
volatile bool reflow = false; // Baking process is underway!
volatile bool buttonPressed = false;
volatile unsigned long lastButtonPress = 0;
#define BUTTON_DEBOUNCE_MSEC 100

bool preheatComplete = false;
bool stageWait = false;
bool soakComplete = false;
bool reflowAComplete = false;
bool reflowBComplete = false;
bool coolComplete = false;

double T_start; // Starting temperature before reflow process
int windowSize = 2000;
unsigned long sendRate = 2000; // Send data to app every 2s
unsigned long t_start = 0; // For keeping time during reflow process
unsigned long previousMillis = 0;
unsigned long previousDebugMillis = 0;

unsigned long duration, t_final, windowStartTime, timer;

void setup() {
  Serial.begin(9600); // This should be different from the Bluetooth baud rate
  BT.begin(57600);
  delay(10); // wait a bit before initting

  pinMode(buzzer, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(relay, OUTPUT);

  digitalWrite(BTN_P2, INPUT_PULLUP);
  digitalWrite(BTN_P3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_P2), buttonPress, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_P3), buttonPress, FALLING);

  digitalWrite(LED, LOW);
  digitalWrite(relay, LOW); // Set default relay state to OFF

  myPID.SetOutputLimits(0, windowSize);
  myPID.SetSampleTime(PID_sampleTime);
  myPID.SetMode(AUTOMATIC); // Turn on PID control

  //while (!Serial) delay(1); // OPTIONAL: Wait for serial to connect
  Serial.println("*****Reflowduino demo*****");

  if (enableKeyboard) Keyboard.begin(); // Only if you want to type data into Excel
  reflow = false;
  buttonPressed = false;
}

void buttonPress() {
  buttonPressed = true;
}


void loop() {

  if (buttonPressed && (millis() - lastButtonPress) > BUTTON_DEBOUNCE_MSEC) {
    lastButtonPress = millis();
    justStarted = true;
    buttonPressed = false;
    reflow = true;
    Serial.println("*********************** button pressed ******************************");
  }

  bool printThisLoop = false;
  if (millis() - previousDebugMillis > sendRate) {
    previousDebugMillis = millis();
    printThisLoop = true;
  }

  /***************************** MEASURE TEMPERATURE *****************************/
  temperature = thermocouple.readCelsius(); // Read temperature
//  temperature = thermocouple.readFarenheit(); // Alternatively, read in deg F but will need to modify code

  /***************************** REFLOW PROCESS CODE *****************************/
  if (reflow) {
    digitalWrite(LED, HIGH); // Red LED indicates reflow is underway

    // This only runs when you first start the reflow process
    if (justStarted) {
      currState = idle; // {idle,preheat,soak,reflowA,reflowB,cool};

      justStarted = false;

      t_start = millis(); // Begin timers
      windowStartTime = millis();
      T_start = temperature;

      if (isnan(T_start)) {
       Serial.println("Invalid reading, check thermocouple!");
      }
      else {
       Serial.print("Starting temperature: ");
       Serial.print(T_start);
       Serial.println(" *C");
      }
    }

    // Determine the amount of time that elapsed in any particular phase (preheat, soak, etc)
    duration = millis() - t_start;

    // Determine the desired set point according to where are in the reflow process
    // Perform a linear extrapolation of what desired temperature we want to be at.
    /********************* PREHEAT *********************/
    if (!preheatComplete) {
      currState = preheat; // ,soak,reflowA,reflowB,cool};

      if (temperature >= T_preheat) { // Check if the current phase was just completed
        preheatComplete = true;
        currState = soak; // ,soak,reflowA,reflowB,cool};

        t_start = millis(); // Reset timer for next phase
        Serial.println("Preheat phase complete!");
      }
      else {
        // Calculate the projected final time based on temperature points and temperature rates
        t_final = (T_preheat - T_start) / preheat_rate + t_start;
        // Calculate desired temperature at that instant in time using linear interpolation
        setPoint = duration * (T_preheat - T_start) / (t_final - t_start);
        if (false && printThisLoop) {
          Serial.print("preheat set temp: ");
          Serial.println(setPoint);
        }
      }
    }
    /********************* SOAK *********************/
    else if (!soakComplete) {
      currState = soak; // ,soak,reflowA,reflowB,cool};

      if (!stageWait && temperature >= T_soak) {
        stageWait = true; // start the timer
        //soakComplete = true;
        t_start = millis();
        Serial.println("Soaking phase starting!");
      }
      else {
        t_final = (T_soak - T_start) / soak_rate + t_start;
        setPoint = duration * (T_soak - T_start) / (t_final - t_start);
        if (false && printThisLoop) {
          Serial.print("soak set temp: ");
          Serial.println(t_final);
        }
      }

              if (printThisLoop) {
        Serial.print("soak duration: ");
        Serial.println(duration);
        }

      if (!soakComplete && stageWait) {
        if (duration > soak_time) {
          currState = reflowA; // ,soak,reflowA,reflowB,cool};
          soakComplete = true;
          stageWait = false;
          Serial.println("Soaking phase completed.");
        }
      }
    }
    /********************* REFLOW A *********************/
    else if (!reflowAComplete) {
      currState = reflowA; // ,soak,reflowA,reflowB,cool};

      if (!stageWait && temperature >= T_reflow_a) {
        stageWait = true; // start the timer
        //reflowAComplete = true;
        t_start = millis();
        Serial.println("Reflow phase A starting!");
        tone(buzzer, openDoorTune, 2000); // Alert the user to open the door!
      }
      else {
        t_final = (T_reflow_a - T_start) / reflow_rate + t_start;
        setPoint = duration * (T_reflow_a - T_start) / (t_final - t_start);
        if (false && printThisLoop) {
          Serial.print("reflowA set temp: ");
          Serial.println(t_final);
        }
      }

      if (!reflowAComplete && stageWait) {
        Serial.println("Reflow phase A waiting.");

        if (duration > reflow_a_time) {
          currState = reflowB; // ,soak,reflowA,reflowB,cool};
          reflowAComplete = true;
          stageWait = false;
          Serial.println("Reflow phase A completed.");
        }
      }
    }
    /********************* REFLOW B *********************/
    else if (!reflowBComplete) {
      currState = reflowB; // ,soak,reflowA,reflowB,cool};
      if (!stageWait && temperature >= T_reflow_b) {
        stageWait = true; // start the timer
        //reflowBComplete = true;
        t_start = millis();
        Serial.println("Reflow phase B starting!");
        tone(buzzer, openDoorTune, 2000); // Alert the user to open the door!
      }
      else {
        t_final = (T_reflow_b - T_start) / reflow_rate + t_start;
        setPoint = duration * (T_reflow_b - T_start) / (t_final - t_start);
        if (false && printThisLoop) {
          Serial.print("reflowB set temp: ");
          Serial.println(t_final);
        }
      }

      if (!reflowBComplete && stageWait) {
        Serial.println("Reflow phase B waiting.");
        if (duration > reflow_b_time) {
          currState = cool; // ,soak,reflowA,reflowB,cool};
          reflowBComplete = true;
          stageWait = false;
          Serial.println("Reflow phase B completed.");
        }
      }
    }
    /********************* COOLDOWN *********************/
    else if (!coolComplete) {
      if (temperature <= T_cool) {
        coolComplete = true;
        reflow = false;
        Serial.println("PCB reflow complete!");
        BT.print(stopChar); // Tell the app that the entire process is finished!
        playTune(doneDealMelody); // Play the buzzer melody
      }
      else {
        t_final = (T_cool - T_start) / cool_rate + t_start;
        setPoint = duration * (T_cool - T_start) / (t_final - t_start);
        if (false && printThisLoop) {
          Serial.print("cool set temp: ");
          Serial.println(t_final);
        }
      }
    }

    // Use the appropriate PID parameters based on the current phase
    if (!soakComplete) myPID.SetTunings(Kp_soak, Ki_soak, Kd_soak);
    else if (!reflowBComplete) myPID.SetTunings(Kp_reflow, Ki_reflow, Kd_reflow);

    // Compute PID output (from 0 to windowSize) and control relay accordingly
    myPID.Compute(); // This will only be evaluated at the PID sampling rate
    if (millis() - windowStartTime >= windowSize) windowStartTime += windowSize; // Shift the time window
    if (output > millis() - windowStartTime) digitalWrite(relay, HIGH); // If HIGH turns on the relay
//    if (output < millis() - windowStartTime) digitalWrite(relay, HIGH); // If LOW turns on the relay
    else digitalWrite(relay, LOW);
  }
  else digitalWrite(LED, LOW);

  /***************************** BLUETOOTH CODE *****************************/
  BT.flush();
  char request = ' ';

  // Send data to the app periodically
  if (millis() - previousMillis > sendRate) {
    previousMillis = millis();
    Serial.print("--> Temperature: "); // The right arrow means it's sending data out
    Serial.print(temperature);
    Serial.print(" *C; reflow? ");
    Serial.print(reflow);
    Serial.print("; preheat complete? ");
    Serial.print(preheatComplete);
    Serial.print("; soak complete? ");
    Serial.print(soakComplete);
    Serial.print("; stage wait? ");
    Serial.print(stageWait);
    Serial.print("; enum status? ");
    Serial.print(currState);
    //Serial.print("; relay? ");
    //Serial.println(relay);
    Serial.println("");

    if (!isnan(temperature)) { // Only send the temperature values if they're legit
      BT.print(dataChar); // This tells the app that it's data
      BT.print(String(temperature)); // Need to cast to String for the app to receive it properly

      if (enableKeyboard && reflow) {
        // Type time and temperature data into Excel on separate columns!
        Keyboard.print((millis()-timer)/1000); // Convert elapsed time from ms to s
        Keyboard.print('\t'); // Tab to go to next column
        Keyboard.print(temperature);
        Keyboard.println('\n'); // Jump to new row
      }
    }
  }

  // Check for an incoming command. If nothing was sent, return to loop()
  if (BT.available() < 1) return;

  request = BT.read();  // Read request
  Serial.print("REQUEST: "); Serial.println(request); // DEBUG

  if (request == startReflow) { // Command from app to start reflow process
    justStarted = true;
    reflow = true; // Reflow started!
    t_start = millis(); // Record the start time
    timer = millis(); // Timer for logging data points
    Serial.println("<-- ***Reflow process started!"); // Left arrow means it received a command
  }
  else if (request == stopReflow) { // Command to stop reflow process
    digitalWrite(relay, LOW); // Turn off appliance and set flag to stop PID control
    reflow = false;
    Serial.println("<-- ***Reflow process aborted!");
  }
  // Add you own functions here and have fun with it!
}

// This function plays the melody for the buzzer.
// Make this as simple or as elaborate as you wish!
void playTune(int *melody) {
  // Iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    // To calculate the note duration, take one second divided by the note type
    // e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(buzzer, melody[thisNote], noteDuration);

    // To distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(buzzer);
  }
}
