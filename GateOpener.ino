#include "RF24.h"
#include <SPI.h>
#define ARRAY_SIZE(a)  (sizeof(a) / sizeof(*(a)))

RF24 radio(9, 53);
const byte addresses[][6] = {"1Node", "2Node"};
byte radio_command = 0;

struct Version_struct {
  char* version_num;
  char* date;
  char* author;
  char* comment;
};

enum commands {None, Close, Retry1, Retry2, Open};
commands command;

enum gate_positions {OpenPosition, ClosedPosition};
gate_positions gate_position = ClosedPosition;

struct Light_struct {
  char* Light_name;
  int pin;
  int OnButton;
  int OffButton;
  int TimerButton;
  unsigned long Duration;
  unsigned long OnTime;
  bool TimerRunning;
};

struct Light_struct ExteriorLight = {"Exterior Light", 7, 6, 5, 4, 300000, 0, false};
struct Light_struct InteriorLight = {"Interior Light", 23, 25, 27, 29, 300000, 0, false};

struct Furnace_struct {
  int  OnButton;                // Interrupt 0
  int  OffButton;
  int  Pin;
  unsigned long OnTime;
  unsigned long Duration;      // In miliseconds
};

struct Furnace_struct Furnace = {2, 14, 16, 0, 900000};

////////////////////Turns on the light and starts the timer //////////////////////////////////////////////////////
struct Light_struct StartLightTimer (Light_struct Light) {
  Serial.print("Start Timer ");
  Serial.println(Light.Light_name);
  digitalWrite(Light.pin, HIGH);
  Light.OnTime = millis();
  Light.TimerRunning = true;
  return Light;
}

/////////////////Checks if the light is running on timer and turns off if expired.////////////////////////////////////////////////////////
struct Light_struct CheckLightTimer(Light_struct Light) {
  if (Light.TimerRunning) {
    Serial.print(Light.Light_name);
    if ((millis()>(Light.OnTime+Light.Duration))||((millis()<Light.OnTime)&&(millis()<(Light.OnTime+Light.Duration)))) {
      digitalWrite(Light.pin, LOW);
      Light.TimerRunning = false;
      Serial.println(" timer off");
    }
    else    {
      Serial.print(" timer ");
      Serial.println(Light.Duration - (millis() - Light.OnTime));
    }
  }
  return Light;
}

///////////////Checks the three buttons that control the lights///////////////////////////////////////////////////////////
struct Light_struct CheckLightButtons(Light_struct Light) {
  if (!digitalRead(Light.OnButton)) {
    digitalWrite(Light.pin, HIGH);
    Light.TimerRunning = false;
    Serial.print(Light.Light_name);
    Serial.println(" on");
  }
  if (!digitalRead(Light.OffButton)) {
    digitalWrite(Light.pin, LOW);
    Light.TimerRunning = false;
    Serial.print(Light.Light_name);
    Serial.println(" off");
  }
  if (!digitalRead(Light.TimerButton)) {
    Light = StartLightTimer(Light);
  }
  return Light;
}

/*Mega2560 Interrupts:*/
//const int Furnace.OnButton = 2; //interrupt 0 (defined in Furnace struct)
const int OpenButton = 3;       //interrupt 1
const int CloseButton = 21;     //interrupt 2
const int OpenCloseButton = 20; //interrupt 3
const int Eye = 19;             //interrupt 4
const int Toggle = 18;          //interrupt 5

enum terminations {Null = 0, Steps = 1, Cancel_Button = 2, Eye_term = 4, Limit_Switch = 8};
terminations termination = Null;

// Stepper Motor Controller Pin Assignments
const int Pulse          =   22;
const int Enable_Motor         =   28;              //Enable the stepper motor
const int Direction      =   24;
bool Open_Dir = HIGH;

struct OpenClose_struct {
  unsigned long Delay;
  unsigned long OpenTime;
  bool WaitingToClose;
};
struct OpenClose_struct OpenCloseMode = {30000, 0, false};

//Eye sensor Pin Assigment
const int Eye_Enable = 45;
unsigned long RetryCloseTime;

//Limit Switches Pin Assignment
const int OpenSwitch = 10; // Originally 37;
const int CloseSwitch = 12; // Originally 39;
int active_switch;

//LED Indicators Pin Assigment
const int EyeTriggeredLED = 44;
const int OpenSwitchLED = 40;
const int ClosedSwitchLED = 36;
const int ClosingLED = 38;
const int OpeningLED = 42;

int Inputs[] = {Eye, OpenSwitch, CloseSwitch, OpenButton, CloseButton, OpenCloseButton, ExteriorLight.OnButton, ExteriorLight.OffButton, ExteriorLight.TimerButton, InteriorLight.OnButton, InteriorLight.OffButton, InteriorLight.TimerButton, Toggle, Furnace.OnButton, Furnace.OffButton};
int Outputs[] = {Pulse, Direction, Enable_Motor, OpeningLED, ClosingLED, OpenSwitchLED, ClosedSwitchLED, EyeTriggeredLED, Eye_Enable, ExteriorLight.pin, InteriorLight.pin, Furnace.Pin};
const int LEDs[] = {EyeTriggeredLED, OpenSwitchLED, OpeningLED, ClosingLED, ClosedSwitchLED};

//Fine-tuning after limit switch is met; in case the limit switches are slightly out of position
struct FineTune_Struct {
  bool Direction;
  int Inches;
};

const struct FineTune_Struct FineTune_Open {
  Open_Dir, 0
};
const struct FineTune_Struct FineTune_Close {
  !Open_Dir, 0
};

// DIMENSIONS OF THE GATE AND MOTOR//
const int  ft_per_gate = 12;
const int inch_per_ft  =  12;
const int teeth_per_inch = 2;
const float rev_per_teeth = 1.0 / 10.0;
const float small_gear = 14;
const float large_gear = 80;
const int steps_per_rev = 200;
const int uSteps_per_step = 1;
const long GateSteps = uSteps_per_step * steps_per_rev * rev_per_teeth * teeth_per_inch * inch_per_ft * ft_per_gate * large_gear / small_gear;
float step_delay;
const float RPM = 2.00 * large_gear / small_gear;
const float max_delay = 60000000 / (uSteps_per_step*steps_per_rev*RPM);            //slowest point in microseconds
const float min_delay = max_delay / 1.75;                                  //fastest point in microseconds
const float ramp_length = 0.25;                                             //portion of the Gate Length over which the ramp-up occurs. (i.e, 0.25 means ramp-up for the first quarter of travel)
const float ramp = (max_delay - min_delay) / (ramp_length*GateSteps);                          //

unsigned long counter = 0;
int count = 0;

//////////////////////////////////////setup/////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);                                                     //Display Version Info
  Serial.println("Gate Opener");

  Serial.print("Microstepping: 1/");
  Serial.println(uSteps_per_step);
  Serial.print("max_delay (microseconds): ");
  Serial.println(max_delay);
  Serial.print("min_delay (microseconds): ");
  Serial.println(min_delay);

  InitializeInOuts();//Inputs, NumOfInputs, Outputs, NumOfOutputs);
  InitializeRadio();
  blink_LEDs();                                                              //indicate bootup
  AttachInterrupts();
}  //SETUP

/////////////////////////////////loop//////////////////////////////////////////////////
void loop()
{
  //////////////////Read Open/Close switches and set LEDs/////////////////
  digitalWrite(OpenSwitchLED, !digitalRead(OpenSwitch));
  digitalWrite(ClosedSwitchLED, !digitalRead(CloseSwitch));
  if (!digitalRead(OpenSwitch)) gate_position = OpenPosition;
  if (!digitalRead(CloseSwitch)) gate_position = ClosedPosition;

  //////////////////Check Lights///////////////////////////
  ExteriorLight = CheckLightTimer(CheckLightButtons(ExteriorLight));
  InteriorLight = CheckLightTimer(CheckLightButtons(InteriorLight));

  /////////////////Check if it's time to close gate//////////////
  CheckTimeToClose();

  ////////////////Check if it's time to turn off furnace//////////////////
  if (digitalRead(Furnace.Pin)) {
    if ((millis() > Furnace.OnTime + Furnace.Duration)||((millis() < Furnace.OnTime)&&(millis() < Furnace.OnTime + Furnace.Duration))){
      Serial.println("Furnace timer met;  turning off");
      digitalWrite(Furnace.Pin, LOW);
    }
  }

  //////////////Commands come in through ISRs and Radio///////////////////
  RadioISR();

  /////////////////Act on the command//////////////////////////
  switch (command) {
    case Close:
    case Retry1:
    case Retry2:
      CloseGate();
      break;
    case Open:
      OpenGate();
  }
} // loop

///////////////////////////Initialize Inputs and Outputs///////////////////////////////
void InitializeInOuts () {
  int count;
  for (count = 0; count < ARRAY_SIZE(Inputs); count++) {                        //Set the Inputs to Pullup
    pinMode(Inputs[count], INPUT_PULLUP);
  }
  for (count = 0; count < ARRAY_SIZE(Outputs); count++) {                      //Set the Outputs to LOW
    pinMode(Outputs[count], OUTPUT);
    digitalWrite (Outputs[count], LOW);
  }
};

/////////////////Blink the LEDS -- indicates bootup and completion of movement/////////////////////////////////////////
void blink_LEDs() {
  int i;
  Serial.println("BLINK");
  for (count = 0; count < 5 ; count++) {
    for (i = 0; i < ARRAY_SIZE(LEDs); i++) {
      digitalWrite (LEDs[i], HIGH);
      delay(constrain(100 - 30 * count / 10, 20, 100));
      digitalWrite (LEDs[i], LOW);
    }
  }
}//blink_LEDs

///////////////Translate termination values to plain text///////////
char* TerminationString(int termination) {
  char* temp = "Null";
  if (termination >= 1) temp = "Steps";
  if (termination >= 2) temp = "Cancel Button";
  if (termination >= 4) temp = "Eye Triggered";
  if (termination >= 8) temp = "Limit Switch";
  return temp;
}

/////////////////////Fine Tune the Gate Position////////////////////////////
void Fine_Tune(struct FineTune_Struct FineTune) {
  digitalWrite(Direction, FineTune.Direction);
  long TuneSteps = FineTune.Inches * teeth_per_inch * rev_per_teeth * steps_per_rev * uSteps_per_step;
  while (TuneSteps) {
    SendPulse(step_delay);
    TuneSteps--;
  }
}

//////////////////////////////Pulse the Stepper Controller High/Low//////////////////
void SendPulse(float step_delay) {
  digitalWrite(Pulse, HIGH);
  delayMicroseconds(step_delay);
  digitalWrite(Pulse, LOW);
  delayMicroseconds(step_delay);
}

//////////////////////////////Move the Gate////////////////////////////////
void Move_Gate(int active_switch, struct FineTune_Struct FineTune) {
  ExteriorLight = StartLightTimer(ExteriorLight);
  InteriorLight = StartLightTimer(InteriorLight);





  digitalWrite(Enable_Motor, HIGH);                                                                //Enable Motor
  long  step_count = GateSteps;
  termination = Null;
  step_delay = max_delay;                                                                 //start slow
  while (termination == Null) {
    SendPulse(step_delay);
    Serial.println(step_delay);              //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    step_count--;
    step_delay = constrain(step_delay - ramp, min_delay, max_delay);            //  Ramp down the step delay (i.e, speed up the gate, but not more than min delay)
    if (step_count == 0) termination = Steps;
    if (!digitalRead(active_switch)) {                                          //If the limit switch is triggered, do the fine-tune steps (may be either direction)
      Fine_Tune(FineTune);
      termination = Limit_Switch;
    }
    if (radio.available()) RadioISR();
  }
  digitalWrite(Enable_Motor, LOW);
  digitalWrite(Eye_Enable, LOW);
  Serial.print("Termination on: ");
  Serial.print(termination);
  Serial.print(" ");
  Serial.println(TerminationString(termination));
  Serial.print("Step_count: ");
  Serial.println(step_count);
  if ((termination == Steps) || (termination = Limit_Switch)) command = None;      ///////////If movement terminates successfully, command = None, else the ISRs will determine the next command.////////////
  blink_LEDs();
}

////////////////Check if it's time to close the gate during Open/Close Cycle/////////////
void CheckTimeToClose () {
  if (OpenCloseMode.WaitingToClose) {
    Serial.print("Waiting to close  ");
    Serial.print(OpenCloseMode.Delay - (millis() - OpenCloseMode.OpenTime));
    if ((millis() > OpenCloseMode.OpenTime + OpenCloseMode.Delay)||((millis() < OpenCloseMode.OpenTime)&&(millis() < OpenCloseMode.OpenTime + OpenCloseMode.Delay))){
      CloseISR();
    }
  }
}

/////////////////////Attach Interrupts//////////////////////  The Radio is active at all times. The buttons are active except when they are being acted upon. The Eye is active only during closing
void AttachInterrupts() {
  attachInterrupt(digitalPinToInterrupt(Furnace.OnButton), FurnaceISR, LOW);
  attachInterrupt(digitalPinToInterrupt(OpenButton), OpenISR, LOW);
  attachInterrupt(digitalPinToInterrupt(CloseButton), CloseISR, LOW);
  attachInterrupt(digitalPinToInterrupt(OpenCloseButton), OpenCloseISR, LOW);
  detachInterrupt(digitalPinToInterrupt(Eye));
  attachInterrupt(digitalPinToInterrupt(Toggle), ToggleISR, LOW);
}

void FurnaceISR () {             // Furnace OnButton has gone low
  Serial.print("FurnaceISR ");
  Serial.println(digitalRead(Furnace.OffButton));
  digitalWrite(Furnace.Pin, Furnace.OffButton); // If Furnace.OffButton has also gone low, turn off furnace, else turn on Furnace
  Furnace.OnTime = millis();                    //set the timer either way.
}

void ToggleISR () {
  if (gate_position == OpenPosition) CloseISR;
  else OpenISR;
}

void RadioISR() {
  noInterrupts();
  if (radio.available()) {
    Serial.println("Radio Available");
    int radio_count = 1000;
    while (radio.available() && radio_count) {
      radio.read(&radio_command, sizeof(radio_command));
      radio_count--;
    }
    Serial.println(radio_command);
    switch (radio_command) {
      case 1:
        OpenISR();
        break;
      case 2:
        CloseISR();
        break;
      case 3:
        OpenCloseISR();
        break;
      case 4:
        ToggleISR();
        break;
      case 5:
        ExteriorLight = StartLightTimer(ExteriorLight);
        break;
      case 6:
        InteriorLight = StartLightTimer(InteriorLight);
        break;
      case 7:
        ExteriorLight = StartLightTimer(ExteriorLight);
        InteriorLight = StartLightTimer(InteriorLight);
        break;
      case 8:
        digitalWrite(InteriorLight.pin, HIGH);
        break;
      case 9:
        digitalWrite(InteriorLight.pin, LOW);
        break;
      case 10:
        digitalWrite(ExteriorLight.pin, HIGH);
        break;
      case 11:
        digitalWrite(ExteriorLight.pin, LOW);
        break;
      case 12:
        digitalWrite(InteriorLight.pin, HIGH);
        digitalWrite(ExteriorLight.pin, HIGH);
        break;
      case 13:
        digitalWrite(InteriorLight.pin, LOW);
        digitalWrite(ExteriorLight.pin, LOW);
        break;
      case 14:
        digitalWrite(Furnace.Pin, LOW);
        break;
      case 15:
        digitalWrite(Furnace.Pin, HIGH);
        Furnace.OnTime = millis();
    }
  }
  interrupts();
}

void OpenISR() {
  Serial.println("Open ISR");
  termination = Cancel_Button;
  OpenCloseMode.WaitingToClose = false;
  command = Open;
}

void CloseISR() {
  Serial.println("Close ISR");
  termination = Cancel_Button;
  OpenCloseMode.WaitingToClose = false;
  command = Close;
}

void OpenCloseISR() {
  Serial.println("Open-Close ISR");
  termination = Cancel_Button;
  OpenCloseMode.WaitingToClose = true;
  OpenCloseMode.OpenTime = millis();
  command = Open;
}

void EyeISR() {
  Serial.println("Eye ISR");
  AttachInterrupts();       //////// Disable the Eye ISR, but attach other interrupts.
  delay(3000);
  termination = Eye_term;
  if (command == Retry2) command = Open;
  if (command == Retry1) command = Retry2;
  if (command == Close) command = Retry1;
}

//////////////////////////////Close Gate/////////////////////////
void CloseGate() {
  Serial.println("Close Gate");
  gate_position = ClosedPosition;
  detachInterrupt(digitalPinToInterrupt(CloseButton));                                    ///When closing, the Eye is active and the Close Button is inactive
  attachInterrupt(digitalPinToInterrupt(Eye), EyeISR, LOW);
  digitalWrite(Eye_Enable, HIGH);
  digitalWrite(Direction, !Open_Dir);
  digitalWrite(ClosingLED, HIGH);
  Move_Gate(CloseSwitch, FineTune_Close);
  digitalWrite(Eye_Enable, LOW);
  AttachInterrupts();
}

/////////////////////////Open Gate////////////////////////////
void OpenGate() {
  Serial.println("Open Gate");
  gate_position = OpenPosition;
  detachInterrupt(digitalPinToInterrupt(OpenButton));                                    ////////When opening, the Open and Open/Close buttons are inactive. The Eye remains inactive.
  detachInterrupt(digitalPinToInterrupt(OpenCloseButton));
  digitalWrite (Direction, Open_Dir);
  digitalWrite (OpeningLED, HIGH);
  Move_Gate(OpenSwitch, FineTune_Open);
  AttachInterrupts();
}

//////////////////////Initialize Radio/////////////////////////
void InitializeRadio() {
  radio.begin();
  radio.setPayloadSize(sizeof(radio_command));
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.startListening();
}
