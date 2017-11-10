
/* This one is not using any PinChangeInterrupt library */

/*
   This program uses an Arduino for a closed-loop control of a DC-motor.
   Motor motion is detected by a quadrature encoder.
   Two inputs named STEP and DIR allow changing the target position.
   Serial port prints current position and target position every second.
   Serial input can be used to feed a new location for the servo (no CR LF).

   Pins used:
   Digital inputs 2 & 8 are connected to the two encoder signals (AB). (PD2) (PB0)
   Digital input 3 is the STEP input. (PD3) YOU NEED CHANGE IT IN INTERRUPT ROUTINES
   Analog input A0 is the DIR input. (PC0) YOU NEED CHANGE IT IN INTERRUPT ROUTINES
   Digital outputs 9 & 10 control the PWM outputs for the motor (I am using half L298 here). (PB1) (PB2)


   Please note PID gains kp, ki, kd need to be tuned to each different setup.
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <PID_v1.h>


#include <ESP8266WiFi.h>
#include "FS.h"
#include <WiFiClient.h>
#include <Hash.h>
#include <TimeLib.h>
#include <NtpClientLib.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP8266mDNS.h>
// #include <Ticker.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "FSWebServerLib.h"

#include "pwm.c" // https://github.com/thierer/ESP8266_new_pwm/tree/fix_boundary_check
// #define encoder0PinA  2 // PD2; YOU NEED CHANGE IT IN INTERRUPT ROUTINES
// #define encoder0PinB  8  // PB0; YOU NEED CHANGE IT IN INTERRUPT ROUTINES

// #define LED           13  // onboard LED

#define chartSize 500 // NO of samples for chart

// Do not use GPIO15 for PWM
const int encoder0PinA = 13;
const int encoder0PinB = 12;
const int Step = 14;
const int M1=2; // D4 H bridge DIR
const int M2=4; // D2 H bridge PWM
const int DIR=0; //0
// const int PWM_MOT=15;

/* Configure new_pwm */
#define PWM_CHANNELS 2
uint32_t pwmPeriod = 50000; // 5000 * 200ns ^= 1 kHz
uint32_t pwmPrescale = 200;

// PWM setup
uint32 io_info[PWM_CHANNELS][3] = {
	// MUX, FUNC, PIN
	{PERIPHS_IO_MUX_GPIO4_U,  FUNC_GPIO4, 4},
	{PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5 ,  5},
  /*MUX_REGISTERS
GPIO0: PERIPHS_IO_MUX_GPIO0_U
GPIO1: PERIPHS_IO_MUX_U0TXD_U
GPIO2: PERIPHS_IO_MUX_GPIO2_U
GPIO3: PERIPHS_IO_MUX_U0RXD_U
GPIO4: PERIPHS_IO_MUX_GPIO4_U
GPIO5: PERIPHS_IO_MUX_GPIO5_U
GPIO6: PERIPHS_IO_MUX_SD_CLK_U
GPIO7: PERIPHS_IO_MUX_SD_DATA0_U
GPIO8: PERIPHS_IO_MUX_SD_DATA1_U
GPIO9: PERIPHS_IO_MUX_SD_DATA2_U
GPIO10: PERIPHS_IO_MUX_SD_DATA3_U
GPIO11: PERIPHS_IO_MUX_SD_CMD_U
GPIO12: PERIPHS_IO_MUX_MTDI_U
GPIO13: PERIPHS_IO_MUX_MTCK_U
GPIO14: PERIPHS_IO_MUX_MTMS_U
GPIO15: PERIPHS_IO_MUX_MTDO_U*/
};

// initial duty: all off
uint32 pwm_duty_init[PWM_CHANNELS] = {0, 0};
double pwmOld;

/* Configure PID */
int chartSamples[chartSize]; int p = 0; // Samples for draw runing
//PID
//double kp = 5, ki = 0.0, kd = 0.08;
double kp = 5, ki = 1.23, kd = 0.02;
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
bool onPosition = false;

// speed loop
double feed = 200000;
double motor = 0, setspeed = 100, vel = 0;
double vkp = 1, vki = 1, vkd = 0;
PID speed(&vel, &motor, &setspeed, vkp, vki, vkd, DIRECT);
float accel = 200000.0; // desired acceleration in mm/s^2

volatile long encoder0Pos = 0;
volatile int directionLast = -1;
boolean auto1 = false, auto2 = false, counting = false , safeguard = false;
long previousMillis = 0;        // will store last time LED was updated
long lastEncPos = 0;          // will store the target value when last max output was measured
unsigned long curTime = 0UL;  // will store current time to avoid multiple millis() calls
unsigned long lastSafeCheck = 0;  // will store last value when 255 output was measured
unsigned long motorSafe = 2000UL; // will store the interval to protect the motor - 2seconds
int lastMax = 0;              // have to also store the max to avoid +-255 values


long target1 = 0; // destination location at any moment

//for motor control ramps 1.4
bool newStep = false;
bool oldStep = false;
bool dir = false;
int skip = 0;

// void toggle() {
//   static int state = 0;
//   state = !state;
//   digitalWrite(BUILTIN_LED, state);
// }

/*
// PWM_L | PWM_R
void pwmOut(int out) {
   if(out<0) { analogWrite(M1,0); analogWrite(M2,abs(out)); }
   else { analogWrite(M2,0); analogWrite(M1,abs(out)); }
 }
*/
// PWM | DIR
 /*void pwmOut(int out) {
    if(out<0) { digitalWrite(M1,0); analogWrite(M2,abs(out)); }
    else { digitalWrite(M1,1); analogWrite(M2,abs(out)); }
  }*/
void pwmOut(int out) {
  if (pwmOld != out) {
    if(out<0) { digitalWrite(M1,0); pwm_set_duty(abs(out), 0); }
    else { digitalWrite(M1,1); pwm_set_duty(abs(out), 0); }
    pwm_start();           // commit
    pwmOld = out;
    if (auto2) {
      Serial.print(F(" Out="));
      Serial.println(abs(out));
    }
  }
}

/*
  void pwmOut(int out) {
   if(out>0) { digitalWrite(M1,0); digitalWrite(M2,1); }
   else      { digitalWrite(M1,1); digitalWrite(M2,0); }
  //  analogWrite(9,abs(out));
   //PWM = out;
  }
*/

/* Motor safe code */
void motorProtect() {
  if (abs(output) == 255) {
    if (lastSafeCheck == 0) {
      lastSafeCheck = curTime;
      lastEncPos = encoder0Pos;
    } else {
      if (lastEncPos != encoder0Pos) {
        lastSafeCheck = 0;
      } else {
        if (curTime - motorSafe > lastSafeCheck) {
          // we have to protect the motor - looks like even with max output we are not moving
          // we will set the target to current position to stop output
          Serial.println(F("Will decrease output!!!"));
          target1 = encoder0Pos - (output / 255 * 5);
          lastEncPos = 0;
          lastSafeCheck = 0;
        }
      }
    }
  } else {
    lastSafeCheck = 0;
    lastEncPos = 0;
  }
}

void motion() {
  // curTime = millis();
  vel =  encoder0Pos - input;
  input = encoder0Pos;
  setpoint = target1;
  while (!myPID.Compute()); // wait till PID is actually computed
  setspeed = output;
  //  pwmOut(output);
  /*  if (counting &&  (skip++ % 10) == 0 ) {
      chartSamples[p] = encoder0Pos;
      if (p < chartSize - 1) p++;
      else counting = false;
    }*/
  //if(counting && abs(input-target1)<15) counting=false;
  if (  speed.Compute() && counting ) { // Collect samples for chart //only sample when PID updates
    chartSamples[p] = encoder0Pos;
    if (p < chartSize - 1) p++;
    else counting = false;
  }
  pwmOut(motor*pwmPrescale);
  // analogWrite(, abs(output));
  if (abs(input - target1) < 15)
    onPosition = true;
  else
    onPosition = false;
  if (safeguard) motorProtect();

}

void trapezoidal(int destination) { // it will use acceleration and feed values to restrict the motion following a trapezoidal pattern
  long a1 = millis();
  int distance = destination - setpoint; // if positive go +x
  int finalPoint = setpoint + distance;
  boolean dirPos = true;
  if (distance < 0) {
    distance = -distance;  // me quedo con el valor absoluto del movimiento
    dirPos = false;
  }
  float xm = feed * feed / accel;
  float t1, t2;
  if (distance <= xm) t1 = t2 = sqrt(distance / accel); // triangular
  else { // trapezoidal
    t1 = sqrt(xm / accel); // t1 = end of accel
    t2 = (distance - xm) / feed + t1; // t2 = end of coasting
  }
  // Ok, I know what to do next, so let's perform the actual motion
  float t = 0, spd = 0.0;
  float dt = 1e-3;
  float da = accel * dt;
  float covered = setpoint;
  float maxt = t1 + t2;
  while (t < maxt) {
    t += dt;
    if (t < t1) spd += da; else if (t >= t2) spd -= da;
    if ( dirPos ) covered += spd * dt; else covered -= spd * dt; // calculate new target position
    //vel =  encoder0Pos - input;
    input = encoder0Pos;
    setpoint = covered;
    while (!myPID.Compute()) yield(); // espero a que termine el cálculo
    setspeed = output;
    //speed.Compute();
    pwmOut( output );
    //digitalWrite(0,1-digitalRead(0));; just for time tracing purposes
    // record data for S command
    if ( counting  ) { // Collect samples for chart
      chartSamples[p] = encoder0Pos;
      if (p < chartSize - 1) p++;
      else counting = false;
    }
  }
  Serial.print(millis() - a1); Serial.print(F("  Err=")); Serial.println(encoder0Pos - covered);
  target1 = covered;
}

/* INTERRUPTS */
const int QEM [16] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0}; // Quadrature Encoder Matrix
static unsigned char New, Old;

void encoderInt() { // handle pin change interrupt for D2  // encoder0PinA, YOU NEED CHANGE PINS IN THE CODE BELLOW
  Old = New;
  //New = PIND & 3; //(PINB & 1 )+ ((PIND & 4) >> 1); //   Mauro Manco
  New = digitalRead(encoder0PinA)*2 + digitalRead(encoder0PinB);
  encoder0Pos+= QEM [Old * 4 + New];
  // Must clear this bit in the interrupt register, ??
  // it gets set even when interrupts are disabled
  GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << encoder0PinA);
  GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << encoder0PinB);
}

void countStep(){
  if (digitalRead(DIR)== HIGH) target1--;else target1++;
  // Must clear this bit in the interrupt register, ??
  // it gets set even when interrupts are disabled
  GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, 1 << Step);
} // pin A0 represents direction == PF7 en Pro Micro

/* Load save settings */
void eeput(double value, int dir) { // Snow Leopard keeps me grounded to 1.0.6 Arduino, so I have to do this :-(
  char * addr = (char * ) &value;
  for (int i = dir; i < dir + 4; i++)  EEPROM.write(i, addr[i - dir]);
}

void writetoEEPROM() { // keep PID set values in EEPROM so they are kept when arduino goes off
  eeput(kp, 0);
  eeput(ki, 4);
  eeput(kd, 8);
  double cks = 0;
  for (int i = 0; i < 12; i++) cks += EEPROM.read(i);
  eeput(cks, 12);
  Serial.println("\nPID values stored to EEPROM");
  //Serial.println(cks);
}

double eeget(int dir) { // Snow Leopard keeps me grounded to 1.0.6 Arduino, so I have to do this :-(
  double value;
  char * addr = (char * ) &value;
  for (int i = dir; i < dir + 4; i++) addr[i - dir] = EEPROM.read(i);
  return value;
}

void recoverPIDfromEEPROM() {
  double cks = 0;
  double cksEE;
  for (int i = 0; i < 12; i++) cks += EEPROM.read(i);
  cksEE = eeget(12);
  //Serial.println(cks);
  if (cks == cksEE) {
    Serial.println(F("*** Found PID values on EEPROM"));
    kp = eeget(0);
    ki = eeget(4);
    kd = eeget(8);
    myPID.SetTunings(kp, ki, kd);
  }
  else Serial.println(F("*** Bad checksum"));
}

void eedump() {
  for (int i = 0; i < 16; i++) {
    Serial.print(EEPROM.read(i), HEX);
    Serial.print(" ");
  } Serial.println();
}

/* Serial Communication */
void printPos() {
  Serial.print(F("Position="));
  Serial.print(encoder0Pos);
  Serial.print(F(" PID_output="));
  Serial.print(output);
  Serial.print(F(" PID_motor="));
  Serial.print(motor);
  Serial.print(F(" Target="));
  Serial.print(setpoint);
  Serial.print(F(" LastCheck="));
  Serial.print(lastSafeCheck);
  Serial.print(F(" LastEnc="));
  Serial.println(lastEncPos);

}

void help() {
  Serial.println(F("\nPID DC motor controller and stepper interface emulator"));
  Serial.println(F("by misan"));
  Serial.println(F("Available serial commands: (lines end with CRLF or LF)"));
  Serial.println(F("P123.34 sets proportional term to 123.34"));
  Serial.println(F("I123.34 sets integral term to 123.34"));
  Serial.println(F("D123.34 sets derivative term to 123.34"));
  Serial.println(F("? prints out current encoder, output and setpoint values"));
  Serial.println(F("X123 sets the target destination for the motor to 123 encoder pulses"));
  Serial.println(F("T will start a sequence of random destinations (between 0 and 2000) every 3 seconds. T again will disable that"));
  Serial.println(F("Q will print out the current values of P, I and D parameters"));
  Serial.println(F("W will store current values of P, I and D parameters into EEPROM"));
  Serial.println(F("H will print this help message again"));
  Serial.println(F("A will toggle on/off showing regulator status every second"));
  Serial.println(F("M will toggle on/off decreasing motor power for protection when running at 100% for some time"));
  Serial.println(F("F sets desired motion speed"));
  Serial.println(F("V sets speed proportional gain"));
  Serial.println(F("G sets speed integral gain"));
  Serial.println(F("Y123.34 it is like X but using trapezoidal motion"));
  Serial.println(F("@123.34 sets [trapezoidal] acceleration"));
  Serial.println(F("Z disables STEP input\n"));
}

// TODO: rewrite to process whole lines in case of human entering data char by char
void process_line() {
  char cmd = Serial.read();
  if (cmd > 'Z') cmd -= 32;
  switch (cmd) {
    case 'P': kp = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case 'D': kd = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case 'I': ki = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case '?': printPos(); break;
    case 'X': target1 = Serial.parseInt(); p = 0; counting = true; for (int i = 0; i < chartSize; i++) chartSamples[i] = 0; break;
    case 'T': auto1 = !auto1; break;
    case 'A': auto2 = !auto2; break;
    case 'M': safeguard = !safeguard; break;
    case 'Q': Serial.print("P="); Serial.print(kp); Serial.print(" I="); Serial.print(ki); Serial.print(" D="); Serial.print(kd); Serial.print(" VP="); Serial.print(vkp); Serial.print(" VI="); Serial.println(vki); break;
    case 'H': help(); break;
    case 'W': writetoEEPROM(); break;
    case 'K': eedump(); break;
    case 'R': recoverPIDfromEEPROM() ; break;
    case 'S': for (int i = 0; i < p; i++) Serial.println(chartSamples[i]); break; // Send chart data
    case 'Z': detachInterrupt(Step); break; // from then on, ignore step pulses (good for tests)
    case 'F': feed = Serial.parseFloat(); break;
    case 'V': vkp = Serial.parseFloat(); speed.SetTunings(vkp, vki, vkd); break;
    case 'G': vki = Serial.parseFloat(); speed.SetTunings(vkp, vki, vkd); break;
    case 'Y': counting = true; for (int i = 0; i < chartSize; i++) chartSamples[i] = 0; p = 0; trapezoidal(Serial.parseInt()); break; // performs a trapezoidal move
    case '@': accel = Serial.parseFloat(); break;
  }
  // while (Serial.read() != 10) {yield()}; // dump extra characters till LF is seen (you can use CRLF or just LF)
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(Step, INPUT_PULLUP); // Configure pin 3 as input for STEP
  pinMode(DIR, INPUT_PULLUP); // Configure pin A0 as input for DIR

/* PWM*/
// Legacy
/*  analogWriteFreq(20000);  // set PWM to 20Khz
  analogWriteRange(255);   // set PWM to 255 levels (not sure if more is better)*/
// new_pwm
  pwm_init(pwmPeriod, pwm_duty_init, PWM_CHANNELS, io_info);
  pwm_start();

  attachInterrupt(encoder0PinA, encoderInt, CHANGE);
  attachInterrupt(encoder0PinB, encoderInt, CHANGE);
  attachInterrupt(Step, countStep, RISING);
  // toggle();

  // WiFi is started inside library
  SPIFFS.begin(); // Not really needed, checked inside library and started if needed
  ESPHTTPServer.begin(&SPIFFS);

  // prepare web handlers
auto handler_pid_values = ESPHTTPServer.on("/pid_values", HTTP_GET, [](AsyncWebServerRequest *request) {
  for (uint8_t i = 0; i < request->args(); i++) {
    DEBUGLOG("Arg %d: %s\r\n", i, request->arg(i).c_str());
		Serial.print(request->argName(i));
		Serial.print(" : ");
		Serial.println(ESPHTTPServer.urldecode(request->arg(i)));
    if (request->argName(i) == "enable") {
		}
		if (request->argName(i) == "kp") {
			kp = request->arg(i).toFloat();
			myPID.SetTunings(kp, ki, kd);
		}
		if (request->argName(i) == "ki") {
			ki = request->arg(i).toFloat();
			myPID.SetTunings(kp, ki, kd);
		}
		if (request->argName(i) == "kd") {
			kd = request->arg(i).toFloat();
			myPID.SetTunings(kp, ki, kd);
		}
		if (request->argName(i) == "vkp") {
			vkp = request->arg(i).toFloat();
			speed.SetTunings(vkp, vki, vkd);
		}
		if (request->argName(i) == "vki") {
			vki = request->arg(i).toFloat();
			speed.SetTunings(vkp, vki, vkd);
		}
		if (request->argName(i) == "vkd") {
			vkd = request->arg(i).toFloat();
			speed.SetTunings(vkp, vki, vkd);
		}
	}
	String values = "";
	values = "kp|" + (String)kp + "|input\n";
	values += "ki|" + String(ki,4) + "|input\n";
	values += "kd|" + String(kd,5) + "|input\n";
	values += "vkp|" + (String)vkp + "|input\n";
	values += "vki|" + String(vki,5) + "|input\n";
	values += "vkd|" + String(vkd,4) + "|input\n";
	request->send(200, "text/plain", values);
});
auto handler_stats_values = ESPHTTPServer.on("/motion_values", HTTP_GET, [](AsyncWebServerRequest *request) {
  for (uint8_t i = 0; i < request->args(); i++) {
    DEBUGLOG("Arg %d: %s\r\n", i, request->arg(i).c_str());
		Serial.print(request->argName(i));
		Serial.print(" : ");
		Serial.println(ESPHTTPServer.urldecode(request->arg(i)));
		if (request->argName(i) == "goto") {
			for (int i = 0; i < chartSize; i++) chartSamples[i] = 0;
			p = 0; counting = true;
			target1 = request->arg(i).toInt();
		}
		if (request->argName(i) == "move") {
			target1 += request->arg(i).toInt();
			for (int i = 0; i < chartSize; i++) chartSamples[i] = 0;
			p = 0; counting = true;
		}
		if (request->argName(i) == "gotoT") {
			counting = true;
			for (int i = 0; i < chartSize; i++) chartSamples[i] = 0;
			p = 0;
			trapezoidal(request->arg(i).toInt());
		}
		if (request->argName(i) == "moveT") {
			counting = true;
			for (int i = 0; i < chartSize; i++) chartSamples[i] = 0;
			p = 0;
			trapezoidal(target1 + request->arg(i).toInt());
		}
		if (request->argName(i) == "feed") {
			feed = request->arg(i).toFloat();
		}
		if (request->argName(i) == "accel") {
			accel = request->arg(i).toFloat();
		}
	}
	String values = "";
	values = "encoder0Pos|" + (String)encoder0Pos + "|input\n";
	values += "target1|" + (String)target1 + "|input\n";
  values += "setpoint|" + (String)setpoint + "|input\n";
	values += "output|" + (String)output + "|input\n";
	values += "motor|" + (String)motor + "|input\n";
	values += "feed|" + (String)feed + "|input\n";
	values += "accel|" + (String)accel + "|input\n";
	values += "lastSafeCheck|" + (String)lastSafeCheck + "|input\n";
	values += "lastEncPos|" + (String)lastEncPos + "|input\n";
	request->send(200, "text/plain", values);
});
auto handler_pwm_values = ESPHTTPServer.on("/pwm_values", HTTP_GET, [](AsyncWebServerRequest *request) {
  for (uint8_t i = 0; i < request->args(); i++) {
    DEBUGLOG("Arg %d: %s\r\n", i, request->arg(i).c_str());
		Serial.print(request->argName(i));
		Serial.print(" : ");
		Serial.println(ESPHTTPServer.urldecode(request->arg(i)));
		if (request->argName(i) == "pwmPeriod") {
			pwmPeriod = request->arg(i).toInt();
			pwm_init(pwmPeriod, pwm_duty_init, PWM_CHANNELS, io_info);
			pwm_start();
			myPID.SetOutputLimits(-(double)pwmPeriod/pwmPrescale, (double)pwmPeriod/pwmPrescale);
			speed.SetOutputLimits(-(double)pwmPeriod/pwmPrescale, (double)pwmPeriod/pwmPrescale);
		}
		if (request->argName(i) == "pwmPrescale") {
			pwmPrescale = request->arg(i).toInt();
			myPID.SetOutputLimits(-(double)pwmPeriod/pwmPrescale, (double)pwmPeriod/pwmPrescale);
			speed.SetOutputLimits(-(double)pwmPeriod/pwmPrescale, (double)pwmPeriod/pwmPrescale);
		}
	}
	String values = "";
	values = "pwmPeriod|" + (String)pwmPeriod + "|input\n";
	values += "pwmPrescale|" + (String)pwmPrescale + "|input\n";
	request->send(200, "text/plain", values);
});

  /* add setup code here */

  Serial.begin (115200);
  help();
  recoverPIDfromEEPROM();

  //Setup the pid
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-(double)pwmPeriod/pwmPrescale, (double)pwmPeriod/pwmPrescale);

  speed.SetMode(AUTOMATIC);
  speed.SetSampleTime(1);
  speed.SetOutputLimits(-(double)pwmPeriod/pwmPrescale, (double)pwmPeriod/pwmPrescale);

}

void loop() {
  curTime = millis();
  motion();
  if (Serial.available()) process_line(); // it may induce a glitch to move motion, so use it sparingly
  if (auto1) if (curTime % 3000 == 0) target1 = random(2000); // that was for self test with no input from main controller
  if (auto2) if (curTime % 1000 == 0) printPos();
  // DO NOT REMOVE. Attend OTA update from Arduino IDE
 // ESPHTTPServer.handle();
}
