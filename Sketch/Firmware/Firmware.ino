

/*
  FilaEx FW02

  Raphael Toselli

  This program is free software: you can redistribute it and/or modify
  it under the terms of the version 3 GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Libraries:
#include <LiquidCrystal.h>
#include <Bounce2.h>
#include <PID_v1.h>
#include <Encoder.h>

//#define DEBUGMODE 1

#define FW_VER "V1.3.6 - Encoder"
#define ABS_TEMP 220
#define PLA_TEMP 180
#define PET_TEMP 220

#define THERMISTOR_PIN A0
#define MOTOR_PIN 3
#define HEATER_PIN 5
#define ENCODER0_SW A3
#define ENCODER0_A A1
#define ENCODER0_B A2

#define TEMP_SAMPLES 10
#define MAX_TEMP 280
#define MIN_TEMP_DIFF_MOTOR 15
#define MIN_EXTRUSION_TEMP 160

#define NUMTEMPS 20
short temptable[NUMTEMPS][2] = {
  {1, 841},
  {54, 255},
  {107, 209},
  {160, 184},
  {213, 166},
  {266, 153},
  {319, 142},
  {372, 132},
  {425, 124},
  {478, 116},
  {531, 108},
  {584, 101},
  {637, 93},
  {690, 86},
  {743, 78},
  {796, 70},
  {849, 61},
  {902, 50},
  {955, 34},
  {1008, 3}
};



// Globals 

int debugTemp = 165;
int incrementRate = 5; 
int tempSet = 0;
int rpmSet = 255;
int actualTemp = 0;
int sample = 0;
int tempArray[TEMP_SAMPLES];
int EncSWLastState = 1;
int animState = 0;
long timertemp = 0;
long btmillis = 500;
long encoder0OldPos  = -999;
bool heaterOn = false;
bool motorOn = false;
bool menuState = false;




// PID
double Input, Output;
double Setpoint;
int NewSetpoint;
//Define the aggressive and conservative Tuning Parameters
double aggKp = 2400, aggKi = 0, aggKd = 0;
double consKp = 1200, consKi = 0.00, consKd = 0.00;
//Timer
int WindowSize = 400;
unsigned long windowStartTime;

//Specify the links and initial tuning parameters
PID heaterPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 9, 8, 7, 6);

Bounce btEncoder = Bounce();

Encoder E0(ENCODER0_A, ENCODER0_B);

void setup() {
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  btEncoder.attach(ENCODER0_SW);
  btEncoder.interval(100);

  pinMode(ENCODER0_SW, INPUT);
  digitalWrite(ENCODER0_SW, HIGH);

  pinMode(ENCODER0_A, INPUT);
  digitalWrite(ENCODER0_A, HIGH);

  pinMode(ENCODER0_B, INPUT);
  digitalWrite(ENCODER0_B, HIGH);

  digitalWrite(HEATER_PIN, heaterOn);
  digitalWrite(MOTOR_PIN, motorOn);
  Serial.begin(9600);
  



  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  lcd.print("FilaExt");
  lcd.setCursor(0, 1);
  lcd.print(FW_VER);
  delay(1200);
  lcd.clear();
  windowStartTime = millis();
  PIDsetup();

  Serial.print("OK");

}

void loop() {
  
  btEncoder.update();
  
  long newPosition = E0.read();
  if ( newPosition !=  encoder0OldPos ) { 
    
    if ( menuState == false )
    {
      if (newPosition >  encoder0OldPos) 
        temp_up();
      else 
        temp_down();
    }
    else if ( menuState == true ) 
    {
      if ( newPosition >  encoder0OldPos ) 
        rpm_up();
      else 
        rpm_down();
    
    }
  encoder0OldPos = newPosition;
}

  #ifdef DEBUGMODE
    char incomingByte;
    char bufferc[] = {' ',' ',' ',' ',' '};
    if (Serial.available())
    {
        Serial.readBytesUntil('\n', bufferc, 5);
        debugTemp = constrain(atoi(bufferc),0,MAX_TEMP);
    }
  #endif
  
  AquireTempSamples();

  
  if ( menuState == 0 )
    SetTemp();
  else
    RpmControl();

  RunPID();

  if ( motorOn )
    analogWrite(MOTOR_PIN, rpmSet);
  else
    digitalWrite(MOTOR_PIN, LOW);

  int SWState =  btEncoder.read();
  
  if ( SWState == 0 && EncSWLastState != SWState )
  {
    menuState = !menuState;
    EncSWLastState = SWState; 
    
  } else {
    
    EncSWLastState = 1;
  }

    
}

void RunPID()
{
  Setpoint = tempSet;
  Input = actualTemp;
  
  double gap = abs(Setpoint - Input); //distance away from setpoint
  printDebugData("Gap: " + String(gap));
  if (gap < 10)
  { //we're close to setpoint, use conservative tuning parameters
    printDebugData("PID MODE: Conservative");
    heaterPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
    printDebugData("PID MODE: Aggressive");
    //we're far from setpoint, use aggressive tuning parameters
    heaterPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
  heaterPID.Compute();
  
  unsigned long now = millis();
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }

  if(Output > now - windowStartTime) 
  {
    digitalWrite(HEATER_PIN,HIGH);
    heaterOn = true ;
  }
  else 
  {
    digitalWrite(HEATER_PIN,LOW);
    heaterOn = false;
  }

}

void RpmControl()
{

  lcd.setCursor(0, 0);
  lcd.print("Motor PWM");
  lcd.print("        ");
  lcd.setCursor(0, 1);
  lcd.print((rpmSet * 100) / 255);
  lcd.print("%                ");

}

void printDebugData(String data)
{
  #ifdef DEBUGMODE
    //Serial.println(data);
    //delay(400);
  #endif
}

void SetTemp()
{



  if ( sample >= TEMP_SAMPLES && abs(millis() - timertemp) > 100)
  {
    lcd.setCursor(0, 0);
    actualTemp = getTemp();
    String tempStr = String("Temp ");
    tempStr += actualTemp ;
    tempStr += "/" +  String(tempSet);
    //module.clearDisplay();
    lcd.print(tempStr);
    lcd.print("        ");
    lcd.setCursor(0, 1);
    char c = animate();
    lcd.print("Heat:");
    lcd.print(( heaterOn ? c : '-'));
    lcd.print(" Mot:");
    lcd.print(( motorOn ? c : '-'));
    lcd.print("        ");
    if ( abs(tempSet - actualTemp) > MIN_TEMP_DIFF_MOTOR || actualTemp < MIN_EXTRUSION_TEMP ) {
      motorOn = false;
    }
    else if ( ( abs(tempSet - actualTemp)  <= MIN_TEMP_DIFF_MOTOR && actualTemp >= MIN_EXTRUSION_TEMP ))
    {
      motorOn = true;
    }
    sample = 0;
    timertemp = millis();
  }

}


void AquireTempSamples()
{

  if (sample < TEMP_SAMPLES )
  {
    tempArray[sample] = read_temp();
    sample++;
  }

}


char animate()
{
  char c;
  switch(animState)
  {
    case 0:
          c = '|';
          break;
    case 1:
          c = '/';
          break;
    case 2:
          c = '-';
          break;
    case 3:
          c = 92;
          animState = -1;
          break;
          
  }
  animState++;
  return c;
  
}
int getTemp()
{
  int auxTemp = 0;
  for ( int i = 0; i < TEMP_SAMPLES; i++)
  {
    //Serial.println(tempArray[i]);
    auxTemp += tempArray[i];
  }

  /*Serial.print("T:");
    Serial.println(auxTemp / TEMP_SAMPLES);*/

#ifdef DEBUGMODE
  return debugTemp;
#else
  return auxTemp / TEMP_SAMPLES;
#endif

}
void temp_up()
{

  if ( millis() -  btmillis > incrementRate && tempSet < MAX_TEMP )
  {
    tempSet += 1;
    btmillis = millis();
  }
}

void temp_down()
{
  if (  millis() - btmillis > incrementRate && tempSet > 0 )
  {
    tempSet -= 1;
    btmillis = millis();
  }
}


void rpm_down()
{
  if (  millis() - btmillis > incrementRate && rpmSet > 0 )
  {
    rpmSet -= 5;
    btmillis = millis();
  }
}


void rpm_up()
{

  if ( millis() -  btmillis > incrementRate && rpmSet < 255 )
  {
    rpmSet += 5;
    btmillis = millis();
  }
}


int read_temp()
{
  int rawtemp = analogRead(THERMISTOR_PIN);
  int current_celsius = 0;

  byte i;
  for (i = 1; i < NUMTEMPS; i++)
  {
    if (temptable[i][0] > rawtemp)
    {
      int realtemp  = temptable[i - 1][1] + (rawtemp - temptable[i - 1][0]) * (temptable[i][1] - temptable[i - 1][1]) / (temptable[i][0] - temptable[i - 1][0]);

      if (realtemp > 255)
        realtemp = 255;

      current_celsius = realtemp;

      break;
    }
  }

  // Overflow: We just clamp to 0 degrees celsius
  if (i == NUMTEMPS)
    current_celsius = 0;

  return current_celsius;
}

void PIDsetup() {
  lcd.setCursor(0, 0);
  lcd.print("  Configurando ");
  lcd.setCursor(0, 1);
  lcd.print("     P I D    ");
  delay(1000);
  //analogReference(EXTERNAL);
  for (int i = 0; i < TEMP_SAMPLES; i++)
    AquireTempSamples();

  //Timer
  windowStartTime = millis();

  heaterPID.SetOutputLimits(0, WindowSize);//tell the PID to range between 0 and the full window size
  //inizializzo le variabili
  Input = getTemp();
  printDebugData("Temp: " + String(Input));
  heaterPID.SetMode(AUTOMATIC);
}

