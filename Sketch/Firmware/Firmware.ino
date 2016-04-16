/*
  FilaEx FW01

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

// include the library code:
#include <LiquidCrystal.h>
#include <Bounce2.h>
#include <PID_v1.h>


//#define DEBUGMODE 1

#define FW_VER "V1.0.1 - Rapps"
#define ABS_TEMP 220
#define PLA_TEMP 180
#define PET_TEMP 220

#define THERMISTOR_PIN A5
#define MOTOR_PIN 5
#define HEATER_PIN 3

#define BUTTON_UP A4
#define BUTTON_DOWN A1
#define MENU_SWITCH 2

#define TEMP_SAMPLES 10
#define MAX_TEMP 280
#define MIN_TEMP_DIFF_MOTOR 5
#define MIN_EXTRUSION_TEMP 160

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 7, 8, 9, 10);

// PID
double Input, Output;
double Setpoint;
int NewSetpoint;
//Define the aggressive and conservative Tuning Parameters
double aggKp = 2400, aggKi = 0, aggKd = 0;
double consKp = 1200, consKi = 0.00, consKd = 0.00;
//Timer
int WindowSize = 5000;
unsigned long windowStartTime;

//Specify the links and initial tuning parameters
PID heaterPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);



Bounce btup = Bounce();
Bounce btdw = Bounce();
Bounce btmenu = Bounce();

int debugTemp = 160;
int incrementRate = 100;
int tempSet = 0;
int rpmSet = 255;
long timertemp = 0;
long btmillis = 500;
int actualTemp = 0;
int sample = 0;


int tempArray[TEMP_SAMPLES];

bool heaterOn = false;
bool motorOn = false;

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

void setup() {
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  pinMode(BUTTON_UP, INPUT);
  digitalWrite(BUTTON_UP, HIGH);

  pinMode(BUTTON_DOWN, INPUT);
  digitalWrite(BUTTON_DOWN, HIGH);

  digitalWrite(HEATER_PIN, heaterOn);
  digitalWrite(MOTOR_PIN, motorOn);
  Serial.begin(9600);

  btup.attach(BUTTON_UP);
  btup.interval(50);

  btdw.attach(BUTTON_DOWN);
  btdw.interval(50);

  btmenu.attach(MENU_SWITCH);
  btmenu.interval(50);


  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  
  lcd.print("FilaExt");
  lcd.setCursor(0, 1);
  lcd.print(FW_VER);
  delay(1200);
  
  lcd.clear();
  PIDsetup();

}

void loop() {
  btup.update ( );
  btdw.update ( );
  btmenu.update ( );
  char incomingByte;
  
  #ifdef DEBUGMODE
    char bufferc[] = {' ',' ',' ',' ',' '};
    if (Serial.available())
    {
        Serial.readBytesUntil('\n', bufferc, 5);
        debugTemp = constrain(atoi(bufferc),0,MAX_TEMP);
    }
  #endif
  
  AcquireTempSamples();

  if ( btup.read()  == 0 && btmenu.read() == 0 )
    button_up();
  else if ( btup.read() == 0 && btmenu.read() == 1 )
    rpm_up();

  if ( btdw.read()  == 0 && btmenu.read() == 0)
    button_down();
  else if ( btdw.read() == 0 && btmenu.read() == 1 )
    rpm_down();

  if (  btmenu.read() == 0 )
    SetTemp();
  else
    RpmControl();

  RunPID();

  if ( motorOn )
    analogWrite(MOTOR_PIN, rpmSet);
  else
    digitalWrite(MOTOR_PIN, LOW);

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
    Serial.println(data);
    //delay(400);
  #endif
}

void SetTemp()
{
  if ( sample >= TEMP_SAMPLES )
  {
    lcd.setCursor(0, 0);
    actualTemp = getTemp();
    String tempStr = String("Temp ");
    tempStr += actualTemp ;
    tempStr += "/" +  String(tempSet);

    
    lcd.print(tempStr);
    lcd.print("        ");
    
    lcd.setCursor(0, 1);
    lcd.print("Heat-");
    lcd.print(( heaterOn ? "On" : "Off"));
    
    lcd.print(" Mot-");
    lcd.print(( motorOn ? "On" : "Off"));
    lcd.print("        ");
    
    if ( abs(tempSet - actualTemp) > MIN_TEMP_DIFF_MOTOR) 
        motorOn = false;
    else if ( ( abs(tempSet - actualTemp)  <= MIN_TEMP_DIFF_MOTOR && actualTemp > MIN_EXTRUSION_TEMP  ))
        motorOn = true;
        
    sample = 0;
  }
}


void AcquireTempSamples()
{
  if (sample < TEMP_SAMPLES )
  {
    tempArray[sample] = read_temp();
    sample++;
    delay(30);
  }
}

int getTemp()
{
  int auxTemp = 0;
  for ( int i = 0; i < TEMP_SAMPLES; i++)
  {
    auxTemp += tempArray[i];
  }

  #ifdef DEBUGMODE
    return debugTemp;
  #else
    return auxTemp / TEMP_SAMPLES;
  #endif
}

void button_up()
{
  if ( millis() -  btmillis > incrementRate && tempSet < MAX_TEMP )
  {
    tempSet += 5;
    btmillis = millis();
  }
}

void button_down()
{
  if (  millis() - btmillis > incrementRate && tempSet > 0 )
  {
    tempSet -= 5;
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
  
  for (int i = 0; i < TEMP_SAMPLES; i++)
    AquireTempSamples();

  windowStartTime = millis();
  heaterPID.SetOutputLimits(0, WindowSize);//tell the PID to range between 0 and the full window size
  Input = getTemp();
  printDebugData("Temp: " + String(Input));
  heaterPID.SetMode(AUTOMATIC);



}
