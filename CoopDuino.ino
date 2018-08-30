
/********************************************************************/
//Libraries

//String Library for concat strings
#include <string.h>

//setup the I2C bus
#include <Wire.h>

// LCD Display
#include <LiquidCrystal_I2C.h>

//DHT 22 (Temp and RH) Sensor
#include <DHT.h>
// 1=vcc, 2=data, 3=nothing, 4=ground
//DS18b20 sensor
#include <OneWire.h>
#include <DallasTemperature.h>

//distance sensor 
#include <NewPing.h>

//Xbee wireless
//no libraries needed

//actuator
//nolibrary

//door relay
//no library

//LDR
//no library for LDR

// RTC using a DS1307 RTC connected via I2C and Wire lib
#include "RTClib.h"

/********************************************************************/
//Constants and init

// LCD Display
// uses I2C on pins A4 and A5 
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display
//SDA pin to the I2C data SDA pin on your Arduino. known as A4,
//SCL pin to the I2C clock SCL pin on your Arduino. known as A5

//RTC
// uses I2C on pins A4 and A5
DS1307 rtc;


//DHT 22 (Temp and RH) Sensor1
#define DHTPIN 6    // pin we're connected to
#define DHTTYPE DHT11   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

//DS18b20 sensor
#define ONE_WIRE_PIN 5 // Data wire is plugged into pin 5 on the Arduino
// Setup a oneWire instance to communicate with any OneWire devices  
OneWire oneWire(ONE_WIRE_PIN); 

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

//door switches
#define DoorOpenPIN 11 //GREEN
#define DoorClosedPIN 12 //RED

//door relay
int doorin = 7;
int doorout = 8;

//distance sensor 

#define DS0TRIGPIN    9
#define DS0ECHOPIN    10

//LDR
#define LDRPIN A3


/********************************************************************/
//Variables

int LoopWaitTime = (1*1000); //(in milliseconds There are 1000 milliseconds in a second.)

String nodeID = "SN0";  // I'm sensor node zero  
float SN0_RHIn;  //Stores humidity value
float SN0_tempIn; //Stores temperature value
float SN0_tempOut; //Stores outside temp
float SN0_tempWater; //Stores water temp
boolean SN0_dOpen; //Stores door open value
boolean SN0_dClosed; //Stores door closed value
boolean doorError = false; //indicates if door is errored or not
boolean doorBlocked = false; //indicates if door is blocked or not
int SN0_LDR; //Store LDR value

//internal timing variables
unsigned long prevStepMillis = 0;
unsigned long curMillis;
unsigned long millisBetweenSteps = 15; // milliseconds between steps
unsigned long currentMillis = millis(); //what time is it now?
unsigned long startMillis  = millis(); //when we start a task
unsigned long taskMillis = millis(); //time taken doing the task

uint8_t door_state; //door state FSM
uint8_t light_state; //light state FSM

//Distance sensor
long duration;
int distance;

//time
  int sleephour = 21;
  int wakehour = 6 ;
boolean FALSE = 0;
boolean TRUE = 1;
/********************************************************************/
void setup() {
  // put your setup code here, to run once.  Init sensors
    //Serial Port setup
  Serial.begin (9600);

// LCD Display
lcd.init();      // initialize the lcd
  lcd.backlight(); //turn on backlight
  lcd.clear();// clear previous values from screen
  // 1st row
  lcd.setCursor (0,0); //character zero, line 0
  lcd.print("CoopDuino 0.1"); // print text  
    
//setup DHT 22
    dht.begin();

//setup DS18b20
sensors.begin();

//setup pin for door switches
//configure pins as an input and enable the internal pull-up resistor
  pinMode(DoorOpenPIN, INPUT_PULLUP);
  pinMode(DoorClosedPIN, INPUT_PULLUP);

//door motor
  pinMode(doorin, OUTPUT); 
  pinMode(doorout, OUTPUT);

 
//setup distance sensor 
  pinMode(DS0TRIGPIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(DS0ECHOPIN, INPUT); // Sets the echoPin as an Input
  
// setup LDR PIN
  pinMode(LDRPIN, INPUT);
  Serial.println("Finished startup sequence ");

//setup RTC
#ifdef AVR
  Wire.begin();
#else
  Wire1.begin(); // Shield I2C pins connect to alt I2C bus on Arduino Due
#endif
  rtc.begin(); 
   
//Light State
//Define the states of the machine
#define DARK 0
#define LIGHT 1
#define TWILIGHT 2
light_state = DARK;

 //Door State
//Define the states of the machine
#define Door_Open 0
#define Door_Closed 1
#define Door_transition 2
door_state = Door_transition;
}
/********************************************************************/
void loop()
{  
  //DHT 22 (Temp and RH) Sensor1
    //Read data and store it to variables hum and temp
    SN0_RHIn = dht.readHumidity();
    SN0_tempIn= dht.readTemperature();

//RTC
    DateTime now = rtc.now();
    Serial.print(now.hour(), DEC);
    Serial.print(':');
     if(now.minute()<10)
    Serial.print('0');
    Serial.print(now.minute(), DEC);
    Serial.println();

// DS18b20
// call sensors.requestTemperatures() to issue a global temperature 
// request to all devices on the bus 
 sensors.requestTemperatures(); 
 // Send the command to get temperature readings 
 SN0_tempOut = sensors.getTempCByIndex(1); //outside temp. 1 is first IC on the wire
 SN0_tempWater = sensors.getTempCByIndex(0); //water temp 0 is first IC on the wire

// Door switches to verify door state
//check_door();
//LDR
check_light();

//door check routine Any chickens in the way?
doorBlocked = FALSE;
// Clears the trigPin
digitalWrite(DS0TRIGPIN, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(DS0TRIGPIN, HIGH);
delayMicroseconds(10);
digitalWrite(DS0TRIGPIN, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(DS0ECHOPIN, HIGH);
// Calculating the distance
distance= duration*0.034/2;
if (distance < 38){
  //someone in the way
  doorBlocked = TRUE;
}

// Prints the distance on the Serial Monitor
Serial.print("Distance in cm : ");
Serial.println(distance);

//NOW adjust door to reach desired states
if (((now.hour() >= sleephour) || (now.hour() < wakehour )) && (door_state != Door_Closed)) //&& (doorBlocked == FALSE))
{
      Serial.println("it's sleep time and the door is open. Closing door");
      Serial.println(door_state);
    lcd.clear();// clear previous values from screen
    lcd.setCursor (0,0); //character zero, line 0
    lcd.print("Time"); // print text  
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    if(now.minute()<10)
      lcd.print('0');
    lcd.print(now.minute(), DEC);    
    lcd.setCursor (0,1); //character 0, line 1
    lcd.print("Its sleep time "); // print text 
      lcd.setCursor (0,2); //character 9, line 2
  lcd.print("and door is open "); // print text
  
    lcd.setCursor (0,3); //character 9, line 2
  lcd.print("Closing it... "); // print text
   
      close_door(); //it's sleep time and the door is open. Closing door
}
if (((now.hour() >= wakehour) && (now.hour() < sleephour)) && (door_state != Door_Open))
{
     Serial.print("Greater than wakehour, less than sleephour, Door state not open ");
     Serial.println("Opening the door");
      Serial.println(door_state);
    lcd.clear();// clear previous values from screen
    lcd.setCursor (0,0); //character zero, line 0
    lcd.print("Time"); // print text  
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    if(now.minute()<10)
      lcd.print('0');
    lcd.print(now.minute(), DEC);    
    lcd.setCursor (0,1); //character 0, line 1
    lcd.print("Time to be awake "); // print text 
      lcd.setCursor (0,2); //character 9, line 2
  lcd.print("and door is closed "); // print text
    lcd.setCursor (0,3); //character 9, line 2
  lcd.print("Opening it... "); // print text
        open_door();//it's between wake and sleep hours and the door is closed. Opening door
}

// digitalWrite(enablePin,HIGH); //HIGH is off and servo is saving power

//send all info via XBEE to Coordinator node
  Serial.println("RHIn,  tempIn,tempOut,tempWater,doorstate,lightstate, LDR reading, doorError,doorBlocked");
  Serial.print('<');  //Starting symbol
  Serial.print(SN0_RHIn); //RHumidity inside
  Serial.print(",");
  Serial.print(SN0_tempIn); //Temp Inside Celsius
  Serial.print(",");
  Serial.print(SN0_tempOut);   //Outside Temperature Celsius
  Serial.print(",");
  Serial.print(SN0_tempWater); //Water Temperature Celsius
  Serial.print(",");    
  Serial.print(door_state); //0=open, 1 = close, 2= unknown
  Serial.print(",");
  Serial.print(light_state); //0=dark, 1 = light, 2= unknown
  Serial.print(",");
  Serial.print(SN0_LDR); //raw light sensor value
  Serial.print(",");
  Serial.print(doorError);
  Serial.print(",");
  Serial.print(doorBlocked);
  
  Serial.println('>');//Ending symbol

//LCD DISPLAY SECTION
   
  lcd.backlight(); //turn on backlight
  lcd.clear();// clear previous values from screen
  // 1st row
  lcd.setCursor (0,0); //character zero, line 0
  lcd.print("Time"); // print text  
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    if(now.minute()<10)
      lcd.print('0');
    lcd.print(now.minute(), DEC);    
  lcd.setCursor (10,0); //character 12, line 0
  lcd.print("Wake@ "); // print text
  lcd.print(wakehour);
  lcd.print(":00");
//insert waketime variable here
//2nd Row
  lcd.setCursor (0,1); //character 0, line 1
  lcd.print("RH "); // print text 
  char RHStr[5];
  dtostrf(SN0_RHIn, 5, 2, RHStr );
  lcd.print(RHStr); // print voltage
  lcd.setCursor (9,1); //character 9, line 1
  lcd.print("Sleep@"); // print V at the end of voltage  
  lcd.print(sleephour);
  lcd.print(":00");
  //print the sleep time variable here
  //3rd Row
  lcd.setCursor (0,2); //character 0, line 2
  lcd.print("TmpIn "); 
  char TempStr[3];
  dtostrf(SN0_tempIn, 3, 0, TempStr );
  lcd.print(TempStr); // print inside temperature 
  lcd.print((char)223); // degree symbol
  lcd.print("C Dr "); // degree symbol 
  //convert door state to txt
  if (door_state == Door_Open)
  {
    lcd.print("Open");
    }
  if (door_state == Door_Closed)
  {
    lcd.print("Closd");
    } 
  if (doorError == 1)
  {
    lcd.setCursor (15,2); //character 0, line 1
    lcd.print("ERROR");
    }
  //4TH rOW
  lcd.setCursor (0,3); //character 0, line 4
  lcd.print("TmpOut"); 
  dtostrf(SN0_tempOut, 3, 0, TempStr );
  lcd.print(TempStr); // print Outside temperature 
  lcd.print((char)223); // degree symbol
  lcd.print("C LDR "); // degree symbol 
  lcd.print(SN0_LDR); //print the LDR var

delay(LoopWaitTime); 
}
void check_door() {
  // use door switches to verify door state
  SN0_dOpen = digitalRead(DoorOpenPIN);
  SN0_dClosed = digitalRead(DoorClosedPIN);
  door_state = Door_transition; //default
  doorError = 0;  //reset the error var
  if (SN0_dOpen and !SN0_dClosed)
  {
    door_state = Door_Open;
  }
  else if (!SN0_dOpen and SN0_dClosed)
  {
    door_state = Door_Closed;
  }
 }
void check_light() {
  //use LDR to verify light status
  SN0_LDR = analogRead(LDRPIN);
//Serial.println(SN0_LDR);
if ((SN0_LDR <=400) && (SN0_LDR >=100)){light_state = TWILIGHT;}
                     else if (SN0_LDR <100){light_state = DARK;}
                                      else{light_state = LIGHT;}
}
void close_door() {
  startMillis = millis(); //when did we start?
  //enable the stepper motor in the right direction
  //make sure we arent running the door the other way
  digitalWrite(doorout, LOW);
  delay(1000);
  digitalWrite(doorin, HIGH);
  delay(75000);
  digitalWrite(doorin, LOW);
  /*
  while (door_state != Door_Closed)
  {
    check_door();
      taskMillis = millis();
    if (taskMillis - startMillis > 15000)
    {
      doorError = 1;
        break; //taking too long.  something wrong
    }
      curMillis = millis();
    singleStep();
  }
  //disable the stepper motor to conserve power
//  digitalWrite(enablePin,HIGH); //HIGH is off and saving power
*/
door_state = Door_Closed;
  Serial.println("door closed") ;
}
void open_door() {
  startMillis = millis(); //when did we start?
  //enable the stepper motor in the right direction
  //make sure we arent running the door the other way
  digitalWrite(doorin, LOW);
  delay(1000);
  digitalWrite(doorout, HIGH);
  delay(75000);
  digitalWrite(doorout, LOW);
door_state = Door_Open;
  /*
  while (door_state != Door_Open)
  {
    check_door();
      taskMillis = millis();
    if (taskMillis - startMillis > 15000)
    {
      doorError = 1;
        break; //taking too long.  something wrong
    }
    curMillis = millis();
    singleStep();
  }
  //disable the stepper motor to conserve power
//  digitalWrite(enablePin,HIGH); //HIGH is off and saving power
*/
Serial.println("door opened") ;
}
void singleStep() {
 if (curMillis - prevStepMillis >= millisBetweenSteps) {
 prevStepMillis += millisBetweenSteps;
 //digitalWrite(stepPin, HIGH);
 //digitalWrite(stepPin, LOW);
 }

}

