

/*
  code works on a car mp3 remote controller

  IR Sensor:[(Pins front left to right) Sig GND Plus];
            D1; 5V
  LED Blue: D12
  LED RED:  D13
  
  Stepper Motor:
    IN1  D8
    IN2  D9
    IN3  D10
    IN4  D11

  Temperature Sensor(LM35):
    A0
    5V
    GND

   LCD SCREEN:
   * LCD RS pin to digital pin 7
   * LCD Enable pin to digital pin 6
   * LCD D4 pin to digital pin 5
   * LCD D5 pin to digital pin 4
   * LCD D6 pin to digital pin 3
   * LCD D7 pin to digital pin 2
   * LCD R/W pin to ground
*/

/*Code Theory:
 * 
 * setup
 * 1. Print current Temperature
 * 2. Adjust Maximum Temperature (with limitation)
 * 3. Adjust Minimum Temperature (with limitation)
 * 
 *  loop
 *  - Measure Temperature
 *  - if too high run Windowmotor 1 Time around anticlockwise (run Fan Motor) (Send SMS to owner with CHILD IN DANGER ///HEAT\\\)
 *  - if too low run Windowmotor 1 Time around clockwise (stop Fan Motor) (Send SMS to owner CHILD IN DANGER ///COLD\\\)
 *  - 
 *  Fututre Extra
 *  - if moisture is Present close windows
 *  - Adjust Temperature any time
 *  - detect if someone is inside the car and wake up from sleep mode
*/

      /////////
      //INTRO//
      /////////
#include <IRremote.h> //Infrared Library from Github
#include <LiquidCrystal.h> //LCD library
#include <SoftwareSerial.h> //GSM Library
#include <String.h> //String library 

//Values Preset
int runs = 0; //Instructions for temperature setting start at 0 runs
int maxruns = 1; //this determines how many times it should be displayed in total before the user can press CH
int TempMax = 23; //default maximum temperature
int passMax = 0; // as long as it is 0 the maximum temperature has not been approved
int passMin = 0; // as long as it is 0 the minimum temperature has not been approved
int TempMin = 19; //default minimum temperature
int del =1; //DELAY between coils being powered
int Contrast=15; //set LCD contrast

int smsHot1 = 0; //predefines that no first sms has been sent
int smsHot2 = 0; //predefines that no second sms has been sent
int smsCold1 = 0; //predefines that no first sms has been sent
int smsCold2 = 0; //predefines that no second sms has been sent

//Values outcome
float tempC; //defines data Type of temperature as float
int reading; //defines read value from LM35 as integer
unsigned long timeSMS1; //time when sms 1 was sent
unsigned long timeSMS2; //time when sms 2 was sent
unsigned long timelimit = 180000; //time in millisecs til 2. sms

//IR Button Signals
long TempMINUS   = 0xFFA25D;    // CH- Button
long TempPLUS    = 0xFFE21D;    // CH+ Button
long TempSetDone = 0xFF629D;    // CH Button

//Pins
//Define Motor Pins as IN 1-4
#define IN1  8
#define IN2  9
#define IN3  10
#define IN4  11

int tempPin = 0; //LM35
int RECV_PIN = 15; //IR
int fanPin = 16; //Fan Pin
int BLUE = 12; //LED
int RED = 13; //LED
int SMS1 = 18; // Pin D7 of SIM900 relocated
int SMS2= 19; // Pin D8 of SIM900 relocated
SoftwareSerial mySerial(SMS1, SMS2);

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);// initialize the library with the numbers of the interface pins

IRrecv irrecv(RECV_PIN); //configure pin for Infra Red
decode_results results; //decode the IR signal


      ////////////
      //FUNCTION//
      ////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
void SetTempMaxIR(){                       //this function sets TempMax with IR
long TempMINUS = 0xFFA25D; // CH-
long TempPLUS =  0xFFE21D; // CH+
long TempSetDone = 0xFF629D;   // CH



if (results.value == TempMINUS)
      {
        Serial.println (passMax);
        TempMax=TempMax-1; //temperature gets decreased by 1
        Serial.print ("Maximum Temperature in Degrees Celsius:\t");
        Serial.println (TempMax);
        
        digitalWrite(BLUE,HIGH);
        delay(1000);
        digitalWrite(BLUE,LOW);
        
        
      }
     else if (results.value == TempPLUS )
      {
        Serial.println (passMax);
        TempMax=TempMax+1; //temperature gets increased by 1

         //preset Temperature limit
         if (TempMax >37){
          TempMax=37;
          Serial.print ("Maximum Temperature Limit reached\n");
          } 
        Serial.print ("Maximum Temperature in Degrees Celsius:\t");
        Serial.println (TempMax);
        
        digitalWrite(RED,HIGH);
        delay(1000);
        digitalWrite(RED,LOW);
      }
     else if (results.value == TempSetDone )
      {
        passMax=1; //temperature stays and the variable to proceed turns to 1
        Serial.println (passMax); 
        Serial.println ("Temperature set");
        Serial.print ("Maximum Temperature in Degrees Celsius:\t");
        Serial.println (TempMax);
        digitalWrite(RED,HIGH);
        digitalWrite(BLUE,HIGH);
        delay (500);
        digitalWrite(RED,LOW);
        digitalWrite(BLUE,LOW);
      }
      else{
        Serial.print("Wrong Button or pressed too long\n (CH- for minus, CH+ for plus, CH for OK)\n\n");
        Serial.println(results.value, HEX);
      }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void SetTempMinIR(){                       //this function sets TempMin with IR
long TempMINUS = 0xFFA25D; // CH-
long TempPLUS =  0xFFE21D; // CH+
long TempSetDone = 0xFF629D;   // CH



if (results.value == TempMINUS)
      {
        Serial.println (passMin);
        TempMin=TempMin-1;

        //preset Temperature limit
        if (TempMin <15){
          TempMin=15; //temperature gets decreased by 1
          Serial.print ("Minimum Temperature Limit reached\n");
          } 
        Serial.print ("Minimum Temperature in Degrees Celsius:\t");
        Serial.println (TempMin);
        
        digitalWrite(BLUE,HIGH);
        delay(500);
        digitalWrite(BLUE,LOW);
      }
     else if (results.value == TempPLUS )
      {
        Serial.println (passMin);
        TempMin=TempMin+1; //temperature gets increased by 1
        Serial.print ("Minimum Temperature in Degrees Celsius:\t");
        Serial.println (TempMin);
        digitalWrite(RED,HIGH);
        delay(500);
        digitalWrite(RED,LOW);
      }
     else if (results.value == TempSetDone )
      {
        passMin=1; //temperature stays and the variable to proceed turns to 1
        Serial.println (passMin); 
        Serial.println ("Temperature set");
        Serial.print ("Minimum Temperature in Degrees Celsius:\t");
        Serial.println (TempMin);
        digitalWrite(RED,HIGH);
        digitalWrite(BLUE,HIGH);
        delay (500);
        digitalWrite(RED,LOW);
        digitalWrite(BLUE,LOW);
      }
      else{
        Serial.print("Wrong Button or pressed too long\n (CH- for minus, CH+ for plus, CH for OK)\n\n");
        Serial.println(results.value, HEX);
      }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void windowDOWN(){
  for (int x=0;x<=4096/8;x++)
{
         digitalWrite(IN1, LOW); 
         digitalWrite(IN2, LOW);
         digitalWrite(IN3, LOW);
         digitalWrite(IN4, HIGH);
    delay(del);
         digitalWrite(IN1, LOW); 
         digitalWrite(IN2, LOW);
         digitalWrite(IN3, HIGH);
         digitalWrite(IN4, HIGH);
    delay(del);     
         digitalWrite(IN1, LOW); 
         digitalWrite(IN2, LOW);
         digitalWrite(IN3, HIGH);
         digitalWrite(IN4, LOW);
    delay(del);
         digitalWrite(IN1, LOW); 
         digitalWrite(IN2, HIGH);
         digitalWrite(IN3, HIGH);
         digitalWrite(IN4, LOW);
    delay(del);
         digitalWrite(IN1, LOW); 
         digitalWrite(IN2, HIGH);
         digitalWrite(IN3, LOW);
         digitalWrite(IN4, LOW);
    delay(del);
         digitalWrite(IN1, HIGH); 
         digitalWrite(IN2, HIGH);
         digitalWrite(IN3, LOW);
         digitalWrite(IN4, LOW);
    delay(del);
         digitalWrite(IN1, HIGH); 
         digitalWrite(IN2, LOW);
         digitalWrite(IN3, LOW);
         digitalWrite(IN4, LOW);
    delay(del);
         digitalWrite(IN1, HIGH); 
         digitalWrite(IN2, LOW);
         digitalWrite(IN3, LOW);
         digitalWrite(IN4, HIGH);
    delay(del);
         digitalWrite(IN1, LOW); 
         digitalWrite(IN2, LOW);
         digitalWrite(IN3, LOW);
         digitalWrite(IN4, LOW);
    }
    delay (500);  
 }
  
/////////////////////////////////////////////////////////////////////////////////////////////////
void windowUP(){
  for (int x=0;x<=4096/8;x++)
{
         digitalWrite(IN1, HIGH); 
         digitalWrite(IN2, LOW);
         digitalWrite(IN3, LOW);
         digitalWrite(IN4, LOW);
    delay(del);
         digitalWrite(IN1, HIGH); 
         digitalWrite(IN2, HIGH);
         digitalWrite(IN3, LOW);
         digitalWrite(IN4, LOW);
    delay(del);     
         digitalWrite(IN1, LOW); 
         digitalWrite(IN2, HIGH);
         digitalWrite(IN3, LOW);
         digitalWrite(IN4, LOW);
    delay(del);
         digitalWrite(IN1, LOW); 
         digitalWrite(IN2, HIGH);
         digitalWrite(IN3, HIGH);
         digitalWrite(IN4, LOW);
    delay(del);
         digitalWrite(IN1, LOW); 
         digitalWrite(IN2, LOW);
         digitalWrite(IN3, HIGH);
         digitalWrite(IN4, LOW);
    delay(del);
         digitalWrite(IN1, LOW); 
         digitalWrite(IN2, LOW);
         digitalWrite(IN3, HIGH);
         digitalWrite(IN4, HIGH);
    delay(del);
         digitalWrite(IN1, LOW); 
         digitalWrite(IN2, LOW);
         digitalWrite(IN3, LOW);
         digitalWrite(IN4, HIGH);
    delay(del);
         digitalWrite(IN1, HIGH); 
         digitalWrite(IN2, LOW);
         digitalWrite(IN3, LOW);
         digitalWrite(IN4, HIGH);
    delay(del);
         digitalWrite(IN1, LOW); 
         digitalWrite(IN2, LOW);
         digitalWrite(IN3, LOW);
         digitalWrite(IN4, LOW);
    }
    delay (500);  
 }


/////////////////////////////////////////////////////////////////////////////////////////////////
///SendTextMessageHOT()
///this function is to send a sms message when too hot
 void SendTextMessageHOT()
{
  mySerial.print("AT+CMGF=1\r");    //Because we want to send the SMS in text mode
  delay(100);
  mySerial.println("AT + CMGS = \"+4581948268\"");//send sms message, be careful need to add a country code before the cellphone number
  delay(100);
  mySerial.println("Your car has heated up to the preset limit!\n Left behind passengers are in danger!");//the content of the message
  delay(100);
  mySerial.println((char)26);//the ASCII code of the ctrl+z is 26
  delay(100);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
///SendTextMessageHOT2()
///this function is to send a sms message when too hot
 void SendTextMessageHOT2()
{
  mySerial.print("AT+CMGF=1\r");    //Because we want to send the SMS in text mode
  delay(100);
  mySerial.println("AT + CMGS = \"+4581948268\"");//send sms message, be careful need to add a country code before the cellphone number
  delay(100);
  mySerial.println("15 Minutes have passed and you have not deactivated the System.\n Left behind passengers are in danger!");//the content of the message
  delay(100);
  mySerial.println((char)26);//the ASCII code of the ctrl+z is 26
  delay(100);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
///SendTextMessageCOLD()
///this function is to send a sms message when too cold
void SendTextMessageCOLD()
{
  mySerial.print("AT+CMGF=1\r");    //Because we want to send the SMS in text mode
  delay(100);
  mySerial.println("AT + CMGS = \"+4581948268\"");//send sms message, be careful need to add a country code before the cellphone number
  delay(100);
  mySerial.println("Your car has cooled down to the preset limit!\n Left behind passengers are in danger!");//the content of the message
  delay(100);
  mySerial.println((char)26);//the ASCII code of the ctrl+z is 26
  delay(100);
}

///SendTextMessageCOLD2()
///this function is to send a sms message when too cold
void SendTextMessageCOLD2()
{
  mySerial.print("AT+CMGF=1\r");    //Because we want to send the SMS in text mode
  delay(100);
  mySerial.println("AT + CMGS = \"+4581948268\"");//send sms message, be careful need to add a country code before the cellphone number
  delay(100);
  mySerial.println("15 Minutes have passed and you have not deactivated the System.\n Left behind passengers are in danger!");//the content of the message
  delay(100);
  mySerial.println((char)26);//the ASCII code of the ctrl+z is 26
  delay(100);
}

      /////////
      //SETUP//
      /////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {                
  // initialize the digital pin as an output.
  pinMode(RECV_PIN, INPUT);  

  //Fan Pin Output
  pinMode(fanPin, OUTPUT);      // sets the analog/digital pin as output
  
  //Pins Stepper Motor
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT);
  //Set built in analog read value to 1.1 Volts 
  //LM35 only produces voltages from 0-1 Volt 
  //so 3.3 Volts would be a waste
  //http://playground.arduino.cc/Main/LM35HigherResolution
  analogReference(INTERNAL);

  irrecv.enableIRIn(); // Start the receiver
  mySerial.begin(19200);  // the GSM Shield baud rate 
  Serial.begin(9600);
  
  //LCD
  Serial.println("LCD test with PWM contrast adjustment");  // Print text to Serial monitor
  lcd.begin(16, 2); // Set up the LCD's number of columns and rows:
  analogWrite(6,Contrast);                                       // 

  lcd.setCursor(1, 0);  // Set LCD cursor position (column, row)
  lcd.print("Guardien Angel");
  lcd.setCursor(2, 1);
  lcd.print("is running");
  delay(3000);
  

  Serial.print("Current Temperature:\t");
    //AVERAGE oF 8 Values
  reading = analogRead(tempPin);
  delay(50);
  reading = (reading + analogRead(tempPin))/2;
  delay(50);
  reading = (reading*2 + analogRead(tempPin))/3;
  delay(50);
  reading = (reading*3 + analogRead(tempPin))/4;
  delay(50);
  reading = (reading*4 + analogRead(tempPin))/5;
  delay(50);
  reading = (reading*5 + analogRead(tempPin))/6;
  delay(50);
  reading = (reading*6 + analogRead(tempPin))/7;
  delay(50);
  reading = (reading*7 + analogRead(tempPin))/8;
  delay(50);
  tempC = reading / 9.31;
  Serial.println(tempC);

  //Display Temperature
  lcd.clear();          // Clear the display
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("Car Temperature:");
  lcd.setCursor(0, 1);
  lcd.print(tempC);
  lcd.setCursor(5, 1);
  lcd.print(" ");
  lcd.setCursor(6, 1);
  lcd.print((char)223); 
  lcd.setCursor(7, 1);
  lcd.print("C"); 
  delay(5000);
  
//ADJUST TEMPERATURE
Serial.println("adjust maximum temperature");

  lcd.clear();
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("Adjust safety");
  lcd.setCursor(2, 1);
  lcd.print("Temperatures");
  delay(3000);

  boolean understood =0;
while (understood ==0){ //Maybe activate signal first

if (irrecv.decode(&results)) 
  {
  
   
  if (results.value == TempSetDone )
      {
        understood=1;
        Serial.println (understood); 
        Serial.println ("UNDERSTOOD!");
        Serial.println ("GREAT!");

        lcd.clear();
        lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
        lcd.print("UNDERSTOOD!");
        lcd.setCursor(2, 1);
        delay(2000);
        
        digitalWrite(RED,HIGH);
        digitalWrite(BLUE,HIGH);
        delay (500);
        digitalWrite(RED,LOW);
        digitalWrite(BLUE,LOW);
      }
        irrecv.resume(); // Receive the next value
  }
  
  else {
    
    while (runs <= maxruns){
  lcd.clear();
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("CH+ to raise");
  lcd.setCursor(2, 1);
  lcd.print("Temperature");
  delay(2000);
  lcd.clear();

  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("CH- to reduce");
  lcd.setCursor(2, 1);
  lcd.print("Temperature");
  delay(2000);
  lcd.clear();


  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("CH to approve");
  lcd.setCursor(2, 1);
  lcd.print("Temperature");
  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("Press CH");
  lcd.setCursor(0, 1);
  lcd.print("when understood");
  delay(2000);
  runs =runs + 1;
    }
  }
}
//set Maximum Temperature
while (passMax==0){
  lcd.clear();
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("Upper Limit:");
  lcd.setCursor(2, 1);
  lcd.print(TempMax);
  lcd.setCursor(7, 1);
  lcd.print(" ");
  lcd.setCursor(8, 1);
  lcd.print((char)223); 
  lcd.setCursor(9, 1);
  lcd.print("C");

  
   if (irrecv.decode(&results)) {

   SetTempMaxIR();
  
  lcd.setCursor(2, 1);
  lcd.print(TempMax);
  lcd.setCursor(7, 1);
  lcd.print(" ");
  lcd.setCursor(8, 1);
  lcd.print((char)223); 
  lcd.setCursor(9, 1);
  lcd.print("C");   
     
     irrecv.resume(); // Receive the next value
   }
   }

//final chosen setting:
  lcd.clear();
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("Limit set to:");
  lcd.setCursor(2, 1);
  lcd.print(TempMax);
  lcd.setCursor(7, 1);
  lcd.print(" ");
  lcd.setCursor(8, 1);
  lcd.print((char)223); 
  lcd.setCursor(9, 1);
  lcd.print("C"); 

  delay(3000);
//set minimum temperature
while (passMin==0){
  lcd.clear();
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("Lower Limit:");
  lcd.setCursor(2, 1);
  lcd.print(TempMin);
  lcd.setCursor(7, 1);
  lcd.print(" ");
  lcd.setCursor(8, 1);
  lcd.print((char)223); 
  lcd.setCursor(9, 1);
  lcd.print("C"); 
  
   if (irrecv.decode(&results)) {
   
   SetTempMinIR();

  lcd.setCursor(2, 1);
  lcd.print(TempMin);
  lcd.setCursor(7, 1);
  lcd.print(" ");
  lcd.setCursor(8, 1);
  lcd.print((char)223); 
  lcd.setCursor(9, 1);
  lcd.print("C");
  
     irrecv.resume(); // Receive the next value
   }
   }

//final chosen setting:
  lcd.clear();
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("Limit set to:");
  lcd.setCursor(2, 1);
  lcd.print(TempMin);
  lcd.setCursor(7, 1);
  lcd.print(" ");
  lcd.setCursor(8, 1);
  lcd.print((char)223); 
  lcd.setCursor(9, 1);
  lcd.print("C");
  delay(3000);
  
}
      ////////
      //MAIN//
      ////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
    Serial.println("Temperature Reading");
    //AVERAGE oF 8 Values
    reading = analogRead(tempPin);
    delay(50);
    reading = (reading + analogRead(tempPin))/2;
    delay(50);
    reading = (reading*2 + analogRead(tempPin))/3;
    delay(50);
    reading = (reading*3 + analogRead(tempPin))/4;
    delay(50);
    reading = (reading*4 + analogRead(tempPin))/5;
    delay(50);
    reading = (reading*5 + analogRead(tempPin))/6;
    delay(50);
    reading = (reading*6 + analogRead(tempPin))/7;
    delay(50);
    reading = (reading*7 + analogRead(tempPin))/8;
    delay(50);
    tempC = reading / 9.31; //analog signal to Temperature for LM35
    Serial.println(tempC);

  lcd.clear();          // Clear the display
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("Car Temperature:");
  lcd.setCursor(0, 1);
  lcd.print(tempC);
  lcd.setCursor(5, 1);
  lcd.print(" ");
  lcd.setCursor(6, 1);
  lcd.print((char)223); 
  lcd.setCursor(7, 1);
  lcd.print("C"); 
  delay(2000);
  
    
    
    if(tempC >= TempMax){

  //if too high
  lcd.clear();          // Clear the display
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("DANGER!");
  lcd.setCursor(0, 1);
  lcd.print("HIGH TEMPERATURE");
  delay(2000);

  lcd.clear();          // Clear the display
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("Car Temperature:");
  lcd.setCursor(0, 1);
  lcd.print(tempC);
  lcd.setCursor(5, 1);
  lcd.print(" ");
  lcd.setCursor(6, 1);
  lcd.print((char)223); 
  lcd.setCursor(7, 1);
  lcd.print("C"); 
  delay(2000);

  digitalWrite(fanPin, HIGH);   // turns the FAN on

  lcd.clear();          // Clear the display
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("FAN IS");
  lcd.setCursor(0, 1);
  lcd.print("TURNED ON");
  delay(2000);
  
  lcd.clear();          // Clear the display
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("WINDOWS ARE");
  lcd.setCursor(0, 1);
  lcd.print("BEING OPENED");
  
      windowDOWN(); //Function to lower windows (written before setup)
  delay(2000);

  lcd.clear();          // Clear the display
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("WINDOWS ARE");
  lcd.setCursor(0, 1);
  lcd.print("OPEN NOW");
  
      Serial.println("Window Down");

      while(smsHot1 == 0)
      {
       SendTextMessageHOT();
        lcd.clear();          // Clear the display
        lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
        lcd.print("SMS SENT TO");
        lcd.setCursor(0, 1);
        lcd.print("RESPONSIBLE");
                  delay(2000);

        smsHot1= 1;
        timeSMS1 = millis();
        }
        
        timeSMS2 = millis();
        unsigned long delta = timeSMS2 - timeSMS1;
        if( delta > timelimit && smsHot2 == 0)
        {
          SendTextMessageHOT2();
          lcd.clear();          // Clear the display
          lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
          lcd.print("REMINDER TO");
          lcd.setCursor(0, 1);
          lcd.print("RESPONSIBLE");
          delay(2000);
          smsHot2 =1;
          timeSMS1 = 0;
          timeSMS2 = 0;
          
          }
      }



    if(tempC <= TempMin){
 
  //if too low
  lcd.clear();          // Clear the display
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("DANGER!");
  lcd.setCursor(0, 1);
  lcd.print("LOW TEMPERATURE");
  delay(2000);

  lcd.clear();          // Clear the display
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("Car Temperature:");
  lcd.setCursor(0, 1);
  lcd.print(tempC);
  lcd.setCursor(5, 1);
  lcd.print(" ");
  lcd.setCursor(6, 1);
  lcd.print((char)223); 
  lcd.setCursor(7, 1);
  lcd.print("C"); 
  delay(2000);

  digitalWrite(fanPin, LOW);    // sets the FAN off
  
  lcd.clear();          // Clear the display
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("FAN IS");
  lcd.setCursor(0, 1);
  lcd.print("TURNED OFF");
  delay(2000);
  
  lcd.clear();          // Clear the display
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("WINDOWS ARE");
  lcd.setCursor(0, 1);
  lcd.print("BEING CLOSED");

        windowUP();
  delay(2000);

  lcd.clear();          // Clear the display
  lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
  lcd.print("WINDOWS ARE");
  lcd.setCursor(0, 1);
  lcd.print("CLOSED NOW");
  delay(2000);
      Serial.println("Window Up");

     while(smsCold1 == 0)
      {
       SendTextMessageCOLD();
        lcd.clear();          // Clear the display
        lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
        lcd.print("SMS SENT TO");
        lcd.setCursor(0, 1);
        lcd.print("RESPONSIBLE");
                  delay(2000);

        smsCold1= 1;
        timeSMS1 = millis();
        }
        
        timeSMS2 = millis();
        unsigned long delta = timeSMS2 - timeSMS1;
        if(delta > timelimit && smsCold2 == 0)
        {
          SendTextMessageCOLD2();
          lcd.clear();          // Clear the display
          lcd.setCursor(0, 0);  // Set LCD cursor position (column, row)
          lcd.print("REMINDER TO");
          lcd.setCursor(0, 1);
          lcd.print("RESPONSIBLE");
                    delay(2000);

          smsCold2=0;
          timeSMS1 = 0;
          timeSMS2 = 0;
          }
      }
}
