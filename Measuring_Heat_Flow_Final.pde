//MEASURING HEAT FLOW


 /*
 The pins used by the 4-20mA as you can check at the libraries are 
 ANALOG1, ANALOG2, ANALOG5, ANALOG6, DIGITAL5 and 5V sensor power from Sensor connector. 
 Also the board uses the Battery pin and GND from Auxiliary SPI-UART connector.
 */
 
// Include this library for using current loop functions
#include <currentLoop.h>
#include "wiring.h"

    char* filename="LOG.TXT";
    uint8_t sd_answer;

//Volume
    unsigned long t1 = 0;
    unsigned long t3 = 0;
    unsigned long deltat;
    float flowRate;
    volatile int counter; //This integer needs to be set as volatile to ensure it updates correctly during the interrupt process.  

//Temperatures
    float temp1 = 0;
    float temp2 = 0;

//provided Thermal Energy for Water
    float Q;
    float D = 1000; //Density in kg/m^3
    float c = 4.1855; //kJ/(kg⋅K) Heat capacity

//Time:
    int hour;
    int minute;
    int second;
    int day;
    int month;
    int year;
  
double flowrate(int X, unsigned long Y) //Funktion to calculate the Volume per hour
{
    double flow;
    flow = ((float)X / 435); // X Impulse/tinterval, 435 Impulse/l,  --> l
    flow = (flow/Y);//returns m^3 or -->*1000; //litres per second
    return flow;
}

void Flow()
{
   counter++; //Every time this function is called, increment "count" by 1
}

void setup()
{
  // Power on the USB for viewing data in the serial monitor
    USB.ON();
    USB.println(F("Waspmote Start"));
    USB.println(F("++++++++++++++++++++++++++++++++++"));

// Powers RTC up, init I2C bus and read initial values
    USB.println(F("Init RTC"));
    RTC.ON();

// Setting time [yy:mm:dd:dow:hh:mm:ss]
    RTC.setTime("16:08:11:04:15:25:00");
    USB.print(F("Setting time: "));
    USB.println(F("16:08:11:04:15:25:00"));
  
// Set SD ON
    SD.ON();
//Overwriting old file on SD card
  // Delete present file
    sd_answer = SD.del(filename);
  
  if( sd_answer == 1 )
  {
    USB.println(F("file deleted"));
  }
  else 
  {
    USB.println(F("file NOT deleted"));  
  }
    
  // Create file
    sd_answer = SD.create(filename);
  
  if( sd_answer == 1 )
  {
    USB.println(F("file created"));
  }
  else 
  {
    USB.println(F("file NOT created"));  
  } 
  
//Turn on Power for Temperature Sensors
// Sets the 5V switch ON - current LOOP
    currentLoopBoard.ON(SUPPLY5V);
    delay(100);


//ATTACH FIUNCTION TO INTERRUPTS
          //READ UP IF NECESSARY
          //http://www.libelium.com/api/waspmote/df/dbb/WInterrupts_8c.html#acd46400dadb5ae0ba962ab74ba71b592
          //MU_RX: http://www.libelium.com/api/waspmote/db/d5d/pins__waspmote_8h_source.html#l00074
          //RXD1_PIN: http://www.libelium.com/api/waspmote/d4/d0c/WaspConstants_8h_source.html#l00220
          // attachInterrupt: http://www.libelium.com/api/waspmote/dd/d4c/wiring_8h.html#a3ed3d1d750935833a6f9b0363a5e6f13
    pinMode(MUX_RX,INPUT);
    attachInterrupt(RXD1_PIN, Flow, RISING);

 
// 1 - Write header of LOG table    
    sd_answer = SD.writeSD(filename,"dTIME\tFlow\tTempZU\tTempAB\tThermalE\thour\tminute\tsecond\tday\tmonth\tyear\n", 0);
    sd_answer = SD.append(filename,"(ms)\t(m^3/s)\t(degC)\t(degC)\t(kJ/s)\t(h)\t(min)\t(s)\t(d)\t(+-30d)\t(a)\n");
  
  if( sd_answer == 1 ) 
  {
    USB.println(F("\n1 - Write HEADERRS in file at position 0 ")); 
  }
  else
  {
    USB.println(F("\n1 - Write failed"));  
  }

  
}

void loop()
{  
     
  
//FLOW SENSOR INETERRUPTS
  // Counting Time for a FIXED number of impulses (1 Litre)
 
  

    USB.println(F("InterruptStart"));
    USB.println(RTC.getTime());
      
    noInterrupts();
    counter = 0;      // Reset the counter so we start counting from 0 again
//      FOR DEBUGGUNG
//      USB.print("#Interrupts");
//      USB.print("\t");
//      USB.println("millis");
      
    interrupts();
  for (counter = 0;counter < 1; counter)
  {
  }
    t1=millis();
  
  for (counter = 1;counter <= 435; counter)
  {
//      FOR DEBUGGUNG
//      USB.print(counter);
//      unsigned long t2=millis();
//      USB.print("\t");
//      USB.println(t2-t1);
  }
  
    noInterrupts();
    unsigned long t3=millis();
    int counter1=counter;
  //enable interrupts because noInterrupts stop PC communication after 2 lines
    interrupts();
    
    deltat= t3-t1;
//Volume Flow
    flowRate = flowrate(counter1,deltat); //in Liters/Timeintervalinterval

    USB.println("Number of counts");
    USB.println(counter1);
    USB.println("tStart\ttStop\tDelta t\tVolumeFlow");
    USB.print(t1);
    USB.print("\t");
    USB.print(t3);
    USB.print("\t");
    USB.print(t3-t1);
    USB.print("\t");
    USB.println(flowRate);    
    USB.print("\n");
    
    delay(1000);
//TEMPERATURE MEASURING

  // Sets the 12V switch ON - current LOOP
    currentLoopBoard.ON(SUPPLY12V); 
    delay(100); 

  // Get the sensor value in integer format (0-1023)
    int value1 = currentLoopBoard.readChannel(CHANNEL1); 
    USB.print("Int value read from channel 1 : ");
    USB.println(value1);

  // Get the sensor1 value as a current in mA
    float current1 = currentLoopBoard.readCurrent(CHANNEL1);
    USB.print("Current value read from channel 1: ");
    USB.print(current1);
    USB.println("mA");
  
  // Get the sensor value in integer format (0-1023)
    int value2 = currentLoopBoard.readChannel(CHANNEL2); 
    USB.print("Int value read from channel 2 : ");
    USB.println(value2);
  
  // Get the sensor2 value as a current in mA
    float current2 = currentLoopBoard.readCurrent(CHANNEL2);
    USB.print("Current value read from channel 1: ");
    USB.print(current2);
    USB.println("mA");
  //Temperature calculation: pt100 & MU-PT100-I420
  //https://www.sensorshop24.de/tpl/download/4-20mA-Temperatur1.pdf
  
 //WANTED VALUES TEMPERATURE
 
      float p1 =     0.03678;  //(0.01689, 0.05666)
      float p2 =       8.253;  //(7.716, 8.79)
      float p3 =      -82.61;  //(-86.22, -79.01)
       
 
    temp1 =p1*current1*current1 + p2*current1 + p3;
    temp2 =p1*current2*current2 + p2*current1 + p3;
     
  // Sets the 12V switch OFF - current LOOP
    currentLoopBoard.OFF(SUPPLY12V); 
    delay(100); 
    
    USB.print(temp1);
    USB.println(" degrees Celsius1");
    
    USB.print(temp2);
    USB.println(" degrees Celsius2");
    
    USB.println("***************************************");
    USB.print("\n");
    delay(1000);
  
  //Calculating thermal energy
    Q=flowRate*D*c*(temp2-temp1);
  
  
  //Time
    USB.println("hour, minute, seconds, day, month, year");  
    hour=RTC.hour;
    USB.println(hour, DEC);
    minute=RTC.minute;
    USB.println(minute, DEC);
    second=RTC.second;
    USB.println(second, DEC);
    USB.print("\n");  

    day=RTC.day;
    USB.println(day, DEC);
    month=RTC.month;
    USB.println(month, DEC);
    year=(int)2000+RTC.year;
    USB.println(year, DEC);
    
  
  
//Float to String, Preperation for logging
    char dtime[20];
    Utils.long2array(deltat, dtime); // Gets the number ‘1356’ into the string  
    USB.println(dtime);
      
    char flow[20];
    Utils.float2String (flowRate, flow, 8); // (float value, destination string, 5 decimals)
    USB.println(flow);
    
    char tempZU[20];
    Utils.float2String (temp1, tempZU, 5);
    USB.println(tempZU);
    
    char tempAB[20];
    Utils.float2String (temp2, tempAB, 5);
    USB.println(tempAB);
  
    char thermalE[20];
    Utils.float2String (Q, thermalE, 5);
    USB.println(thermalE);

    char Hours[20];
    Utils.float2String(hour, Hours,0); // Gets the number ‘1356’ into the string  
    USB.println(Hours);
  
    char Minutes[20];
    Utils.float2String(minute, Minutes,0); // Gets the number ‘1356’ into the string  
    USB.println(Minutes);
  
    char Seconds[20];
    Utils.float2String(second, Seconds,0); // Gets the number ‘1356’ into the string  
    USB.println(Seconds);
  
    char Days[20];
    Utils.float2String(day, Days,0); // Gets the number ‘1356’ into the string  
    USB.println(Days);
  
    char Months[20];
    Utils.float2String(month, Months,0); // Gets the number ‘1356’ into the string  
    USB.println(Months);
  
    char Years[20];
    Utils.float2String(year, Years,0); // Gets the number ‘1356’ into the string  
    USB.println(Years);

    
//SD - LOGGING PRINTING (LINE after LINE)
    sd_answer = SD.append(filename, dtime);
    sd_answer = SD.append(filename, "\t");
    
    sd_answer = SD.append(filename, flow);
    sd_answer = SD.append(filename, "\t");
    
    sd_answer = SD.append(filename, tempZU);
    sd_answer = SD.append(filename, "\t");
    
    sd_answer = SD.append(filename, tempAB);
    sd_answer = SD.append(filename, "\t");
    
    sd_answer = SD.append(filename, thermalE);
    sd_answer = SD.append(filename, "\t");
    
    sd_answer = SD.append(filename, Hours);
    sd_answer = SD.append(filename, "\t");
    
    sd_answer = SD.append(filename, Minutes);
    sd_answer = SD.append(filename, "\t");
    
    sd_answer = SD.append(filename, Seconds);
    sd_answer = SD.append(filename, "\t");
    
    sd_answer = SD.append(filename, Days);
    sd_answer = SD.append(filename, "\t");
    
    sd_answer = SD.append(filename, Months);
    sd_answer = SD.append(filename, "\t");
    
    sd_answer = SD.appendln(filename, Years);
    //sd_answer = SD.append(filename, "\t");
    
    //sd_answer = SD.append(filename, RTC.getTime());
    //sd_answer = SD.append(filename, "\n");

    SD.showFile(filename);
    delay(1000);
}

