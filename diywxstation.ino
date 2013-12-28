/*
TODO:
- Temperatura: inviare valori medi periodici, con max e min?
- Implementare controllo errori in caso di guasti dei sensori
- Implementare taratura semplice dei sensori tramite define
- Implementare debug mode

----
PROJECT: DIY Weather Station
created on Dec. 2013
by IZ1ZCK (Enrico Gallesio)

Weather graphs (when available):
https://thingspeak.com/channels/8836

DESCRIPTION:
Sketch for Arduino Fio to operate a DIY Weather Station based on several 
sensor modules, store weather data on SD Card and publish them 
on any "Internet of Things" websites or via APRS Amateur Radio network.
XBee Series1 modules, 1 XBee Explorer and 1 Raspberry Pi with internet 
connection that runs a Python script are currently needed in this configuration
to allow wireless operation and remote Arduino Fio Programming
More details coming soon on: http://sites.google.com/site/iz1zckweb/

LICENSE:
EXEPT FOR ALL PARTS OF CODE I GOT ONLINE where other license restrictions may apply
(that's a lot! See Credits below) this project is released under 
The MIT License (MIT) http://opensource.org/licenses/MIT
The above copyright notice  shall be included in all copies or substantial portions of this Software.

LIMITATION OF RESPONSIBILITY:
This project is distributed as-is in the hope that it may be useful, but 
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
or FITNESS FOR A PARTICULAR PURPOSE. You are free to adapt the code the to
your needs but I can't be held responsible for any phisical or electronic 
damage, electric shock or injuries. You should ALWAYS DOUBLE CHECK all connections
and make sure you TAKE ALL THE SAFETY MEASURES related to projects where electric
currents are involved.

CREDITS:
This project is built copy-pasting and adjusting lots of code preciously shared 
by the arduino & hacking community around the world, and specifically...

- BMP085 Pressure sensor
    http://bildr.org/2011/06/bmp085-arduino/
    http://www.sparkfun.com/tutorials/253
- RTC Clock
    http://learn.adafruit.com/ds1307-real-time-clock-breakout-board-kit/understanding-the-code
- SD Card
    http://arduino.cc/en/Tutorial/Datalogger
- Arduino Fio Programming
    http://arduino.cc/en/Main/ArduinoBoardFioProgramming
- Temperature DS18S20 sensor
    http://bildr.org/2011/07/ds18b20-arduino/
- DHT22 Humidity sensor
    http://gist.github.com/dmccreary/7503212
- LED as Light sensor
    http://www.instructables.com/id/LED-as-lightsensor-on-the-arduino/
- Vcc internal sensor
    http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
    http://www.semifluid.com/2012/09/09/arduino-fio-internal-voltmeter-and-thermometer/
- Wind sensor / Vane / Rain gauge
    http://kesslerarduino.wordpress.com/2012/06/21/sparkfun-weather-station/
- Barometric Pressure sensor
    http://bildr.org/2011/06/bmp085-arduino/
- Arduino Sleep Mode
    http://playground.arduino.cc/Learning/arduinoSleepCode
- Arduino Fio and XBee Sleep Modes
    http://www.semifluid.com/2012/09/07/arduino-fio-low-power-setup/
- XBee
    http://www.johnhenryshammer.com/WOW2/pagesHowTo/xbeeSeries1.php
    http://www.desert-home.com/p/the-world-of-xbee.html
- Raspberry Pi and ThingSpeak (Python script)
    http://www.desert-home.com/2013/09/raspberry-pi-and-thingspeak.html
    http://pyserial.sourceforge.net/

*/

// Config
#define TIME_TO_LOG 15000    // Time to log sensors values on SD Card 
#define TIME_TO_SEND 60000   // Time to send sensors values via serial
#define TIME_TO_POLL_HUMIDITY 2000   // Min time to poll humidity sensor
#define TIME_TO_POLL_WIND 60000      // do not changeTIME_TO_POLL_WIND
#define INC_PROGRAM_DELAY 60000      // pause with XBee ON to allow remote Fio programmation
#define HEARTBIT_TIME 5000   // Heartbit flash ms interval
#define HEARTBIT_FLASH 10
#define WARM_UP 2000         // Sensors warm-up time
#define XBEE_WAIT 1000       // delay for XBee connection wake up

// Pins
#define LED_PIN        8     // TODO: Move SD card?
#define XBEE_SLEEP     4
#define CHIP_SELECT   10
#define DHT22_PIN      5          
#define DS18S20_PIN    9
#define ANEMOMETER_PIN 2     // Interrupt    
#define RAIN_GAUGE_PIN 3     // Interrput
#define VANE_PWR       6
#define VANE_PIN      A1
#define LIGHT_PIN     A0

// Interrupts
#define ANEMOMETER_INT 0
#define RAIN_GAUGE_INT 1

// Maths
#define LIGHT_AVG_SAMPLES 10
#define WIND_FACTOR 2.03472
#define RAIN_FACTOR 0.2794
#define ALTITUDE_DIFF 68

// Modules
#define BMP085_ADDRESS 0x77  // I2C address of BMP085


#include <SD.h>              // SD Card
#include <OneWire.h>         // DS18S20 temp. sensor
#include <Wire.h>            // I2C Bus
#include "RTClib.h"          // Real Time Clock
#include <MemoryFree.h>      // freeMemory()
#include <DHT22.h>           // Humidity sensor

/*
#include <avr/wdt.h>         // libs to put arduino to sleep
#include <avr/sleep.h>
#include <avr/interrupt.h>
*/
  
// MODULES
  OneWire ds(DS18S20_PIN);      // Temperature chip DS18S20
  RTC_DS1307 RTC;               // RTC (real time clock)
  DHT22 myDHT22(DHT22_PIN);     // Humidity Sensor 

  //BMP085 calibr. settings 
    const unsigned char OSS = 3;  // Oversampling Setting BMP085 
    int ac1;
    int ac2;
    int ac3;
    unsigned int ac4;
    unsigned int ac5;
    unsigned int ac6;
    int b1;
    int b2;
    int mb;
    int mc;
    int md;
    // b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
    // so ...Temperature(...) must be called before ...Pressure(...).
    long b5; 

// SD
/*
   * SD card attached to SPI bus as follows:
   ** MOSI - pin 11  ** MISO - pin 12 ** CLK - pin 13  ** CS - pin 4
   * Even if it's not used as the CS pin, the hardware CS pin 
     (10 on most Arduino boards, 53 on the Mega) must be left as 
     an output or the SD library functions will not work. 
*/

// GLOBAL VARIABLES               TODO: initialize all variables to zero
  unsigned long currentMillis = 0;  
  int Light_SensValue = 0;
  int ledState = LOW;   
  float temperature = 0;
  float Humidity_value = 0;
  double Wind = 0;
  int light = 0;
  long previousMillis_log = 0;
  long previousMillis_send = 0;
  long previousMillis_humidity_poll = 0;
  long previousMillis_heartbit = 0;
  long previousMillis_wind = 0;
  long previousMillis_gust = 0;
  const long time_to_log = TIME_TO_LOG;    //SI POSSONO EVITARE?
  const long time_to_send = TIME_TO_SEND;
  const long time_to_poll_humidity = TIME_TO_POLL_HUMIDITY;
  const long time_to_heartbit = HEARTBIT_TIME;
  const long time_to_poll_wind = TIME_TO_POLL_WIND;
  boolean restart = false;
  
  volatile unsigned long anem_count=0;
  volatile unsigned long anem_last=0;
  volatile unsigned long anem_max=0xffffffff;

  volatile unsigned long rain_count=0;
  volatile unsigned long rain_last=0;

  static int vaneValues[] PROGMEM={66,84,92,127,184,244,287,406,461,600,631,702,786,827,889,946};
  static int vaneDirections[] PROGMEM={1125,675,900,1575,1350,2025,1800,225,450,2475,2250,3375,0,2925,3150,2700};
   

// FUNCTIONS            TODO: Declare all functions
  void log_data_info();
  void initialize_SD();
  float Temp();
  int Light();
  float Humidity();
  void log_data();
  void print_time();


void setup()                      // ### SETUP ###
{
    Serial.begin(57600);
    while (!Serial) {}            // wait for serial port to connect. Needed for Leonardo only
    
    pinMode(LED_PIN, OUTPUT); 
    pinMode(XBEE_SLEEP, OUTPUT);

    XBee_enable();                // wake up XBee for initial status comms
      
    pinMode(ANEMOMETER_PIN,INPUT_PULLUP);     // TODO find a reliable clicking mode
    //digitalWrite(ANEMOMETER_PIN,HIGH);      // Turn on the internal Pull Up Resistor
    pinMode(RAIN_GAUGE_PIN,INPUT_PULLUP);
    //digitalWrite(RAIN_GAUGE_PIN,HIGH);      // Turn on the internal Pull Up Resistor
    //pinMode(VANE_PWR,OUTPUT);
    //digitalWrite(VANE_PWR,LOW);
    attachInterrupt(ANEMOMETER_INT,anemometerClick,FALLING);
    attachInterrupt(RAIN_GAUGE_INT,rainGaugeClick,FALLING);
    interrupts();

    Wire.begin();                  // initialize I2C bus
    RTC.begin();                   // initialize RTC on I2C bus
    check_adjust_clock();          // check if clock is running 
    bmp085Calibration();
    
    Serial.print("Started up on ");
    print_time();
    
    Serial.println("Warming up... ");
    delay(WARM_UP);

    initialize_SD();       // initialize and check SD card    
    log_data_info();       // initialize columns on .csv log file
}

void loop()                        // ### LOOP ###
{
  
  currentMillis = millis();  //take present elapsed time since reboot in milliseconds
  
  if(currentMillis - previousMillis_heartbit > time_to_heartbit) {
     previousMillis_heartbit=currentMillis;
    digitalWrite(LED_PIN, HIGH);
     delay(HEARTBIT_FLASH);
    digitalWrite(LED_PIN, LOW);
    }

  
  if(currentMillis - previousMillis_log > time_to_log) {
      Serial.print("logging... at ");     
      log_data();   
      Serial.print(millis()/1000); 
      Serial.print("sec - FreeRAM: ");
      Serial.print(freeMemory());
      Serial.print(" Rain: ");
      Serial.print(rain_count);
      Serial.print(" Wind: ");
      Serial.println(anem_count);
      Serial.print("Wind: ");
      Serial.println(getWind());
      Serial.print("Gusts: ");
      Serial.println(getGust());
      Serial.print("Rain: ");
      Serial.println(getUnitRain());
      Serial.print("Temperature: ");   // print light to serial
      Serial.println(Temp()); 
      Serial.print("Light: ");   // print light to serial
      //Serial.println(analogRead(LIGHT_PIN)); 
      Serial.println(Light()); 
      Serial.print("Humidity: ");
      Serial.println(Humidity());
      Serial.print("Battery: "); 
      Serial.println(Vcc());
      Serial.print("Pressure: ");
      Serial.println(Pressure());
      
      restart = false;
      previousMillis_log   = currentMillis;
      }
   
  if(currentMillis - previousMillis_send > time_to_send) {
      Serial.println("sending...");   
      send_data();   
      restart = true;
      previousMillis_send = currentMillis;
      }

}                                // LOOP END


void check_adjust_clock() {
  if (! RTC.isrunning()) {  // clock check with optional auto-set on compile time
      Serial.println("RTC is NOT running!");
      // following line sets the RTC to the date & time this sketch was compiled
      RTC.adjust(DateTime(__DATE__, __TIME__));
    }    
}

void initialize_SD() {
    Serial.print("Initializing SD card...");
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(10, OUTPUT);
    
    // see if the card is present and can be initialized:
    if (!SD.begin(CHIP_SELECT)) {
      Serial.println("Card failed, or not present");
      // don't do anything more:
      return;
    }
    Serial.println("card initialized.");
}

void XBee_enable () {                  // TODO avoid if Xbee already ON
    Serial.println("Activating XBee...");
    delay(200);                        // dont wake uo too early TODO to avoid previous serial text to be tx
    digitalWrite(XBEE_SLEEP, LOW);      // Enable XBee
    digitalWrite(LED_PIN, HIGH);
    delay(XBEE_WAIT);                  // wait for XBee and link to wake up (14ms is enough)
}

void XBee_disable () {
  delay(200);
  digitalWrite(XBEE_SLEEP, HIGH);     // Disable XBee
  digitalWrite(LED_PIN, LOW);
  delay(200);                         // make sure xbee is off TODO
  Serial.println("XBee shutdown");
  
}


void print_time() {
    DateTime now = RTC.now();       // get present date-time
    Serial.print(now.year(), DEC);  // print time to serial
    Serial.print('.');
    Serial.print(now.month(), DEC);
    Serial.print('.');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print('.');
    Serial.print(now.minute(), DEC);
    Serial.print('.');
    Serial.print(now.second(), DEC);
    Serial.println();
}


void send_data() {
    Serial.println("");
    Serial.println("Now sending data...");

    XBee_enable();
    
    print_time();
    Serial.print("data=;");
    Serial.print(getWind());
    Serial.print(";");
    Serial.print(getGust());
    Serial.print(";");
    Serial.print(getUnitRain());
    Serial.print(";");   // print light to serial
    //delay(10);                                    // TODO minidelay to avoid crash ????
    Serial.print(Temp()); 
    Serial.print(";");   // print light to serial
    Serial.print(Light()); 
    //Serial.println("%");
    Serial.print(";");
    Serial.print(Humidity());
    Serial.print(";"); 
    Serial.print(Vcc());
    Serial.print(";");
    Serial.println(Pressure());
    
    incoming_program_pause();    // check for incoming program mesg
    
    //Serial.flush();       // ?
    XBee_disable();
}

void incoming_program_pause() {  // if "Y" is sent via serial pause 5 secs with Xbee On
  if (Serial.available()) {
      char ser = Serial.read();
     if(ser == 'Y'){
      Serial.println("Incoming program command detected. XBee will stay awake a little while now");
      delay(INC_PROGRAM_DELAY);
      Serial.println("New program not received. Resuming loop");
   }
   }
}

void log_data_info()  // initialize columns on csv file
{
  File dataFile = SD.open("datalog.csv", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.print("Time");
      dataFile.print(", ");
      
      dataFile.print("Wind_Speed");
      dataFile.print(", ");
      
      dataFile.print("Wind_Gust");
      dataFile.print(", ");
      
      dataFile.print("Rain");
      dataFile.print(", ");
      
      dataFile.print("Temp");
      dataFile.print(", ");
      
      dataFile.print("Light");
      dataFile.print(", "); 
      
      dataFile.print("Humidity");
      dataFile.print(", "); 
      
      dataFile.print("Pressure");
      dataFile.print(", "); 
      
      dataFile.print("Batt_Vcc");
      
      dataFile.println("");
      dataFile.close();
    } else {
      //Serial.println("Log Error");
    }
}
  
void log_data() {            // read all data, print them to serial and save them to SD card
    DateTime now = RTC.now();  // get present date-time
                                      
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("datalog.csv", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.print(now.year(), DEC);
      dataFile.print('.');
      dataFile.print(now.month(), DEC);
      dataFile.print('.');
      dataFile.print(now.day(), DEC);
      dataFile.print(' ');
      dataFile.print(now.hour(), DEC);
      dataFile.print('.');
      dataFile.print(now.minute(), DEC);
      dataFile.print('.');
      dataFile.print(now.second(), DEC);
      dataFile.print(", ");
      
      dataFile.print(getWind());
      dataFile.print(", ");
      
      dataFile.print(getGust());
      dataFile.print(", ");
      
      dataFile.print(getUnitRain());
      dataFile.print(", ");
      
      dataFile.print(Temp());
      dataFile.print(", ");
      
      dataFile.print(Light());
      dataFile.print(", "); 
      
      dataFile.print(Humidity());
      dataFile.print(", "); 
      
      dataFile.print(Pressure());
      dataFile.print(", "); 
      
      dataFile.print(Vcc());
      
      dataFile.println("");
      dataFile.close();
    
  }  
    // if the file isn't open, pop up an error:
    else {
      //Serial.println("EE: SD Card I/O Error");
    } 
}


float Temp(){    //returns the temperature from one DS18S20 in DEG Celsius
     byte data[12];
     byte addr[8];
     
     if ( !ds.search(addr)) {
       //no more sensors on chain, reset search
       ds.reset_search();
       return 0;
     }
    
     if ( OneWire::crc8( addr, 7) != addr[7]) {
       //Serial.println("CRC is not valid!");
       return 0;
     }
    
     if ( addr[0] != 0x10 && addr[0] != 0x28) {
       //Serial.print("Device is not recognized");
       return 0;
     }
    
     ds.reset();
     ds.select(addr);
     ds.write(0x44,1); // start conversion, with parasite power on at the end
    
     byte present = ds.reset();
     ds.select(addr);  
     ds.write(0xBE); // Read Scratchpad
    
     
     for (int i = 0; i < 9; i++) { // we need 9 bytes
      data[i] = ds.read();
     }
     
     ds.reset_search();
     
     byte MSB = data[1];
     byte LSB = data[0];
    
     float tempRead = ((MSB << 8) | LSB); //using two's compliment
     float TemperatureSum = tempRead / 16;
     
     return TemperatureSum;      
}


float Humidity()
{
     if(currentMillis - previousMillis_humidity_poll > time_to_poll_humidity) {
      //Serial.println("polling humidity...");   
      previousMillis_humidity_poll = currentMillis;   

      DHT22_ERROR_t errorCode;  
      // The sensor can only be read from every 1-2s, and requires a minimum
      // 2s warm-up after power-on.
      //  Serial.print("Requesting data...");
      errorCode = myDHT22.readData();
      switch(errorCode)
      {
        case DHT_ERROR_NONE:
          Humidity_value = myDHT22.getHumidity();
          return(Humidity_value);
          
          // Alternately, with integer formatting which is clumsier but more compact to store and
    	  // can be compared reliably for equality. See lib example.
          break;
        case DHT_ERROR_CHECKSUM:
          return(1);
          //Serial.println("Checksum error ");
          break;
        case DHT_BUS_HUNG:
          //Serial.println("BUS Hung ");
          return(2);
          break;
        case DHT_ERROR_NOT_PRESENT:
          //Serial.println("Not Present ");
          return(3);
          break;
        case DHT_ERROR_ACK_TOO_LONG:
          //Serial.println("ACK time out ");
          return(4);
          break;
        case DHT_ERROR_SYNC_TIMEOUT:
          //Serial.println("Sync Timeout ");
          return(5);
          break;
        case DHT_ERROR_DATA_TIMEOUT:
          //Serial.println("Data Timeout ");
          return(6);
          break;
        case DHT_ERROR_TOOQUICK:
          //Serial.println("Polled too quick ");
          return(7);
          break;
        }
     }
     else {
       return(Humidity_value);
       }
}


int Light() {
    float sens = 0;
    int x = 0;
    int total = 0;
    int total1 = 0;
    long mapped_sens = 0;
  
    // here we do some readings to set the sensors
    for(x =0; x < LIGHT_AVG_SAMPLES; x++) {
       sens = analogRead(A0);
       total = total + sens;
       }
    sens = total/LIGHT_AVG_SAMPLES;                         // divide the samples readings by .. 
    total = 0;
    total1 = 0;
    //Serial.print("Valore del sensore= ");
    //Serial.println(sens);
    
    mapped_sens= map(sens, 800, 1000, 0, 100);
    mapped_sens= constrain(mapped_sens, 0, 100);
        
    return mapped_sens;
    sens= 0;
    
}


float Vcc() {                          // TODO really useful?
  signed long resultVcc;
  float resultVccFloat;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(10);                           // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                 // Convert
  while (bit_is_set(ADCSRA,ADSC));
  resultVcc = ADCL;
  resultVcc |= ADCH<<8;
  resultVcc = 1126400L / resultVcc;    // Back-calculate AVcc in mV
  resultVccFloat = (float) resultVcc / 1000.0; // Convert to Float
  return resultVccFloat;
}


double getWindVane()
{
  analogReference(DEFAULT);
  digitalWrite(VANE_PWR,HIGH);
  delay(100);
  for(int n=0;n<10;n++)
  {
    analogRead(VANE_PIN);
  }
 
  unsigned int reading=analogRead(VANE_PIN);
  digitalWrite(VANE_PWR,LOW);
  unsigned int lastDiff=2048;
 
  for (int n=0;n<16;n++)
  {
    int diff=reading-pgm_read_word(&vaneValues[n]);
    diff=abs(diff);
    if(diff==0)
       return pgm_read_word(&vaneDirections[n])/10.0;
 
    if(diff>lastDiff)
    {
      return pgm_read_word(&vaneDirections[n-1])/10.0;
    }
 
    lastDiff=diff;
 }
 
  return pgm_read_word(&vaneDirections[15])/10.0;
}

double getUnitRain()
{
  unsigned long reading=rain_count;
  rain_count=0;
  double unit_rain=reading*RAIN_FACTOR;
 
  return unit_rain;
}
 
void rainGaugeClick()
{
    long thisTime=micros()-rain_last;
    rain_last=micros();
    if(thisTime>500)
    {
      rain_count++;
    }
}
 
double getWind()      //RINOMINARE TODO
{
  if(currentMillis - previousMillis_wind > TIME_TO_POLL_WIND) {
    previousMillis_wind=currentMillis;
    Wind = getUnitWind();
    
  }    
return(Wind);
}
 
double getUnitWind()
{
  unsigned long reading=anem_count;
  anem_count=0;
  return (WIND_FACTOR*reading)/(TIME_TO_POLL_WIND/1000);
}
 
double getGust()
{
  unsigned long reading=anem_max;
  double Gust=0;
  //anem_max=0xffffffff;
  double time=reading/1000000.0; 
  Gust=(1/(reading/1000000.0))*WIND_FACTOR;
  
   if(currentMillis - previousMillis_gust > TIME_TO_POLL_WIND) {
      previousMillis_gust=currentMillis;
      anem_max=0xffffffff;
    }
     return(Gust);
}
 
void anemometerClick()
{
  long thisTime=micros()-anem_last;
  anem_last=micros();
  if(thisTime>500)
  {
    anem_count++;
    if(thisTime<anem_max)
    {
      anem_max=thisTime;
    }
  }
}


float Pressure(){
  //float temperatureBMP = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
  //float pressure = bmp085GetPressure(bmp085ReadUP());
  float pressure = bmp085GetPressure(bmp085ReadUP());
  //float atm = pressure / 101325; // "standard atmosphere"
  //float altitude = calcAltitude(pressure); //Uncompensated caculation - in Meters 
  
  //Serial.print("Pressure: ");
  //Serial.print(pressure, 2); // 2 decimals
  //Serial.println(" Pa");
  return((pressure/100)+ALTITUDE_DIFF);
  }

char bmp085Read(byte address)
{
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available()) {};
  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(byte address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}



// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  Serial.println("Calibrating BMP085... ");
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
  //Serial.println("Calibrated!");
}

// Calculate temperature in deg C
float bmp085GetTemperature(unsigned int ut){
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}


// Read 1 byte from the BMP085 at 'address'
// Read the uncompensated temperature value
unsigned int bmp085ReadUT(){
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write((byte)0xF4);
  Wire.write((byte)0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}


/*
float calcAltitude(float pressure){

  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
  C = C /0.0000225577;

  return C;
}
*/

/*
void sleepNow()
{
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
   * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
   * there is a list of sleep modes which explains which clocks and 
   * wake up sources are available in which sleep modes.
   *
   * In the avr/sleep.h file, the call names of these sleep modus are to be found:
   *
   * The 5 different modes are:
   *     SLEEP_MODE_IDLE         -the least power savings 
   *     SLEEP_MODE_ADC
   *     SLEEP_MODE_PWR_SAVE
   *     SLEEP_MODE_STANDBY
   *     SLEEP_MODE_PWR_DOWN     -the most power savings
   *
   *  the power reduction management &lt;avr/power.h&gt;  is described in 
   *  http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
 
      
  set_sleep_mode(SLEEP_MODE_PWR_SAVE); // Sleep mode is set here
   
  sleep_enable();                      // Enables the sleep bit in the mcucr register
                                       // so sleep is possible. just a safety pin 
  sleep_mode();                        // Here the device is actually put to sleep!!
                                       // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  sleep_disable();                     // Dirst thing after waking from sleep:
                                       // disable sleep...
}*/ 
