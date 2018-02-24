#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif
#define DHT22_PIN 2 //define the DHT22 Pin as digital pin 2.

#include "cactus_io_DHT22.h" // Includes the DHT22 library (other libraries were not working in conjunction with PulseSensor as 
#include <Wire.h>
#include <LiquidCrystal_I2C.h> //Includes the I2C LiquidCrystal library.
#include <PulseSensorPlayground.h>     // Includes the PulseSensorPlayground Library.   



LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display

// define LED at pin 13 for RED LED
int REDLED = 13;
int tempAnalogRead;
int tempPin = 1;


int debug = 0;

// Initialize DHT sensor for normal 16mhz Arduino. 
DHT22 dht(DHT22_PIN); //

//define pulse sensor output at A0
const int PulseWire = 0;
PulseSensorPlayground pulseSensor;  // Creates an instance of the PulseSensorPlayground object called "pulseSensor"
int Threshold = 570;

// define special characters to be used later
uint8_t bell[8]  = {0x4, 0xe, 0xe, 0xe, 0x1f, 0x0, 0x4};
uint8_t note[8]  = {0x2, 0x3, 0x2, 0xe, 0x1e, 0xc, 0x0};
uint8_t clock[8] = {0x0, 0xe, 0x15, 0x17, 0x11, 0xe, 0x0};
uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};
uint8_t duck[8]  = {0x0, 0xc, 0x1d, 0xf, 0xf, 0x6, 0x0};
uint8_t check[8] = {0x0, 0x1, 0x3, 0x16, 0x1c, 0x8, 0x0};
uint8_t cross[8] = {0x0, 0x1b, 0xe, 0x4, 0xe, 0x1b, 0x0};
uint8_t retarrow[8] = {  0x1, 0x1, 0x5, 0x9, 0x1f, 0x8, 0x4};

void setup(){
  lcd.init();                      // initialize the lcd
  lcd.backlight();
  lcd.createChar(0, heart); // create the heart character
  lcd.createChar(1, cross); // create the heart character
  lcd.begin(20, 4); // define the LCD for 20 character and 4 lines
  Serial.begin(9600);
  if(debug)
    Serial.println("Health Monitoring System");
  printStartScreen(); // print the start screen
  pinMode(REDLED, OUTPUT); // define Red LED pin i.e. 13 as the output Pin
  pulseSensor.analogInput(PulseWire); // define the pulse sensor analog input at A0
  pulseSensor.setThreshold(Threshold); // set the threshold as 570 to reduce the noise.
  if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object !");  //This prints one time at Arduino power-up,  or on Arduino reset.
  }
  dht.begin(); // begin the DHT (Humidity sensor)
  digitalWrite(REDLED, LOW); // set the Red Led as low.
  delay(5000); // delay of 2 seconds to have the message stay for 2 second initially
}

void printStartScreen(){
  lcd.setCursor(0, 0); // set the cursor to column 0 and row 0
  for(int i = 0;i < 20; i++)  lcd.printByte(1); //print cross from column 0-19 in row 1 based on previous statement
  lcd.setCursor(0, 1); // Set cursor on column 0 and row 1
  lcd.printByte(1);// print cross on column 0 and row 1
  lcd.print("Health Monitoring "); //print the given text
  lcd.printByte(1); // print cross after the text above
  lcd.setCursor(0, 2);// Set cursor on column 0 and row 2
  lcd.printByte(1);// print cross on column 0 and row 2
  lcd.print("      System      ");//print the given text
  lcd.printByte(1);// print cross after the text above
  lcd.setCursor(0, 3);// Set cursor on column 0 and row 3
  for(int i = 0;i < 20; i++)  lcd.printByte(1);//print cross from column 0-19 in row 3 based on previous statement
}

void loop(){

  lcd.init();

  int myBPM = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".
  
  //print the first line on lcd with default value as 0
  lcd.print("Heart Beat Rate:");
  lcd.print("0");
  lcd.printByte(0);

  
  if (pulseSensor.sawStartOfBeat()) {            // Constantly test to see if "a beat happened".
    if(debug){
      Serial.println("♥  A HeartBeat Happened ! "); // If test is "true", print a message "a heartbeat happened".
      Serial.print("BPM: ");                        // Print phrase "BPM: "
      Serial.println(myBPM);                        // Print the value inside of myBPM.
    }
    lcd.setCursor(16,0);// set the cursor in such a way so that BPM prints right after the static text "Health Monitoring:"
    lcd.print(myBPM);
    lcd.printByte(1);//Print Heart
  }

  tempAnalogRead = analogRead(tempPin);// Read the analog reading of temperature
  float voltage = tempAnalogRead * 5.0;// convert the reading to voltage
  voltage /= 1024.0; //divit by 1024

  // if Debug true then print out the voltage
  if(debug){
    Serial.print(voltage); 
    Serial.println(" volts");
  }
  float temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset to degrees ((voltage - 500mV) times 100)


  // if Debug true then now print out the temperature in C
  if(debug){
    Serial.print(temperatureC); 
    Serial.println(" degrees C");
  }
  // now convert to Fahrenheit
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
  
  // if Debug true then print out the temperature in F
  if(debug){
    Serial.print(temperatureF); 
    Serial.println(" degrees F");
  }
  lcd.setCursor(0, 1); //Set the cursor to column 0 and row 1
  lcd.print("Temp: ");
  lcd.print(temperatureF);
  lcd.print("*F");

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  dht.readHumidity();
  //dht.readTemperature();
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(dht.humidity) || isnan(dht.temperature_C)) {
    if(debug)
      Serial.println("DHT sensor read failure!");
    return;
  }
  float humidity = dht.humidity;
  if(debug){
    Serial.print(dht.humidity); Serial.print(" %\t\t");
    /*Serial.print(dht.temperature_C); Serial.print(" *C\t");
    Serial.print(dht.temperature_F); Serial.print(" *F\t");
    Serial.print(dht.computeHeatIndex_C()); Serial.print(" *C\t");
    Serial.print(dht.computeHeatIndex_F()); Serial.println(" *F");*/
  }
  lcd.setCursor(0,2);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");

  if (myBPM > 72 || temperatureF >= 100 || humidity>65)
    digitalWrite(REDLED, HIGH);
  else
    digitalWrite(REDLED, LOW);


  
  // Wait a few seconds between measurements. The DHT22 should not be read at a higher frequency of
  // about once every 2 seconds. So we add a 3 second delay to cover this.
  delay(3000);

  /* if(!digitalRead(Reset))
    {
     rate=0;
      lcd.init();
       lcd.print("Heart Beat Rate:");
       lcd.setCursor(0,1);
       lcd.printByte(1);
       lcd.print(rate);
       k=0;
    }*/
}

