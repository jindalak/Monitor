#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.

#include "cactus_io_DHT22.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PulseSensorPlayground.h>     // Includes the PulseSensorPlayground Library.   

#define DHT22_PIN 2

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display

int Led = 12;
int tempPin = 1;
int debug = 1;
DHT22 dht(DHT22_PIN);

const int PulseWire = 0;
int tempAnalogRead;
PulseSensorPlayground pulseSensor;  // Creates an instance of the PulseSensorPlayground object called "pulseSensor"
int Threshold = 570;

uint8_t bell[8]  = {0x4, 0xe, 0xe, 0xe, 0x1f, 0x0, 0x4};
uint8_t note[8]  = {0x2, 0x3, 0x2, 0xe, 0x1e, 0xc, 0x0};
uint8_t clock[8] = {0x0, 0xe, 0x15, 0x17, 0x11, 0xe, 0x0};
uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};
uint8_t duck[8]  = {0x0, 0xc, 0x1d, 0xf, 0xf, 0x6, 0x0};
uint8_t check[8] = {0x0, 0x1, 0x3, 0x16, 0x1c, 0x8, 0x0};
uint8_t cross[8] = {0x0, 0x1b, 0xe, 0x4, 0xe, 0x1b, 0x0};
uint8_t retarrow[8] = {  0x1, 0x1, 0x5, 0x9, 0x1f, 0x8, 0x4};

void setup()
{
  lcd.init();                      // initialize the lcd
  lcd.backlight();
  lcd.createChar(1, heart);
  lcd.begin(20, 4);
  Serial.begin(9600);
  Serial.println("Heart Beat Monitoring");
  lcd.print("Heart Beat ");
  lcd.printByte(1);
  lcd.setCursor(0, 1);
  lcd.print("Monitering");
  pinMode(Led, OUTPUT);
  pulseSensor.analogInput(PulseWire);
  pulseSensor.setThreshold(Threshold);
  if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object !");  //This prints one time at Arduino power-up,  or on Arduino reset.
  }
  dht.begin();
  digitalWrite(Led, LOW);
  //delay(1000);
}

void loop()
{

  lcd.init();

  Serial.println(analogRead(PulseWire));
  int myBPM = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".
  //Serial.println(myBPM);                                               // "myBPM" hold this BPM value now.

  if (pulseSensor.sawStartOfBeat()) {            // Constantly test to see if "a beat happened".
    Serial.println("♥  A HeartBeat Happened ! "); // If test is "true", print a message "a heartbeat happened".
    Serial.print("BPM: ");                        // Print phrase "BPM: "
    Serial.println(myBPM);                        // Print the value inside of myBPM.
  }
  lcd.init();
  lcd.print("Heart Beat Rate:");
  //lcd.setCursor(0,1);
  lcd.print(myBPM);
  //lcd.print(" ");
  lcd.printByte(1);
  if (myBPM > 72)
  {
    digitalWrite(Led, HIGH);
  }
  else
    digitalWrite(Led, LOW);

  tempAnalogRead = analogRead(tempPin);
  float voltage = tempAnalogRead * 5.0;
  voltage /= 1024.0;

  // print out the voltage
  Serial.print(voltage); Serial.println(" volts");

  // now print out the temperature
  float temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
  //to degrees ((voltage - 500mV) times 100)


  Serial.print(temperatureC); Serial.println(" degrees C");

  // now convert to Fahrenheit
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
  Serial.print(temperatureF); Serial.println(" degrees F");

  lcd.setCursor(0, 1);
  // Uncomment this if you need to display tem on lcd screen
  lcd.print("Temp: ");
  lcd.print(temperatureF);
  lcd.print("*F");

  if (temperatureF >= 100)
  {
    digitalWrite(Led, HIGH);
  }

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  dht.readHumidity();
  dht.readTemperature();
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(dht.humidity) || isnan(dht.temperature_C)) {
    Serial.println("DHT sensor read failure!");
    return;
  }
  float humidity = dht.humidity;
  Serial.print(dht.humidity); Serial.print(" %\t\t");
  Serial.print(dht.temperature_C); Serial.print(" *C\t");
  Serial.print(dht.temperature_F); Serial.print(" *F\t");
  Serial.print(dht.computeHeatIndex_C()); Serial.print(" *C\t");
  Serial.print(dht.computeHeatIndex_F()); Serial.println(" *F");

  lcd.setCursor(0,2);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");
  
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

