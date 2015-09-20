/* Weather Station connected to www.wunderground.com
 * Parts used Arduino Version:
 * - Davis Vantage Pro anemometer (wind speed, wind direction)
 * - Arduino Uno
 * - Ethernet Shield
 * - DHT21 temperature and humidity sensor
 * 
 * Parts used Teensy 3.1 Version:
 * - Davis Vantage Pro anemometer (wind speed, wind direction)
 * - Teensy 3.1
 * - WIZ820io & Micro SD Card Adaptor
 * - WIZ820io ethernet module
 * - DHT21 temperature and humidity sensor
 * - BMP180 barometric pressure sensor
 * 
 * Some code is based on https://gist.github.com/jaymham/7792782
 * More info about how to connect to wunderground.com can be found 
 * here: http://wiki.wunderground.com/index.php/PWS_-_Upload_Protocol
 * 
 *
 * Weather Station by George Timmermans
 * http://www.georgetimmermans.com/weather-station.html
*/

#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

/* Pins used Arduino Uno + Ethernet shield:
 * A0 - Wind Direction
 * D2 - Anemometer
 * D7 - DHT21
 * D10 - Ethernet shield
 * D11 - Ethernet shield
 * D12 - Ethernet shield
 * D13 - Ethernet shield
 */

/* Pins used Teensy 3.1 + Wiznet WIZ820io ethernet module :
 * A1 - Wind Direction
 * 14 - Anemometer
 * D7 - DHT21
 * D10 - Ethernet shield
 * D11 - Ethernet shield
 * D12 - Ethernet shield
 * D13 - Ethernet shield
 */
 
// assign a MAC address for the ethernet controller.
// fill in your address here:
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

// fill in an available IP address on your network here,
// for manual configuration:
IPAddress ip(192,168,1,177);
// fill in your Domain Name Server address here:
IPAddress myDns(8,8,8,8);
// initialize the library instance:
EthernetClient client;

// Davis Vantage pro windvane/anemometer
#define VANEPIN A1
#define ANEMOISR 14
const double WindTo_mph = 2.25;   // Davis = 2.25

// Temperature/Humidity sensor
#define DHTTYPE DHT21     
#define DHTPIN  7
DHT_Unified dht(DHTPIN, DHTTYPE, 30);

// BMP085 Baromeric Pressure Sensor
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// Wunderground 
//char server[] = "weatherstation.wunderground.com";
char server[] = "rtupdate.wunderground.com";
char ID[] = "*******";   //your registered station id
char PASSWORD[] = "*******";  //your wunderground password

// Status LED
const int ledPin =  0;      // the number of the LED pin
int ledState = LOW;             // ledState used to set the LED

//variables
unsigned long lastConnectionTime = 0;          // last time you connected to the server, in milliseconds
boolean lastConnected = false;                 // state of the connection last time through the main loop
const unsigned long postingInterval = 5*1000;  // delay between updates, in milliseconds

long previousMillis = 0;
unsigned long currentMillis;
unsigned long updateLED = 1000;
unsigned long updateAverage = 5000; 
unsigned long update2minAverage = 120000;
unsigned long updateWunderground = 5000;

unsigned int winddir; //wind vane
double DirectionVolt; //wind vane
double tempf, humidity, dewptf, indoortempf; // Temp/hum sensor
double baromin; // barometric pressure

unsigned long PulseTimeNow = 0; // Time stamp (in millisecons) for pulse triggering the interrupt
double WindSpeed_mps, WindSpeed_cnt,WindSpeed_rpm;
double WindSpeed_mph = 0.0;
double WindSpeed_Hz = 0.0;
volatile unsigned long PulseTimeLast = 0; // Time stamp of the previous pulse
volatile unsigned long PulseTimeInterval = 0; // Time interval since last pulse
 
volatile unsigned long PulsesCumulatedTime = 0; // Time Interval since last wind speed computation 
volatile unsigned long PulsesNbr = 0;           // Number of pulses since last wind speed computation 
volatile unsigned long LastPulseTimeInterval = 1000000000;
volatile unsigned long MinPulseTimeInterval = 1000000000;
double MaxWind = 0.0; // Max wind speed 
double WindGust = 0.0;

//***************************************************************************************************

void setup() {
  pinMode(4, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(ANEMOISR, INPUT);
  pinMode(ledPin, OUTPUT);    
  
  attachInterrupt(ANEMOISR, AnemometerPulse, FALLING);
  PulseTimeLast = micros();
  
  // start serial port:
  Serial.begin(115200);

  // start barometric pressure sensor
  bmp.begin();
  
  // give the ethernet module time to boot up:
  delay(1000);
  
  Serial.println("Configuring ethernet connection...");
  if (Ethernet.begin(mac) == 0)  {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
    //Ethernet.begin(mac, ip, myDns);
    Ethernet.begin(mac, ip);
    // start the Ethernet connection using a fixed IP address and DNS server:
  }
  
  // print the Ethernet board/shield's IP address:
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
}

//***************************************************************************************************

void loop()  {
  // if there's incoming data from the net connection.
  // send it out the serial port.  This is for debugging
  // purposes only:
  if (client.available()) {
    char c = client.read();
    Serial.print(c);
  }
  
  // if there's no net connection, but there was one last time
  // through the loop, then stop the client:
  if (!client.connected() && lastConnected) {
    Serial.println();
    Serial.println("disconnecting.");
    client.stop();
  }
  
  currentMillis = millis();
  unsigned long timePassed = currentMillis - previousMillis;
  
  if(timePassed > updateLED) {
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
  
  if(timePassed > updateAverage) {
    previousMillis = currentMillis;   
    readSensors();
  }

  // if you're not connected, and one minute have passed since
  // your last connection, then connect again and send data:
  if(!client.connected() && ( currentMillis - lastConnectionTime > updateWunderground)) {
    httpRequest();
  }
  
  // store the state of the connection for next time through
  // the loop:
  lastConnected = client.connected();
}

//***************************************************************************************************

// this method makes a HTTP connection to the server:
void httpRequest() {
  
  if (isnan(WindSpeed_mph)) // filter results in case Anemometer is left unplugged.
    WindSpeed_mph = 0;
  if (WindSpeed_mph > 250) 
    WindSpeed_mph = 0;  
  if (isnan(WindGust))
    WindGust = 0; 
  if (WindGust > 250) 
    WindGust = 0;  
    
  Serial.println("Trying to connect to wunderground");
  // if there's a successful connection:
  if (client.connect(server, 80)) { 
    Serial.println("connecting...");
    // send the HTTP PUT request:
    client.print("GET /weatherstation/updateweatherstation.php?");
    client.print("ID=");
    client.print(ID);
    client.print("&PASSWORD=");
    client.print(PASSWORD);
    client.print("&dateutc=now");

    client.print("&tempf=");
    client.print(tempf);
    
    client.print("&humidity=");
    client.print(humidity);
    
    client.print("&dewptf=");
    client.print(dewptf);

    client.print("&indoortempf=");
    client.print(indoortempf);

    client.print("&winddir=");
    client.print(winddir); 

    client.print("&windspeedmph=");
    client.print(WindSpeed_mph); 
   
    client.print("&windgustmph=");
    client.print(WindGust); 

    client.print("&baromin=");
    client.print(baromin); 
    
    client.print("&action=updateraw");
    client.println("&realtime=1&rtfreq=5");
    client.println();
    
    // note the time that the connection was made:
    lastConnectionTime = millis();
    
    Serial.println("Send");
    Serial.println();
    
    MinPulseTimeInterval = 1000000000;
    LastPulseTimeInterval = 1000000000;

  } else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
    Serial.println("disconnecting.");
    client.stop();
  }
}

//***************************************************************************************************

void readSensors()  {
  readDHT();
  readBMP();
  readWindDirection();
  AnemometerLoop();
 
  Serial.println();
}

//***************************************************************************************************

void readDHT()  {
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
    tempf = 0;
  }
  else {
    tempf = event.temperature;
    Serial.print("Temperature:    ");
    Serial.print(tempf);
    Serial.println(" *C");
    tempf = tempf * 9 / 5 + 32;
    Serial.print("Temperature:    ");
    Serial.print(tempf);
    Serial.println(" *F");
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
    humidity = -1;
  }
  else {
    Serial.print("Humidity:       ");
    Serial.print(event.relative_humidity);
    Serial.println("%");
    humidity = event.relative_humidity;
  }
  dewptf = (dewPoint(tempf, humidity)); //Dew point calc(wunderground)
  Serial.print("dewPoint:       ");
  Serial.print(dewptf);
  Serial.println(" *F");
  Serial.print("dewPoint:       ");
  Serial.print(dewptf * 9 / 5 + 32);
  Serial.println(" *C");
}

//***************************************************************************************************

void readBMP() {
  sensors_event_t event;
  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure) {
    baromin = event.pressure;
    /* Display atmospheric pressue in hPa */
    Serial.print("Pressure:       ");
    Serial.print(baromin);
    Serial.println(" hPa");

    baromin = baromin / 33.8638866667;
    Serial.print("Pressure:       ");
    Serial.print(baromin);
    Serial.println(" inHg");
    
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print("indoortempf:    ");
    Serial.print(temperature);
    Serial.println(" C");
    temperature = temperature * 9 / 5 + 32;
    Serial.print("indoortempf:    ");
    Serial.print(temperature);
    Serial.println(" *F");
    indoortempf = temperature;

  } else  {
    baromin = -1;
    indoortempf = -1;
    Serial.println("Error reading barometric pressure!");
  }
}

//***************************************************************************************************

void AnemometerPulse()  {
  noInterrupts();             // disable global interrupts
  PulseTimeNow = micros();   // Micros() is more precise to compute pulse width that millis();
  PulseTimeInterval = PulseTimeNow - PulseTimeLast;
  PulseTimeLast = PulseTimeNow;
  PulsesCumulatedTime = PulsesCumulatedTime + PulseTimeInterval;
  PulsesNbr++;
 
  if ( PulseTimeInterval < LastPulseTimeInterval ) {// faster wind speed == shortest pulse interval
    MinPulseTimeInterval = PulseTimeInterval;
    LastPulseTimeInterval = MinPulseTimeInterval;
  }
  interrupts();              // Re-enable Interrupts
}

//***************************************************************************************************

void AnemometerLoop ()  {  // START Anemometer section
  noInterrupts();  // precaution using shared resources.
  WindSpeed_Hz = 1000000.0*PulsesNbr/PulsesCumulatedTime; 
  MaxWind      = 1000000*WindTo_mph/MinPulseTimeInterval;
  interrupts();
  
  WindSpeed_mph=WindSpeed_Hz*WindTo_mph;
  
  Serial.print("Wind speed Hz:     ");  
  Serial.print(WindSpeed_Hz); 
  Serial.println(" hz");  

  Serial.print("Wind speed:     ");  
  Serial.print(WindSpeed_mph); 
  Serial.println(" mph");  
       
  PulsesCumulatedTime = 0;
  PulsesNbr = 0;
  WindGust = MaxWind;
  MaxWind = 0;
  
  Serial.print("Wind gust:      ");  
  Serial.print(WindGust); 
  Serial.println(" mph");
}

//***************************************************************************************************

void readWindDirection()  {
  DirectionVolt = analogRead (VANEPIN);
  winddir = (DirectionVolt / 1024.0) * 360.0; 
         
  if (winddir > 360 ) { 
    winddir = winddir - 360; }
  if (winddir < 0 ) 
    { winddir = winddir + 360; } 
  Serial.print("Wind direction: ");  
  Serial.print(winddir); 
  Serial.println(" degrees");  
}

//***************************************************************************************************

double dewPoint(double tempf, double humidity)  {
  double A0 = 373.15/(273.15 + tempf);
  double SUM = -7.90298 * (A0-1);
  SUM += 5.02808 * log10(A0);
  SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/A0)))-1) ;
  SUM += 8.1328e-3 * (pow(10,(-3.49149*(A0-1)))-1) ;
  SUM += log10(1013.246);
  double VP = pow(10, SUM-3) * humidity;
  double T = log(VP/0.61078);   
  return (241.88 * T) / (17.558-T);
}

//***************************************************************************************************
