/*
BSstat
home smart thermostat
documentation coming soon...
*/

#include <Wire.h>                     // need for I2C
#include <Adafruit_GFX.h>             // need to include this for some reason
#include "Adafruit_LEDBackpack.h"     // library for the display
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

//GPIO pins for encoder and switch
#define PIN_ENCODER_A      5    // D3
#define PIN_ENCODER_B      4    // D5
#define PIN_ENCODER_SWITCH 2    // D4

// WiFi access point
// paste in wifi name and pass
#define WLAN_SSID ""
#define WLAN_PASS ""

// Adafruit.io
#define AIO_SERVER        "io.adafruit.com"
#define AIO_SERVERPORT    1883
#define AIO_USERNAME      "paste your username here"
#define AIO_KEY           "paste your key here"

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish pubtemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp");

// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe subsetpoint = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/setpoint");

Adafruit_MQTT_Publish pubsetpoint = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/setpoint");

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

// Some initial states for the encoder
static uint8_t enc_prev_pos   = 0;
static uint8_t enc_flags      = 0;
static char    sw_was_pressed = 0;
 

Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();   //create object for the 4 digit display

//GPIO pins for temperature input and relay out
const int temperaturePin = 0;
const int relayPin = 13;    //D7
int setpoint;



void setup()
{
  Serial.begin(115200);
  delay(10);

  Serial.println(F("\nAdafruit MQTT demo"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&subsetpoint);


  alpha4.begin(0x70);  // pass in the address
  alpha4.setBrightness(10 );  // brightness from 0 10 15

  pinMode(relayPin, OUTPUT);  // set pin as an output


  alpha4.clear();
  alpha4.writeDisplay();
  
  // set pins as input with internal pull-up resistors enabled
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_SWITCH, INPUT_PULLUP);
 
  // get an initial reading on the encoder pins
  if (digitalRead(PIN_ENCODER_A) == LOW)
  {
    enc_prev_pos |= (1 << 0);
  }
  if (digitalRead(PIN_ENCODER_B) == LOW)
  {
    enc_prev_pos |= (1 << 1);
  }

  setpoint = 68;    // default setpoint to something reasonable (degrees F)
  MQTT_connect();
  if (! pubsetpoint.publish(setpoint)) {
    Serial.println(F("Failed"));
  } 
  else {
    Serial.println(F("OK!"));
  } 

  
}



char displaybuffer[4] = {' ', ' ', ' ', ' '};

float voltage, degreesF, Rsense, logR;
float A = .001129148;
float B = .000234125;
float C = .0000000876741;
bool heat = false;


void loop()
{
  
    // Ensure the connection to the MQTT server is alive (this will make the first
    // connection and automatically reconnect when disconnected).  See the MQTT_connect
    // function definition further below.
    MQTT_connect();
  
  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &subsetpoint) {
      Serial.print(F("Got: "));
      setpoint = (int)subsetpoint.lastread;
      Serial.println((char *)setpoint);
      
    }
  }
  ////////////////// lots of code stolen from adafruit to read the encoder...
  for (int i = 0; i < 1000000; i++)
  {
    if (i % 100000 == 0)
    {
      yield();
    }
    
    int8_t enc_action = readEncoder();
    
    //////////// done reading the encoder
  
    if (enc_action > 0 && setpoint < 75) 
    {
      setpoint++;  // clockwise, turn setpoint up 1 degree F
        // Now we can publish stuff!
        Serial.print(F("\nEncoder incremented; Sending temp val "));
        Serial.print(setpoint);
        Serial.print("...");
        if (! pubsetpoint.publish(setpoint)) {
          Serial.println(F("Failed"));
        } 
        else {
          Serial.println(F("OK!"));
        }
    }
    else if (enc_action < 0 && setpoint > 45)
    {
      setpoint--; // widdershins, turn setpoint down 1 degree F
        // Now we can publish stuff!
        Serial.print(F("\nEncoder decremented; Sending temp val "));
        Serial.print(setpoint);
        Serial.print("...");
        if (! pubsetpoint.publish(setpoint)) {
          Serial.println(F("Failed"));
        } 
        else {
          Serial.println(F("OK!"));
        }
    }

    // remember that the switch is active low 
    if (digitalRead(PIN_ENCODER_SWITCH) == LOW) 
    {
      if (sw_was_pressed == 0) // only on initial press, so the keystroke is not repeated while the button is held down
      {
        if (setpoint == 68)
        {
          setpoint = 54;
        }
        else
        {
          setpoint = 68;
        }
        delay(5); // debounce delay
      }
      sw_was_pressed = 1;
    }
    else
    {
      if (sw_was_pressed != 0) {
        delay(5); // debounce delay
      }
      sw_was_pressed = 0;
    }
  }
  
  voltage = getVoltage(temperaturePin);
  Rsense = 10000*voltage/(5-voltage);
  logR = log(Rsense);
  degreesF = 1 / (A + B * logR + C * logR * logR * logR) - 273.15;
  degreesF = degreesF * 1.8 + 32;

  int temp = (int)degreesF;

  Serial.print(F("\nSending temp val "));
  Serial.print(temp);
  Serial.print("...");
  if (! pubtemp.publish(temp)) {
    Serial.println(F("Failed"));
  } 
  else {
    Serial.println(F("OK!"));
  }
  
  float delta = degreesF - setpoint;
  
  if (delta < -1 && !heat)
  {
    heat = true;
    digitalWrite(relayPin, HIGH);
  }
  else if (delta > 0.5 && heat)
  {
    heat = false;
    digitalWrite(relayPin, LOW);
  }


  Serial.print("voltage: ");
  Serial.print(voltage);
  Serial.print("  resistance: ");
  Serial.print(Rsense);  
  Serial.print("  delta: ");
  Serial.print(delta);
  Serial.print("  deg F: ");
  Serial.println(degreesF);


  int zero = 48;      //48 is zero on ascii table
  
  // scroll down display
  displaybuffer[0] = setpoint / 10 + zero;    //Tens place
  displaybuffer[1] = setpoint % 10 + zero;    //Ones place
  displaybuffer[2] = temp / 10 + zero;        //Tens place
  displaybuffer[3] = temp % 10 + zero;        //Ones place
  
  // set every digit to the buffer
  alpha4.writeDigitAscii(0, displaybuffer[0]);
  alpha4.writeDigitAscii(1, displaybuffer[1]);
  alpha4.writeDigitAscii(2, displaybuffer[2]);
  alpha4.writeDigitAscii(3, displaybuffer[3]);

  // write it out!
  alpha4.writeDisplay();
 
  //delay(100); // repeat once per second (change as you wish!)

  
}


void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}

int8_t readEncoder()
{

    int8_t enc_action = 0; // 1 or -1 if moved, sign is direction
    
      // note: for better performance, the code will use
    // direct port access techniques
    // http://www.arduino.cc/en/Reference/PortManipulation
    uint8_t enc_cur_pos = 0;
    // read in the encoder state first
    if (digitalRead(PIN_ENCODER_A) == LOW)
    {
      enc_cur_pos |= (1 << 0);
    }
    if (digitalRead(PIN_ENCODER_B) == LOW)
    {
      enc_cur_pos |= (1 << 1);
    }
   
    // if any rotation at all
    if (enc_cur_pos != enc_prev_pos)
    {
      if (enc_prev_pos == 0x00)
      {
        // this is the first edge
        if (enc_cur_pos == 0x01)
        {
          enc_flags |= (1 << 0);
        }
        else if (enc_cur_pos == 0x02)
        {
          enc_flags |= (1 << 1);
        }
      }
   
      if (enc_cur_pos == 0x03)
      {
        // this is when the encoder is in the middle of a "step"
        enc_flags |= (1 << 4);
      }
      else if (enc_cur_pos == 0x00)
      {
        // this is the final edge
        if (enc_prev_pos == 0x02)
        {
          enc_flags |= (1 << 2);
        }
        else if (enc_prev_pos == 0x01)
        {
          enc_flags |= (1 << 3);
        }
   
        // check the first and last edge
        // or maybe one edge is missing, if missing then require the middle state
        // this will reject bounces and false movements
        //if ( bit_is_set(enc_flags, 0) && ( bit_is_set(enc_flags, 2) || bit_is_set(enc_flags, 4) ) )
        if (enc_flags & ~0x01 && (enc_flags & ~0x14))
        {
          enc_action = 1;
        }
        else if (enc_flags & ~0x04 && enc_flags & ~0x11)
        //(bit_is_set(enc_flags, 2) && (bit_is_set(enc_flags, 0) || bit_is_set(enc_flags, 4)))
        {
          enc_action = 1;
        }
        else if (enc_flags & ~0x02 && enc_flags & ~0x18)
        //(bit_is_set(enc_flags, 1) && (bit_is_set(enc_flags, 3) || bit_is_set(enc_flags, 4)))
        {
          enc_action = -1;
        }
        else if (enc_flags & ~0x11 && enc_flags & ~0x12)
        //(bit_is_set(enc_flags, 3) && (bit_is_set(enc_flags, 1) || bit_is_set(enc_flags, 4)))
        {
          enc_action = -1;
        }
   
        enc_flags = 0; // reset for next time
      }
    }
   
    enc_prev_pos = enc_cur_pos;

    return enc_action;
}

float getVoltage(int pin)
{

  return (analogRead(pin) * 0.004882814);
  
  // This equation converts the 0 to 1023 value that analogRead()
  // returns, into a 0.0 to 5.0 value that is the true voltage
  // being read at that pin.
}


