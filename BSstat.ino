/*
BSstat
home smart thermostat
documentation coming soon...
*/

#include "credentials.h"
#include <Wire.h>                     // need for I2C
#include <Adafruit_GFX.h>             // need to include this for some reason
#include "Adafruit_LEDBackpack.h"     // library for the display
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <TimeLib.h>
#include <WidgetRTC.h>


//GPIO pins for encoder and switch
#define PIN_ENCODER_A      D3
#define PIN_ENCODER_B      D5
#define PIN_ENCODER_SWITCH D4
// A, B, C are thermistor constants
#define A .001129148
#define B .000234125
#define C .0000000876741

// Some initial states for the encoder
static uint8_t enc_prev_pos   = 0;
static uint8_t enc_flags      = 0;
static char    sw_was_pressed = 0;
 
//GPIO pins for temperature input and relay out
const int temperaturePin = 0;
const int relayPin = D7;
int setpoint, alarmOn;
long startTimeInSecs;

Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();   //create object for the 4 digit display

BlynkTimer timer;

WidgetRTC rtc;


void setup() {
  Serial.begin(9600);
  delay(10);

  alpha4.begin(0x70);  // pass in the address
  alpha4.setBrightness(10);  // brightness from 0 10 15

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
  alarmOn = 0;      // initialize alarm flag

  // these will come from credentials.h
  Blynk.begin(blynkcred, ssidcred, passcred);

  setSyncInterval(10 * 60);   // Sync interval in sec, 10 min

  timer.setInterval(10L, thermostat);
  timer.setInterval(1000L, vpins);
  
}

char displaybuffer[4] = {' ', ' ', ' ', ' '};

char* disp = (char*)malloc(30 * sizeof(char));

//float voltage, Rsense, logR;
float degreesF = 68.0;
float prev = 68.0;    // record previous temp to filter large jumps in sensor

bool heat = false;    // State, is heat on or off
int temp;
int tnow;

void thermostat() {
 
  ////////////////// lots of code stolen from adafruit to read the encoder...
  // Loop thru this many times because you need to read the encoder very fast...
  // But don't do it for too long or you will trigger ESP8266 watchdog timer reset
  for (int i = 0; i < 10000; i++)
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
        if (enc_flags & 0x01 && (enc_flags & 0x14))
        {
          enc_action = 1;
        }
        else if (enc_flags & 0x04 && enc_flags & 0x11)
        //(bit_is_set(enc_flags, 2) && (bit_is_set(enc_flags, 0) || bit_is_set(enc_flags, 4)))
        {
          enc_action = 1;
        }
        else if (enc_flags & 0x02 && enc_flags & 0x18)
        //(bit_is_set(enc_flags, 1) && (bit_is_set(enc_flags, 3) || bit_is_set(enc_flags, 4)))
        {
          enc_action = -1;
        }
        else if (enc_flags & 0x11 && enc_flags & 0x12)
        //(bit_is_set(enc_flags, 3) && (bit_is_set(enc_flags, 1) || bit_is_set(enc_flags, 4)))
        {
          enc_action = -1;
        }
   
        enc_flags = 0; // reset for next time
      }
    }
   
    enc_prev_pos = enc_cur_pos;
    
    //////////// done reading the encoder
    if (enc_action > 0 && setpoint < 75) 
    {
      setpoint++;  // clockwise, turn setpoint up 1 degree F
    }
    else if (enc_action < 0 && setpoint > 45)
    {
      setpoint--; // widdershins, turn setpoint down 1 degree F
    }
    // remember that the switch is active low 
    // Some setpoints for the pushbutton, 68 for home and 54 for away
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
  }   //end of big encoder loop

  // Timer/alarm functionality
  time_t t = now();
  tnow = hour(t) * 3600 + minute(t) * 60 + second(t);  // time in seconds since midnight
  if (tnow == startTimeInSecs && alarmOn)
  {
    setpoint = 68;
    alarmOn = 0;
  }
  
  // Reading the thermistor using the Steinhart–Hart equation
  degreesF = getVoltage(temperaturePin);
  degreesF = 10000*degreesF/(3.3-degreesF);
  degreesF = log(degreesF);
  degreesF = 1 / (A + B * degreesF + C * degreesF * degreesF * degreesF) - 273.15;
  degreesF = degreesF * 1.8 + 32;
  float stp = 0.1;
  degreesF = (prev + degreesF)/2;
  if (degreesF > prev + stp)
  {
    degreesF = prev + stp; 
  }
  else if (degreesF < prev - stp)
  {
    degreesF = prev - stp;
  }
  prev = degreesF;

  temp = (int)degreesF;
  float delta = degreesF - setpoint;

  
  // Turn the heat on or off using some histeresis
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
  //Serial.print("voltage: ");
  //Serial.print(voltage);
  //Serial.print("  resistance: ");
  //Serial.print(Rsense);  
  Serial.print("  delta: ");
  Serial.print(delta);
  Serial.print("  deg F: ");
  Serial.print(degreesF);
  Serial.print("  alarm: ");
  Serial.print(alarmOn);
  Serial.print("  time now: ");
  Serial.print(tnow);
  Serial.print("  alm time: ");
  Serial.println(startTimeInSecs);
  
  // scroll down display
  displaybuffer[0] = setpoint / 10 + '0';    //Tens place
  displaybuffer[1] = setpoint % 10 + '0';    //Ones place
  displaybuffer[2] = temp / 10 + '0';        //Tens place
  displaybuffer[3] = temp % 10 + '0';        //Ones place
  
  // set every digit to the buffer
  alpha4.writeDigitAscii(0, displaybuffer[0]);
  alpha4.writeDigitAscii(1, displaybuffer[1]);
  alpha4.writeDigitAscii(2, displaybuffer[2]);
  alpha4.writeDigitAscii(3, displaybuffer[3]);

  // write it out!
  alpha4.writeDisplay();

}

void vpins(void)
{
  Blynk.virtualWrite(V0, temp);
  Blynk.virtualWrite(V1, setpoint);
  if (alarmOn)
  {
    sprintf(disp, "Alarm is set to %i:%2.2i.", startTimeInSecs/3600, (startTimeInSecs % 3600) / 60);
  }
  else
  {
    sprintf(disp, "Alarm is off.");
  }
  Blynk.virtualWrite(V5, disp);
  Serial.println(disp);
  
}

// get the setpoint from the slider in Blynk app
BLYNK_WRITE(V2)
{
  setpoint = param.asInt();
}

// Get the time input from Blynk app
BLYNK_WRITE(V3)
{
  startTimeInSecs = param[0].asLong();
}

// Get alarm on/off button from Blynk app
BLYNK_WRITE(V4)
{
  alarmOn = param.asInt();
}

float getVoltage(int pin)
{ 
  return (analogRead(pin) * 3.3/1023);
}

BLYNK_CONNECTED()
{
  // Synchronize time on connection
  rtc.begin();
}

void loop()
{
  Blynk.run();
  timer.run();
}




