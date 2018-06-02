/*
 * Temperature Monitor Version 1.7
 * Author: matthew.j.graham@gmail.com
 * Build Date: 30 May 2018
 *  ---------------------------------------------------------------------------------------------------
 * Changes:  
 *  Removed statusPIN (builtinLED) - lines are commented out for now.
 *  Added sevenSegment library for driving 7-seg display with decimal point to show temp
 *    7-seg display connects as follows:
 *         _a_
         f|   |b
          |_g_|
         e|   |ci2
          |_d_|   . P
  
          array goes in order of {a,b,c,d,e,f,g,P} for a total of eight bits.

 * Overview:
 *  This sketch reads data in from a DS18B20 on PIN 2 using the One Wire protocol
 *  The temperature will be read from the sensor once every 5 seconds (roughly)
 *  High temperature will be stored to EEPROM starting at byte 0, consuming 3 bytes
 *  Low temperature will be stored to EEPROM starting at byte 3, consuming 3 bytes
 *  The onboard LED will be used to denote that temp is within range (flashing once per second)
 *  Aux LEDs can be used as follows:
 *          Good LED (Green) on pin 14 (will flash every second if temp is within range)
 *          Warning LED (Red) on pin 15 (will light solid if temp is out of range)
 *  Temperatures are collected in Fahrenheit
 *  Default high limit is 88F
 *  Default low limit is 28F
 *  A pushbutton can be connected to pin 3 using an internal pullup resistor
 *      When pressed the banner will display on the serial port showing high and low values
 *      When held for 10 loops, the EEPROM values will be cleared and collection will restart
 *            Good, Warning, and Status LED's will be lit for 3 seconds during factory reset procedure
 *  A pushbutton can be connected to pin A0 (14) using an internal pullup resistor
 *      When pressed, a seven-segment display will show the current temperature
 *  ---------------------------------------------------------------------------------------------------
 *  Notes:
 *    - Feel free to change anything you like
 *    - If you come up with some sort of novel use for this sketch, send me an email and let me know about it
 *    - If you find any errors with the code,send me an email and let me know about it
 *    
 */

//Include libraries
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <Wire.h>
#include <sevenSegment.h>

int ssPins[8] = {4,5,6,7,8,9,10,11};
sevenSegment ss(ssPins);

/*------------------------------------------------------------------------------------
                INITIALIZE LIBRARIES
------------------------------------------------------------------------------------*/
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2          // Data wire is plugged into pin 2 on the Arduino
#define SLAVE_ADDRESS 0x6f        // Arduino is configured as i2c slave address 0x6f (which rarely gets  used in examples)

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

/*------------------------------------------------------------------------------------
                GLOBAL VARIABLES
------------------------------------------------------------------------------------*/
//const int statusPIN =  LED_BUILTIN;					// the number of the LED pin (13 on an Arduino UNO)
const int goodPIN = 14;								// Good indicator LED pin (green LED)
const int warningPIN = 15;							// Bad indicator LED pin (red LED)
const int infoPIN = 3;								// The input button pin number
const int dispPin = 12;
const float high_limit = 88.0;            // High Temp Limit
const float low_limit = 28.0;           // Low Temp Limit
const unsigned long tempCheckInterval = 4221;		// Check temp every 5 seconds (takes about 779 ms to read the sensor)
const int blinkDelay = 1000;                // Time in between blinks of the good LED
const int blinkDuration = 100;              // How long hte good LED stays on for
float CurrentTemp = 0;
unsigned long goodMillis = 0;						// The amount of time since the previous good LED blink sequence has started in milliseconds
int loopCount = 0;									// A simple loop counter
float high_temp;									// The stored high temp in EEPROM
float low_temp;										// The stored low temp in EEPROM
unsigned long lastTempCheckMillis = millis(); 		// Timestamp of last temperature read
bool tempInRange;							// Boolean flag to determine if temp is within range or not (use good LED or bad LED)
float SessionHighTemp = 0.0;						// Generally, the high temp for a session will be more than 0F
float SessionLowTemp = 255.0;						// Generally, the low temp for a session will be higher than 255F
String default_temp_suffix = "F";
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                GLOBAL VARIABLES - i2c Integration
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/       
int commandReceived = 0;              // Commands are only ints
unsigned int i2cResponse = 0;               // Responses are only ints
bool commandCompleted = true;           // When set to true, system is ready to process the next inbound command

void goodOn()
{
  // Turn the good LED and status LED on, update goodMillis as this begins a new sequence of blinking the good LED
  digitalWrite(goodPIN, HIGH);
  //digitalWrite(statusPIN, HIGH);
  goodMillis = millis();
}

void goodOff()
{
  // Turn off the good LED and the status LED
  digitalWrite(goodPIN, LOW);
  //digitalWrite(statusPIN, LOW);
}

void allOn()
{
  // Turn on all LEDs
  digitalWrite(goodPIN, HIGH);
  digitalWrite(warningPIN, HIGH);
  //digitalWrite(statusPIN, HIGH);
}


void allOff()
{
  // Turn off all LEDs
  digitalWrite(goodPIN, LOW);
  digitalWrite(warningPIN, LOW);
  //digitalWrite(statusPIN, LOW);
}

bool storeTempToEEPROM(float temp, int addr)
{
  /*
     MAP FOR TEMP ADDRESSES:
     Since temp will always be below 255, first byte will hold full decimal of degrees F
     Since the decimal only goes to hundredths, (00-99) we will store that in the second byte
     And we need a positive negative flag.  If the third byte is 0 then it's a positive number
     if it's a 1, then it's a negative number 
  */
   
   // If the address isn't divisible by three, then it's not a valid address for this sketch.
   if (addr % 3 == 0)
   {
     int neg_temp = 0;
     int base_tempF;
     int dec_tempF;
  
     if (temp < 0)
     {
        neg_temp = 1;
     }
  
     base_tempF = temp;
     dec_tempF = ((temp * 100) - (base_tempF * 100));
     
     //Serial.println("Base TempF: " + String(base_tempF) + " :: Dec TempF: " + String(dec_tempF) + " :: Temp: " + String(temp));
     EEPROM.write(addr, base_tempF);
     EEPROM.write((addr+1), dec_tempF);
     EEPROM.write((addr+2), neg_temp);
     return true;
   }
   else
   {
	// Something went wrong - return false
    return false;
   }
}

float readTempFromEEPROM(int addr)
{
  // Beginning at the address, find the next three bytes.  Interpret them as needed.
  if (addr % 3 == 0)
  {
    int base_tempF = EEPROM.read(addr);
    int dec_tempF = EEPROM.read(addr+1);
    int neg_temp = EEPROM.read(addr+2);
    String str_neg = "";

    //Serial.println("[READ FROM EEPROM]: Base TempF: " + String(base_tempF) + " :: Dec TempF: " + String(dec_tempF) + " :: negative: " + String(neg_temp));
    if (neg_temp == 1)
    {
      str_neg = "-";
    }
    
    String str_temp = str_neg + String(base_tempF) + "." + String(dec_tempF);
    float f_temp = str_temp.toFloat();
    
    return f_temp;
  }
  else
  {
	// Something went wrong, return 255.0
    return 255.0;
  }  
}

bool factoryReset()
{ 
  // Turn on all LEDs, Write a serial warning, Wait 3 seconds, Turn off all LEDs
  allOn();
  Serial.println("[WARN] Defaulting high and low temp values.");
  storeTempToEEPROM(0.0, 0);
  storeTempToEEPROM(254, 3);
  SessionHighTemp = 0.0;
  SessionLowTemp = 255.0;
  delay(3000);
  allOff();
  ss.loadAnimation();
  return true;
}

void printBanner()
{
  // Print an informational chart for the program
  // This can be printed on demand by pressing the button
  // It also prints on microcontroller startup (e.g. when starting serial monitor and board resets)
  high_temp = readTempFromEEPROM(0);    // Retrieve the stored high temp from EEPROM      
  low_temp = readTempFromEEPROM(3);      // Retrieve the stored low temp from EEPROM
  Serial.println(" -------------------------------------------------");
  Serial.println("   Graham Engineering, LLC - TempMon Version 1.7");
  Serial.println(" -------------------------------------------------");
  Serial.println("     Build Date: 01 June 2018");
  Serial.println("     Author: matthew.j.graham@gmail.com");
  Serial.println("     All Rights Reserved 2017-2018");
  Serial.println(" -------------------------------------------------");
  Serial.println("     Current Temp: " + String(CurrentTemp));
  Serial.println("     High Temp Limit: " + String(high_limit) + "F");
  Serial.println("     Low Temp Limit: " + String(low_limit) + "F");
  Serial.println("     [EEPROM] High Temp: " + String(high_temp));
  Serial.println("     [EEPROM] Low Temp: " + String(low_temp));
  Serial.println(" -------------------------------------------------");
}

void setup(void)
{
  Serial.begin(9600); 						//Begin serial communication
  sensors.begin();							// Begin sensors (allows reading in of temp data)
  //pinMode(statusPIN, OUTPUT);				// Set the onboard LED pin for OUTPUT
  pinMode(goodPIN, OUTPUT);					// Set the green LED pin for OUTPUT
  pinMode(warningPIN, OUTPUT);				// Set the red LED pin for OUTPUT
  pinMode(infoPIN, INPUT_PULLUP);			// Set the button pin for INPUT with the internal PULLUP resistor
  pinMode(dispPin, INPUT_PULLUP);
  high_temp = readTempFromEEPROM(0);		// Retrieve the stored high temp from EEPROM			
  low_temp = readTempFromEEPROM(3);			// Retrieve the stored low temp from EEPROM
  Wire.begin(SLAVE_ADDRESS);        // Initialize i2c communications
  Wire.onReceive(receiveData);        // When data is received, all that data will be sent to the "receiveData" function
  Wire.onRequest(sendData);           // When data is requested to be sent, it will be sent via the "sendData" function
  CurrentTemp = sensors.getTempFByIndex(0);
  printBanner();
  ss.loadAnimation();
}

void loop()
{
  // Check to see if it's time to read the temperature
  if ((millis() - lastTempCheckMillis) > tempCheckInterval)
  {
	// It's been 5 seconds, conduct a temp read.
    sensors.requestTemperatures();  
    CurrentTemp = sensors.getTempFByIndex(0);
    lastTempCheckMillis = millis();
    
	if (CurrentTemp > SessionHighTemp)
	{
		// This is the highest temp we've seen this session.
		SessionHighTemp = CurrentTemp;
		if (SessionHighTemp > high_temp)
		{
			// This is the highest temp seen ever, record to eeprom
			storeTempToEEPROM(SessionHighTemp, 0);
		}
	}
	else if (CurrentTemp < SessionLowTemp)
	{
		// This is the lowest temp we've seen this session
		SessionLowTemp = CurrentTemp;
		if (SessionLowTemp < low_temp)
		{
			// This is the lowest temp seen ever, record to eeprom
			storeTempToEEPROM(SessionLowTemp, 3);
		}
	}
	
	if (CurrentTemp)
	{
		// We have a temp.  Probably some good data.
		if ((CurrentTemp < high_limit) && (CurrentTemp > low_limit))
		{
			// Temperature is within range
			Serial.println("[INFO] - " + String(millis()) + " - Current Temp: " + String(CurrentTemp) + " - Temp in range");
			tempInRange = true;
		}
		else
		{
			// Temperature is out of range
			Serial.println("[WARN] - " + String(millis()) + " - Current Temp: " + String(CurrentTemp) + " - Temp out of range");
			tempInRange = false;
		}
	}
  }
  
  // Make some LED's react to temp being in or out of range
  if (tempInRange)
  {
	  if ((millis() - goodMillis) > blinkDelay && digitalRead(goodPIN) != HIGH)
	  {
		  // If the bad indicator is on, turn it off
		  if (digitalRead(warningPIN) == HIGH)
		  {
			  digitalWrite(warningPIN, LOW);
		  }
		  // Turn the good indicator on
		  goodOn();
		  goodMillis = millis();
	  }
	  else if ((millis() - goodMillis) > (blinkDuration) && digitalRead(goodPIN) == HIGH)
	  {
		  // The good LED is on, and it has been 100 seconds into the blink
		  goodOff();
	  }
  }
  else
  {
	  // Temp is out of range
	  if (digitalRead(goodPIN) == HIGH)
	  {
		  // the good light is on, force it off.
		  goodOff();
	  }
	  if (digitalRead(warningPIN) == LOW)
	  {
		  // The warning light is off, turn it on
		  digitalWrite(warningPIN, HIGH);
	  }
  }
  
	// Check for a button press
	int btn_push = digitalRead(infoPIN);

	if (!btn_push)
	{
    delay(75);         // This is a short delay to debounce
		// The button has been pressed
		if (loopCount == 0)
		{
			// Loop count is 0, so this is the first loop through that the button has been pressed on	
			printBanner();			// On first press, show the chart of info
			loopCount++;			// Increment the loop counter
		}
		else
		{
			// This is not the first loop through, do not show the chart
			loopCount++;			// Increment the loop counter
			if (loopCount >= 10)
			{
				// We have more than 10 loops
				factoryReset();		// Clear EEPROM values
				printBanner();		// Print the chart showing cleared/reset values
				loopCount = 0;		// Reset the loop counter
			}
		}
	}
	else
	{
		// The button was not pressed (or was released)
		// Reset the loop counter if needed
		if (loopCount != 0)
		{
			loopCount = 0;
		}
	}

  // quick check of the dispPin to see if the button is pressed to show temp on display
  int disp_btn_push = digitalRead(dispPin);

  if (!disp_btn_push)
  {
    delay(50); // debounce timer
    ss.showString(String(CurrentTemp) + default_temp_suffix);
  }
}

void receiveData(int byteCount)
{
  commandCompleted=false;
  while (Wire.available()) 
  {
    commandReceived = Wire.read();
    /*
      Requests are 3-digit codes.
      70: Temp in F
      71: Temp in C
      72: Highest Temp recorded [eeprom]
      73: Lowest Temp recorded [eeprom]
      74: Highest Temp since power on
      75: Lowest Temp since power on
      76: high temp limit
      77: low temp limit
      79: Print banner info to serial
      Vars:
        int commandReceived = 0;              // Commands are only ints
        int i2cResponse = 0;                // Responses are only ints
        bool commandCompleted = true;           // When set to true, system is ready to process the next inbound command
      Function:
        format_temp_for_i2c(temp)             // converts a float to an int that can be parsed by the receiver
     */
  }
  switch(commandReceived)
  {
    case 0:
    {
      // Sometimes a zero is sent in as well.  Ignore it.
      break;
    }
    case 42:
    {
      i2cResponse = 42;
      Serial.println("You found the secret number");
      break;
    }
    case 70:
    {
      // User requested temp in F
      i2cResponse = format_temp_for_i2c(CurrentTemp);
      break;
    }
    case 71:
    {
      // User requested temp in C
      // C = (F * (9/5)) + 32
      i2cResponse = format_temp_for_i2c((CurrentTemp - 32) * 5/9);
      break;
    }
    case 72:
    {
      // User requested highest temp from eeprom
      i2cResponse = format_temp_for_i2c(high_temp);
      break;
    }
    case 73:
    {
      // User requested lowest temp from eeprom
      i2cResponse = format_temp_for_i2c(low_temp);
      break;
    }
    case 74:
    {
      // User requested highest temp since power on
      i2cResponse = format_temp_for_i2c(SessionHighTemp);
      break;
    }
    case 75:
    {
      // User requested lowest temp since power on
      i2cResponse = format_temp_for_i2c(SessionLowTemp);
      break;
    }
    case 76:
    {
    // User requested high temp limit
    i2cResponse = format_temp_for_i2c(high_limit);
    break;
    }
    case 77:
    {
    // User requested low temp limit
    i2cResponse = format_temp_for_i2c(low_limit);
    break;
    }
    case 79:
    {
      // Print banner info to serial port
      printBanner();
      i2cResponse = 200;
      break;
    }
    case 89:
    {
    // Factory default EEPROM values
    if (factoryReset())
    {
      i2cResponse=200;
    }
    else
    {
      Serial.println("[ERROR] - An error has occurred while attempting to set values to factory defaults via i2c");
      i2cResponse = 500;
    }
    printBanner();  

    break;
    }
   default:
   {
    Serial.println("Invalid request received: " + String(commandReceived));
    i2cResponse = 400;
   }
  }
}
 

void sendData()
{
  // If we have data, send it.
  if (i2cResponse >= 0 && commandCompleted == false)
  {
    Wire.write((byte *)&i2cResponse, sizeof(i2cResponse));
    commandCompleted = true;
  }
  else
  {
    Wire.write(0);
    commandCompleted = false;
  }
}

int format_temp_for_i2c(float temp)
{
  unsigned int o_temp;          // Use an unsigned int.  This way if a value is negative, it's subtracted from 65536. (e.g. -28.0 F is 62736 which is 65536-2800)
  o_temp = (temp * 100);        // Response must be int, so we have to multiply by 100 to make a nice int from the float
  //Serial.println(o_temp);
  String str_temp = String(o_temp); // I needed this line because if I left it out and returned a negative number, it would just return zero instead of the actual value.  By converting the unsigned int to a string, it saves the unsigned int how I wanted it.
  return o_temp;
}

