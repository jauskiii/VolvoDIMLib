/*
  VolvoDIM.cpp - Library for powering and controlling a P2 Volvo DIM.
  Created by Andrew J. Gabler, August 17, 2021.

Reworked by Mikael R, March 29, 2026.
*/
#include "VolvoDIM.h"
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#endif

mcp2515_can CAN(0);
int _relayPin = 0;
int _parkingBrakePin = 0;
int _engineLightPin = 0;
VolvoDIM::VolvoDIM(int SPI_CS_PIN, int relayPin, int parkingBrakePin, int engineLightPin)
{
	mcp2515_can temp_CAN(SPI_CS_PIN);
	CAN = temp_CAN;
	_relayPin = relayPin;


	  // Initialize parking brake relay control pin (digital pin 7)
   _parkingBrakePin = parkingBrakePin;
  pinMode(_parkingBrakePin, OUTPUT);
  // Set default state: relay off (assuming active low for parking brake)
  digitalWrite(_parkingBrakePin, HIGH);



  _engineLightPin = engineLightPin;
  pinMode(_engineLightPin, OUTPUT);
  // Set default state: relay off (assuming active low for parking brake)
  digitalWrite(_engineLightPin, HIGH);
}
bool enableSerialErrMsg = false;
int genCnt = 0;
int cnt = 0;
constexpr int listLen = 13;
int startUpWait = 0;
int customMessageCnt = 0, customTextChanged = 0;
int mileageCounter = 0, mileagePace = 0, genSpeed = 0;
int mileageEnabled = 0;
#define CALIBRATION_FACTOR 830.0
unsigned long lastUpdateTimeGlobal = 0;
float mileageAccumulatorGlobal = 0.0;
int carConCnt = 0;
int configCnt = 0;
int blinkerInterval = 0;
unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long address;
constexpr int arrSpeed = 0, arrRpm = 1, arrCoolant = 2, arrTime = 3, arrBrakes = 4, arrBlinker = 5, arrAntiSkid = 6, arrAirbag = 7, arr4c = 8, arrConfig = 9, arrGear = 10, arrDmWindow = 11, arrDmMessage = 12;
constexpr unsigned long addrLi[listLen] = {0x217FFC, 0x2803008, 0x3C01428, 0x381526C, 0x3600008, 0xA10408, 0x2006428, 0x1A0600A, 0x2616CFC, 0x1017FFC, 0x3200408, 0x02A0240E, 0x1800008};
/*
   addrLi[0] = Speed/KeepAlive
   addrLi[1] = RPM/Backlights
   addrLi[2] = Coolant/OutdoorTemp
   addrLi[3] = Time/GasTank
   addrLi[4] = Brakes Keep alive?
   addrLi[5] = Blinker
   addrLi[6] = Anti-Skid
   addrLi[7] = Aibag Light
   addrLi[8] = 4C Error
   addrLi[9] = Car Config
   addrLi[10] = Gear Pos
   addrLi[11] = Trailer Attached
   addrLi[12] = Dim Message Window
   addrLi[13] = Dim Message Content
*/

unsigned char defaultData[listLen][8] = {
	{0x01, 0xEB, 0x40, 0xD8, 0xF0, 0x58, 0x00, 0x00}, // 0, Speed/cruise/KeepAlive , 0x217FFC
	{0xFF, 0xE1, 0xFF, 0xF0, 0xFF, 0xCF, 0x00, 0x00}, // 1, RPM/Backlights , 0x2803008
	{0xC0, 0x80, 0x51, 0x89, 0x0E, 0x00, 0x00, 0x00}, // 2, Coolant/OutdoorTemp , 0x3C01428
	{0x00, 0x01, 0x05, 0xBC, 0x05, 0xA0, 0x40, 0x40}, // 3, Time/GasTank , 0x381526C
	{0x00, 0x00, 0xB0, 0x60, 0x30, 0x00, 0x00, 0x00}, // 4, Brake system Keep alive , 0x3600008
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // 5, Blinker , 0xA10408
	{0x01, 0xE3, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00}, // 6, Anti-Skid , 0x2006428
	{0x00, 0x00, 0x00, 0x00, 0x00, 0xBE, 0x49, 0x00}, // 7, Aibag Light , 0x1A0600A
	{0x0B, 0x42, 0x00, 0x00, 0xFD, 0x1F, 0x00, 0xFF}, // 8, 4C keep alive / prevent error , 0x2616CFC
	{0x01, 0x0F, 0xF7, 0xFA, 0x00, 0x00, 0x00, 0xC0}, // 9, Car Config default , 0x1017FFC
	{0x11, 0xDE, 0x53, 0x00, 0x24, 0x00, 0x10, 0x00}, // 10 Gear Postion , 0x3200408
	{0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35}, // 12 Dim Message Window , 0x02A0240E
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}  // 13 Dim Message Content , 0x1800008
};

// Sets the data for what the car is equiped with.
//original code and some parts taken from 2004 d5 diesel volvo with automatic trasnmission
//originla code is from manual car so then the gear text wont work

unsigned char carConfigData[16][8] = {
	{0x10, 0x30, 0x01, 0x02, 0x01, 0x03, 0x00, 0x02}, // modified default config data to from mnaual to automatic gearbox gear screen works now

	//original code
	{0x11, 0x18, 0x05, 0x05, 0x02, 0x03, 0x01, 0x05},
	{0x12, 0x03, 0x02, 0x02, 0x01, 0x02, 0x05, 0x01},
	{0x13, 0x01, 0x01, 0x01, 0x02, 0x02, 0x01, 0x04},
	{0x14, 0x04, 0x01, 0x02, 0x01, 0x02, 0x03, 0x11},
	{0x15, 0x15, 0x03, 0x02, 0x02, 0x01, 0x01, 0x01},
	{0x16, 0x02, 0x62, 0x02, 0x01, 0x02, 0x01, 0x02},
	{0x17, 0x01, 0x02, 0x02, 0x01, 0x03, 0x03, 0x01},
	{0x18, 0x01, 0x05, 0x03, 0x03, 0x12, 0x04, 0x04},
	{0x19, 0x11, 0x02, 0x10, 0x01, 0x05, 0x01, 0x02},
	{0x1A, 0x01, 0x02, 0x01, 0x04, 0x01, 0x01, 0x03},
	{0x1B, 0x01, 0x02, 0x10, 0x01, 0x01, 0x01, 0x01},
	{0x1C, 0x01, 0x04, 0x01, 0x03, 0x01, 0x06, 0x01},
	{0x1D, 0x04, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01},
	{0x1E, 0x01, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01},
	{0x1F, 0x01, 0x02, 0x05, 0x02, 0x02, 0x22, 0x01}};

void VolvoDIM::sendMsgWrapper(unsigned long wId, unsigned char *wBuf)
{
	CAN.sendMsgBuf(wId, 1, 8, wBuf);
}









void VolvoDIM::initSRS()
{
	unsigned char temp[8] = {0xC0, 0x0, 0x0, 0x0, 0x0, 0xBC, 0xDB, 0x80};
	unsigned long tempAddr = addrLi[arrAirbag];
	sendMsgWrapper(tempAddr, temp);
	delay(15);
	temp[0] = (char)0x0;
	sendMsgWrapper(tempAddr, temp);
	delay(15);
	temp[0] = (char)0xC0;
	temp[6] = (char)0xC9;
	sendMsgWrapper(tempAddr, temp);
	delay(15);
	temp[0] = (char)0x80;
	sendMsgWrapper(tempAddr, temp);
}

void VolvoDIM::genSRS(long address, byte stmp[])
{
	int randomNum = random(0, 794);
	if (randomNum >= 0 && randomNum < 180)
	{
		stmp[0] = (char)0x40;
	}
	else if (randomNum >= 180 && randomNum < 370)
	{
		stmp[0] = (char)0xC0;
	}
	else if (randomNum >= 370 && randomNum < 575)
	{
		stmp[0] = (char)0x0;
	}
	else if (randomNum >= 575 && randomNum < 794)
	{
		stmp[0] = (char)0x80;
	}
	sendMsgWrapper(address, stmp);
	delay(15);
}

void VolvoDIM::init4C()
{
	unsigned char temp[8] = {0x09, 042, 0x0, 0x0, 0x0, 0x50, 0x00, 0x00};
	unsigned long tempAddr = addrLi[arr4c];
	sendMsgWrapper(tempAddr, temp);
	delay(15);
	sendMsgWrapper(tempAddr, temp);
	delay(15);
	temp[0] = (char)0x0B;
	sendMsgWrapper(tempAddr, temp);
	delay(15);
	temp[0] = (char)0x0B;
	sendMsgWrapper(tempAddr, temp);
}

void VolvoDIM::genCC(long address, byte stmp[])
{
	int randomNum = random(0, 7);
	if (randomNum > 3)
	{
		stmp[6] = (char)0xFF;
		stmp[7] = (char)0xF3;
	}
	sendMsgWrapper(address, stmp);
	delay(15);
}

void VolvoDIM::genTemp(long address, byte stmp[])
{
	int randomNum = random(0, 133);
	if (randomNum < 16)
	{
		stmp[0] = (char)0x00;
	}
	else if (randomNum >= 16 && randomNum < 41)
	{
		stmp[0] = (char)0x40;
	}
	else if (randomNum >= 41 && randomNum < 76)
	{
		stmp[0] = (char)0xC0;
	}
	else if (randomNum >= 76 && randomNum < 133)
	{
		stmp[0] = (char)0x80;
	}
	if (randomNum < 76)
	{
		stmp[1] = (char)0x80;
	}
	else
	{
		stmp[1] = (char)0x00;
	}
	randomNum = random(0, 105);
	if (randomNum < 7)
	{
		stmp[2] = (char)0x11;
	}
	else if (randomNum >= 7 && randomNum < 15)
	{
		stmp[2] = (char)0x71;
	}
	else if (randomNum >= 15 && randomNum < 25)
	{
		stmp[2] = (char)0x61;
	}
	else if (randomNum >= 25 && randomNum < 60)
	{
		stmp[2] = (char)0x51;
	}
	else if (randomNum >= 60 && randomNum < 105)
	{
		stmp[2] = (char)0x41;
	}
	sendMsgWrapper(address, stmp);
	delay(15);
}

void VolvoDIM::genMileageAndSpeed() {
    unsigned long currentTime = millis();
    if (lastUpdateTimeGlobal == 0) {
        lastUpdateTimeGlobal = currentTime;
    }
    unsigned long deltaMillis = currentTime - lastUpdateTimeGlobal;
    lastUpdateTimeGlobal = currentTime;
    
    if (mileageEnabled == 1 && startUpWait == 0) {
        float deltaHours = deltaMillis / 3600000.0;
        float distanceIncrement = genSpeed * deltaHours;
        
        // Use the calibration factor to match the odometer
        mileageAccumulatorGlobal += distanceIncrement * CALIBRATION_FACTOR;
        
        int rawUnitsToAdd = (int)mileageAccumulatorGlobal;
        if (rawUnitsToAdd > 0) {
            mileageCounter += rawUnitsToAdd;
            mileageAccumulatorGlobal -= rawUnitsToAdd;
        }
        if (mileageCounter > 255) {
            mileageCounter = 0;
        }
        defaultData[arrSpeed][7] = mileageCounter;
    }
    sendMsgWrapper(addrLi[arrSpeed], defaultData[arrSpeed]);
}

void VolvoDIM::powerOn()
{
	pinMode(_relayPin, OUTPUT);
	digitalWrite(_relayPin, HIGH);
}

void VolvoDIM::powerOff()
{
	pinMode(_relayPin, OUTPUT);
	digitalWrite(_relayPin, LOW);
}

void VolvoDIM::init()
{
	while (CAN_OK != CAN.begin(CAN_125KBPS, MCP_16MHz))
	{
		delay(100);
	}
	if (_relayPin > 0)
	{
		powerOn();
	}
	initSRS();
	init4C();

}

void VolvoDIM::simulate()
{
	address = addrLi[cnt];

	// Copy default data
	memcpy(stmp, defaultData[cnt], sizeof(stmp));

	// Process based on address
	switch (address)
	{
	case addrLi[arrAirbag]:
		genSRS(address, stmp);
		break;
	case addrLi[arrCoolant]:
		genTemp(address, stmp);
		break;
	case addrLi[arrConfig]:
		carConCnt++;
		if (carConCnt >= 50) 
		{
			// Burst send all 16 lines of configuration sequentially
			for (int i = 0; i < 16; i++) {
				sendMsgWrapper(address, carConfigData[i]);
				delay(2); // Give the cluster 2ms to process each line
			}
			carConCnt = 0;
		}
		else
		{
			// Send the normal 0x01 heartbeat
			genCC(address, stmp);
		}
		break;
	case addrLi[arrDmWindow]:
		// doesn't need to be sent every cycle, send every 4
		if (cnt % 4 == 0)
		{
			sendMsgWrapper(address, stmp);
		}
	case addrLi[arrDmMessage]:
		if(customMessageCnt < 5){
			genCustomText();
		}

	case addrLi[arrSpeed]:
		genMileageAndSpeed();

	default:
		sendMsgWrapper(address, stmp);
		sendMsgWrapper(addrLi[arrBlinker], defaultData[arrBlinker]);
		// delay(15); // send data per 15ms
	}


	cnt++;
	blinkerInterval++;
	if(startUpWait > 0){
		startUpWait--;
	}

	// Reset counters if necessary
	if (cnt == listLen)
	{
		cnt = 0;
	}

	if (blinkerInterval >= 50)
	{
		blinkerInterval = 0;
	}
}

void VolvoDIM::setTime(int inputTime)
{
    if (inputTime >= 0 && inputTime <= 1440)
    {
        int b4 = 0;
        for (int i = 0; i < 6; i++)
        {
            if (inputTime >= (i * 256) && inputTime < ((i + 1) * 256))
            {
                b4 = i;
            }
        }
        int b5 = inputTime - (b4 * 256);
        
        // Save the current state of the Battery Light!
        unsigned char batteryBit = defaultData[arrTime][4] & 0x10; 
        
        // Merge the clock math with the battery light
        defaultData[arrTime][4] = b4 | batteryBit;
        defaultData[arrTime][5] = b5;
    }
    else
    {
        if (enableSerialErrMsg)
        {
            // SERIAL.println("Not a valid time");
        }
    }
}
int VolvoDIM::clockToDecimal(int hour, int minute)
{
    if (hour >= 0 && hour < 24 && minute >= 0 && minute < 60)
    {
        return (hour * 60) + minute;
    }
    else
    {
        if (enableSerialErrMsg)
        {
            // SERIAL.println("invalid time");
        }
        return 0;
    }
}
double VolvoDIM::celsToFahr(double temp)
{
  return ((temp * (9.0 / 5.0)) + 32.0);
}

void VolvoDIM::setOutdoorTemp(int oTemp)
{
	if (!(oTemp < -49 || oTemp > 176))
	{
		if (oTemp > -50 && oTemp <= 32)
		{
			defaultData[arrCoolant][4] = (char)0x0D;
			defaultData[arrCoolant][5] = ceil((oTemp + 83) * 2.21);
		}
		else if (oTemp > 32 && oTemp <= 146)
		{
			defaultData[arrCoolant][4] = (char)0x0E;
			defaultData[arrCoolant][5] = ceil((oTemp - 33) * 2.25);
		}
		else if (oTemp > 146 && oTemp < 177)
		{
			defaultData[arrCoolant][4] = (char)0x0F;
			defaultData[arrCoolant][5] = ceil((oTemp - 147) * 2.20);
		}
	}
	else
	{
		if (enableSerialErrMsg)
		{
			// SERIAL.println("Temp out of range");
		}
	}
}
void VolvoDIM::setCoolantTemp(int range)
{
	if (range >= 0 && range <= 55)
	{
		defaultData[arrCoolant][3] = range + 88;
	}
	else if (range > 55 && range <= 100)
	{
		// upper limit scales very slowly.
		// This calculation makes it fit the 0-100 scale much better.
		defaultData[arrCoolant][3] = ceil((range - 55) * .333) + 173;
	}
	else
	{
		if (enableSerialErrMsg)
		{
			// SERIAL.println("Value out of range");
		}
	}
}

void VolvoDIM::setGasLevel(int level)
{
	if (level >= 0 && level <= 100)
	{
		defaultData[arrTime][6] = round(level * .64);
		defaultData[arrTime][7] = round(level * .64);
	}
	else
	{
		if (enableSerialErrMsg)
		{
			// SERIAL.println("Gas level out of range");
		}
	}
}

void VolvoDIM::setRpm(int rpm) {
  // Clamp rpm between 0 and 8000
  if (rpm < 0)
    rpm = 0;
  if (rpm > 8000)
    rpm = 8000;
  
  // Adjusted full-scale constant: 8000 rpm should map to ~31.62 (instead of 24)
  const float scale = 31.62 / 8000.0;
  float fixed_point = rpm * scale;  // This gives a value from 0.0 up to ~31.62
  
  // Convert to a Q8.8 fixed–point value
  uint16_t encoded = (uint16_t)round(fixed_point * 256.0);
  
  // Byte 6 holds the high byte (the "major" part)
  defaultData[arrRpm][6] = encoded >> 8;
  // Byte 7 holds the low byte (the "minor" adjustments)
  defaultData[arrRpm][7] = encoded & 0xFF;
}

void VolvoDIM::setSpeed(int carSpeed)
{
    genSpeed = carSpeed;
    
    // Check if Cruise Control is currently ON (Is bit 7 active?)
    unsigned char isCruiseOn = defaultData[arrSpeed][5] & 0x80; 
    
    // Set the base byte depending on the Cruise state
    unsigned char baseByte;
    if (isCruiseOn) {
        baseByte = 0x90; // Cruise is ON
    } else {
        baseByte = 0x50; // Cruise is OFF
    }
    
    if (carSpeed >= 0 && carSpeed <= 160)
    {
        if (carSpeed >= 0 && carSpeed <= 40)
        {
            defaultData[arrSpeed][5] = baseByte | 0x08; // Becomes 0x58 or 0x98
            defaultData[arrSpeed][6] = round(carSpeed * 6.375);
        }
        else if (carSpeed > 40 && carSpeed <= 80)
        {
            defaultData[arrSpeed][5] = baseByte | 0x09; // Becomes 0x59 or 0x99
            defaultData[arrSpeed][6] = round(carSpeed * 6.375);
        }
        else if (carSpeed > 80 && carSpeed <= 120)
        {
            defaultData[arrSpeed][5] = baseByte | 0x0A; // Becomes 0x5A or 0x9A
            defaultData[arrSpeed][6] = round(carSpeed * 6.375);
        }
        else if (carSpeed > 120 && carSpeed <= 160)
        {
            defaultData[arrSpeed][5] = baseByte | 0x0B; // Becomes 0x5B or 0x9B
            defaultData[arrSpeed][6] = round(carSpeed * 6.375);
        }
    }
}

void VolvoDIM::enableCruise(int enabled)
{
    if (enabled == 1) {
        // TURN CRUISE ON
        defaultData[arrSpeed][0] |= 0x06; // Engine Armed (0x07)
        
        // Switch upper nibble of Byte 5 from '5' to '9'
        defaultData[arrSpeed][5] &= ~0x40; // Turn OFF bit 6
        defaultData[arrSpeed][5] |= 0x80;  // Turn ON bit 7
        
    } else {
        // TURN CRUISE OFF
        defaultData[arrSpeed][0] &= ~0x06; // Engine Disarmed (0x01)
        defaultData[arrSpeed][5] |= 0x40;  // Turn ON bit 6
        defaultData[arrSpeed][5] &= ~0x80; // Turn OFF bit 7
    }
}

void VolvoDIM::setTotalBrightness(int value)
{
    if (value < 0)
        value = 0;
    else if (value > 255)
        value = 255;
    defaultData[arrRpm][2] = value;
    int byte3 = 0x30 + static_cast<int>((value * 15.0f / 255.0f) + 0.5f);
    defaultData[arrRpm][3] = byte3;
    int byte4 = 0x30 + static_cast<int>((value * 13.0f / 255.0f) + 0.5f);
    defaultData[arrRpm][4] = byte4;
}

void VolvoDIM::enableHighBeam(int enabled) {
  if (enabled == 1)
    defaultData[arrRpm][1] = 0xFF;
  else
    defaultData[arrRpm][1] = 0xEA;
}

void VolvoDIM::enableBattery(int enabled) 
{
  if (enabled == 1) {
    // Turn Battery Light ON: Force Bit 4 (0x10) to 1
    defaultData[arrTime][4] |= 0x10; 
  } else {
    // Turn Battery Light OFF: Return Bit 4 (0x10) to 0
    defaultData[arrTime][4] &= ~0x10; 
  }
}

void VolvoDIM::enableFlashingTC(int enabled) 
{
  if (enabled == 1) {
    // Turn Flashing TC ON: Drop Bit 7 (0x80) to 0
    defaultData[arrSpeed][3] &= ~0x80; 
  } else {
    // Turn Flashing TC OFF: Return Bit 7 (0x80) to 1
    defaultData[arrSpeed][3] |= 0x80; 
  }	
}

void VolvoDIM::enableFlashingABS(int enabled) 
{
  if (enabled == 1) {
    // Turn Flashing ABS ON: Drop Bit 5 (0x20) to 0
    defaultData[arrBrakes][4] &= ~0x20; 
  } else {
    // Turn Flashing ABS OFF: Return Bit 5 (0x20) to 1
    defaultData[arrBrakes][4] |= 0x20; 
  }
}

void VolvoDIM::enableABS(int enabled) 
{
  if (enabled == 1) {
    defaultData[arrBrakes][2] &= ~0x10; 
  } else {
    defaultData[arrBrakes][2] |= 0x10; 
  }
}

void VolvoDIM::enableTC(int enabled) 
{
  if (enabled == 1)
    defaultData[arrAntiSkid][1] &= ~0x20; 
  else
    defaultData[arrAntiSkid][1] |= 0x20; 
}


void VolvoDIM::enableOil(int enabled) 
{
  if (enabled == 1) {
    defaultData[arrGear][1] |= 0x20; 
  } else {
    defaultData[arrGear][1] &= ~0x20; 
  }
}

void VolvoDIM::enableFog(int enabled)
{
  if (enabled == 1)
    defaultData[arr4c][2] = 0xE6;
  else
    defaultData[arr4c][2] = 0x00;
}

void VolvoDIM::enableBrake(int enabled)
{
  if (enabled == 1)
    defaultData[arrBrakes][3] = 0x00;
  else
    defaultData[arrBrakes][3] = 0x60;
}

void VolvoDIM::enableParkingBrake(int enabled) {
  if (enabled == 1)
    digitalWrite(_parkingBrakePin, HIGH);
  else
    digitalWrite(_parkingBrakePin, LOW);
}

void VolvoDIM::enableEngineLight(int enabled) {
  if (enabled == 1)
    digitalWrite(_engineLightPin, HIGH);
  else
    digitalWrite(_engineLightPin, LOW);
}

void VolvoDIM::setGearPosText(const char *gear)
{
	if (gear == 'low' || gear == 'Low' || gear == 'l' || gear == 'L')
	{
		defaultData[arrGear][4] = (char)0x40;
		defaultData[arrGear][6] = (char)0x96;
		// Gear L (low?)
	}
	else if (gear == 'park' || gear == 'Park' || gear == 'p' || gear == 'P')
	{
		defaultData[arrGear][4] = (char)0x24;
		defaultData[arrGear][6] = (char)0x10;
		// Park
	}
	else if (gear == 'reverse' || gear == 'Reverse' || gear == 'r' || gear == 'R')
	{
		defaultData[arrGear][4] = (char)0xE4;
		defaultData[arrGear][6] = (char)0x20;
		// Reverse
	}
	else if (gear == 'Neutral' || gear == 'neutral' || gear == 'N' || gear == 'n')
	{
		defaultData[arrGear][4] = (char)0x24;
		defaultData[arrGear][6] = (char)0x30;
		// Neutral
	}
	else if (gear == '1' || gear == 'one' || gear == 'One')
	{
		defaultData[arrGear][4] = (char)0x34;
		defaultData[arrGear][6] = (char)0x40;
		// 1st Gear
	}
	else if (gear == '2' || gear == 'two' || gear == 'Two')
	{
		defaultData[arrGear][4] = (char)0x40;
		defaultData[arrGear][6] = (char)0x70;
		// 2nd Gear
	}
	else if (gear == '3' || gear == 'three' || gear == 'Three')
	{
		defaultData[arrGear][4] = (char)0x40;
		defaultData[arrGear][6] = (char)0x60;
		// 3rd Gear
	}
	else if (gear == '4' || gear == 'four' || gear == 'Four')
	{
		defaultData[arrGear][4] = (char)0x44;
		defaultData[arrGear][6] = (char)0x50;
		// 4th Gear
	}
	else if (gear == '5' || gear == 'five' || gear == 'Five')
	{
		defaultData[arrGear][4] = (char)0xBE;
		defaultData[arrGear][6] = (char)0x00;
		// 5th Gear
	}
	else if (gear == '6' || gear == 'six' || gear == 'Six')
	{
		defaultData[arrGear][4] = (char)0xD5;
		defaultData[arrGear][6] = (char)0x00;
		// 6th Gear
	}
	else
	{
		defaultData[arrGear][4] = (char)0x10;
		defaultData[arrGear][6] = (char)0x40;
		// Nothing Displayed
	}
}
void VolvoDIM::setGearPosInt(int gear)
{
	// Don't know what to do with the automatic gears for an integer assignment so we'll use negatives
	switch (gear)
	{
	case -4:
		defaultData[arrGear][4] = (char)0x40;
		defaultData[arrGear][6] = (char)0x96;
		break; // Gear L (low?)
	case -3:
		defaultData[arrGear][4] = (char)0x24;
		defaultData[arrGear][6] = (char)0x10;
		break; // Park
	case -2:
		defaultData[arrGear][4] = (char)0xE4;
		defaultData[arrGear][6] = (char)0x20;
		break; // reverse
	case -1:
		defaultData[arrGear][4] = (char)0x10;
		defaultData[arrGear][6] = (char)0x40;
		break; // Nothing Displayed
	case 0:
		defaultData[arrGear][4] = (char)0x24;
		defaultData[arrGear][6] = (char)0x30;
		break; // Neutral
	case 1:
		defaultData[arrGear][4] = (char)0x34;
		defaultData[arrGear][6] = (char)0x40;
		break; // 1st Gear
	case 2:
		defaultData[arrGear][4] = (char)0x40;
		defaultData[arrGear][6] = (char)0x70;
		break; // 2nd Gear
	case 3:
		defaultData[arrGear][4] = (char)0x40;
		defaultData[arrGear][6] = (char)0x60;
		break; // 3rd Gear
	case 4:
		defaultData[arrGear][4] = (char)0x44;
		defaultData[arrGear][6] = (char)0x50;
		break; // 4th Gear
	case 5:
		defaultData[arrGear][4] = (char)0xBE;
		defaultData[arrGear][6] = (char)0x00;
		break; // 5th Gear
	case 6:
		defaultData[arrGear][4] = (char)0xD5;
		defaultData[arrGear][6] = (char)0x00;
		break; // 6th Gear
	}
}

// uptightsuperlabs - 4/10/2026 safety: copy string into our own persistent buffer (custom_text), this solves the dangling poiner issue entirely
void VolvoDIM::setCustomText(const char *text) {
    if (text == nullptr) {
        return;
    }

    strncpy(persistent_custom_text, text, 32);
    persistent_custom_text[32] = '\0';

    customMessageCnt = 0;
    customTextChanged = 1;
}

void VolvoDIM::genCustomText() {
	// uptightsuperlabs - 4/24/2026 idk wtf i was doing i forgot, so im gonna implement this a different way.
	unsigned char header[8];
	memcpy(stmp, defaultData[arrDmWindow], sizeof(stmp));
	stmp[7] = 0x31;
	sendMsgWrapper(addrLi[arrDmWindow], header);

	for (int i = 0; i < 4; i++) {
		unsigned char chunk[8];

		// uptightsuperlabs - 4/24/2026 as far as i can research 0x60 semes to be the sequence index for volvo dims?
		chunk[0] = 0x60 + i;

		// uptightsuperlabs - 4/24/2026 we can do some pretty gay pointer arithmetic here, it's faster than array indexing
		char *source = &persistent_custom_text[i * 7];

		for (int j = 1; j <= 7; j++) {
			if ((i * 7 + (j - 1)) < 32) {
				chunk[j] = source[j - 1];
			} else {
				chunk[j] = 0x20; // uptightsuperlabs - 4/24/2026 filling the space with 0x20 if the string is short
			}
		}

		send sendMsgWrapper(addrLi[arrDmMessage], chunk);
		delayMicrosecods(500); // uptightsuperlabs - 4/24/2026 https://docs.arduino.cc/language-reference/en/functions/time/delayMicroseconds/
	}

	customMessageCnt++;
}

void VolvoDIM::enableMilageTracking(int on){
	mileageEnabled = on;
}


void VolvoDIM::enableDisableDingNoise(int on){
	memcpy(stmp, defaultData[arrTime], sizeof(stmp));
	if(on == 0){
		stmp[1] = 0x30;
		sendMsgWrapper(addrLi[arrTime], stmp);
	} else if (on == 1){
		stmp[1] = 0x18;
		sendMsgWrapper(addrLi[arrTime], stmp);
	}
}

void VolvoDIM::clearCustomText()
{
	unsigned char clearValues[] = {0xE1, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	sendMsgWrapper(addrLi[arrDmWindow], clearValues);
}

void VolvoDIM::gaugeReset()
{
	powerOn();
	delay(7000);
	powerOff();
}
void VolvoDIM::sweepGauges()
{
    setRpm(8000);
    setSpeed(160); 
    delay(500);
    setRpm(0);
    setSpeed(0);
}

void VolvoDIM::setBlinker(int right, int left, int hazard) {
  // Priority: hazard > right > left
  if (hazard == 1 || (right == 1 && left == 1))
    defaultData[arrBlinker][7] = 0x0E;
  else if (right == 1)
    defaultData[arrBlinker][7] = 0x0C;
  else if (left == 1)
    defaultData[arrBlinker][7] = 0x0A;
  else
    defaultData[arrBlinker][7] = 0x08;
}
void VolvoDIM::enableSerialErrorMessages()
{
	enableSerialErrMsg = true;
}
void VolvoDIM::disableSerialErrorMessages()
{
	enableSerialErrMsg = false;
}

