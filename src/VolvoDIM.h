/*
  VolvoDIM.h - Library for powering and controlling a P2 Volvo DIM.
  Created by Andrew J. Gabler, August 17, 2021.

  Reworked by Mikael R, March 29, 2026.
*/

#ifndef VolvoDIM_h
#define VolvoDIM_h

#include "mcp2515_can.h"
#include <mcp_can.h>
#include <SPI.h>
#include <math.h>
#include <time.h>

class VolvoDIM
{
    public:
        VolvoDIM(int SPI_CS_PIN, int relayPin=0, int parkingBrakePin=0, int engineLightPin=0);
        void setTime(int inputTime);
        int clockToDecimal(int hour, int minute); 
        double celsToFahr(double temp);
        void setOutdoorTemp(int oTempCelsius);
        void setCoolantTemp(int range);
        void setSpeed(int carSpeed);
        void setGasLevel(int level);
        void setRpm(int rpm);
        void setTotalBrightness(int value);
        void setGearPosText(const char* gear);
        void setGearPosInt(int gear);
        void setCustomText(const char* text);
        void init();
        void simulate();
        void powerOff();
        void powerOn();
        void gaugeReset();
        void sweepGauges();
        void enableSerialErrorMessages();
        void disableSerialErrorMessages();
        void enableMilageTracking(int on);
        void enableDisableDingNoise(int on);
        void enableFog(int enabled);
        void enableBrake(int enabled);
        void enableHighBeam(int enabled);
        void setBlinker(int right, int left, int hazard);
        void enableABS(int enabled);
        void enableTC(int enabled);
        void enableOil(int enabled);
        void enableCruise(int enabled);
        void enableBattery(int enabled);
        void enableFlashingTC(int enabled);
        void enableFlashingABS(int enabled);
        void enableParkingBrake(int enabled);
        void enableEngineLight(int enabled);
    private:
        void sendMsgWrapper(unsigned long wId, unsigned char* wBuf);
        void initSRS();
        void genSRS(long address, byte stmp[]);
        void init4C();
        void genCC(long address, byte stmp[]);
        void genTemp(long address, byte stmp[]);
        void genCustomText();
        void clearCustomText();
        void genMileageAndSpeed();

        // uptightsuperlabs - 4/10/2026 i'll work on fixing all of this up and you know, crap like that sooner or later...
        //                              easily TONS of work to do. i like it. also this is 32 chars + 1 null term
        char persistent_custom_text[33]; // also yes i had nothing better to name this, bite me.
        int text_scroll_index = 0;
};

#endif
