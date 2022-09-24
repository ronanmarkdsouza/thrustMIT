/*=============================================================================
 |   File: enigma.h                                                         
 |   Version: 1.0
 |   Authors:  Ronan Mark D'souza
 |   Language:  C++
 |   To Compile:  Use Arduino IDE or PlatformIO
 +-----------------------------------------------------------------------------
 |
 |  Description:  This is the main .cpp for the Enigma.h package intended to be
 |                used on the Flight Computer for SAC-2023
 |
 |        Hardware:  Microcontroller Unit: Teensy 4.1
 |                   Pressure Sensor: BMP388
 |                   IMU: MPU6050
 |                   RF Module: XBee ZigBee Pro S2C 2.4 Ghz
 |                   Differential Pressure Sensor: MPX5100DP
 |                   GPS: Adafruit Ultimate GPS
 |                          
 |       Software:   The following code has classes for getting data from the
 |                   different sensors on-board the Flight Computer.
 |                   It also includes the following features and modules:
 |                            - Filters
 |                            - CRC Check
 |                            - Ultra Low Power mode
 |                            - RF Link
 |                            - GPS Link
 |                            - Apogee Detection
 |                            - Video Feed            
 | 
 *===========================================================================*/
#ifndef ENIGMA_H
#define ENIGMA_H
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BUZZ 8
#define SEALEVELPRESSURE_HPA (1013.25)

class beeps{
    private:
        void single_beep(){
            digitalWrite(LED_BUILTIN, HIGH);
            delay(250);
            digitalWrite(LED_BUILTIN, LOW);
            delay(250);
        }
    public:
        beeps(){
        pinMode(LED_BUILTIN, OUTPUT);
    }
        void BMP_ERR(){
            for(int i=0;i<3;i++)
                single_beep();
        }
        void IMU_ERR(){
            for(int i=0;i<5;i++)
                single_beep();
        }
} beep;

class Airbrakes{
    public:
        void open();
        void close();
        void flex();
};

class BMP{
    Adafruit_BMP3XX bmp;
    public:
        bool BMP_ACT;
        BMP(int addr){
            if(bmp.begin_I2C(addr)){
                BMP_ACT = true;
            }
            else{
                while(true){beep.BMP_ERR();delay(1000);};
                BMP_ACT = false;
            }
        }
        float getalt(){
            if(BMP_ACT){
            return bmp.readAltitude(SEALEVELPRESSURE_HPA);
            }
            else{
                return -1;
            }
        }
        float getpres(){
            if(BMP_ACT){
            return bmp.readPressure();
            }
            else{
                return -1;
            }
        }
};

class IMU{
  public:
      IMU(int addr){

      }
      float getAccl(){
        return 0.0;
      }
      float getGyro(){
        return 0.0;
      }  
};
#endif