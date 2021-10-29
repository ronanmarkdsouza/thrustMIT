//Code for Deployment of Air Brakes with 2 states (50% and 100%) for Kill-Joy 2021
#include <SPI.h>
#include <SD.h>

#include <AccelStepper.h>
#include "SimpleStepperLib.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <utility/imumaths.h>

AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
SimpleStepper stepperx(SimpleStepper::HalfStep, 2,3,4,5);

float cur_v,cur_h; //velocity and height in m/s and m resp
float *cur_vp = &cur_v;
float *cur_hp=&cur_h;
float pred_vel;
float *pred_velp= &pred_vel;
const float thre_h = 100; //minimum height required for parachute deployment in case of failure in meters
const float setp = 500; //required apogee in meters
float ar,ax,ay,az; //acceleration in m/s^2
float *arp=&ar,*axp=&ax,*ayp=&ay,*azp=&az;
float cd; //drag coefficient
float *cdp=&cd;
float k1,k2,k3,k4,L1,L2,L3,L4;  // rk4 variables k for finding x and L for finding v
float dt=0.01;
float pred_ap; //predicted apogee in meters
float *pred_app=&pred_ap;
float rho; //air density in SI units
float *rhop=&rho;
int counter=0;
float timer;
const float dry_mass = 8; //dry mass of the rocket in kg
const float A[3]={0.0049,1,2};// 0% area 50% area and 100%area need array with state change
const int CS = 4; //Chip select pin for SPI
int total=2048; //total steps
int cur_state = 0,dep_state; //current state of Air Brakes and deployment state of Air Brakes
int *cur_statep=&cur_state, *dep_statep=&dep_state;
static int change_to=0; // change in state=
const int motorSpeed = 1000;

//BNO055
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensors_event_t event;

//BMP388
#define BMP_SCK 13 //BMP SCK Pin
#define BMP_MISO 12 //BMP MISO Pin
#define BMP_MOSI 11 //BMP MOSI Pin
#define BMP_CS 10 //BMP CS Pin
#define SEALEVELPRESSURE_HPA (1013.25) //BMP Sea Level Pressure
Adafruit_BMP3XX bmp; //BMP declaration

//parameters for pitot
int offset = 0;
int offset_size = 10;
int veloc_mean_size = 20;
int zero_span = 2;
float V_0 = 5; // supply voltage to the pressure sensor

void setup()
{
  Serial.begin(9600);
  SD.begin(CS);
  //BNO setup
  if(!bno.begin())
  {
    Serial.print("Ooops");
    while(1);
  }
  delay(1000);
  //BNO055
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  bno.setExtCrystalUse(true);
  bmp.begin_SPI(BMP_CS,BMP_SCK,BMP_MISO,BMP_MOSI);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

float velocity() //function to return current velocity
{
  float adc_avg = 0; float veloc = 0.0;
  // average a few ADC readings for stability
  for (int ii=0;ii<veloc_mean_size;ii++){
    adc_avg+= analogRead(A0)-offset;
  }
  adc_avg/=veloc_mean_size;
  // make sure if the ADC reads below 512, then we equate it to a negative velocity
  if (adc_avg>512-zero_span and adc_avg<512+zero_span){
  } else{
   
      veloc = sqrt((10000.0*((adc_avg/1023.0)+0.5))/rho)-64.53;
    
  }
  return veloc; // return velocity
   // delay for stability
}

float height() //function to return current height
{
  return bmp.readAltitude(SEALEVELPRESSURE_HPA);
}

float drag_coeff(float *cur_vp,int *cur_statep)
{
    float sqa = (*cur_vp)*(*cur_vp);
    float v = *cur_vp;
    if (*cur_statep == 0)
    {
        cd = (1.378*pow(10,-6)*sqa)-(3.218*pow(10,-4)*v)+0.9663; //equation for drag coefficient when air brakes is closed (determined through simulation)
    }
    else if (*cur_statep == 1)
    {
        //equation for drag coefficient when air brakes 50% open (determined through simulation)
    }
    else if (cur_state == 2)
    {
        cd = (4.82*pow(10,-5)*sqa)+(0.00732*v)-0.029; //equation for drag coefficient when air brakes 100% open (determined through simulation)
    }
    return cd;
}

float rho_alt(float *cur_hp) //function to return rho
{
    return rho=1.225010879723*pow((1-6.87558563248308*pow(10,-6)*(*cur_hp)),(4.25591641274834)); //Air density in kg/m3
}

bool phase(float *arp,float *axp,float *ayp,float *azp,float *cdp,float *rhop,float *cur_vp)
{
    (*arp) = sqrt(pow((*axp),2) + pow((*ayp),2) + pow((*azp),2));
    if((*arp)<0 && ((*arp) + (0.5*(*cdp)*(*rhop)*pow((*cur_vp), 2)*A[0]/(dry_mass)))<0)
    {
        return true;
    }
}

float ap_pred(float *cur_vp,float *cur_hp,float *cdp)
{
  *pred_velp=*cur_vp;
  *pred_app=*cur_hp;
for(counter=0;*pred_velp>0;counter+=1) // runs rk till v goes 0 or -ve  //check once pred or current
   {
    k1 = *pred_velp;
    L1 = (((-*cdp)*(*rhop)*pow(*pred_velp,2)*A[cur_state])/(2*dry_mass))- 9.81;
    k2 = *pred_velp;
    L2 = (((-*cdp)*(*rhop)*pow(*pred_velp+(0.5*dt*k1),2)*A[cur_state]))/(2*dry_mass)- 9.81;
    k3 = *pred_velp;
    L3 = (((-*cdp)*(*rhop)*pow(*pred_velp+(0.5*dt*k2),2)*A[cur_state]))/(2*dry_mass)- 9.81;
    k4 = *pred_velp;
    L4 = (((-*cdp)*(*rhop)*pow(*pred_velp+(dt*k3),2)*A[cur_state]))/(2*dry_mass)- 9.81;
    *pred_velp=(*pred_velp+(dt*(L1+(2*L2)+(2*L3)+L4))/6);
    *pred_app=(*pred_app+(dt*(k1+(2*k2)+(2*k3)+k4))/6);
    timer = timer + dt;
}
  return *pred_app;
}

int state(float *cur_vp)
{
    //Velocity values need to be updated
    if((*cur_vp)<200)
    {
        return 2;
    }
    if(400>(*cur_vp)>200)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int deploy(int *dep_statep, int *cur_statep) //function to deploy Air Brakes
{
  change_to = (*dep_statep)-(*cur_statep);
  if(change_to==0)
 {
  (*dep_statep)=(*cur_statep);
  return *cur_statep;
 }
else
{
  if(((*dep_statep)-(*cur_statep))>0 && ((*dep_statep)-(*cur_statep))==1 )
  {
  for (int i = 0; i <total/4; i++)
  {
    stepperx.CW();
    delayMicroseconds(motorSpeed); 
  }
   return *dep_statep;
  }
  if(((*dep_statep)-(*cur_statep))>0 && ((*dep_statep)-(*cur_statep))==2 )
  {
  for (int i = 0; i <total/2; i++)
  {
    stepperx.CW();
    delayMicroseconds(motorSpeed); 
  }
  return *dep_statep;
  }
  if(((*dep_statep)-(*cur_statep))<0 && ((*dep_statep)-(*cur_statep))==-1)
  {
  for (int i = 0; i <total/4; i++)
  {
    stepperx.CCW();
    delayMicroseconds(motorSpeed); 
  }
   return *dep_statep;
  }
  if(((*dep_statep)-(*cur_statep))<0 && ((*dep_statep)-(*cur_statep))==-2)
  {
  for (int i = 0; i <total/2; i++)
  {
    stepperx.CCW();
    delayMicroseconds(motorSpeed); 
  }
   return *dep_statep;
  }
} 
 ap_pred(&cur_v,&cur_h,&cd);
}

void sd_log(float *cur_hp,float *cur_vp,float *axp,float *ayp,float *azp,float *pred_app,int *cur_statep) //function to log data to SD Card
{
  String dataString = "";
  dataString += String(*cur_hp);
  dataString += ",";
  dataString += String(*cur_vp);
  dataString += ",";
  dataString += String(*axp);
  dataString += ",";
  dataString += String(*ayp);
  dataString += ",";
  dataString += String(*azp);
  dataString += ",";
  dataString += String(*pred_app);
  dataString += ",";
  dataString += String(*cur_statep);
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.println(dataString);
  dataFile.close();
}

void loop()
{
  cur_h = height();
  cur_v = velocity();
  cd = drag_coeff(&cur_v,&cur_state);
  rho = rho_alt(&cur_h);
  pred_ap = ap_pred(&cur_v,&cur_h,&cd);//make ap pred indepenent of stage
  bno.getEvent(&event);
  ax = event.acceleration.x;
  ay = event.acceleration.y;
  az = event.acceleration.z;
  
  while (phase(&ar,&ax,&ay,&az,&cd,&rho,&cur_v)== true && cur_h>thre_h && pred_ap>setp)
  {
    cur_h = height();
    cur_v = velocity();
    cd = drag_coeff(&cur_v,&cur_state);
    rho = rho_alt(&cur_h);
    pred_ap = ap_pred(&cur_v,&cur_h,&cd);
    dep_state = state(&cur_v);
    cur_state = deploy(&dep_state, &cur_state);
    bno.getEvent(&event);
    ax = event.acceleration.x;
    ay = event.acceleration.y;
    az = event.acceleration.z;
    sd_log(&cur_h,&cur_v,&ax,&ay,&az,&pred_ap,&cur_state);
  }
  if(pred_ap<=setp)
  {
    cur_state = deploy(0,&cur_state);
  }
  sd_log(&cur_h,&cur_v,&ax,&ay,&az,&pred_ap,&cur_state);
}
