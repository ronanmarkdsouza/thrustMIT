#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(8,9); // CE, CSN         
const byte addresses [][6] = {"00001", "00002"};  //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.
int ledState = HIGH; 
int arm =3 ;
int launch=2;
int arm_state=0;
int launch_state=0;
int button_state1 = 0;
int x=0,y=0;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;
char check1;
void setup()
{
  Serial.begin(9600);
  pinMode(arm,INPUT);
  pinMode(launch,INPUT);
  radio.begin();                  //Starting the Wireless communication
  radio.openWritingPipe(addresses[1]);     //Setting the address at which we will send the data
  radio.openReadingPipe(1, addresses[0]); //Setting the address where we will send the data
  radio.setPALevel(RF24_PA_MIN);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
}
void loop()
{
  delay(5);
  radio.startListening(); 
  
  radio.read(&check1,sizeof(check1));
  if(check1 == 'f')
  {
    Serial.println("f");
    digitalWrite(10,LOW);
    delay(1000);
    digitalWrite(10,HIGH);
  }
  
  delay(5);
  radio.stopListening();
  
  arm_state=digitalRead(arm);
  if(arm_state==1 && x==0)
  {
    x=1;
    
    radio.write(&x, sizeof(x));
    Serial.println("Armed");
  }
  launch_state=digitalRead(launch);
  if(arm_state==1 && x==1 && y==0 && launch_state==1)
  {
    y=2;
    radio.write(&y,sizeof(y));
    Serial.println("Launched");
  }
  delay(900);
}
