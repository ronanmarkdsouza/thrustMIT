#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(8, 10); // CE, CSN
char check1='f';
int led_counter=0;
const byte addresses [][6] = {"00001", "00002"};
int relay = 4;
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(addresses[0]);      //Setting the address at which we will send the data
  radio.openReadingPipe(1, addresses[1]);
  radio.setPALevel(RF24_PA_MIN);
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
}
void loop() 
{
  delay(5);
  radio.startListening();
  
  if (radio.available())
  {
    int text;
    radio.read(&text, sizeof(text));
    if (text == 1)
    {
      led_counter=1;
      Serial.println("Armed");
    }
    else if (text == 2)
    {
      Serial.println("Launched");
      digitalWrite(relay, LOW);
      delay(5000);
      digitalWrite(relay, HIGH);
    }
  }
  
  delay(5);
  radio.stopListening(); 
  
   if(led_counter==0)
  {
  radio.write(&check1,sizeof(check1));
  }
}
