#include <ESP32Servo.h>  
#include <SoftwareSerial.h>
#include <BluetoothSerial.h> 
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif 

BluetoothSerial SerialA ;
char command ;

unsigned long timer = 0;
#define refresh 1000

Servo mysevo1;
Servo mysevo2;

int measurePin = 34;
int ledPower = 2;
int botton = 14;
int waterlevel =39 ;
#define relayPin 27 //pump
int buz = 3;


unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
int valuewater ;


int servo1 = 13 ;	
int servo2 = 12 ;

#define rainAnalog  35




void setup() {

  pinMode(botton,INPUT_PULLUP);

 Serial.begin(115200);

   SerialA.begin("esp32_control");
  Serial.println("Start");

  pinMode(ledPower,OUTPUT);
    pinMode(relayPin, OUTPUT);
    pinMode(buz, OUTPUT);


    mysevo1.attach(servo1);
    mysevo1.write(0);
    mysevo2.attach(servo2);
    mysevo2.write(180);
digitalWrite(relayPin, HIGH);


}
  
void loop() {

 int rainAnalogVal =  analogRead(rainAnalog);
 int rainDigitalVal =  digitalRead(rainAnalog);

  if (millis() - timer >= refresh) {
    timer = millis();
        Serial.print(F("Water Level: "));
        SerialA.print(valuewater,1);
        SerialA.print(" ");
        Serial.print(F("dustDensity: "));
        SerialA.print(dustDensity,1);
        SerialA.print(" ");
        Serial.print(F("rain: "));
        SerialA.print(rainAnalogVal,1);
        SerialA.print(" ");

  }

  if (SerialA.available()) {
     command = SerialA.read();
    Serial.println("Read!");
    while(SerialA.available()){
      SerialA.read();
     }
       if (command == '1') { 
        cleanlow();
      }
   else  if (command == '0') { //Led Off
    stop();
         }
          else if (command == 'X') { //Relay On
          pump();
    }
    }

    Serial.println("rain value :");
        delay(500);
    Serial.println(rainAnalogVal);
    if(rainAnalogVal < 4095){
      cleanlow();
    }
    else if (rainAnalogVal <1000)
    {
      cleanHigh();
    }

    else if (rainAnalogVal == 4095)
    {
      stop();
    }

    digitalWrite(ledPower,LOW);
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin);

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH);
  delayMicroseconds(sleepTime);

  calcVoltage = voMeasured*(5.0/1024);
  dustDensity = 0.17*calcVoltage-0.1;

  if ( dustDensity < 0)
  {
    dustDensity = 0.00;
  }

  Serial.println("Raw Signal Value (0-1023):");
  Serial.println(voMeasured);

  Serial.println("Voltage:");
  Serial.println(calcVoltage);

  Serial.println("Dust Density:");
  Serial.println(dustDensity);

  delay(1000);
  if ( dustDensity > 1)
  {
    dustDensity = 1.00;
        pump ();
    cleanlow();
      

  }
  else{stop();}

int state = digitalRead(botton);
if (state == 0){
   pump () ;
  cleanlow();
}
 valuewater = analogRead(waterlevel);
 if (valuewater == 0) {
    Serial.print("Empty ");
    //  digitalWrite(relayPin , HIGH);
    //  digitalWrite(buz , HIGH);
  } else if (valuewater > 1 && valuewater < 350) {
    Serial.print("LOW   ");
    digitalWrite(relayPin , HIGH);

  } else if (valuewater > 350 && valuewater < 510) {
   Serial.print("Medium");
  } else if (valuewater > 510){
    Serial.print("HIGH  ");
  }
	}

void cleanHigh() {
  for(int e=0 ; e < 4 ; e++){

mysevo1.write(0);
mysevo2.write(180);
delay(500);
mysevo1.write(180);
mysevo2.write(0);
delay(500);
  }
}
void cleanlow() {
for(int i=0 ; i<4 ; i++){

mysevo1.write(0);
mysevo2.write(180);
delay(1000);
mysevo1.write(180);
mysevo2.write(0);
delay(1000);
	}
}

void stop ()
{
    mysevo1.write(0);
    mysevo2.write(180);

}

void pump ()
{
  digitalWrite(relayPin , LOW);
  delay(1000);
  digitalWrite(relayPin , HIGH);
    delay(1000);


}