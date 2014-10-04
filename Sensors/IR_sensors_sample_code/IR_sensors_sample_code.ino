/*
Sample code for long/short range IR sensor
val increases as object gets closer to the sensor, need to calibrate val with ref to a wall.
DIfferent objects will result in different readings from the sensor at the SAME distance

Long range sensor min distance ~12cm

Connections:
  Wire         Arduino Pin
  Red          +5v
  Black        GND
  Yellow       A0        

*/

int sensorPin = 0; // ANALOG_IN A0
int led = 13;

void setup(){
  Serial.begin(9600);
  pinMode(led,OUTPUT);
}

void loop(){
  int val = analogRead(sensorPin);
  
  // print val onto serial monitor
  Serial.println(val);
  
  // light up LED if val is above a certain value
  if(val > 400)
    digitalWrite(led,HIGH);
  else
    digitalWrite(led,LOW);  
  //just to slow down the output - remove if trying to catch an object passing by
  //delay(100);

}
