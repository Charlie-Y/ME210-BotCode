//Testing the input from ir sensors


// -- linear actuator with l293 test

unsigned char direction_pin = 7; //black
unsigned char enable_pin = 6; // dark green

void setup(){
  pinMode(direction_pin, OUTPUT);    
  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, HIGH);

}

void loop(){
  digitalWrite(direction_pin, LOW);
  delay(1000);
  digitalWrite(direction_pin, HIGH);
  delay(1000);
}

//--- analog input read
//unsigned char inputPin = A0;
//
//
//int sensorValue = 0;
//
//void setup(){
//  Serial.begin(9600);
//}
//
//void loop(){
//  sensorValue = analogRead(inputPin);
//  Serial.print("Value: ");
//  Serial.println(sensorValue);
//  delay(1);
//}

