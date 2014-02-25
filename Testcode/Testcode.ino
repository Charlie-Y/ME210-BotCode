//Testing the input from ir sensors





unsigned char inputPin = A0;

int sensorValue = 0;

void setup(){
  Serial.begin(9600);
}

void loop(){
  sensorValue = analogRead(inputPin);
  Serial.print("Value: ");
  Serial.println(sensorValue);
  delay(1);
}

