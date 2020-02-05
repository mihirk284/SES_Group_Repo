
void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop()
{
  int inputval = analogRead(A0);
  Serial.println(inputval);
  int outputval = inputval *255/1023;
  analogWrite(13,outputval); 
  delay(0.01);
}
