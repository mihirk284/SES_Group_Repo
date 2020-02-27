
void setup()
{
  Serial.begin(9600);
  pinMode(A0, INPUT_PULLUP);
}

void loop()
{
  int inputval = analogRead(A0);
  Serial.println(inputval);
  int outputval = inputval *255/1023;
  analogWrite(13,outputval); 
  delay(0.01);
}
