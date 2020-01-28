

void setup()
{
	Serial.begin(9600);
	pinMode(9, OUTPUT);
}

void loop()
{
	int inputval = analogRead(A0);
	Serial.println(inputval);
	delay(0.01);
}
