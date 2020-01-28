const int FW_PWM[4] = {2, 3, 4, 5};
const int RE_PWM[4] = {6, 7, 8, 9};
const int F_EN[4] = {22, 36, 24, 25};
const int R_EN[4] = {28, 29, 30, 31};
const int Cytron_PWM[2] = {10, 11};
const int Cytron_DIR[2] = {32, 33};
//First two wheels are for left side
//last two wheels are for right side

void init_pins()
{
  for (int i = 0; i < 4; i++)
  {
    pinMode(F_EN[i], OUTPUT);
    digitalWrite(F_EN[i], 0);
    analogWrite(FW_PWM[i], 0);
    pinMode(R_EN[i], OUTPUT);
    digitalWrite(R_EN[i], 0);
    analogWrite(RE_PWM[i], 0);
  }

  for (int i = 0; i < 2; i++)
  {
    pinMode(Cytron_DIR[i], OUTPUT);
    digitalWrite(Cytron_DIR[i], 0);
    analogWrite(Cytron_PWM[i], 0);
  }
}

void setup()
{
  Serial.begin(57600); // set line end to Newline at

  init_pins();

  Serial.println("READY"); // bottom of serial monitor
}

float vals[6] = {0, 0, 0, 0};

void set_vals(const String &string_data)
{
  char *pch;
  char str[20];
  strcpy(str, string_data.c_str());
  pch = strtok(str, ",");
  int i = 0;
  while (pch != NULL)
  {
    vals[i] = atoi(pch);
    pch = strtok(NULL, ",");
    i++;
  }
}

void loop()
{
  if (Serial.available() > 0)
  {
    String data = Serial.readString();
    Serial.println(data);
    set_vals(data);

    // left wheels
    for (int i = 0; i < 2; i++)
    {
      if (vals[0] == 0)
      {
        digitalWrite(F_EN[i], HIGH);
        digitalWrite(R_EN[i], HIGH);
        analogWrite(RE_PWM[i], 0);
        analogWrite(FW_PWM[i], vals[1]);
      }
      else
      {
        digitalWrite(F_EN[i], HIGH);
        digitalWrite(R_EN[i], HIGH);
        analogWrite(FW_PWM[i], 0);
        analogWrite(RE_PWM[i], vals[1]);
      }
    }

    //right wheels
    for (int i = 2; i < 4; i++)
    {
      if (vals[2] == 0)
      {
        digitalWrite(F_EN[i], HIGH);
        digitalWrite(R_EN[i], HIGH);
        analogWrite(RE_PWM[i], 0);
        analogWrite(FW_PWM[i], vals[3]);
      }
      else
      {
        digitalWrite(F_EN[i], HIGH);
        digitalWrite(R_EN[i], HIGH);
        analogWrite(FW_PWM[i], 0);
        analogWrite(RE_PWM[i], vals[3]);
      }
    }

    digitalWrite(Cytron_DIR[0], vals[0]);
    analogWrite(Cytron_PWM[0], vals[1]);

    digitalWrite(Cytron_DIR[1], vals[2]);
    analogWrite(Cytron_PWM[1], vals[3]);
  }
  else
  {
    init_pins();
  }
  delay(2);
}
