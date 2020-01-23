String response = "";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(check_serial())
  {
    Serial.println(response);
  }
}

boolean check_serial()
{
  if(Serial2.available() > 0)
  {
    response = Serial2.readStringUntil('\n');
    if(response[0] == '#')
    {
      return true;
    }
  }
  return false;
}
