void setup() 
{
  Serial.begin(9600);
}

////const double VOLTAGE_ADC_RATIO = ((3.3 / 1023) / ((3.3 / (3.3 + 1)) * 4.2)) * 4.2;
//
//const double VOLTAGE_ADC_RATIO = (3.2232 / 1023) * ((3.3 + 1)/3.3);

const double INPUT_MAX_VOLTS = 3.3;
const double R1 = 1; //1k resistor
const double R2 = 3.3; //3.3k resistor

void loop() 
{
//Serial.println(analogRead(1) * VOLTAGE_ADC_RATIO);

int voltage_DividerOut = analogRead(1);

Serial.println(voltage_DividerOut * (INPUT_MAX_VOLTS / 1023) * ((R1 + R2)/ R2));
}
