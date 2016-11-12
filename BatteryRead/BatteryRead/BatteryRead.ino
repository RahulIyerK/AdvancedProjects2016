void setup() 
{
  Serial.begin(9600);
}

////const double VOLTAGE_ADC_RATIO = ((3.3 / 1023) / ((3.3 / (3.3 + 1)) * 4.2)) * 4.2;
//
//const double VOLTAGE_ADC_RATIO = (3.2232 / 1023) * ((3.3 + 1)/3.3);

const double INPUT_MAX_VOLTS = 3.496;
const double R1 = .99; //1k resistor
const double R2 = 3.26; //3.3k resistor

void loop() 
{

int voltage_DividerOut = analogRead(1);

Serial.print(voltage_DividerOut);
Serial.print(" , ");
Serial.println(voltage_DividerOut * (INPUT_MAX_VOLTS / 1023) * ((R1 + R2)/ R2));
}
