void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

//const double VOLTAGE_ADC_RATIO = ((3.3 / 1023) / ((3.3 / (3.3 + 1)) * 4.2)) * 4.2;

const double VOLTAGE_ADC_RATIO = (3.2232 / 1023) * ((3.3 + 1)/3.3);

void loop() {
  // put your main code here, to run repeatedly:
Serial.println(analogRead(1) * VOLTAGE_ADC_RATIO);
}
