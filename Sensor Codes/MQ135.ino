const int mq135Pin = A0;

void setup() {
    Serial.begin(115200); // Start serial communication at 115200 baud
}

void loop() {
    // Read the analog value from the MQ135 sensor
    int mq135Value = analogRead(mq135Pin);

    //calibration formulas
    float ammoniaConcentration = (1.5 * mq135Value) - 100; // Ammonia concentration (ppm)
    float co2Concentration = (2.0 * mq135Value) - 100;     // CO2 concentration (ppm)

    //MQ135 value and concentrations over serial
    Serial.print("MQ135 Value: ");
    Serial.print(mq135Value);
    Serial.print(", Ammonia Concentration (ppm): ");
    Serial.print(ammoniaConcentration);
    Serial.print(", CO2 Concentration (ppm): ");
    Serial.println(co2Concentration);
  
    delay(1000); // Read every 1 second
}
