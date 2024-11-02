const int ch1Pin = 2;          // PWM input from FS-iA6B Channel 1 to Arduino D2
const int motorPWMPin = 5;     // Cytron PWM pin connected to Arduino D5
const int motorDirPin = 6;     // Cytron DIR pin connected to Arduino D6

int pwmValue = 0;              //store PWM pulse width from receiver
unsigned long pulseStart = 0;  //pulse timing

void setup() {
    // Set motor driver pins as outputs
    pinMode(motorPWMPin, OUTPUT);
    pinMode(motorDirPin, OUTPUT);

    // Set the channel input pin as input
    pinMode(ch1Pin, INPUT);

    Serial.begin(9600);  // For debugging
}

void loop() {
    //PWM signal from the receiver
    pwmValue = pulseIn(ch1Pin, HIGH, 25000);  // Reads pulse width in microseconds

    if (pwmValue > 1000 && pwmValue < 2000) {  //valid PWM range
        // Map the PWM signal from receiver to motor speed and direction
        int motorSpeed = map(abs(pwmValue - 1500), 0, 500, 0, 255);

        if (pwmValue > 1500) {
            digitalWrite(motorDirPin, HIGH);  //forward
        } else if (pwmValue < 1500) {
            digitalWrite(motorDirPin, LOW);   //reverse
        }

        //ctrl motor speed
        analogWrite(motorPWMPin, motorSpeed);

        //print the PWM value and motor speed
        Serial.print("PWM: ");
        Serial.print(pwmValue);
        Serial.print(" -> Motor Speed: ");
        Serial.println(motorSpeed);
    } else {
        //Signal is out of range, stop
        analogWrite(motorPWMPin, 0);
    }

    delay(20);
}
