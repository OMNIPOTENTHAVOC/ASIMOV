// Cytron Motor Driver Pins
const int MOTOR_A_DIR = 9;
const int MOTOR_A_PWM = 10;
const int MOTOR_B_DIR = 7;
const int MOTOR_B_PWM = 5;

// Actuator Pins
const int ACTUATOR_DIR = 4;
const int ACTUATOR_PWM = 2;

void setup() {
    Serial.begin(9600);
    
    // Set up motor driver pins
    pinMode(MOTOR_A_DIR, OUTPUT);
    pinMode(MOTOR_A_PWM, OUTPUT);
    pinMode(MOTOR_B_DIR, OUTPUT);
    pinMode(MOTOR_B_PWM, OUTPUT);
    
    // Set up actuator pins
    pinMode(ACTUATOR_DIR, OUTPUT);
    pinMode(ACTUATOR_PWM, OUTPUT);
}

void loop() {
    if (Serial.available()) {
        String coords = Serial.readStringUntil('\n');
        int commaIndex = coords.indexOf(',');
        if (commaIndex > 0) {
            String xStr = coords.substring(0, commaIndex);
            String yStr = coords.substring(commaIndex + 1);
            float x = xStr.toFloat();
            float y = yStr.toFloat();
            
            move_rover_to_target(x, y);
            
            if (reached_target(x, y)) {
                collect_soil();
            }
        }
    }
}

void move_rover_to_target(float targetX, float targetY) {
    while (true) {
        float deltaX = targetX - currentX;
        float deltaY = targetY - currentY;
        
        float threshold = 0.1;

        if (abs(deltaX) < threshold && abs(deltaY) < threshold) {
            stop_motors();
            break;
        }

        if (abs(deltaX) > abs(deltaY)) {
            if (deltaX > 0) {
                forward();
            } else {
                backward();
            }
        } else {
            if (deltaY > 0) {
                right();
            } else {
                left();
            }
        }
    }
}

void forward() {   
    digitalWrite(MOTOR_A_DIR, HIGH);
    analogWrite(MOTOR_A_PWM, 200); 
    digitalWrite(MOTOR_B_DIR, HIGH);
    analogWrite(MOTOR_B_PWM, 200);  
    delay(1000);  
}

void backward() {    
    digitalWrite(MOTOR_A_DIR, LOW);
    analogWrite(MOTOR_A_PWM, 200);  
    digitalWrite(MOTOR_B_DIR, LOW);
    analogWrite(MOTOR_B_PWM, 200); 
    delay(1000); 
}

void left() {
    digitalWrite(MOTOR_A_DIR, LOW);
    analogWrite(MOTOR_A_PWM, 200);  
    digitalWrite(MOTOR_B_DIR, HIGH);
    analogWrite(MOTOR_B_PWM, 200);  
    delay(1000);  
}

void right() {
    digitalWrite(MOTOR_A_DIR, HIGH);
    analogWrite(MOTOR_A_PWM, 200); 
    digitalWrite(MOTOR_B_DIR, LOW);
    analogWrite(MOTOR_B_PWM, 200); 
    delay(1000);  
}

void stop_motors() {
    analogWrite(MOTOR_A_PWM, 0);
    analogWrite(MOTOR_B_PWM, 0);
}

bool reached_target(float currentX, float currentY) {
    return true;
}

void collect_soil() {
    extend();
    delay(3000); 
    pullback();
    delay(3000);
}

void extend() {
    digitalWrite(ACTUATOR_DIR, HIGH);
    analogWrite(ACTUATOR_PWM, 255);  
}

void pullback() {
    digitalWrite(ACTUATOR_DIR, LOW);
    analogWrite(ACTUATOR_PWM, 255);  
}
