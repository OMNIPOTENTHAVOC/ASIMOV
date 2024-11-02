#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const int numReadings = 10;
float ax_filtered = 0.0;    
float ay_filtered = 0.0;   
float az_filtered = 0.0; 
const float alpha = 0.8;  


int16_t ax_offset = -230; 
int16_t ay_offset = 66; 
int16_t az_offset = 295; 

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();

  
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }
}

void loop() {
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;

    // average readings
    for (int i = 0; i < numReadings; i++) {
        int16_t ax, ay, az;
        mpu.getMotion6(&ax, &ay, &az, nullptr, nullptr, nullptr);

        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;

        delay(50); 
    }

    
    int16_t ax_avg = ax_sum / numReadings - ax_offset;
    int16_t ay_avg = ay_sum / numReadings - ay_offset;
    int16_t az_avg = az_sum / numReadings - az_offset;

    //low-pass filter
    ax_filtered = alpha * ax_filtered + (1 - alpha) * ax_avg;
    ay_filtered = alpha * ay_filtered + (1 - alpha) * ay_avg;
    az_filtered = alpha * az_filtered + (1 - alpha) * az_avg;

    //print filtered values
    Serial.print("Filtered Ax: ");
    Serial.print(ax_filtered);
    Serial.print(" Ay: ");
    Serial.print(ay_filtered);
    Serial.print(" Az: ");
    Serial.println(az_filtered);

    delay(500); // output rate delay
}
