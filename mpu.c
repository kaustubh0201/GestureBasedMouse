// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float base_x_accel;
float base_y_accel;
float base_z_accel;

float base_x_gyro;
float base_y_gyro;
float base_z_gyro;


typedef struct {
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;
} sensorValue;

unsigned long last_read_time;
float last_x_angle;    
float last_y_angle;
float last_z_angle;    

float last_gyro_x_angle;    
float last_gyro_y_angle;
float last_gyro_z_angle;



void set_last_read_angle_data(unsigned long time, float x, float y, float z, 
                             float x_gyro, float y_gyro, float z_gyro) {
    last_read_time = time;
    last_x_angle = x;
    last_y_angle = y;
    last_z_angle = z;
    last_gyro_x_angle = x_gyro;
    last_gyro_y_angle = y_gyro;
    last_gyro_z_angle = z_gyro;
}

void calibrate_sensors() {
    int   num_readings = 10;
    
    float x_accel = 0;
    float y_accel = 0;
    float z_accel = 0;
    float x_gyro = 0;
    float y_gyro = 0;
    float z_gyro = 0;
    
    sensorValue accel_t_gyro;
    
    read(&accel_t_gyro);
    
    // Read and average the raw values from the IMU
    for (int i = 0; i < num_readings; i++) {
        read(&accel_t_gyro);


        x_accel += accel_t_gyro.ax;
        y_accel += accel_t_gyro.ay;
        z_accel += accel_t_gyro.az;
        x_gyro += accel_t_gyro.gx;
        y_gyro += accel_t_gyro.gy;
        z_gyro += accel_t_gyro.gz;
        delay(100);
    }

    x_accel /= num_readings;
    y_accel /= num_readings;
    z_accel /= num_readings;
    x_gyro /= num_readings;
    y_gyro /= num_readings;
    z_gyro /= num_readings;
    
    // Store the raw calibration values globally
    base_x_accel = x_accel;
    base_y_accel = y_accel;
    base_z_accel = z_accel;
    base_x_gyro = x_gyro;
    base_y_gyro = y_gyro;
    base_z_gyro = z_gyro;
}

void read(sensorValue * sv){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sv->ax = a.acceleration.x; 
    sv->ay = a.acceleration.y; 
    sv->az = a.acceleration.z; 
    
    sv->gx = g.gyro.x; 
    sv->gy = g.gyro.y; 
    sv->gz = g.gyro.z; 
}

void setup(void) {
    Serial.begin(115200);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    // Serial.println("Adafruit MPU6050 test!");

    // Try to initialize!
    if (!mpu.begin()) {
        //Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    //Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    //Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
        //Serial.println("+-2G");
        break;
    case MPU6050_RANGE_4_G:
        //Serial.println("+-4G");
        break;
    case MPU6050_RANGE_8_G:
        //Serial.println("+-8G");
        break;
    case MPU6050_RANGE_16_G:
        //Serial.println("+-16G");
        break;
    }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    //Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
        //Serial.println("+- 250 deg/s");
        break;
    case MPU6050_RANGE_500_DEG:
     //Serial.println("+- 500 deg/s");
        break;
    case MPU6050_RANGE_1000_DEG:
        //Serial.println("+- 1000 deg/s");
        break;
    case MPU6050_RANGE_2000_DEG:
        //Serial.println("+- 2000 deg/s");
        break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    //Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
        //Serial.println("260 Hz");
        break;
    case MPU6050_BAND_184_HZ:
        //Serial.println("184 Hz");
        break;
    case MPU6050_BAND_94_HZ:
        //Serial.println("94 Hz");
        break;
    case MPU6050_BAND_44_HZ:
        //Serial.println("44 Hz");
        break;
    case MPU6050_BAND_21_HZ:
        //Serial.println("21 Hz");
        break;
    case MPU6050_BAND_10_HZ:
        //Serial.println("10 Hz");
        break;
    case MPU6050_BAND_5_HZ:
     // Serial.println("5 Hz");
        break;
    }

    calibrate_sensors();
    set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
    

    delay(100);
}

void loop() {
    sensorValue sv;
    read(&sv);
    unsigned long t_now = millis();

    float FS_SEL = 131;

    float gyro_x = (sv.gx - base_x_gyro)/FS_SEL;
    float gyro_y = (sv.gy - base_y_gyro)/FS_SEL;
    float gyro_z = (sv.gz - base_z_gyro)/FS_SEL;
    
    
    //Get raw acceleration values
    //float G_CONVERT = 16384;
    float accel_x = sv.ax;
    float accel_y = sv.ay;
    float accel_z = sv.az;

    float RADIANS_TO_DEGREES = 180/3.14159;
    float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
    float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
    
    float accel_angle_z = 0;

    float dt = (t_now - last_read_time)/1000.0;

    //Filtered
    float gyro_angle_x = gyro_x*dt + last_x_angle;
    float gyro_angle_y = gyro_y*dt + last_y_angle;
    float gyro_angle_z = gyro_z*dt + last_z_angle;
    
    //Unfiltered
    float unfiltered_gyro_angle_x = gyro_x*dt + last_gyro_x_angle;
    float unfiltered_gyro_angle_y = gyro_y*dt + last_gyro_y_angle;
    float unfiltered_gyro_angle_z = gyro_z*dt + last_gyro_z_angle;

    float alpha = 0.96;
    float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
    float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
    float angle_z = gyro_angle_z;    //Accelerometer doesn't give z-angle

    set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
    // Send the data to the serial port
    Serial.print(F("DEL:"));                            //Delta T
    Serial.print(dt, DEC);
    Serial.print(F("#ACC:"));                            //Accelerometer angle
    Serial.print(accel_angle_x, 2);
    Serial.print(F(","));
    Serial.print(accel_angle_y, 2);
    Serial.print(F(","));
    Serial.print(accel_angle_z, 2);
    Serial.print(F("#GYR:"));
    Serial.print(unfiltered_gyro_angle_x, 2);                //Gyroscope angle
    Serial.print(F(","));
    Serial.print(unfiltered_gyro_angle_y, 2);
    Serial.print(F(","));
    Serial.print(unfiltered_gyro_angle_z, 2);
    Serial.print(F("#FIL:"));                         //Filtered angle
    Serial.print(angle_x, 2);
    Serial.print(F(","));
    Serial.print(angle_y, 2);
    Serial.print(F(","));
    Serial.print(angle_z, 2);
    Serial.println(F(""));
    
    // Delay so we don't swamp the serial port
    delay(5);
}
