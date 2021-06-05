#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


Adafruit_MPU6050 mpu;
float RADIANS_TO_DEGREES = 180/3.14159;

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

/********************** KALMAN FILTER ************************/

class Filter {
    private:
        float accVariance = 0.001;
        float gyroBias = 0.003;
        float noiseVariance = 0.03;
        float rate = 0.0;
        float bias = 0.0;


        float errorCoVMat[2][2] = { 
            0, 0000.0,
            0000.0, 0
        };

        void caclUnbiasedRate(float dt, float newRate){
            rate = newRate - bias;
            angle += dt * (rate);
        }

        void updateErrorCovMat(float dt){
            errorCoVMat[0][0] += dt * (dt*errorCoVMat[1][1] - errorCoVMat[0][1] - errorCoVMat[1][0] + accVariance);
            errorCoVMat[0][1] -= dt * errorCoVMat[1][1];
            errorCoVMat[1][0] -= dt * errorCoVMat[1][1];
            errorCoVMat[1][1] += gyroBias * dt;
        }

        float getInnovation(float newAngle) {
            return newAngle - angle;
        }


        float getInnovationVar(){
            return errorCoVMat[0][0] + noiseVariance;
        }

        void calculateKalmanGain(float kalmanGain[2], float innovationVar){
            kalmanGain[0] = errorCoVMat[0][0] / innovationVar;
            kalmanGain[1] = errorCoVMat[1][0] / innovationVar;
        }

        void updateAngleBias(float kalmanGain[2], float innovation){
            angle += kalmanGain[0] * innovation;
            bias += kalmanGain[1] * innovation;
        }

        void updateErrorCovMat2(float kalmanGain[2]){
            float errorCoVMat00 = errorCoVMat[0][0];
            float errorCoVMat01 = errorCoVMat[0][1];

            errorCoVMat[0][0] -= kalmanGain[0] * errorCoVMat00;
            errorCoVMat[0][1] -= kalmanGain[0] * errorCoVMat01;
            errorCoVMat[1][0] -= kalmanGain[1] * errorCoVMat00;
            errorCoVMat[1][1] -= kalmanGain[1] * errorCoVMat01;
        }
    
    public:


        float angle = 0.0;

        void beginFiltering(float dt, float newAngle, float newRate){

            caclUnbiasedRate(dt, newRate);
            updateErrorCovMat(dt);
            
            float innovation = getInnovation(newAngle);
            float innovationVar = getInnovationVar();
            float kalmanGain[2] = {0, 0};

            calculateKalmanGain(kalmanGain, innovationVar);
            updateAngleBias(kalmanGain, innovation);
            updateErrorCovMat2(kalmanGain);
        }
};


Filter xFilter;
Filter yFilter;


/********************** END KALMAN FILTER ************************/



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

void calibrate_sensors() {
    int num_readings = 100;
    
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
        delay(150);
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

void setup(void) {
    Serial.begin(115200);
    while (!Serial)
        delay(10); 

    if (!mpu.begin()) {
        while (1) {
            delay(10);
        }
    }

    calibrate_sensors();
    set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);    

    delay(100);
}

float getYangle(float accel_x, float accel_y, float accel_z){
    return atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2))) * RADIANS_TO_DEGREES;
}

float getXangle(float accel_x, float accel_y, float accel_z){
    return atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))    * RADIANS_TO_DEGREES;
}

void loop() {
    sensorValue sv;
    read(&sv);
    unsigned long t_now = millis();

    //Get gyro angle and convert to degree
    float gyro_x = (sv.gx - base_x_gyro) * RADIANS_TO_DEGREES;
    float gyro_y = (sv.gy - base_y_gyro) * RADIANS_TO_DEGREES;
    float gyro_z = (sv.gz - base_z_gyro) * RADIANS_TO_DEGREES;
    
    //Get acc value
    float accel_x = sv.ax;
    float accel_y = sv.ay;
    float accel_z = sv.az;

    //Calculate x and y angle from accelerometer reading 
    float accel_angle_y = getYangle(accel_x, accel_y, accel_z);
    float accel_angle_x = getXangle(accel_x, accel_y, accel_z);
    float accel_angle_z = 0;

    float dt = (t_now - last_read_time)/1000.0;

    //Complementary Filter
    float gyro_angle_x = gyro_x*dt + last_x_angle;
    float gyro_angle_y = gyro_y*dt + last_y_angle;
    float gyro_angle_z = gyro_z*dt + last_z_angle;
    
    float alpha = 0.96;
    float angle_x = alpha * gyro_angle_x + (1.0 - alpha) * accel_angle_x;
    float angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;
    float angle_z = gyro_angle_z; 

    //Kalman Filter
    
    //X
    
    xFilter.beginFiltering(dt, accel_angle_x, gyro_x);

    //Y
    
    yFilter.beginFiltering(dt, accel_angle_y, gyro_y);


    //Unfiltered
    float unfiltered_gyro_angle_x = gyro_x * dt + last_gyro_x_angle;
    float unfiltered_gyro_angle_y = gyro_y * dt + last_gyro_y_angle;
    float unfiltered_gyro_angle_z = gyro_z * dt + last_gyro_z_angle;

  

    set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
    // Send the data to the serial port

    Serial.write((byte *) &xFilter.angle, 4);
    Serial.write((byte *) &yFilter.angle, 4);
    
    delay(150);
}