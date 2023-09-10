#include <Wire.h>
#include <Servo.h>

#define KP 3
#define KI 0.5
#define KD 0.0
#define Q 1
#define RANGE 200
#define FIX 5
#define MPU6050_AXOFFSET 158
#define MPU6050_AYOFFSET 9
#define MPU6050_AZOFFSET -91
#define MPU6050_GXOFFSET 19
#define MPU6050_GYOFFSET -42
#define MPU6050_GZOFFSET -26

long sampling_timer;
const int MPU_addr=0x68;  // I2C address of the MPU-6050

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; // Raw data of MPU6050
float GAcX, GAcY, GAcZ; // Convert accelerometer to gravity value
float gyro_theta, acc_theta, theta, error_diff, error_int, past_error, past_theta, error, target_angle;
float alpha = 0.96; // Complementary constant
long pulse, pulse_counter;
Servo myservo;

void setup(){
  myservo.attach(9, 1480 - RANGE, 1480 + RANGE);
  myservo.write(90);
  delay(2000);
  Wire.begin();
  init_MPU6050();
  Serial.begin(115200);
  target_angle = 0;
  error_int = 0;
  past_error = 0;
  past_theta = 0;
  pulse_counter = 0;
}

void loop(){
  // Read raw data of MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // Raw data of accelerometer corrected by offset value
//  AcX -= MPU6050_AXOFFSET;
//  AcY -= MPU6050_AYOFFSET;
//  AcZ -= MPU6050_AZOFFSET;

  // Convert accelerometer to gravity value
  GAcX = (float) AcX / 4096.0;
  GAcY = (float) AcY / 4096.0;
  GAcZ = (float) AcZ / 4096.0;

  // Calculate Pitch, Roll & Yaw from Accelerometer value
  // Reference are 
  // https://engineering.stackexchange.com/questions/3348/calculating-pitch-yaw-and-roll-from-mag-acc-and-gyro-data
  // https://www.dfrobot.com/wiki/index.php/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
  acc_theta = atan ((GAcY - (float)MPU6050_AYOFFSET/4096.0) / sqrt(GAcX * GAcX + GAcZ * GAcZ)) * 57.29577951; // 180 / PI = 57.29577951
  gyro_theta = (float)(GyX - MPU6050_GXOFFSET) * 0.000244140625;


  // Calculate Pitch, Roll & Yaw by Complementary Filter
  // Reference is http://www.geekmomprojects.com/gyroscopes-and-accelerometers-on-a-chip/
  // Filtered Angle = α × (Gyroscope Angle) + (1 − α) × (Accelerometer Angle)     
  // where α = τ/(τ + Δt)   and   (Gyroscope Angle) = (Last Measured Filtered Angle) + ω×Δt
  // Δt = sampling rate, τ = time constant greater than timescale of typical accelerometer noise
  theta = alpha * (gyro_theta + theta) + (1 - alpha) * acc_theta;
  error = target_angle + theta;

  if (pulse_counter < 30) pulse = -1;
  else if (pulse_counter < 50) pulse = -1;
  else if (pulse_counter < 70) pulse = 2;
  else pulse_counter = pulse = 0;

  pulse_counter++;

  // if (theta < FIX && theta > -FIX)
  //   error = (theta - past_theta) ? -FIX : FIX;

  error = (int) error / Q;
  error *= Q;
  Serial.println(error);
  error_int += error * 4e-4;
  error_diff = (error - past_error) / 4e-4;
  past_error = error;
  past_theta = theta;

  float output = KP * error + KI * error_int + KD * error_diff;

  output = output * pulse;

  output = constrain(output + 90, 0, 180);
  output = map(output, 0, 180, 1480 - RANGE, 1480 + RANGE);
  myservo.writeMicroseconds(output);

  // Sampling Timer
  while(micros() - sampling_timer < 4000); //
  sampling_timer = micros(); //Reset the sampling timer  
}

void init_MPU6050(){
  //MPU6050 Initializing & Reset
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //MPU6050 Clock Type
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x03);     // Selection Clock 'PLL with Z axis gyroscope reference'
  Wire.endTransmission(true);

  //MPU6050 Gyroscope Configuration Setting
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // Gyroscope Configuration register
  //Wire.write(0x00);     // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec]
  //Wire.write(0x08);     // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec]
  //Wire.write(0x10);     // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec]
  Wire.write(0x18);     // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]
  Wire.endTransmission(true);

  //MPU6050 Accelerometer Configuration Setting
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);  // Accelerometer Configuration register
  //Wire.write(0x00);     // AFS_SEL=0, Full Scale Range = +/- 2 [g]
  //Wire.write(0x08);     // AFS_SEL=1, Full Scale Range = +/- 4 [g]
  Wire.write(0x10);     // AFS_SEL=2, Full Scale Range = +/- 8 [g]
  //Wire.write(0x18);     // AFS_SEL=3, Full Scale Range = +/- 10 [g]
  Wire.endTransmission(true);

  //MPU6050 DLPF(Digital Low Pass Filter)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);  // DLPF_CFG register
  Wire.write(0x00);     // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz 
  Wire.endTransmission(true);
}