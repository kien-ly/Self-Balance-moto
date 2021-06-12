/*==================================================================================

LUẬN VĂN TỐT NGHIỆP: THIẾT KẾ CHẾ TẠO XE MÁY ĐIỆN CÂN BẰNG SỬ DỤNG REACTIONWHEEL
Programer: Nguyễn Trọng Trân - Phan Hữu Thanh Tú

===================================================================================*/

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Servo.h>
#include <TimerOne.h>
#include <Encoder.h>
#include <string.h>


#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define enablea 44
#define enableb 13
#define in1 24
#define in2 22
#define in3 23
#define in4 25
#define en_wheel1 19
#define en_wheel2 27
#define en_run1 18
#define en_run2 26

#define tocdo   250


Kalman kalmanX; 
Kalman kalmanY;

/* IMU Data */
uint8_t t_chay;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Góc được tính chỉ sử dụng vận tốc góc
double kalAngleX, kalAngleY; // Tính góc sử dụng bộ lọc Kalman

uint32_t timer;
uint8_t i2cData[14]; // Buffer dữ liệu I2c


//Bo so LQR
float k1 = 2574.7;
float k2 = 273.9;
float k3 = 0;
float k4 = 1.1;
//float k1 = 600;
//float k2 = 120;
//float k3 = 0;
//float k4 = 0.25;
int chay = 0;
int state = 0;
int pwm_output = 0;//gia tri xung pwm

//long time_now = 0; 
uint32_t time_start = 0;

float a = 0;
float rpm = 0.0;
String v = "";

Encoder dem(19,27);
// TODO: Make calibration routine
//void callback();
void setup() {
  pinMode(in2,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(enablea,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(enableb,OUTPUT);
  pinMode(en_run1,INPUT);
  pinMode(en_run2,INPUT);
  pinMode(en_wheel1,INPUT);
  pinMode(en_wheel2,INPUT);
  pinMode(31,OUTPUT);
  //digitalWrite(23,LOW);
  //digitalWrite(25,HIGH);
  Serial3.begin(9600);
  Serial.begin(9600);
  Wire.begin();
  
//============================================
Timer1.initialize(100000);
Timer1.attachInterrupt(callback);
dem.write(0);

//=============================================
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Cài đặt tần số I2c là 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Cài đặt tần số I2c là 400kHz
#endif
  i2cData[0] = 7; // cài đặt tần số lấy mẫu 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
 #ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  timer = micros();
}

void loop() {

//====================================================================================================================
//Đọc encoder
  a = dem.read();
//Đọc cảm biên
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll); 
    kalAngleX = roll;
    
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif
  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
//================================================================================================================
//xuất dữ liệu

    Serial.println(rpm);Serial.print("\t");
    Serial.print(kalAngleX); Serial.print("\t");
    Serial.print(gyroXrate); Serial.print("\t");
    Serial.println(pwm_output);Serial.print("\t");
//================================================================================================================
// tính toán bộ LQR
if (chay == 1){
     pwm_output = 21*(k1*(kalAngleX+1.2)*3.14/180+k2*((gyroXrate-0.35)*3.14/180)+k4*rpm);//+0.1
  }
else pwm_output =0;
  
  if (pwm_output>255) pwm_output =255;
  if (pwm_output<-255) pwm_output = -255;
//================================================================================================================

  if (pwm_output>0) {
   // if(rpm>110) pwm_output =100 ;
    digitalWrite(22,HIGH);
    digitalWrite(24,LOW);
    analogWrite(44,pwm_output);
  }
//================================================================================================================
  if (pwm_output<0){
     //if(rpm<-100) pwm_output = -100;
    digitalWrite(22,LOW);
    digitalWrite(24,HIGH);
    analogWrite(44,abs(pwm_output));
  }
  else analogWrite(44,abs(pwm_output));

//=================================================================================================================
//code giao tiep với máy tính
//   if(Serial3.available()==1) 
//    {
//        //v = Serial3.readString();
//        char temp = Serial3.read();
//        //Serial.println(temp);
//        if(temp == 'p')
//        {
//            //state = 0;
//            state = 5;//dung dong co chay
//        }
//        if(temp == 'r')
//            state = 1;
//        if(temp == '2')
//            state = 2;//dung de xoa
//        if (temp == 'f')
//            state = 3;//xe chay toi
//        if (temp == 'b')
//            state = 4;//xe chay lui
//        if (temp == 's')
//            state = 0;
//    }
 // Thực thi các trường hợp với các giá trị của biến state
    switch(state)
    {
        // state = 0: dừng chạy
        case 0:
            analogWrite(45,0);//dung dong co
            
        break;
        // state = 1: thực thi hàm tạo Random, xuất dữ liệu và thời gian thực qua Serial, phân tách nhau bởi ký tự gạch đứng “|”
        case 1:
            chay = 1;
        break;
        // state = 2: Reset dữ liệu và thời gian về 0
        case 2:
            //time_now = 0;
        break;
        //state = 3
        case 3:
            digitalWrite(23,LOW);
            digitalWrite(25,HIGH);
            analogWrite(45,tocdo);
       
        break;
        //state = 4: xe chay lui
        case 4:
            digitalWrite(23,HIGH);
            digitalWrite(25,LOW);
            analogWrite(45,tocdo);
         /*   t_chay +=1;
            if(t_chay ==100)
            {
              state =0;
              t_chay=0;
            }
            */
         break;
         case 5:
         chay = 0;
         break;
         default:
          state = 1;
         break;
    }
//===============================================================================================================
  //Serial.print("\r\n");
  //delay(2);

}
//================================================================================================================
//Hàm tính vận tốc
void callback()
{
  time_start +=50;
  rpm = (((float)a)/2000.0)*20*3.14;
  dem.write(0);
  Serial3.print(time_start);Serial3.print("|");
  Serial3.print(-(kalAngleX+1.0));Serial3.print("|");
  Serial3.println(rpm);
}
