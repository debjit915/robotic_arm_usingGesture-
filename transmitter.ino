#include <Wire.h>
#include <WiFi.h>
//#include <MPU6050.h>

//These are needed for MPU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

const char *ssid = "drager";
const char *password = "debjit123";
const IPAddress receiverIP(192, 168, 0, 102); // IP address of the receiver ESP32

MPU6050 mpu;
  bool dmpReady = false;  // set true if DMP init was successful
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


struct PacketData 
{
  byte xAxisValue;
  byte yAxisValue;
  byte zAxisValue;  
};
PacketData data;

void setupMPU()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) 
  {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.setDMPEnabled(true);
      dmpReady = true;
  } 
}

void loop() {
  // Read MPU6050 sensor data
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO. Get the Latest packet
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  {  
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    int xAxisValue = constrain(ypr[2] * 180/M_PI, -90, 90);
    int yAxisValue = constrain(ypr[1] * 180/M_PI, -90, 90);
    int zAxisValue = constrain(ypr[0] * 180/M_PI, -90, 90);    
    data.xAxisValue = map(xAxisValue, -90, 90, 0, 254); 
    data.yAxisValue = map(yAxisValue, -90, 90, 0, 254);
    data.zAxisValue = map(zAxisValue, -90, 90, 0, 254);    


  // Create a UDP client
  WiFiUDP udp;
  udp.beginPacket(receiverIP, 12345); // Use the receiver's IP address and port
 /* udp.write((uint8_t *)&ax, sizeof(ax));
  udp.write((uint8_t *)&ay, sizeof(ay));
  udp.write((uint8_t *)&az, sizeof(az));*/
  udp.write((uint8_t *)&data, sizeof(data));
  udp.endPacket();
 /*Serial.print("Acceleration X: ");
    Serial.println(ax);
    Serial.print("Acceleration Y: ");
    Serial.println(ay);
    Serial.print("Acceleration Z: ");
    Serial.println(az);
  delay(100); */// Adjust the sending frequency as needed
  String inputData  = inputData + "values " + xAxisValue + "  " + yAxisValue + "  " + zAxisValue;
    Serial.println(inputData);
    delay(50);            
  }
}
void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
   setupMPU(); 
}
