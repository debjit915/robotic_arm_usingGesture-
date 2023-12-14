#include <WiFi.h>
#include <WiFiUdp.h>
#include <vector>

const char *ssid = "drager";
const char *password = "debjit123";
#include <ESP32Servo.h>

Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoWrist;

// Define servo angles for different movements
int baseCenter = 90;
int shoulderCenter = 90;
int elbowCenter = 90;
int wristCenter = 90;







#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal
unsigned long lastRecvTime = 0;

WiFiUDP udp;
int16_t ax, ay, az;
struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
  byte zAxisValue;
};
PacketData receiverData;


void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
   /* udp.read((uint8_t *)&ax, sizeof(ax));
    udp.read((uint8_t *)&ay, sizeof(ay));
    udp.read((uint8_t *)&az, sizeof(az));

    // Process received MPU6050 data
    Serial.print("Acceleration X: ");
    Serial.println(ax);
    Serial.print("Acceleration Y: ");
    Serial.println(ay);
    Serial.print("Acceleration Z: ");
    Serial.println(az);*/

    udp.read((uint8_t *)&receiverData, sizeof(receiverData));
  String inputData ;
  inputData = inputData + "values " + receiverData.xAxisValue + "  " + receiverData.yAxisValue + "  " + receiverData.zAxisValue;
  Serial.println(inputData);
   
   if( receiverData.xAxisValue < 75 && receiverData.yAxisValue < 75)//forward right 
   {
  servoShoulder.write(shoulderCenter - 30);
  servoElbow.write(elbowCenter - 30);
   }else if ( receiverData.xAxisValue > 175 && receiverData.yAxisValue < 75) //forward left 
  {
     servoShoulder.write(shoulderCenter - 30);
  servoElbow.write(elbowCenter + 30);
  }   else if ( receiverData.xAxisValue < 75 && receiverData.yAxisValue > 175) // backward left
  {
 servoShoulder.write(shoulderCenter + 30);
  servoElbow.write(elbowCenter - 30);
  } else if ( receiverData.xAxisValue > 175 && receiverData.yAxisValue > 175) //backward right
  {
    servoShoulder.write(shoulderCenter + 30);
  servoElbow.write(elbowCenter + 30);

  } else if (receiverData.zAxisValue > 175){
   servoElbow.write(elbowCenter - 30);

  } //turn right
   else if (receiverData.zAxisValue < 75){
     servoElbow.write(elbowCenter + 30);
   }
    else if (receiverData.yAxisValue < 75){
    servoShoulder.write(shoulderCenter - 30);//forward  
    }  else if (receiverData.yAxisValue > 175){
      servoShoulder.write(shoulderCenter + 30); //backward
    }else if (receiverData.xAxisValue > 175){
      servoBase.write(baseCenter + 30);//right
    
    } else if (receiverData.xAxisValue < 75)
    {
       servoBase.write(baseCenter - 30);//left
    }

}
}

void setup(){
   Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  IPAddress ipAddress = WiFi.localIP();
  Serial.println("Connected to WiFi");
  Serial.println(ipAddress);
  udp.begin(12345); // Listen on port 12345
   servoBase.attach(12);      // Attach servos to appropriate pins
  servoShoulder.attach(4);
  servoElbow.attach(2);
  servoWrist.attach(10);

  // Initialize servos to center positions
  servoBase.write(baseCenter);
  servoShoulder.write(shoulderCenter);
  servoElbow.write(elbowCenter);
  servoWrist.write(wristCenter);
}




