/*
  Hel-Smart
*/

#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
#define REPORTING_PERIOD_MS 1000
//#include <Adafruit_BME280.h>
#include "time.h"

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"



// Insert your network credentials
#define WIFI_SSID "Galaxy F41BCFE"
#define WIFI_PASSWORD "M079337*"

// Insert Firebase project API Key
#define API_KEY "AIzaSyDr6dhGf5k-59KVoOmYx8nKfS4_xg5CU38"

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "test@gmail.com"
#define USER_PASSWORD "pass@123"

// Insert RTDB URLefine the RTDB URL
#define DATABASE_URL "https://ride-safe-bdc03-default-rtdb.asia-southeast1.firebasedatabase.app/"

// Define Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variable to save USER UID
String uid;

// Database main path (to be updated in setup with the user UID)
String databasePath;
// Database child nodes
String BPMPath = "/Rider Vitals/BPM";
String SpO2Path = "/Rider Vitals/SpO2";
String RiderTempPath = "/Rider Vitals/Rider Temp";
String AxPath = "/Accleration/Ax";
String AyPath = "/Accleration/Ay";
String AzPath = "/Accleration/Az";
String flagPath = "/Flag";
String timePath = "/timestamp";

// Parent Node (to be updated in every loop)
String parentPath;

int timestamp;
FirebaseJson json;

const char* ntpServer = "pool.ntp.org";

// BME280 sensor
//Adafruit_BME280 bme; // I2C
//float temperature;
//float humidity;
//float pressure;

// Timer variables (send new readings every three minutes)
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 1000;

// Initialize BME280
//void initBME(){
//  if (!bme.begin(0x76)) {
//    Serial.println("Could not find a valid BME280 sensor, check wiring!");
//    while (1);
//  }
//}

// Initialize WiFi
void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  Serial.println();
}

// Function that gets current epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}
String flag = "SAFE";
PulseOximeter pox;
float BPM, SpO2;
uint32_t tsLastReport = 0;

// For Accelerometer sensor
const int MPU_addr=0x68;
int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
float gForceX, gForceY, gForceZ,rotX, rotY, rotZ;

void onBeatDetected()
{
    Serial.println("Beat Detected!");
}

void setup(){
  Serial.begin(115200);
  /*-----------------SENSOR INITIALIZATION-------------*/
  Serial.println("Initializing Pulse Oximeter..");
 
    if (!pox.begin())
    {
         Serial.println("FAILED");
         for(;;);
    }
    else
    {
         Serial.println("SUCCESS");
         pox.setOnBeatDetectedCallback(onBeatDetected);
    }
 
    // The default current for the IR LED is 50mA and it could be changed by uncommenting the following line.
        pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
    if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };
  Serial.print("Emissivity = "); Serial.println(mlx.readEmissivity());
  Serial.println("================================================");
      
  Wire1.begin(33,32,100000ul); 
  Wire1.beginTransmission(MPU_addr);
  Wire1.write(0x6B);  // PWR_MGMT_1 register
  Wire1.write(0);     // set to zero (wakes up the MPU-6050)
  Wire1.endTransmission(true);
  // Initialize BME280 sensor
  //initBME();
  initWiFi();
  configTime(0, 0, ntpServer);

  // Assign the api key (required)
  config.api_key = API_KEY;

  // Assign the user sign in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Assign the RTDB URL (required)
  config.database_url = DATABASE_URL;

  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);

  // Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  // Assign the maximum retry of token generation
  config.max_token_generation_retry = 5;

  // Initialize the library with the Firebase authen and config
  Firebase.begin(&config, &auth);

  // Getting the user UID might take a few seconds
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);

  // Update database path
  databasePath = "/UsersData/" + uid + "/readings";  
}

void loop(){

  // Reading data from Accelerometer sensor
  
  Wire1.beginTransmission(MPU_addr);
  Wire1.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire1.endTransmission(false);
  Wire1.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  accelX=Wire1.read()<<8|Wire1.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accelY=Wire1.read()<<8|Wire1.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accelZ=Wire1.read()<<8|Wire1.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  gyroX=Wire1.read()<<8|Wire1.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY=Wire1.read()<<8|Wire1.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ=Wire1.read()<<8|Wire1.read();

  gForceX = (accelX-2050) / 16384.0;
  gForceY = (accelY-77) / 16384.0; 
  gForceZ = (accelZ-1947) / 16384.0;

  rotX = (gyroX+270) / 131.0;
  rotY = (gyroY-351) / 131.0; 
  rotZ = (gyroZ+136) / 131.0;

  Serial.print("RotX: ");
  Serial.print(rotX);
  Serial.println();
  Serial.print("RotY: ");
  Serial.print(rotY);
  Serial.println();
  Serial.print("RotZ: ");
  Serial.print(rotZ);
  Serial.println();
  //Serial.print("RotY: ");
  Serial.print("aX: ");
  Serial.print(gForceX);
  Serial.println();
  Serial.print("aY: ");
  Serial.print(gForceY);
  Serial.println();
  Serial.print("aZ: ");
  Serial.print(gForceZ);
  Serial.println(" ");
  delay(50);
  //delay(100);
  // calculating Amplitute vactor for 3 axis
  float Raw_Amp = pow(pow(gForceX,2)+pow(gForceY,2)+pow(gForceZ,2),0.5);
  int Amp = Raw_Amp * 10;  // Mulitiplied by 10 bcz values are between 0 to 1
  int angleChange = pow(pow(rotX,2)+pow(rotY,2)+pow(rotZ,2),0.5);

  // Reading data from pulseoximeter sensor
  //pox.begin();
  pox.update();
  BPM = pox.getHeartRate();
  SpO2 = pox.getSpO2();
  //pox.shutdown();
//  Serial.println(" ");
  Serial.print("BPM: ");
  Serial.print(BPM);
  Serial.println();
  Serial.print("SpO2: ");
  Serial.print(SpO2);
  Serial.println(" ");
  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());
  Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
  //delay(1000);
  // Send new readings to database
  //bool check= true;
  if((gForceX>1.00 || gForceY>1.00 || gForceZ>1.00)||(gForceX<-1.00 || gForceY<-1.00 || gForceZ<-1.00)){
    flag = "SOS";
    //check = false;
  }
//  if(check == false){
//    delay(5000);
//    flag = "SAFE";
//  }
  Serial.println(flag);
  if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();

    //Get current timestamp
    timestamp = getTime();
    Serial.print ("time: ");
    Serial.println (timestamp);

    //parentPath= databasePath + "/" + String(timestamp);

    json.set(BPMPath.c_str(), String(BPM));
    json.set(SpO2Path.c_str(), String(SpO2));
    json.set(RiderTempPath.c_str(), String(mlx.readObjectTempC()));
    json.set(AxPath.c_str(), String(gForceX));
    json.set(AyPath.c_str(), String(gForceY));
    json.set(AzPath.c_str(), String(gForceZ));
    json.set(flagPath.c_str(), String(flag));
    json.set(timePath, String(timestamp));
    Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, databasePath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
  }
}
