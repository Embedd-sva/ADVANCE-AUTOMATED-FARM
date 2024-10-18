#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Base64.h>
#include <SoftwareSerial.h>

#define ESP32_CMD_UART_RX 16  // RX pin for ESP32 CMD
#define ESP32_CMD_UART_TX 17  // TX pin for ESP32 CMD

#define ESP32_CAM_UART_RX 26  // RX pin for ESP32 CAM
#define ESP32_CAM_UART_TX 27  // TX pin for ESP32 CAM

SoftwareSerial serial3(18,19); //Serial for Voice command communication

// FIREBASE REQUIRED CONSTANTS 
#define FIREBASE_HOST "https://aaf-app-9ba60-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "AIzaSyCpWLG8aPMLBZtiCaysBdtvKzL8WKDAbP4"
const char *ssid = "Siva";
const char *password = "1234567890";

FirebaseData fbd;
FirebaseJson json;
FirebaseConfig config;
FirebaseAuth auth;

// VARIABLES TO GET VALUES FROM FIREBASE
String m1 ;
String rs ;
String m2;
String cam;
String lts;
String ftr;
#define ON 1
#define OFF 0



void setup() {
  Serial.begin(115200);  // For debugging 
  Serial1.begin(230400, SERIAL_8N1, ESP32_CMD_UART_RX, ESP32_CMD_UART_TX); // For receiving CMD data
  Serial2.begin(230400, SERIAL_8N1, ESP32_CAM_UART_RX, ESP32_CAM_UART_TX); // For receiving CAM data
  serial3.begin(115200);
  Serial.println();

  // CONNECTING ESP32 TO WIFI
  Serial.print("[WiFi] Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  //INITIALISING FIREBASE
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {

   Serial.println("loop is running");
   //GETTING VALUES FROM FIREBASE
   String LIGHTS, CAM, FTRAP, M1, M2, RSENSOR, VCMD;
    
    if (Firebase.getString(fbd, "/AAF_APP/lights")) LIGHTS = fbd.stringData();
    else Serial.println("Failed to get LIGHTS");

    if (Firebase.getString(fbd, "/AAF_APP/cam")) CAM = fbd.stringData();
    else Serial.println("Failed to get CAM");

    if (Firebase.getString(fbd, "/AAF_APP/fly_trap")) FTRAP = fbd.stringData();
    else Serial.println("Failed to get FTRAP");

    if (Firebase.getString(fbd, "/AAF_APP/motor_1")) M1 = fbd.stringData();
    else Serial.println("Failed to get MOTOR_1");

    if (Firebase.getString(fbd, "/AAF_APP/motor_2")) M2 = fbd.stringData();
    else Serial.println("Failed to get MOTOR_2");

    if (Firebase.getString(fbd, "/AAF_APP/rain_sensor")) RSENSOR = fbd.stringData();
    else Serial.println("Failed to get RSENSOR");

    if (Firebase.getString(fbd, "/AAF_APP/voice_cmds")) VCMD = fbd.stringData();
    else Serial.println("Failed to get RSENSOR");

    // Send commands to K210 via Serial1 and update Firebase only if necessary
    if (M1 == "1") {
        Serial1.println("M1_ON");
    } 
    else {
        Serial1.println("M1_OFF"); 
    }
    if (M2 == "1") {
        Serial1.println("M2_ON");
    } 
    else {
        Serial1.println("M2_OFF");
    }    
    if (LIGHTS == "1") {
        Serial1.println("LIGHTS_ON");      
    }
    else {
        Serial1.println("LIGHTS_OFF");   
    }
    if (RSENSOR == "1") {
        Serial1.println("RSENSOR_ON");
    } 
    else {
        Serial1.println("RSENSOR_OFF");    
    }
    if (FTRAP == "1") {
        Serial1.println("FTRAP_ON");   
    } else {
        Serial1.println("FTRAP_OFF");   
    }
    if (CAM == "1") {
       Serial1.println("CAM_ON");
    } 
    else {
       Serial1.println("CAM_OFF");
    }
    if (VCMD == "1") {
       Serial1.println("V1");
    } 
    else {
       Serial1.println("V0");
    }
    delay(200);
    if (Serial2.available() >= sizeof(uint16_t) * 2) { // Check if enough bytes for width and height
        uint16_t width, height;
        // Read width and height from camera data
        Serial2.readBytes((uint8_t*)&width, sizeof(width));
        Serial2.readBytes((uint8_t*)&height, sizeof(height));
        Serial.print("Width: ");
        Serial.print(width);
        Serial.print(", Height: ");
        Serial.println(height);
        // Prepare buffer to receive image data
        uint8_t* img = new uint8_t[width * height * 2]; //buffer for image data
        // Read image data
        Serial2.readBytes(img, width * height * 2);
        // Encode image data to base64 string
        String base64Image = base64::encode(img, width * height * 2);
        Firebase.setString(fbd, "/AAF_APP/cam_image", base64Image);
        // Send base64 image string to Firebase Realtime Database
        if (Firebase.setString(fbd, "/AAF_APP/cam_image", base64Image)) {
            Serial.println("Image sent to Firebase");
        } else {
            Serial.println("Failed to send image to Firebase");
        }
        delete[] img;  // Free the allocated buffer
    }
    delay(100); // Avoid task starvation

    //RECEVING COMMAND FROM K210 WHEN VOICE COMMANDS ACTIVATED
    if(serial3.available()){
        String K210CMD = serial3.readStringUntil('\n');
        K210CMD.trim();
        Serial.println(K210CMD);

        if(K210CMD == "M1_ON_ACK"){
            Firebase.setString(fbd,"/AAF_APP/motor_1","1");
        }
        else if(K210CMD == "M1_OFF_ACK"){
            Firebase.setString(fbd,"/AAF_APP/motor_1","0");
        }
        else if(K210CMD == "M2_ON_ACK"){
            Firebase.setString(fbd,"/AAF_APP/motor_2","1");
        }
        else if(K210CMD == "M2_OFF_ACK"){
            Firebase.setString(fbd,"/AAF_APP/motor_2","0");
        }
        else if(K210CMD == "FTRAP_ON_ACK"){
            Firebase.setString(fbd,"/AAF_APP/fly_trap","1");
        }
        else if(K210CMD == "FTRAP_OFF_ACK"){
            Firebase.setString(fbd,"/AAF_APP/fly_trap","0");
        }
        else if(K210CMD == "RSENSOR_ON_ACK"){
            Firebase.setString(fbd,"/AAF_APP/rain_sensor","1");
        }
        else if(K210CMD == "RSENSOR_OFF_ACK"){
            Firebase.setString(fbd,"/AAF_APP/rain_sensor","0");
        }
        else if(K210CMD == "LIGHTS_ON_ACK"){
            Firebase.setString(fbd,"/AAF_APP/lights","1");
        }
        else if(K210CMD == "LIGHTS_OFF_ACK"){
            Firebase.setString(fbd,"/AAF_APP/lights","0");
        }
        else if(K210CMD == "CAM_ON_ACK"){
            Firebase.setString(fbd,"/AAF_APP/cam","1");
        }
        else if(K210CMD == "CAM_OFF_ACK"){
            Firebase.setString(fbd,"/AAF_APP/cam","0");
        }
        else if(K210CMD == "VCMD_OFF_ACK"){
            Firebase.setString(fbd,"/AAF_APP/cam","0");
        }
    }


}


