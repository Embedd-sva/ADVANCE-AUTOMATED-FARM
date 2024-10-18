#include <Arduino.h>
#include <Sipeed_ST7789.h>
#include <Sipeed_OV2640.h>
#include <SPI.h>
#include <Maix_Speech_Recognition.h>
#include <voice_model.h>
#include "FreeRTOS.h"
#include "task.h"

//UART PINS
#define K210_CMD_UART_TX 13  // TX pin for K210
#define K210_CMD_UART_RX 12  // RX pin for K210

#define K210_CAM_UART_TX 11  // TX pin for K210
#define K210_CAM_UART_RX 10  // RX pin for K210

#define K210_VCMD_UART_TX 9  // TX pin for K210
#define K210_VCMD_UART_RX 8  // RX pin for K210

// Define Pin Constants
#define M1_PIN 2 
#define M2_PIN 3 
#define LIGHT_PIN 4 
#define FTRAP_PIN 5 
#define RSENSOR_PIN 6
#define BUZZERPIN 7
#define READ_PIN A0


// LCD Display Constants
#define LABEL_TEXT_SIZE 3 // Font size multiplier
#define BG_COLOR COLOR_DARKCYAN

//SPEECH RECOGNITION CONSTANTS
SpeechRecognizer rec;
int VCMDS;

// SPI and LCD Initialization
SPIClass spi_(SPI0); // Must be SPI0 for Maix series on-board LCD
Sipeed_ST7789 lcd(320, 240, spi_, SIPEED_ST7789_DCX_PIN, SIPEED_ST7789_RST_PIN, DMAC_CHANNEL2);
Sipeed_OV2640 camera(FRAMESIZE_QQVGA, PIXFORMAT_RGB565);

// Function to print centered text on the LCD
void printCenterOnLCD(Sipeed_ST7789 &lcd_, const char *msg, uint8_t textSize = LABEL_TEXT_SIZE) {
    lcd_.setCursor((lcd_.width() - (6 * textSize * strlen(msg))) / 2, (lcd_.height() - (8 * textSize)) / 2);
    lcd_.print(msg);
}

//FUNCTION TO DISPLAT THE LOGO
void displayLogo(Sipeed_ST7789 &lcd_) {
    lcd_.fillScreen(COLOR_BLACK);  // Set the background color

    // Display large "AAF" text
    lcd_.setTextColor(COLOR_WHITE); 
    lcd_.setTextSize(6);             
    
    // Calculate position to center "AAF"
    int16_t x = (lcd_.width() - (6 * 3 * 6)) / 2;  
    int16_t y = (lcd_.height() - (8 * 6)) / 3;   

    // Set cursor to the calculated position and print "AAF"
    lcd_.setCursor(x, y);
    lcd_.print("AAF");

    // Display smaller "FARMING MADE SIMPLE" text below the "AAF"
    lcd_.setTextSize(2);  
    int16_t x_small = (lcd_.width() - (6 * 18 * 2)) / 2; 
    int16_t y_small = y + (8 * 6) + 30;  

    lcd_.setCursor(x_small, y_small);
    lcd_.print("FARMING MADE SIMPLE");
}


// Global variables
int camera_state = 0;  // This controls the camera state globally
unsigned long lastCaptureTime = 0;
const unsigned long captureInterval = 200;  // Capture every 200 ms

void setup() {
    Serial1.begin(230400, K210_CMD_UART_RX, K210_CMD_UART_TX); // To send COMMANDS
    Serial2.begin(230400, K210_CAM_UART_RX, K210_CAM_UART_TX); // To send CAM DATA
    Serial3.begin(115200,K210_VCMD_UART_RX,K210_VCMD_UART_TX); // To send Voice CMD
    Serial.begin(115200);  // For communicating

    // Initialize GPIO Pins
    pinMode(M1_PIN, OUTPUT);
    pinMode(M2_PIN, OUTPUT);
    pinMode(LIGHT_PIN, OUTPUT);
    pinMode(FTRAP_PIN, OUTPUT);
    pinMode(RSENSOR_PIN, OUTPUT);
    pinMode(BUZZERPIN,OUTPUT);
    pinMode(READ_PIN,INPUT);

    // Initialize LCD
    if (!lcd.begin(15000000, BG_COLOR)) {
        Serial.println("LCD initialization failed.");
        while (1); // Halt if LCD fails
    }
    lcd.setTextSize(LABEL_TEXT_SIZE);
    lcd.setTextColor(COLOR_WHITE);

    //TO PRINT TAG LINE ON LCD WHEN POWRED ON
    displayLogo(lcd);  
    delay(2000);
    lcd.fillScreen(BG_COLOR);

    rec.begin();
    // Load voice models for commands (the actual commands will vary based on the data you are using)
    rec.addVoiceModel(0, 0, motor1_on_0, fram_num_motor1_on_0); // TURN ON MOTOR1
    rec.addVoiceModel(0, 1, motor1_on_1, fram_num_motor1_on_1); 
    rec.addVoiceModel(0, 2, motor1_on_2, fram_num_motor1_on_2); //IN REGIONAL LANGUAGE
    rec.addVoiceModel(0, 3, motor1_on_3, fram_num_motor1_on_3); 

    rec.addVoiceModel(1, 0, motor2_on_0, fram_num_motor2_on_0); // TURN ON MOTOR2
    rec.addVoiceModel(1, 1, motor2_on_1, fram_num_motor2_on_1); 
    rec.addVoiceModel(1, 2, motor2_on_2, fram_num_motor2_on_2); //IN REGIONAL LANGUAGE
    rec.addVoiceModel(1, 3, motor2_on_3, fram_num_motor2_on_3); 

    rec.addVoiceModel(2, 0, flytrap_on_0, fram_num_flytrap_on_0); // TURN ON FLY TRAPS
    rec.addVoiceModel(2, 1, flytrap_on_1, fram_num_flytrap_on_1); 
    rec.addVoiceModel(2, 2, flytrap_on_2, fram_num_flytrap_on_2); //IN REGIONAL LANGUAGE
    rec.addVoiceModel(2, 3, flytrap_on_3, fram_num_flytrap_on_3); 

    rec.addVoiceModel(3, 0, rainsensor_on_0, fram_num_rainsensor_on_0); // TURN ON RAIN SENSOR
    rec.addVoiceModel(3, 1, rainsensor_on_1, fram_num_rainsensor_on_1); 
    rec.addVoiceModel(3, 2, rainsensor_on_2, fram_num_rainsensor_on_2); //IN REGIONAL LANGUAGE
    rec.addVoiceModel(3, 3, rainsensor_on_3, fram_num_rainsensor_on_3);

    rec.addVoiceModel(4, 0, lights_on_0, fram_num_lights_on_0); // TURN ON LIGHTs
    rec.addVoiceModel(4, 1, lights_on_1, fram_num_lights_on_1); 
    rec.addVoiceModel(4, 2, lights_on_2, fram_num_lights_on_2); //IN REGIONAL LANGUAGE
    rec.addVoiceModel(4, 3, lights_on_3, fram_num_lights_on_3); 

    rec.addVoiceModel(5, 0, camera_on_0, fram_num_camera_on_0); // TURN ON CAM
    rec.addVoiceModel(5, 1, camera_on_1, fram_num_camera_on_1); 
    rec.addVoiceModel(5, 2, camera_on_2, fram_num_camera_on_2); //IN REGIONAL LANGUAGE
    rec.addVoiceModel(5, 3, camera_on_3, fram_num_camera_on_3); 

    rec.addVoiceModel(6, 0, motor1_off_0, fram_num_motor1_off_0); // TURN OFF MOTOR1
    rec.addVoiceModel(6, 1, motor1_off_1, fram_num_motor1_off_1); 
    rec.addVoiceModel(6, 2, motor1_off_2, fram_num_motor1_off_2); //IN REGIONAL LANGUAGE
    rec.addVoiceModel(6, 3, motor1_off_3, fram_num_motor1_off_3); 

    rec.addVoiceModel(7, 0, motor2_off_0, fram_num_motor2_off_0); // TURN OFF MOTOR2    
    rec.addVoiceModel(7, 1, motor2_off_1, fram_num_motor2_off_1); 
    rec.addVoiceModel(7, 2, motor2_off_2, fram_num_motor2_off_2);//IN REGIONAL LANGUAGE 
    rec.addVoiceModel(7, 3, motor2_off_3, fram_num_motor2_off_3); 
    
    rec.addVoiceModel(8, 0, flytrap_off_0, fram_num_flytrap_off_0); // TURN OFF FLY TRAPS
    rec.addVoiceModel(8, 1, flytrap_off_1, fram_num_flytrap_off_1); 
    rec.addVoiceModel(8, 2, flytrap_off_2, fram_num_flytrap_off_2); //IN REGIONAL LANGUAGE
    rec.addVoiceModel(8, 3, flytrap_off_3, fram_num_flytrap_off_3); 

    rec.addVoiceModel(9, 0, rainsensor_off_0, fram_num_rainsensor_off_0); // TURN OFF RAIN SENSOR    
    rec.addVoiceModel(9, 1, rainsensor_off_1, fram_num_rainsensor_off_1); 
    rec.addVoiceModel(9, 2, rainsensor_off_2, fram_num_rainsensor_off_2); //IN REGIONAL LANGUAGE
    rec.addVoiceModel(9, 3, rainsensor_off_3, fram_num_rainsensor_off_3); 

    rec.addVoiceModel(10, 0, lights_off_0, fram_num_lights_off_0); // TURN OFF LIGHTS
    rec.addVoiceModel(10, 1, lights_off_1, fram_num_lights_off_1); 
    rec.addVoiceModel(10, 2, lights_off_2, fram_num_lights_off_2); //IN REGIONAL LANGUAGE
    rec.addVoiceModel(10, 3, lights_off_3, fram_num_lights_off_3); 

    rec.addVoiceModel(11, 0, camera_off_0, fram_num_camera_off_0); // TURN OFF CAMERA    
    rec.addVoiceModel(11, 1, camera_off_1, fram_num_camera_off_1); 
    rec.addVoiceModel(11, 2, camera_off_2, fram_num_camera_off_2); //IN REGIONAL LANGUAGE
    rec.addVoiceModel(11, 3, camera_off_3, fram_num_camera_off_3);   

    rec.addVoiceModel(12, 0, vcmd_off_0, fram_num_vcmd_off_0); // TURN OFF VOICE COMMANDS    
    rec.addVoiceModel(12, 1, vcmd_off_1, fram_num_vcmd_off_1); 
    rec.addVoiceModel(12, 2, vcmd_off_2, fram_num_vcmd_off_2); //IN REGIONAL LANGUAGE
    rec.addVoiceModel(12, 3, vcmd_off_3, fram_num_vcmd_off_3);   
    

}
void loop() {

    for(int i=0;i<9;i++){
        // Check for serial data to control the camera and other components
        if (Serial1.available()) {
            String CMDESP32 = Serial1.readStringUntil('\n');
            CMDESP32.trim();
            Serial.println(CMDESP32);
            
            // Control logic based on commands
            if (CMDESP32 == "CAM_ON" && camera_state == 0) {
                camera_state = 1;  // Set camera state to ON
                camera.begin();  // Initialize the camera only when turning it ON
                Serial.println("Camera started");
            } 
            else if (CMDESP32 == "CAM_OFF" && camera_state == 1) {
                camera_state = 0;  // Set camera state to OFF
                camera.end();  // Stop the camera when turning it OFF
                lcd.fillScreen(BG_COLOR);  // Clear LCD screen
                Serial.println("Camera stopped");
            } 
            else if (CMDESP32 == "M1_ON") {
                digitalWrite(M1_PIN, LOW);
            } 
            else if (CMDESP32 == "M1_OFF") {
                digitalWrite(M1_PIN, HIGH);
            } 
            else if (CMDESP32 == "M2_ON") {
                digitalWrite(M2_PIN, LOW);            
            } 
            else if (CMDESP32 == "M2_OFF") {
                digitalWrite(M2_PIN, HIGH);            
            } 
            else if (CMDESP32 == "LIGHTS_ON") {
                digitalWrite(LIGHT_PIN, LOW);
            } 
            else if (CMDESP32 == "LIGHTS_OFF") {
                digitalWrite(LIGHT_PIN, HIGH);
            } 
            else if (CMDESP32 == "RSENSOR_ON") {
                digitalWrite(RSENSOR_PIN, HIGH);
            } 
            else if (CMDESP32 == "RSENSOR_OFF") {
                digitalWrite(RSENSOR_PIN, LOW);
            } 
            else if (CMDESP32 == "FTRAP_ON") {
                digitalWrite(FTRAP_PIN, LOW);
            } 
            else if (CMDESP32 == "FTRAP_OFF") {
                digitalWrite(FTRAP_PIN, HIGH);
            }
            else if (CMDESP32 == "V1") {
                VCMDS = 1;
            }
            else if (CMDESP32 == "V0") {
                VCMDS = 0;
            }

        }
    }

    delay(1000);

    // Only run the camera if it's turned ON
    if (camera_state == 1) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastCaptureTime >= captureInterval) {
            lastCaptureTime = currentMillis;
            // Capture and send the image
            camera.run(true);
            uint8_t *img = camera.snapshot();
            uint16_t width = camera.width();
            uint16_t height = camera.height();
            if (img == nullptr) {
                Serial.println("Camera snap failed");
            } else {
                lcd.drawImage(0, 0, width, height, (uint16_t *)img); // Update the LCD display
                Serial2.write((uint8_t *)&width, sizeof(width));    // Send width
                Serial2.write((uint8_t *)&height, sizeof(height));  // Send height
                Serial2.write(img, width * height * 2);  // Send image data 
            }
        }
    }
    delay(10);  // Avoid task starvation

    // RAIN DETECTION AND ALERT SIGNAL GENERATION 
    if(analogRead(READ_PIN) < 350){
        digitalWrite(BUZZERPIN,HIGH);
    }
    else{
        digitalWrite(BUZZERPIN,LOW);
    }

    //VOICE COMMANDS 
    if(VCMDS == 1){
         // Start the voice recognition system
        int res = rec.recognize(); 
        Serial.printf("Recognized command: %d --> ", res);
        lcd.fillScreen(BG_COLOR);

        // Respond based on recognized result and sending acknowlede
        switch (res) {
            case 0: // TURN ON MOTOR1
                Serial3.println("M1_ON_ACK");
                digitalWrite(M1_PIN, LOW);
                printCenterOnLCD(lcd, "Motor1 On");
                break;
            case 1: // TURN ON MOTOR2
                Serial3.println("M2_ON_ACK");
                digitalWrite(M2_PIN, LOW);
                printCenterOnLCD(lcd, "Motor2 On");
                break;
            case 2: // TURN ON FLY TRAP
                Serial3.println("FTRAP_ON_ACK");
                digitalWrite(FTRAP_PIN, LOW);
                printCenterOnLCD(lcd, "Fly Trap On");
                break;
            case 3: // TURN ON RAIN SENSOR
                Serial3.println("RSENSOR_ON_ACK");
                digitalWrite(RSENSOR_PIN, LOW);
                printCenterOnLCD(lcd, "Rain Sensor On");
                break;
            case 4: // TURN ON LIGHTS
                Serial3.println("LIGHTS_ON_ACK");
                digitalWrite(LIGHT_PIN, LOW);
                printCenterOnLCD(lcd, "Lights On");
                break;
            case 5: // TURN ON CAMERA
                Serial3.println("CAM_ON_ACK");
                printCenterOnLCD(lcd, "Camera On");
                break;
            case 6: // TURN OFF MOTOR1
                Serial3.println("M1_OFF_ACK");
                digitalWrite(M1_PIN, HIGH);
                printCenterOnLCD(lcd, "Motor1 Off");
                break;
            case 7: // TURN OFF MOTOR2
                Serial3.println("M2_OFF_ACK");
                digitalWrite(M2_PIN, HIGH);
                printCenterOnLCD(lcd, "Motor2 Off");
                break;
            case 8: // TURN OFF FLY TRAP
                Serial3.println("FTRAP_OFF_ACK");
                digitalWrite(FTRAP_PIN, HIGH);
                printCenterOnLCD(lcd, "Fly Trap Off");
                break;
            case 9: // TURN OFF RAIN SENSOR
                Serial3.println("RSENSOR_OFF_ACK");
                digitalWrite(RSENSOR_PIN, HIGH);
                printCenterOnLCD(lcd, "Rain Sensor Off");
                break;
            case 10: // TURN OFF LIGHTS
                Serial3.println("LIGHTS_OFF_ACK");
                digitalWrite(LIGHT_PIN, HIGH);
                printCenterOnLCD(lcd, "Lights Off");
                break;
            case 11: // TURN OFF LIGHTS
                Serial3.println("LIGHTS_OFF_ACK");
                digitalWrite(LIGHT_PIN, HIGH);
                printCenterOnLCD(lcd, "Lights Off");
                break;
            case 12: // TURN OFF VOICE COMMANDS
                Serial3.println("VCMD_OFF_ACK");
                printCenterOnLCD(lcd, "Voice Commands Off");
                break;
            default:
                Serial1.println("Command not recognized");
                printCenterOnLCD(lcd, "Unrecognized Command");
                break;
        }
    }
    delay(1000);

}

