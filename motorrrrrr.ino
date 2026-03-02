#include <SPI.h>
#include "mcp2518fd_can.h"
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>

/* LTC Includes */
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6811.h"
/* Custom Includes */
#include "pinModes.h"
#include <mcp2518fd_can.h>
float getMotorRPM() const { return motorRPM; }
void recieve() {
  bool chargerStatus;
  unsigned char len = 0;
  uint32_t buf2[8];
  buf2[2] << 8;
  CAN.sendMsgBuf(0x1806E5F4, 1, 8, BMSA);
  CAN.readMsgBuf(&len, buf);
  


void canSend() {
  uint8_t canMsg[8]; // 8-byte ka dabba
  failedMessages = 0;
  canMsg[0] = lowByte(motorRPM);  
  canMsg[1] = highByte(motorRPM); 

  
  for(int i = 2; i < 8; i++) {
    canMsg[i] = 0;
  }

  if (CAN.sendMsgBuf(can_ids[canIDindex], 0, 8, canMsg) != CAN_OK) {
    failedMessages++;
  }

  // Status check
  if (failedMessages == 0) {
    canSend_status = true;
  } else {
    canSend_status = false;
  }
}













void canSend() {
  uint8_t canMsg[8];
  uint8_t canMsg_index = 0;
  uint8_t canIDindex = 0;
  uint16_t index = 0;  
  failedMessages = 0;

  for (uint8_t currentIc = 0; currentIc < TOTAL_IC; currentIc++) {
    for (uint8_t cell = 0; cell < TOTAL_CELL; cell++) {
      allData[index++] = BMS_IC[currentIc].cells.c_codes[cell];
    }

    for (uint8_t temp = 0; temp < TEMPS; temp++) {
      allData[index++] = BMS_IC[currentIc].aux.a_codes[temp];
    }
  }

  uint16_t currentReading = analogRead(CSOUT);

  allData[index++] = currentReading;
  allData[index++] = vsHVin;
  allData[index++] = vsBat;
  allData[index++] = faultOutStatus;

  for (uint16_t i = 0; i < 184; i++) {
    canMsg[canMsg_index++] = lowByte(allData[i]);
    canMsg[canMsg_index++] = highByte(allData[i]);

    if (canMsg_index == 8) {
      if (canIDindex < 46) {
        if (CAN.sendMsgBuf(can_ids[canIDindex], 0, 8, canMsg) != CAN_OK) {
          failedMessages++;
        }
        canIDindex++;
      }
      canMsg_index = 0;
    }
  }
  if (failedMessages == 0) {
    canSend_status = true;
  } else {
    canSend_status = false;
  }
}


void handleBamocarMessage(const CAN_FRAME &frame) {
  if (frame.length < 1) return;
  
  uint8_t regID = frame.data.bytes[0];
  
  // Basic parsing of common registers
  switch (regID) {
    case 0x30: { // N_ACTUAL (Speed)
      if (frame.length >= 3) {
        int16_t raw = frame.data.bytes[1] | (frame.data.bytes[2] << 8);
        // Convert to actual RPM based on N_MAX (might need lookup)
        motorRPM = raw * 0.5f; // Example scaling, adjust as needed
      }
      break;
    }
    
    case 0x90: { // TORQUE
      if (frame.length >= 3) {
        int16_t raw = frame.data.bytes[1] | (frame.data.bytes[2] << 8);
        motorTorque = raw * 0.01f; // Example scaling
      }
      break;
    }
    
    case 0x49: { // TEMP_MOTOR
      if (frame.length >= 3) {
        uint16_t raw = frame.data.bytes[1] | (frame.data.bytes[2] << 8);
        motorTemp = raw * 0.1f; // Example: temp in degrees C
      }
      break;
    }
    
    case 0x40: { // STATUS
      if (frame.length >= 5) {
        uint32_t status = frame.data.bytes[1] | 
                         (frame.data.bytes[2] << 8) | 
                         (frame.data.bytes[3] << 16) | 
                         (frame.data.bytes[4] << 24);
        
        // Example status bit check
        systemError = (status & 0x01) != 0;
        
        if (debugLevel >= 2 && systemError) {
          Serial.print("Bamocar ERROR status: 0x");
          Serial.println(status, HEX);
        }
      }
      break;
    }
  }
}
bool requestBamocarData(uint8_t regID, uint8_t interval) {
  uint8_t data[3];
  data[0] = 0x3D;     // REG_REQUEST
  data[1] = regID;    // Register to request
  data[2] = interval; // Update interval
  
  return canSend();
  recieve();

}
void update() {
  // Check for incoming messages
  while (Can0.available() > 0) {
    CAN_FRAME frame;
    if (Can0.read(frame)) {
      // Print all received messages if debug enabled
      
        Serial.print("CAN RX: ID=0x");
        Serial.print(frame.id, HEX);
        Serial.print(" Len=");
        Serial.print(frame.length);
        Serial.print(" Data=[");
        for (int i = 0; i < frame.length; i++) {
          if (i > 0) Serial.print(",");
          Serial.print("0x");
          if (frame.data.bytes[i] < 16) Serial.print("0");
          Serial.print(frame.data.bytes[i], HEX);
        }
        Serial.println("]");
      
      // Route message based on ID
      if (frame.id == BAMOCAR_TX_ID) {
        handleBamocarMessage(frame);
      } 
      else if (frame.id == BMS_ID_PACK_INFO || frame.id == BMS_ID_LIMITS_TEMP) {
        handleBMSMessage(frame);
        // Update timestamp for comms tracking
        lastBmsMessageTime = millis();
      }
      // Add other device IDs as needed
    }
  }
  
  // Print status periodically if enabled
  if (debugLevel >= 2) {
    unsigned long now = millis();
    if (now - lastStatusPrint > 5000) { // Every 5 seconds
      Serial.println("--- CAN Status ---");
      Serial.print("Motor: ");
      Serial.print(motorRPM);
      Serial.print(" RPM, ");
      Serial.print(motorTorque);
      Serial.print(" Nm, ");
      Serial.print(motorTemp);
      Serial.println("°C");
      
      Serial.print("Battery: ");
      Serial.print(packVoltage);
      Serial.print("V, ");
      Serial.print(packCurrent);
      Serial.print("A, SOC ");
      Serial.print(packSOC);
      Serial.print("%, DCL ");
      Serial.print(packDCL);
      Serial.println("A");
      
      Serial.print("BMS Temps: High ");
      Serial.print(highTemp);
      Serial.print("°C, Low ");
      Serial.print(lowTemp);
      Serial.print("°C, Relay State 0x");
      Serial.println(relayState, HEX);
      
      lastStatusPrint = now;
    }
  }
}

void initializeMCP() {
  Serial.println(F("(CAN): Initializing..."));

  setMode(CAN_CLASSIC_MODE);

  while (CAN_OK != CAN.begin(CAN20_500KBPS)) {  // init can bus : baudrate = 500k
    Serial.println(F("(CAN): Initialization failed."));
  }
  Serial.println(F("(CAN): Initialized."));
}
// 2. CREATE THE OBJECT (This fixes the "not declared" error)

// 1. Fixed: Avoid double declaration. Stick to float for precision.
float motorRPM = 0; 

// Note: If requestBamocarData and getMotorRPM are part of a class, 
// they should be called through the object (e.g., can.getMotorRPM).
mcp2518fd CAN(CCS);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);  // Wait max 2 seconds
  Serial.println("\n--- UCD FS EV Controller Starting ---");
  initializeMCP();
  // Initialize CAN
  // Assuming 'can' is your CAN object/handler
  
  // Request initial data from motor controller
  Serial.println("Requesting initial motor data...");
  
  // 0x30 is the RegID for Speed (RPM), 0x64 is the interval in ms
  requestBamocarData(0x30, 0x64);  
  
  Serial.println("--- Setup Complete ---");
}

void loop() {
  // 2. Fixed: Braces were misplaced in your snippet.
  update();
  
  // Update the global motorRPM variable with data from the CAN object
  motorRPM = getMotorRPM();
  Serial.println(motorRPM);

  // Optional: Add a small delay or timer to prevent flooding Serial
  // Serial.println(motorRPM);
}
