////////////////////// PLANE/DRONE CODE //////////////////////////////////

#include "AERO_RFM95W.hpp"
#include "Wire.h"

const int DEFAULT_BAUD = 112500;
const int LED_PIN = 13;
const int CS_PIN = 10;
const int RST_PIN = 34;
const int INT_PIN = 31;

RFM95WServer radio = RFM95WServer(CS_PIN, RST_PIN, INT_PIN);

bool radio_success;

Message message;

void setup() {
  
  Serial.begin(DEFAULT_BAUD);
  
  pinMode(LED_PIN, OUTPUT);
  
  bool is_success = true;

  if (radio.init()) {
    radio.location = Message_Location::Message_Location_DRONE;
    Serial.println("Radio online.");    
  }
  else {
    Serial.println("Error connecting to radio.");
    is_success = false;
  }

  if(is_success) {
    Serial.println("\nSystem successfully started.\n\n");
  }
  else {
    Serial.println("\nSystem started with errors.\n\n");
  }
}

void loop() {  
  digitalWrite(LED_PIN, LOW);
  if(radio.message_available()) {
    if(radio.receive(&message)) {
      radio_success = true;
      digitalWrite(LED_PIN, HIGH);

      Serial.print("Message received from ");
      if(message.sender == Message_Location::Message_Location_DRONE) Serial.print("DRONE");
      else if(message.sender == Message_Location::Message_Location_GROUND_STATION) Serial.print("GROUND");
      Serial.print(" (Packet #: "); Serial.print(message.packet_number);
      Serial.println(")");
      Serial.printf(" (Time: "); Serial.print((long)message.time);
      Serial.println(")");

    }
    
    else {
      radio_success = false;
      Serial.println("recv failed");
    }
  }
}

///////////////////////////// GROUNDSTATION CODE ////////////////////////////////////////////

//#include "AERO_RFM95W.hpp"
//
//const int LED_PIN = LED_BUILTIN;
//const int LED_PIN = 13;
//const int CS_PIN = 10;
//const int RST_PIN = 34;
//const int INT_PIN = 31;
//
//Message message = Message_init_zero;
//Message response = Message_init_zero;
//int32_t packet_num = 0;
//
//RFM95WClient radio = RFM95WClient( CS_PIN, RST_PIN, INT_PIN );
//
//void setup() {
//  Serial.begin(112500);
//  pinMode(LED_PIN, OUTPUT);
//
//  if(!radio.init()) {
//    radio.location = Message_Location::Message_Location_GROUND_STATION;
//    Serial.println("Radio init failed");
//    while(1);
//  }
//}
//
//void loop() {
//  Serial.println("Sending to rf95_server");
//  digitalWrite(LED_PIN, HIGH);
// 
//
//  message = Message_init_zero;
//  message.sender = Message_Location::Message_Location_GROUND_STATION;
//  message.recipient = Message_Location::Message_Location_DRONE;
//  message.packet_number = packet_num;
//  message.time = (int64_t)millis();
//
//  if (radio.send_message(message, &response)) {
//    digitalWrite(LED_PIN, LOW);
//    Serial.print("Response received from ");
//    if(response.sender == Message_Location::Message_Location_DRONE) Serial.print("DRONE");
//    else if(response.sender == Message_Location::Message_Location_GROUND_STATION) Serial.print("GROUND");
//    Serial.print(" (Packet #: "); Serial.print(response.packet_number);
//    Serial.println(")");
//    Serial.printf(" (Time: "); Serial.print((long)response.time);
//    Serial.println(")");
//  }
//  else {
//    Serial.println("No reply, is rf95_server running?");
//  }
//  delay(500);
//  packet_num++;
//}
