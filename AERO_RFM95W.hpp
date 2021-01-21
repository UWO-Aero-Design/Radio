#pragma once

#include "src/Rfm95w/RH_RF95.h"
#include "src/message/message.pb.h"
#include "src/message/pb_encode.h"
#include "src/message/pb_decode.h"

/**
 * @brief RFM95W base class
 * @details Used as a base class for RFM95WServer and RFM95WClient-> Can also be used to test RFM95W initialization
 */
class RFM95W {
public:
  
  /**
   * @brief Construct a new RFM95W object
   * 
   * @param cs_pin Radio Chip Select Pin
   * @param rst_pin Radio Reset Pin
   * @param int_pin Radio Interrupt Pin
   */
  RFM95W(int cs_pin, int rst_pin, int int_pin) {
    this->radio = new RH_RF95(cs_pin, int_pin);
    this->m_rst_pin = rst_pin;
  } 

  /**
   * @brief Initialize the RFM95W radio
   * 
   * @return true if the radio successfully initialized
   * @return false if the radio failed to initialize
   */
  bool init() {
    // Set the reset pin
    pinMode(m_rst_pin, OUTPUT);
    digitalWrite(m_rst_pin, LOW);
    delay(10);
    digitalWrite(m_rst_pin, HIGH);
    delay(10);

    // Initialize the radio
    if (!radio->init()) {
      return false;
    }

    // Set radio frequency
    if (!radio->setFrequency(RADIO_FREQ)) {
      return false;
    }

    // Set radio power
    radio->setTxPower(RADIO_POWER, false);

    return true;
  }

  //Radio Location
  Message_Location location;

protected:
  // Radio object
  RH_RF95 *radio;

  // Radio reset pin
  unsigned int m_rst_pin;

private:
  static constexpr float RADIO_FREQ = 905.0f;
  // Max power
  static constexpr int RADIO_POWER = 23;
    
};

/**
 * @brief RFM95W object used as a client
 * @details A client will sent messages to all servers and wait for their responses
 */
class RFM95WClient : public RFM95W {
public:

  /**
   * @brief Construct a new RFM95W Client object
   * 
   * @param cs_pin Radio Chip Select Pin
   * @param rst_pin Radio Reset Pin
   * @param int_pin Radio Interrupt Pin
   */
  RFM95WClient(int cs_pin, int rst_pin, int int_pin): RFM95W(cs_pin, rst_pin, int_pin) {} 
  
  /**
   * @brief Set the timeout
   * 
   * @param int new timeout value
   */
  void setTimeout(const unsigned int& timeout) {
    m_timeout = timeout;
  }

  /**
   * @brief Send a message from the client and return the response from the appropriate server
   * 
   * @param message Message to send from the client to all active devices
   * @return Response from server that recevied the message
   */
  bool send_message(Message message, Message *response) {
    uint8_t sendbuf[Message_size];
    size_t msglen = sizeof(sendbuf);
    uint8_t recbuf[Message_size];
    uint8_t resplen = sizeof(recbuf);
    *response = Message_init_zero;

    pb_ostream_t ostream = pb_ostream_from_buffer(sendbuf, sizeof(sendbuf));
    bool status = pb_encode(&ostream, Message_fields, &message);
    msglen = ostream.bytes_written;

    if(!status) {
      Serial.print("Error encoding: "); Serial.println(PB_GET_ERROR(&ostream));
      return false;
    }

    // Send packet
    status = radio->send(sendbuf, msglen);
    radio->waitPacketSent();

    if(!status) {
      Serial.println("Send fail");
      return false;
    }

    if (radio->waitAvailableTimeout(m_timeout)) { 
      if (radio->recv(recbuf, &resplen)) {

        pb_istream_t istream = pb_istream_from_buffer(recbuf, (size_t)resplen);
        status = pb_decode(&istream, Message_fields, response);

        if(!status) {
          Serial.print("Error decoding: "); Serial.println(PB_GET_ERROR(&istream));
          return false;
        }
        if (response->sender == message.recipient && response->recipient == location) {
          return true;
        }
        else {
          Serial.println("Not acked properly");
          return false;
        }
      } 
      else {
        Serial.println("Receive response failed");
        return false;
      }
    } 
    else {
      Serial.println("Timeout");
      return false;
    }
  }

  int16_t rssi() {
    return radio->lastRssi();
  }

private:
  // Timeout that defines how long the client will wait for a valid response from the servers
  static constexpr unsigned int DEFAULT_TIMEOUT = 1000;
  unsigned int m_timeout = DEFAULT_TIMEOUT;

};

/**
 * @brief RFM95W object used as a server
 * @details A server will wait for messages from a client and respond accordingly
 */
class RFM95WServer : public RFM95W {
public:

  /**
   * @brief Construct a new RFM95W Server object
   * 
   * @param cs_pin Radio Chip Select Pin
   * @param rst_pin Radio Reset Pin
   * @param int_pin Radio Interrupt Pin
   */
  RFM95WServer(int cs_pin, int rst_pin, int int_pin): RFM95W(cs_pin, rst_pin, int_pin) {} 

  bool message_available(void) {
    return radio->available();
  }
  
  bool receive(Message *message) {
    uint8_t buf[Message_size];
    uint8_t len = sizeof(buf);
    uint8_t sendbuf[Message_size];
    
    if (radio->available()) {
      if (radio->recv(buf, &len)) {
        
        *message = Message_init_zero;
        Message response = Message_init_zero;
        
        pb_istream_t istream = pb_istream_from_buffer(buf, (size_t)len);
        bool status = pb_decode(&istream, Message_fields, message);
        if(!status) {
          Serial.print("Error decoding: "); Serial.println(PB_GET_ERROR(&istream));
          return false;
        }

        if(message->recipient == location) {
          response = Message_init_zero;
          response.sender = message->recipient;
          response.recipient = message->sender;
          response.packet_number = message->packet_number;
          response.time = (int64_t)millis();
          
          pb_ostream_t ostream = pb_ostream_from_buffer(sendbuf, sizeof(sendbuf));
          bool status = pb_encode(&ostream, Message_fields, &response);
          size_t resplen = ostream.bytes_written;
    
          if(!status) {
            Serial.print("Error encoding response: "); Serial.println(PB_GET_ERROR(&ostream));
            return false;
          }
    
          if(respond(sendbuf, resplen)) {
            return true;
          }
          else {
            Serial.println("Response send failed");
            return false;
          }
        }
    
        else {
          Serial.println("message not for me");
          return false;
        }
      } 
      else {
        Serial.println("recv failed");
        return false;
      }
    }
    else {
      Serial.println("no message available");
      return false;
    }
  }

  bool respond(uint8_t *buffer, uint8_t message_length) {

    // Send a reply
    bool status = radio->send(buffer, message_length);
    radio->waitPacketSent();
    return status;
  }

  private:
};
