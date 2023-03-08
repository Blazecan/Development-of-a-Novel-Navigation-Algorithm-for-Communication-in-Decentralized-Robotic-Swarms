/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include "esp_wifi.h"
#include <esp_now.h>
#include <WiFi.h>


// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xEC, 0x62, 0x60, 0x75, 0x6B, 0x10};
uint8_t broadcastAddress1[] = {0xEC, 0x62, 0x60, 0x5C, 0x59, 0xD4};
uint8_t broadcastAddress2[] = {0xCC, 0xDB, 0xA7, 0x02, 0xE9, 0x24};
// Define variables to store BME280 readings to be sent
boolean outgoingFound;

const char FORWARD = '1';
const char REVERSE = '2';
const char NOTHING = '0';
const char TURNRIGHT = '3';
const char TURNLEFT = '4';


int rssi = 60;
int rssi1 = 60;
// Define variables to store incoming readings
boolean incomingFound;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
  boolean found;
} struct_message;

typedef struct computer_message {
  int rssi;
  int rssi1;
  int direct;
} computer_message;

computer_message compGoing;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message outgoing;

// Create a struct_message to hold incoming sensor readings
struct_message incoming;

esp_now_peer_info_t peerInfo;
esp_now_peer_info_t compInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  
//  Serial.print("Bytes received: ");
//  Serial.println(len);
  incomingFound = incoming.found;
//
//  Serial.println("Data Recieved");
//  Serial.println(incomingFound);
//  Serial.print("RSSI");
//  Serial.println(rssi);
}

//Credit to Marcelo for the promiscious wifi setup
typedef struct {
  unsigned frame_ctrl: 16;
  unsigned duration_id: 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl: 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;



//Callback when packets are recieved to have rssi value
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {

    // All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
    if (type != WIFI_PKT_MGMT)
        return;

    static const uint8_t ACTION_SUBTYPE = 0xd0;
    static const uint8_t ESPRESSIF_OUI[] = {0x18, 0xfe, 0x34};

    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
    const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
    const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

    if(ppkt->mac == broadcastAddress){
      rssi = ppkt->rx_ctrl.rssi;
      rssi = -rssi;
    }
    if(ppkt->mac == broadcastAddress1) {
      rssi1 = ppkt->rx_ctrl.rssi;
      rssi1 = -rssi1;
    }
    
}

 
void setup() {
  // Init Serial Monitor
//  Serial.begin(115200);
  Serial2.begin(9600);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
//    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  memcpy(compInfo.peer_addr, broadcastAddress2, 6);
  compInfo.channel = 0;
  compInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
//    Serial.println("Failed to add peer");
    return;
  }
  esp_now_add_peer(&compInfo);
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}



void loop() {



  // Send message via ESP-NOW
  outgoing.found = false;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
  
//  if (result == ESP_OK) {
//    Serial.println("Sent with success");
//  }
//  else {
//    Serial.println("Error sending the data");
//  }
  compGoing.rssi = rssi;
  if(rssi > 75){
    Serial2.println(FORWARD);
    compGoing.direct = 1;
  }
  else if(rssi < 50){
    Serial2.println(REVERSE);
    compGoing.direct = 2;
  }
  else if (rssi1 > 75){
    Serial2.println(TURNRIGHT);
    compGoing.direct = 3;
  }
  else if (rssi1 < 50){
    Serial2.println(TURNLEFT);
    compGoing.direct = 4;
  }
  else{
    Serial2.println(NOTHING);
    compGoing.direct = 0;
  }


  esp_now_send(broadcastAddress2, (uint8_t *) &compGoing, sizeof(compGoing));


  delay(1000);
}
