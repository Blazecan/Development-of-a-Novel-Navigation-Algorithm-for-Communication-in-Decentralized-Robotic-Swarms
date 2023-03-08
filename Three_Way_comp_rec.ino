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


const char FORWARD = '1';
const char REVERSE = '2';
const char NOTHING = '0';
int rssi = 60;


// Variable to store if sending data was successful
String success;


typedef struct computer_message {
  int rssi;
  int direct;
} computer_message;

computer_message compIncoming;
int rssiIncoming;
int directIncoming;


//// Callback when data is sent
//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
////  Serial.print("\r\nLast Packet Send Status:\t");
////  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//  if (status ==0){
//    success = "Delivery Success :)";
//  }
//  else{
//    success = "Delivery Fail :(";
//  }
//}


// Callback when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&compIncoming, incomingData, sizeof(compIncoming));
  
  Serial.print("Bytes received: ");
  Serial.println(len);
  rssiIncoming = compIncoming.rssi;
  directIncoming = compIncoming.direct;

  Serial.println("Data Recieved");
  Serial.println(directIncoming);
  Serial.print("RSSI");
  Serial.println(rssiIncoming);
}
//
//typedef struct {
//  unsigned frame_ctrl: 16;
//  unsigned duration_id: 16;
//  uint8_t addr1[6]; /* receiver address */
//  uint8_t addr2[6]; /* sender address */
//  uint8_t addr3[6]; /* filtering address */
//  unsigned sequence_ctrl: 16;
//  uint8_t addr4[6]; /* optional */
//} wifi_ieee80211_mac_hdr_t;
//
//typedef struct {
//  wifi_ieee80211_mac_hdr_t hdr;
//  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
//} wifi_ieee80211_packet_t;
//


//Callback when packets are recieved to have rssi value
//void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
//
//    // All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
//    if (type != WIFI_PKT_MGMT)
//        return;
//
//    static const uint8_t ACTION_SUBTYPE = 0xd0;
//    static const uint8_t ESPRESSIF_OUI[] = {0x18, 0xfe, 0x34};
//
//    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
//    const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
//    const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;
//    
//    rssi = ppkt->rx_ctrl.rssi;
//    rssi = -rssi;
//}

 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial.println("Starting");
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
//    Serial.println("Error initializing ESP-NOW");
    return;
  }

//  esp_wifi_set_promiscuous(true);
//  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  
  // Register peer
//  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
//  peerInfo.channel = 0;  
//  peerInfo.encrypt = false;
//  memcpy(compInfo.peer_addr, broadcastAddress2, 6);
//  compInfo.channel = 0;
//  compInfo.encrypt = false;
  
  // Add peer        
//  if (esp_now_add_peer(&peerInfo) != ESP_OK){
////    Serial.println("Failed to add peer");
//    return;
//  }
//  esp_now_add_peer(&compInfo);
//  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}



void loop() {



//  // Send message via ESP-NOW
//  outgoing.found = false;
//  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
  


  delay(1000);
}
