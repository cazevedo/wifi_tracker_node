/*
API ref: https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/wifi/esp_now.html
*/

#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFi.h>
#include <string>
#include <inttypes.h>

#include "main.h"
#include "crc.h"

// Global
esp_now_peer_info_t gateway_info;
Message packet;
uint8_t GatewayMacAddress[] = {0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};

#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

// save variable in the RTC memmory such that it doesn't get lost upon sleep
// RTC_DATA_ATTR

// esp_err_t do_firmware_upgrade()
// {
//     esp_http_client_config_t config = {
//         .url = CONFIG_FIRMWARE_UPGRADE_URL,
//         .cert_pem = (char *)server_cert_pem_start,
//     }; 
//     esp_err_t ret = esp_https_ota(&config);
//     if (ret == ESP_OK) {
//         esp_restart();
//     } else {
//         return ESP_FAIL;
//     }
//     return ESP_OK;
// }

// Init ESP Now with fallback
void InitESPNow()
{
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
    Serial.println("ESPNow Init Success");
    
  else
  {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a count and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

void unpair()
{
  const uint8_t *peer_addr = gateway_info.peer_addr;
  esp_err_t delStatus = esp_now_del_peer(peer_addr);
  Serial.print("Gateway Delete Status: ");
  if (delStatus == ESP_OK)
    Serial.println("Success");
  else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT)
    Serial.println("ESPNOW Not Init");
  else if (delStatus == ESP_ERR_ESPNOW_ARG)
    Serial.println("Invalid Argument");
  else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND)
    Serial.println("Peer not found.");
  else
    Serial.println("Not sure what happened");
}

bool pair()
{
  for (int k = 0; k < 6; ++k )
    gateway_info.peer_addr[k] = (uint8_t) GatewayMacAddress[k]; // predefined MAC address
  gateway_info.channel = CHANNEL; // pick a channel
  gateway_info.encrypt = 0; // no encryption

  const esp_now_peer_info_t *peer = &gateway_info;
  // pair
  esp_err_t peerStatus = esp_now_add_peer(peer);

  switch(peerStatus)
  {
    case ESP_OK:
      Serial.println("Pair success");
      return true;
    case ESP_ERR_ESPNOW_NOT_INIT:
      Serial.println("ESPNOW Not Init");
      return false;
    case ESP_ERR_ESPNOW_ARG:
      Serial.println("Invalid Argument");
      return false;
    case ESP_ERR_ESPNOW_FULL:
      Serial.println("Peer list full");
      return false;
    case ESP_ERR_ESPNOW_NO_MEM:
      Serial.println("Out of memory");
      return false;
    case ESP_ERR_ESPNOW_EXIST:
      Serial.println("Peer Exists");
      return true;
    default:
      Serial.println("Not sure what happened");
      return false;
  }
}

// send data
void sendData(const void *data, uint16_t siz)
{
  const uint8_t *data_ptr = (const uint8_t*) data;

  // const uint8_t *peer_addr = gateway.peer_addr;
  const uint8_t *peer_addr = &GatewayMacAddress[0];
  // Serial.print("Sending: "); Serial.println(data);
  // esp_err_t result = esp_now_send(peer_addr, &data, sizeof(data));
  esp_err_t result = esp_now_send(peer_addr, data_ptr, siz);

  Serial.print("Send Status: ");
  if (result == ESP_OK)
    Serial.println("Success");
  else if (result == ESP_ERR_ESPNOW_NOT_INIT)
    Serial.println("ESPNOW not Init.");
  else if (result == ESP_ERR_ESPNOW_ARG)
    Serial.println("Invalid Argument");
  else if (result == ESP_ERR_ESPNOW_INTERNAL)
    Serial.println("Internal Error");
  else if (result == ESP_ERR_ESPNOW_NO_MEM)
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
    Serial.println("Peer not found.");
  else
    Serial.println("Not sure what happened");
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // TODO: add counter ratio, success/total
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

uint64_t get_time() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

uint8_t CRC8(const uint8_t *data, int len)
{
   uint8_t crc = 0;

   for (uint8_t i = 0; i < len; i++)
       crc = CRC8tab[crc ^ data[i]];

   return crc;
}

void setup()
{
  // disable wakeup source
  // esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);

  delay(2000);
  Serial.begin(115200);

  packet.seq = 0;

  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow Node");

  // This is the mac address of the Node in Station Mode
  Serial.print("Node MAC: ");
  Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  pair();
}

void loop()
{
  packet.seq++;
  packet.timestamp = get_time();
  packet.crc = 2;

  // char timestampStr[8];
  // snprintf(timestampStr, sizeof(timestampStr), "timestamp : %"PRIu64, packet.timestamp);
  Serial.print("Seq : ");
  Serial.println(packet.seq);
  Serial.print("Timestamp : ");
  Serial.println(packet.timestamp);
  Serial.print("CRC : ");
  Serial.println(packet.crc);

  sendData(&packet, sizeof(packet));

  // TODO: Save packet to the eeprom and sleep
  delay(3000);

  // disable wifi
  // esp_wifi_stop();
  // // set pull down on 
  // gpio_pulldown_en(GPIO_NUM_21);
  // // enable wake up gpio
  // esp_sleep_enable_ext0_wakeup(GPIO_NUM_21,1); //1 = High, 0 = Low

  // // go to deep sleep
  // esp_deep_sleep_start();
}