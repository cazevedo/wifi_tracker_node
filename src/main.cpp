/*
API ref: https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/wifi/esp_now.html
*/

#include <esp_now.h>
#include <WiFi.h>
#include <string>
#include <inttypes.h>

#include "main.h"
#include "crc.h"

// Global
esp_now_peer_info_t gateway;
Message packet;

#define CHANNEL 3
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0


// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// Scan for devices in AP mode
bool ScanForAP()
{
  int8_t n_scanResults = WiFi.scanNetworks();
  // reset on each scan
  bool gateway_found = false;
  // memset(&gateway, 0, sizeof(gateway));

  Serial.println("");
  if (n_scanResults == 0)
    Serial.println("No WiFi devices in AP Mode found");

  else
  {
    Serial.print("Found ");
    Serial.print(n_scanResults);
    Serial.println(" devices ");
    for (int i = 0; i < n_scanResults; ++i)
    {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Gateway`
      if (SSID.indexOf("Gateway") == 0)
      {
        // SSID of interest
        Serial.println("Found the gateway");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Gateway
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x%c",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) )
          for (int k = 0; k < 6; ++k )
            gateway.peer_addr[k] = (uint8_t) mac[k];

        gateway.channel = CHANNEL; // pick a channel
        gateway.encrypt = 0; // no encryption

        // clean up ram
        WiFi.scanDelete();

        // As soon as the gateway is found stop searching
        return true;
      }
    }
  }

  return false;
}

void deletePeer()
{
  const esp_now_peer_info_t *peer = &gateway;
  const uint8_t *peer_addr = gateway.peer_addr;
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

// Check if the gateway is already paired with the node.
// If not, pair the gateway with the node.
bool pair()
{
  if (DELETEBEFOREPAIR)
    deletePeer();

  Serial.print("Gateway Status: ");
  const esp_now_peer_info_t *peer = &gateway;
  const uint8_t *peer_addr = gateway.peer_addr;
  // check if the peer exists
  if ( esp_now_is_peer_exist(peer_addr) )
  {
    // already paired.
    Serial.println("Already Paired");
    return true;
  }
  else
  {
    // not paired, attempt pair
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
}

// send data
void sendData(const void *data, uint16_t siz)
{
  const uint8_t *data_ptr = (const uint8_t*) data;

  const uint8_t *peer_addr = gateway.peer_addr;
  // Serial.print("Sending: "); Serial.println(data);
  // esp_err_t result = esp_now_send(peer_addr, &data, sizeof(data));
  esp_err_t result = esp_now_send(peer_addr, data_ptr, siz);

  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
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
}

void loop()
{
  // In the loop we scan for the gateway
  if (ScanForAP())
  {
    // Add gateway as peer if it has not been added already
    if (pair())
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

      // Serial.print((const char *) timestampStr);
      // const uint8_t *data_ptr = (const uint8_t*) packet.timestamp;
      // packet.crc = CRC8(data_ptr, sizeof(packet.timestamp));

      sendData(&packet, sizeof(packet)); // Tell the gateway this node is alive
    }
    else
      Serial.println("Gateway pair failed!");
  }

  // TODO: Save packet to the eeprom and sleep (turn everything off) for 5 mnts
  delay(3000);
}