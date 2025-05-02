#include <Arduino.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Load Wi-Fi networking
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <WebSerial.h>
#include <time.h>

static AsyncWebServer server(80);

// Load FastLED
#include <FastLED.h>

// Webserver and file system
#define SPIFFS LittleFS
#include <LittleFS.h>
#include <FS.h>
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson?utm_source=platformio&utm_medium=piohome

// my wifi secrets
#include "secrets.h"

// esp32 native TWAI / CAN library
#include "driver/twai.h"
TaskHandle_t canbus_task_handle = NULL; // task handle for canbus task

// my canbus stuff
#include "canbus_msg.h"
#include "canbus_flags.h"

#define CAN_SELF_MSG 0

// Interval:
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 1000

// time stuff
#define NTP_SERVER     "us.pool.ntp.org"
#define UTC_OFFSET     0
#define UTC_OFFSET_DST 0


// organize nodes 
struct remoteNode {
  // 32-bit node id number
  uint8_t   nodeID[4]      = {0,0,0,0}; 
  // 11-bit can bus message id and node type
  uint16_t  nodeType       = 0;
  // node feature mask storaege (optional)
  uint8_t   featureMask[2] = {0,0};
  // storage for any sub modules
  uint16_t  subModules[4]  = {0,0,0,0}; 
  // sub module count for each sub module
  uint8_t   subModCnt[4]   = {0,0,0,0};
  // total sub module count
  uint8_t   moduleCnt      = 0; 
  // last time message received from node 
  uint32_t  lastSeen       = 0;
  // first time message received from node 
  uint32_t  firstSeen       = 0;
};

struct remoteNode nodeList[8]; // list of remote nodes
volatile uint8_t  nodeListPtr   = 0; // node list pointer
const uint8_t     nodeListMax   = 8; // max number of nodes in the list
const uint8_t     modListMax    = 4; // max number of modules per node

static bool driver_installed = false;

unsigned long previousMillis = 0;  // will store last time a message was send

static const char *TAG = "can_control";

volatile uint8_t  nodeSwitchState[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // switch state
volatile uint8_t  nodeSwitchMode[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // switch mode

volatile uint8_t  testState[3] = {1, 0, 2}; // test state
volatile uint8_t  testPtr = 0; // test pointer
volatile uint8_t  testRestart = true; // set flag to true to restart test message squence

volatile uint16_t introMsg[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // intro messages
volatile uint8_t  introMsgPtr = 0; // intro message pointer
volatile uint8_t  introMsgData[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // intro message data
volatile uint8_t  introMsgCnt = 0; // intro message count

const char* AP_SSID  = "m5stamp-pico";
const char* hostname = "m5stamp-pico";
#define CAN_MY_TYPE IFACE_TOUCHSCREEN_TYPE_A
const uint8_t* myNodeFeatureMask = FEATURE_IFACE_TOUCHSCREEN_TYPE_A; // node feature mask
const uint16_t myNodeIntro = REQ_INTERFACES; // intro request for my node type
const uint8_t otherNodeID[] = {0xFA, 0x61, 0x5D, 0xDC}; // M5STACK node id

const char* ssid     = SECRET_SSID;
const char* password = SECRET_PSK;

int period = 1000;
int8_t ipCnt = 0;

// unsigned long time_now = 0;

CRGB leds[ARGB_LEDS];

unsigned long ota_progress_millis = 0;

static volatile bool wifi_connected = false;
static volatile uint8_t myNodeID[] = {0, 0, 0, 0}; // node ID

// search the node list for specified node ID
/* uint8_t nodeSearch(uint8_t* rxNodeID) {
  static int compare = 0; // comparison variable

  for (uint8_t i = 0; i < nodeListMax; i++) {
    // uint32_t nodeID32   = (nodeList[i].nodeID[0] << 24) | (nodeList[i].nodeID[1] << 16) | (nodeList[i].nodeID[2] << 8) | nodeList[i].nodeID[3];
    // uint32_t rxNodeID32 = (rxNodeID[0] << 24) | (rxNodeID[1] << 16) | (rxNodeID[2] << 8) | rxNodeID[3];
    if (memcmp((const void*)nodeList[i].nodeID, (const void*)rxNodeID, 4) == 0) { // check if node ID matches
      return i; // return index of node
    }
  }
  return 255; // node not found
}
 */

// Define the node ID length for clarity and maintainability
#define NODE_ID_LENGTH 4
// Define clear return values for "not found"
#define NODE_NOT_FOUND -1
#define MODULE_NOT_FOUND -10

// Define a value for invalid nodeID
#define NODE_ID_INVALID -2


// Function that gets current epoch time
unsigned long getEpoch() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

// print timestamp to webserial console
static void printEpoch() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return;
  }
  time(&now);
  WebSerial.printf("%lu\n", now);
}

/**
 * @brief Searches the global nodeList for a specified node ID.
 *
 * @param rxNodeID Pointer to the 4-byte node ID to search for. Should not be NULL.
 * @return The index of the found node in nodeList if successful,
 *         NODE_NOT_FOUND (-1) if the node ID is not found or NODE_ID_INVALID if rxNodeID is NULL.
 */
int nodeSearch(const uint8_t rxNodeID[4]) {
  // Basic input validation
  if (rxNodeID == NULL) {
      return NODE_ID_INVALID; // Cannot search for a NULL ID
  }

  for (uint8_t i = 0; i < nodeListMax; i++) {
    // Compare the 4 bytes of the IDs
    if (memcmp(nodeList[i].nodeID, rxNodeID, NODE_ID_LENGTH) == 0) {
      return (int)i; // Return index of the found node
    }
  }

  return NODE_NOT_FOUND; // Node not found after searching the entire list
}

/**
 * @brief Searches the global nodeList for a specified node ID.
 *
 * @param rxNodeID Pointer to the 4-byte node ID to search for. Should not be NULL.
 * @return The index of the found node in nodeList if successful,
 *         NODE_NOT_FOUND (-1) if the node ID is not found or rxNodeID is NULL.
 */
int modSearch(const uint8_t rxNodeID[4], const uint16_t modID) {
  // Basic input validation
  if (rxNodeID == NULL) {
      return NODE_NOT_FOUND; // Cannot search for a NULL ID
  }

  for (uint8_t i = 0; i < nodeListMax; i++) {
    // Compare the 4 bytes of the IDs
    if (memcmp(nodeList[i].nodeID, rxNodeID, NODE_ID_LENGTH) == 0) { // matched node, now step through sub modules
      for (uint8_t j = 0; j < modListMax; j++) {
        if (nodeList[i].subModules[j] == modID) { // matched sub module
          return (int)i; // Return index of the found node
        }
      }
      return MODULE_NOT_FOUND; // Node found but not the sub module, return not found flag
    }
  }

  return NODE_NOT_FOUND; // Node not found after searching the entire list
}

static void updateLasteen(const uint8_t* rxNodeID) {
  // update last seen time for node
  for (uint8_t i = 0; i < nodeListMax; i++) {
    if (memcmp(nodeList[i].nodeID, rxNodeID, NODE_ID_LENGTH) == 0) { // check if node ID matches
      nodeList[i].lastSeen = getEpoch(); // update last seen time
      break;
    }
  }
}

// dump node list to WebSerial
static void dumpNodeList() {
  WebSerial.println(" ");
  WebSerial.println(" ");
  WebSerial.println("--------------------------------------------------------------------------");
  WebSerial.println("Node List:");
  for (uint8_t i = 0; i < nodeListMax; i++) {
    if (nodeList[i].nodeID[0] != 0) { // check if node ID is not empty
      WebSerial.printf("Node %d: %02x:%02x:%02x:%02x\n", i, nodeList[i].nodeID[0], nodeList[i].nodeID[1], nodeList[i].nodeID[2], nodeList[i].nodeID[3]);
      WebSerial.printf("Type: %03x\n", nodeList[i].nodeType);
      WebSerial.printf("Feature Mask: %02x %02x\n", nodeList[i].featureMask[0], nodeList[i].featureMask[1]);
      WebSerial.printf("Sub Modules: %03x %03x %03x %03x\n", nodeList[i].subModules[0], nodeList[i].subModules[1], nodeList[i].subModules[2], nodeList[i].subModules[3]);
      WebSerial.printf("Sub Module Count: %d %d %d %d\n", nodeList[i].subModCnt[0], nodeList[i].subModCnt[1], nodeList[i].subModCnt[2], nodeList[i].subModCnt[3]);
      WebSerial.printf("Module Count: %d\n", nodeList[i].moduleCnt);
      WebSerial.printf("First Seen: %lu\n", nodeList[i].firstSeen);
      WebSerial.printf("Last Seen: %lu\n", nodeList[i].lastSeen);
    }
    WebSerial.println(" ");
  }
  WebSerial.println("--------------------------------------------------------------------------");
  WebSerial.println(" ");
  WebSerial.println(" ");

}


void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    /* Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]); */
    myNodeID[0] = baseMac[2];
    myNodeID[1] = baseMac[3];
    myNodeID[2] = baseMac[4];
    myNodeID[3] = baseMac[5];
    Serial.printf("Node ID: %02x:%02x:%02x:%02x\n", myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3]);
  } else {
    Serial.println("Failed to set NODE ID");
  }
}

void wifiOnConnect(){
  Serial.println("STA Connected");
  Serial.print("STA SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("STA IPv4: ");
  Serial.println(WiFi.localIP());
}

//when wifi disconnects
void wifiOnDisconnect(){
  Serial.println("STA disconnected, reconnecting...");
  delay(1000);
  WiFi.begin(ssid, password);
}

void WiFiEvent(WiFiEvent_t event){
  
}

static void send_message(const uint16_t msgID, const uint8_t *msgData, const uint8_t dlc) {
  static twai_message_t message;
  // static uint8_t dataBytes[] = {0, 0, 0, 0, 0, 0, 0, 0}; // initialize dataBytes array with 8 bytes of 0

  if ((msgData == NULL) || (dlc > 8)) {
    // exit if msgData is NULL or DLC more than 8 bytes
    return;
  }


  leds[0] = CRGB::Blue;
  FastLED.show();

  // Format message
  message.identifier = msgID;       // set message ID
  message.extd = 0;                 // 0 = standard frame, 1 = extended frame
  message.rtr = 0;                  // 0 = data frame, 1 = remote frame
  message.self = CAN_SELF_MSG;      // 0 = normal transmission, 1 = self reception request 
  message.dlc_non_comp = 0;         // non-compliant DLC (0-8 bytes)  
  message.data_length_code = dlc;   // data length code (0-8 bytes)
  memcpy(message.data, (const uint8_t*) msgData, dlc);  // copy data to message data field 
  
  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(3000)) == ESP_OK) {
    // ESP_LOGI(TAG, "Message queued for transmission\n");
    // printf("Message queued for transmission\n");
    // WebSerial.printf("TX: MSG: %03x WITH %u DATA", msgID, dlc);
    // WebSerial.printf("TX: MSG: %03x Data: ", msgID);
    // for (int i = 0; i < dlc; i++) {
    //   WebSerial.printf("%02x ", message.data[i]);
    // }
    // WebSerial.printf("\n");
  } else {
    leds[0] = CRGB::Red;
    FastLED.show();
    // ESP_LOGE(TAG, "Failed to queue message for transmission, initiating recovery");
    WebSerial.printf("ERR: Failed to queue message for transmission, resetting controller\n");
    twai_initiate_recovery();
    twai_stop();
    WebSerial.printf("WARN: twai Stopped\n");
    vTaskDelay(500);
    twai_start();
    WebSerial.printf("WARN: twai Started\n");
    // ESP_LOGI(TAG, "twai restarted\n");
    // wifiOnConnect();
    vTaskDelay(500);
    leds[0] = CRGB::Black;
    FastLED.show();
  }
  leds[0] = CRGB::Black;
  FastLED.show();
  // vTaskDelay(100);
}

static void rxDisplayMode(uint8_t *data, uint8_t displayMode) {
  static uint8_t rxdisplayID = data[4]; // display id
  WebSerial.printf("RX: Display: %d Mode: %d\n", rxdisplayID, displayMode);

  switch (displayMode) {
    case 0: // display off
      break;
    case 1: // display on
      break;
    case 2: // clear display
      break;
    case 3: // flash display
      break;
    default:
      WebSerial.println("Invalid display mode");
      break;
  }
}

// assemble message to change output switch state on remote nodes
static void txSwitchState(const uint8_t *nodeID, const uint8_t switchID, const uint8_t swState) {
  if (nodeID == NULL) {
    // WebSerial.println("Invalid node ID");
    return;
  }

  const uint8_t txDLC = 5;
  const uint8_t dataBytes[] = {nodeID[0], nodeID[1], nodeID[2], nodeID[3], switchID}; // set node id and switch ID
  
  WebSerial.printf("TX: %02x:%02x:%02x:%02x Switch %d State %d\n", nodeID[0],nodeID[1],nodeID[2],nodeID[3], switchID, swState);

  switch (swState) {
    case 0: // switch off
      send_message(SW_SET_OFF, dataBytes, txDLC);
      break;
    case 1: // switch on
      send_message(SW_SET_ON, dataBytes, txDLC);
      break;
    case 2: // momentary press
      send_message(SW_MOM_PRESS, dataBytes, txDLC);
      break;
    default: // unsupported state
      // WebSerial.println("Invalid switch state for transmission");
      break;
  }
}

// assemble message to change output switch mode on remote nodes
static void txSwitchMode(const uint8_t *data, const uint8_t switchID, const uint8_t switchMode) {
  const uint8_t txDLC = 6;
  const uint8_t dataBytes[] = {data[0], data[1], data[2], data[3], switchID, switchMode}; // set node id switch ID
  WebSerial.printf("TX: %02x:%02x:%02x:%02x Switch %d Mode %d\n",data[0], data[1], data[2], data[3], switchID, switchMode);
  send_message(SW_SET_MODE, dataBytes, sizeof(dataBytes)); // send message to set switch mode
}

// assemble node introduction message
static void txIntroduction() {
    // if (introMsgPtr == 0) {
    //   static uint8_t dataBytes[6] = { myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3], 
    //                                   myNodeFeatureMask[0], myNodeFeatureMask[1] }; 

    //   send_message(introMsg[introMsgPtr], dataBytes, sizeof(dataBytes));
    // } else {
    //   static uint8_t dataBytes[5] = { myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3], 
    //                                   introMsgData[introMsgPtr] }; 

    //   send_message(introMsg[introMsgPtr], dataBytes, sizeof(dataBytes));
    // }
}

// send command to clear normal op flag on remote
static void txSendHalt(const uint8_t* txNodeID) {
  // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55}; // data bytes
  send_message(MSG_HALT_OPER, txNodeID, 4);
}

static void txIntroack(const u_int16_t msgID, const uint8_t* txNodeID) {
  // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55}; // data bytes
  send_message(msgID, txNodeID, 4);
}

// eventually check on connected nodes here
static void nodeCheckStatus() {

}

// handle incoming can messages
static void handle_rx_message(twai_message_t &message) {
  bool msgFlag = false;
  bool haveRXID = false; 
  int msgIDComp;
  uint8_t rxNodeID[4] = {0, 0, 0, 0}; // node ID

  leds[0] = CRGB::Orange;
  FastLED.show();

  // check if message contains enough data to have node id
  if (message.data_length_code > 3) { 
    memcpy((void *)rxNodeID, (const void *)message.data, 4); // copy node id from message
    msgIDComp = memcmp((const void *)rxNodeID, (const void *)myNodeID, 4);
    haveRXID = true; // set flag to true if message contains node id
    updateLasteen(rxNodeID);  // update last seen time for this node

    if (msgIDComp == 0) { // message is for us
      msgFlag = true; // message is for us, set flag to true
    }
  }

/*   if (message.data_length_code > 0) { // message contains data, check if it is for us
    if (msgFlag) {
      WebSerial.printf("RX: ID MATCH MSG: 0x%x WITH Data\n", message.identifier);
    } else {
      WebSerial.printf("RX: NO MATCH MSG: 0x%x WITH DATA\n", message.identifier);
    }
    /* for (int i = 0; i < message.data_length_code; i++) {
      WebSerial.printf(" %d = %02x", i, message.data[i]);
    }
    WebSerial.println(""); 
  } else {
    if (msgFlag) {
      WebSerial.printf("RX: ID MATCH MSG: 0x%x NO DATA", message.identifier);
    } else {
      WebSerial.printf("RX: NO MATCH MSG: 0x%x NO DATA", message.identifier);
    }
  } */
  

  uint8_t rxSwitchID = message.data[4]; // get switch ID
  switch (message.identifier) {
    case SET_DISPLAY_OFF:          // set display off
      rxDisplayMode(message.data, 0); 
      break;
    case SET_DISPLAY_ON:          // set display on
      rxDisplayMode(message.data, 1); 
      break;    
    case SET_DISPLAY_CLEAR:          // clear display
      rxDisplayMode(message.data, 2); 
      break;
    case SET_DISPLAY_FLASH:          // flash display
      rxDisplayMode(message.data, 3); 
      break;
    case DATA_OUTPUT_SWITCH_OFF:   // we received a switch off message, send a switch on message
      txSwitchState((uint8_t *)rxNodeID, rxSwitchID, 1);
      break;
    case DATA_OUTPUT_SWITCH_ON:    // we received a switch on message, send a switch momentary message
      txSwitchState((uint8_t *)rxNodeID, rxSwitchID, 2);
      break;      
    case DATA_OUTPUT_SWITCH_MOM_PUSH: // we received a momentary switch message, send a switch off message
      txSwitchState((uint8_t *)rxNodeID, rxSwitchID, 0);
      break;
    case REQ_INTERFACES: // request for interface introduction     
      WebSerial.printf("RX: IFACE intro req, responding to %02x:%02x:%02x:%02x\n", message.data[0], message.data[1], message.data[2], message.data[3]);
      // FLAG_SEND_INTRODUCTION = true; // set flag to send introduction message
      break;


    
    default:
      if ((message.identifier & MASK_24BIT) == (INTRO_BOX)) { // box introduction
        // check if the message arrived with a node id
        if (haveRXID) {
          int nodePtr = nodeSearch(rxNodeID); // node pointer
          // WebSerial.printf("RX: BOX intro %02x:%02x:%02x:%02x PTR:%i\n", rxNodeID[0], rxNodeID[1], rxNodeID[2], rxNodeID[3], nodePtr);
          if (nodePtr == NODE_NOT_FOUND) { // node not found in list
            if (nodeListPtr < nodeListMax) { // check if we have space in the list
              nodeList[nodeListPtr].nodeID[0] = rxNodeID[0]; // node id
              nodeList[nodeListPtr].nodeID[1] = rxNodeID[1];
              nodeList[nodeListPtr].nodeID[2] = rxNodeID[2];
              nodeList[nodeListPtr].nodeID[3] = rxNodeID[3];
              nodeList[nodeListPtr].nodeType  = message.identifier; // node type
              nodeList[nodeListPtr].featureMask[0] = message.data[4]; // feature mask
              nodeList[nodeListPtr].featureMask[1] = message.data[5]; // feature mask
              nodeList[nodeListPtr].firstSeen  = getEpoch(); // set first seen time
              nodeListPtr = nodeListPtr + 1; // increment node list pointer
              WebSerial.printf("RX: ADDED BOX #%d: %02x:%02x:%02x:%02x\n", nodeListPtr, rxNodeID[0], rxNodeID[1], rxNodeID[2], rxNodeID[3]);
            } else {
              // WebSerial.println("RX: BOX LIST FULL");
              // nodeListPtr = 0; // reset node list pointer
            }
          } else {
            // WebSerial.println("RX: BOX ALREADY IN LIST");
            // nodeList[nodePtr].lastSeen  = getEpoch(); // update last seen time
          }
          txIntroack(ACK_SWITCHBOX, rxNodeID); // ack introduction message
        }
      } else if ((message.identifier & MASK_25BIT) == (INTRO_OUTPUT)) { // output introduction
        // check if the message arrived with a node id
        if (haveRXID) {
          int nodePtr = nodeSearch(rxNodeID); // node pointer
          // WebSerial.printf("RX: OUTP intro PTR %i\n", nodePtr);
          if (nodePtr == NODE_NOT_FOUND) { // node not found in list
            // WebSerial.println("RX: OUTP PARENT NODE NOT FOUND");
          } else {
            if (nodeList[nodePtr].moduleCnt < modListMax) { // check if we have space in the list
              int modSearchPtr = modSearch(rxNodeID, message.identifier); // check if the sub module is already in the list
              // WebSerial.printf("RX: MOD FOUND AT PTR %i ON NODE %i\n", modSearchPtr, nodePtr);
              if (modSearchPtr == MODULE_NOT_FOUND) { // check if the sub module is already in the list
                nodeList[nodePtr].lastSeen  = getEpoch(); // update last seen time
                nodeList[nodePtr].subModules[nodeList[nodePtr].moduleCnt] = message.identifier; // set sub module type
                nodeList[nodePtr].subModCnt[nodeList[nodePtr].moduleCnt] = message.data[4]; // set number of outputs
                nodeList[nodePtr].moduleCnt = nodeList[nodePtr].moduleCnt + 1; // increment module count
                WebSerial.printf("RX: OUTP %03x ADDED TO NODE %02x:%02x:%02x:%02x\n", message.identifier, rxNodeID[0], rxNodeID[1], rxNodeID[2], rxNodeID[3]);
              } else {
                // WebSerial.printf("RX: OUTP %03x ALREADY ON NODE %02x:%02x:%02x:%02x\n", message.identifier, rxNodeID[0], rxNodeID[1], rxNodeID[2], rxNodeID[3]);
              }
            } else {
              WebSerial.println("RX: NODE MODULE LIST FULL");
              // nodeList[nodePtr].moduleCnt = 0; // reset module count
            }
          }
          txIntroack(ACK_SWITCHBOX, rxNodeID);
        }
      }
      break;
  }

  leds[0] = CRGB::Black;
  FastLED.show();

} // end of handle_rx_message

void TaskTWAI(void *pvParameters) {
  // give some time at boot the cpu setup other parameters
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  leds[0] = CRGB::Red;
  FastLED.show();

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);  // TWAI_MODE_NO_ACK , TWAI_MODE_LISTEN_ONLY , TWAI_MODE_NORMAL
  g_config.rx_queue_len = 20; // RX queue length
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    WebSerial.println("Driver installed");
  } else {
    WebSerial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    WebSerial.println("Driver started");
  } else {
    WebSerial.println("Failed to start driver");
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    WebSerial.println("CAN Alerts reconfigured");
  } else {
    WebSerial.println("Failed to reconfigure alerts");
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;


  for (;;) {
    if (!driver_installed) {
      // Driver not installed
      vTaskDelay(1000);
      return;
    }
   
    // Check if alert happened
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    // Handle alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      WebSerial.println("Alert: TWAI controller has become error passive.");
      leds[0] = CRGB::Red;
      FastLED.show();
    }

    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      WebSerial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      WebSerial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
      leds[0] = CRGB::Red;
      FastLED.show();
    }

    if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
      WebSerial.println("Alert: The Transmission failed.");
      WebSerial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
      WebSerial.printf("TX error: %d\t", twaistatus.tx_error_counter);
      WebSerial.printf("TX failed: %d\n", twaistatus.tx_failed_count);
      leds[0] = CRGB::Red;
      FastLED.show();
    }

    if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
      leds[0] = CRGB::Green;
      FastLED.show();
      // Serial.println("Alert: The Transmission was successful.");
      // Serial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
    }

    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      WebSerial.println("Alert: The RX queue is full causing a received frame to be lost.");
      WebSerial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
      WebSerial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
      WebSerial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
    }

    // Check if message is received
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {

      // One or more messages received. Handle all.
      twai_message_t message;
      while (twai_receive(&message, 0) == ESP_OK) {
        handle_rx_message(message);
      }
    }
    // polling goes here
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) { // run this code every 1000 ms
      previousMillis = currentMillis;
      leds[0] = CRGB::DarkBlue;
      FastLED.show();
      // if (introMsgPtr < introMsgCnt) {
      //   WebSerial.printf("TX: Intro message ptr %d\n", introMsgPtr);
      //   if (introMsg[introMsgPtr] > 0) {
      //     send_message(introMsg[introMsgPtr], (uint8_t*) myNodeID, 4); // send introduction request
      //   }
      //   if (introMsgPtr == 0) {
      //     uint8_t dataBytes[4] = { myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3] }; 

      //     send_message(introMsg[introMsgPtr], dataBytes, sizeof(dataBytes));
      //     introMsgPtr = introMsgPtr + 1; // increment intro message pointer 1st step
      //   }
      // }
      nodeCheckStatus();
      // printEpoch();
      WebSerial.printf(".\n");
      // Serial.printf(".");
    }
    vTaskDelay(10);

  }
}

void printWifi() {
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
  if (d == "ON"){
    send_message(REQ_SWITCHBOX, (uint8_t*) myNodeID, 4); // send introduction request
    // digitalWrite(LED, HIGH);
  }
  if (d == "C0"){
    vTaskSuspend(canbus_task_handle); // suspend canbus task
    // digitalWrite(LED, HIGH);
  }
  if (d == "C1"){
    vTaskResume(canbus_task_handle); // resume canbus task
    // digitalWrite(LED, HIGH);
  }
  if (d == "W"){
    printWifi();
    // digitalWrite(LED, HIGH);
  }

  if (d=="LIST"){
    dumpNodeList();
    // digitalWrite(LED, LOW);
  }
}


void setup() {

  introMsgCnt = 5; // number of intro messages
  introMsgPtr = 0; // start at zero
  introMsg[1] = (uint16_t) REQ_SWITCHBOX; // ask for boxes
  introMsg[2] = 0; // first ack introduction
  introMsg[3] = 0; // second ack introduction
  introMsg[4] = (uint16_t) MSG_NORM_OPER; // send normal operation message  
  
  delay(5000);

  // Timer0_Cfg = timerBegin(0, 80, true);
  // timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  // timerAlarmWrite(Timer0_Cfg, 100000, true);
  // timerAlarmEnable(Timer0_Cfg);

  xTaskCreatePinnedToCore(
    TaskTWAI,     // Task function.
    "Task TWAI",  // name of task.
    4096,         // Stack size of task
    NULL,         // parameter of the task
    1,            // priority of the task
    &canbus_task_handle,           // Task handle to keep track of created task
    tskNO_AFFINITY); // allow task to run on either core, automatic depends the load of each core
  // );              

  // xTaskCreate(
  //   TaskFLED,     // Task function.
  //   "Task FLED",  // name of task.
  //   2048,         // Stack size of task
  //   NULL,         // parameter of the task
  //   1,            // priority of the task
  //   NULL    // Task handle to keep track of created task
  // );              // pin task to core 0
  //tskNO_AFFINITY); // pin task to core is automatic depends the load of each core

  FastLED.addLeds<SK6812, ARGB_PIN, GRB>(leds, ARGB_LEDS);
  leds[0] = CRGB::Black;
  FastLED.show();

  Serial.begin(9600, SERIAL_8N1, 19, 18); // alternate serial port
  Serial.println("Hello, world!");
  
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.softAP(AP_SSID);
  WiFi.begin(ssid, password);

  ArduinoOTA
  .onStart([]() {
    String type;
    vTaskSuspend(canbus_task_handle); // suspend canbus task
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();

  Serial.println("AP Started");
  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP IPv4: ");
  Serial.println(WiFi.softAPIP());

  // Make it possible to access webserver at http://myEsp32.local
  if (!MDNS.begin(hostname)) {
    Serial.println("Error setting up mDNS responder!");
  } else {
    Serial.printf("Access at http://%s.local\n", hostname);
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is AsyncWebServer.");
  });

  server.begin();
  Serial.println("HTTP server started");
  
  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);
  WebSerial.onMessage(recvMsg);
  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
  // printWifi();
  configTime(UTC_OFFSET, UTC_OFFSET_DST, NTP_SERVER);
}

void loop() {
  ArduinoOTA.handle();
  // NOP;
}