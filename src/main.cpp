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
// TaskHandle_t 

// my canbus stuff
#include "canbus_msg.h"
#include "canbus_flags.h"

#define CAN_SELF_MSG 0

// Interval:
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS  100

// time stuff
#define NTP_SERVER     "us.pool.ntp.org"
#define UTC_OFFSET     0
#define UTC_OFFSET_DST 0

struct imuDataType {
  float xaccel = 0.0;    
  float yaccel = 0.0;
  float zaccel = 0.0;
  float xgyro  = 0.0;
  float ygyro  = 0.0;
  float zgyro  = 0.0;
  float temperature = 0.0;
  uint32_t timestamp = 0;
};

volatile struct imuDataType IMUdata;
volatile bool imuDumpFlag = false;

// organize nodes 
struct remoteNode {
  // 32-bit node id number
  uint8_t   nodeID[4]      = {0,0,0,0}; 
  // 11-bit can bus message id and node type
  uint16_t  nodeType       = 0;
  // node feature mask storaege (optional)
  uint8_t   featureMask[2] = {0,0};
  // storage for any sub modules
  uint16_t  subModuleList[8]  = {0,0,0,0}; 
  // sub module count for each sub module
  uint8_t   subModCntList[8]   = {0,0,0,0};
  // total sub module count
  uint8_t   moduleCnt      = 0; 
  // last time message received from node 
  uint32_t  lastSeen       = 0;
  // first time message received from node 
  uint32_t  firstSeen       = 0;
} ; 

struct remoteNode nodeList[8]; // list of remote nodes
volatile uint8_t  nodeListPtr     =     0; // node list pointer
const    uint8_t  NODE_LIST_MAX   =     8; // max number of nodes in the list
const    uint8_t  MOD_LIST_MAX    =     8; // max number of modules per node
    
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
const uint16_t myNodeIntro = REQ_IFACE; // intro request for my node type
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

  for (uint8_t i = 0; i < NODE_LIST_MAX; i++) {
    // uint32_t nodeID32   = (nodeList[i].nodeID[0] << 24) | (nodeList[i].nodeID[1] << 16) | (nodeList[i].nodeID[2] << 8) | nodeList[i].nodeID[3];
    // uint32_t rxNodeID32 = (rxNodeID[0] << 24) | (rxNodeID[1] << 16) | (rxNodeID[2] << 8) | rxNodeID[3];
    if (memcmp((const void*)nodeList[i].nodeID, (const void*)rxNodeID, 4) == 0) { // check if node ID matches
      return i; // return index of node
    }
  }
  return 255; // node not found
}
 */


// convert byte array into 32-bit integer
static uint32_t unchunk32(const uint8_t* dataBytes){
  static uint32_t result = ((dataBytes[0]<<24) || (dataBytes[1]<<16) || (dataBytes[2]<<8) || (dataBytes[3]));

  return result;
}


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

  for (uint8_t i = 0; i < NODE_LIST_MAX; i++) {
    // Compare the 4 bytes of the IDs
    if (memcmp(nodeList[i].nodeID, rxNodeID, NODE_ID_SIZE) == 0) {
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

  for (uint8_t i = 0; i < NODE_LIST_MAX; i++) {
    // Compare the 4 bytes of the IDs
    if (memcmp(nodeList[i].nodeID, rxNodeID, NODE_ID_SIZE) == 0) { // matched node, now step through sub modules
      for (uint8_t j = 0; j < MOD_LIST_MAX; j++) {
        if (nodeList[i].subModuleList[j] == modID) { // matched sub module
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
  for (uint8_t i = 0; i < NODE_LIST_MAX; i++) {
    if (memcmp(nodeList[i].nodeID, rxNodeID, NODE_ID_SIZE) == 0) { // check if node ID matches
      nodeList[i].lastSeen = getEpoch(); // update last seen time
      break;
    }
  }
}

static int updateNodeList(const uint8_t* rxNodeID, const uint16_t rxNodeType, 
                           const uint8_t* rxFeatureMask, const uint8_t rxSubModCnt = 0, 
                           const bool addSubModule = false) {
  if (rxNodeID == NULL) {
    return NODE_NOT_FOUND; // Cannot update with a NULL ID
  }
  // check if node ID is already in the list
  int nodeIndex = nodeSearch(rxNodeID); // step one check if node is already in the list

  
  if ((nodeIndex == NODE_NOT_FOUND) && (!addSubModule)) { // node not in the list and not adding sub module
    if (nodeListPtr < NODE_LIST_MAX) { // step two check if there is space in the list
      nodeList[nodeListPtr].nodeType = rxNodeType; // copy node type
      memcpy(nodeList[nodeListPtr].nodeID, rxNodeID, NODE_ID_SIZE); // copy node ID to list
      memcpy(nodeList[nodeListPtr].featureMask, rxFeatureMask, FEATURE_MASK_SIZE); // copy feature mask to list
      nodeList[nodeListPtr].firstSeen = getEpoch(); // set first seen time
      nodeList[nodeListPtr].lastSeen = getEpoch(); // set last seen time
      // WebSerial.printf("RX: ADDED NODE %02x:%02x:%02x:%02x TO LIST AT %u\n", 
                        // rxNodeID[0], rxNodeID[1], rxNodeID[2], rxNodeID[3], nodeListPtr);
      nodeListPtr = nodeListPtr + 1; // increment node list pointer
      return true; // new node added to list
    } else {
      return NODE_LIST_FULL; // no space in the list for new node
    }
  } else if (addSubModule) {                                                                       // add sub module to existing node
      if ((nodeIndex == NODE_NOT_FOUND) || (nodeIndex >= NODE_LIST_MAX)) {                                                         // node not in the list
        return NODE_NOT_FOUND;                                                                     // unable to add sub module, parent node not on the list
      } else {                                                                                     // parent node found, add sub module to existing node
        nodeList[nodeIndex].lastSeen = getEpoch();                                               // update node's last seen time
        if (nodeList[nodeIndex].moduleCnt < MOD_LIST_MAX) {                                      // check if we have space in the sub module list
          int modIndex = modSearch(rxNodeID, rxNodeType);                                          // check if the sub module is already in the list
          if (modIndex == MODULE_NOT_FOUND) {                                                      // module not on the node already, add it now
            uint8_t  moduleCnt                           = nodeList[nodeIndex].moduleCnt;      // get current sub module count
            nodeList[nodeIndex].subModuleList[moduleCnt] = rxNodeType;                           // add sub module to list
            nodeList[nodeIndex].subModCntList[moduleCnt] = rxSubModCnt;                          // add sub module count to list
            nodeList[nodeIndex].moduleCnt                = moduleCnt + 1;                        // increment sub module count
            // WebSerial.printf("RX: SUB MOD %03x ADDED TO NODE %u (%02x:%02x:%02x:%02x) PTR %u\n", 
                            //  rxNodeType, nodeIndex, rxNodeID[0], rxNodeID[1], rxNodeID[2], 
                            //  rxNodeID[3], moduleCnt);
            return true; // new sub module added to list
          } else { // module already in the list, disregard
            // WebSerial.printf("RX: MOD FOUND PTR %i\n", modIndex);
            return MODULE_ALREADY_EXISTS; // module already in the list, return error code
          }
        }

      return MODULE_LIST_FULL; // sub module list full
    }
  }
  return NODE_ALREADY_EXISTS; // node on the list but not adding a submodule
}  // end of updateNodeList

// dump IMU data to WebSerial
static void dumpIMU() {
  if (!imuDumpFlag) {
    return; // bail out of the flag is not set
  }

  WebSerial.printf("IMU: Time stamp: %u Accel: x %.3f y %.3f z %.3f Gyro: x %.3f y %.3f z %.3f Temp: %.2f\n",
                    IMUdata.timestamp, IMUdata.xaccel, IMUdata.yaccel, IMUdata.zaccel,
                    IMUdata.xgyro, IMUdata.ygyro, IMUdata.zgyro, IMUdata.temperature);

  imuDumpFlag = false;
}

// dump node list to WebSerial
static void dumpNodeList() {
  WebSerial.println(" ");
  WebSerial.println(" ");
  WebSerial.println("--------------------------------------------------------------------------");
  if (nodeListPtr < 1) {
    WebSerial.println("Node list is empty!");

  } else { 
    WebSerial.printf("There are %u nodes:", nodeListPtr); 
  }
  for (uint8_t i = 0; i < NODE_LIST_MAX; i++) {
    if (nodeList[i].nodeID[0] != 0) { // check if node ID is not empty
      WebSerial.printf("Node %d: %02x:%02x:%02x:%02x\n", 
                      i, nodeList[i].nodeID[0], nodeList[i].nodeID[1],
                      nodeList[i].nodeID[2], nodeList[i].nodeID[3]);
      
      WebSerial.printf("Type: %03x\n", nodeList[i].nodeType);
      WebSerial.printf("Feature Mask: %02x %02x\n", nodeList[i].featureMask[0], nodeList[i].featureMask[1]);
      WebSerial.printf("Sub Modules: %03x %03x %03x %03x ", 
        nodeList[i].subModuleList[0], nodeList[i].subModuleList[1], nodeList[i].subModuleList[2], nodeList[i].subModuleList[3]);
      WebSerial.printf("%03x %03x %03x %03x\n", 
        nodeList[i].subModuleList[4], nodeList[i].subModuleList[5], nodeList[i].subModuleList[6], nodeList[i].subModuleList[7]);
      
        WebSerial.printf("Sub Module Count: %d %d %d %d ", 
          nodeList[i].subModCntList[0], nodeList[i].subModCntList[1], nodeList[i].subModCntList[2], nodeList[i].subModCntList[3]);
        WebSerial.printf("%d %d %d %d\n", 
          nodeList[i].subModCntList[4], nodeList[i].subModCntList[5], nodeList[i].subModCntList[6], nodeList[i].subModCntList[7]);
        
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
    // WebSerial.printf("TX: QUEUED MSG: %03x DATA: ", msgID);
    // for (int i = 0; i < dlc; i++) {
    //   WebSerial.printf("%02x ", message.data[i]);
    // }
    // WebSerial.printf("\n");
  } else {
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

  }
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
  
  // WebSerial.printf("TX: %02x:%02x:%02x:%02x Switch %d State %d\n", nodeID[0],nodeID[1],nodeID[2],nodeID[3], switchID, swState);

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
  // WebSerial.printf("TX: %02x:%02x:%02x:%02x Switch %d Mode %d\n",data[0], data[1], data[2], data[3], switchID, switchMode);
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
  send_message(MSG_HALT_OPER, txNodeID, NODE_ID_SIZE);
}

static void txIntroack(const u_int16_t msgID, const uint8_t* txNodeID) {
  // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55}; // data bytes
  send_message(msgID, txNodeID, NODE_ID_SIZE);
}

static uint8_t* getEpochStr() {
  static char epochStr[20] = {0}; // buffer for epoch string
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return NULL;
  }
  time(&now);
  // strftime(epochStr, sizeof(epochStr), "%Y-%m-%d %H:%M:%S", &timeinfo); // format epoch string
  sprintf(epochStr, "%lu", now); // copy the 32bit value to c-string
  return (uint8_t*)epochStr; // return pointer to epoch string
}

volatile ulong lastMillis;
// eventually check on connected nodes here
static void nodeCheckStatus() {
  unsigned long currentMillis = millis();

  // if (currentMillis - previousMillis >= 30000) { // run every 30 seconds
  //   WebSerial.printf(".\n");

  //   send_message(DATA_EPOCH, getEpochStr(), 4);  // send four bytes of the epoch time string
  //   lastMillis = currentMillis; // update last millis timestamp
  // }
}

// handle incoming can messages
static void handle_rx_message(twai_message_t &message) {
  bool msgFlag = false;
  bool haveRXID = false; 
  int msgIDComp;
  uint8_t dlc = message.data_length_code;
  uint16_t msgID = message.identifier; // store message ID
  uint8_t rxNodeID[NODE_ID_SIZE] = {0, 0, 0, 0}; // node ID
  uint8_t imuDataBytes[CAN_MAX_DLC] = {0,0,0,0,0,0,0,0}; // storage for data string from IMU

  leds[0] = CRGB::Orange;
  FastLED.show();

  // check if message contains enough data to have node id
  if (message.data_length_code > 3) { 
    memcpy((void *)rxNodeID, (const void *)message.data, NODE_ID_SIZE); // copy node id from message
    msgIDComp = memcmp((const void *)rxNodeID, (const void *)myNodeID, NODE_ID_SIZE);
    haveRXID = true; // set flag to true if message contains node id
    updateLasteen(rxNodeID);  // update last seen time for this node

    if (msgIDComp == 0) { // message is for us
      msgFlag = true; // message is for us, set flag to true
    }
  }

  if (message.data_length_code > 0) { // message contains data, check if it is for us
    if (msgFlag) {
      WebSerial.printf("RX: ID MATCH MSG: %x WITH Data\n", message.identifier);
    } else {
      // WebSerial.printf("RX: NO MATCH MSG: 0x%x WITH DATA ", message.identifier);
    }
    // for (int i = 0; i < message.data_length_code; i++) {
    //   WebSerial.printf("%d = %02x ", i, message.data[i]);
    // }
    // WebSerial.println(""); 
  } else {
    if (msgFlag) {
      WebSerial.printf("RX: ID MATCH MSG: %x NO DATA\n", message.identifier);
    } else {
      // WebSerial.printf("RX: NO MATCH MSG: 0x%x NO DATA", message.identifier);
    }
  }
  
  // handle IMU messages
  if ((msgID >= DATA_IMU_X_AXIS) && (msgID <= DATA_IMU_TEMPERATURE)) {
    for (uint8_t i=0; i<CAN_MAX_DLC; i++) imuDataBytes[i] = message.data[i]; // copy the imu data from the message string
       
    IMUdata.timestamp = getEpoch(); // update imu data timestamp
  }
  

  uint8_t rxSwitchID = message.data[4]; // get switch ID
  switch (msgID) {
    case DATA_IMU_X_AXIS:
      IMUdata.xaccel = atof((char*)imuDataBytes);
      break;
    case DATA_IMU_Y_AXIS:
      IMUdata.yaccel = atof((char*)imuDataBytes);
      break;
    case DATA_IMU_Z_AXIS:
      IMUdata.zaccel = atof((char*)imuDataBytes);
      break;
    case DATA_IMU_X_GYRO:
      IMUdata.xgyro = atof((char*)imuDataBytes);
      break;
    case DATA_IMU_Y_GYRO:
      IMUdata.ygyro = atof((char*)imuDataBytes);
      break;
    case DATA_IMU_Z_GYRO:
      IMUdata.zgyro = atof((char*)imuDataBytes);
      break;
    case DATA_IMU_TEMPERATURE:
      IMUdata.temperature = atof((char*)imuDataBytes);
      break;

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
    // case REQ_IFACE: // request for interface introduction     
    //   WebSerial.printf("RX: IFACE intro req, responding to %02x:%02x:%02x:%02x\n", message.data[0], message.data[1], message.data[2], message.data[3]);
    //   // FLAG_SEND_INTRODUCTION = true; // set flag to send introduction message
    //   break;
    
    /* this codeblock lives in the updateNodeList function now, saving it for reference
            if (haveRXID) { // check if the message arrived with a node id
          int nodePtr = nodeSearch(rxNodeID); // check if this node is already in the list
          WebSerial.printf("RX: IFACE intro %02x:%02x:%02x:%02x PTR:%i\n", rxNodeID[0], rxNodeID[1], rxNodeID[2], rxNodeID[3], nodePtr);
          if (nodePtr == NODE_NOT_FOUND) { // node not found in list
            if (nodeListPtr < NODE_LIST_MAX) { // check if we have space in the list
              nodeList[nodeListPtr].nodeID[0] = rxNodeID[0]; // node id
              nodeList[nodeListPtr].nodeID[1] = rxNodeID[1];
              nodeList[nodeListPtr].nodeID[2] = rxNodeID[2];
              nodeList[nodeListPtr].nodeID[3] = rxNodeID[3];
              nodeList[nodeListPtr].nodeType  = msgID; // node type
              nodeList[nodeListPtr].featureMask[0] = message.data[4]; // feature mask
              nodeList[nodeListPtr].featureMask[1] = message.data[5]; // feature mask
              nodeList[nodeListPtr].firstSeen  = getEpoch(); // set first seen time
              nodeListPtr = nodeListPtr + 1; // increment node list pointer
              WebSerial.printf("RX: ADDED IFACE #%d: %02x:%02x:%02x:%02x\n", nodeListPtr, rxNodeID[0], rxNodeID[1], rxNodeID[2], rxNodeID[3]);

    */      

    default:
 
      // add main node to list here
      if ((msgID & NODE_MOD_MASK) >= ADDR_FIRST_NODE) { // node introduction messages
        uint8_t rxFeatureMask[FEATURE_MASK_SIZE] = {0, 0}; // feature mask storage
        memcpy((void *)rxFeatureMask, (const void *)&message.data[4], FEATURE_MASK_SIZE); // copy feature mask from message
        int updateResult = updateNodeList(rxNodeID, msgID, rxFeatureMask); // update node list with new node
        if (updateResult == true) { // node added to list
          WebSerial.printf("RX: ADDED %03x NODE: %02x:%02x:%02x:%02x\n", msgID, rxNodeID[0], rxNodeID[1], rxNodeID[2], rxNodeID[3]);
        // } else if (updateResult == NODE_LIST_FULL) { // node list full
        //   WebSerial.println("RX: NODE LIST FULL");
        // } else if (updateResult == NODE_ALREADY_EXISTS) { // node already exists in the list
        //   WebSerial.println("RX: NODE ALREADY ON LIST");
        // } else if (updateResult == NODE_NOT_FOUND) { // rxnodeid was blank
        //   WebSerial.println("RX: INVALID NODE ID");
        } else {
          WebSerial.println("RX: UNKNOWN ERROR");
        }
        txIntroack(ACK_INTRO, rxNodeID); // ack introduction message

      // add sub module to list here
      } else if (((msgID & NODE_MOD_MASK) >= ADDR_FIRST_MODULE) && ((msgID & NODE_MOD_MASK) < ADDR_FIRST_NODE)) { // sub module introduction messages
        const uint8_t subModuleCnt = message.data[4]; // get number of sub modules

        int updateResult = updateNodeList(rxNodeID, msgID, NULL, subModuleCnt, true ); // add sub module to node list
        if (updateResult == true) { // sub mod added to list
          WebSerial.printf("RX: ADDED SUB MODULE %03x TO NODE %02x:%02x:%02x:%02x\n", msgID, rxNodeID[0], rxNodeID[1], rxNodeID[2], rxNodeID[3]);
        // } else if (updateResult == NODE_NOT_FOUND) { // parent node not on the list
        //   WebSerial.println("RX: NODE NOT FOUND");
        // } else if (updateResult == MODULE_ALREADY_EXISTS) { // this module is already listed
        //   WebSerial.println("RX: NODE ALREADY ON LIST");
        // } else if (updateResult == MODULE_LIST_FULL) { // sub module list is full for this node
        //   WebSerial.println("RX: NODE SUB MOD LIST FULL");
        // } else if (updateResult == NODE_NOT_FOUND) { // rxnodeid was blank
        //   WebSerial.println("RX: INVALID NODE ID");
        } else {
          WebSerial.println("RX: UNKNOWN ERROR");
        }
        txIntroack(ACK_INTRO, rxNodeID); // ack introduction message
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
      // leds[0] = CRGB::DarkBlue;
      // FastLED.show();
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
      // nodeCheckStatus();
      WebSerial.printf(".");
      dumpIMU();

      // send_message(DATA_EPOCH, getEpochStr(), 4);  // send four bytes of the epoch time string
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
    send_message(REQ_NODE_INTRO, (uint8_t*) myNodeID, 4); // send introduction request
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

  if (d == "RESTART"){
    WebSerial.println("Restarting...");
    ESP.restart();
    // digitalWrite(LED, HIGH);
  }

  if (d == "NODEID"){
    WebSerial.printf("Node ID: %02x:%02x:%02x:%02x\n", myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3]);
    // digitalWrite(LED, HIGH);
  }

  if (d == "TIME"){
    printEpoch();
    // digitalWrite(LED, HIGH);
  }

  if (d == "IMU"){
    imuDumpFlag = !imuDumpFlag;
    WebSerial.printf("IMU dump flag %d", imuDumpFlag);
  }
}


void setup() {

  introMsgCnt = 5; // number of intro messages
  introMsgPtr = 0; // start at zero
  introMsg[1] = (uint16_t) REQ_NODE_INTRO; // ask for remote nodes
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