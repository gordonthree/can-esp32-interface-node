# ESP32 Interface Node
Part of the CAN Bus node project.

This node supports only user-interface functions, such as buttons, displays and indicators.

nodeDB structure pseudo code 
```
struct remoteNode {
   uint8_t   nodeID[4]; // four byte node identifier 
   uint16_t  nodeType; // first introduction type
   uint16_t  subModules[4]; // introductions for up to four sub modules 
   uint8_t   moduleCnt; // sub module count
   uint32_t  lastSeen; // unix timestamp 
};
```

Json sample code

```
int main() {
  // Allocate the JSON document
  JsonDocument doc;

  // JSON input string

  const char* json = "{ \"sw\":[{ \"id\": 0, \"mem\":0, \"data\": [1,1,500,0,0,0,0] },{ \"id\": 1, \"mem\":0, \"data\": [1,1,500,0,0,0,0] }]}";
    
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, json);

  // Test if parsing succeeds
  if (error) {
    std::cerr << "deserializeJson() failed: " << error.c_str() << std::endl;
    return 1;
  }

  for (JsonObject sw_item : doc["sw"].as<JsonArray>()) {

    int swid = sw_item["id"]; // 0, 1
    int swmem = sw_item["mem"]; // 0, 0

    JsonArray sw_item_data = sw_item["data"];
    int savestate = sw_item_data[0]; // 1, 1
    int swmode = sw_item_data[1]; // 1, 1
    int momdur = sw_item_data[2]; // 500, 500
    int pwmfreq = sw_item_data[3]; // 0, 0
    int pwmduty = sw_item_data[4]; // 0, 0
    int blinkdelay = sw_item_data[5]; // 0, 0
    int strobepat = sw_item_data[6]; // 0, 0

    // Print the values
    std::cout << swid << std::endl;
    std::cout << swmem << std::endl;
    std::cout << savestate << std::endl;
    std::cout << swmode << std::endl;
    std::cout << momdur << std::endl;
    std::cout << pwmfreq << std::endl;
    std::cout << pwmduty << std::endl;
    std::cout << blinkdelay << std::endl;
    std::cout << strobepat << std::endl;

  }
  ```
