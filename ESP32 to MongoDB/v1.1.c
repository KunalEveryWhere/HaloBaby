#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

void scanNetworks ();
void connectNetwork ();
void postData (float payload);

// WiFi credentials
const char* ssid = "Kunal's iPhone";                 // update SSID
const char* password = "Advantageous";               // update Password

//MongoDB credentials
String mongoDB_APIKeyName = "DataLog";
String mongoDB_URLEndPoint = "https://data.mongodb-api.com/app/data-dlasa/endpoint/data/v1";

// MongoDB Req Headers
String mongoDB_ContentType = "application/json";
String mongoDB_AccessControlRequestHeaders = "*";
String mongoDB_APIKey = "tIYdbdCDdSSfZy4nK9AX501SdLncPlP0relbClg9FCUesv1VCuHwpeH4oswTIUMs";

//MongoDB Req Payload Structures
String mongoDB_Collection = "ECG";
String mongoDB_Database = "ESP32";
String mongoDB_DataSource = "TTRICluster";

//MongoDB Data Payload
String mongoDB_PayloadName = "ECG_RawData";
float mongoDB_PayloadData = 13.55;

//Others
int HTTPResponseCode;
String StringPayload;


void setup() {
  delay(200);
  Serial.begin(115200);
  delay(200);

  //connect to WiFi
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  Serial.flush();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

void loop() {
  postData(mongoDB_PayloadData);
  Serial.println("Attempting to Connect & Post...");Serial.println();

  //Make post every 2 secs
  delay(2000);
}

void postData (float payload)
{
  //creating JSON Object
  //Elements for creating JSON Object - Check https://arduinojson.org/v6/assistant/
  /*
  we are going for such a structure ->

  {
    dataSource: ..,
    dataBase: ..,
    collection: ..,
    document:
      {
        "ECG_RawData" : 67.3
      }
    }
  */
    //Attempted Conversion into JSON
                          // StaticJsonDocument<256> jsonPayload;

                          // jsonPayload["dataSource"] = mongoDB_DataSource;
                          // jsonPayload["database"] = mongoDB_Database;
                          // jsonPayload["collection"] = mongoDB_Collection;

                          // JsonObject innerObj = jsonPayload.createNestedObject("document");
                          // innerObj[mongoDB_PayloadName] = mongoDB_PayloadData;

                          // //Create formatted JSON and sent to SerialIO
                          // serializeJsonPretty(jsonPayload, Serial);

  StringPayload = "{\"dataSource\":"+mongoDB_DataSource+",\"database\":"+mongoDB_Database+",\"collection\":"+mongoDB_Collection+",\"document\":{"+mongoDB_PayloadName+":"+payload+"}}";  
  Serial.println(StringPayload);

  if(WiFi.status()== WL_CONNECTED){
    HTTPClient http;

    //Begin connect to URL Endpoint AKA MongoDB Server + insertOne Method
    http.begin(mongoDB_URLEndPoint + "/action/insertOne");

    //Adding all the required headers (ref. PostMan for more details)
    http.addHeader("Content-Type", mongoDB_ContentType);
    http.addHeader("Access-Control-Request-Headers", mongoDB_AccessControlRequestHeaders);
    http.addHeader("api-key", mongoDB_APIKey);

    HTTPResponseCode = http.POST(StringPayload);
    Serial.print("HTTP Response Code: ");
    Serial.println(HTTPResponseCode);

    if(HTTPResponseCode == 200) {Serial.println("Post Sucessfull!"); Serial.println("________________________");}
  }
}

void connectNetwork ()
{
  WiFi.begin(ssid, password);
  // Wait for wifi to be connected
  uint32_t notConnectedCounter = 0;
  while (WiFi.status() != WL_CONNECTED) {
      delay(100);
      Serial.println("Wifi connecting...");
      notConnectedCounter++;
      if(notConnectedCounter > 150) { // Reset board if not connected after 5s
          Serial.println("Resetting due to Wifi not connecting...");
          ESP.restart();
      }
  }
  Serial.print("Wifi connected, IP address: ");
  Serial.println(WiFi.localIP());
}

void scanNetworks ()
{
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0) {
      Serial.println("no networks found");
  } else {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
      delay(10);
    }
  }
  Serial.println("");

  // Wait a bit before scanning again
  delay(5000);
}
