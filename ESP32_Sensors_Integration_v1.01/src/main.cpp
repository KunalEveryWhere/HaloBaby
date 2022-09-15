#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <driver/i2s.h>
#include <Wire.h>

//Temperature Sensor TMP36
#define tempOutputPin 12

//MIC Sensor INMP441
#define I2S_WS 34
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_PORT I2S_NUM_0    // Define to use I2S Processor 0
#define bufferLen 64          // Define input buffer length
int16_t sBuffer[bufferLen];   // Initialize the sBuffer

//ECG Sensor AD8232
#define ECG_Low_Positive 16
#define ECG_Low_Negative 17
#define ECG_Output 19
#define PLOTT_DATA        //ECG Manual Data Variables
#define MAX_BUFFER 100
uint32_t prevData[MAX_BUFFER];
uint32_t sumData=0;
uint32_t maxData=0;
uint32_t avgData=0;
uint32_t roundrobin=0;
uint32_t countData=0;
uint32_t period=0;
uint32_t lastperiod=0;
uint32_t millistimer=millis();
double ECG_Frequency;
double beatspermin=0;
uint32_t ECG_New_Data;

//Gyroscope Sensor MPU6050
#define Gyro_SDA 21
#define Gyro_SCL 22
#define Gyro_INT 18
uint8_t Gyro_MPU_ADDR = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

//Function Declarations
float TempData ();
float MicData ();
int ECGDataSimple ();
int ECGDataManual ();
String GyroData ();
void ScanWiFi ();

//Kunal’s iPhone //kennylin0124
const char* ssid = "Kunal’s iPhone";
const char* password =  "12345678";
  
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
  
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  
  if(type == WS_EVT_CONNECT){
  
    Serial.println("Websocket client connection received");
     
  } else if(type == WS_EVT_DISCONNECT){
 
    Serial.println("Client disconnected");
  
  }
}
 
void i2s_install() {
  // Set up I2S Processor configuration
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = bufferLen,
    .use_apll = false
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
} 

void i2s_setpin() {
  // Set I2S pin configuration
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

void ECGFreqDetec() {
  if (countData==MAX_BUFFER) {
   if (prevData[roundrobin] < avgData*1.5 && ECG_New_Data >= avgData*1.5){ // increasing and crossing last midpoint
    period = millis()-millistimer;//get period from current timer value
    millistimer = millis();//reset timer
    maxData = 0;
   }
  }
 roundrobin++;
 if (roundrobin >= MAX_BUFFER) {
    roundrobin=0;
 }
 if (countData<MAX_BUFFER) {
    countData++;
    sumData+=ECG_New_Data;
 } else {
    sumData+=ECG_New_Data-prevData[roundrobin];
 }
 avgData = sumData/countData;
 if (ECG_New_Data>maxData) {
  maxData = ECG_New_Data;
 }

 /* Ask your Ask your cardiologist
 * how to place the electrodes and read the data!
 */
#ifdef PLOTT_DATA
Serial.print(ECG_New_Data);
 Serial.print("\t");
 Serial.print(avgData);
 Serial.print("\t");
 Serial.print(avgData*1.5);
 Serial.print("\t");
 Serial.print(maxData);
 Serial.print("\t");
 Serial.println(beatspermin);
#endif
 prevData[roundrobin] = ECG_New_Data;//store previous value
}

void setup(){
  //begin Serail port @115200
  Serial.begin(115200);



  //pinModes
  //Temp Sensor
  pinMode(tempOutputPin, INPUT);

  //ECG Sensor
  pinMode(ECG_Low_Negative, INPUT);
  pinMode(ECG_Low_Positive, INPUT);
  pinMode(ECG_Output, INPUT);


  // Set up I2S for MIC
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);

  //Set up for Gyro
  Wire.begin(Gyro_SDA, Gyro_SCL, 100000ul); // sda, scl, clock speed(set using unsigned long)
  Wire.beginTransmission(Gyro_MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU−6050)
  Wire.endTransmission(true);


  //Set and connect to WiFi
   WiFi.mode(WIFI_STA);
   ScanWiFi();
   WiFi.begin(ssid, password);
   Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }

  //Setup and open WS
  Serial.print("WebSocket Local IP Address: "); Serial.println(WiFi.localIP());
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
}
  
void loop(){

  float data_temp = TempData ();
  float data_mic = MicData ();
  int data_sECG = ECGDataSimple ();
  int data_mECG = ECGDataManual ();
  String data_gyro = GyroData ();
 
 String seprator = ";";
 String all_sensors_constructed = data_temp + seprator + data_mic + seprator + data_sECG + seprator + data_mECG + seprator + data_gyro + seprator;
 
 ws.textAll(all_sensors_constructed+"\n");                // Send all_sensors_constructed string to WS
 
 delay(100);                  //Check if functioning of all sensors are as predicted or not.
}

float TempData()
{
  int raw_sensor_feed = analogRead(tempOutputPin);        //Input raw value
  float volts = raw_sensor_feed / 1023;                   //Need to check: Normalize data, and use 4095 instead of 1023 since ESP32 is 12-bit system
  float tempC = (volts - 0.5) * 100;                      //Calculate temp. value in Celcius

  return(tempC);
}

float MicData()
{
  // False print statements to "lock range" on serial plotter display
  // Change rangelimit value to adjust "sensitivity"
  int rangelimit = 3000;                        //Kindly look into adjusting sensitivity for within fabric
  Serial.print(rangelimit * -1);
  Serial.print(" ");
  Serial.print(rangelimit);
  Serial.print(" ");

  // Get I2S data and place in data buffer
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);

  //Check before proceeding
  if (result == ESP_OK)
  {
    // Read I2S data buffer
    int16_t samples_read = bytesIn / 8;
    if (samples_read > 0) {
      float mean = 0;
      for (int16_t i = 0; i < samples_read; ++i) {
        mean += (sBuffer[i]);
      }

      // Average the data reading
      mean /= samples_read;

      return (mean);
    }
  }
  else{                   //Check with the second return statement without else-block, and another with no second-return statement.
    return (999999999.99999999999);       //'999999999.99999999999' means that there was an error in Audio
  }

  //ref. for working: https://dronebotworkshop.com/esp32-i2s/
}

int ECGDataSimple()
{
  if((digitalRead(ECG_Low_Positive) == 1) || (digitalRead(ECG_Low_Negative) == 1)){ //check if leads are removed
  return(888888888.111111111);      //'888888888.111111111' means that there was an error in ECG-Simple Data / Leads are removed
  }
  else{
  int raw_sensor_feed = analogRead(ECG_Output);

  delay(10);          //check if delay is okay to place here
  return(raw_sensor_feed);
  }
delay(1);
}

int ECGDataManual (){
  ECG_New_Data = analogRead(34);

  ECGFreqDetec();

  if (period!=lastperiod) {
     ECG_Frequency = 1000/(double)period;//timer rate/period
     if (ECG_Frequency*60 > 20 && ECG_Frequency*60 < 200) { // supress unrealistic Data
      beatspermin=ECG_Frequency*60;
#ifndef PLOTT_DATA
        Serial.print(ECG_Frequency);
        Serial.print(" hz");
        Serial.print(" ");
        Serial.print(beatspermin);
        Serial.println(" bpm");
#endif
        lastperiod=period;
     }
  }

  return(ECG_New_Data);
  delay(5);             //check if delay is okay to place here

  //ref. for manual code: https://nbkomputer.com/ -> GitHub repo
}

String GyroData (){
  Wire.beginTransmission(Gyro_MPU_ADDR);  
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(true);
  Wire.beginTransmission(Gyro_MPU_ADDR);
  Wire.requestFrom(Gyro_MPU_ADDR,  14, (bool)true); // request a total of 14 registers   //check if fix available now
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  String seprator = ";";
  String data_constructed_string = "|" + AcX + seprator + AcY + seprator + AcZ + seprator + Tmp + seprator + GyX + seprator + GyY + seprator + GyZ + "|";

  return(data_constructed_string);

  //ref. for setting: https://www.tutorialspoint.com/esp32_for_iot/interfacing_esp32_with_mpu6050.htm -> GitHub
}

void ScanWiFi()
{
  Serial.println("scan start");

  // WiFi.scanNetworks will return the number of networks found
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
}
