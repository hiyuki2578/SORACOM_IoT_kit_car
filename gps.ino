#define __VERSION__ "1.0.0"
#define debug false

#define _LOOP_INTERVAL (10000) /* ms / using in `delay()` in `loop()` */

#define ARDUINOJSON_USE_DOUBLE 1 /* for high precision float */
#include <ArduinoJson.h> /* 5.13.2 (don't use 6.x) */
StaticJsonBuffer<1024> jsonBuffer;

#include <WioLTEforArduino.h>
WioLTE Wio;

#define HUMI_SENSOR_PIN (WIOLTE_D38)

#include <ADXL345.h>          // https://github.com/Seeed-Studio/Accelerometer_ADXL345
ADXL345 Accel;

HardwareSerial* GpsSerial;
#include <TinyGPS++.h>
TinyGPSPlus TinyGPS;
#define lat_1m 0.00000898314861
#define lng_1m 0.000010966382364

float before_lat = 0.0;
float before_lng = 0.0;

#define BUTTON_PIN  (WIOLTE_A6)

#define COLOR_ON  127, 127, 127
#define COLOR_OFF   0,   0,   0
volatile bool State = false;

void setup() {
  SerialUSB.println("Software Version : " + String(__VERSION__));
  SerialUSB.println("Debug mode : " + String(debug));

  /* for Wio LTE */
  Wio.Init();
  Wio.PowerSupplyGrove(true);
  Wio.PowerSupplyLTE(true);
  delay(500);
  DebugPrint("### Turn on or reset.");
  if (!Wio.TurnOnOrReset()) {
    DebugPrint("### ERROR! ###");
    return;
  }
  delay(300);
  DebugPrint("### Connecting to \"soracom.io\".");
  if (!Wio.Activate("soracom.io", "sora", "sora")) {
    DebugPrint("### ERROR! ###");
    return;
  }

  /* for Grove Temp & Humi Sensor */
  TemperatureAndHumidityBegin(HUMI_SENSOR_PIN);

  /* for Grove GPS */
  GpsSerial = &Serial;
  GpsSerial->begin(9600);
  while(!GpsSerial->available());
  DebugPrint("GPS ready");

  /* for Grove Accelerometer */
  Accel.powerOn();

  /* for Grove Button */
  pinMode(BUTTON_PIN, INPUT);
  attachInterrupt(BUTTON_PIN, change_state, RISING);
}

void loop() {
  float lat_diff,lng_diff,latitude,longitude;
  /* required ArduinoJson */
  JsonObject& json = jsonBuffer.createObject();
  add_geo_timestamp_by_gps(json,latitude,longitude);
  TemperatureAndHumidityRead(json);
  read_accel(json);
  char payload[1024];
  json.printTo(payload, sizeof(payload));
  String geo_src = json["geo_src"].asString();
  DebugPrint(payload);
  if(geo_src.compareTo("NONE") != 0){
    lat_diff = abs(latitude - before_lat) / lat_1m;
    lng_diff = abs(longitude - before_lng) / lng_1m;
    if(lat_diff + lng_diff > 5){
    //if(true){
      if(!debug){
        send_to_soracom_with_ondemand(payload);
      }else{
        DebugPrint(payload);
      }
      before_lat = latitude;
      before_lng = longitude;
    }
  }else{
    if (State) {
      Wio.LedSetRGB(COLOR_ON);
    }
    DebugPrint("before_lat :" + String(before_lat) + "\nbefore_lng :" + String(before_lng));
    DebugPrint("NO GPS FIX");
  }
  jsonBuffer.clear();
  delay(100);
  Wio.LedSetRGB(COLOR_OFF);
  delay(_LOOP_INTERVAL - 100);
}

void send_to_soracom_with_ondemand(const char *payload) {
  SerialUSB.println("send_to_soracom_with_ondemand()");
  /* required WioLTEforArduino */
  const char *host = "http://harvest.soracom.io";
  int status;
  if (!Wio.HttpPost(host, payload, &status)) {
    DebugPrint("### ERROR! ###");
  }else{
    DebugPrint(String(status));
  }
}

void add_geo_timestamp_by_gps(JsonObject &json,float &latitude,float &longitude) {
  /* required ArduinoJson, TinyGPS++ */
  int retry_cnt = 0;
  gps_parse_retry:
  while(GpsSerial->available()) TinyGPS.encode(GpsSerial->read());  
  if (!TinyGPS.location.isValid()) {
    if (retry_cnt > 1000) {
      json["geo_src"] = "NONE";
      return;
    }
    retry_cnt++;
    goto gps_parse_retry;
  }
  json["geo_src"] = "GPS";
  json["lat"] = TinyGPS.location.lat();
  latitude = TinyGPS.location.lat();
  json["lng"] = TinyGPS.location.lng();
  longitude = TinyGPS.location.lng();
  json["alt_m"] = TinyGPS.altitude.meters();
  json["speed_kmh"] = TinyGPS.speed.kmph();
  json["course"] = TinyGPS.course.deg();
  json["satellites"] = TinyGPS.satellites.value();
  struct tm t;
  t.tm_year = TinyGPS.date.year() - 1900;
  t.tm_mon  = TinyGPS.date.month() - 1;
  t.tm_mday = TinyGPS.date.day();
  t.tm_hour = TinyGPS.time.hour();
  t.tm_min  = TinyGPS.time.minute();
  t.tm_sec  = TinyGPS.time.second();
  t.tm_isdst= -1;
  time_t epoch = mktime(&t);
  json["timestamp"] = epoch;
}

void read_accel(JsonObject &json)
{
  int x,y,z;
  Accel.readXYZ(&x, &y, &z);
  json["accel_x"] = x;
  json["accel_y"] = y;
  json["accel_z"] = z;
}

void change_state()
{
  State = !State;
  DebugPrint(String(State));
  if(State){
    Wio.LedSetRGB(0,255,0);
  }else{
    Wio.LedSetRGB(255,0,0);
  }
}

//////////////
int TemperatureAndHumidityPin;


void TemperatureAndHumidityBegin(int pin)
{
  TemperatureAndHumidityPin = pin;
  DHT11Init(TemperatureAndHumidityPin);
}

void TemperatureAndHumidityRead(JsonObject &json)
{
  byte data[5];
  float humidity;
  float temperature;
  
  DHT11Start(TemperatureAndHumidityPin);
  for (int i = 0; i < 5; i++) data[i] = DHT11ReadByte(TemperatureAndHumidityPin);
  DHT11Finish(TemperatureAndHumidityPin);

  humidity = (float)data[0] + (float)data[1] / 10.0f;
  temperature = (float)data[2] + (float)data[3] / 10.0f;
  json["temp"] = temperature;
  json["humidity"] = humidity;
}
//////////////////////

void DebugPrint(String s){
  if(debug){
    SerialUSB.println(s);
  }
}

void DHT11Init(int pin)
{
  digitalWrite(pin, HIGH);
  pinMode(pin, OUTPUT);
}

void DHT11Start(int pin)
{
  // Host the start of signal
  digitalWrite(pin, LOW);
  delay(18);
  
  // Pulled up to wait for
  pinMode(pin, INPUT);
  while (!digitalRead(pin)) ;
  
  // Response signal
  while (digitalRead(pin)) ;
  
  // Pulled ready to output
  while (!digitalRead(pin)) ;
}

byte DHT11ReadByte(int pin)
{
  byte data = 0;
  
  for (int i = 0; i < 8; i++) {
    while (digitalRead(pin)) ;

    while (!digitalRead(pin)) ;
    unsigned long start = micros();

    while (digitalRead(pin)) ;
    unsigned long finish = micros();

    if ((unsigned long)(finish - start) > 50) data |= 1 << (7 - i);
  }
  
  return data;
}

void DHT11Finish(int pin)
{
  // Releases the bus
  while (!digitalRead(pin)) ;
  digitalWrite(pin, HIGH);
  pinMode(pin, OUTPUT);
}

bool DHT11Check(const byte* data, int dataSize)
{
  if (dataSize != 5) return false;

  byte sum = 0;
  for (int i = 0; i < dataSize - 1; i++) {
    sum += data[i];
  }

  return data[dataSize - 1] == sum;
}
////////////////////
