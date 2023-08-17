#include "KVStore.h"
#include "kvstore_global_api.h"
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <SD.h>
#include <TaskScheduler.h>
#include <TinyGPS++.h>

bool developmentMode = false;

BLEDevice central;

BLEService dataService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEIntCharacteristic speedCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214",
                                         BLERead | BLENotify);
BLEIntCharacteristic
    movementCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214",
                           BLERead | BLENotify);

// Setup the incoming data characteristic (RX).
const int RX_BUFFER_SIZE = 256;
bool RX_BUFFER_FIXED_LENGTH = false;

BLEService settingsService(
    "20B20000-E8F2-537E-4F6C-D104768A1214"); // create service for settings
BLECharacteristic
    settingsCharacteristic("20B20001-E8F2-537E-4F6C-D104768A1214",
                           BLEWriteWithoutResponse | BLEWrite | BLERead,
                           RX_BUFFER_SIZE, RX_BUFFER_FIXED_LENGTH);

// Callback methods prototypes
void t1Callback();
void t2Callback();
void t3Callback();
void t4Callback();
void t5Callback();

// Tasks
Task t1(500, TASK_FOREVER, &t1Callback);
Task t2(200, TASK_FOREVER, &t2Callback);
Task t3(4000, TASK_FOREVER, &t3Callback);
Task t4(200, TASK_FOREVER, &t4Callback);
Task t5(1000, TASK_FOREVER, &t5Callback);

Scheduler runner;

File myFile;

// double-cap is the older one
float xx, yy, zz;
float x, y, z;

TinyGPSPlus gps;

UART mySerial(digitalPinToPinName(8), digitalPinToPinName(7), NC, NC);

int res = MBED_ERROR_NOT_READY;
kv_info_t info;

char key_lock_in[16] = {"lock"};

char *key_contact_order[3] = {"telephone1", "telephone2", "telephone3"};

void setLock(char const *value) {
  res = kv_set(key_lock_in, value, strlen(value), 0);
  //Serial.print("Set lock value: [%s], error: %d\n", value, res);
}

// Task no.1: get IMU data
void t1Callback() {
  // Serial.println("t1");
  if (IMU.accelerationAvailable()) {
    updateOldImuValues();
    IMU.readAcceleration(x, y, z);
  }
  delay(1000);
}

void updateOldImuValues() {
  xx = x;
  yy = y;
  zz = z;
}

// Task no.2: get GPS data
void t2Callback() {
  // Serial.println("t2");
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }
  // printGpsInfo();
  yield();
}

// Task no.3: store data in SD
void t3Callback() {
  // Serial.println("t3");
  if (gps.speed.kmph() != 0) {
    myFile = SD.open("mydata.txt", FILE_WRITE);
    if (myFile) {
      myFile.print(gps.date.value());
      myFile.print(" - ");
      myFile.println(gps.time.value());
      char text[100];
      sprintf(text, "X: %f - Y: %f - Z: %f\n", x, y, z);
      myFile.write(text);
      sprintf(text, "Latitude: %f | Longitude: %f | Speed (km/h): %f\n",
              gps.location.lat(), gps.location.lng(), gps.speed.kmph());
      myFile.write(text);
      myFile.println("---");
    } else {
      Serial.println("error opening mydata.txt");
      digitalWrite(LED_BUILTIN, HIGH);
    }
    myFile.close();
  }
}

// Task no.4: send GPS data to BLE device
void t4Callback() {
  BLEDevice central = BLE.central();
  if (central) {
    if (central.connected()) {
      speedCharacteristic.writeValue((int)gps.speed.kmph());
    }
  }
  yield();
}

// Task no.5: When motorbike is lock, check if it's moved
void t5Callback() {
  if (axelMovement(x, xx) || axelMovement(y, yy) || axelMovement(z, zz)) {
    Serial.println("movement!");
    movementCharacteristic.writeValue(1);
    sendSMS();
  } else {
    movementCharacteristic.writeValue(0);
  }
}

bool axelMovement(float newValue, float oldValue) {
  return (abs(newValue - oldValue) >= 0.15);
}

void settingsCharacteristicsWritten(BLEDevice central,
                                    BLECharacteristic characteristic) {
  byte tmp[256];
  int dataLength = settingsCharacteristic.readValue(tmp, 256);

  for (int i = 0; i < dataLength; i++) {
    Serial.print((char)tmp[i]);
  }

  int operation = (int)((char)tmp[0]) - 48;
  if (operation == 0 || operation == 1 || operation == 2) {
    char telephone[10];
    for (int i = 1; i <= 10; i++) {
      telephone[i - 1] = tmp[i];
    }
    storeTelephone(operation, telephone);
  } else if (operation == 3 || operation == 4 || operation == 5) {
    sendTelephone(operation);
  } else if (operation == 9) {
    developmentMode = !developmentMode;
  }
}

void storeTelephone(int contactOrder, char const *telephone) {
  res =
      kv_set(key_contact_order[contactOrder], telephone, strlen(telephone), 0);
  //Serial.println("Set telefone %d in key %s value: %s, error: %d\n",
  //             contactOrder, key_contact_order[contactOrder], telephone, res);
}

void sendTelephone(int operation) {
  int contactOrder = operation - 2;

  // First get the value from the kv_store
  res = kv_get_info(key_contact_order[contactOrder], &info);
  char *value_lock_out = new char[info.size + 1];
  memset(value_lock_out, 0, info.size + 1);
  res = kv_get(key_contact_order[contactOrder], value_lock_out, info.size, 0);

  char message[11];
  itoa(contactOrder, message, 10);
  strcat(message, value_lock_out);

  // Serial.println("kv_get value: %s\n", message);

  // Now send via Bluetooth
  settingsCharacteristic.writeValue(message);
}

void sendSMS() {
  Serial.println("Tratando de enviar SMS");
  mySerial.println("AT+CMGF=1");
  delay(100);
  mySerial.println("AT+CMGS=\"+34653040136\"");
  delay(50);
  mySerial.println("HELLO WORLD");
  delay(50);
  mySerial.println((char)26);
}

void inicializeGPS() {
  Serial.print("Initializing GPS...");
  Serial1.begin(9600);
  while (!Serial1) {
    Serial.println("Failed to initialize GPS!");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
  }
  Serial.println("GPS initialization done.");
}

void inicializeGSM() {
  mySerial.begin(9600);
  while (!mySerial) {
    Serial.println("Failed to initialize GSM!");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
  }

  mySerial.println("AT+CPIN=9102");
  mySerial.println("AT");
  mySerial.println("AT+CMEE=1");

}

void inicializeIMU() {
  Serial.print("Initializing IMU...");
  while (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
  }
  Serial.println("IMU initialization done.");
}

void inicializeSD() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("Failed to initialize SD!");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
  }
  Serial.println("SD initialization done.");
}

void inicializeTasks() {
  runner.init();
  Serial.println("Initialized scheduler");

  runner.addTask(t1);
  Serial.println("Added t1");

  runner.addTask(t2);
  Serial.println("Added t2");

  runner.addTask(t3);
  Serial.println("Added t3");

  runner.addTask(t4);
  Serial.println("Added t4");

  runner.addTask(t5);
  Serial.println("Added t5");

  delay(5000);

  t1.enable();
  Serial.println("Enabled t1");
  t2.enable();
  Serial.println("Enabled t2");
  t3.enable();
  Serial.println("Enabled t3");
  t4.enable();
  Serial.println("Enabled t4");
  t5.enable();
  Serial.println("Enabled t5");
}

void inicializeBLE() {
  Serial.print("Initializing BLE...");
  while (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
  }
  Serial.println("BLE initialization done.");

  // set the local name peripheral advertises
  BLE.setLocalName("MotoGuard");
  BLE.setDeviceName("MotoGuard");

  // set the UUID for the service this peripheral advertises:
  BLE.setAdvertisedService(settingsService);

  // add the services
  dataService.addCharacteristic(speedCharacteristic);
  dataService.addCharacteristic(movementCharacteristic);
  BLE.addService(dataService);

  settingsService.addCharacteristic(settingsCharacteristic);
  BLE.addService(settingsService);

  BLE.setConnectable(true);

  // Bluetooth LE connection handlers.
  settingsCharacteristic.setEventHandler(BLEWritten,
                                         settingsCharacteristicsWritten);
  BLE.setEventHandler(BLEConnected, onBLEConnected);
  BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void onBLEConnected(BLEDevice central) {
  setLock("false");
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void onBLEDisconnected(BLEDevice central) {
  // TODO: no bloquear si está en movimiento? Esperar un rato y comprobar
  setLock("true");
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);

  inicializeGPS();
  inicializeIMU();
  inicializeSD();
  inicializeBLE();
  inicializeGSM();

  inicializeTasks();
}

void loop() { runner.execute(); }

void printImuInfo() { 
  // Serial.print("X: %f - Y: %f - Z: %f\n", x, y, z); 
}

void printGpsInfo() {
  // Latitude in degrees (double)
  Serial.print("Latitude= ");
  Serial.print(gps.location.lat(), 6);
  // Longitude in degrees (double)
  Serial.print(" Longitude= ");
  Serial.println(gps.location.lng(), 6);

  // Raw date in DDMMYY format (u32)
  Serial.print("Raw date DDMMYY = ");
  Serial.println(gps.date.value());

  // Raw time in HHMMSSCC format (u32)
  Serial.print("Raw time in HHMMSSCC = ");
  Serial.println(gps.time.value());

  // Speed in kilometers per hour (double)
  Serial.print("Speed in km/h = ");
  Serial.println(gps.speed.kmph());

  // Altitude in meters (double)
  Serial.print("Altitude in meters = ");
  Serial.println(gps.altitude.meters());

  // Horizontal Dim. of Precision (100ths-i32)
  // 1 Ideal, 1-2 Excellent, 2-5 Good, 5-10 Moderate, 10-20 Fair, >20 Poor
  Serial.print("HDOP = ");
  Serial.println(gps.hdop.value());
}
