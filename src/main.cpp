#include <Arduino.h>
//#include <splash.h>
//#include <Adafruit_SSD1306.h>

/*
   -Based on Neil Kolban examples for IDF: https://github.com/nkolban/esp32-snippets/master/cpp_util
   -Ported to Arduino ESP32 by Evandro Copercini
   -Modified for DirtyLaundry by BiatuAutMiahn
*/

/*
 TODO
 -Implement Machine Timer
 -If Machine not isMine, and Not found in scan for 5000ms, remove from MyMachines.
*/

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <BLEAddress.h>
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEUtils.h>
#include <CRC32.h>
#include <SPI.h>
#include <Wire.h>

#include <array>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "DirtyLaundry.h"
#include "icons.h"

//#include "consola8pt.h"
#include "mbedtls/md.h"
#include "soc/rtc_wdt.h"

//#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 10000ULL /* Time ESP32 will go to sleep (in seconds) */

//#define OLED_ADDRESS 0x3D
//#define SCREEN_WIDTH 128
//#define SCREEN_HEIGHT 64
//#define OLED_RESET_PIN -1
//Adafruit_SSD1306 display(128, 64, &Wire, -1);

TaskHandle_t BootAnim;

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
GFXcanvas1 menu_canvas(MENU_WIDTH, MENU_HEIGHT);
GFXcanvas1 status_canvas(STATUS_WIDTH, STATUS_HEIGHT);
GFXcanvas1 frame_canvas(OLED_WIDTH, OLED_HEIGHT);
GFXcanvas1 canvas(OLED_WIDTH, OLED_HEIGHT);

//std::vector<uint8_t *> rxBuffer;

CRC32 crc;
BLERemoteService *machineRS;
BLERemoteCharacteristic *machineRespRC;
BLERemoteCharacteristic *machineTxRC;
BLERemoteCharacteristic *machineRxRC;
BLEScan *pBLEScan;
BLEClient *pClient;
mbedtls_md_context_t ctx;
Machine *myMachine;
bool isScanning = false;
bool isConnected = false;
bool updateStatus = false;
bool updateDisplay = false;
bool updateMenu = false;
bool isCharging = false;
bool statBlink = false;
bool isSleeping = false;
bool isDim = false;
uint8_t statBattery = 0;
uint8_t iSpin = 0;
int levelBattery = 0;
int lowestBattery = 0;
int highestBattery = 0;
int avgBattery[BATT_AVG];
int offsBattery;
unsigned long timerBlink = 0;
unsigned long testTimer = 0;
unsigned long batteryTimer = 0;
unsigned long batteryDiffTimer = 0;
unsigned long sleepTimer = 0;
uint8_t testLevel = 0;
uint8_t menuSelect = 0;
uint8_t bState;
uint8_t bStateLast;
const char *statusMsg;
bool screenScan = true;
uint8_t iSignal = 0;

//class MyDeviceCallbacks: public BLEClientCallbacks {
//  void onConnect(BLEClient device) {
//
//  }
//}

void print_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            Serial.println("Wakeup caused by external signal using RTC_IO");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            Serial.println("Wakeup caused by external signal using RTC_CNTL");
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("Wakeup caused by timer");
            break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD:
            Serial.println("Wakeup caused by touchpad");
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            Serial.println("Wakeup caused by ULP program");
            break;
        default:
            Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
            break;
    }
}

void printMachine(Machine &m) {
    Serial.printf("index: %d \n", m.index);
    Serial.printf("name: %s \n", m.name.c_str());
    Serial.printf("address: %s \n", m.addr.toString().c_str());
    Serial.printf("rssi: %d \n", m.rssi);
    Serial.printf("pong: %d \n", m.pong);
    Serial.printf("status: %s \n", m.status.c_str());
    Serial.printf("isUpper: %s \n", m.isUpper ? "true" : "false");
    Serial.printf("isAvailable: %s \n", m.isAvailable ? "true" : "false");
    Serial.printf("isRunning: %s \n", m.isRunning ? "true" : "false");
    Serial.printf("isBusy: %s \n", m.isBusy ? "true" : "false");
    Serial.printf("isOffline: %s \n", m.isOffline ? "true" : "false");
    Serial.printf("minPrice: %d \n", m.minPrice);
    Serial.printf("maxPrice: %d \n", m.maxPrice);
    Serial.printf("minPriceDollars: %.2f \n", m.minPriceDollars);
    Serial.printf("maxPriceDollars: %.2f \n", m.maxPriceDollars);
    Serial.printf("basePrice: %d \n", m.basePrice);
    Serial.printf("superCyclePrice: %d \n", m.superCyclePrice);
    Serial.printf("superCyclePossible: %s \n", m.superCyclePossible ? "true" : "false");
    Serial.printf("singleTopOffPossible: %s \n", m.singleTopOffPossible ? "true" : "false");
    Serial.printf("minutesLeft: %d \n", m.minutesLeft);
    Serial.printf("additionalVend: %s \n", m.additionalVend ? "true" : "false");
    Serial.printf("superCyclePrice: %d \n", m.superCyclePrice);
    Serial.printf("checkMachineStatus: %d \n", m.checkMachineStatus);
    Serial.printf("isTopOffAvailable: %s \n", m.isTopOffAvailable ? "true" : "false");
    Serial.printf("isMachineRunning: %s \n", m.isMachineRunning ? "true" : "false");
    Serial.printf("priceRangeAllowedFor: %d \n\n", m.priceRangeAllowedFor);
}

void scanMachines() {
    Serial.println("Scanning for Machines...");
    pBLEScan->start(BLESCANTIME, false);
    Serial.println("Scanning for Machines...done");
    pBLEScan->clearResults();  // delete results fromBLEScan buffer to release memory
    Serial.println(myMachines.size());
    Serial.println(ESP.getFreeHeap());
}

static void scanCompleteCB(BLEScanResults scanResults) {
}

void startScan() {
    Serial.println("Starting Scan...");
    if (isScanning) {
        Serial.println("Starting Scan...Failed, Already Scanning.");
        return;
    }
    pBLEScan->start(0, scanCompleteCB);
    isScanning = true;
    Serial.println("Starting Scan...Success");
}

void stopScan() {
    Serial.println("Stopping Scan...");
    if (!isScanning) {
        Serial.println("Stopping Scan...Failed, Not Scanning.");
        return;
    }
    pBLEScan->stop();
    pBLEScan->clearResults();  // delete results fromBLEScan buffer to release memory
    isScanning = false;
    Serial.println("Stopping Scan...Success");
}

std::string uint8_to_hex_string(const uint8_t *v, const size_t s) {
    std::stringstream ss;

    ss << std::hex << std::setfill('0');

    for (int i = 0; i < s; i++) {
        ss << std::hex << std::setw(2) << static_cast<int>(v[i]);
    }

    return ss.str();
}

void printHex(uint8_t num) {
    char hexCar[2];

    sprintf(hexCar, "%02X", num);
    Serial.print(hexCar);
}

void handleResponse(uint8_t *pData, size_t length) {
    uint8_t retStatus = (uint8_t)(pData[0] &= 0x7F);
    uint8_t retError = (uint8_t)pData[1];
    //std::string retValue = pData;
    mState = retStatus;
    // Serial.printf("\nretValue:%s\nretStatus: %d, retError: %d\n", uint8_to_hex_string((const uint8_t *)pData, length).c_str(), retStatus, retError);
    if (retError != 0x00) {
        //disconnectMachine(); ?
        return;  // Failure!
    }
    if (retStatus == MCMD_GET_AUTH_CHALLENGE_DATA[0]) {
        //Serial.println("Authenticating Machine...");
        byte hmacResult[32];
        mbedtls_md_init(&ctx);
        mbedtls_md_setup(&ctx, hashInfo, 1);
        mbedtls_md_hmac_starts(&ctx, cKey, 16);
        mbedtls_md_hmac_update(&ctx, &pData[2], 16);
        mbedtls_md_hmac_finish(&ctx, hmacResult);
        mbedtls_md_free(&ctx);
        memset(respBuffer, 0, RESP_BUF_SIZE);
        memcpy(respBuffer, (const uint8_t *)MCMD_GET_AUTH_UNLOCK, sizeof(uint8_t));
        memcpy(respBuffer + sizeof(uint8_t), hmacResult, 16);
        // Serial.print("uuidResponse -> ");
        // for (int i = 0; i < RESP_BUF_SIZE; i++) {
        //     printHex(respBuffer[i]);
        // }
        // Serial.println();
    } else if (retStatus == MCMD_GET_AUTH_UNLOCK[0]) {
        //Serial.println("Authenticating Machine...Success");
    } else if (retStatus == MCMD_READ_READY[0]) {
        txLength = pData[2] | (pData[3] << 8);
        memset(txBuffer, 0, TX_BUF_SIZE);
        txOffset = 0;
    } else if (retStatus == MCMD_READ_START[0]) {
        return;
    } else if (retStatus == MCMD_READ_DATA[0]) {
        return;
    } else if (retStatus == MCMD_WRITE_START[0]) {
        if (rxLength == 0) {
            return;
        }
        // Serial.printf("%d,%d, uuidRx -> ", rxOffset, rxLength);
        // for (int i = 0; i < MIO_PAYLOAD + 1; i++) {
        //     printHex(rxBuffer[rxOffset + i]);
        // }
        // Serial.println();
        machineRxRC->writeValue(&rxBuffer[rxOffset], MIO_PAYLOAD + 1);
        rxOffset += MIO_PAYLOAD + 1;
    } else if (retStatus == MCMD_WRITE_DATA[0]) {
    } else if (retStatus == MCMD_RESET[0]) {
    } else {
    }
}

void writeResponse(uint8_t *cmd, size_t len) {
    // Serial.printf("uuidResponse -> ");
    // for (int i = 0; i < len; i++) {
    //     printHex(cmd[i]);
    // }
    // Serial.println();
    machineRespRC->writeValue(cmd, len, true);
    std::string retValue = machineRespRC->readValue();
    std::vector<uint8_t> vRet(retValue.begin(), retValue.end());
    uint8_t *retBuf = &vRet[0];
    handleResponse(retBuf, retValue.length());
}

void writeRx(uint8_t *cmd, size_t len) {
    crc.reset();
    rxOffset = 0;
    rxLength = 0;
    for (size_t i = 0; i < len; i++) {
        crc.update(cmd[i]);
    }
    uint32_t crc32 = crc.finalize();
    memset(respBuffer, 0, RESP_BUF_SIZE);
    memcpy(respBuffer, MCMD_WRITE_START, sizeof(uint8_t));
    memcpy(respBuffer + sizeof(uint8_t), (char *)&len, sizeof(uint16_t));
    memcpy(respBuffer + sizeof(uint8_t) + sizeof(uint16_t), (char *)&crc32, sizeof(uint32_t));
    uint8_t ic = 0;
    uint16_t io = 0;
    uint16_t c = len / MIO_PAYLOAD;
    uint16_t r = len % MIO_PAYLOAD;
    if (c) {
        for (uint16_t i = 0; i < c; i++) {
            memcpy(&rxBuffer[rxOffset], (char *)&ic, sizeof(uint8_t));
            memcpy(&rxBuffer[rxOffset + sizeof(uint8_t)], &cmd[io], MIO_PAYLOAD);
            io += MIO_PAYLOAD;
            rxOffset += MIO_PAYLOAD + sizeof(uint8_t);
            ic++;
        }
        if (r) {
            memcpy(&rxBuffer[rxOffset], (char *)&ic, sizeof(uint8_t));
            memcpy(&rxBuffer[rxOffset + sizeof(uint8_t)], &cmd[io], r);
            rxOffset += r + sizeof(uint8_t);
        }
    }
    rxLength = rxOffset;
    rxOffset = 0;
    // Serial.printf("uuidRx -> ");
    // for (int i = 0; i < RESP_BUF_SIZE; i++) {
    //     printHex(rxBuffer[i]);
    // }
    // Serial.println();
    writeResponse(respBuffer, 7);
    while (mState != MCMD_WRITE_START[0]) {
        delay(1);
    }
    while (mState != MCMD_WRITE_DATA[0]) {
        delay(1);
    }
    while (mState == MCMD_WRITE_DATA[0]) {
        if (rxOffset < rxLength) {
            // Serial.printf("%d,%d, uuidRx -> ", rxOffset, rxLength);
            if (rxLength - rxLength >= MIO_PAYLOAD + 1) {
                // for (int i = 0; i < MIO_PAYLOAD + 1; i++) {
                //     printHex(rxBuffer[rxOffset + i]);
                // }
                machineRxRC->writeValue(&rxBuffer[rxOffset], MIO_PAYLOAD + 1);
            } else {
                // for (int i = 0; i < rxLength - rxOffset; i++) {
                //     printHex(rxBuffer[rxOffset + i]);
                // }
                machineRxRC->writeValue(&rxBuffer[rxOffset], rxLength - rxOffset);
            }
            // Serial.println();

            rxOffset += MIO_PAYLOAD + 1;
        }
        delay(1);
    }
    // Serial.println(mState);
    while (mState != 3) {
        delay(1);
    }
    while (mState == 3) {
        writeResponse((uint8_t *)MCMD_READ_START, 1);
    }
    while (txOffset < txLength) {
        delay(1);
    }
    // Serial.printf("uuidTx <- ");
    // for (int i = 0; i < txLength; i++) {
    //     printHex(txBuffer[i]);
    // }
    // Serial.println();
    // Serial.printf("resp: ");
    for (int i = 0; i < txLength; i++) {
        Serial.printf("%c", txBuffer[i]);
    }
    Serial.println();
}

//     retValue = uint8_to_hex_string((const uint8_t *)txBuffer, txLength).c_str();
//     Serial.printf("uuidTx <- %s\n", retValue.c_str());
//     return true;
// }

static void responseCallback(BLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
    // Serial.printf("Notify callback for response characteristic %s of data length %d\n",
    //               pCharacteristic->getUUID().toString().c_str(), length);
    if (mState == MCMD_WRITE_DATA[0] || mState == MCMD_WRITE_START[0] || mState == MCMD_READ_READY[0]) {
        handleResponse(pData, length);
    }
}
static void rxCallback(BLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
    // Serial.printf("Notify callback for RX characteristic %s of data length %d\n",
    //               pCharacteristic->getUUID().toString().c_str(), length);
}
static void txCallback(BLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
    // Serial.printf("Notify callback for TX characteristic %s of data length %d\n",
    //               pCharacteristic->getUUID().toString().c_str(), length);
    memcpy(&txBuffer[txOffset], pData + 1, length - 1);
    txOffset += length - 1;
}

bool disconnectMachine(bool noScan = true) {
    Serial.println("Disconnecting Machine...");
    if (!isConnected || myMachine == nullptr) {
        Serial.println("Disconnecting Machine...Failed, Not Connected.");
        return false;
    }
    if (!myMachine->isConnected) {
        Serial.println("Disconnecting Machine...Failed, Not Connected.");
        return false;
    }

    pClient->disconnect();
    myMachine->isConnected = false;
    isConnected = false;
    myMachine = nullptr;
    Serial.println("Disconnecting Machine...Success");
    if (!isScanning) {
        startScan();
    }
    return true;
}

bool rssiMachine() {
    if (!isConnected || myMachine == nullptr) {
        return false;
    }
    if (!myMachine->isConnected) {
        return false;
    }
    myMachine->rssi = pClient->getRssi();
    return true;
}

bool connectMachine(int m) {  //bool connectMachine(int iMachine){
    Serial.println("Connecting to Machine...");
    if (isConnected) {
        Serial.println("Connecting to Machine...Failed, Already Connected.");
        return true;
    }
    if (myMachine != nullptr) {
        if (myMachine->isConnected) {
            isConnected = true;
            Serial.println("Connecting to Machine...Failed, Already Connected.");
            return true;
        }
    }
    if (isScanning) {
        stopScan();
    }
    if (myMachine == nullptr) {
        myMachine = &myMachines[m];
    }
    pClient = BLEDevice::createClient();
    unsigned long timeout = millis();
    uint8_t tries = 0;
    while (!pClient->isConnected() && tries < 5) {
        pClient->connect(&myMachine->device);
        while (!pClient->isConnected()) {
            if (millis() - timeout >= 1000) {
                timeout = millis();
                tries++;
                Serial.printf("Connecting to Machine...Timed out, Retrying, Try: %d\n", tries);
                break;
            }
            delay(125);
        }
    }
    if (!pClient->isConnected()) {
        // Serial.printf("Connecting to Machine...Failed after %d tries.\n", tries);
        Serial.printf("Connecting to Machine...Failed, Timed out.\n");
        disconnectMachine();
        return false;
    }
    machineRS = pClient->getService(uuidService);
    timeout = millis();
    tries = 0;
    while (machineRS == nullptr && tries < 5) {
        machineRS = pClient->getService(uuidService);
        while (machineRS == nullptr) {
            if (millis() - timeout >= 1000) {
                timeout = millis();
                tries++;
                Serial.printf("Connecting to Machine...Timed out searching for Service UUID, Retrying, Try: %d\n", tries);
                break;
            }
            delay(125);
        }
    }
    if (machineRS == nullptr) {
        Serial.println("Connecting to Machine...Failed, Cannot find Service UUID");
        disconnectMachine();
        return false;
    }
    std::string value;
    machineRespRC = machineRS->getCharacteristic(uuidResponse);
    if (machineRespRC == nullptr) {
        Serial.println("Connecting to Machine...Failed, Cannot find Response Characteristic UUID");
        disconnectMachine();
        return false;
    }
    machineRespRC->registerForNotify(responseCallback);
    value = machineRespRC->readValue();
    machineRxRC = machineRS->getCharacteristic(uuidRX);
    if (machineRxRC == nullptr) {
        Serial.println("Connecting to Machine...Failed, Cannot find RX Characteristic UUID");
        disconnectMachine();
        return false;
    }
    machineRxRC->registerForNotify(rxCallback);
    value = machineRxRC->readValue();
    machineTxRC = machineRS->getCharacteristic(uuidTX);
    if (machineTxRC == nullptr) {
        Serial.println("Connecting to Machine...Failed, Cannot find TX Characteristic UUID");
        disconnectMachine();
        return false;
    }
    machineTxRC->registerForNotify(txCallback);
    value = machineTxRC->readValue();
    Serial.println("Connecting to Machine...Authenticating");
    writeResponse((uint8_t *)MCMD_GET_AUTH_CHALLENGE_DATA, 1);
    while (mState != 1) {
        delay(1);
    }
    writeResponse((uint8_t *)respBuffer, 17);
    Serial.println("Connecting to Machine...Authenticated");
    myMachine->isConnected = true;
    isConnected = true;
    Serial.println("Connecting to Machine...Success");
    return true;
}

bool inquiryMachine() {
    if (!isConnected) {
        return false;
    }
    std::string msg = "<statInq>0</statInq>";
    std::vector<uint8_t> vRet(msg.begin(), msg.end());
    uint8_t *retBuf = &vRet[0];
    writeRx(retBuf, msg.length());
    return true;
}

bool startMachine() {
    if (!isConnected) {
        return false;
    }
    char retBuf[64];
    size_t len = sprintf(retBuf, "<begin><ref>%d</ref><bal>%d</bal></begin>", 1531932775, myMachine->maxPrice);
    // std::string msg = "";
    // std::vector<uint8_t> vRet(msg.begin(), msg.end());
    // uint8_t *retBuf = &vRet[0];
    writeRx((uint8_t *)retBuf, len);
    return true;
}

void drawBattery(int16_t x, int16_t y, uint8_t l = 0) {
    int16_t byteWidth = (I_BATTERY_WIDTH + 7) / 8;
    uint8_t byte = 0;
    for (int16_t j = 0; j < I_BATTERY_HEIGHT; j++, y++) {
        for (int16_t i = 0; i < I_BATTERY_WIDTH; i++) {
            if (i & 7)
                byte <<= 1;
            else {
                byte = pgm_read_byte(&i_battery[j * byteWidth + i / 8]);
                if (l) {
                    if (j > 1 && j < 8) {
                        if (i == 0) {
                            if (l > 0)
                                byte |= i_charge[0];
                            if (l > 1)
                                byte |= i_charge[1];
                        }
                        if (i > 0) {
                            if (l > 2)
                                byte |= i_charge[2];
                            if (l > 3)
                                byte |= i_charge[3];
                        }
                    }
                }
            }
            if (byte & 0x80)
                status_canvas.writePixel(x + i, y, SH110X_WHITE);
        }
    }
}

void drawSignal(uint16_t x, uint16_t y, int rssi, bool invert = false) {
    if (rssi >= -70) {
        iSignal = 3;
    }
    if (rssi < -70 && rssi >= -82) {
        iSignal = 2;
    }
    if (rssi < -82 && rssi >= -94) {
        iSignal = 1;
    }
    if (rssi < -94) {
        iSignal = 0;
    }
    if (invert) {
        status_canvas.drawBitmap(x, y, i_signal[iSignal], I_SIGNAL_WIDTH, I_SIGNAL_HEIGHT, SH110X_BLACK, SH110X_WHITE);
    } else {
        status_canvas.drawBitmap(x, y, i_signal[iSignal], I_SIGNAL_WIDTH, I_SIGNAL_HEIGHT, SH110X_WHITE, SH110X_BLACK);
    }
}

void drawStatus() {
    status_canvas.startWrite();
    status_canvas.fillScreen(SH110X_BLACK);
    //status_canvas.drawBitmap(2, 2, i_tglyph, I_TGLYPH_WIDTH, I_TGLYPH_HEIGHT, SH110X_WHITE);
    status_canvas.setCursor(2, 3);
    status_canvas.print(statusMsg);
    if (isCharging) {
        status_canvas.drawBitmap(STATUS_WIDTH - I_BATTERY_WIDTH - I_LIGHTNING_WIDTH - 2, 2, i_lightning, I_LIGHTNING_WIDTH, I_LIGHTNING_HEIGHT, SH110X_WHITE);
        if (millis() - timerBlink >= 500) {
            timerBlink = millis();
            if (statBlink) {
                statBlink = false;
            } else {
                statBlink = true;
            }
        }
    } else {
        if (lowestBattery <= BATT_0) {
            status_canvas.drawBitmap(STATUS_WIDTH - I_BATTERY_WIDTH - I_EXCLAIM_WIDTH - 2, 2, i_exclaim, I_EXCLAIM_WIDTH, I_EXCLAIM_HEIGHT, SH110X_WHITE);
            if (millis() - timerBlink >= 1000) {
                timerBlink = millis();
                if (statBlink) {
                    statBlink = false;
                } else {
                    statBlink = true;
                }
            }
            // } else {
            //     if (levelBattery >= BATT_USB) {
            //         status_canvas.drawBitmap(STATUS_WIDTH - I_BATTERY_WIDTH - I_LIGHTNING_WIDTH - 2, 2, i_lightning, I_LIGHTNING_WIDTH, I_LIGHTNING_HEIGHT, SH110X_WHITE);
            //     }
        }
    }
    // if (lowestBattery > BATT_100) {
    //     statBattery = 4;  // 4.2
    // } else if (lowestBattery < BATT_100 && lowestBattery > BATT_75) {
    //     statBattery = 3;  // 3.975
    // } else if (lowestBattery < BATT_75 && lowestBattery > BATT_50) {
    //     statBattery = 2;  // 3.75
    // } else if (lowestBattery < BATT_50 && lowestBattery > BATT_25) {
    //     statBattery = 1;  // 3.525
    // } else if (lowestBattery < BATT_25 && lowestBattery > BATT_0) {
    //     statBattery = 0;  // 3.3v
    // }
    // if (statBattery == 0 || lowestBattery <= BATT_0) {
    //     if (statBlink) {
    //         drawBattery(STATUS_WIDTH - I_BATTERY_WIDTH - 2, 2, 0);
    //     }
    // } else {
    //     drawBattery(STATUS_WIDTH - I_BATTERY_WIDTH - 2, 2, statBlink ? statBattery - 1 : statBattery);
    // }
    //drawBattery(STATUS_WIDTH - I_BATTERY_WIDTH - 2, 2, statBlink ? statBattery - 1 : statBattery);
    // statBattery = 4;
    // drawBattery(STATUS_WIDTH - I_BATTERY_WIDTH - 2, 2, statBlink ? statBattery - 1 : statBattery);
    if (statBlink) {
        drawBattery(STATUS_WIDTH - I_BATTERY_WIDTH - 2, 2, 0);
    }
    if (!isCharging || levelBattery <= BATT_USB) {
        uint8_t iTimer = int(floor(float(25) * (float(millis() - sleepTimer) / SLEEP_TIMER)));
        status_canvas.drawBitmap(STATUS_WIDTH - I_BATTERY_WIDTH - I_LIGHTNING_WIDTH - I_TIMEOUT_WIDTH - 2, -1, i_timeout[iTimer], I_TIMEOUT_WIDTH, I_TIMEOUT_HEIGHT, SH110X_WHITE, SH110X_BLACK);
        status_canvas.drawBitmap(STATUS_WIDTH - I_BATTERY_WIDTH - I_LIGHTNING_WIDTH - I_TIMEOUT_WIDTH - I_TIMEOUT_Zz_WIDTH, 2, i_timeout_Zz, I_TIMEOUT_Zz_WIDTH, I_TIMEOUT_Zz_HEIGHT, SH110X_WHITE, SH110X_BLACK);
    }
    if (screenScan) {
        status_canvas.drawBitmap(64, -1, i_spin[iSpin], I_SPIN_WIDTH, I_SPIN_HEIGHT, SH110X_WHITE, SH110X_BLACK);
        iSpin++;
        if (iSpin > 13) {
            iSpin = 0;
        }
    } else {
        status_canvas.fillRect(64, -1, I_SPIN_WIDTH, I_SPIN_HEIGHT, SH110X_BLACK);
        drawSignal(70, 3, myMachines[menuSelect].rssi);
    }
    status_canvas.endWrite();
    canvas.drawBitmap(0, 0, status_canvas.getBuffer(), STATUS_WIDTH, STATUS_HEIGHT, SH110X_WHITE, SH110X_BLACK);
}

void drawDisplay() {
    drawStatus();
    canvas.drawBitmap(0, 0, frame_canvas.getBuffer(), OLED_WIDTH, OLED_HEIGHT, SH110X_WHITE);
    canvas.drawBitmap(2, STATUS_HEIGHT, menu_canvas.getBuffer(), MENU_WIDTH - 4, MENU_HEIGHT - 4, SH110X_WHITE, SH110X_BLACK);
    display.drawBitmap(0, 0, canvas.getBuffer(), OLED_WIDTH, OLED_HEIGHT, SH110X_WHITE, SH110X_BLACK);
    display.display();
}

// void BootAnimator(void *pvParameters) {
//     uint8_t acc = 1;
//     for (;;) {
//         for (uint8_t i = 0; i < 7; i++) {
//             display.drawBitmap(82, 9, i_washer, I_WASHER_WIDTH, I_WASHER_HEIGHT, SH110X_WHITE, SH110X_BLACK);
//             //display.drawBitmap(91, 26, i_washer_mask, I_WASHER_MASK_WIDTH, I_WASHER_MASK_HEIGHT, SH110X_BLACK);
//             display.drawBitmap(90, 25, i_glyph[i], I_GLYPH_WIDTH, I_GLYPH_HEIGHT, SH110X_WHITE);
//             display.display();
//             //delay(50);
//             vTaskDelay((100 / acc) / portTICK_PERIOD_MS);
//         }
//         acc++;
//     }
// }

void battCalc() {
    for (int i = 0; i < BATT_AVG - 1; i++) {
        levelBattery += avgBattery[i];
    }
    levelBattery = levelBattery / BATT_AVG;
    if (levelBattery < lowestBattery) {
        if (isCharging && (lowestBattery - levelBattery) > 1) {
            isCharging = false;
            highestBattery = levelBattery;
        }
        lowestBattery = levelBattery;
    }
    if (lowestBattery > highestBattery) {
        highestBattery = lowestBattery;
    }
    if (!isCharging) {
        if ((levelBattery - highestBattery) > 2) {
            isCharging = true;
            highestBattery = levelBattery;
            lowestBattery = levelBattery;
        }
    } else {
        if (levelBattery > highestBattery) {
            lowestBattery = levelBattery;
            highestBattery = levelBattery;
        }
    }
}

int o = 0;
char s[256];
char c[] = {0, 0};
size_t i;
void setup() {
    int rawValue = analogRead(A13);
    lowestBattery = 4095;
    highestBattery = 0;
    // Battery is useless at 1840
    //setCpuFrequencyMhz(80);
    Serial.begin(115200);
    pinMode(A13, INPUT);
    pinMode(BUTTON_A, INPUT_PULLUP);
    pinMode(BUTTON_B, INPUT_PULLUP);
    pinMode(BUTTON_C, INPUT_PULLUP);
    display.begin(0x3C, true);
    display.setRotation(1);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.clearDisplay();
    if (rawValue <= BATT_CRITICAL) {
        display.oled_command(SH110X_DISPLAYOFF);
        for (;;) {
            delay(1);
        };
    }
    //1850 min value
    if (!digitalRead(BUTTON_A) || rawValue < BATT_TOO_LOW) {
        levelBattery = 0;
        display.setContrast(1);
        for (;;) {
            offsBattery = 0;
            while (offsBattery < BATT_AVG - 1) {
                avgBattery[offsBattery] = analogRead(A13);
                offsBattery++;
            }
            if (millis() - batteryTimer >= 1000) {
                batteryTimer = millis();
                battCalc();
                display.clearDisplay();
                display.drawBitmap(40, 8, i_badbatt, I_BADBATT_WIDTH, I_BADBATT_HEIGHT, SH110X_WHITE, SH110X_BLACK);
                display.setCursor(0, 0);
                if (lowestBattery > BATT_100) {
                    statBattery = 4;  // 4.2
                } else if (lowestBattery < BATT_100 && lowestBattery > BATT_75) {
                    statBattery = 3;  // 3.975
                } else if (lowestBattery < BATT_75 && lowestBattery > BATT_50) {
                    statBattery = 2;  // 3.75
                } else if (lowestBattery < BATT_50 && lowestBattery > BATT_25) {
                    statBattery = 1;  // 3.525
                } else if (lowestBattery < BATT_25 && lowestBattery > BATT_0) {
                    statBattery = 0;  // 3.3v
                }
                display.printf("Battery is too Low!\n%d\n%d\n%d\n%d\n%s", levelBattery, lowestBattery, highestBattery, statBattery, isCharging ? "True" : "False");
                display.display();
                if (rawValue < BATT_TOO_LOW) {
                    delay(10000);
                    display.oled_command(SH110X_DISPLAYOFF);
                    esp_deep_sleep_start();
                }
            }
            delay(1);
        };
    }

    display.drawBitmap(0, 8, i_text1, I_TEXT1_WIDTH, I_TEXT1_HEIGHT, SH110X_WHITE);
    display.drawBitmap(2, 20, i_text2, I_TEXT2_WIDTH, I_TEXT2_HEIGHT, SH110X_WHITE);
    display.drawBitmap(82, 9, i_washer, I_WASHER_WIDTH, I_WASHER_HEIGHT, SH110X_WHITE, SH110X_BLACK);
    uint8_t j;
    uint8_t i;
    for (j = 10; j > 1; j--) {
        for (i = 0; i < 8; i++) {
            display.drawBitmap(90, 25, i_washer_mask, I_WASHER_MASK_WIDTH, I_WASHER_MASK_HEIGHT, SH110X_WHITE, SH110X_BLACK);
            display.drawBitmap(90, 25, i_glyph[i], I_GLYPH_WIDTH, I_GLYPH_HEIGHT, SH110X_WHITE);
            display.display();
            delay((100 / j));
        }
    }
    display.drawBitmap(90, 25, i_washer_mask, I_WASHER_MASK_WIDTH, I_WASHER_MASK_HEIGHT, SH110X_WHITE, SH110X_BLACK);
    display.drawBitmap(90, 25, i_glyph[0], I_GLYPH_WIDTH, I_GLYPH_HEIGHT, SH110X_WHITE);
    display.display();

    levelBattery = 0;
    offsBattery = 0;
    while (offsBattery < BATT_AVG - 1) {
        avgBattery[offsBattery] = analogRead(A13);
        offsBattery++;
    }
    battCalc();
    // for (int i = 0; i < BATT_AVG - 1; i++) {
    //     levelBattery += avgBattery[i];
    // }
    // levelBattery = levelBattery / BATT_AVG;
    // highestBattery = levelBattery;
    // lowestBattery = levelBattery;
    offsBattery = 0;
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();  //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);  //active scan uses more power, but get results faster
    pBLEScan->setInterval(125);     //100
    pBLEScan->setWindow(125);       // 100, less or equal setInterval value
    frame_canvas.drawRect(0, STATUS_HEIGHT - 2, MENU_WIDTH, MENU_HEIGHT, SH110X_WHITE);
    isScanning = false;
    memset(s, 0, 256);
    startScan();
    //isConnected = false;
    //delay(2000);
    //connectMachine(0);
    //frame_canvas.drawRect(0, 0, STATUS_WIDTH, STATUS_HEIGHT - 2, SH110X_WHITE);
    // Machine tst1;
    // Machine tst2;
    // tst1.name = "202-TST";
    // tst2.name = "101-TST";
    // tst1.status = "Running";
    // tst2.status = "Running";
    // tst1.minutesLeft = 26;
    // tst2.minutesLeft = 55;
    // tst1.rssi = -90;
    // tst2.rssi = -70;
    // tst1.checkMachineStatus = MSTAT_RUNNING;
    // tst2.checkMachineStatus = MSTAT_RUNNING;
    // myMachines.push_back(tst1);
    // myMachines.push_back(tst2);
    statusMsg = "Scanning..";
    menuSelect = 0;
    sleepTimer = millis();
    //Serial.println("Finished setup.");
    // delay(5000);
    // vTaskDelete(BootAnim);
    display.clearDisplay();
    display.display();
}

void loop() {
    delay(1);
    offsBattery = 0;
    while (offsBattery < BATT_AVG - 1) {
        avgBattery[offsBattery] = analogRead(A13);
        offsBattery++;
    }
    if (!digitalRead(BUTTON_A)) {
        bState = 0;
        sleepTimer = millis();
        display.setContrast(0x7F);
        isDim = false;
    }
    if (!digitalRead(BUTTON_B)) {
        bState = 1;
        sleepTimer = millis();
        display.setContrast(0x7F);
        isDim = false;
    }
    if (!digitalRead(BUTTON_C)) {
        bState = 2;
        sleepTimer = millis();
        display.setContrast(0x7F);
        isDim = false;
    }
    if (millis() - batteryTimer >= 1000) {
        batteryTimer = millis();
        battCalc();
    }
    if (bStateLast != bState) {
        bStateLast = bState;
        if (bState == 0) {
            if (screenScan) {
                if (menuSelect > 0) {
                    menuSelect--;
                }
            } else {
                if (isConnected) {
                    menu_canvas.print("Disconnecting...");
                    drawDisplay();
                    if (!disconnectMachine()) {
                        menu_canvas.println("Failed");
                    }
                    menu_canvas.println("Success");
                    drawDisplay();
                    delay(1000);
                }
                screenScan = true;
                menuSelect = 0;
            }
        }
        if (bState == 2) {
            if (screenScan) {
                if (menuSelect + 1 <= myMachines.size()) {
                    menuSelect++;
                }
            }
        }
        if (bState == 1 && screenScan) {
            screenScan = false;
        }
    }
    if (millis() - testTimer >= 125) {
        testTimer = millis();
        status_canvas.startWrite();
        if (screenScan) {
            statusMsg = "Scanning..";
            menu_canvas.fillScreen(SH110X_BLACK);
            if (myMachines.size()) {
                for (uint8_t i = 0; i < myMachines.size(); i++) {
                    // if (myMachines[i].rssi >= -70) {
                    //     iSignal = 3;
                    // }
                    // if (myMachines[i].rssi < -70 && myMachines[i].rssi > -82) {
                    //     iSignal = 2;
                    // }
                    // if (myMachines[i].rssi < -82 && myMachines[i].rssi > -94) {
                    //     iSignal = 1;
                    // }
                    // if (myMachines[i].rssi < -94) {
                    //     iSignal = 0;
                    // }
                    if (i == menuSelect) {
                        menu_canvas.fillRect(0, 0 + i * 12, MENU_WIDTH, 12, SH110X_WHITE);
                        menu_canvas.setTextColor(SH110X_BLACK);
                    } else {
                        menu_canvas.fillRect(0, 0 + i * 12, MENU_WIDTH, 12, SH110X_BLACK);
                        menu_canvas.setTextColor(SH110X_WHITE);
                    }
                    menu_canvas.setCursor(2, 3 + i * 12);
                    menu_canvas.print(myMachines[i].name.c_str());
                    menu_canvas.setCursor(48, 3 + i * 12);
                    menu_canvas.print(myMachines[i].status.c_str());
                    if (myMachines[i].checkMachineStatus == MSTAT_RUNNING) {
                        menu_canvas.print(" -");
                        menu_canvas.print(myMachines[i].minutesLeft);
                        menu_canvas.print("m");
                    }
                    // if (i == menuSelect) {
                    //     drawSignal(MENU_WIDTH - I_SIGNAL_WIDTH - 4, 2 + i * 12, myMachines[i].rssi, true);
                    // } else {
                    //     drawSignal(MENU_WIDTH - I_SIGNAL_WIDTH - 4, 2 + i * 12, myMachines[i].rssi, false);
                    // }
                    // if (i == menuSelect) {
                    //     menu_canvas.drawBitmap( i_signal[iSignal], I_SIGNAL_WIDTH, I_SIGNAL_HEIGHT, SH110X_BLACK, SH110X_WHITE);
                    // } else {
                    //     menu_canvas.drawBitmap(MENU_WIDTH - I_SIGNAL_WIDTH - 4, 2 + i * 12, i_signal[iSignal], I_SIGNAL_WIDTH, I_SIGNAL_HEIGHT, SH110X_WHITE, SH110X_BLACK);
                    // }
                }
            }

        } else {
            if (!isConnected) {
                if (menuSelect + 1 > myMachines.size())
                    return;
                if (isScanning) {
                    stopScan();
                }
                drawDisplay();
                statusMsg = myMachines[menuSelect].name.c_str();
                menu_canvas.fillScreen(SH110X_BLACK);
                menu_canvas.setTextColor(SH110X_WHITE);
                menu_canvas.setCursor(2, 3);
                isConnected = false;
                menu_canvas.print("Connecting...");
                drawDisplay();
                if (!connectMachine(menuSelect)) {
                    menu_canvas.println("Failed");
                    drawDisplay();
                    return;
                }
                menu_canvas.println("Success");
                drawDisplay();
                menu_canvas.print("Inquiring...");
                drawDisplay();
                if (!inquiryMachine()) {
                    menu_canvas.println("Failed");
                    drawDisplay();
                    return;
                }
                menu_canvas.println("Success");
                drawDisplay();
                menu_canvas.print("Starting...");
                drawDisplay();
                if (!startMachine()) {
                    menu_canvas.println("Failed");
                    drawDisplay();
                    return;
                }
                menu_canvas.println("Success");
                // myMachines[menuSelect].isMine = true;
            } else {
                rssiMachine();
                myMachines[menuSelect].rssi = myMachine->rssi;
                // .rssi = ;
            }
        }
        status_canvas.endWrite();
    }
    drawDisplay();
    if (isSleeping || millis() - sleepTimer >= SLEEP_TIMER) {
        if (isConnected) {
            menu_canvas.print("Disconnecting...");
            drawDisplay();
            if (!disconnectMachine()) {
                menu_canvas.println("Failed");
                drawDisplay();
                return;
            }
            menu_canvas.println("Success");
            drawDisplay();
            delay(500);
        }
        sleepTimer = millis();
        if (isSleeping) {
            esp_light_sleep_start();
            return;
        }
        esp_sleep_enable_touchpad_wakeup();
        if (levelBattery < BATT_USB) {
            display.clearDisplay();
            display.display();
            isSleeping = true;
            display.oled_command(SH110X_DISPLAYOFF);
            esp_deep_sleep_start();
        }
    }
    if (!isDim && millis() - sleepTimer >= DIM_TIMER) {
        isDim = true;
        display.setContrast(1);
    }
    Serial.flush();
    if (!Serial.available()) {
        return;
    }
    i = Serial.read(c, 1);
    if (i == 0) {
        return;
    }
    if (c[0] != '\n' && c[0] != '\r') {
        if (c[0] == '\b') {
            Serial.printf("%s", c);
            s[o] = 0;
            o--;
            return;
        }
        Serial.printf("%s", c);
        s[o] = c[0];
        o++;
        return;
    }
    if (o == 0) {
        Serial.println();
        return;
    }
    Serial.printf("\n");
    if (s[0] == 'l' && s[1] == 'm' && s[2] == '\0') {
        for (int i = 0; i < myMachines.size(); i++) {
            printMachine(myMachines[i]);
        }
    } else if (s[0] == 's' && s[1] == 's' && s[2] == '\0') {
        startScan();
    } else if (s[0] == 'd' && s[1] == 's' && s[2] == '\0') {
        stopScan();
    } else if (s[0] == 'c' && s[1] == 'm' && s[2] >= '0' && s[2] <= '9' && s[3] == '\0') {
        connectMachine(atoi(&s[2]));
    } else if (s[0] == 'd' && s[1] == 'm' && s[2] == '\0') {
        disconnectMachine();
    } else if (s[0] == 'i' && s[1] == 'm' && s[2] == '\0') {
        // Serial.println("Inquiring Machine...Failed, Not Implemented");
        inquiryMachine();
    } else if (s[0] == 's' && s[1] == 'm' && s[2] == '\0') {
        startMachine();
    }
    o = 0;
    memset(s, 0, 256);
    memset(c, 0, 2);
}
