#include <Arduino.h>
#include <BLEUtils.h>

#include "mbedtls/md.h"

// Class Prototypes
class AdvertiseInfo {
   private:
    uint16_t stoui16(const char *src, int offs);
    uint32_t stoui32(const char *src, int offs);
    float getFloat(uint16_t amount);

   public:
    uint8_t messageType = 0;
    uint16_t locationId = 0;
    uint32_t controllerId = 0;
    uint8_t protocol = 0;
    uint8_t type = 0;
    uint8_t statusLower = 0;
    uint8_t statusUpper = 0;
    uint16_t maxPriceLower = 0;
    uint16_t maxPriceUpper = 0;
    uint16_t minPriceLower = 0;
    uint16_t minPriceUpper = 0;
    uint8_t minutesLeftLower = 0;
    uint8_t minutesLeftUpper = 0;
    uint8_t specialBits = 0;
    bool superCyclePossibleLower = false;
    bool multiTopOffPossibleLower = false;
    bool superCyclePossibleUpper = false;
    bool multiTopOffPossibleUpper = false;
    bool lowerIsAvailable = false;
    bool lowerIsAdditionalVend = false;
    bool lowerIsRunning = false;
    bool lowerIsBusy = false;
    bool lowerIsOffline = false;
    bool upperIsAvailable = false;
    bool upperIsAdditionalVend = false;
    bool upperIsRunning = false;
    bool upperIsBusy = false;
    bool upperIsOffline = false;
    uint8_t basePriceLower = 0;
    uint8_t basePriceUpper = 0;
    uint8_t superCyclePriceLower = 0;
    uint8_t superCyclePriceUpper = 0;
    float maxPriceLowerDollars = 0;
    float maxPriceUpperDollars = 0;
    float minPriceLowerDollars = 0;
    float minPriceUpperDollars = 0;
    AdvertiseInfo();
    AdvertiseInfo(const char *mfgData);
};

class Machine {
   public:
    int index = 0;
    BLEAddress addr = BLEAddress((uint8_t *)"\0\0\0\0\0\0");
    std::string name = "";
    int rssi = 0;
    uint16_t pong = 1;
    AdvertiseInfo advertiseInfo;
    std::string status = "";
    bool isConnected = false;
    bool isMine = false;
    bool isUpper = false;
    bool isAvailable = false;
    bool isRunning = false;
    bool isBusy = false;
    bool isOffline = false;
    uint16_t minPrice = 0;
    uint16_t maxPrice = 0;
    float minPriceDollars = false;
    float maxPriceDollars = false;
    uint16_t basePrice = 0;
    uint16_t superCyclePrice = 0;
    bool superCyclePossible = false;
    bool singleTopOffPossible = false;
    bool multiTopOffPossible = false;
    uint16_t minutesLeft = 0;
    bool additionalVend = false;
    uint16_t checkMachineStatus = 0;
    bool isTopOffAvailable = false;
    bool isMachineRunning = false;
    uint16_t priceRangeAllowedFor = 0;
    BLEAdvertisedDevice device;
    Machine();
    Machine(BLEAdvertisedDevice &device, bool is_upper);
    std::string statToStr(uint8_t status);
};

// Func Protypes
void writeResponse(uint8_t *cmd, size_t len);
bool disconnectMachine(bool noScan);
bool connectMachine(int m);
void scanMachines();
void startScan();
void stopScan();
static void scanCompleteCB(BLEScanResults scanResults);
void printMachine(Machine &m);
void drawBattery(int16_t x, int16_t y, uint8_t l);
void drawStatus();
void drawDisplay();
void battCalc();

// Consts
const uint16_t BATT_AVG = 100;
const uint16_t BATT_USB = 2470;
const uint16_t BATT_CRITICAL = 1850;
const uint16_t BATT_TOO_LOW = 2070;
const uint16_t BATT_0 = 2100;
const uint16_t BATT_25 = 2180;
const uint16_t BATT_50 = 2260;
const uint16_t BATT_75 = 2340;
const uint16_t BATT_100 = 2400;
const uint8_t OLED_WIDTH = 128;
const uint8_t OLED_HEIGHT = 64;
const uint8_t STATUS_WIDTH = OLED_WIDTH;
const uint8_t STATUS_HEIGHT = 16;
const uint8_t MENU_WIDTH = OLED_WIDTH;
const uint8_t MENU_HEIGHT = OLED_HEIGHT - STATUS_HEIGHT;
const uint8_t MENU_COLS = 20;
const uint8_t MENU_ROWS = 5;
const uint8_t BUTTON_A = 15;
const uint8_t BUTTON_B = 32;
const uint8_t BUTTON_C = 14;
//const char menu[MENU_ROWS][MENU_COLS];
const bool PRTMACH = false;
const unsigned char cKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // !! REDACTED !!
const mbedtls_md_type_t md_type = MBEDTLS_MD_SHA1;
const mbedtls_md_info_t *hashInfo = mbedtls_md_info_from_type((mbedtls_md_type_t)md_type);
const unsigned int SLEEP_TIMER = 45000;
const unsigned int DIM_TIMER = 15000;
const unsigned int MIO_PAYLOAD = 19;
const unsigned int RESP_BUF_SIZE = 256;
const unsigned int TX_BUF_SIZE = 256;
const unsigned int RX_BUF_SIZE = 256;
const unsigned int BLESCANTIME = 5;  //In seconds
const uint8_t MSTAT_IDLE = 0x00;
const uint8_t MSTAT_RESET = 0x01;
const uint8_t MSTAT_DOOR_OPEN = 0x03;
const uint8_t MSTAT_START = 0x04;
const uint8_t MSTAT_RUNNING = 0x05;
const uint8_t MSTAT_DONE = 0x06;
const uint8_t MSTAT_ERROR = 0x07;
const uint8_t MSTAT_MANUAL = 0x08;
const uint8_t MSTAT_IN_SESSION = 0x09;
const uint8_t MSTAT_BAD_CONFIG = 0x0A;
const uint8_t MSTAT_UNAVAILABLE = 0x0B;
const uint8_t MSTAT_ADDITIONAL_VEND = 0x0C;
const uint8_t MSTAT_POWER_CYCLE = 0x40;
const uint8_t MSTAT_CONFIGURING = 0x80;
const uint8_t MSTAT_CONFIGURED = 0x81;
const uint8_t MSTAT_DISABLED = 0x82;
const uint8_t MSTAT_KILL = 0x90;
const uint8_t MSTAT_DIAGNOSING = 0x91;
const uint8_t MSTAT_OOS = 0x92;
const uint8_t MSTAT_DAILY_SPECIAL = 0x93;
const uint8_t MSTAT_PRESS_START = 0x94;
const uint8_t MSTAT_EXTEND_WASH = 0x95;
const uint8_t MSTAT_EXTRA_RINSE = 0x96;
const uint8_t MSTAT_SELECT_TEMP = 0x97;
const uint8_t MSTAT_CLOSE_DOOR = 0x98;
const uint8_t MSTAT_UNUSED = 0xFF;
const uint8_t MCMD_GET_AUTH_LOCK_STATUS[] = {0x00};
const uint8_t MCMD_GET_AUTH_CHALLENGE_DATA[] = {0x01};
const uint8_t MCMD_GET_AUTH_UNLOCK[] = {0x02};
const uint8_t MCMD_READ_READY[] = {0x03};
const uint8_t MCMD_READ_START[] = {0x04};
const uint8_t MCMD_READ_DATA[] = {0x05};
const uint8_t MCMD_WRITE_START[] = {0x06};
const uint8_t MCMD_WRITE_DATA[] = {0x07};
const uint8_t MCMD_RESET[] = {0x08};
const uint8_t MCMD_READ_FLUSH[] = {0x09};
const uint8_t MRESP_STATUS_INQUIRY = 0x00;
const uint8_t MRESP_BEGIN_LOAD = 0x01;
const uint8_t MRESP_MANUAL_START = 0x02;
const uint8_t MRESP_SALE = 0x03;
const uint8_t MRESP_NO_SALE = 0x04;

const char *MSGSTAT_P_SCAN = "Scanning..";
const char *MSGSTAT_SCAN = "Scanning for Machines...";
const char *MSGSTAT_SSCAN = "Starting Scan...";
const char *MSGSTAT_STSCAN = "Stopping Scan...";
const char *MSGSTAT_CONNECT = "Connecting to Machine...";
const char *MSGSTAT_DISCONNECT = "Disconnecting Machine...";
const char *MSGSTAT_P_CONNECT = "Connecting...";
const char *MSGSTAT_P_DISCONNECT = "Disconnecting...";
const char *MSGSTAT_P_INQ = "Inquiring...";
const char *MSGSTAT_P_START = "Starting...";
const char *MSGSTAT_FAIL_NCONN = "Failed, Not Connected.";
const char *MSGSTAT_FAIL_ACONN = "Failed, Already Connected.";
const char *MSGSTAT_FAIL_NSCAN = "Failed, Not Scanning.";
const char *MSGSTAT_FAIL_ASCAN = "Failed, Already Scanning.";
const char *MSGSTAT_FAIL = "Failed";
const char *MSGSTAT_SUCCESS = "Success";
const char *MSGSTAT_DONE = "Done";

// Vars
uint16_t txLength = 0;
uint16_t txOffset = 0;
uint8_t txBuffer[TX_BUF_SIZE];
uint16_t rxLength = 0;
uint16_t rxOffset = 0;
uint8_t rxBuffer[RX_BUF_SIZE];
uint8_t respBuffer[RESP_BUF_SIZE];
uint8_t mState = 0;
std::vector<Machine> myMachines;
static BLEUUID uuidService("f0e60001-4b8c-41d8-a61c-eeafa4eb6077");
static BLEUUID uuidRX("f0e60002-4b8c-41d8-a61c-eeafa4eb6077");
static BLEUUID uuidTX("f0e60003-4b8c-41d8-a61c-eeafa4eb6077");
static BLEUUID uuidResponse("f0e60004-4b8c-41d8-a61c-eeafa4eb6077");

// Class Funcs
AdvertiseInfo::AdvertiseInfo(){};

AdvertiseInfo::AdvertiseInfo(const char *mfgData) {
    this->messageType = mfgData[2];                                              // 0
    this->locationId = stoui16(mfgData, 3);                                      // 19995
    this->controllerId = stoui32(mfgData, 5);                                    // 36464
    this->protocol = mfgData[9];                                                 // 1
    this->type = mfgData[10];                                                    // 9
    this->statusLower = mfgData[11];                                             // 0
    this->statusUpper = mfgData[12];                                             // 255
    this->maxPriceLower = stoui16(mfgData, 13);                                  // 175
    this->maxPriceUpper = stoui16(mfgData, 15);                                  // 0
    this->minPriceLower = std::min(stoui16(mfgData, 17), stoui16(mfgData, 13));  // 175
    this->minPriceUpper = std::min(stoui16(mfgData, 19), stoui16(mfgData, 15));  // 0
    this->minutesLeftLower = mfgData[21];                                        // 0
    this->minutesLeftUpper = mfgData[22];                                        // 0
    this->superCyclePossibleLower = bitRead(mfgData[23], 0) ? true : false;      // 0
    this->multiTopOffPossibleLower = bitRead(mfgData[23], 1) ? true : false;     // 0
    this->superCyclePossibleUpper = bitRead(mfgData[23], 4) ? true : false;      // 0
    this->multiTopOffPossibleUpper = bitRead(mfgData[23], 5) ? true : false;     // 0
    this->lowerIsAvailable = (this->statusLower == MSTAT_IDLE && this->maxPriceLower > 0) ? true : false;
    this->lowerIsAdditionalVend = (this->statusLower == MSTAT_ADDITIONAL_VEND) ? true : false;
    this->lowerIsRunning = (this->statusLower == MSTAT_RUNNING) ? true : false;
    this->lowerIsBusy = (this->lowerIsRunning || this->lowerIsAvailable) == false ? true : false;
    this->lowerIsOffline = (this->statusLower == MSTAT_BAD_CONFIG || this->statusLower == MSTAT_ERROR) ? true : false;
    this->upperIsAvailable = (this->statusUpper == MSTAT_IDLE && this->maxPriceUpper > 0) ? true : false;
    this->upperIsAdditionalVend = (this->statusUpper == MSTAT_ADDITIONAL_VEND) ? true : false;
    this->upperIsRunning = (this->statusUpper == MSTAT_RUNNING) ? true : false;
    this->upperIsBusy = (this->upperIsRunning || this->upperIsAvailable) == false ? true : false;
    this->upperIsOffline = (this->statusUpper == MSTAT_BAD_CONFIG || this->statusUpper == MSTAT_ERROR) ? true : false;
    this->basePriceLower = this->minPriceLower;
    this->basePriceUpper = this->minPriceUpper;
    this->superCyclePriceLower = this->superCyclePossibleLower ? this->maxPriceLower : 0;
    this->superCyclePriceUpper = this->superCyclePossibleUpper ? this->maxPriceUpper : 0;
    this->maxPriceLowerDollars = getFloat(this->maxPriceLower);
    this->maxPriceUpperDollars = getFloat(this->maxPriceUpper);
    this->minPriceLowerDollars = getFloat(this->minPriceLower);
    this->minPriceUpperDollars = getFloat(this->minPriceUpper);
}

uint16_t AdvertiseInfo::stoui16(const char *src, int offs) {
    return src[offs] << 8 | src[offs + 1];
}

uint32_t AdvertiseInfo::stoui32(const char *src, int offs) {
    return src[offs] << 24 | src[offs + 1] << 16 | src[offs + 2] << 8 | src[offs + 3];
}

float AdvertiseInfo::getFloat(uint16_t amount) {
    return float(amount) / 100;
}

Machine::Machine(){};

Machine::Machine(BLEAdvertisedDevice &device, bool is_upper = false) {
    this->device = device;
    this->addr = device.getAddress();
    this->name = device.getName();
    this->rssi = device.getRSSI();
    this->advertiseInfo = AdvertiseInfo(device.getManufacturerData().c_str());
    this->isUpper = is_upper;
    this->isAvailable = is_upper ? this->advertiseInfo.upperIsAvailable : this->advertiseInfo.lowerIsAvailable;
    this->isRunning = is_upper ? this->advertiseInfo.upperIsRunning : this->advertiseInfo.lowerIsRunning;
    this->isBusy = is_upper ? this->advertiseInfo.upperIsBusy : this->advertiseInfo.lowerIsBusy;
    this->isOffline = is_upper ? this->advertiseInfo.upperIsOffline : this->advertiseInfo.lowerIsOffline;
    this->minPrice = is_upper ? this->advertiseInfo.minPriceUpper : this->advertiseInfo.minPriceLower;
    this->maxPrice = is_upper ? this->advertiseInfo.maxPriceUpper : this->advertiseInfo.maxPriceLower;
    this->minPriceDollars = is_upper ? this->advertiseInfo.minPriceUpperDollars : this->advertiseInfo.minPriceLowerDollars;
    this->maxPriceDollars = is_upper ? this->advertiseInfo.maxPriceUpperDollars : this->advertiseInfo.maxPriceLowerDollars;
    this->basePrice = is_upper ? this->advertiseInfo.basePriceUpper : this->advertiseInfo.basePriceLower;
    this->superCyclePrice = is_upper ? this->advertiseInfo.superCyclePriceUpper : this->advertiseInfo.superCyclePriceLower;
    this->superCyclePossible = is_upper ? this->advertiseInfo.superCyclePossibleUpper : this->advertiseInfo.superCyclePossibleLower;
    this->singleTopOffPossible = this->isRunning && this->minutesLeft != 0 && this->maxPrice != 0;
    this->multiTopOffPossible = is_upper ? this->advertiseInfo.multiTopOffPossibleUpper : this->advertiseInfo.multiTopOffPossibleLower;
    this->minutesLeft = is_upper ? this->advertiseInfo.minutesLeftUpper : this->advertiseInfo.minutesLeftLower;
    this->additionalVend = is_upper ? this->advertiseInfo.upperIsRunning : this->advertiseInfo.lowerIsRunning;
    this->checkMachineStatus = is_upper ? this->advertiseInfo.statusUpper : this->advertiseInfo.statusLower;
    this->isTopOffAvailable = (this->checkMachineStatus > 0 && this->maxPrice > 0 && this->minutesLeft > 0) ? true : false;
    this->isMachineRunning = (this->checkMachineStatus > 0 && !this->singleTopOffPossible && this->minutesLeft > 0) ? true : false;
    this->priceRangeAllowedFor = is_upper ? this->advertiseInfo.maxPriceUpper : this->advertiseInfo.maxPriceLower;
    this->status = statToStr(this->checkMachineStatus);
}

std::string Machine::statToStr(uint8_t status) {
    if (status == MSTAT_IDLE) {
        return "Idle";
    } else if (status == MSTAT_RESET) {
        return "Reset";
    } else if (status == MSTAT_DOOR_OPEN) {
        return "Door Open";
    } else if (status == MSTAT_START) {
        return "Start";
    } else if (status == MSTAT_RUNNING) {
        return "Running";
    } else if (status == MSTAT_DONE) {
        return "Done";
    } else if (status == MSTAT_ERROR) {
        return "Error";
    } else if (status == MSTAT_MANUAL) {
        return "Manual";
    } else if (status == MSTAT_IN_SESSION) {
        return "In Session";
    } else if (status == MSTAT_BAD_CONFIG) {
        return "Bad Config";
    } else if (status == MSTAT_UNAVAILABLE) {
        return "Unavailable";
    } else if (status == MSTAT_ADDITIONAL_VEND) {
        return "Additional Vend";
    } else if (status == MSTAT_POWER_CYCLE) {
        return "Power Cycle";
    } else if (status == MSTAT_CONFIGURING) {
        return "Configuring";
    } else if (status == MSTAT_CONFIGURED) {
        return "Configured";
    } else if (status == MSTAT_DISABLED) {
        return "Disabled";
    } else if (status == MSTAT_KILL) {
        return "Kill";
    } else if (status == MSTAT_DIAGNOSING) {
        return "Diagnosing";
    } else if (status == MSTAT_OOS) {
        return "Out of Service";
    } else if (status == MSTAT_DAILY_SPECIAL) {
        return "Daily Special";
    } else if (status == MSTAT_PRESS_START) {
        return "Press Start";
    } else if (status == MSTAT_EXTEND_WASH) {
        return "Extended Wash";
    } else if (status == MSTAT_EXTRA_RINSE) {
        return "Extra Rinse";
    } else if (status == MSTAT_SELECT_TEMP) {
        return "Select Temp";
    } else if (status == MSTAT_CLOSE_DOOR) {
        return "Close Door";
    } else if (status == MSTAT_UNUSED) {
        return "Unused";
    }
    return "Unknown";
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(uuidService)) {
            //Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
            Machine foundDevice = Machine(advertisedDevice);
            if (foundDevice.advertiseInfo.statusUpper != 255) {
                Machine upperDevice = Machine(advertisedDevice, true);
                upperDevice.name = upperDevice.name + " - Upper";
                foundDevice.name = foundDevice.name + " - Lower";
                // add/replace upperDevice in devices;
                int i;
                uint16_t pong;
                for (i = 0; i < myMachines.size(); i++) {
                    if (myMachines[i].name == upperDevice.name) {
                        break;
                    }
                }
                if (i == myMachines.size()) {
                    myMachines.push_back(upperDevice);
                } else {
                    pong = myMachines[i].pong;
                    myMachines[i] = upperDevice;
                    myMachines[i].pong = pong + 1;
                }
            }
            // add/replace foundDevice in devices;
            int i;
            uint16_t pong;
            for (i = 0; i < myMachines.size(); i++) {
                if (myMachines[i].name == foundDevice.name) {
                    break;
                }
            }
            if (i == myMachines.size()) {
                myMachines.push_back(foundDevice);
            } else {
                pong = myMachines[i].pong;
                myMachines[i] = foundDevice;
                myMachines[i].pong = pong + 1;
            }
            for (int i = 0; i < myMachines.size(); i++) {
                myMachines[i].index = i;
            }
            if (PRTMACH) {
                for (int i = 0; i < myMachines.size(); i++) {
                    printMachine(myMachines[i]);
                }
            }
        }
    }
};
