
#ifndef ModbusMaster_h
#define ModbusMaster_h

#include <stdint.h>
#include "mbed.h"

struct ModbusMaster {
    enum class Status: uint8_t {
        success             = 0x00,
        // modbus exceptions
        illegalFunction     = 0x01,
        illegalDataAddress  = 0x02,
        illegalDataValue    = 0x03,
        slaveDeviceFailure  = 0x04,
        acknowledge         = 0x05,
        slaveDeviceBusy     = 0x06,
        negativeAcknowledge = 0x07,
        memoryParityError   = 0x08,
        // response errors
        invalidSlaveID      = 0xE0,
        invalidFunction     = 0xE1,
        responseTimeout     = 0xE2,
        invalidCRC          = 0xE3,
    };

    enum class Function: uint8_t {
        none                        = 0x00,
        // bit access
        readCoils                   = 0x01,
        readDiscreteInputs          = 0x02,
        writeSingleCoil             = 0x05,
        writeMultipleCoils          = 0x0F,
        // word access
        readHoldingRegisters        = 0x03,
        readInputRegisters          = 0x04,
        writeSingleRegister         = 0x06,
        writeMultipleRegisters      = 0x10,
    };

    typedef Callback<void(Status)> CB;

    EventQueue* queue;
    Serial* serial;
    uint8_t slaveID;
    int rxTimeout;
    MbedCRC<POLY_16BIT_IBM, 16> crc16;
    Timeout frameTimeout;
    us_timestamp_t frameDelimTime;
    Callback<void()> preTransmit = NULL;
    Callback<void()> postTransmit = NULL;
    Callback<uint16_t(uint8_t* adu, uint16_t len)> postReceive = NULL;
    Callback<void(Status)> complete = NULL;

    uint8_t adu[256];
    uint16_t txLen = 0;
    uint16_t rxLen = 0;
    int receiveTimeoutID = 0;
    Function reqFunction = Function::none;

    ModbusMaster(EventQueue* queue, Serial* serial, int baud, uint8_t slaveID, int timeout = 50):
        queue(queue),
        serial(serial),
        slaveID(slaveID),
        rxTimeout(timeout),
        crc16(MbedCRC<POLY_16BIT_IBM, 16>(0xFFFF, 0x0000, true, true)),
        frameTimeout(Timeout()),
        frameDelimTime((35 * 1000 * 1000) / baud) {
            serial->baud(baud);
        }

    inline void attachPreTransmit(Callback<void()> f) { preTransmit = f; };
    inline void attachPostTransmit(Callback<void()> f) { postTransmit = f; };
    inline void attachPostReceive(Callback<uint16_t(uint8_t* adu, uint16_t len)> f) { postReceive = f; };
    inline void setSlaveID(uint8_t id) { slaveID = id; };
    inline void setTimeout(int t) { rxTimeout = t; };
    inline uint8_t* getCoils() { return static_cast<uint8_t*>(adu + 3); };
    inline uint16_t* getRegisters() { return reinterpret_cast<uint16_t*>(adu + 3); }

    void readCoils(uint16_t addr, uint16_t num, CB cb);
    void readDiscreteInputs(uint16_t addr, uint16_t num, CB cb);
    void writeSingleCoil(uint16_t addr, bool val, CB cb);
    void writeMultipleCoils(uint16_t addr, uint16_t num, uint8_t* val, CB cb);

    void readHoldingRegisters(uint16_t addr, uint16_t num, CB cb);
    void readInputRegisters(uint16_t addr, uint16_t num, CB cb);
    void writeSingleRegister(uint16_t addr, uint16_t val, CB cb);
    void writeMultipleRegisters(uint16_t addr, uint16_t num, uint16_t* val, CB cb);

    void transaction(Function func, uint16_t addr, uint16_t num, uint8_t* val, CB cb);
    void rxHandler();
    void rxCompleteHandler();
    void rxTimeoutHandler();

    inline void writeUInt16(uint16_t val) {
        adu[txLen++] = val >> 8;
        adu[txLen++] = val;
    }
};

#endif

