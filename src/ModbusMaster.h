
#ifndef ModbusMaster_h
#define ModbusMaster_h

#include <stdint.h>
#include "mbed.h"

class ModbusMaster {
    public:
        EventQueue* queue;
        Serial* serial;
        uint8_t slaveID;
        int rxTimeout;
        us_timestamp_t frameDelimTime;
        Callback<void()> preTransmission = NULL;
        Callback<void()> postTransmission = NULL;
        Callback<void(Status)> complete = NULL;
        
        Timeout frameTimeout;
        MbedCRC<POLY_16BIT_IBM, 16> crc16(0xFFFF, 0, false, false);
        uint8_t adu[256];
        uint16_t txLen = 0;
        uint16_t rxLen = 0;
        int receiveTimeoutID = 0;
        uint8_t reqFunction = 0;
        
        ModbusMaster(EventQueue* queue, Serial* serial, int baud, uint8_t slaveID, int timeout = 50)
            queue(queue),
            serial(serial),
            slaveID(slaveID),
            rxTimeout(timeout)
        {
            frameDelimTime = (35 * 1000 * 1000) / baud;
            serial->baud(baud);
        }

        void attachPreTransmission(Callback<void()> f) { preTransmission = f; };
        void attachPostTransmission(Callback<void()> f) { postTransmission = f; };
        void setSlaveID(uint8_t id) { slaveID = id; };
        void setTimeout(int t) { timeout = t; };
        uint8_t* getCoils() { return  &adu + 3; };
        uint16_t* getRegisters() { return static_cast<uint16_t*>(&adu + 3); }

        void readCoils(uint16_t addr, uint16_t num, Callback<void(Status)> cb = NULL);
        void readDiscreteInputs(uint16_t addr, uint16_t num, Callback<void(Status)> cb = NULL);
        void writeSingleCoil(uint16_t addr, bool val, Callback<void(Status)> cb = NULL);
        void writeMultipleCoils(uint16_t addr, uint16_t num, uint8_t* val, Callback<void(Status)> cb = NULL);

        void readHoldingRegisters(uint16_t addr, uint16_t num, Callback<void(Status)> cb = NULL);
        void readInputRegisters(uint16_t addr, uint8_t num, Callback<void(Status)> cb = NULL);
        void writeSingleRegister(uint16_t addr, uint16_t val, Callback<void(Status)> cb = NULL);
        void writeMultipleRegisters(uint16_t addr, uint16_t num, uint16_t* val, Callback<void(Status)> cb = NULL);

        void transaction(uint8_t func, uint16_t addr, uint16_t num, uint8_t* val, Callback<void(Status)> cb);
        
        void writeUInt16(uint16_t val);
        void txCompleteHandler(int ev);
        void rxHandler();
        void rxCompleteHandler();
        void rxTimeoutHandler();

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
        }

        enum class Function: uint8_t {
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
        }
};

#endif
