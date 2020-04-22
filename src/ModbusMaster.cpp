#include "ModbusMaster.h"

typedef ModbusMaster M;
typedef M::Function F;
typedef M::Status S;

void M::rxHandler() {
    while(serial->readable()) {
        if(rxLen >= 255) rxLen = 0; // prevent buffer overflow
        adu[rxLen++] = serial->getc();
    }
    frameTimeout.attach_us([this](){
        queue->call([this](){ rxCompleteHandler(); });
    }, frameDelimTime);
}

void M::rxCompleteHandler() {
    if(receiveTimeoutID != 0) queue->cancel(receiveTimeoutID);
    serial->attach(NULL);

    if(postReceive) rxLen = postReceive(adu, rxLen);

    if(rxLen < 4) {
        complete(S::invalidCRC);
        return;
    }
    uint32_t compCRC = 0;
    crc16.compute(adu, rxLen - 2, &compCRC);
    uint16_t recvCRC = adu[rxLen - 2] | (adu[rxLen - 1] << 8);
    if(recvCRC != compCRC) {
        complete(S::invalidCRC);
        return;
    }
    if(adu[0] != slaveID) {
        complete(S::invalidSlaveID);
        return;
    }
    if(static_cast<F>(adu[1] & 0x7F) != reqFunction) {
        complete(S::invalidFunction);
        return;
    }
    if(adu[1] & 0x80) { // Exception
        complete(static_cast<S>(adu[2]));
        return;
    }
    switch(reqFunction) {
        case F::readHoldingRegisters:
        case F::readInputRegisters:
            uint16_t* reg = getRegisters();
            uint8_t regCount = (rxLen - 5) / 2;
            for(int i = 0; i < regCount; i++) reg[i] = __builtin_bswap16(reg[i]);
    }
    complete(S::success);
}

void M::rxTimeoutHandler() {
    frameTimeout.detach();
    serial->attach(NULL);
    complete(S::responseTimeout);
}

void M::transaction(F func, uint16_t addr, uint16_t num, uint8_t* val, Callback<void(M::Status)> cb) {
    txLen = 0;
    rxLen = 0;
    receiveTimeoutID = 0;
    reqFunction = func;
    complete = cb;
    while(serial->readable()) serial->getc();

    // Build Request
    adu[txLen++] = slaveID;
    adu[txLen++] = static_cast<uint8_t>(func);
    writeUInt16(addr);
    switch(func) {
        case F::readCoils:
        case F::readDiscreteInputs:
        case F::writeMultipleCoils:
        case F::readHoldingRegisters:
        case F::readInputRegisters:
        case F::writeMultipleRegisters:
            writeUInt16(num);
    }
    uint8_t payloadLen = 0;
    switch(func) {
        case F::writeSingleCoil:
            payloadLen = 2;
            break;
        case F::writeMultipleCoils:
            payloadLen = (num / 8) + 1;
            adu[txLen++] = payloadLen;
            break;
        case F::writeSingleRegister:
            payloadLen = 2;
            break;
        case F::writeMultipleRegisters:
            payloadLen = num * 2;
            adu[txLen++] = payloadLen;
            break;
    }
    for(int i = 0; i < payloadLen; i++) adu[txLen++] = val[i];
    uint32_t crc = 0;
    crc16.compute(adu, txLen, &crc);
    adu[txLen++] = crc;
    adu[txLen++] = crc >> 8;

    // Send Request
    if(preTransmit) preTransmit();
    serial->write(adu, txLen, [this](int e){
        if(postTransmit) postTransmit();
        serial->attach([this](){ rxHandler(); });
        receiveTimeoutID = queue->call_in(rxTimeout, [this](){ rxTimeoutHandler(); });
    });
}

// Modbus Functions ===================================================================

void M::readCoils(uint16_t addr, uint16_t num, Callback<void(M::Status)> cb) {
    transaction(F::readCoils, addr, num, NULL, cb);
}

void M::readDiscreteInputs(uint16_t addr, uint16_t num, Callback<void(M::Status)> cb) {
    transaction(F::readDiscreteInputs, addr, num, NULL, cb);
}

void M::writeSingleCoil(uint16_t addr, bool val, Callback<void(M::Status)> cb) {
    uint16_t payload = __builtin_bswap16(val ? 0xFF00 : 0x0000);
    transaction(F::writeSingleCoil, addr, 1, reinterpret_cast<uint8_t*>(&payload), cb);
}

void M::writeMultipleCoils(uint16_t addr, uint16_t num, uint8_t* val, Callback<void(M::Status)> cb) {
    transaction(F::writeMultipleCoils, addr, num, val, cb);
}

void M::readHoldingRegisters(uint16_t addr, uint16_t num, Callback<void(M::Status)> cb) {
    transaction(F::readHoldingRegisters, addr, num, NULL, cb);
}

void M::readInputRegisters(uint16_t addr, uint16_t num, Callback<void(M::Status)> cb) {
    transaction(F::readInputRegisters, addr, num, NULL, cb);
}

void M::writeSingleRegister(uint16_t addr, uint16_t val, Callback<void(M::Status)> cb) {
    val = __builtin_bswap16(val);
    uint8_t* payload = reinterpret_cast<uint8_t*>(&val);
    transaction(F::writeSingleRegister, addr, 1, payload, cb);
}

void M::writeMultipleRegisters(uint16_t addr, uint16_t num, uint16_t* val, Callback<void(M::Status)> cb) {
    for(int i = 0; i < num; i++) val[i] = __builtin_bswap16(val[i]);
    uint8_t* payload = reinterpret_cast<uint8_t*>(&val);
    transaction(F::writeMultipleRegisters, addr, num, payload, cb);
}
