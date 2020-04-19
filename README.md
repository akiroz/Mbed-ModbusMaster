# ModbusMaster

Modbus-RTU library for MbedOS 5 using asynchronous Serial API.

This project is inspired by the Arduino [ModbusMaster][] library by 4-20ma.

[ModbusMaster]: https://github.com/4-20ma/ModbusMaster

## API

#### `ModbusMaster(EventQueue* queue, Serial* serial, int baud, uint8_t slaveID, int timeout = 50)`

Create new Modbus Master.

Params:
- `queue`: EventQueue used for processing response off IRQ context
- `serial`: Serial hardware to use for Modbus
- `baud`: Serial baud rate used for Modbus frame delimiting
- `slaveID`: Modbus Slave ID
- `timeout`: Modbus response timeout (ms)

#### `void attachPreTransmission(Callback<void()> f)`

#### `void attachPostTransmission(Callback<void()> f)`

#### `void setSlaveID(uint8_t id)`

#### `void setTimeout(int t)`

#### `uint8_t* getCoils()`

#### `uint16_t* getRegisters()`

#### `void readCoils(uint16_t addr, uint16_t num, Callback<void(Status)> cb = NULL)`

#### `void readDiscreteInputs(uint16_t addr, uint16_t num, Callback<void(Status)> cb = NULL)`

#### `void writeSingleCoil(uint16_t addr, bool val, Callback<void(Status)> cb = NULL)`

#### `void writeMultipleCoils(uint16_t addr, uint16_t num, uint8_t* val, Callback<void(Status)> cb = NULL)`

#### `void readHoldingRegisters(uint16_t addr, uint16_t num, Callback<void(Status)> cb = NULL)`

#### `void readInputRegisters(uint16_t addr, uint8_t num, Callback<void(Status)> cb = NULL)`

#### `void writeSingleRegister(uint16_t addr, uint16_t val, Callback<void(Status)> cb = NULL)`

#### `void writeMultipleRegisters(uint16_t addr, uint16_t num, uint16_t* val, Callback<void(Status)> cb = NULL)`

## Example

Read registers:
```
EventQueue queue(32 * EVENTS_EVENT_SIZE);
Serial serial(PA_2, PA_3);
ModbusMaster modbus(&queue, &serial, 115200, 1, 100);

int main() {
    modbus.readHoldingRegisters(1, 4, [](uint8_t s){
        if(s != ModbusMaster::Status::success) {
            // Handle error
        } else {
            uint16_t* reg = modbus.getRegisters();
            reg[0]; // => register 1 value
            reg[1]; // => register 2 value
            reg[2]; // => register 3 value
            reg[3]; // => register 4 value
        }
    });
    queue.dispatch_forever();
    return 0;
}

```

