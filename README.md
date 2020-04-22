# ModbusMaster

Modbus-RTU library for MbedOS 5 using asynchronous Serial API.

This project is inspired by the Arduino [ModbusMaster][] library by 4-20ma.

[ModbusMaster]: https://github.com/4-20ma/ModbusMaster

## API

#### `ModbusMaster()`

Create new Modbus Master.

Params:
- `EventQueue* queue`: EventQueue used for processing response off IRQ context
- `Serial* serial`: Serial hardware to use for Modbus
- `int baud`: Serial baud rate used for Modbus frame delimiting
- `uin8_t slaveID`: Modbus Slave ID
- `int timeout = 50`: Modbus response timeout (ms)

#### `void attachPreTransmission(Callback<void()> f)`

Attach callback to be called before transmit. Can be used to toggle RS485 direction.

#### `void attachPostTransmission(Callback<void()> f)`

Attach callback to be called after transmit. Can be used to toggle RS485 direction.

#### `void setSlaveID(uint8_t id)`

Change modbus slave ID for subsiquent requests.

#### `void setTimeout(int t)`

Chnage receive timeout for subsiquent requests.

#### `uint8_t* getCoils()`

Get coils result after `readCoils` or `readDiscreteInputs`.
First result is on first byte LSB.

#### `uint16_t* getRegisters()`

Get register result after `readHoldingRegisters` or `readInputRegisters`.
Registers values are converted to little-endien.

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

