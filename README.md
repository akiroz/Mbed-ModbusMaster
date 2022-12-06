# ModbusMaster

Asynchronous Modbus-RTU library for mbedOS 6.

This project is inspired by the Arduino [ModbusMaster][] library by 4-20ma.

**NOTE: Previous mbed 5 version can be found on the `mbed5` branch.**

[ModbusMaster]: https://github.com/4-20ma/ModbusMaster

## API

#### `ModbusMaster<>()`

Create a new ModbusMaster instance.

Template args:
- `size_t txBufSize` Transmit buffer size
- `size_t rxBufSize` Receive buffer size

Constructor args:
- `EventQueue* queue` EventQueue used for processing response off IRQ context
- `PinName txPin` Target TX pin
- `PinName rxPin` Target RX pin
- `int baud`: Serial baud rate (also used for Modbus frame delimiting)
- `uin8_t slaveId`: Modbus slave ID
- `std::chrono::milliseconds rxTimeout = 50ms`: Modbus response timeout

#### `void attachPreTransmit(Callback<void()> f)`

Attach callback to be called before transmit. Can be used to toggle RS485 direction.

#### `void attachPostTransmit(Callback<void()> f)`

Attach callback to be called after transmit. Can be used to toggle RS485 direction.

#### `void attachPostReceive(Callback<uint16_t(uint8_t* data, uint16_t len)> f)`

Attach callback to be called after receive frame, before CRC calculation.
(Can be used to modify received frame to workaround slave device quirks)

Callback function must return the buffer length.

#### `void setSlaveId(uint8_t id)`

Change modbus slave ID for subsiquent requests.

#### `void setTimeout(std::chrono::milliseconds t)`

Change receive timeout for subsiquent requests.

#### `void setCrcCheck(bool c)`

Change whether CRC check is performed on the received response.
(Can be used for slave devices with incorrect CRCs)

#### `uint8_t* getCoils()`

Get coils result after `readCoils` or `readDiscreteInputs`.
First result is on first byte LSB.

#### `uint16_t* getRegisters()`

Get register result after `readHoldingRegisters` or `readInputRegisters`.
Registers values are converted to little-endien.

#### `void readCoils(uint16_t addr, uint16_t num, Callback<void(Result)>)`

Modbus Read Coils function.

#### `void readDiscreteInputs(uint16_t addr, uint16_t num, Callback<void(Result)>)`

Modbus Read Discrete Inputs function.

#### `void writeSingleCoil(uint16_t addr, bool val, Callback<void(Result)>)`

Modbus Write Single Coils function.

#### `void writeMultipleCoils(uint16_t addr, uint16_t num, uint8_t* val, Callback<void(Result)>)`

Modbus Write Multiple Coils function.

#### `void readHoldingRegisters(uint16_t addr, uint16_t num, Callback<void(Result)>)`

Modbus Read Holding Registers function.

#### `void readInputRegisters(uint16_t addr, uint8_t num, Callback<void(Result)>)`

Modbus Read Input Registers function.

#### `void writeSingleRegister(uint16_t addr, uint16_t val, Callback<void(Result)>)`

Modbus Write Single Register function.

#### `void writeMultipleRegisters(uint16_t addr, uint16_t num, uint16_t* val, Callback<void(Result)>)`

Modbus Write Multiple Registers function.

## Example

Read registers:
```cpp
typedef ModbusMaster<16, 32> MBM;
EventQueue queue;
MBM modbus(&queue, PA_2, PA_3, 115200, 1);

int main() {
    modbus.readHoldingRegisters(1, 4, [](auto res){
        if(res != MBM::Result::success) {
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

