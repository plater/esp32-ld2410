# LD2410B/C ESP-IDF Component

This component eases the connection, configuration and use of the LD2410B/C presence sensor.
Most of the code is a ESP32 port of the [LD2410 Arduino library](https://github.com/iavorvel/MyLD2410).

Note: this library has been tested only on the ESP32-H2 dev board. Also not all of the methods have been tested. If you encounter a problem with a specific device or method please open an issue.

## How to use

### Installation

Add this component to an esp-idf project with the following command:

```bash
idf.py add-dependency "cosmavergari/ld2410"
```

### Setup the Hardware

Connect the LD2410 sensor to the board as follows.

```
  -----------------------------------------------------------------------------------------
  | Target chip Interface | Kconfig Option     | Default ESP Pin      | LD2410 pin        |
  | ----------------------|--------------------|----------------------|--------------------
  | Transmit Data (TxD)   | LD2410_UART_RX     | GPIO14               | RX                |
  | Receive Data (RxD)    | LD2410_UART_TX     | GPIO13               | TX                |
  | 5V                    | n/a                | 5V                   | 5V                |
  | Ground                | n/a                | GND                  | GND               |
  -----------------------------------------------------------------------------------------
```

Note: Some GPIOs can not be used with certain chips because they are reserved for internal use. Please refer to UART documentation for selected target.

### Configure the project

Use the command below to configure project using Kconfig menu as showed in the table above. The options for this sensor are under the "Driver LD2410 presence sensor" menu.

The default Kconfig values can be changed such as: LD2410_TASK_STACK_SIZE, LD2410_UART_BAUD_RATE, LD2410_UART_PORT_NUM (Refer to Kconfig files).

```
idf.py menuconfig
```
