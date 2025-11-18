| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- | -------- | -------- |

# LD2410 presence sensor usage example

This project uses the LD2410 component to continuously check if there is a human in its range and write to log if there is.

## How to use

### Hardware Required

The example can be run on any development board, that is based on the Espressif SoC. The board shall be connected to a computer with a single USB cable for flashing and monitoring. The external interface should have 3.3V outputs.

Moreover a LD2410 should be connected to the board, and its pins can be configured as described in the next section.

### Setup the Hardware

Connect the LD2410 to the board as follows.

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

Use the command below to configure project using Kconfig menu as showed in the table above.
The default Kconfig values can be changed such as: LD2410_TASK_STACK_SIZE, LD2410_UART_BAUD_RATE, LD2410_UART_PORT_NUM (Refer to Kconfig files).

```
idf.py menuconfig
```

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type `Ctrl-]`.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.
