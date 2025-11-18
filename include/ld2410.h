#pragma once

#include <stdbool.h>

#define LD2410_LATEST_FIRMWARE "2.44"
#define LD2410_BUFFER_SIZE 0x40
#define FIRMWARE_STR_SIZE (20)
#define MAC_SIZE (6)
#define MAC_STR_SIZE (20)

#ifdef CONFIG_LD2410_OUT_PIN_CONNECTED
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

/**
 * @brief The function signature for the LD2410 OUT pin state change handler.
 * @param presence true if the sensor detects a presence (positive edge), false otherwise
 */
typedef void (*ld2410_callback_t)(bool presence);
#endif

/**
 * @brief The auxiliary light control status
 */
typedef enum LightControl
{
   LC_NOT_SET = -1,
   LC_NO_LIGHT_CONTROL,
   LC_LIGHT_BELOW_THRESHOLD,
   LC_LIGHT_ABOVE_THRESHOLD
} LightControl_t;

/**
 * @brief The auxiliary output control status
 */
typedef enum OutputControl
{
   OC_NOT_SET = -1,
   OC_DEFAULT_LOW,
   OC_DEFAULT_HIGH,
} OutputControl_t;

/**
 * @brief The status of the auto-thresholds routine
 */
typedef enum AutoStatus
{
   AS_NOT_SET = -1,
   AS_NOT_IN_PROGRESS,
   AS_IN_PROGRESS,
   AS_COMPLETED
} AutoStatus_t;

typedef enum Response
{
   RP_FAIL = 0,
   RP_ACK,
   RP_DATA
} Response_t;

typedef struct ValuesArray
{
   uint8_t values[9];
   uint8_t N;
} ValuesArray_t;

void va_copy(ValuesArray_t *destArray, ValuesArray_t *srcArray);
void va_setN(ValuesArray_t *array, uint8_t n);

typedef struct SensorData
{
   uint8_t status;
   int64_t timestamp_ms;
   unsigned long mTargetDistance;
   uint8_t mTargetSignal;
   unsigned long sTargetDistance;
   uint8_t sTargetSignal;
   unsigned long distance;
   // Enhanced data
   ValuesArray_t mTargetSignals;
   ValuesArray_t sTargetSignals;
} SensorData_t;

typedef struct LD2410_device
{
   bool isConfig;
   char MACstr[MAC_STR_SIZE];
   char firmware[FIRMWARE_STR_SIZE];
   unsigned long version;
   unsigned long bufferSize;
   uint8_t MAC[MAC_SIZE];
   uint8_t firmwareMajor;
   uint8_t firmwareMinor;
   LightControl_t lightControl;
   OutputControl_t outputControl;
   AutoStatus_t autoStatus;
   SensorData_t sData;
   ValuesArray_t stationaryThresholds;
   ValuesArray_t movingThresholds;
   uint8_t maxRange;
   uint8_t noOne_window;
   uint8_t lightLevel;
   uint8_t outLevel;
   uint8_t lightThreshold;
   int fineRes;
   bool isEnhanced;
   unsigned long timeout;
   unsigned long dataLifespan;

   // Buffer pointers
   uint8_t *data;
   uint8_t *head_buf;
   int head_buf_i;
   uint8_t *in_buf;
   uint8_t in_buf_i;

#ifdef CONFIG_LD2410_OUT_PIN_CONNECTED
   gpio_num_t out_pin;
   ld2410_callback_t out_pin_callback;
#endif
} LD2410_device_t;

// CONTROLS
/**
 * @brief Create an instance of the LD2410 device
 */
LD2410_device_t *ld2410_new();

/**
 * @brief Call this function in setup() to ascertain whether the device is responding
 */
bool ld2410_begin(LD2410_device_t *device);

/**
 * @brief This function empties the UART FIFO buffers related to the device.
 * Call this function when you want to discard the previous device messages.
 */
void ld2410_flush(LD2410_device_t *device);

/**
 * @brief Free the memory allocated to the device handle. Call this only if you
 * do not intend to use the device handle anymore.
 */
void ld2410_free(LD2410_device_t *device);

/**
 * @brief Call this function to gracefully close the sensor. Useful for entering sleep mode.
 */
void ld2410_end(LD2410_device_t *device);

/**
  @brief Call this function in the main loop
  @return RP_DATA = (evaluates to true) if the latest frame contained data
  @return RP_ACK  = (evaluates to true) if the latest frame contained a reply to a command
  @return RP_FAIL = (evaluates to false) if no useful info was processed
  */
Response_t ld2410_check(LD2410_device_t *device);

#ifdef CONFIG_LD2410_OUT_PIN_CONNECTED
/**
 * @brief Attach the callback for the device
 * @note This function must be called BEFORE installing the ISR service (gpio_install_isr_service)
 *
 * @param device LD2410 device handle
 * @param cb Callback function pointer
 * @param out_pin The GPIO the OUT pin is connected to
 * @return esp_err_t ESP_OK if the callback is correctly attached
 */
esp_err_t ld2410_out_pin_init_handler(LD2410_device_t *device, gpio_num_t out_pin, ld2410_callback_t cb);
/**
 * @brief Attach the callback for the device
 * @note This function must be called AFTER installing the ISR service (gpio_install_isr_service)
 *
 * @param device LD2410 device handle
 * @return esp_err_t ESP_OK if the callback is correctly attached
 */
esp_err_t ld2410_out_pin_attach_handler(LD2410_device_t *device);
#endif

// GETTERS

/**
 * @brief Check whether the device is in config mode
 * (accepts commands)
 *
 * @param device LD2410 device handle
 */
bool ld2410_in_config_mode(LD2410_device_t *device);

/**
 * @brief Check whether the device is in basic mode
 * (continuously sends basic presence data)
 *
 * @param device LD2410 device handle
 */
bool ld2410_in_basic_mode(LD2410_device_t *device);

/**
 * @brief Check whether the device is in enhanced mode
 * (continuously sends enhanced presence data)
 *
 * @param device LD2410 device handle
 */
bool ld2410_in_enhanced_mode(LD2410_device_t *device);

/**
 * @brief Get the status of the sensor:
 * 0 - No presence;
 * 1 - Moving only;
 * 2 - Stationary only;
 * 3 - Both moving and stationary;
 * 4 - Auto thresholds in progress;
 * 5 - Auto thresholds successful;
 * 6 - Auto thresholds failed;
 * 255 - The sensor status is invalid
 *
 *
 * @param device LD2410 device handle
 * @return uint8_t
 */
uint8_t ld2410_get_status(LD2410_device_t *device);

/**
 * @brief Get the presence status as a c-string
 *
 *
 * @param device LD2410 device handle
 * @return const char* :
 * "No target",
 * "Moving only",
 * "Stationary only",
 * "Both moving and stationary",
 * "Auto thresholds in progress",
 * "Auto thresholds successful",
 * "Auto thresholds failed".
 */
const char *ld2410_status_string(LD2410_device_t *device);

/**
 * @brief Check whether presence was detected in the latest frame
 *
 * @param device LD2410 device handle
 */
bool ld2410_presence_detected(LD2410_device_t *device);

/**
 * @brief Check whether a stationary target was detected in the latest frame
 *
 * @param device LD2410 device handle
 */
bool ld2410_stationary_target_detected(LD2410_device_t *device);

/**
 * @brief Get the distance to the stationary target in [cm]
 *
 * @param device LD2410 device handle
 * @return unsigned long - distance in [cm]
 */
unsigned long ld2410_stationary_target_distance(LD2410_device_t *device);

/**
 * @brief Get the signal from the stationary target
 *
 * @param device LD2410 device handle
 * @return uint8_t - signal value [0:100]
 */
uint8_t ld2410_stationary_target_signal(LD2410_device_t *device);

/**
 * @brief Get the Stationary Signals object, if in enhanced mode
 *
 * @param device LD2410 device handle
 * @return ValuesArray_t - the signals for each detection gate
 */
ValuesArray_t ld2410_get_stationary_signals(LD2410_device_t *device);

/**
 * @brief Check whether a moving target was detected in the latest frame
 *
 * @param device LD2410 device handle
 */
bool ld2410_moving_target_detected(LD2410_device_t *device);

/**
 * @brief Get the distance to the moving target in [cm]
 *
 * @param device LD2410 device handle
 * @return unsigned long - distance in [cm]
 */
unsigned long ld2410_moving_target_distance(LD2410_device_t *device);

/**
 * @brief Get the signal from the moving target
 *
 * @param device LD2410 device handle
 * @return uint8_t - signal value [0:100]
 */
uint8_t ld2410_moving_target_signal(LD2410_device_t *device);

/**
 * @brief Get the Moving Signals object, if in enhanced mode
 *
 * @param device LD2410 device handle
 * @return ValuesArray_t - the signals for each detection gate
 */
ValuesArray_t ld2410_get_moving_signals(LD2410_device_t *device);

/**
 * @brief Get the detected distance
 *
 * @param device LD2410 device handle
 * @return unsigned long - distance in [cm]
 */
unsigned long ld2410_detected_distance(LD2410_device_t *device);

/**
 * @brief Get the Bluetooth MAC address as an array uint8_t[6]
 *
 * @param device LD2410 device handle
 * @param return_ACM uint8_t array to output the MAC address on
 * @return const uint8_t*
 */
void ld2410_get_MAC(LD2410_device_t *device, uint8_t *return_MAC);

/**
 * @brief Get the Bluetooth MAC address as a String
 *
 * @param device LD2410 device handle
 * @param return_ACM char array to output the MAC address on
 * @return String
 */
void ld2410_get_MAC_str(LD2410_device_t *device, char *return_str);

/**
 * @brief Get the Firmware as a String
 *
 * @param device LD2410 device handle
 * @param return_ACM char array to output the firmware version on
 * @return String
 */
void ld2410_get_firmware(LD2410_device_t *device, char *return_str);

/**
 *  @brief Get the Firmware Major
 *
 *  @param device LD2410 device handle
 *  @return uint8_t
 */
uint8_t ld2410_get_firmware_major(LD2410_device_t *device);

/**
 *  @brief Get the Firmware Minor
 *
 * @param device LD2410 device handle
 *  @return uint8_t
 */
uint8_t ld2410_get_firmware_minor(LD2410_device_t *device);

/**
 * @brief Get the protocol version
 *
 * @param device LD2410 device handle
 * @return unsigned long
 */
unsigned long ld2410_get_version(LD2410_device_t *device);

/**
 * @brief Get the SensorData object
 *
 * @param device LD2410 device handle
 * @return SensorData_t
 */
SensorData_t ld2410_get_sensor_data(LD2410_device_t *device);

/**
 * @brief Get the sensor resolution (gate-width) in [cm]
 *
 * @param device LD2410 device handle
 * @return uint8_t either 20 or 75 on success, 0 on failure
 */
uint8_t ld2410_get_resolution(LD2410_device_t *device);

// parameters

/**
 * @brief Get the detection thresholds for moving targets
 *
 * @param device LD2410 device handle
 * @return ValuesArray_t
 */
ValuesArray_t ld2410_get_moving_thresholds(LD2410_device_t *device);

/**
 * @brief Get the detection thresholds for stationary targets
 *
 * @param device LD2410 device handle
 * @return ValuesArray_t
 */
ValuesArray_t ld2410_get_stationary_thresholds(LD2410_device_t *device);

/**
 * @brief Get the maximum detection gate
 *
 * @param device LD2410 device handle
 * @return uint8_t
 */
uint8_t ld2410_get_range(LD2410_device_t *device);

/**
 * @brief Get the maximum detection range in [cm]
 *
 * @param device LD2410 device handle
 * @return unsigned long
 */
unsigned long ld2410_get_range_cm(LD2410_device_t *device);

/**
 * @brief Get the time-lag of "no presence" in [s].
 * The sensor begins reporting "no presence"
 * only after no motion has been detected for that many seconds.
 *
 * @param device LD2410 device handle
 * @return uint8_t
 */
uint8_t ld2410_get_no_one_window(LD2410_device_t *device);
// end parameters

// REQUESTS

/**
 * @brief Request config mode
 *
 * @param device LD2410 device handle
 * @param enable [true]/false
 * @return true on success
 */
bool ld2410_config_mode(LD2410_device_t *device, bool enable);

/**
 * @brief Request enhanced mode
 *
 * @param device LD2410 device handle
 * @param enable [true]/false
 * @return true on success
 */
bool ld2410_enhanced_mode(LD2410_device_t *device, bool enable);

/**
 * @brief Request the current auxiliary configuration
 *
 * @param device LD2410 device handle
 * @return true on success
 */
bool ld2410_request_aux_config(LD2410_device_t *device);

/**
 * @brief Begin the automatic threshold detection routine
 * (firmware >= 2.44)
 *
 * @param device LD2410 device handle
 * @param _timeout - allow for timeout [s] to leave the room: default 10
 * @return true on success
 */
bool ld2410_auto_thresholds(LD2410_device_t *device, uint8_t _timeout);

/**
 * @brief Get the status of the automatic threshold detection routine
 * (firmware >= 2.44)
 *
 * @param device LD2410 device handle
 * @return AutoStatus_t
 */
AutoStatus_t ld2410_get_auto_status(LD2410_device_t *device);

/**
 * @brief Request the Bluetooth MAC address
 *
 * @param device LD2410 device handle
 * @return true on success
 */
bool ld2410_request_MAC(LD2410_device_t *device);

/**
 * @brief Request the Firmware
 *
 * @param device LD2410 device handle
 * @return true on success
 */
bool ld2410_request_firmware(LD2410_device_t *device);

/**
 * @brief Request the resolution (gate-width)
 *
 * @param device LD2410 device handle
 * @return true on success
 */
bool ld2410_request_resolution(LD2410_device_t *device);

/**
 * @brief Set the resolution of the sensor
 *
 * @param device LD2410 device handle
 * @param fine true=20cm; [default: false]=75cm
 * @return true on success
 */
bool ld2410_set_resolution(LD2410_device_t *device, bool fine);

/**
 * @brief Request the sensor parameters:
 * range, motion thresholds, stationary thresholds, no-one window
 *
 * @param device LD2410 device handle
 * @return true on success
 */
bool ld2410_request_parameters(LD2410_device_t *device);

/**
 * @brief Set the gate parameters for a particular gate, or for all gates at once
 *
 * @param device LD2410 device handle
 * @param gate the gate to configure;
 * pass a value greater than 8 (e.g 0xFF) to apply the same thresholds to all gates
 * @param movingThreshold [0 - 100] default: 100
 * @param stationaryThreshold [0 - 100] default: 100
 * @return true on success
 */
bool ld2410_set_gate_parameters(LD2410_device_t *device, uint8_t gate, uint8_t movingThreshold, uint8_t stationaryThreshold);

/**
 * @brief Set the parameters for all gates at once, as well as the no-one window
 *
 * @todo
 * @param device LD2410 device handle
 * @param moving_thresholds as a ValueArray
 * @param stationary_thresholds as a ValueArray
 * @param noOneWindow
 * @return true on success
 */
// bool setGateParameters(ValuesArray_t moving_thresholds, ValuesArray_t stationary_thresholds, uint8_t noOneWindow); // TODO: noOneWindow default 5

/**
 * @brief Set the detection range for moving targets, stationary targets, as well as the no-one window
 *
 * @param device LD2410 device handle
 * @param movingGate
 * @param stationaryGate
 * @param noOneWindow default: 5
 * @return true on success
 */
bool ld2410_set_max_gate(LD2410_device_t *device, uint8_t movingGate, uint8_t staticGate, uint8_t noOneWindow);

/**
 * @brief Set the no-one window parameter
 *
 * @param device LD2410 device handle
 * @param noOneWindow in [s]
 * @return true on success
 */
bool ld2410_set_no_one_window(LD2410_device_t *device, uint8_t noOneWindow);

/**
 * @brief Set the maximum moving gate
 *
 * @param device LD2410 device handle
 * @param movingGate
 * @return true on success
 */
bool ld2410_set_max_moving_gate(LD2410_device_t *device, uint8_t movingGate);

/**
 * @brief Set the maximum stationary gate
 *
 * @param device LD2410 device handle
 * @param stationaryGate
 * @return true on success
 */
bool ld2410_set_max_stationary_gate(LD2410_device_t *device, uint8_t stationaryGate);

/**
 * @brief Get the maximum moving gate
 *
 * @param device LD2410 device handle
 * @return the maximum moving-target gate
 */
uint8_t ld2410_get_max_moving_gate(LD2410_device_t *device);

/**
 * @brief Get the maximum stationary gate
 *
 * @param device LD2410 device handle
 * @return the maximum stationary-target gate
 */
uint8_t ld2410_get_max_stationary_gate(LD2410_device_t *device);

/**
 * @brief Request reset to factory default parameters
 *
 * @param device LD2410 device handle
 * @return true on success
 */
bool ld2410_request_reset(LD2410_device_t *device);

/**
 * @brief Request reboot
 *
 * @param device LD2410 device handle
 * @return true on success
 */
bool ld2410_request_reboot(LD2410_device_t *device);

/**
 * @brief Turn Bluetooth ON
 *
 * @param device LD2410 device handle
 * @return true on success
 */
bool ld2410_request_BT_on(LD2410_device_t *device);

/**
 * @brief Turn Bluetooth OFF
 *
 * @param device LD2410 device handle
 * @return true on success
 */
bool ld2410_request_BT_off(LD2410_device_t *device);

/**
 * @brief Set a new BT password.
 *
 * The BT password must be 6 characters long. If the string is shorter, it will be padded with spaces '\20'. If it is longer, only the first 6 characters will be used.
 *
 * @param device LD2410 device handle
 * @param passwd char array
 * @return true on success
 */
bool ld2410_set_BT_password(LD2410_device_t *device, const char *passwd);

/**
 * @brief Reset the BT password
 *
 * @param device LD2410 device handle
 * @return true on success
 */
bool ld2410_reset_BT_password(LD2410_device_t *device);

/**
 * @brief Reset the serial baud rate. The sensor reboots at the new rate on success
 *
 * @param device LD2410 device handle
 * @return true on success
 */
bool ld2410_set_baud(LD2410_device_t *device, uint8_t baud);

/**
 * @brief Get the Light Level
 *
 * @param device LD2410 device handle
 * @return uint8_t
 */
uint8_t ld2410_get_light_level(LD2410_device_t *device);

/**
 * @brief Get the Light Control parameter
 *
 * @param device LD2410 device handle
 * @return LightControl_t enum
 */
LightControl_t ld2410_get_light_control(LD2410_device_t *device);

/**
 * @brief Set the Auxiliary Control parameters
 *
 * @param device LD2410 device handle
 * @param light_control
 * @param light_threshold
 * @param output_control
 * @return true on success
 */
bool ld2410_set_aux_control(LD2410_device_t *device,
                            LightControl_t light_control,
                            uint8_t light_threshold,
                            OutputControl_t output_control);

/**
 * @brief Reset the Auxiliary Control parameters to their default values
 *
 * @param device LD2410 device handle
 * @return true on success
 */
bool ld2410_reset_aux_control(LD2410_device_t *device);

/**
 * @brief Get the Light Threshold
 *
 * @param device LD2410 device handle
 * @return uint8_t
 */
uint8_t ld2410_get_light_threshold(LD2410_device_t *device);

/**
 * @brief Get the Output Control parameter
 *
 * @param device LD2410 device handle
 * @return OutputControl_t enum
 */
OutputControl_t ld2410_get_output_control(LD2410_device_t *device);

/**
 * @brief Get the Light Level
 *
 * @param device LD2410 device handle
 * @return uint8_t
 */
uint8_t getOutLevel(LD2410_device_t *device);
