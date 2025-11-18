#include <stdio.h>
#include <string.h>
#include "ld2410.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"

#ifdef DEBUG
#define DEBUG 1
#else
#define DEBUG 0
#endif

#define BUF_SIZE (1024)
#define HEAD_BUF_SIZE (4)

#define UART_RX_READ_TIMEOUT_MS 20
#define OUT_PIN_NULL -1

static const char *TAG = "LD2410";

const char *LD2410_tStatus[7] = {
    "No target",
    "Moving only",
    "Stationary only",
    "Both moving and stationary",
    "Auto thresholds in progress",
    "Auto thresholds successful",
    "Auto thresholds failed"};
const uint8_t LD2410_Head_Data[4] = {0xF4, 0xF3, 0xF2, 0xF1};
const uint8_t LD2410_Tail_Data[4] = {0xF8, 0xF7, 0xF6, 0xF5};
const uint8_t LD2410_Head_Config[4] = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t LD2410_Tail_Config[4] = {4, 3, 2, 1};
const uint8_t LD2410_Config_Enable[6] = {4, 0, 0xFF, 0, 1, 0};
const uint8_t LD2410_Config_Disable[4] = {2, 0, 0xFE, 0};
const uint8_t LD2410_MAC[6] = {4, 0, 0xA5, 0, 1, 0};
const uint8_t LD2410_firmware[4] = {2, 0, 0xA0, 0};
const uint8_t LD2410_Res[4] = {2, 0, 0xAB, 0};
const uint8_t LD2410_Res_Coarse[6] = {4, 0, 0xAA, 0, 0, 0};
const uint8_t LD2410_Res_Fine[6] = {4, 0, 0xAA, 0, 1, 0};
const uint8_t LD2410_Change_Baud[6] = {4, 0, 0xA1, 0, 7, 0};
const uint8_t LD2410_Reset[4] = {2, 0, 0xA2, 0};
const uint8_t LD2410_Reboot[4] = {2, 0, 0xA3, 0};
const uint8_t LD2410_BTon[6] = {4, 0, 0xA4, 0, 1, 0};
const uint8_t LD2410_BToff[6] = {4, 0, 0xA4, 0, 0, 0};
const uint8_t LD2410_BTpasswd[10] = {8, 0, 0xA9, 0, 0x48, 0x69, 0x4C, 0x69, 0x6E, 0x6B};
const uint8_t LD2410_Param[4] = {2, 0, 0x61, 0};
const uint8_t LD2410_Eng_On[4] = {2, 0, 0x62, 0};
const uint8_t LD2410_Eng_Off[4] = {2, 0, 0x63, 0};
const uint8_t LD2410_Aux_Query[4] = {2, 0, 0xAE, 0};
const uint8_t LD2410_Aux_Config[8] = {6, 0, 0xAD, 0, 0, 0x80, 0, 0};
const uint8_t LD2410_Auto_Begin[6] = {4, 0, 0x0B, 0, 0x0A, 0};
const uint8_t LD2410_Auto_Query[4] = {2, 0, 0x1B, 0};
uint8_t LD2410_Gate_Param[0x16] = {0x14, 0, 0x64, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0};
uint8_t LD2410_Max_Gate[0x16] = {0x14, 0, 0x60, 0, 0, 0, 8, 0, 0, 0, 1, 0, 8, 0, 0, 0, 2, 0, 5, 0, 0, 0};

#pragma region UtilFunctions
void byte2hex(uint8_t byte, char *hex, bool addZero)
{
   const char hex_digits[] = "0123456789ABCDEF";
   uint8_t high_nibble = (byte >> 4) & 0x0F;
   if (high_nibble == 0 && !addZero)
   {
      hex[0] = hex_digits[byte & 0x0F];
      hex[1] = '\0';
      return;
   }
   hex[0] = hex_digits[high_nibble]; // High nibble
   hex[1] = hex_digits[byte & 0x0F]; // Low nibble
   hex[2] = '\0';
}

void printBuf(const uint8_t *buf, uint8_t size)
{
   char hex[3];
   for (uint8_t i = 0; i < size; i++)
   {
      byte2hex(buf[i], hex, true);
      printf("%s ", hex);
   }
   printf("\n");
}

bool buffer_ends_with(const uint8_t *buf, int iMax, const uint8_t *other)
{
   for (int j = 3; j >= 0; j--)
   {
      if (--iMax < 0)
         iMax = 3;
      if (buf[iMax] != other[j])
         return false;
   }
   return true;
}
#pragma endregion

LD2410_device_t *ld2410_new()
{
   // Create instance
   LD2410_device_t *device = (LD2410_device_t *)malloc(sizeof(LD2410_device_t));
   // Initialize members
   device->isConfig = false;
   device->version = 0;
   device->bufferSize = 0;
   device->firmwareMajor = 0;
   device->firmwareMinor = 0;
   device->lightControl = LC_NOT_SET;
   device->outputControl = OC_NOT_SET;
   device->lightControl = AS_NOT_SET;
   device->maxRange = 0;
   device->noOne_window = 0;
   device->lightLevel = 0;
   device->outLevel = 0;
   device->lightThreshold = 0;
   device->fineRes = -1;
   device->isEnhanced = false;
   device->timeout = 2000;
   device->dataLifespan = 500;
   // Initialize the buffers
   device->data = (uint8_t *)malloc(BUF_SIZE);
   device->head_buf = (uint8_t *)malloc(HEAD_BUF_SIZE);
   device->head_buf_i = 0;
   device->in_buf = (uint8_t *)malloc(LD2410_BUFFER_SIZE);
   device->in_buf_i = 0;

#ifdef CONFIG_LD2410_OUT_PIN_CONNECTED
   device->out_pin = OUT_PIN_NULL;
   device->out_pin_callback = NULL;
#endif

   return device;
}

bool ld2410_process_ack();
bool ld2410_process_data();

Response_t ld2410_check(LD2410_device_t *device)
{
   int len = 0;
   ESP_ERROR_CHECK(uart_get_buffered_data_len(CONFIG_LD2410_UART_PORT_NUM, (size_t *)&len));
   while (len > 0)
   {
      // There is actual data to be read, read it starting from the header
      len = uart_read_bytes(CONFIG_LD2410_UART_PORT_NUM, device->head_buf + device->head_buf_i, 1, UART_RX_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
      device->head_buf_i++;
      device->head_buf_i %= 4;
      if (DEBUG)
         ESP_LOGD(TAG, "Reading in check: %02x %02x %02x %02x ", (device->head_buf)[0], (device->head_buf)[1], (device->head_buf)[2], (device->head_buf)[3]);
      if (buffer_ends_with(device->head_buf, device->head_buf_i, LD2410_Head_Config) && ld2410_process_ack(device))
         return RP_ACK;
      if (buffer_ends_with(device->head_buf, device->head_buf_i, LD2410_Head_Data) && ld2410_process_data(device))
         return RP_DATA;
      ESP_ERROR_CHECK(uart_get_buffered_data_len(CONFIG_LD2410_UART_PORT_NUM, (size_t *)&len));
   }
   return RP_FAIL;
}

bool ld2410_send_command(LD2410_device_t *device, const uint8_t *command)
{
   uint8_t size = command[0] + 2;
   if (DEBUG)
      ESP_LOGD(TAG, "Sending following command");
   // printBuf(command, size);
   uart_write_bytes(CONFIG_LD2410_UART_PORT_NUM, LD2410_Head_Config, 4);
   uart_write_bytes(CONFIG_LD2410_UART_PORT_NUM, command, size);
   uart_write_bytes_with_break(CONFIG_LD2410_UART_PORT_NUM, LD2410_Tail_Config, 4, 10 / portTICK_PERIOD_MS);
   unsigned long giveUp = (esp_timer_get_time() / 1000) + device->timeout;
   if (DEBUG)
      ESP_LOGD(TAG, "Waiting for ACK");

   while ((esp_timer_get_time() / 1000) < giveUp)
   {
      int len = 0;
      ESP_ERROR_CHECK(uart_get_buffered_data_len(CONFIG_LD2410_UART_PORT_NUM, (size_t *)&len));
      while (len > 0)
      {
         len = uart_read_bytes(CONFIG_LD2410_UART_PORT_NUM, device->head_buf + device->head_buf_i, 1, UART_RX_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
         if (DEBUG)
            ESP_LOGD(TAG, "Reading header in sendCommand: %02x %02x %02x %02x ", device->head_buf[0], device->head_buf[1], device->head_buf[2], device->head_buf[3]);
         (device->head_buf_i)++;
         (device->head_buf_i) %= 4;
         if (buffer_ends_with(device->head_buf, device->head_buf_i, LD2410_Head_Config))
            return ld2410_process_ack(device);
      }
   }
   ESP_LOGE(TAG, "Send command fail, timeout");
   return false;
}

bool ld_2410_read_frame(LD2410_device_t *device)
{
   int frameSize = 0, bytes = 2, len;
   device->in_buf_i = 0;
   while (bytes > 0)
   {
      ESP_ERROR_CHECK(uart_get_buffered_data_len(CONFIG_LD2410_UART_PORT_NUM, (size_t *)&len));
      if (len > 0)
      {
         len = uart_read_bytes(CONFIG_LD2410_UART_PORT_NUM, device->in_buf + device->in_buf_i, bytes, UART_RX_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
         device->in_buf_i += len;
         bytes -= len;
      }
   }

   for (uint8_t i = 0; i < device->in_buf_i; i++)
   {
      frameSize |= (device->in_buf)[i] << i * 8;
   }
   if (frameSize <= 0)
      return false;
   frameSize += 4;

   device->in_buf_i = 0;
   while (frameSize > 0)
   {
      ESP_ERROR_CHECK(uart_get_buffered_data_len(CONFIG_LD2410_UART_PORT_NUM, (size_t *)&len));
      if (len > 0)
      {
         len = uart_read_bytes(CONFIG_LD2410_UART_PORT_NUM, device->in_buf + device->in_buf_i, frameSize, UART_RX_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
         device->in_buf_i += len;
         frameSize -= len;
      }
   }
   return true;
}

bool ld2410_process_ack(LD2410_device_t *device)
{
   if (DEBUG)
      ESP_LOGD(TAG, "Processing ACK");
   if (!ld_2410_read_frame(device))
   {
      if (DEBUG)
         ESP_LOGD(TAG, "Unable to read ack frame");
      return false;
   }
   // printBuf(device->in_buf, device->in_buf_i);
   if (!buffer_ends_with(device->in_buf, device->in_buf_i, LD2410_Tail_Config))
   {
      if (DEBUG)
         ESP_LOGD(TAG, "Invalid ACK buffer termination");
      return false;
   }
   unsigned long command = (device->in_buf)[0] | ((device->in_buf)[1] << 8);
   if ((device->in_buf)[2] | ((device->in_buf)[3] << 8))
   {
      if (DEBUG)
         ESP_LOGD(TAG, "Invalid ACK");
      return false;
   }
   switch (command)
   {
   case 0x1FF: // entered config mode
      device->isConfig = true;
      device->version = (device->in_buf)[4] | ((device->in_buf)[5] << 8);
      device->bufferSize = (device->in_buf)[6] | ((device->in_buf)[7] << 8);
      break;
   case 0x1FE: // exited config mode
      device->isConfig = false;
      break;
   case 0x1A5: // MAC
      for (int i = 0; i < 6; i++)
         (device->MAC)[i] = (device->in_buf)[i + 4];
      snprintf(device->MACstr, MAC_STR_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X",
               (device->MAC)[0], (device->MAC)[1], (device->MAC)[2], (device->MAC)[3], (device->MAC)[4], (device->MAC)[5]);
      break;
   case 0x1A0: // Firmware
      snprintf(device->firmware, FIRMWARE_STR_SIZE, "%X.%02X.%02X%X%X%X", (device->in_buf)[7], (device->in_buf)[6],
               (device->in_buf)[11], (device->in_buf)[10], (device->in_buf)[9], (device->in_buf)[8]);
      device->firmwareMajor = (device->in_buf)[7];
      device->firmwareMinor = (device->in_buf)[6];
      break;
   case 0x1AB: // Query Resolution
      device->fineRes = (device->in_buf)[4];
      break;
   case 0x1AE: // Query auxiliary control parameters
      device->lightControl = (LightControl_t)(device->in_buf)[4];
      device->lightThreshold = (device->in_buf)[5];
      device->outputControl = (OutputControl_t)(device->in_buf)[6];
      break;
   case 0x11B:
      device->autoStatus = (AutoStatus_t)(device->in_buf)[4];
      break;
   case 0x1A3: // Reboot
      device->isEnhanced = false;
      device->isConfig = false;
      break;
   case 0x161: // Query parameters
      device->maxRange = (device->in_buf)[5];
      va_setN(&(device->movingThresholds), (device->in_buf)[6]);
      va_setN(&(device->stationaryThresholds), (device->in_buf)[7]);
      for (uint8_t i = 0; i <= (device->movingThresholds).N; i++)
         (device->movingThresholds).values[i] = (device->in_buf)[8 + i];
      for (uint8_t i = 0; i <= (device->stationaryThresholds).N; i++)
         (device->stationaryThresholds).values[i] = (device->in_buf)[17 + i];
      device->noOne_window = (device->in_buf)[26] | ((device->in_buf)[27] << 8);
      break;
   case 0x162:
      device->isEnhanced = true;
      break;
   case 0x163:
      device->isEnhanced = false;
      break;
   case 0x164:
      if (LD2410_Gate_Param[7] == 0xFF)
         LD2410_Gate_Param[7] = 0;
   }
   return (true);
}

bool ld2410_process_data(LD2410_device_t *device)
{
   if (DEBUG)
      ESP_LOGD(TAG, "Processing data");
   if (!ld_2410_read_frame(device))
   {
      if (DEBUG)
         ESP_LOGD(TAG, "Unable to read data frame");
      return false;
   }
   // printBuf(device->in_buf, device->in_buf_i);
   if (!buffer_ends_with(device->in_buf, device->in_buf_i, LD2410_Tail_Data))
   {
      if (DEBUG)
         ESP_LOGD(TAG, "Invalid data buffer termination");
      return false;
   }
   if ((((device->in_buf)[0] == 1) || ((device->in_buf)[0] == 2)) && ((device->in_buf)[1] == 0xAA))
   { // Basic mode and Enhanced
      device->sData.timestamp_ms = esp_timer_get_time() / 1000;
      device->sData.status = (device->in_buf)[2] & 7;
      device->sData.mTargetDistance = (device->in_buf)[3] | ((device->in_buf)[4] << 8);
      device->sData.mTargetSignal = (device->in_buf)[5];
      device->sData.sTargetDistance = (device->in_buf)[6] | ((device->in_buf)[7] << 8);
      device->sData.sTargetSignal = (device->in_buf)[8];
      device->sData.distance = (device->in_buf)[9] | ((device->in_buf)[10] << 8);
      if (device->in_buf[0] == 1)
      { // Enhanced mode only
         device->isEnhanced = true;
         va_setN(&(device->sData.mTargetSignals), (device->in_buf)[11]);
         va_setN(&(device->sData.sTargetSignals), (device->in_buf)[12]);
         uint8_t *p = device->in_buf + 13;
         for (uint8_t i = 0; i <= (device->sData).mTargetSignals.N; i++)
            (device->sData).mTargetSignals.values[i] = *(p++);
         for (uint8_t i = 0; i <= (device->sData).sTargetSignals.N; i++)
            (device->sData).sTargetSignals.values[i] = *(p++);
         device->lightLevel = *(p++);
         device->outLevel = *p;
      }
      else
      { // Basic mode only
         device->isEnhanced = false;
         va_setN(&(device->sData.mTargetSignals), 0);
         va_setN(&(device->sData.sTargetSignals), 0);
         device->lightLevel = 0;
         device->outLevel = 0;
      }
   }
   else
      return false;
   return true;
}

bool ld2410_begin(LD2410_device_t *device)
{
   /* Configure parameters of an UART driver,
    * communication pins and install the driver */
   uart_config_t uart_config = {
       .baud_rate = CONFIG_LD2410_UART_BAUD_RATE,
       .data_bits = UART_DATA_8_BITS,
       .parity = UART_PARITY_DISABLE,
       .stop_bits = UART_STOP_BITS_1,
       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
       .source_clk = UART_SCLK_DEFAULT,
   };

   int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
   intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

   ESP_ERROR_CHECK(uart_driver_install(CONFIG_LD2410_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
   ESP_ERROR_CHECK(uart_param_config(CONFIG_LD2410_UART_PORT_NUM, &uart_config));
   ESP_ERROR_CHECK(uart_set_pin(CONFIG_LD2410_UART_PORT_NUM, CONFIG_LD2410_UART_TX, CONFIG_LD2410_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

   if (DEBUG)
      ESP_LOGD(TAG, "Begin initiated");
   // Wait for the sensor to come online, or to timeout.
   unsigned long giveUp = (esp_timer_get_time() / 1000) + device->timeout;
   bool online = false;
   device->isConfig = false;
   if (DEBUG)
      ESP_LOGD(TAG, "Sending config disable command");
   ld2410_send_command(device, LD2410_Config_Disable);
   while ((esp_timer_get_time() / 1000) < giveUp)
   {
      if (ld2410_check(device))
      {
         if (DEBUG)
            ESP_LOGD(TAG, "ACK received");
         online = true;
         break;
      }
      vTaskDelay(10);
   }
   if (DEBUG)
      ESP_LOGD(TAG, "%s begin", online ? "Successful" : "Failed");
   return online;
}

void ld2410_flush(LD2410_device_t *device)
{
   uart_flush_input(CONFIG_LD2410_UART_PORT_NUM);
   uart_wait_tx_done(CONFIG_LD2410_UART_PORT_NUM, 1000 / portTICK_PERIOD_MS);
}

void ld2410_free(LD2410_device_t *device)
{
   if (device->in_buf)
      free(device->in_buf);
   if (device->data)
      free(device->data);
   if (device->head_buf)
      free(device->head_buf);
   free(device);
}

#ifdef CONFIG_LD2410_OUT_PIN_CONNECTED
#define OUT_PIN_EVENT_QUEUE_SIZE 10

/**
 * Single event queue since all we care is propagating to device, the reading of the new
 * GPIO value is done in the handler task.
 */
static QueueHandle_t ld2410_gpio_evt_queue;

static void IRAM_ATTR ld2410_isr_handler(void *arg)
{
   LD2410_device_t *device = (LD2410_device_t *)arg;
   xQueueSendFromISR(ld2410_gpio_evt_queue, &device, NULL);
}

bool ld2410_presence_detected_no_validity_check(LD2410_device_t *device);
static void ld2410_out_pin_task(void *arg)
{
   LD2410_device_t *device;
   for (;;)
   {
      if (xQueueReceive(ld2410_gpio_evt_queue, &device, portMAX_DELAY))
      {
         if (DEBUG)
            ESP_LOGD(TAG, "Device on out pin %d changed state", device->out_pin);

         // Clear the FIFO queues
         ld2410_flush(device);

         // Take the value stored before this state change
         bool prevPresence = ld2410_presence_detected_no_validity_check(device);

         bool confirmed = false;
         for (int i = 0; i < 100; i++)
         {
            if (ld2410_check(device))
            {
               bool newPresence = ld2410_presence_detected(device);
               if (prevPresence != newPresence)
               {
                  if (DEBUG)
                     ESP_LOGD(TAG, "Presence change confirmed by device. Calling callback with value: %d", newPresence);
                  device->out_pin_callback(newPresence);
                  confirmed = true;
                  break;
               }
            }
            vTaskDelay(10 / portTICK_PERIOD_MS); // Needed for device to read UART
         }
         if (!confirmed)
         {
            ESP_LOGW(TAG, "The device issued a change on OUT pin %d but did not output the new value via UART (port %d)",
                     device->out_pin, CONFIG_LD2410_UART_PORT_NUM);
         }
      }
   }
}

esp_err_t ld2410_out_pin_init_handler(LD2410_device_t *device, gpio_num_t out_pin, ld2410_callback_t cb)
{
   device->out_pin = out_pin;
   device->out_pin_callback = cb;

   // Create the GPIO config structure
   gpio_config_t io_conf = {};
   io_conf.intr_type = GPIO_INTR_ANYEDGE; // Trigger ISR on positive and negative edge
   io_conf.pin_bit_mask = (1ULL << out_pin);
   io_conf.mode = GPIO_MODE_INPUT;
   io_conf.pull_up_en = 1; // enable pull-up
   ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Failed to config OUT pin GPIO");

   // This is the first time this method is run so create the handling queue and task
   if (!ld2410_gpio_evt_queue)
   {
      ld2410_gpio_evt_queue = xQueueCreate(OUT_PIN_EVENT_QUEUE_SIZE, sizeof(LD2410_device_t *));
      xTaskCreate(ld2410_out_pin_task, "ld2410_out_pin_task", CONFIG_LD2410_OUT_PIN_TASK_STACK_SIZE, NULL, 10, NULL);
   }
   return ESP_OK;
}
esp_err_t ld2410_out_pin_attach_handler(LD2410_device_t *device)
{
   ESP_RETURN_ON_FALSE(device->out_pin != OUT_PIN_NULL && device->out_pin_callback != NULL,
                       ESP_ERR_NOT_ALLOWED, TAG, "Must call ld2410_out_pin_init_handler before attaching it!");
   return gpio_isr_handler_add(device->out_pin, ld2410_isr_handler, (void *)device);
}
#endif

bool ld2410_in_config_mode(LD2410_device_t *device)
{
   return device->isConfig;
}

bool ld2410_in_basic_mode(LD2410_device_t *device)
{
   return !device->isEnhanced;
}

bool ld2410_in_enhanced_mode(LD2410_device_t *device)
{
   return device->isEnhanced;
}

bool ld2410_is_data_valid(LD2410_device_t *device)
{
   return ((esp_timer_get_time() / 1000) - (device->sData).timestamp_ms < device->dataLifespan);
}

uint8_t ld2410_get_status(LD2410_device_t *device)
{
   return (ld2410_is_data_valid(device)) ? (device->sData).status : 0xFF;
}

const char *ld2410_status_string(LD2410_device_t *device)
{
   return LD2410_tStatus[(device->sData).status];
}

bool ld2410_presence_detected_no_validity_check(LD2410_device_t *device)
{
   return ((device->sData).status) && ((device->sData).status < 4); // 1,2,3
}

bool ld2410_presence_detected(LD2410_device_t *device)
{
   return ld2410_is_data_valid(device) && ld2410_presence_detected_no_validity_check(device);
}

bool ld2410_stationary_target_detected(LD2410_device_t *device)
{
   return ld2410_is_data_valid(device) && (((device->sData).status == 2) || ((device->sData).status == 3)); // 2,3
}

unsigned long ld2410_stationary_target_distance(LD2410_device_t *device)
{
   return (device->sData).sTargetDistance;
}

uint8_t ld2410_stationary_target_signal(LD2410_device_t *device)
{
   return (device->sData).sTargetSignal;
}

ValuesArray_t ld2410_get_stationary_signals(LD2410_device_t *device)
{
   return (device->sData).sTargetSignals;
}

bool ld2410_moving_target_detected(LD2410_device_t *device)
{
   return ld2410_is_data_valid(device) && (((device->sData).status == 1) || ((device->sData).status == 3)); // 1,3
}

unsigned long ld2410_moving_target_distance(LD2410_device_t *device)
{
   return (device->sData).mTargetDistance;
}

uint8_t ld2410_moving_target_signal(LD2410_device_t *device)
{
   return (device->sData).mTargetSignal;
}

ValuesArray_t ld2410_get_moving_signals(LD2410_device_t *device)
{
   return (device->sData).mTargetSignals;
}

unsigned long ld2410_detected_distance(LD2410_device_t *device)
{
   return (device->sData).distance;
}

void ld2410_get_MAC(LD2410_device_t *device, uint8_t *return_MAC)
{
   if (strlen(device->MACstr) == 0)
      ld2410_request_MAC(device);
   memcpy(return_MAC, device->MAC, MAC_SIZE);
}

void ld2410_get_MAC_str(LD2410_device_t *device, char *return_str)
{
   if (strlen(device->MACstr) == 0)
      ld2410_request_MAC(device);
   strcpy(return_str, device->MACstr);
}

void ld2410_get_firmware(LD2410_device_t *device, char *return_str)
{
   if (strlen(device->firmware) == 0)
      ld2410_request_firmware(device);
   strcpy(return_str, device->firmware);
}

uint8_t ld2410_get_firmware_major(LD2410_device_t *device)
{
   if (!device->firmwareMajor)
      ld2410_request_firmware(device);
   return device->firmwareMajor;
}

uint8_t ld2410_get_firmware_minor(LD2410_device_t *device)
{
   if (!device->firmwareMajor)
      ld2410_request_firmware(device);
   return device->firmwareMinor;
}

unsigned long ld2410_get_version(LD2410_device_t *device)
{
   if (device->version == 0)
   {
      ld2410_config_mode(device, true);
      ld2410_config_mode(device, false);
   }
   return device->version;
}

SensorData_t ld2410_get_sensor_data(LD2410_device_t *device)
{
   return device->sData;
}

ValuesArray_t ld2410_get_moving_thresholds(LD2410_device_t *device)
{
   if (!device->maxRange)
      ld2410_request_parameters(device);
   return device->movingThresholds;
}

ValuesArray_t ld2410_get_stationary_thresholds(LD2410_device_t *device)
{
   if (!device->maxRange)
      ld2410_request_parameters(device);
   return device->stationaryThresholds;
}

uint8_t ld2410_get_range(LD2410_device_t *device)
{
   if (!device->maxRange)
      ld2410_request_parameters(device);
   return device->maxRange;
}

unsigned long ld2410_get_range_cm(LD2410_device_t *device)
{
   return (ld2410_get_range(device) + 1) * ld2410_get_resolution(device);
}

uint8_t ld2410_get_no_one_window(LD2410_device_t *device)
{
   if (!device->maxRange)
      ld2410_request_parameters(device);
   return device->noOne_window;
}

bool ld2410_config_mode(LD2410_device_t *device, bool enable)
{
   if (enable && !device->isConfig)
      return ld2410_send_command(device, LD2410_Config_Enable);
   if (!enable && device->isConfig)
      return ld2410_send_command(device, LD2410_Config_Disable);
   return false;
}

bool ld2410_enhanced_mode(LD2410_device_t *device, bool enable)
{
   if (device->isConfig)
      return ld2410_send_command(device, ((enable) ? LD2410_Eng_On : LD2410_Eng_Off));
   else
      return ld2410_config_mode(device, true) && ld2410_send_command(device, ((enable) ? LD2410_Eng_On : LD2410_Eng_Off)) && ld2410_config_mode(device, false);
}

bool ld2410_request_aux_config(LD2410_device_t *device)
{
   if (device->isConfig)
      return ld2410_send_command(device, LD2410_Aux_Query);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, LD2410_Aux_Query) && ld2410_config_mode(device, false);
}

bool ld2410_auto_thresholds(LD2410_device_t *device, uint8_t _timeout)
{
   uint8_t cmd[6];
   memcpy(cmd, LD2410_Auto_Begin, 6);
   if (_timeout)
      cmd[4] = _timeout;
   if (device->isConfig)
      return ld2410_send_command(device, cmd);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, cmd) && ld2410_config_mode(device, false);
}

AutoStatus_t ld2410_get_auto_status(LD2410_device_t *device)
{
   bool res = false;
   if (device->isConfig)
      res = ld2410_send_command(device, LD2410_Auto_Query);
   else
      res = ld2410_config_mode(device, true) && ld2410_send_command(device, LD2410_Auto_Query) && ld2410_config_mode(device, false);
   if (res)
      return device->autoStatus;
   return AS_NOT_SET;
}

bool ld2410_request_MAC(LD2410_device_t *device)
{
   if (device->isConfig)
      return ld2410_send_command(device, LD2410_MAC);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, LD2410_MAC) && ld2410_config_mode(device, false);
}

bool ld2410_request_firmware(LD2410_device_t *device)
{
   if (device->isConfig)
      return ld2410_send_command(device, LD2410_firmware);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, LD2410_firmware) && ld2410_config_mode(device, false);
}

bool ld2410_request_resolution(LD2410_device_t *device)
{
   if (device->isConfig)
      return ld2410_send_command(device, LD2410_Res);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, LD2410_Res) && ld2410_config_mode(device, false);
}

bool ld2410_set_resolution(LD2410_device_t *device, bool fine)
{
   if (device->isConfig && ld2410_send_command(device, ((fine) ? LD2410_Res_Fine : LD2410_Res_Coarse)))
      return ld2410_send_command(device, LD2410_Res);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, ((fine) ? LD2410_Res_Fine : LD2410_Res_Coarse)) &&
          ld2410_send_command(device, LD2410_Res) && ld2410_config_mode(device, false);
}

bool ld2410_request_parameters(LD2410_device_t *device)
{
   if (device->isConfig)
      return ld2410_send_command(device, LD2410_Param);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, LD2410_Param) && ld2410_config_mode(device, false);
}

bool ld2410_set_gate_parameters(LD2410_device_t *device, uint8_t gate, uint8_t movingThreshold, uint8_t stationaryThreshold)
{
   if (movingThreshold > 100)
      movingThreshold = 100;
   if (stationaryThreshold > 100)
      stationaryThreshold = 100;
   uint8_t *cmd = LD2410_Gate_Param;
   if (gate > 8)
   {
      cmd[6] = 0xFF;
      cmd[7] = 0xFF;
   }
   else
   {
      cmd[6] = gate;
      cmd[7] = 0;
   }
   cmd[12] = movingThreshold;
   cmd[18] = stationaryThreshold;
   if (device->isConfig && ld2410_send_command(device, cmd))
      return ld2410_send_command(device, LD2410_Param);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, cmd) && ld2410_send_command(device, LD2410_Param) && ld2410_config_mode(device, false);
}

bool ld2410_set_max_gate(LD2410_device_t *device, uint8_t movingGate, uint8_t staticGate, uint8_t noOneWindow)
{
   if (movingGate > 8)
      movingGate = 8;
   if (staticGate > 8)
      staticGate = 8;
   uint8_t *cmd = LD2410_Max_Gate;
   cmd[6] = movingGate;
   cmd[12] = staticGate;
   cmd[18] = noOneWindow;
   if (device->isConfig && ld2410_send_command(device, cmd))
      return ld2410_send_command(device, LD2410_Param);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, cmd) && ld2410_send_command(device, LD2410_Param) && ld2410_config_mode(device, false);
}

/* bool ld2410_set_gate_parameters(
    LD2410_device_t *device,
    ValuesArray_t moving_thresholds,
    ValuesArray_t stationary_thresholds,
    uint8_t noOneWindow)
{
   if (!device->isConfig)
      ld2410_config_mode(device, true);
   bool success = device->isConfig;
   if (success)
   {
      for (uint8_t i = 0; i < 9; i++)
      {
         if (!setGateParameters(i, moving_thresholds.values[i], stationary_thresholds.values[i]))
         {
            success = false;
            break;
         }
         delay(20);
      }
   }
   return success && ld2410_set_max_gate(device, moving_thresholds.N, stationary_thresholds.N, noOneWindow) && ld2410_config_mode(device, false);
} */

bool ld2410_set_no_one_window(LD2410_device_t *device, uint8_t noOneWindow)
{
   if (!device->maxRange)
      ld2410_request_parameters(device);
   if (device->noOne_window == noOneWindow)
      return true;
   return ld2410_set_max_gate(device, (device->movingThresholds).N, (device->stationaryThresholds).N, noOneWindow);
}

bool ld2410_set_max_moving_gate(LD2410_device_t *device, uint8_t movingGate)
{
   if (!device->maxRange)
      ld2410_request_parameters(device);
   if ((device->movingThresholds).N == movingGate)
      return true;
   if (!device->noOne_window)
      device->noOne_window = 5;
   if (movingGate > 8)
      movingGate = 8;
   return ld2410_set_max_gate(device, movingGate, (device->stationaryThresholds).N, device->noOne_window);
}

bool ld2410_set_max_stationary_gate(LD2410_device_t *device, uint8_t stationaryGate)
{
   if (!device->maxRange)
      ld2410_request_parameters(device);
   if ((device->stationaryThresholds).N == stationaryGate)
      return true;
   if (!device->noOne_window)
      device->noOne_window = 5;
   if (stationaryGate > 8)
      stationaryGate = 8;
   return ld2410_set_max_gate(device, (device->movingThresholds).N, stationaryGate, device->noOne_window);
}

uint8_t ld2410_get_max_moving_gate(LD2410_device_t *device)
{
   if (!(device->movingThresholds).N)
      ld2410_request_parameters(device);
   return (device->movingThresholds).N;
}

uint8_t ld2410_get_max_stationary_gate(LD2410_device_t *device)
{
   if (!(device->stationaryThresholds).N)
      ld2410_request_parameters(device);
   return (device->stationaryThresholds).N;
}

bool ld2410_request_reset(LD2410_device_t *device)
{
   if (device->isConfig)
      return ld2410_send_command(device, LD2410_Reset) && ld2410_send_command(device, LD2410_Param) && ld2410_send_command(device, LD2410_Res);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, LD2410_Reset) && ld2410_send_command(device, LD2410_Param) &&
          ld2410_send_command(device, LD2410_Res) && ld2410_config_mode(device, false);
}

bool ld2410_request_reboot(LD2410_device_t *device)
{
   if (device->isConfig)
      return ld2410_send_command(device, LD2410_Reboot);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, LD2410_Reboot);
}

bool ld2410_request_BT_on(LD2410_device_t *device)
{
   if (device->isConfig)
      return ld2410_send_command(device, LD2410_BTon);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, LD2410_BTon) && ld2410_config_mode(device, false);
}

bool ld2410_request_BT_off(LD2410_device_t *device)
{
   if (device->isConfig)
      return ld2410_send_command(device, LD2410_BToff);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, LD2410_BToff) && ld2410_config_mode(device, false);
}

bool ld2410_set_BT_password(LD2410_device_t *device, const char *passwd)
{
   uint8_t cmd[10];
   for (unsigned int i = 0; i < 4; i++)
      cmd[i] = LD2410_BTpasswd[i];

   for (unsigned int i = 0; i < 6; i++)
   {
      if (i < strlen(passwd))
         cmd[4 + i] = passwd[i];
      else
         cmd[4 + i] = ' ';
   }
   if (device->isConfig)
      return ld2410_send_command(device, cmd);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, cmd) && ld2410_config_mode(device, false);
}

bool ld2410_reset_BT_password(LD2410_device_t *device)
{
   if (device->isConfig)
      return ld2410_send_command(device, LD2410_BTpasswd);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, LD2410_BTpasswd) && ld2410_config_mode(device, false);
}

bool ld2410_set_baud(LD2410_device_t *device, uint8_t baud)
{
   if ((baud < 1) || (baud > 8))
      return false;
   uint8_t cmd[6];
   memcpy(cmd, LD2410_Change_Baud, 6);
   cmd[4] = baud;
   if (device->isConfig)
      return ld2410_send_command(device, cmd) && ld2410_request_reboot(device);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, cmd) && ld2410_request_reboot(device);
}

uint8_t ld2410_get_resolution(LD2410_device_t *device)
{
   if (device->fineRes >= 0)
      return ((device->fineRes == 1) ? 20 : 75);
   if (device->isConfig)
   {
      if (ld2410_send_command(device, LD2410_Res))
         return ld2410_get_resolution(device);
   }
   else
   {
      if (ld2410_config_mode(device, true) && ld2410_send_command(device, LD2410_Res) && ld2410_config_mode(device, false))
         return ld2410_get_resolution(device);
   }
   return 0;
}

uint8_t ld2410_get_light_level(LD2410_device_t *device)
{
   return device->lightLevel;
}

LightControl_t ld2410_get_light_control(LD2410_device_t *device)
{
   if (device->lightControl == LC_NOT_SET)
      ld2410_request_aux_config(device);
   return device->lightControl;
}

uint8_t ld2410_get_light_threshold(LD2410_device_t *device)
{
   if (device->lightControl == LC_NOT_SET)
      ld2410_request_aux_config(device);
   return device->lightThreshold;
}

OutputControl_t ld2410_get_output_control(LD2410_device_t *device)
{
   if (device->outputControl == OC_NOT_SET)
      ld2410_request_aux_config(device);
   return device->outputControl;
}

bool ld2410_set_aux_control(LD2410_device_t *device,
                            LightControl_t light_control,
                            uint8_t light_threshold,
                            OutputControl_t output_control)
{
   uint8_t cmd[8];
   memcpy(cmd, LD2410_Aux_Config, 8);
   cmd[4] = light_control;
   cmd[5] = light_threshold;
   cmd[6] = output_control;
   if (device->isConfig)
      return ld2410_send_command(device, cmd) && ld2410_request_aux_config(device);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, cmd) && ld2410_request_aux_config(device) && ld2410_config_mode(device, false);
}

bool ld2410_reset_aux_control(LD2410_device_t *device)
{
   if (device->isConfig)
      return ld2410_send_command(device, LD2410_Aux_Config) && ld2410_request_aux_config(device);
   return ld2410_config_mode(device, true) && ld2410_send_command(device, LD2410_Aux_Config) && ld2410_request_aux_config(device) && ld2410_config_mode(device, false);
}

uint8_t ld2410_get_out_level(LD2410_device_t *device)
{
   return device->outLevel;
}

#pragma region ValuesArrayUtils
void va_copy(ValuesArray_t *destArray, ValuesArray_t *srcArray)
{
   if (destArray != srcArray)
   {
      destArray->N = srcArray->N;
      for (uint8_t i = 0; i <= srcArray->N; i++)
         destArray->values[i] = srcArray->values[i];
   }
}

void va_setN(ValuesArray_t *array, uint8_t n)
{
   array->N = (n <= 8) ? n : 8;
}
#pragma endregion