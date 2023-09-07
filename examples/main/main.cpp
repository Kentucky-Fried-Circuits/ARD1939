/*
 * ARD1939 example using ESP-IDF TWAI library
 *
 * SPDX-FileCopyrightText: 2023 Solar Stik, Inc. All rights reserved.
 *
 * SPDX-License-Identifier: SEE LICENSE IN LICENSE.md
 */
/* Includes */
// standard C/C++ libraries
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "time.h"
#include <sys/unistd.h>
#include <sys/stat.h>

// freertos libraries
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "freertos/queue.h"
// #include "freertos/semphr.h"

// ESP-IDF libraries
// #include "esp_chip_info.h"
// #include "esp_flash.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"

// ESP-IDF drivers and components
#include "driver/gpio.h"
#include "driver/twai.h"
#include "driver/i2c.h"
#include "driver/uart.h"

// 3rd party drivers
#include <sx1509.hpp>
#include "ARD1939.h"

/* definitions */
// Board to use
// #define REMOTE_MONITOR_PCB_VERSION 'A'
#define REMOTE_MONITOR_PCB_VERSION 'B'
// tests to run
#define TEST_SX1509
// #define TEST_DS3232
#define TEST_LED
// #define TEST_RS485 // FIXME there's some SX1509 contention going on with RS485 and CAN
// #define TEST_SDCARD
// #define TEST_ETHERNET
#define TEST_CAN
// #define LOG_MODE (ESP_LOG_DEBUG) // for development
#define LOG_MODE (ESP_LOG_DEBUG) // for deployment

// test dependencies
#ifndef TEST_SX1509
#undef TEST_DS3232
#undef TEST_LED
#undef TEST_RS485
#undef TEST_SDCARD // only for REV A, where SX1509 is required to pull up an unused address line
#endif

// ESP32 GPIO pins
#define ESP32_DI (GPIO_NUM_4)
#define ESP32_CAN_TX (GPIO_NUM_5)
#define ESP32_NRESET (GPIO_NUM_17) // general-purpose reset signal to peripherals, mainly to reset I2C devices
#define ESP32_SCL (GPIO_NUM_16)    /*!< GPIO number used for I2C controller clock */
#define ESP32_SDA (GPIO_NUM_33)    // 2 for REV -, 33 for REV A /*!< GPIO number used for I2C controller data  */
#define ESP32_IO_NINT (GPIO_NUM_34)
#define ESP32_CAN_RX (GPIO_NUM_35)
#define ESP32_RO (GPIO_NUM_36)

// SX1509 Pins:
const uint8_t SX1509_LED_RED = 0;
const uint8_t SX1509_LED_GREEN = 1;
const uint8_t SX1509_NINT_SQW = 3;
#if REMOTE_MONITOR_PCB_VERSION == 'A'
const uint8_t SX1509_SD_CS = 4;
#endif
const uint8_t SX1509_STBY = 5;
const uint8_t SX1509_RELAY1 = 6;
const uint8_t SX1509_RELAY2 = 7;
const uint8_t SX1509_SW1_1 = 8; // 8 is allocated to GEN_CONTROL_ON(?)
const uint8_t SX1509_SW1_2 = 9; // 9 is allocated to GEN_CONTROL_AUTO(?)
const uint8_t SX1509_RE = 10;
const uint8_t SX1509_DE = 11;

// I2C
#define I2C_CONTROLLER_NUM 0            /*!< I2C controller i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_CONTROLLER_FREQ_HZ 400000   /*!< I2C controller clock frequency */
#define I2C_CONTROLLER_TX_BUF_DISABLE 0 /*!< I2C controller doesn't need buffer */
#define I2C_CONTROLLER_RX_BUF_DISABLE 0 /*!< I2C controller doesn't need buffer */
#define I2C_CONTROLLER_TIMEOUT_MS 1000
#if REMOTE_MONITOR_PCB_VERSION == 'A'
#define I2C_SX1509_ADDR (0x3E)
#else
#define I2C_SX1509_ADDR (0x3F)
#endif
// #define I2C_DISPLAY_LCD_ADDR (0x3E)
// #define I2C_DISPLAY_BACKLIGHT_ADDR (0x60)
// #define I2C_DISPLAY_SX1509_ADDR (0x71) // jumper configurable

// ADM2582 RS485/RS422 FIXME change ECHO_ to RS_
#define RS_UART_PORT_NUM (1)
#define RS_UART_BAUD_RATE (19200)
#define RS_TASK_STACK_SIZE (2048)
#define RS_BUF_SIZE (1024)

// CAN FIXME all the CAN code copied from twai_network_example and is untested.
#define PING_PERIOD_MS 100
#define NO_OF_DATA_MSGS 2
#define NO_OF_ITERS 1
#define ITER_DELAY_MS 1000
#define RX_TASK_PRIO 8
#define TX_TASK_PRIO 9
#define CTRL_TSK_PRIO 10

// Sample CAN IDs
#define ID_MASTER_STOP_CMD 0x0A0
#define ID_MASTER_START_CMD 0x0A1
#define ID_MASTER_PING 0x0A2
#define ID_SLAVE_STOP_RESP 0x0B0
#define ID_SLAVE_DATA 0x0B1
#define ID_SLAVE_PING_RESP 0x0B2

static twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
// static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(ESP32_CAN_TX, ESP32_CAN_RX, TWAI_MODE_NORMAL);

static const twai_message_t ping_message = {{.ss = 0x01}, .identifier = ID_MASTER_PING, .data_length_code = 0, .data = {0, 0, 0, 0, 0, 0, 0, 0}};
static const twai_message_t start_message = {{.ss = 0x00}, .identifier = ID_MASTER_START_CMD, .data_length_code = 0, .data = {0, 0, 0, 0, 0, 0, 0, 0}};
static const twai_message_t stop_message = {{.ss = 0x00}, .identifier = ID_MASTER_STOP_CMD, .data_length_code = 0, .data = {0, 0, 0, 0, 0, 0, 0, 0}};

// ** lazy globals
esp_err_t can_err = ESP_OK; // a lazy global flag for a local purpose

static SemaphoreHandle_t done_sem;

typedef enum
{
    TX_SEND_PINGS,
    TX_SEND_START_CMD,
    TX_SEND_STOP_CMD,
    TX_TASK_EXIT,
} tx_task_action_t;

typedef enum
{
    RX_RECEIVE_PING_RESP,
    RX_RECEIVE_DATA,
    RX_RECEIVE_STOP_RESP,
    RX_TASK_EXIT,
} rx_task_action_t;

static const char *TAG = "example";
static const char *TAG_CAN = "example CAN";

// macros
#define DELAY(X) vTaskDelay(pdMS_TO_TICKS(X)) // emulate Arduino delay()

/* forward declarations*/
static void twai_receive_task(void *arg);
// static void twai_transmit_task(void *arg);

/* instantiations */
SX1509 io;
ARD1939 j1939(&g_config, &t_config);

/**
 * @brief emulate Arduino pinMode()
 *
 * @param pin
 * @param pin_mode
 * @return esp_err_t
 *
 */
esp_err_t pinMode(gpio_num_t pin, unsigned long pin_mode)
{
    gpio_config_t io_config;
    io_config.pin_bit_mask = (1ULL << pin);
    if (pin_mode == OUTPUT)
    {
        io_config.mode = GPIO_MODE_OUTPUT;
        io_config.pull_up_en = GPIO_PULLUP_DISABLE;
        io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_config.intr_type = GPIO_INTR_DISABLE;
    }
    else
    {
        ESP_LOGE(TAG, "output mode %ld not yet supported", pin_mode);
    }
    return gpio_config(&io_config);
}

/**
 * @brief block until a key is pressed on stdin
 *
 */
int press_any_key()
{
    int mychar = -1;
    while (mychar == -1)
    {
        mychar = getchar();
        vTaskDelay(1);
    }
    return mychar;
}

void restart()
{
    printf("Restarting in ");
    for (int i = 10; i >= 0; i--)
    {
        printf("%d...", i);
        fflush(stdout);
        DELAY(1000);
    }
    printf("restarting now.\n");
    fflush(stdout);
    esp_restart();
}

/**
 * @brief Set the ~reset pin on the local SX1509 to reset, enable or disable it
 *
 * @param level 0 - disable, 1 - enable, toggle 1, 0, 1 to reset
 * @return esp_err_t
 */
esp_err_t set_nreset(uint32_t level)
{
    // pinModeOutput(ESP32_NRESET);
    ESP_RETURN_ON_ERROR(pinMode(ESP32_NRESET, OUTPUT), TAG, "Unable to set pin mode for ~Reset");
    // disable/enable SX1509
    ESP_RETURN_ON_ERROR(gpio_set_level(ESP32_NRESET, level), TAG, "Unable to set ~Reset pin to %d", level);
    return ESP_OK;
}

esp_err_t test_can()
{
    can_err = ESP_OK;
    ESP_LOGD(TAG_CAN, "started");
    ESP_LOGI(TAG_CAN, "Connect PCAN-USB pin 2 to 05-1000193 J4-2 (CAN_L) and PCAN-USB pin 7 to J4-1 (CAN_H)");
    ESP_LOGI(TAG_CAN, "Plug PCAN-USB into computer, start PCAN-View software.");
    ESP_LOGI(TAG_CAN, "Create a Transmit Message with: Extended Frame unchecked, ID 0B1, and Cycle Time 1000.");

    // enable can xceiver
    ESP_ERROR_CHECK(io.pinMode(SX1509_STBY, OUTPUT));
    io.digitalWrite(SX1509_STBY, LOW); // enable CAN chip in high-speed mode

    // Create tasks, queues, and semaphores
    done_sem = xSemaphoreCreateBinary();
    ESP_LOGD(TAG_CAN, "claiming done_sem");
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    // xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    ESP_LOGD(TAG_CAN, "task(s) created");

    j1939.Init(1000 / CONFIG_FREERTOS_HZ > 0 ? 1000 / CONFIG_FREERTOS_HZ : 1);
    j1939.SetPreferredAddress(SA_PREFERRED);
    j1939.SetAddressRange(ADDRESSRANGEBOTTOM, ADDRESSRANGETOP);
    j1939.SetNAME(NAME_IDENTITY_NUMBER,
                    NAME_MANUFACTURER_CODE,
                    NAME_FUNCTION_INSTANCE,
                    NAME_ECU_INSTANCE,
                    NAME_FUNCTION,
                    NAME_VEHICLE_SYSTEM,
                    NAME_VEHICLE_SYSTEM_INSTANCE,
                    NAME_INDUSTRY_GROUP,
                    NAME_ARBITRARY_ADDRESS_CAPABLE);
    // j1939.SetMessageFilter(PGN_ECU_IDENTIFICATION_INFORMATION); // we OPT IN to RTS CTS messages
    // All Tranport PGNs must be explicitly filtered *in*
 
    ESP_LOGD(TAG_CAN, "j1939.Init() complete");
    ESP_LOGI(TAG_CAN, "Start sending messages. They will be echoed below. A to abort");

    xSemaphoreGive(done_sem);                // okay receive task can start
    xSemaphoreTake(done_sem, portMAX_DELAY); // Wait for completion
    // Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_stop());
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGD(TAG_CAN, "Driver uninstalled");
    // Cleanup
    j1939.Terminate();
    vSemaphoreDelete(done_sem);
    if (can_err == ESP_OK)
    {
        ESP_LOGW(TAG_CAN, "If the messages you sent in PCAN-View were displayed, then press P. Otherwise press any other key");
        int mychar = press_any_key();
        if (mychar != 'P' && mychar != 'p')
        {
            can_err = ESP_ERR_INVALID_RESPONSE;
        }
    }

    ESP_LOGD(TAG_CAN, "completed");
    return can_err;
}

esp_err_t test_hello_world()
{
    ESP_LOGI(TAG, "****************************************************************************************************************************");
    ESP_LOGI(TAG,
             "*** Test started                                                                                                         ***");
    ESP_LOGI(TAG, "****************************************************************************************************************************");
    return ESP_OK;
}

esp_err_t test_i2c_scan()
{
    esp_err_t ret;
    bool sx1509_found = false;
    ESP_LOGD(TAG, "I2C Scan: start");
    for (uint8_t i = 1; i < 127; i++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            if (i == 0x3f)
            { // SX1509 off board
                ESP_LOGD(TAG, "I2C Scan: found something at SX1509 address 0x%2x", i);
                sx1509_found = true;
            }
            else
            {
                ESP_LOGD(TAG, "I2C Scan: found something at 0x%2x", i);
            }
        }
    }
    ESP_LOGD(TAG, "I2C Scan: complete");
    if (!sx1509_found)
    {
        ESP_LOGE(TAG, "FAILED! I2C Scan: onboard SX1509 I/O Expander not found at 0x3F");
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

esp_err_t test_led()
{
    esp_err_t ret;

    ESP_LOGD(TAG, "test_led(): start");

    ret = io.pinMode(SX1509_LED_GREEN, OUTPUT); // Set LED pin to OUTPUT
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "TEST LED: io.pinMode(%i, OUTPUT) returned %i", SX1509_LED_GREEN, ret);
    ret = io.pinMode(SX1509_LED_RED, OUTPUT); // Set LED pin to OUTPUT
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "TEST LED: io.pinMode(%i, OUTPUT) returned %i", SX1509_LED_RED, ret);
    // Blink the green LED pin
    // Blink the red LED pin
    // Blink time resolution is 5 bit
    // The timing parameters are in milliseconds, but they
    // aren't 100% exact. The library will estimate to try to
    // get them as close as possible. Play with the clock
    // divider to maybe get more accurate timing.

    ESP_LOGD(TAG, "test_led(): blink green");
    ret = io.blink(SX1509_LED_GREEN, 100, 900);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "io.blink(%i, 100, 900) returned %i", SX1509_LED_GREEN, ret);
        return ret;
    }
    ESP_LOGW(TAG, "TEST LED: Verify green LED is blinking");
    ret = io.blink(SX1509_LED_RED, 100, 1900);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "io.blink(%i, 100, 1900) returned %i", SX1509_LED_RED, ret);
        return ret;
    }
    ESP_LOGW(TAG, "TEST LED: Verify red LED is blinking");
    // ESP_LOGW(TAG, "Press any key to continue");
    // press_any_key();
    ESP_LOGD(TAG, "test_led(): complete");
    return ret;
}

/**
 * @brief verify we are communicating with the onboard sx1509
 *
 * @return esp_err_t result of io.begin() if it fails, else result of io.clock()
 */
esp_err_t test_sx1509()
{
    esp_err_t ret;
    ESP_LOGD(TAG, "test_sx1509(): start");

    i2c_config_t cfg;
    cfg.sda_io_num = ESP32_SDA;
    cfg.scl_io_num = ESP32_SCL;
    cfg.master.clk_speed = 400000;
    cfg.clk_flags = 0;

    ret = io.begin(I2C_NUM_0, &cfg, I2C_SX1509_ADDR, ESP32_NRESET, ESP32_IO_NINT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "io.begin(I2C_SX1509_ADDR) returned %i", ret); // TODO set BIT
        return ret;
    }
    ret = io.clock(INTERNAL_CLOCK_2MHZ, 4);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "io.clock(INTERNAL_CLOCK_2MHZ, 4) returned %i", ret); // TODO set BIT
        return ret;
    }
    ESP_LOGD(TAG, "test_sx1509(): complete");
    return ret;
}

/**
 * @brief task to receive CAN messages
 *
 * @param arg
 */
static void twai_receive_task(void *arg)
{
    struct
    {
        byte nMsgId;
        long lPGN;
        byte pMsg[8];
        int nMsgLen;
        byte nDestAddr;
        byte nSrcAddr;
        byte nPriority;
    } rxMsg;
    memset(&rxMsg, 0, sizeof(rxMsg));

    const char* j1939Statuses[4] = {"INIT", "IN PROGRESS", "NORMAL", "FAILED"};
    byte j1939Status, oldJ1939Status;
    oldJ1939Status = ADDRESSCLAIM_INIT;
    char myChar;
    if( xSemaphoreTake(done_sem, 1000) != pdTRUE) {
        ESP_LOGE(TAG_CAN, "Could not take done_sem!");
    };
    while (1)
    {
        // Listen for messages. Check nMsgId for result
        j1939Status = j1939.Operate(&rxMsg.nMsgId, &rxMsg.lPGN, rxMsg.pMsg, &rxMsg.nMsgLen, &rxMsg.nDestAddr, &rxMsg.nSrcAddr, &rxMsg.nPriority);
        if (j1939Status != oldJ1939Status)
        {
            ESP_LOGD(TAG_CAN, "rev task():new J1939Status:%s (%d)", j1939Statuses[j1939Status], j1939Status);
            oldJ1939Status = j1939Status;
        }
        if (rxMsg.nMsgId != J1939_MSG_NONE)
            ESP_LOGD(TAG_CAN, "recv task(): j1939Status:%s (%d) ID:%d", j1939Statuses[j1939Status], j1939Status, rxMsg.nMsgId);
        if (rxMsg.nMsgId == J1939_MSG_APP)
        {
            ESP_LOGD(TAG_CAN, "received MsgId:%d PGN:0x%lx len:%d msg[0:1]:0x%x 0x%x dest:0x%x src:0x%x pri:0x%x",
                     rxMsg.nMsgId, rxMsg.lPGN, rxMsg.pMsg[0], rxMsg.pMsg[1], rxMsg.nMsgLen, rxMsg.nDestAddr, rxMsg.nSrcAddr, rxMsg.nPriority);
            // myChar = getchar();
            // if (myChar == 'A') // abort test
            // {
            //     xSemaphoreGive(done_sem);
            //     break;
            // }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            if (rxMsg.nMsgId == J1939_MSG_NONE) {
                printf("N");
            } else {
                printf(".");
            }
        }
    }
    vTaskDelete(NULL);
}

// TODO needs rewritten
static void twai_transmit_task(void *arg)
{
    // // ping_message.ss = 1; // FIXME can't figure out how to initialize this flag in the definition
    // while (1)
    // {
    //     tx_task_action_t action;
    //     xQueueReceive(tx_task_queue, &action, portMAX_DELAY);

    //     if (action == TX_SEND_PINGS)
    //     {
    //         // Repeatedly transmit pings
    //         ESP_LOGD(TAG_CAN, "Transmitting ping");

    //         xSemaphoreTake(stop_ping_sem, portMAX_DELAY); // TEST
    //         ESP_LOGD(TAG_CAN, "got stop_ping_sem");
    //         for (int i = 0; i < 10; i++)
    //         {
    //             twai_transmit(&ping_message, portMAX_DELAY);
    //             printf(".");
    //             fflush(stdout);
    //             DELAY(PING_PERIOD_MS);
    //         }
    //         printf("\n");
    //     }
    //     else if (action == TX_SEND_START_CMD)
    //     {
    //         // Transmit start command to slave
    //         twai_transmit(&start_message, portMAX_DELAY);
    //         ESP_LOGD(TAG_CAN, "Transmitted start command");
    //     }
    //     else if (action == TX_SEND_STOP_CMD)
    //     {
    //         // Transmit stop command to slave
    //         twai_transmit(&stop_message, portMAX_DELAY);
    //         ESP_LOGD(TAG_CAN, "Transmitted stop command");
    //     }
    //     else if (action == TX_TASK_EXIT)
    //     {
    //         break;
    //     }
    //     else
    //     {
    //         ESP_LOGE(TAG_CAN, "unexpected action:%i", action);
    //         break;
    //     }
    // }
    // ESP_LOGD(TAG_CAN, "transmit task complete");
    // vTaskDelete(NULL);
}

extern "C" void app_main(void)
{

    // esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("*", LOG_MODE);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    esp_log_level_set("gpio", ESP_LOG_WARN);
    esp_log_level_set("intr_alloc", ESP_LOG_INFO);
    esp_log_level_set("sx1509", ESP_LOG_DEBUG);
    esp_log_level_set("i2cdev", ESP_LOG_DEBUG);
    if (LOG_MODE < ESP_LOG_DEBUG)
    {
        esp_log_level_set("system_api", ESP_LOG_WARN);
    }
    esp_log_level_set(TAG, LOG_MODE); // set *TAG to debug. This works, but I'd rather use the LOG_LOCAL_LEVEL macro. Also didn't work to put this in main() TEST
    bool failed = false;

    test_hello_world();

    // prep flash for storing intermediate test states

    // I2C-dependent tests
    ESP_ERROR_CHECK(set_nreset(1));

    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(test_sx1509());
    ESP_LOGI(TAG, "SX1509 Test passed");
    ESP_ERROR_CHECK(test_led());
    ESP_LOGI(TAG, "LED Test passed");
#ifdef TEST_CAN
    if (test_can() == ESP_OK)
    {
        ESP_LOGI(TAG, "CAN Test passed");
    }
    else
    {
        failed = true;
        ESP_LOGE(TAG, "CAN Test failed");
    }
#endif
    if (failed)
    {
        ESP_LOGE(TAG, "*** TEST FAILED ***");
    }
    else
    {
        ESP_LOGI(TAG, "*** All tests passed ***");
    }
    ESP_ERROR_CHECK(i2cdev_done());
}
