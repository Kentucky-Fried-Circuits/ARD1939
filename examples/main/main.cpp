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

// static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(ESP32_CAN_TX, ESP32_CAN_RX, TWAI_MODE_NORMAL);

static const twai_message_t ping_message = {{.ss = 0x01}, .identifier = ID_MASTER_PING, .data_length_code = 0, .data = {0, 0, 0, 0, 0, 0, 0, 0}};
static const twai_message_t start_message = {{.ss = 0x00}, .identifier = ID_MASTER_START_CMD, .data_length_code = 0, .data = {0, 0, 0, 0, 0, 0, 0, 0}};
static const twai_message_t stop_message = {{.ss = 0x00}, .identifier = ID_MASTER_STOP_CMD, .data_length_code = 0, .data = {0, 0, 0, 0, 0, 0, 0, 0}};

// ** lazy globals
esp_err_t can_err = ESP_OK; // a lazy global flag for a local purpose

static QueueHandle_t tx_task_queue;
static QueueHandle_t rx_task_queue;
static SemaphoreHandle_t stop_ping_sem;
static SemaphoreHandle_t ctrl_task_sem;
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

// sd card
#define MOUNT_POINT "/sdcard"

static const char *TAG = "93-0000220";
static const char *TAG_CAN = "93-0000220 CAN";
// static const char *TAG_UI = "93-0000220 UI";

// macros
#define DELAY(X) vTaskDelay(pdMS_TO_TICKS(X)) // emulate Arduino delay()

/* forward declarations*/
static void twai_receive_task(void *arg);
static void twai_transmit_task(void *arg);
static void twai_control_task(void *arg);

/* instantiations */
SX1509 io;

// /**
//  * @brief i2c controller initialization
//  */
// static esp_err_t i2c_begin(void)
// {
//     int i2c_controller_port = I2C_CONTROLLER_NUM;

//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = ESP32_SDA,
//         .scl_io_num = ESP32_SCL,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master = {
//             .clk_speed = I2C_CONTROLLER_FREQ_HZ},
//         .clk_flags = 0};

//     i2c_param_config(i2c_controller_port, &conf);

//     return i2c_driver_install(i2c_controller_port, conf.mode, I2C_CONTROLLER_RX_BUF_DISABLE, I2C_CONTROLLER_TX_BUF_DISABLE, 0);
// }

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

// esp_err_t pinModeOutput(gpio_num_t pin)
// {
//     gpio_config_t io_config;
//     io_config.pin_bit_mask = (1ULL << pin);
//     io_config.mode = GPIO_MODE_OUTPUT;
//     io_config.pull_up_en = GPIO_PULLUP_DISABLE;
//     io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
//     io_config.intr_type = GPIO_INTR_DISABLE;
//     return gpio_config(&io_config);
// }

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
    // io.digitalWrite(SX1509_STBY, HIGH); // low-power (off) mode
    io.digitalWrite(SX1509_STBY, LOW); // enable CAN chip in high-speed mode

    // Create tasks, queues, and semaphores
    rx_task_queue = xQueueCreate(1, sizeof(rx_task_action_t));
    tx_task_queue = xQueueCreate(1, sizeof(tx_task_action_t));
    ctrl_task_sem = xSemaphoreCreateBinary();
    stop_ping_sem = xSemaphoreCreateBinary();
    done_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_control_task, "TWAI_ctrl", 4096, NULL, CTRL_TSK_PRIO, NULL, tskNO_AFFINITY);

    // Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGD(TAG_CAN, "Driver installed");

    xSemaphoreGive(ctrl_task_sem);           // Start control task
    xSemaphoreTake(done_sem, portMAX_DELAY); // Wait for completion
    // Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGD(TAG_CAN, "Driver uninstalled");
    // Cleanup
    vQueueDelete(rx_task_queue);
    vQueueDelete(tx_task_queue);
    vSemaphoreDelete(ctrl_task_sem);
    vSemaphoreDelete(stop_ping_sem);
    vSemaphoreDelete(done_sem);
    if (can_err == ESP_OK)
    {
        ESP_LOGW(TAG_CAN, "If there is a message in PCAN-View with ID 01Ah then press P. Otherwise press any other key");
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

// FIXME this may need reworked to ESP V4
#ifdef TEST_ETHERNET
esp_err_t test_ethernet()
{
    ESP_LOGD(TAG_ETHERNET, "started");
    eth_err = ESP_OK;
    eth_done_sem = xSemaphoreCreateBinary();

    // Create new default instance of esp-netif for Ethernet
    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&cfg);

    // Init MAC and PHY configs to default
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    phy_config.phy_addr = 0;           // CONFIG_EXAMPLE_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = -1;    // CONFIG_EXAMPLE_ETH_PHY_RST_GPIO;
    mac_config.smi_mdc_gpio_num = 23;  // CONFIG_EXAMPLE_ETH_MDC_GPIO;
    mac_config.smi_mdio_gpio_num = 18; // CONFIG_EXAMPLE_ETH_MDIO_GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&mac_config);

    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));
    /* attach Ethernet driver to TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &eth_ip_event_handler, NULL));

    /* start Ethernet driver state machine */
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    // ** Now block until we get an IP address
    ESP_LOGW(TAG_ETHERNET, "Waiting for IP address. Press A to abort");
    int mychar;
    while (xSemaphoreTake(eth_done_sem, 0) != pdTRUE)
    {
        mychar = getchar();
        if (mychar == 'A' || mychar == 'a')
        {
            eth_err = ESP_FAIL;
            break;
        }
        vTaskDelay(1);
    }
    ESP_LOGD(TAG_ETHERNET, "completed");
    // TODO should we be cleaning up after ourselves?
    return eth_err;
}
#endif

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
    ESP_LOGW(TAG, "Press any key to continue");
    press_any_key();
    ESP_LOGD(TAG, "test_led(): complete");
    return ret;
}

/**
 * @brief test RS485 hardware by echoing to an external terminal
 *
 */
esp_err_t test_rs485()
{
#ifdef TEST_RS485
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = RS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;
    ESP_LOGI(TAG_RS485, "UART %i (%i,%i,%c)", RS_UART_PORT_NUM, uart_config.baud_rate, uart_config.data_bits + 5, (uart_config.parity == 0) ? 'N' : 'Y');

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(RS_UART_PORT_NUM, RS_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(RS_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RS_UART_PORT_NUM, ESP32_DI, ESP32_RO, -1, -1));
    // ESP_ERROR_CHECK(uart_set_mode(RS_UART_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX)); // TEST

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(RS_BUF_SIZE);
    int written = 0;
    time_t now, last_time;
    last_time = time(&now);
    char beat[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    int beater = 0;
    int beats = 8;
    // set up sx1509 for DE and RE pins
    // ESP_ERROR_CHECK(io.begin(I2C_SX1509_ADDR, ESP32_NRESET, ESP32_IO_NINT));
    // ESP_ERROR_CHECK(io.clock(INTERNAL_CLOCK_2MHZ, 4));
    ESP_ERROR_CHECK(io.pinMode(SX1509_DE, OUTPUT));  // active high
    ESP_ERROR_CHECK(io.pinMode(SX1509_RE, OUTPUT));  // active low
    ESP_ERROR_CHECK(io.setupBlink(SX1509_DE, 0, 0)); // disable blink
    ESP_ERROR_CHECK(io.setupBlink(SX1509_RE, 0, 0)); // disable blink
    // For RS485 compatibility, hold DE low / RE low unless we're transmitting
    ESP_ERROR_CHECK(io.writePin(SX1509_DE, HIGH)); // TEST enABLE transmit
    ESP_ERROR_CHECK(io.writePin(SX1509_RE, LOW));  // enable receive
    // Set RS485 half duplex mode

    ESP_ERROR_CHECK(uart_set_mode(RS_UART_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX));
    // uart_context_t uart_context;
    // p_uart_obj[RS_UART_PORT_NUM]->rs485_conf.rx_busy_tx_en = 0; // don't send while receiving => collision avoidance
    // UART1.rs485_conf.tx_rx_en = 1;      // loopback (1), so collision detection works
    // ESP_ERROR_CHECK(uart_set_rx_timeout(RS_UART_PORT_NUM, 100));
    //        ESP_ERROR_CHECK(io.writePin(SX1509_DE, LOW)); // disable transmit
    ESP_ERROR_CHECK(io.writePin(SX1509_DE, HIGH)); // e&nable transmit TEST

    ESP_LOGW(TAG, "Connect Cable xxx from right (outer) RJ12 port to DTECH USB adapter and DTECH USB adapter to Computer.");
    ESP_LOGW(TAG, "Start Putty, select Dtech's COM port, set to 9600 baud, 8 bits, no parity, no flow control.");
    ESP_LOGW(TAG, "Press any key when ready");
    press_any_key();
    ESP_LOGW(TAG, "You should see the numbers 0 to 9 appear in the Putty terminal.");
    ESP_LOGW(TAG, "Type in the Putty terminal window. Anything you type should also appear in the terminal. ");
    ESP_LOGW(TAG, "In this window, press 'P' if the numbers appear and your typing is displayed. Otherwise press any key.");
    int mychar = -1;
    vTaskDelay(pdMS_TO_TICKS(1000)); // time to get finger off key TEST

    while (mychar <= 0)
    {
        // Read data from the UART.
        int len = uart_read_bytes(RS_UART_PORT_NUM, data, (RS_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        // Write data back to the UART
#ifdef MONITOR_ONLY
        ESP_LOGI(TAG_RS485, "recv:%x %x", data[0], data[1]);
#else
        ESP_ERROR_CHECK(io.writePin(SX1509_RE, HIGH)); // disable receive
        // ESP_ERROR_CHECK(io.writePin(SX1509_DE, HIGH)); // TEST enable transmit
        DELAY(5);
        written = uart_write_bytes(RS_UART_PORT_NUM, (const char *)data, len);
        // ESP_ERROR_CHECK(io.writePin(SX1509_DE, LOW)); // TEST disable transmit
        // ESP_ERROR_CHECK(io.writePin(SX1509_RE, LOW)); // enable receive
        if (len)
        {
            data[len] = '\0';
            ESP_LOGI(TAG_RS485, "Recv str: %s, bytes written: %i", (char *)data, written);
        }
        if (last_time != time(&now))
        { // heartbeat once per second
            // ESP_ERROR_CHECK(io.writePin(SX1509_RE, HIGH)); // disable receive
            // ESP_ERROR_CHECK(io.writePin(SX1509_DE, HIGH)); // enable transmit TEST
            // DELAY(15);
            written = uart_write_bytes(RS_UART_PORT_NUM, &beat[beater], beats);
            // ESP_ERROR_CHECK(io.writePin(SX1509_DE, LOW)); // disable transmit
            // ESP_ERROR_CHECK(io.writePin(SX1509_RE, LOW)); // enable receive TEST
            ESP_LOGD(TAG_RS485, "heartbeat first char:%c bytes written:%d", beat[beater], written);
            beater = (beater + beats) % (27 - beats);
            last_time = time(&now);
        }
#endif // MONITOR_ONLY
        bool collided = false;
        if (uart_get_collision_flag(RS_UART_PORT_NUM, &collided))
        {
            ESP_LOGI(TAG_RS485, "colision");
        }
        mychar = getchar();
        // ESP_LOGD(TAG, "mychar: %d", mychar);
        ESP_ERROR_CHECK(io.writePin(SX1509_RE, LOW)); // enable receive
        DELAY(5);
    }
    if (mychar != char('P') && mychar != char('p'))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }
#endif // TEST_RS485
    return ESP_OK;
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
    while (1)
    {
        rx_task_action_t action;
        xQueueReceive(rx_task_queue, &action, portMAX_DELAY);

        if (action == RX_RECEIVE_PING_RESP)
        {
            // Listen for ping response from slave
            while (1)
            {
                twai_message_t rx_msg;
                twai_receive(&rx_msg, portMAX_DELAY);
                if (rx_msg.identifier == ID_SLAVE_PING_RESP)
                {
                    ESP_LOGD(TAG_CAN, "ID_SLAVE_PING_RESP %d received", rx_msg.identifier);
                    xSemaphoreGive(stop_ping_sem);
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        }
        else if (action == RX_RECEIVE_DATA)
        {
            // Receive data messages from slave
            uint32_t data_msgs_rec = 0;
            int myChar;
            ESP_LOGW(TAG_CAN, "Press A to abort CAN test");
            can_err = ESP_OK;
            while (data_msgs_rec < NO_OF_DATA_MSGS)
            {
                twai_message_t rx_msg;
                // twai_receive(&rx_msg, portMAX_DELAY);
                twai_receive(&rx_msg, pdMS_TO_TICKS(10));
                if (rx_msg.identifier == ID_SLAVE_DATA)
                {
                    ESP_LOGD(TAG_CAN, "ID_SLAVE_DATA %d received", rx_msg.identifier);
                    uint32_t data = 0;
                    for (int i = 0; i < rx_msg.data_length_code; i++)
                    {
                        data |= (rx_msg.data[i] << (i * 8));
                    }
                    ESP_LOGD(TAG_CAN, "Received data value %" PRIu32, data);
                    data_msgs_rec++;
                }
                myChar = getchar();
                if (myChar == char('A') || myChar == char('a')) // abort test
                {
                    can_err = ESP_FAIL;
                    break;
                }
            }
            xSemaphoreGive(ctrl_task_sem);
        }
        else if (action == RX_RECEIVE_STOP_RESP)
        {
            // // Listen for stop response from slave
            // while (1)
            // {
            //     twai_message_t rx_msg;
            //     twai_receive(&rx_msg, portMAX_DELAY);
            //     if (rx_msg.identifier == ID_SLAVE_STOP_RESP)
            //     {
            xSemaphoreGive(ctrl_task_sem);
            //         break;
            //     }
            // }
        }
        else if (action == RX_TASK_EXIT)
        {
            break;
        }
    }
    vTaskDelete(NULL);
}

static void twai_transmit_task(void *arg)
{
    // ping_message.ss = 1; // FIXME can't figure out how to initialize this flag in the definition
    while (1)
    {
        tx_task_action_t action;
        xQueueReceive(tx_task_queue, &action, portMAX_DELAY);

        if (action == TX_SEND_PINGS)
        {
            // Repeatedly transmit pings
            ESP_LOGD(TAG_CAN, "Transmitting ping");

            xSemaphoreTake(stop_ping_sem, portMAX_DELAY); // TEST
            ESP_LOGD(TAG_CAN, "got stop_ping_sem");
            for (int i = 0; i < 10; i++)
            {
                twai_transmit(&ping_message, portMAX_DELAY);
                printf(".");
                fflush(stdout);
                DELAY(PING_PERIOD_MS);
            }
            printf("\n");
        }
        else if (action == TX_SEND_START_CMD)
        {
            // Transmit start command to slave
            twai_transmit(&start_message, portMAX_DELAY);
            ESP_LOGD(TAG_CAN, "Transmitted start command");
        }
        else if (action == TX_SEND_STOP_CMD)
        {
            // Transmit stop command to slave
            twai_transmit(&stop_message, portMAX_DELAY);
            ESP_LOGD(TAG_CAN, "Transmitted stop command");
        }
        else if (action == TX_TASK_EXIT)
        {
            break;
        }
        else
        {
            ESP_LOGE(TAG_CAN, "unexpected action:%i", action);
            break;
        }
    }
    ESP_LOGD(TAG_CAN, "transmit task complete");
    vTaskDelete(NULL);
}

static void twai_control_task(void *arg)
{
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    tx_task_action_t tx_action;
    rx_task_action_t rx_action;

    for (int iter = 0; iter < NO_OF_ITERS; iter++)
    {
        ESP_ERROR_CHECK(twai_start());
        ESP_LOGD(TAG_CAN, "Driver started");

        // Start transmitting pings, and listen for ping response
        // tx_action = TX_SEND_PINGS;
        // rx_action = RX_RECEIVE_PING_RESP;
        // xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        // xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

        // Send Start command to slave, and receive data messages
        // xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
        tx_action = TX_SEND_START_CMD;
        rx_action = RX_RECEIVE_DATA;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

        // Send Stop command to slave when enough data messages have been received. Wait for stop response
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
        ESP_LOGD(TAG_CAN, "Sending stop command");
        tx_action = TX_SEND_STOP_CMD;
        rx_action = RX_RECEIVE_STOP_RESP;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
        ESP_ERROR_CHECK(twai_stop());
        ESP_LOGI(TAG, "Driver stopped");
        DELAY(ITER_DELAY_MS);
    }
    // Stop TX and RX tasks
    tx_action = TX_TASK_EXIT;
    rx_action = RX_TASK_EXIT;
    xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
    xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

    // Delete Control task
    xSemaphoreGive(done_sem);
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{

    // esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("*", LOG_MODE);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    esp_log_level_set("gpio", ESP_LOG_WARN);
    esp_log_level_set("intr_alloc", ESP_LOG_WARN);
    esp_log_level_set("sx1509", ESP_LOG_DEBUG);
    esp_log_level_set("i2cdev", ESP_LOG_DEBUG);
    if (LOG_MODE < ESP_LOG_DEBUG)
    {
        esp_log_level_set("system_api", ESP_LOG_WARN);
    }
    esp_log_level_set(TAG, LOG_MODE); // set *TAG to debug. This works, but I'd rather use the LOG_LOCAL_LEVEL macro. Also didn't work to put this in main() TEST
    esp_err_t ret;
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
