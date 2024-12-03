#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

// SPI Interface Pins
// #  define HSPI_HOST       SPI2_HOST
// #  define PIN_NUM_MISO      13
// #  define PIN_NUM_MOSI      12
// #  define PIN_NUM_CLK       11
// #  define PIN_NUM_CS        10


// I2C Interface Pins
// #define I2C_MASTER_SCL_IO    19   // GPIO pin for I2C SCL
// #define I2C_MASTER_SDA_IO    18   // GPIO pin for I2C SDA
// #define I2C_MASTER_NUM       I2C_NUM_0  // I2C port number
// #define I2C_MASTER_FREQ_HZ   100000     // I2C master clock frequency

// UART Interface Pins
#define UART_NUM UART_NUM_1
#define TX_PIN 17 // Adjust as per your connection
#define RX_PIN 18 // Adjust as per your connection
#define BUF_SIZE 1024
static uint8_t uart_data_buffer[BUF_SIZE];  // Use a global static buffer

// Cellular
#define APN_NAME "super"
SemaphoreHandle_t test_semaphore;

// MQTT
#define BUZZER_GPIO (gpio_num_t) 3 // GPIO pin for the buzzer
#define CAMERA_ON_TOPIC "/topic/camera_on"
#define STATE_TOPIC "/topic/state"
#define BUZZER_TOPIC "/topic/buzzer"
#define CONFIG_BROKER_URL_FROM_STDIN    1

typedef enum {
    STATE_SAFE,
    STATE_TROUBLE,
    STATE_DANGER
} SystemState;

static SystemState current_state = STATE_SAFE;
static esp_mqtt_client_handle_t mqtt_client;


static const char *TAG = "mqtt_example";

void update_system_state(const char *new_state) {
    if (strcmp(new_state, "safe") == 0) {
        current_state = STATE_SAFE;
        ESP_LOGI(TAG, "System state changed to SAFE");
    } else if (strcmp(new_state, "trouble") == 0) {
        current_state = STATE_TROUBLE;
        ESP_LOGI(TAG, "System state changed to TROUBLE");
    } else if (strcmp(new_state, "danger") == 0) {
        current_state = STATE_DANGER;
        ESP_LOGI(TAG, "System state changed to DANGER");
    } else {
        ESP_LOGW(TAG, "Trouble State: %s", new_state);
    }
}

void publish_location_task(void *pvParameter) {
    esp_mqtt_client_handle_t mqtt_client = (esp_mqtt_client_handle_t)pvParameter;
    while (1) {
        // Buffer to hold formatted location data
        char location_data[128];

        // Get current time
        time_t now = time(NULL);
        struct tm *tm_info = localtime(&now);

        // Code to get and format device location data
        // Poll location from EC25 Module (USB AT Command)
        // Example location coordinates (replace with your actual data)
        const char *latitude = "30.6187 N";
        const char *longitude = "96.3365 W";

        // Format timestamp as "YYYY-MM-DD HH:MM:SS"
        char timestamp[20];
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm_info);

        // Create the location data string
        snprintf(location_data, sizeof(location_data), "{\"timestamp\": \"%s\", \"location\": {\"latitude\": \"%s\", \"longitude\": \"%s\"}}", timestamp, latitude, longitude);
        
        // Publish location data to MQTT topic
        esp_mqtt_client_publish(mqtt_client, "/topic/location", location_data, 0, 1, 0);
        ESP_LOGW(TAG, "Published location data: %s", location_data);

        if (current_state == STATE_TROUBLE) {
            // Double the frequency of location logs in trouble state
            vTaskDelay(0.25 * 30000 / portTICK_PERIOD_MS);  // Publish every 30 seconds
        } else {
            vTaskDelay(0.25* 60000 / portTICK_PERIOD_MS);  // Publish every 60 seconds
        }
    }
}

static void start_camera() {
    printf("Camera on\n");
}

static void stop_camera() {
    printf("Camera off\n");
}

static void configure_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,  // Typical baud rate for EC25
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void send_at_command(const char* command) {
    uart_write_bytes(UART_NUM, command, strlen(command));
    uart_write_bytes(UART_NUM, "\r\n", 2);  // CR+LF for AT command termination
    ESP_LOGW(TAG, "Sent: %s", command);

    int len = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, (size_t*)&len));
    len = uart_read_bytes(UART_NUM, uart_data_buffer, len, pdMS_TO_TICKS(2000));
    if (len > 0) {
        uart_data_buffer[len] = '\0';  // Null-terminate the response
        ESP_LOGW(TAG, "Response: %s", (char*)uart_data_buffer);
    }
}

void reset_ec25() {
    send_at_command("AT+CFUN=1,1");  // Full reset of the EC25 module
    ESP_LOGI(TAG, "EC25 module reset. Waiting for reboot...");
    vTaskDelay(10000 / portTICK_PERIOD_MS);  // Wait for the module to reboot
}

void initialize_ec25() {
    //reset_ec25();
    //vTaskDelay(10000 / portTICK_PERIOD_MS);  // Wait for the module to reboot
    ESP_LOGI(TAG, "Checking Cell Comms");
    send_at_command("AT");              // Check communication
    ESP_LOGI(TAG, "Setting Verbose Error");
    send_at_command("AT+CMEE=1");       // verbose error
    ESP_LOGI(TAG, "Checking SIM Ready");
    send_at_command("AT+CPIN?");        // Check if SIM is ready
    ESP_LOGI(TAG, "Checking GPRS");
    send_at_command("AT+CGATT?");      // GPRS
    ESP_LOGI(TAG, "Attaching GPRS");
    send_at_command("AT+CGATT=1");      // Attach to GPRS
    send_at_command("AT+CGATT?");      // GPRS
    ESP_LOGI(TAG, "Set APN");
    // send_at_command("AT+CGDCONT=1,\"IP\",\"super\"");  // Set APN
    // Set APN
    char apn_command[64];
    snprintf(apn_command, sizeof(apn_command), "AT+CGDCONT=1,\"IP\",\"%s\"", APN_NAME);
    send_at_command(apn_command);
    // send_at_command("AT+QIACT=1");      // Activate PDP context
    // send_at_command("AT+QMTOPEN=0,\"mqtt_broker_url\",1883");  // Connect to MQTT broker
    // send_at_command("AT+QMTCONN=0,\"client_id\"");  // Connect MQTT client
}

void enable_gps() {
    send_at_command("AT+QGPS=1");  // Start GPS
}

void disable_gps() {
    send_at_command("AT+QGPSEND");  // Stop GPS
}

void parse_gps_data(const char* response) {
    ESP_LOGI(TAG, "Parsing GPS...");
    // Example response format: +QGPSLOC: 123456.000,31.2304,121.4737,39.4,0.0,0.0,210613,03
    // Time (UTC), Latitude, Longitude, Altitude, Speed, Course, Date (YYMMDD), Fix Mode
    char time[10], latitude[15], longitude[15], altitude[10], speed[10], date[10];

    sscanf(response, "+QGPSLOC: %9[^,],%14[^,],%14[^,],%9[^,],%9[^,],%*[^,],%9[^,]",
           time, latitude, longitude, altitude, speed, date);

    ESP_LOGI(TAG, "GPS Time (UTC): %s", time);
    ESP_LOGI(TAG, "Latitude: %s", latitude);
    ESP_LOGI(TAG, "Longitude: %s", longitude);
    ESP_LOGI(TAG, "Altitude: %s meters", altitude);
    ESP_LOGI(TAG, "Speed: %s km/h", speed);
    ESP_LOGI(TAG, "Date (YYMMDD): %s", date);
}

void get_gps_location() {
    ESP_LOGI(TAG, "Get GPS Loc");
    send_at_command("AT+QGPSLOC?");

    int len = uart_read_bytes(UART_NUM, uart_data_buffer, BUF_SIZE, pdMS_TO_TICKS(2000)); // Increased timeout
    if (len > 0) {
        uart_data_buffer[len] = '\0';  // Null-terminate the response
        ESP_LOGI(TAG, "GPS Response: %s", (char*)uart_data_buffer);

        // Parse the GPS data
        parse_gps_data((char*)uart_data_buffer);
    } else {
        ESP_LOGW(TAG, "No GPS data received.");
    }
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

void gps_task(void *pvParameters) {
    while (1) {
        get_gps_location();
        vTaskDelay(10000 / portTICK_PERIOD_MS);  // Poll every 10 seconds
    }
}


/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, STATE_TOPIC, "0", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, STATE_TOPIC, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, BUZZER_TOPIC, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, CAMERA_ON_TOPIC, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        if (event->topic_len == strlen(STATE_TOPIC) && strncmp(event->topic, STATE_TOPIC, event->topic_len) == 0) {
            // Update system state based on received message
            update_system_state(event->data);
        }
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        if (event->topic_len == strlen(STATE_TOPIC) && strncmp(event->topic, STATE_TOPIC, event->topic_len) == 0) {
            // Update system state based on received message
            update_system_state(event->data);
        } else if (event->topic_len == strlen(BUZZER_TOPIC) && strncmp(event->topic, BUZZER_TOPIC, event->topic_len) == 0) {
            // Handle buzzer topic here if needed
            char* num = event->topic;
            int N = int(num) - '0';
            if (N > 0) {
                gpio_set_level(BUZZER_GPIO, 1); //  the buzzer
            } else {
                gpio_set_level(BUZZER_GPIO, 0); // Deactivate the buzzer
            }
            
        } else if (event->topic_len == strlen(CAMERA_ON_TOPIC) && strncmp(event->topic, CAMERA_ON_TOPIC, event->topic_len) == 0) {
            // Handle camera topic here if needed
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start()
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {.address = {.uri=CONFIG_BROKER_URL}}
    };
#if 0
    char line[128];

    if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    //xTaskCreate(publish_location_task, "publish_location_task", 4096, (void *)client, 5, NULL);
}

bool test_uart_communication() {
    ESP_LOGW(TAG, "Testing UART communication...");
    send_at_command("AT");  // Simple AT command to check connectivity

    int len = uart_read_bytes(UART_NUM, uart_data_buffer, BUF_SIZE, pdMS_TO_TICKS(2000));

    if (len > 0) {
        uart_data_buffer[len] = '\0';
        ESP_LOGW(TAG, "UART Response: %s", (char*)uart_data_buffer);
        if (strstr((char*)uart_data_buffer, "OK") != NULL) {
            ESP_LOGW(TAG, "UART communication test passed!");
            return true;
        }
    }
    ESP_LOGE(TAG, "UART communication test failed!");
    return false;
}

bool test_gps_functionality() {
    ESP_LOGW(TAG, "Testing GPS functionality...");
    
    // Enable GPS
    enable_gps();
    
    // Give some time for GPS to acquire a fix
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    // Retrieve GPS data
    uart_write_bytes(UART_NUM, "AT+QGPSLOC?\r\n", strlen("AT+QGPSLOC?\r\n"));
    // uint8_t data[BUF_SIZE];
    int len = uart_read_bytes(UART_NUM, uart_data_buffer, BUF_SIZE, pdMS_TO_TICKS(5000));  // Increased timeout

    // Disable GPS
    disable_gps();

    if (len > 0) {
        ESP_LOGW(TAG, "GPS DATA RECV");
        uart_data_buffer[len] = '\0';
        ESP_LOGW(TAG, "GPS Response: %s", (char*)uart_data_buffer);

        if (strstr((char*)uart_data_buffer, "+QGPSLOC:") != NULL) {
            ESP_LOGW(TAG, "GPS functionality test passed!");
            parse_gps_data((char*)uart_data_buffer);  // Optionally parse the GPS data
            return true;
        }
    }

    ESP_LOGE(TAG, "GPS functionality test failed!");
    return false;
}

void run_tests_task() { //void *pvParameters
    // Wait until the semaphore is given
    //if (xSemaphoreTake(test_semaphore, portMAX_DELAY) == pdTRUE) {
        // Run the UART and GPS tests
        bool uart_test_passed = test_uart_communication();
        bool gps_test_passed = test_gps_functionality();

        // Display test results
        if (uart_test_passed && gps_test_passed) {
            ESP_LOGI(TAG, "All tests passed successfully!");
        } else {
            ESP_LOGE(TAG, "Some tests failed. Please check the logs.");
        }
    //}
    // Clean up and delete the task when done
    //vTaskDelete(NULL);
}

extern "C" void app_main()
{
    // MQTT client initialization

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize semaphore
    //test_semaphore = xSemaphoreCreateBinary();

    // Initialize the GPIO for the buzzer
    esp_rom_gpio_pad_select_gpio(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);

    // Configure UART for EC25
    // configure_uart();

    // Delay to allow GPS to acquire a fix
    // vTaskDelay(5000 / portTICK_PERIOD_MS);

    // Initialize EC25
    // initialize_ec25();

    // // Enable GPS functionality
    // enable_gps();

    // // Delay to allow GPS to acquire a fix
    // vTaskDelay(60000 / portTICK_PERIOD_MS);         

    // // Get GPS location data
    // get_gps_location();

    // // Disable GPS if not needed continuously
    // disable_gps();

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_ERROR_CHECK(example_connect());
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    // Run the tests
    // Increased stack size example for test functions
    

    mqtt_app_start();

    //xTaskCreate(&run_tests_task, "run_tests_task", 8192, NULL, 5, NULL);
    // run_tests_task();

    // xTaskCreate(&gps_task, "gps_task", 4096, NULL, 5, NULL);
    while (1) {
        gpio_set_level(BUZZER_GPIO, 0); // Deactivate the buzzer
        if (current_state == STATE_TROUBLE) {
            ESP_LOGW(TAG, "System is in trouble state... doubling location logs.");
        } else if (current_state == STATE_DANGER) {
            ESP_LOGE(TAG, "System is in danger state... starting buzzer.");
            gpio_set_level(BUZZER_GPIO, 1); // Activate the buzzer

        } else {
            gpio_set_level(BUZZER_GPIO, 0); // Deactivate the buzzer
        }
        while (current_state == STATE_SAFE) {
            gpio_set_level(BUZZER_GPIO, 0); // Deactivate the buzzer
            ESP_LOGI(TAG, "System is in safe state... going to sleep.");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
        
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}