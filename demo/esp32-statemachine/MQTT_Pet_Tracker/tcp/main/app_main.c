/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

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

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#define BUZZER_GPIO 33 // GPIO pin for the buzzer
#define CAMERA_ON_TOPIC "/topic/camera_on"

// SPI Interface Pins
#  define HSPI_HOST       SPI2_HOST
#  define PIN_NUM_MISO      13
#  define PIN_NUM_MOSI      12
#  define PIN_NUM_CLK       11
#  define PIN_NUM_CS        10

static const char *TAG = "mqtt_example";

typedef enum {
    STATE_SAFE,
    STATE_TROUBLE,
    STATE_DANGER
} SystemState;

static SystemState current_state = STATE_SAFE;
static esp_mqtt_client_handle_t mqtt_client;
static spi_device_handle_t spi_camera;

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
        ESP_LOGW(TAG, "Unknown state received: %s", new_state);
    }
}

void publish_location_task(void *pvParameter) {
    esp_mqtt_client_handle_t mqtt_client = (esp_mqtt_client_handle_t)pvParameter;
    while (1) {
        // Code to get and format device location data
        // Poll location from EC25 Module (USB AT Command)
        const char *location_data = "30.6187 N, 96.3365 W";  // Example location data
        
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

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

// Function to initialize the camera and start capturing images
// SPI Interface Pins
// #  define HSPI_HOST       SPI2_HOST
// #  define PIN_NUM_MISO      13
// #  define PIN_NUM_MOSI      12
// #  define PIN_NUM_CLK       11
// #  define PIN_NUM_CS        10
static void start_camera() {
    // Code to initialize SPI2 for camera communication
    // spi_bus_config_t buscfg = {
    //     .miso_io_num = PIN_NUM_MISO,
    //     .mosi_io_num = PIN_NUM_MOSI,
    //     .sclk_io_num = PIN_NUM_CLK,
    //     .quadwp_io_num = -1,
    //     .quadhd_io_num = -1,
    // };
    // ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_DISABLED));

    // spi_device_interface_config_t devcfg = {
    //     .clock_speed_hz = 10 * 1000 * 1000,           // Clock out at 10 MHz
    //     .mode = 0,                                    // SPI mode 0
    //     .spics_io_num = PIN_NUM_CS,                           // CS pin not used
    //     .queue_size = 1,                              // We want to be able to queue 1 transaction at a time
    // };
    // ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi_camera));
    // // Code to send initialization commands to the camera via SPI
    // // Example initialization commands:
    // uint8_t init_cmd[] = {0x3008, 0x80}; // Example initialization commands
    // spi_transaction_t trans = {
    //     .tx_buffer = init_cmd,
    //     .length = 8 * sizeof(init_cmd), // Length in bits
    // };
    // esp_err_t ret = spi_device_transmit(spi_camera, &trans);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to send camera initialization commands");
    // }
    
    printf("Camera on\n");
}

static void stop_camera() {
    printf("Camera off\n");
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
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);

        // Publish the "safe" state on startup
        msg_id = esp_mqtt_client_subscribe(client, "/topic/state", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_publish(client, "/topic/state", "safe", 0, 1, 0);
        ESP_LOGI(TAG, "Published safe state, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/camera_on", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_publish(client, "/topic/camera_on", "0", 0, 1, 0);
        ESP_LOGI(TAG, "Published camera_on - off, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
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

        // Check if the received topic is the state topic
        if (strncmp(event->topic, "/topic/state", event->topic_len) == 0) {
            char state_buffer[10];
            memcpy(state_buffer, event->data, event->data_len);
            state_buffer[event->data_len] = '\0';
            update_system_state(state_buffer);
        } else if (strncmp(event->topic, CAMERA_ON_TOPIC, event->topic_len) == 0) {
            // Check the value received
            if (event->data_len == 1 && event->data[0] == '1') {
                // Start the camera
                start_camera();
            } else if (event->data_len == 1 && event->data[0] == '0') {
                // Stop the camera
                stop_camera();
            } else {
                printf("Invalid value for camera control: %.*s\n", event->data_len, event->data);
            }
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

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };
#if CONFIG_BROKER_URL_FROM_STDIN
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
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    // Create task to periodically publish device location and pass MQTT client handle
    xTaskCreate(publish_location_task, "publish_location_task", 4096, (void *)client, 5, NULL);
}

void app_main(void)
{
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

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();

    // Initialize the GPIO for the buzzer
    esp_rom_gpio_pad_select_gpio(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(BUZZER_GPIO, 0); // Deactivate the buzzer
        while (current_state == STATE_SAFE) {
            ESP_LOGI(TAG, "System is in safe state... going to sleep.");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
        if (current_state == STATE_TROUBLE) {
            ESP_LOGW(TAG, "System is in trouble state... doubling location logs.");
        } else if (current_state == STATE_DANGER) {
            ESP_LOGE(TAG, "System is in danger state... starting buzzer.");
            gpio_set_level(BUZZER_GPIO, 1); // Activate the buzzer

        } else {
            gpio_set_level(BUZZER_GPIO, 0); // Deactivate the buzzer
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

