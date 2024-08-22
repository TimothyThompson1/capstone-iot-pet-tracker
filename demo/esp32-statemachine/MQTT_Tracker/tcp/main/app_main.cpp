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

#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "ArduCAM.h"

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
// #define OV5642_I2C_ADDRESS   0x78       // I2C 

#define BMPIMAGEOFFSET 66




static const char *TAG = "mqtt_example";


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
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
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
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
    esp_mqtt_client_register_event(client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}


// Image capture and MQTT publishing logic
void capture_and_publish_image() {
    // Capture image from ArduCAM module (SPI communication)
    // Read image data from SPI

    // Publish image data over MQTT
    // Example MQTT publishing logic
    //esp_mqtt_client_publish(client, "/topic/image", image_data, image_size, 0, 0);
}




extern "C" void app_main()
{
    // MQTT client initialization

    // ESP_LOGI(TAG, "[APP] Startup..");
    // ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    // ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    // esp_log_level_set("*", ESP_LOG_INFO);
    // esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    // esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    // esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    // esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    // esp_log_level_set("transport", ESP_LOG_VERBOSE);
    // esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    // ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    // /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    //  * Read "Establishing Wi-Fi or Ethernet Connection" section in
    //  * examples/protocols/README.md for more information about this function.
    //  */
    // ESP_ERROR_CHECK(example_connect());

    // mqtt_app_start();

    uint8_t bmp_header[BMPIMAGEOFFSET] =
    {
    0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
    0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
    0x00, 0x00
    };
    
    
    ArduCAM myCAM(OV5642, PIN_CS);
    //set_fifo_burst(ArduCAM myCAM);

    uint8_t vid, pid;
    uint8_t cameraCommand;
    //stdio_init_all();

    myCAM.Arducam_init();	
    gpio_set_direction(PIN_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_CS, 1);

    // Example camera initialization command
    //uint8_t init_cmd[] = {0x3008,0x80}

    myCAM.write_reg(0x07, 0x80);
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Sleep for 100ms
    myCAM.write_reg(0x07, 0x00);
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Sleep for 100ms

    while (1) 
    {
        //Check if the ArduCAM SPI bus is OK
        myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
        cameraCommand = myCAM.read_reg(ARDUCHIP_TEST1);
        if (cameraCommand != 0x55) {
            printf(" SPI interface Error!");
            vTaskDelay(1000 / portTICK_PERIOD_MS); continue;
        } else {
            printf("ACK CMD SPI interface OK.END"); break;
        }
	}

    while (1) 
    {
        //Check if the camera module type is OV5640
        myCAM.wrSensorReg16_8(0xff, 0x01);
        myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
        myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
        if((vid != 0x56) || (pid != 0x42))
        {
        printf("Can't find OV5642 module!");
        vTaskDelay(100 / portTICK_PERIOD_MS); continue;
        }
        else 
        {
        printf("OV5642 detected.END"); break;
        }
    }
}



// void ov5642_write_register(uint8_t reg, uint8_t val) {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (OV5642_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(cmd, reg, true);
//     i2c_master_write_byte(cmd, val, true);
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, portMAX_DELAY);
//     i2c_cmd_link_delete(cmd);
// }

// void i2c_write_reg(uint8_t reg, uint8_t val)
// {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (OV5642_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(cmd, reg, true);
//     i2c_master_write_byte(cmd, val, true);
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
//     i2c_cmd_link_delete(cmd);
// }

// uint8_t i2c_read_reg(uint8_t reg)
// {
//     uint8_t val;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (OV5642_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(cmd, reg, true);
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
//     i2c_cmd_link_delete(cmd);
//     cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (OV5642_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
//     i2c_master_read_byte(cmd, &val, I2C_MASTER_NACK);
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
//     i2c_cmd_link_delete(cmd);
//     return val;
// }

// void spi_write_reg(uint16_t addr, uint8_t data)
// {
//     spi_transaction_t trans = {};
//     trans.length = 16;
//     trans.tx_buffer = (uint8_t*)&addr;
//     spi_device_polling_transmit(spi, &trans);
//     trans.length = 8;
//     trans.tx_buffer = &data;
//     spi_device_polling_transmit(spi, &trans);
// }

// uint8_t spi_read_reg(uint16_t addr)
// {
//     spi_transaction_t trans = {};
//     trans.length = 16;
//     trans.tx_buffer = (uint8_t*)&addr;
//     spi_device_polling_transmit(spi, &trans);
//     uint8_t data;
//     trans.length = 8;
//     trans.rxlength = 8;
//     trans.rx_buffer = &data;
//     spi_device_polling_transmit(spi, &trans);
//     return data;
// }

// void spi_write_reg16_8(uint16_t addr, uint8_t data)
// {
//     uint8_t tx_data[3];
//     tx_data[0] = (addr >> 8) & 0xFF; // MSB of address
//     tx_data[1] = addr & 0xFF;        // LSB of address
//     tx_data[2] = data;               // Data to write
//     spi_transaction_t trans = {};
//     trans.length = 24;               // 16 bits address + 8 bits data
//     trans.tx_buffer = tx_data;
//     spi_device_polling_transmit(spi, &trans);
// }

// uint8_t spi_read_reg16_8(uint16_t addr)
// {
//     uint8_t tx_data[2], rx_data[3];
//     tx_data[0] = (addr >> 8) & 0xFF; // MSB of address
//     tx_data[1] = addr & 0xFF;        // LSB of address
//     spi_transaction_t trans = {};
//     trans.length = 24;               // 16 bits address + 8 bits data
//     trans.tx_buffer = tx_data;
//     trans.rxlength = 8;              // 8 bits data
//     trans.rx_buffer = rx_data;
//     spi_device_polling_transmit(spi, &trans);
//     return rx_data[2];               // Return received data
// }

// void i2c_write_reg16_8(uint8_t addr, uint16_t reg, uint8_t data)
// {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true); // MSB of register
//     i2c_master_write_byte(cmd, reg & 0xFF, true);        // LSB of register
//     i2c_master_write_byte(cmd, data, true);              // Data to write
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
//     i2c_cmd_link_delete(cmd);
// }

// uint8_t i2c_read_reg16_8(uint8_t addr, uint16_t reg)
// {
//     uint8_t data;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true); // MSB of register
//     i2c_master_write_byte(cmd, reg & 0xFF, true);        // LSB of register
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
//     i2c_cmd_link_delete(cmd);
    
//     cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
//     i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
//     i2c_cmd_link_delete(cmd);
    
//     return data;
// }

// #define ARDUCHIP_FIFO 0x04
// #define FIFO_CLEAR_MASK 0x01
// #define FIFO_START_MASK 0x02

// void flush_fifo(void)
// {
//     i2c_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
// }

// void start_capture(void)
// {
// 	i2c_write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
// }

// void clear_fifo_flag(void )
// {
// 	i2c_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
// }

// #define FIFO_SIZE1 0x42
// #define FIFO_SIZE2 0x43
// #define FIFO_SIZE3 0x44


// uint32_t read_fifo_length(void)
// {
// 	uint32_t len1,len2,len3,length=0;
// 	len1 = read_reg(FIFO_SIZE1);
//   len2 = read_reg(FIFO_SIZE2);
//   len3 = read_reg(FIFO_SIZE3) & 0x7f;
//   length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
// 	return length;	
// }

// ArduCAM initialization function
//void init_arducam() {
    // uint8_t vid, pid;
    // uint8_t cameraCommand;
    // stdio_init_all();

    // myCAM.Arducam_init();	
    // gpio_set_direction(CS_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_level(CS_PIN, 1);

    // // Example camera initialization command
    // //uint8_t init_cmd[] = {0x3008,0x80}

    // myCAM.write_reg(0x07, 0x80);
    // vTaskDelay(100 / portTICK_PERIOD_MS);  // Sleep for 100ms
    // myCAM.write_reg(0x07, 0x00);
    // vTaskDelay(100 / portTICK_PERIOD_MS);  // Sleep for 100ms

    // while (1) 
    // {
    //     //Check if the ArduCAM SPI bus is OK
    //     myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    //     cameraCommand = myCAM.read_reg(ARDUCHIP_TEST1);
    //     if (cameraCommand != 0x55) {
    //         printf(" SPI interface Error!");
    //         vTaskDelay(1000 / portTICK_PERIOD_MS); continue;
    //     } else {
    //         printf("ACK CMD SPI interface OK.END"); break;
    //     }
	// }

    // while (1) 
    // {
    //     //Check if the camera module type is OV5640
    //     myCAM.wrSensorReg16_8(0xff, 0x01);
    //     myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    //     myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    //     if((vid != 0x56) || (pid != 0x42))
    //     {
    //     printf("Can't find OV5642 module!");
    //     vTaskDelay(1000 / portTICK_PERIOD_MS); continue;
    //     }
    //     else 
    //     {
    //     printf("OV5642 detected.END"); break;
    //     }
    // }

    //Change to JPEG capture mode and initialize the OV5642 module
    // myCAM.set_format(JPEG);
    // myCAM.InitCAM();
    // myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);   //VSYNC is active HIGH
    // myCAM.OV5642_set_JPEG_size(OV5642_320x240);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // myCAM.clear_fifo_flag();
    // myCAM.write_reg(ARDUCHIP_FRAMES,0x00);





    // esp_err_t ret = send_command(spi, CMD_INIT, init_cmd, sizeof(init_cmd));
    // if (ret != ESP_OK) {
    //     ESP_LOGE("ARDUCAM", "Camera initialization failed");
    //     // Handle error
    // } else {
    //     ESP_LOGI("ARDUCAM", "Camera initialized successfully");
    //     // Proceed with image capture or other operations
    // }
//}