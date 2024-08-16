#include "math.h"

#include <time.h>

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include <sensor_msgs/msg/laser_scan.h>

#include "driver/gpio.h"
#include "driver/uart.h"

static const char *TAG = "Lidar";

#define BUF_SIZE (1024)

#define UART_GPIO_TX                        19 //gpio esp tx
#define UART_GPIO_RX                        8 //gpio esp rx
#define GPIO_LIDAR_PWM                      20 //gpio motor pin
#define GPIO_LIDAR_POWER                    3 //gpio motor pin

#define GPIO_LED_STATUS                     16 //gpio led buildin

#define UART_PORT_NUM                       1
#define UART_BAUD_RATE                      115200

void lidar_task(void *argument);
void init_lidar();
void read_lidar();

extern void timestamp_update(void*arg);
extern void lidar_pub_callback();

extern sensor_msgs__msg__LaserScan msgs_laserscan;

extern SemaphoreHandle_t uros_boot_lidar;

uint8_t LIDAR_PKG_STOP_SCAN[] = {0xA5, 0x25}; //sem resposta
uint8_t LIDAR_PKG_RESET_CORE[] = {0xA5, 0x40}; //sem resposta

uint8_t LIDAR_PKG_START_SCAN[] = {0xA5, 0x20}; //resposta descriptor: A5 5A 05 00 00 40 81 - 5 bytes
uint8_t LIDAR_PKG_EXPRESS_SCAN[] = {0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22}; //resposta descriptor: A5 5A 54 00 00 40 82 - 84 bytes
uint8_t LIDAR_PKG_FORCE_SCAN[] = {0xA5, 0x21}; //resposta descriptor: A5 5A 05 00 00 40 81 - 5 bytes

uint8_t LIDAR_PKG_GET_INFO[] = {0xA5, 0x50}; //resposta descriptor: A5 5A 14 00 00 00 04 - 20 bytes
uint8_t LIDAR_PKG_GET_HEALTH[] = {0xA5, 0x52}; //resposta descriptor: A5 5A 03 00 00 00 06 - 3 bytes
uint8_t LIDAR_PKG_GET_SAMPLERATE[] = {0xA5, 0x59}; //resposta descriptor: A5 5A 04 00 00 00 15 - 4 bytes

int measures = 0;


void init_lidar(){
    uint8_t *data_uart = (uint8_t *) malloc(BUF_SIZE);
    gpio_set_level(GPIO_LIDAR_PWM, 0);
    gpio_set_level(GPIO_LIDAR_POWER, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    gpio_set_level(GPIO_LIDAR_POWER, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(GPIO_LIDAR_PWM, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));

    //reset
    //uart_write_bytes(UART_PORT_NUM, (const char *) LIDAR_PKG_RESET_CORE, 2);
    //vTaskDelay(pdMS_TO_TICKS(6));

    //read health
    ESP_LOGI(TAG,"Sending Lidar get health...");
    uart_write_bytes(UART_PORT_NUM, (const char *) LIDAR_PKG_GET_HEALTH, 2);

    ESP_LOGI(TAG,"Receiving Lidar health...");
    int len = uart_read_bytes(UART_PORT_NUM, data_uart, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS); //(BUF_SIZE - 1)
    ESP_LOGI(TAG, "BUFFER: %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X", data_uart[0],data_uart[1],data_uart[2],data_uart[3],data_uart[4],data_uart[5],data_uart[6],data_uart[7],data_uart[8],data_uart[9],data_uart[10]);
    //uart_get_buffered_data_len();
    if (len == 10){
        if((data_uart[0] == 0xA5) && (data_uart[1] == 0x5A) && (data_uart[2] == 0x03) && (data_uart[3] == 0x00) && (data_uart[4] == 0x00) && (data_uart[5] == 0x00) && (data_uart[6] == 0x06) && (data_uart[7] == 0x00) && (data_uart[8] == 0x00) && (data_uart[9] == 0x00)){
            ESP_LOGI(TAG,"Lidar health good!");
        } else {
            ESP_LOGE(TAG,"Receive 10bytes but lidar health is bad...");
        }
    } else {
        ESP_LOGE(TAG,"Error on receiving lidar health!");
        uint8_t tentativas = 0;
        while ((data_uart[0] != 0xA5) && (data_uart[1] != 0x5A) && (data_uart[2] != 0x03) && (data_uart[3] != 0x00) && (data_uart[4] != 0x00) && (data_uart[5] != 0x00) && (data_uart[6] != 0x06) && (data_uart[7] != 0x00) && (data_uart[8] != 0x00) && (data_uart[9] != 0x00)){
            ESP_LOGW(TAG,"Trying again: %d", tentativas);
            tentativas++;
            ESP_LOGW(TAG,"Sending Lidar get health...");
            uart_write_bytes(UART_PORT_NUM, (const char *) LIDAR_PKG_GET_HEALTH, 2);

            ESP_LOGW(TAG,"Receiving Lidar health...");
            uart_read_bytes(UART_PORT_NUM, data_uart, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS); //(BUF_SIZE - 1)
            ESP_LOGW(TAG, "BUFFER: %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X", data_uart[0],data_uart[1],data_uart[2],data_uart[3],data_uart[4],data_uart[5],data_uart[6],data_uart[7],data_uart[8],data_uart[9],data_uart[10]);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        ESP_LOGI(TAG,"Lidar health good!");
        //uart_get_buffered_data_len();
    }
    free(data_uart);
}

void read_lidar(){
    uint8_t *data_uart = (uint8_t *) malloc(BUF_SIZE);

    //start scan
    ESP_LOGI(TAG,"Sending Lidar start scan...");
    uart_write_bytes(UART_PORT_NUM, (const char *) LIDAR_PKG_START_SCAN, 2);

    ESP_LOGI(TAG,"Receiving Lidar scan...");
    int len = uart_read_bytes(UART_PORT_NUM, data_uart, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS); //(BUF_SIZE - 1)
    ESP_LOGI(TAG, "BUFFER: %02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X", data_uart[0],data_uart[1],data_uart[2],data_uart[3],data_uart[4],data_uart[5],data_uart[6],data_uart[7],data_uart[8],data_uart[9],data_uart[10]);
    //uart_get_buffered_data_len();
    if (len == 7){
        if((data_uart[0] == 0xA5) && (data_uart[1] == 0x5A) && (data_uart[2] == 0x05) && (data_uart[3] == 0x00) && (data_uart[4] == 0x00) && (data_uart[5] == 0x40) && (data_uart[6] == 0x81)){
            ESP_LOGI(TAG,"Lidar scan started!");
        } else {
            ESP_LOGE(TAG,"Receive 10bytes but lidar scan is bad...");
        }
    } else {
        ESP_LOGE(TAG,"Error on receiving lidar scan!");
        init_lidar();
        //vTaskDelay(pdMS_TO_TICKS(10000));
        //esp_restart();
    }
    free(data_uart);
}

void lidar_task(void * arg){
    gpio_reset_pin(GPIO_LIDAR_PWM);
    gpio_set_direction(GPIO_LIDAR_PWM, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LIDAR_PWM, 0);

    gpio_reset_pin(GPIO_LIDAR_POWER);
    gpio_set_direction(GPIO_LIDAR_POWER, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LIDAR_POWER, 0);

    gpio_reset_pin(GPIO_LED_STATUS);
    gpio_set_direction(GPIO_LED_STATUS, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LED_STATUS, 1);

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_GPIO_TX, UART_GPIO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGW(TAG, "Waiting semaphore from uROS boot");
    xSemaphoreTake(uros_boot_lidar, portMAX_DELAY);
    ESP_LOGI(TAG, "Resuming semaphore...");

    //uart_enable_pattern_det_baud_intr();

    init_lidar();

    read_lidar();

    gpio_set_level(GPIO_LED_STATUS, 0);

    while(1){
        uint8_t *data_scan = (uint8_t *) malloc(10);
        int len = uart_read_bytes(UART_PORT_NUM, data_scan, 5, 20 / portTICK_PERIOD_MS); //(BUF_SIZE - 1)
        //ESP_LOGE(TAG, "Len: %d | BUFFER: %02X,%02X,%02X,%02X,%02X", len, data_scan[0], data_scan[1], data_scan[2], data_scan[3], data_scan[4]);
        //check quality
        if((len == 5) && ( (data_scan[1]&0x01) & ((data_scan[0] ^ (data_scan[0]>>1)) & 0x01))){
            float angle = ((data_scan[1] >> 1) | (data_scan[2] << 7))/64.0; //deg
		    uint8_t start_scan_flag = data_scan[0] & 0x1;
		    uint8_t quality = data_scan[0]>>2;

            if (start_scan_flag == 1){
                gpio_set_level(GPIO_LED_STATUS, 1);
                measures = 0;
                timestamp_update(&msgs_laserscan);
                lidar_pub_callback();
                gpio_set_level(GPIO_LED_STATUS, 0);
                start_scan_flag = 0;
            } else if (start_scan_flag == 0){
                measures++;
            }

            msgs_laserscan.ranges.data[measures] = ((data_scan[3] | (data_scan[4]<<8))/4.0)/1000; //meters

            ESP_LOGI(TAG,"Packet>>> Flag: %d | Quality: %d | Angle: %3.6f | Distance: %2.6f", start_scan_flag, quality, angle, msgs_laserscan.ranges.data[measures]);
            //ESP_LOGI(TAG, "BUFFER: %02X,%02X,%02X,%02X,%02X", data_scan[0],data_scan[1],data_scan[2],data_scan[3],data_scan[4]);
        } else {
            ESP_LOGE(TAG, "Len: %d | BUFFER: %02X,%02X,%02X,%02X,%02X", len, data_scan[0], data_scan[1], data_scan[2], data_scan[3], data_scan[4]);
        }
        free(data_scan);
    }
    ESP_LOGE(TAG, "Task Delete");
    vTaskDelete(NULL);
}