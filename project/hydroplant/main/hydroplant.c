#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <esp_err.h>
#include "lwip/err.h"
#include "lwip/sys.h"


#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "soc/soc_caps.h"
#include "freertos/timers.h"

#include "esp_netif.h"
#include "esp_http_client.h"
#include "my_data.h"
#define BLINK_LED 2 
#include "esp_mac.h"

#include "driver/i2c.h"
#include <string.h>

#include <dht.h>
#include <pcf8574.h>
#include <ds18x20.h>
#include <inttypes.h>

//#if defined(CONFIG_EXAMPLE_TYPE_DHT11)
//#define SENSOR_TYPE DHT_TYPE_DHT11
//#endif
//#if defined(CONFIG_EXAMPLE_TYPE_AM2301)
//#define SENSOR_TYPE DHT_TYPE_AM2301
//#endif
//#if defined(CONFIG_EXAMPLE_TYPE_SI7021)
//#define SENSOR_TYPE DHT_TYPE_SI7021
//#endif

#define SENSOR_TYPE DHT_TYPE_DHT11
#define CONFIG_EXAMPLE_DATA_GPIO 26
#define SENSOR_GPIO 25
#define SENSOR_ADDR 0x7a3c1bf649551728

#define TDS_NUM_SAMPLES             10  //(int) Number of reading to take for an average
#define TDS_SAMPLE_PERIOD           20  //(int) Sample period (delay between samples == sample period / number of readings)
#define TDS_TEMPERATURE             18.0  //(float) Temperature of water (we should measure this with a sensor to get an accurate reading)

#define TDS_VREF                    1.18   //(float) Voltage reference for ADC. We should measure the actual value of each ESP32

/* The examples use WiFi configuration that you can set via project configuration menu
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define SSID "ReggaeHouse2.4G"
#define PASS "123456789"

#define EXAMPLE_ESP_WIFI_SSID      SSID
#define EXAMPLE_ESP_WIFI_PASS      PASS
#define EXAMPLE_ESP_MAXIMUM_RETRY  3
#define CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK 1

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

//#define PCF8574_1_READ_ADDR                 0x7E        /*!< Slave address of the PCF8574 sensor */
//#define PCF8574_2_READ_ADDR                 0x70
//#define PCF8574_1_WRITE_ADDR                0x7F
//#define PCF8574_2_WRITE_ADDR                0x71
#define PCF8574_1_BASE_ADDR                 0x20        /*!< Slave address of the PCF8574 sensor */
#define PCF8574_2_BASE_ADDR                 0x27

#define MPU9250_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT                   7

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


#define bomba1_pin 23
#define sw1_pin 4
#define sw2_pin 13
#define sw3_pin 35
#define fan1_2_pin 14
#define fan3_4_pin 12
#define io_int36_pin 39
#define io_int34_pin 34
#define motor5_pin 15
#define ppm_pin 36
#define ph_pin 36

static const char *TAG = "wifi station";
static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
static int s_retry_num = 0;

float waterTemp = 25.0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
	     * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

esp_err_t client_event_post_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data);
        break;

    default:
        break;
    }
    return ESP_OK;
}

esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data);
        break;

    default:
        break;
    }
    return ESP_OK;
}

static void rest_get()
{
    esp_http_client_config_t config_get = {
        .url = "https://plant-arm-project-default-rtdb.europe-west1.firebasedatabase.app/.json",
        .method = HTTP_METHOD_GET,
        .cert_pem = NULL,
        .event_handler = client_event_get_handler};
     
    esp_http_client_handle_t client = esp_http_client_init(&config_get);
    //esp_http_client_set_timeout_ms(client, 30);   
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

static void post_rest_function()
{
    esp_http_client_config_t config_post = {
        .url = "https://plant-arm-project-default-rtdb.europe-west1.firebasedatabase.app/.json",
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .event_handler = client_event_post_handler};
        
    esp_http_client_handle_t client = esp_http_client_init(&config_post);

    char  *post_data = "AirTemp: 27";
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_http_client_set_header(client, "Content-Type", "application/json");

    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

static void post_test()
{
// POST
    
    esp_http_client_config_t config_post = {
        .url = "https://plant-arm-project-default-rtdb.europe-west1.firebasedatabase.app/.json",
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .event_handler = client_event_post_handler};
        const char *post_data = "{\"title\":\"test\"}";
    //const char *post_data = "{\"title\":\"test\"}";
    esp_http_client_handle_t client = esp_http_client_init(&config_post);
    
    esp_http_client_set_url(client, "https://plant-arm-project-default-rtdb.europe-west1.firebasedatabase.app/.json");
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %lld",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
}

    static void patch_test()
{
    //PATCH
    esp_http_client_config_t config_post = {
        .url = "https://plant-arm-project-default-rtdb.europe-west1.firebasedatabase.app/-NFQX8-b2oqRksquXzo3/.json",
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .event_handler = client_event_post_handler};
    const char *post_data = "{\"Luz\":\"test alterado\"}";
    esp_http_client_handle_t client = esp_http_client_init(&config_post);
    esp_http_client_set_url(client, "https://plant-arm-project-default-rtdb.europe-west1.firebasedatabase.app/-NFQX8-b2oqRksquXzo3/.json");
    esp_http_client_set_method(client, HTTP_METHOD_PATCH);
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP PATCH Status = %d, content_length = %lld",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP PATCH request failed: %s", esp_err_to_name(err));
    }
}

    static void put_test()
{
//PUT
    esp_http_client_config_t config_post = {
        .url = "https://plant-arm-project-default-rtdb.europe-west1.firebasedatabase.app/.json",
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .event_handler = client_event_post_handler};
    const char *post_data = "{\"title\":\"test2\"}";
    esp_http_client_handle_t client = esp_http_client_init(&config_post);
    esp_http_client_set_url(client, "https://plant-arm-project-default-rtdb.europe-west1.firebasedatabase.app/.json");
    esp_http_client_set_method(client, HTTP_METHOD_PUT);
    esp_http_client_set_post_field(client, NULL, 0);
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP PUT Status = %d, content_length = %lld",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP PUT request failed: %s", esp_err_to_name(err));
    }
}

void dht_test(void *pvParameters)
{
    float temperature, humidity;
        while (1)
        {
            if (dht_read_float_data(SENSOR_TYPE, CONFIG_EXAMPLE_DATA_GPIO, &humidity, &temperature) == ESP_OK)
                ESP_LOGI("DHT_test:","Air Humidity: %.1f%% Air Temp: %.1fC\n", humidity, temperature);
            else
                ESP_LOGI("DHT_test:","Could not read data from sensor\n");
            break; 
        }
    vTaskDelete(NULL);
}

void ds18b20_test(void *pvParameter)
{

    float temperature;
    esp_err_t res;
    while (1)
    {
        res = ds18x20_measure_and_read(SENSOR_GPIO, SENSOR_ADDR, &temperature);
        if (res != ESP_OK)
            ESP_LOGE("Water_temp_test", "Could not read from sensor %08" PRIx32 "%08" PRIx32 ": %d (%s)",
                    (uint32_t)(SENSOR_ADDR >> 32), (uint32_t)SENSOR_ADDR, res, esp_err_to_name(res));
        else
            ESP_LOGI("Water_temp_test", "Water Temp: %.2f°C", temperature);
        break;
        //vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

void pcf_test1(void *pvParameters)
{
    i2c_dev_t pcf8574;

    // Zero device descriptor
    memset(&pcf8574, 0, sizeof(i2c_dev_t));

    // Init i2c device descriptor
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, PCF8574_1_BASE_ADDR, 0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));

    // Do some blinking
    uint16_t port_val = 0xaa;
    while (1)
    {
        // invert value
        port_val = ~port_val;

        // write value to port
        pcf8574_port_write(&pcf8574, port_val);

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    vTaskDelete(NULL);
}

void pcf_test2(void *pvParameters)
{
    i2c_dev_t pcf8574;

    // Zero device descriptor
    memset(&pcf8574, 0, sizeof(i2c_dev_t));

    // Init i2c device descriptor
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, PCF8574_2_BASE_ADDR, 0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));

    // Do some blinking
    uint16_t port_val = 0xaa;
    while (1)
    {
        // invert value
        port_val = ~port_val;

        // write value to port
        pcf8574_port_write(&pcf8574, port_val);

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    vTaskDelete(NULL);
}


float convert_to_ppm(float averageVoltage, float waterTemp){
    ESP_LOGI("TDS", "Converting an analog value to a TDS PPM value.");
    //https://www.dfrobot.com/wiki/index.php/Gravity:_Analog_TDS_Sensor_/_Meter_For_Arduino_SKU:_SEN0244#More_Documents
    //float adcCompensation = 1 + (1/3.9); // 1/3.9 (11dB) attenuation.
    //float vPerDiv = (TDS_VREF / 4096) * adcCompensation; // Calculate the volts per division using the VREF taking account of the chosen attenuation value.
    //float averageVoltage = analogReading * vPerDiv; // Convert the ADC reading into volts
    float compensationCoefficient=1.0+0.02*(waterTemp-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = (averageVoltage / compensationCoefficient)/1000;  //temperature compensation
    float tdsValue = (((133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5)/0.79192); //convert voltage value to tds value

    //ESP_LOGI("TDS", "Volts per division = %f", vPerDiv);
    //ESP_LOGI("TDS", "Average Voltage = %f", averageVoltage);
    //ESP_LOGI("TDS", "Temperature (currently fixed, we should measure this) = %f", waterTemp);
    //ESP_LOGI("TDS", "Compensation Coefficient = %f", compensationCoefficient);
    //ESP_LOGI("TDS", "Compensation Voltge = %f", compensationVolatge);
    ESP_LOGI("TDS", "tdsValue = %f ppm", tdsValue);
    return tdsValue;
}

float convert_to_ph(float averageVoltage){
    ESP_LOGI("PH", "Converting an analog value to a PH value.");
    //https://www.dfrobot.com/wiki/index.php/Gravity:_Analog_TDS_Sensor_/_Meter_For_Arduino_SKU:_SEN0244#More_Documents
    //float adcCompensation = 1 + (1/3.9); // 1/3.9 (11dB) attenuation.
    //float vPerDiv = (TDS_VREF / 4096) * adcCompensation; // Calculate the volts per division using the VREF taking account of the chosen attenuation value.
    //float averageVoltage = analogReading * vPerDiv; // Convert the ADC reading into volts
    float compensationCoefficient=1;    // compensation 
    float phValue = (((compensationCoefficient * 2.8 * averageVoltage)/0.20318721)/1000);

    ESP_LOGI("PH", "phValue = %f", phValue);
    return phValue;
}


void app_main(void)
{
    //Initialize NVS
    gpio_reset_pin(bomba1_pin);
    gpio_set_direction(bomba1_pin, GPIO_MODE_OUTPUT);
    gpio_reset_pin(sw1_pin);
    gpio_set_direction(sw1_pin, GPIO_MODE_INPUT); 
    gpio_reset_pin(sw2_pin);
    gpio_set_direction(sw2_pin, GPIO_MODE_INPUT);
    gpio_reset_pin(sw3_pin);
    gpio_set_direction(sw3_pin, GPIO_MODE_INPUT);
    gpio_reset_pin(fan1_2_pin);
    gpio_set_direction(fan1_2_pin, GPIO_MODE_OUTPUT);
    gpio_reset_pin(fan3_4_pin);
    gpio_set_direction(fan3_4_pin, GPIO_MODE_OUTPUT);
    gpio_reset_pin(motor5_pin);
    gpio_set_direction(motor5_pin, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(i2cdev_init());
    //ESP_ERROR_CHECK(i2c_master_init());
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
     //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_0,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));


    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_handle = NULL;
    bool do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_0, &adc1_cali_handle);

    //-------------ADC2 Init---------------//
    adc_oneshot_unit_handle_t adc2_handle;
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    //-------------ADC2 Calibration Init---------------//
    adc_cali_handle_t adc2_cali_handle = NULL;
    bool do_calibration2 = example_adc_calibration_init(ADC_UNIT_2, ADC_ATTEN_DB_0, &adc2_cali_handle);

    //-------------ADC2 Config---------------//
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_7, &config));

    /*
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    
    ESP_LOGI("TESTE", "Primeiro GET");
    rest_get();
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    //post_rest_function();
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_LOGI("TESTE", "Primeiro POST");
    post_test();
    ESP_LOGI("TESTE", "Segundo GET");
    rest_get();
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_LOGI("TESTE", "Primeiro PATCH");
    patch_test();
    ESP_LOGI("TESTE", "Terceiro GET");
    rest_get();
*/

    
    //xTaskCreate(pcf_test1, "pcf_test1", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    //i2c_dev_t pcf8574;

    // Zero device descriptor
    //memset(&pcf8574, 0, sizeof(i2c_dev_t));

    // Init i2c device descriptor
    //ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, PCF8574_1_BASE_ADDR, 0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    //uint16_t port_val = 0xaa;
    //hile (1)
    //{
        // invert value
        //port_val = ~port_val;

        // write value to port
        //pcf8574_port_write(&pcf8574, port_val);
    uint8_t test = 0xaa;
    uint8_t data[2];
    float ppm_voltage;
    float ph_voltage;
    while (1)
    {
            vTaskDelay(3000/portTICK_PERIOD_MS);
            gpio_set_level(bomba1_pin, 1);
            gpio_set_level(fan3_4_pin, 1);
            gpio_set_level(fan1_2_pin, 1);
            gpio_set_level(motor5_pin, 1);
                    test = ~test;

            // write value to port
            //pcf8574_port_write(&pcf8574, port_val);
            xTaskCreate(pcf_test1, "pcf_test1", configMINIMAL_STACK_SIZE * 5, NULL, 1, NULL);
            xTaskCreate(pcf_test2, "pcf_test2", configMINIMAL_STACK_SIZE * 5, NULL, 1, NULL);
            xTaskCreate(dht_test, "dht_test", configMINIMAL_STACK_SIZE * 3, NULL, 3, NULL);
            xTaskCreate(ds18b20_test, "Water_temp_test", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
            
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw[0][0]));
            ESP_LOGI("ADC_test", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_0, adc_raw[0][0]);
            if (do_calibration1) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[0][0], &voltage[0][0]));
            ESP_LOGI("ADC_test", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC_CHANNEL_0, voltage[0][0]);
            }
            ph_voltage = voltage[0][0];
            convert_to_ph(ph_voltage);
            
            ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC_CHANNEL_7, &adc_raw[0][0]));
            ESP_LOGI("ADC_test", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_2 + 1, ADC_CHANNEL_7, adc_raw[0][0]);
            if (do_calibration2) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, adc_raw[0][0], &voltage[0][0]));
            ESP_LOGI("ADC_test", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_2 + 1, ADC_CHANNEL_7, voltage[0][0]);
            }
            ppm_voltage = voltage[0][0];
            convert_to_ppm(ppm_voltage,waterTemp);

            
            //pcf8574_register_write_byte(0x00, test);
            //pcf8574_register_read(0x00,data,1);
            //ESP_LOGI("i2c_test", "WHO_AM_I = %X", data[0]);
            vTaskDelay(3000/portTICK_PERIOD_MS);
            //ESP_LOGI("led", "blink1");
            test = ~test;
            
            gpio_set_level(bomba1_pin, 0);
            gpio_set_level(fan3_4_pin, 0);
            gpio_set_level(fan1_2_pin, 0);
            gpio_set_level(motor5_pin, 0);
                    //port_val = ~port_val;
            //pcf8574_register_write_byte(0x00, test);
        // write value to port
        //pcf8574_port_write(&pcf8574, port_val);
            //xTaskCreate(pcf_test1, "pcf_test1", configMINIMAL_STACK_SIZE * 5, NULL, 2, NULL);
            //xTaskCreate(pcf_test1, "pcf_test1", configMINIMAL_STACK_SIZE * 5, NULL, 1, NULL);
            //ESP_LOGI("led", "blink0");
            
    }

     //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1) {
        example_adc_calibration_deinit(adc1_cali_handle);
    }
    
     //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc2_handle));
    if (do_calibration2) {
        example_adc_calibration_deinit(adc2_cali_handle);
    }
    //put_test();
    //rest_get();
}



/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
