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
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"


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
//#include "driver/adc.h"



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
#define TEMP_SENSOR_GPIO 25
#define TEMP_SENSOR_ADDR 0x7a3c1bf649551728

//#define TDS_NUM_SAMPLES             10  //(int) Number of reading to take for an average
//#define TDS_SAMPLE_PERIOD           20  //(int) Sample period (delay between samples == sample period / number of readings)
////#define TDS_TEMPERATURE             18.0  //(float) Temperature of water (we should measure this with a sensor to get an accurate reading)
//
//#define TDS_VREF                    1.18   //(float) Voltage reference for ADC. We should measure the actual value of each ESP32

/* The examples use WiFi configuration that you can set via project configuration menu
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/

#include "driver/ledc.h"
#include "esp_err.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (127) // Set duty to 50%. ((2 ** 8) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (1000) // Frequency in Hertz. Set frequency at 5 kHz

#define SSID "Chamat"
#define PASS "987654320"

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

    char message_get[10000];
    const char c1[10]= "\"c1\"";
    const char ct[10]= "\"ct\"";
    const char c2[10]= "\"c2\"";
    const char p1[10]= "\"p1\"";
    const char p2[10]= "\"p2\"";
    const char t[10]= "\"t\"";
    const char lum[10]= "\"lum\"";
    

static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
static int s_retry_num = 0;
bool do_calibration1;
bool do_calibration2;
bool solution_control=1;
bool stop_circulation=0;
bool stop_circulation_critc=0;
bool post=0;
bool get=0;

adc_cali_handle_t adc2_cali_handle = NULL;
adc_oneshot_unit_handle_t adc2_handle;
adc_oneshot_unit_init_cfg_t init_config2 = {
    .unit_id = ADC_UNIT_2,
    //.ulp_mode = ADC_ULP_MODE_DISABLE,
};
adc_cali_handle_t adc1_cali_handle = NULL;
adc_oneshot_chan_cfg_t config1 = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_11,
};
adc_oneshot_chan_cfg_t config2 = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_11,
};
adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
};
float waterTemp = 25.0;
float airTemp = 25.0;
float target_airTemp = 25.0;
float airHumi = 55.0;
float PH = 7.5;
float PPM = 750.0;
float humi_min=75.0;
float humi_max=80.0;
float ph_min=5.5;
float ph_max=6.5;
float ppm_max=800.0;
float ppm_min=350.0;
uint8_t port1_val=0xFF;
uint8_t port2_val=0x00;
long int counter=0x00;
bool upload_app=0;
bool veg=0;
bool bloom=0;
bool veg_bloom=1;
float convert_to_ppm(float analogReading, float waterTemp);
float convert_to_ph(float averageVoltage);
void pcf1_write(uint8_t port_val);
void pcf2_write(uint8_t port_val);
uint8_t pcf1_read(void);
uint8_t pcf2_read(void);
void pcf1_write_set_pin(char pin_number);
void pcf1_write_clear_pin(char pin_number);
void pcf2_write_set_pin(char pin_number);
void pcf2_write_clear_pin(char pin_number);
bool pcf1_read_pin(char pin_number);
bool pcf2_read_pin(char pin_number);

#include <ets_sys.h>
#include <esp_idf_lib_helpers.h>

// DHT timer precision in microseconds
#define DHT_TIMER_INTERVAL 2
#define DHT_DATA_BITS 40
#define DHT_DATA_BYTES (DHT_DATA_BITS / 8)


//#include <string.h>
//#include <stdlib.h>
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "esp_log.h"
//#include "esp_system.h"
//#include "nvs_flash.h"
//#include "esp_event.h"
//#include "esp_netif.h"
//#include "protocol_examples_common.h"
#include "esp_tls.h"


#include "esp_http_client.h"

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048
//static const char *TAG = "HTTP_CLIENT";

/* Root cert for howsmyssl.com, taken from howsmyssl_com_root_cert.pem
   The PEM file was extracted from the output of this command:
   openssl s_client -showcerts -connect www.howsmyssl.com:443 </dev/null
   The CA root cert is the last cert given in the chain of certs.
   To embed it in the app binary, the PEM file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
*/
extern const char howsmyssl_com_root_cert_pem_start[] asm("_binary_howsmyssl_com_root_cert_pem_start");
extern const char howsmyssl_com_root_cert_pem_end[]   asm("_binary_howsmyssl_com_root_cert_pem_end");

extern const char postman_root_cert_pem_start[] asm("_binary_postman_root_cert_pem_start");
extern const char postman_root_cert_pem_end[]   asm("_binary_postman_root_cert_pem_end");


esp_err_t _http_event_handler(esp_http_client_event_handle_t evt)
{
    static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD("HTTP REQUEST", "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD("HTTP REQUEST", "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD("HTTP REQUEST", "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD("HTTP REQUEST", "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD("HTTP REQUEST", "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                if (evt->user_data) {
                    memcpy(evt->user_data + output_len, evt->data, evt->data_len);
                } else {
                    if (output_buffer == NULL) {
                        output_buffer = (char *) malloc(esp_http_client_get_content_length(evt->client));
                        output_len = 0;
                        if (output_buffer == NULL) {
                            ESP_LOGE("HTTP REQUEST", "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    memcpy(output_buffer + output_len, evt->data, evt->data_len);
                }
                output_len += evt->data_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD("HTTP REQUEST", "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                ESP_LOG_BUFFER_HEX("HTTP REQUEST", output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        //case HTTP_EVENT_DISCONNECTED:
        //    ESP_LOGI("HTTP REQUEST", "HTTP_EVENT_DISCONNECTED");
        //    int mbedtls_err = 0;
        //    esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
        //    if (err != 0) {
        //        ESP_LOGI("HTTP REQUEST", "Last esp error code: 0x%x", err);
        //        ESP_LOGI("HTTP REQUEST", "Last mbedtls failure: 0x%x", mbedtls_err);
        //    }
        //    if (output_buffer != NULL) {
        //        //ESP_LOG_BUFFER_HEX("HTTP REQUEST", output_buffer, output_len);
        //        free(output_buffer);
        //        output_buffer = NULL;
        //    }
        //    output_len = 0;
        //    break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD("HTTP REQUEST", "HTTP_EVENT_REDIRECT");
            esp_http_client_set_header(evt->client, "From", "user@example.com");
            esp_http_client_set_header(evt->client, "Accept", "text/html");
            esp_http_client_set_redirection(evt->client);
            break;
        default:
        break;
    }
    return ESP_OK;
}

static void http_rest_with_url(void)
{
    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};
    /**
     * NOTE: All the configuration parameters for http_client must be spefied either in URL or as host and path parameters.
     * If host and path parameters are not set, query parameter will be ignored. In such cases,
     * query parameter should be specified in URL.
     *
     * If URL as well as host and path parameters are specified, values of host and path will be considered.
     */
    esp_http_client_config_t config = {
        .url = "https://firestore.googleapis.com/v1/projects/plant-arm-project/databases/(default)/documents/SendingValuesEsp/Parameters/?key=AIzaSyCyDMEBVIO-kxZxl2F5pRgAa34TDye5ExU",
        .method = HTTP_METHOD_GET,
        .cert_pem = NULL,
        .event_handler =_http_event_handler
        //.user_data = local_response_buffer,        // Pass address of local buffer to get response
        //.disable_auto_redirect = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // GET
    esp_http_client_set_timeout_ms(client, 10000); 
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI("HTTP REQUEST", "HTTP GET Status = %d, content_length = %lld",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE("HTTP REQUEST", "HTTP GET request failed: %s", esp_err_to_name(err));
    }
    printf("estou aqui 1");
    ESP_LOG_BUFFER_HEX("HTTP REQUEST", local_response_buffer, strlen(local_response_buffer));
    printf(local_response_buffer);
    printf("estou aqui 2");
    //// POST
    //const char *post_data = "{\"field1\":\"value1\"}";
    //esp_http_client_set_url(client, "http://httpbin.org/post");
    //esp_http_client_set_method(client, HTTP_METHOD_POST);
    //esp_http_client_set_header(client, "Content-Type", "application/json");
    //esp_http_client_set_post_field(client, post_data, strlen(post_data));
    //err = esp_http_client_perform(client);
    //if (err == ESP_OK) {
    //    ESP_LOGI("HTTP REQUEST", "HTTP POST Status = %d, content_length = %lld",
    //            esp_http_client_get_status_code(client),
    //            esp_http_client_get_content_length(client));
    //} else {
    //    ESP_LOGE("HTTP REQUEST", "HTTP POST request failed: %s", esp_err_to_name(err));
    //}

    ////PUT
    //esp_http_client_set_url(client, "http://httpbin.org/put");
    //esp_http_client_set_method(client, HTTP_METHOD_PUT);
    //err = esp_http_client_perform(client);
    //if (err == ESP_OK) {
    //    ESP_LOGI("HTTP REQUEST", "HTTP PUT Status = %d, content_length = %lld",
    //            esp_http_client_get_status_code(client),
    //            esp_http_client_get_content_length(client));
    //} else {
    //    ESP_LOGE("HTTP REQUEST", "HTTP PUT request failed: %s", esp_err_to_name(err));
    //}
//
    ////PATCH
    //esp_http_client_set_url(client, "http://httpbin.org/patch");
    //esp_http_client_set_method(client, HTTP_METHOD_PATCH);
    //esp_http_client_set_post_field(client, NULL, 0);
    //err = esp_http_client_perform(client);
    //if (err == ESP_OK) {
    //    ESP_LOGI("HTTP REQUEST", "HTTP PATCH Status = %d, content_length = %lld",
    //            esp_http_client_get_status_code(client),
    //            esp_http_client_get_content_length(client));
    //} else {
    //    ESP_LOGE("HTTP REQUEST", "HTTP PATCH request failed: %s", esp_err_to_name(err));
    //}
//
    ////DELETE
    //esp_http_client_set_url(client, "http://httpbin.org/delete");
    //esp_http_client_set_method(client, HTTP_METHOD_DELETE);
    //err = esp_http_client_perform(client);
    //if (err == ESP_OK) {
    //    ESP_LOGI("HTTP REQUEST", "HTTP DELETE Status = %d, content_length = %lld",
    //            esp_http_client_get_status_code(client),
    //            esp_http_client_get_content_length(client));
    //} else {
    //    ESP_LOGE("HTTP REQUEST", "HTTP DELETE request failed: %s", esp_err_to_name(err));
    //}
//
    ////HEAD
    //esp_http_client_set_url(client, "http://httpbin.org/get");
    //esp_http_client_set_method(client, HTTP_METHOD_HEAD);
    //err = esp_http_client_perform(client);
    //if (err == ESP_OK) {
    //    ESP_LOGI("HTTP REQUEST", "HTTP HEAD Status = %d, content_length = %lld",
    //            esp_http_client_get_status_code(client),
    //            esp_http_client_get_content_length(client));
    //} else {
    //    ESP_LOGE("HTTP REQUEST", "HTTP HEAD request failed: %s", esp_err_to_name(err));
    //}

    esp_http_client_cleanup(client);
}





static void http_test_task(void *pvParameters)
{
    while(1){
    http_rest_with_url();
    
    //https_with_url();

    ESP_LOGI("HTTP REQUEST", "Finish http example");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}




/*
 *  Note:
 *  A suitable pull-up resistor should be connected to the selected GPIO line
 *
 *  __           ______          _______                              ___________________________
 *    \    A    /      \   C    /       \   DHT duration_data_low    /                           \
 *     \_______/   B    \______/    D    \__________________________/   DHT duration_data_high    \__
 *
 *
 *  Initializing communications with the DHT requires four 'phases' as follows:
 *
 *  Phase A - MCU pulls signal low for at least 18000 us
 *  Phase B - MCU allows signal to float back up and waits 20-40us for DHT to pull it low
 *  Phase C - DHT pulls signal low for ~80us
 *  Phase D - DHT lets signal float back up for ~80us
 *
 *  After this, the DHT transmits its first bit by holding the signal low for 50us
 *  and then letting it float back high for a period of time that depends on the data bit.
 *  duration_data_high is shorter than 50us for a logic '0' and longer than 50us for logic '1'.
 *
 *  There are a total of 40 data bits transmitted sequentially. These bits are read into a byte array
 *  of length 5.  The first and third bytes are humidity (%) and temperature (C), respectively.  Bytes 2 and 4
 *  are zero-filled and the fifth is a checksum such that:
 *
 *  byte_5 == (byte_1 + byte_2 + byte_3 + byte_4) & 0xFF
 *
 */

static const char *TAG = "dht";

#if HELPER_TARGET_IS_ESP32
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define PORT_ENTER_CRITICAL() portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL() portEXIT_CRITICAL(&mux)

#elif HELPER_TARGET_IS_ESP8266
#define PORT_ENTER_CRITICAL() portENTER_CRITICAL()
#define PORT_EXIT_CRITICAL() portEXIT_CRITICAL()
#endif

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#define CHECK_LOGE(x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            PORT_EXIT_CRITICAL(); \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

//#define TDS_ENABLE_GPIO     GPIO_NUM_32 //Note: Power from GPIO is 12mA, board requires 3~6mA, so enable pin powers board
#define TDS_ANALOG_GPIO     ADC_CHANNEL_7 //ADC1 is availalbe on pins 15, 34, 35 & 36

#define TDS_STABILISATION_DELAY     10  //(int) How long to wait (in seconds) after enabling sensor before taking a reading
#define TDS_NUM_SAMPLES             5  //(int) Number of reading to take for an average
#define TDS_SAMPLE_PERIOD           5  //(int) Sample period (delay between samples == sample period / number of readings)
//#define TDS_TEMPERATURE             18.0  //(float) Temperature of water (we should measure this with a sensor to get an accurate reading)
#define TDS_VREF                    2.8   //(float) Voltage reference for ADC. We should measure the actual value of each ESP32

static const char *TDS = "TDS INFO";
float sampleDelay = (TDS_SAMPLE_PERIOD / TDS_NUM_SAMPLES) * 1000;

//oid expose_vref(){
//   // Expose the ADC VREF to a GPIO so we can measure it rather than assume it is 1.1v
//   ESP_ERROR_CHECK(adc_vref_to_gpio(ADC_UNIT_1,GPIO_NUM_25));
//   ESP_LOGI(TDS, "VREF routed to ADC1, pin 25\n");
//

void config_pins(){
    ESP_LOGI(TDS, "Configure pins required for TDS sensor.");
    // Pin to power the TDS sensor
    //esp_rom_gpio_pad_select_gpio()(TDS_ENABLE_GPIO);
    //gpio_set_direction(TDS_ENABLE_GPIO, GPIO_MODE_OUTPUT);
    // Pin to read the TDS sensor analog output
    //adc_config_width(ADC_WIDTH_BIT_12);
    //adc_config_channel_atten(TDS_ANALOG_GPIO, ADC_ATTEN_DB_11);
}

//void enable_tds_sensor(){
//    ESP_LOGI(TDS, "Enabling TDS sensor & waiting 10 seconds.");
//    //gpio_set_level(TDS_ENABLE_GPIO, 1);
//    // Wait 10 seconds
//    vTaskDelay(10000 / portTICK_PERIOD_MS);
//}
//
//void disable_tds_sensor(){
//    ESP_LOGI(TDS, "Disabling TDS sensor.");
//    gpio_set_level(TDS_ENABLE_GPIO, 0);
//}

float read_tds_sensor(int numSamples, float sampleDelay){
    // Take n sensor readings every p millseconds where n is numSamples, and p is sampleDelay.
    // Return the average sample value.
    uint32_t runningSampleValue = 0;

    for(int i = 0; i < numSamples; i++) {
        // Read analogue value
        //int analogSample = adc1_get_raw(TDS_ANALOG_GPIO);
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &adc_raw[0][0]));
        ESP_LOGI("ADC_test", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_7, adc_raw[0][0]);
        int analogSample=adc_raw[0][0];
        ESP_LOGI(TDS, "Read analog value %d then sleep for %f milli seconds.", analogSample, sampleDelay);
        runningSampleValue = runningSampleValue + analogSample;
        vTaskDelay(sampleDelay / portTICK_PERIOD_MS);
        
    }
    
    float tdsAverage = runningSampleValue / TDS_NUM_SAMPLES;
    ESP_LOGI(TDS, "Calculated average = %f", tdsAverage);
    return tdsAverage;
}

float convert_to_ppm(float averageVoltage, float waterTemp){
    ESP_LOGI(TDS, "Converting an analog value to a TDS PPM value.");
    //https://www.dfrobot.com/wiki/index.php/Gravity:_Analog_TDS_Sensor_/_Meter_For_Arduino_SKU:_SEN0244#More_Documents
    //float adcCompensation = 1 + (1/3.9); // 1/3.9 (11dB) attenuation.
    //float vPerDiv = (TDS_VREF / 4096) * adcCompensation; // Calculate the volts per division using the VREF taking account of the chosen attenuation value.
    //float averageVoltage = analogReading * vPerDiv; // Convert the ADC reading into volts
    averageVoltage=(averageVoltage/1000.0)/0.3472;
    float compensation=0.78;
    float compensationCoefficient=1.0+0.02*(waterTemp-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = (averageVoltage / compensationCoefficient);  //temperature compensation
    float tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5 * compensation; //convert voltage value to tds value

    //ESP_LOGI(TDS, "Volts per division = %f", vPerDiv);
    //ESP_LOGI(TDS, "Average Voltage = %f", averageVoltage);
    //ESP_LOGI(TDS, "Temperature (currently fixed, we should measure this) = %f", waterTemp);
    //ESP_LOGI(TDS, "Compensation Coefficient = %f", compensationCoefficient);
    //ESP_LOGI(TDS, "Compensation Voltge = %f", compensationVolatge);
    ESP_LOGI(TDS, "tdsValue = %f ppm", tdsValue);
    return tdsValue;
}

void tds_task(void * pvParameters){
    ESP_LOGI(TDS, "TDS Measurement Control Task: Starting");
    while(1){
        ESP_LOGI(TDS, "TDS Measurement Control Task: Read Sensor");
        //enable_tds_sensor();
        float sensorReading = read_tds_sensor(TDS_NUM_SAMPLES, sampleDelay);
        float tdsResult = convert_to_ppm(sensorReading, waterTemp);
        printf("TDS Reading = %f ppm\n", tdsResult);
        //disable_tds_sensor();
        ESP_LOGI(TDS, "TDS Measurement Control Task: Sleeping 1 minute");
        vTaskDelay(((1000 / portTICK_PERIOD_MS)*60)*1); //delay in minutes between measurements
    }
}






/**
 * Wait specified time for pin to go to a specified state.
 * If timeout is reached and pin doesn't go to a requested state
 * false is returned.
 * The elapsed time is returned in pointer 'duration' if it is not NULL.
 */
static esp_err_t dht_await_pin_state(gpio_num_t pin, uint32_t timeout,
       int expected_pin_state, uint32_t *duration)
{
    /* XXX dht_await_pin_state() should save pin direction and restore
     * the direction before return. however, the SDK does not provide
     * gpio_get_direction().
     */
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    for (uint32_t i = 0; i < timeout; i += DHT_TIMER_INTERVAL)
    {
        // need to wait at least a single interval to prevent reading a jitter
        ets_delay_us(DHT_TIMER_INTERVAL);
        if (gpio_get_level(pin) == expected_pin_state)
        {
            if (duration)
                *duration = i;
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}


/**
 * Request data from DHT and read raw bit stream.
 * The function call should be protected from task switching.
 * Return false if error occurred.
 */
static inline esp_err_t dht_fetch_data(dht_sensor_type_t sensor_type, gpio_num_t pin, uint8_t data[DHT_DATA_BYTES])
{
    uint32_t low_duration;
    uint32_t high_duration;

    // Phase 'A' pulling signal low to initiate read sequence
    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(pin, 0);
    ets_delay_us(sensor_type == DHT_TYPE_SI7021 ? 500 : 20000);
    gpio_set_level(pin, 1);

    // Step through Phase 'B', 40us
    CHECK_LOGE(dht_await_pin_state(pin, 40, 0, NULL),
            "Initialization error, problem in phase 'B'");
    // Step through Phase 'C', 88us
    CHECK_LOGE(dht_await_pin_state(pin, 88, 1, NULL),
            "Initialization error, problem in phase 'C'");
    // Step through Phase 'D', 88us
    CHECK_LOGE(dht_await_pin_state(pin, 88, 0, NULL),
            "Initialization error, problem in phase 'D'");

    // Read in each of the 40 bits of data...
    for (int i = 0; i < DHT_DATA_BITS; i++)
    {
        CHECK_LOGE(dht_await_pin_state(pin, 65, 1, &low_duration),
                "LOW bit timeout");
        CHECK_LOGE(dht_await_pin_state(pin, 75, 0, &high_duration),
                "HIGH bit timeout");

        uint8_t b = i / 8;
        uint8_t m = i % 8;
        if (!m)
            data[b] = 0;

        data[b] |= (high_duration > low_duration) << (7 - m);
    }

    return ESP_OK;
}

/**
 * Pack two data bytes into single value and take into account sign bit.
 */
static inline int16_t dht_convert_data_2(dht_sensor_type_t sensor_type, uint8_t msb, uint8_t lsb)
{
    int16_t data;
    //const int8_t scale=256;

    //if (sensor_type == DHT_TYPE_DHT11)
    //{
    //    //data=msb*10;
    //    data = msb * 100;
    //    data = data+(lsb);
    //    printf("teste msb %d lsb: %d data %d", msb, lsb , data);
    //    data = data/10;
    //}
    //else
    //{
        data = msb;
        data <<= 8;
        data |= lsb;
        if (msb & BIT(7))
            data = -data;       // convert it to negative
    //}

    return data;
}

esp_err_t dht_read_data(dht_sensor_type_t sensor_type, gpio_num_t pin,
        int16_t *humidity, int16_t *temperature)
{
    CHECK_ARG(humidity || temperature);

    uint8_t data[DHT_DATA_BYTES] = { 0 };

    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(pin, 1);

    PORT_ENTER_CRITICAL();
    esp_err_t result = dht_fetch_data(sensor_type, pin, data);
    if (result == ESP_OK)
        PORT_EXIT_CRITICAL();

    /* restore GPIO direction because, after calling dht_fetch_data(), the
     * GPIO direction mode changes */
    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(pin, 1);

    if (result != ESP_OK)
        return result;

    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF))
    {
        ESP_LOGE(TAG, "Checksum failed, invalid data received from sensor");
        return ESP_ERR_INVALID_CRC;
    }

    if (humidity)
        *humidity = dht_convert_data_2(sensor_type, data[0], data[1]);
    if (temperature)
        *temperature = dht_convert_data_2(sensor_type, data[2], data[3]);

    ESP_LOGD(TAG, "Sensor data: humidity=%d, temp=%d", *humidity, *temperature);

    return ESP_OK;
}

esp_err_t dht_read_float_data_2(dht_sensor_type_t sensor_type, gpio_num_t pin,
        float *humidity, float *temperature)
{
    CHECK_ARG(humidity || temperature);

    int16_t i_humidity, i_temp;

    esp_err_t res = dht_read_data(sensor_type, pin, humidity ? &i_humidity : NULL, temperature ? &i_temp : NULL);
    if (res != ESP_OK)
        return res;

    if (humidity)
        *humidity = i_humidity / 10.0;
    if (temperature)
        *temperature = i_temp / 10.0;

    return ESP_OK;
}


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    
    const char *TAG = "Wifi event";
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

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_0 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = fan3_4_pin,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
        ledc_channel_config_t ledc_channel_1 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = fan1_2_pin,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
        ledc_channel_config_t ledc_channel_2 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_2,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = motor5_pin,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_0));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_1));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_2));
}


void wifi_init_sta(void)
{
    const char *TAG = "WIFI init";
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
    static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
                    if (!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                if (evt->user_data) {
                    memcpy(evt->user_data + output_len, evt->data, evt->data_len);
                } else {
                    if (output_buffer == NULL) {
                        output_buffer = (char *) malloc(esp_http_client_get_content_length(evt->client));
                        output_len = 0;
                        if (output_buffer == NULL) {
                            ESP_LOGE("HTTP REQUEST", "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    memcpy(output_buffer + output_len, evt->data, evt->data_len);
                }
                output_len += evt->data_len;
            }
        
        printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data);
        snprintf(message_get, sizeof(message_get), "HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data);
        
        break;
    case HTTP_EVENT_ERROR:
            ESP_LOGD("HTTP REQUEST", "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD("HTTP REQUEST", "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD("HTTP REQUEST", "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD("HTTP REQUEST", "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI("HTTP REQUEST", "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGI("HTTP REQUEST", "Last esp error code: 0x%x", err);
                ESP_LOGI("HTTP REQUEST", "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            if (output_buffer != NULL) {
                //ESP_LOG_BUFFER_HEX("HTTP REQUEST", output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
    default:
        break;
    }
    return ESP_OK;
}

void get_decode_counter()
{
    char aux[10000];
    char aux2[100];
    char *result;
    char* pEnd; 
    int size=10;
    int i=0;
    float lumi=0;
    long int counter_test=0;
    result=strstr(message_get, ct);
    for (i=0; i<size; i++){

        aux2[i]=*(result + (31+i));
        if (i==size) {aux2[i]=0;}
    }
    //printf(aux2);
    counter = strtol(aux2, NULL, 10);
    //counter= strtof(aux2, NULL);
   //printf("novo resultado counter: %ld \n",counter_test);
    printf("novo resultado counter: %.ld \n",counter);
}

void get_decode()
{
    char aux[1000];
    char aux2[10];
    char *result;
    char* pEnd; 
    int size=8;
    int i=0;
    float lumi=0;
    
    result=strstr(message_get, c1);
    for (i=0; i<size; i++){

        aux2[i]=*(result + (30+i));
        if (i==size) {aux2[i]=0;}
    }
    
    ppm_min= strtof(aux2, NULL);
    printf("novo resultado ppm minimo: %.2f \n",ppm_min);

        result=strstr(message_get, c2);
    for (i=0; i<size; i++){

        aux2[i]=*(result + (30+i));
        if (i==size) {aux2[i]=0;}
    }
    
    ppm_max= strtof(aux2, NULL);
    printf("novo resultado ppm maximo: %.2f \n",ppm_max);

        result=strstr(message_get, p1);
    for (i=0; i<size; i++){

        aux2[i]=*(result + (30+i));
        if (i==size) {aux2[i]=0;}
    }
    
    ph_min= strtof(aux2, NULL);
    printf("novo resultado ph minimo: %.2f \n",ph_min);

        result=strstr(message_get, p2);
    for (i=0; i<size; i++){

        aux2[i]=*(result + (30+i));
        if (i==size) {aux2[i]=0;}
    }
    
    ph_max= strtof(aux2, NULL);
    printf("novo resultado ph max: %.2f \n",ph_max);

        result=strstr(message_get, t);
    for (i=0; i<size; i++){

        aux2[i]=*(result + (29+i));
        if (i==size) {aux2[i]=0;}
    }
    
    target_airTemp= strtof(aux2, NULL);
    printf("novo resultado airTemp: %.2f \n",target_airTemp);

        result=strstr(message_get,lum);
    for (i=0; i<size; i++){

        aux2[i]=*(result + (31+i));
        if (i==size) {aux2[i]=0;}
    }
    
    lumi= strtof(aux2, NULL);
    if (lumi==0.0){
        veg=0;
        bloom=0;
        veg_bloom=0;
    }
    else if (lumi==1.0){
        veg=1;
        bloom=0;
        veg_bloom=0;
    }
    else if (lumi==2.0){
        veg=0;
        bloom=1;
        veg_bloom=0;
    }
    else if (lumi==3.0){
        veg=1;
        bloom=1;
        veg_bloom=1;
    }
    else {
        veg=veg;
        bloom=bloom;
        veg_bloom=veg_bloom;
    }
    printf("novo resultado luminosidade: %.2f \n",lumi);
    
    //result=strstr(message_get, c);
    //for (i=0; i<size; i++){
    //
    //    aux2[i]=*(result + (29+i));
    //    if (i==size) {aux2[i]=0;}
    //}
    
    //target_airTemp= strtof(aux2, NULL);
    //printf("novo resultado airTemp: %.2f \n",target_airTemp);


}



//static void rest_get()
//{
//    esp_http_client_config_t config_get_2 = {
//        .url = "https://firestore.googleapis.com/v1/projects/plant-arm-project/databases/(default)/documents/SettingLastCounter/Value/",
//        .method = HTTP_METHOD_GET,
//        .cert_pem = NULL,
//        .event_handler = client_event_get_handler};
//     
//    esp_http_client_handle_t client = esp_http_client_init(&config_get_2);
//    esp_http_client_set_timeout_ms(client, 30);   
//    esp_http_client_perform(client);
//    esp_http_client_cleanup(client);
//    vTaskDelay(pdMS_TO_TICKS(1000));
//}


static void rest_get()
{
    esp_http_client_config_t config = {
        .url = "https://firestore.googleapis.com/v1/projects/plant-arm-project/databases/(default)/documents/SendingValuesEsp/Parameters/?key=AIzaSyCyDMEBVIO-kxZxl2F5pRgAa34TDye5ExU",
        .method = HTTP_METHOD_GET,
        .cert_pem = NULL,
        .event_handler = client_event_get_handler};
     
    esp_http_client_handle_t client = esp_http_client_init(&config);
    //esp_http_client_open(client,0);
    esp_http_client_set_timeout_ms(client, 4000);   
    esp_http_client_perform(client);
    //esp_http_client_cleanup(client_get);
    //esp_http_client_cleanup(client_get);
    //esp_http_client_cleanup(client_get);
    //if (post==1){
    //char message[3000];
    ////snprintf (message, sizeof(message), "{\"Ambient Temperature\":\"%.1f??C\",\"Ambient Humidity\":\"%.1f\",\"Solution Temperature\":\"%.2f??C\", \"Solution PH\":\"%f\", \"Solution PPM\":\"%f\"}", airTemp, airHumi, waterTemp, PPM, PH);
    //snprintf (message, sizeof(message), "{'fields':{\"wt\":{'stringValue':'%.2f'},\"at\":{'stringValue':'%.2f'},\"ph\":{'stringValue':'%.2f'},\"cd\":{'stringValue':'%.2f'},\"hd\":{'stringValue':'%.2f'}, \"ct\":{'stringValue':'%ld'}}}", waterTemp, airTemp, PH, PPM, airHumi, counter);
    //const char *post_data = message;
    //const char *TAG = "POST REQUEST";
    ////esp_http_client_handle_t client = esp_http_client_init(&config_post);
    //
    //esp_http_client_set_url(client, "https://firestore.googleapis.com/v1/projects/plant-arm-project/databases/(default)/documents/posts/ReceivingValuesEsp/.json");
    //esp_http_client_set_method(client, HTTP_METHOD_POST);
    //esp_http_client_set_header(client, "Content-Type", "application/json");
    //esp_http_client_set_post_field(client, post_data, strlen(post_data));
    //esp_err_t err = esp_http_client_perform(client);
    //if (err == ESP_OK) {
    //    ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %lld",
    //            esp_http_client_get_status_code(client),
    //            esp_http_client_get_content_length(client));
    //            if (esp_http_client_get_status_code(client)==200){
    //                
    //            }
    //} else {
    //    ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    //}
    //post=0;
    //}
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    
}


static void rest_get_counter()
{
    esp_http_client_config_t config_get_2 = {
        .url = "https://firestore.googleapis.com/v1/projects/plant-arm-project/databases/(default)/documents/SettingLastCounter/Value/?key=AIzaSyCyDMEBVIO-kxZxl2F5pRgAa34TDye5ExU",
        .method = HTTP_METHOD_GET,
        .cert_pem = NULL,
        .event_handler = client_event_get_handler};
     
    esp_http_client_handle_t client = esp_http_client_init(&config_get_2);
    esp_http_client_set_timeout_ms(client, 10000);   
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
    counter++;
    const char *TAG = "POST REQUEST";
    esp_http_client_config_t config_post = {
        .url = "https://firestore.googleapis.com/v1/projects/plant-arm-project/databases/(default)/documents/posts/ReceivingValuesEsp/.json",
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .event_handler = client_event_post_handler};
        //const char *post_data = "{\"title\":\"test\"}";
    //const char *post_data = "{'fields':{\"wt\":{'stringValue':'10'}}}";
    
    char message[3000];
    //snprintf (message, sizeof(message), "{\"Ambient Temperature\":\"%.1f??C\",\"Ambient Humidity\":\"%.1f\",\"Solution Temperature\":\"%.2f??C\", \"Solution PH\":\"%f\", \"Solution PPM\":\"%f\"}", airTemp, airHumi, waterTemp, PPM, PH);
    snprintf (message, sizeof(message), "{'fields':{\"wt\":{'stringValue':'%.2f'},\"at\":{'stringValue':'%.2f'},\"ph\":{'stringValue':'%.2f'},\"cd\":{'stringValue':'%.2f'},\"hd\":{'stringValue':'%.2f'}, \"ct\":{'stringValue':'%ld'}}}", waterTemp, airTemp, PH, PPM, airHumi, counter);
    const char *post_data = message;
    
    esp_http_client_handle_t client = esp_http_client_init(&config_post);
    //esp_http_client_open(client,0);
    esp_http_client_set_url(client, "https://firestore.googleapis.com/v1/projects/plant-arm-project/databases/(default)/documents/posts/ReceivingValuesEsp/.json");
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %lld",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
                //if (esp_http_client_get_status_code(client)==200){
                //    
                //}
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    printf("estou aqui 1");
    esp_http_client_close(client);
    
    printf("estou aqui 2");
    esp_http_client_cleanup(client);
    printf("estou aqui 3");
}

    static void patch_test()
{
    //PATCH
    const char *TAG = "PATCH REQUEST";
    esp_http_client_config_t config_post = {
        .url = "https://firestore.googleapis.com/v1/projects/plant-arm-project/databases/(default)/documents/posts/ReceivingValuesEsp/.json",
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .event_handler = client_event_post_handler};
    char message[300];
    //snprintf (message, sizeof(message), "{\"Ambient Temperature\":\"%.1f??C\",\"Ambient Humidity\":\"%.1f\",\"Solution Temperature\":\"%.2f??C\", \"Solution PH\":\"%f\", \"Solution PPM\":\"%f\"}", airTemp, airHumi, waterTemp, PPM, PH);
    //snprintf (message, sizeof(message), "{\'fields\':{\"wt\":{\"stringValue\":\"%.2f\"},\"at\":{\"stringValue\":\"%.2f\"},\"ph\":{\"stringValue\":\"%.2f\"},\"cd\":{\"stringValue\":\"%.2f\"},\"ct\":{\"stringValue\":\"%.2f\"}}", waterTemp, airTemp, PH, PPM, airHumi);
    snprintf (message, sizeof(message), "{'fields':{\"wt\":{'stringValue':'%.2f'},\"at\":{'stringValue':'%.2f'},\"ph\":{'stringValue':'%.2f'},\"cd\":{'stringValue':'%.2f'},\"hd\":{'stringValue':'%.2f'}, \"ct\":{'stringValue':'0'}}", waterTemp, airTemp, PH, PPM, airHumi);

    const char *post_data = message;

    //ESP_LOGI("TESTE", "{'fields':{\"wt\":{'stringValue':'%.2f'},\"at\":{'stringValue':'%.2f'},\"ph\":{'stringValue':'%.2f'},\"cd\":{'stringValue':'%.2f'},\"ct\":{'stringValue':'%.2f'}}", waterTemp, airTemp, PH, PPM, airHumi);
    //snprintf (NULL, 0, "{\"Ambient Temperature\":\"%.1f??C\",\"Ambient Humidity\":\"%.1f\",\"Solution Temperature\":\"%.2f??C\", \"Solution PH\":\"%f\", \"Solution PPM\":\"%f\"}", airTemp, airHumi, waterTemp, PPM, PH)
    //int len = ;
    
    esp_http_client_handle_t client = esp_http_client_init(&config_post);
    esp_http_client_set_url(client, "https://firestore.googleapis.com/v1/projects/plant-arm-project/databases/(default)/documents/posts/ReceivingValuesEsp/.json");
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
    const char *TAG = "PATCH REQUEST";
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
            if (dht_read_float_data_2(SENSOR_TYPE, CONFIG_EXAMPLE_DATA_GPIO, &humidity, &temperature) == ESP_OK)
                ESP_LOGI("DHT_test:","Air Humidity: %.3f%% Air Temp: %.3fC", humidity, temperature);
            else
                ESP_LOGI("DHT_test:","Could not read data from sensor");
            vTaskDelay(pdMS_TO_TICKS(500)); //break; 
        }
    //vTaskDelete(NULL);
}

void ds18b20_test(void *pvParameter)
{
    float temperature;
    esp_err_t res;
    while (1)
    {
        res = ds18x20_measure_and_read(TEMP_SENSOR_GPIO, TEMP_SENSOR_ADDR, &temperature);
        if (res != ESP_OK)
            ESP_LOGE("Water_temp_test", "Could not read from sensor %08" PRIx32 "%08" PRIx32 ": %d (%s)",
                    (uint32_t)(TEMP_SENSOR_ADDR >> 32), (uint32_t)TEMP_SENSOR_ADDR, res, esp_err_to_name(res));
        else
            ESP_LOGI("Water_temp_test", "Water Temp: %.2f??C", temperature);
        //break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    //vTaskDelete(NULL);
}

void humi_control(void *pvParameter)
{
    //float airHumi = 30;
    while (1)
    {
       
        if (airHumi<=humi_min){
            pcf2_write_set_pin(2);
            vTaskDelay(pdMS_TO_TICKS(8000));
            pcf2_write_clear_pin(2);
        }     
        else if (airHumi>=humi_max)
            pcf2_write_clear_pin(2);
        else 
            pcf2_write_clear_pin(2);     
        //break;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    //vTaskDelete(NULL);
}

void ph_control(void *pvParameter)
{
    while (1)
    {
       
        if (solution_control==1){
            if (PH<=ph_min){
                pcf2_write_set_pin(6);  //bomb5  
                vTaskDelay(pdMS_TO_TICKS(2000));
                pcf2_write_clear_pin(6);  //bomb5
            }        
            else if (PH>=ph_max){
                pcf2_write_set_pin(3);  //bomb4  
                vTaskDelay(pdMS_TO_TICKS(2000));
                pcf2_write_clear_pin(3);  //bomb4    
            }
            else {
                if (stop_circulation==0 && stop_circulation_critc)
                {
                
                    gpio_set_level(bomba1_pin, 1);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                }
            }
        }
        //break;
        vTaskDelay(pdMS_TO_TICKS(30000));
        
    }
    //vTaskDelete(NULL);
}

void ppm_control(void *pvParameter)
{
    while (1)
    {
        

        if (solution_control==1){    
            if (PPM<=ppm_min){
                pcf2_write_set_pin(5);  //bomb3 
                vTaskDelay(pdMS_TO_TICKS(2000));
                pcf2_write_clear_pin(5);  //bomb3 
            }        
            else if (PPM>=ppm_max){
                pcf2_write_set_pin(4);  //bomb2 
                vTaskDelay(pdMS_TO_TICKS(2000));
                pcf2_write_clear_pin(4);  //bomb2    
            }
            else{ 
                if (stop_circulation==0 && stop_circulation_critc==0){
                    gpio_set_level(bomba1_pin, 1);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    //ESP_LOGI("ppm_control", "estou aqui");
                }
            }
        }
        //break;
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
    //vTaskDelete(NULL);
}


void light_control(void *pvParameter)
{
    while (1)
    {
        
        if (veg_bloom==1){
            veg=1;
            bloom=1;
        }
        else{
            veg=veg;
            bloom=bloom;
        }
        if (veg==1){
            // ESP_LOGI("luz_test", "to no veg =1 ");
            pcf2_write_clear_pin(7);  //Ilu1 //inverted logic because of board logic
            vTaskDelay(pdMS_TO_TICKS(100));
           
        }
        else {       
            //ESP_LOGI("luz_test", "to no veg =0 ");
            pcf2_write_set_pin(7);  //Ilu1 //inverted logic because of board logic
            vTaskDelay(pdMS_TO_TICKS(100));
            
        }
        if (bloom==1){
            //ESP_LOGI("luz_test", "to no bloom =1 ");
            pcf2_write_clear_pin(1);  //Ilu2 //inverted logic because of board logic
            vTaskDelay(pdMS_TO_TICKS(100));
            
        }
        else{ 
            //ESP_LOGI("luz_test", "to no bloom e veg  =0 ");
            pcf2_write_set_pin(1);  //Ilu2 //inverted logic because of board logic
            vTaskDelay(pdMS_TO_TICKS(100)); 
            
        }
        vTaskDelay(pdMS_TO_TICKS(2000));  
        //break;
    }
    //vTaskDelete(NULL);
}


void get_sensors(void *pvParameter)
{

    //float temperature;
    float ppm_voltage;
    float ph_voltage;
    esp_err_t res;
    while (1)
    {
        //read water temp
        res = ds18x20_measure_and_read(TEMP_SENSOR_GPIO, TEMP_SENSOR_ADDR, &waterTemp);
        if (res != ESP_OK)
            ESP_LOGE("Water_temp_test", "Could not read from sensor %08" PRIx32 "%08" PRIx32 ": %d (%s)",
                    (uint32_t)(TEMP_SENSOR_ADDR >> 32), (uint32_t)TEMP_SENSOR_ADDR, res, esp_err_to_name(res));
        else
            ESP_LOGI("Water_temp_test", "Water Temp: %.2f??C", waterTemp);
        
        //read air temp air humidity
        //res = dht_read_float_data(SENSOR_TYPE, CONFIG_EXAMPLE_DATA_GPIO, &airHumi, &airTemp);
        //if (res != ESP_OK)
        //    ESP_LOGI("DHT_test:","Could not read data from sensor");
        //else
        //    ESP_LOGI("DHT_test:","Air Humidity: %f Air Temp: %fC", airHumi, airTemp);
        //read ph
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw[0][0]));
        ESP_LOGI("ADC_test", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_0, adc_raw[0][0]);
        if (do_calibration1) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[0][0], &voltage[0][0]));
        ESP_LOGI("ADC_test", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC_CHANNEL_0, voltage[0][0]);
        }
        ph_voltage = voltage[0][0];
        PH=convert_to_ph(ph_voltage);
        //read ppm
        //ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC_CHANNEL_7, &adc_raw[0][0]));
        //ESP_LOGI("ADC_test", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_2 + 1, ADC_CHANNEL_7, adc_raw[0][0]);
        //if (do_calibration2) {
        //ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, adc_raw[0][0], &voltage[0][0]));
        //ESP_LOGI("ADC_test", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_2 + 1, ADC_CHANNEL_7, voltage[0][0]);
        //}
        
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &adc_raw[0][0]));
        ESP_LOGI("ADC_test", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_7, adc_raw[0][0]);
        if (do_calibration1) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[0][0], &voltage[0][0]));
        ESP_LOGI("ADC_test", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC_CHANNEL_7, voltage[0][0]);
        }
        
        ppm_voltage = voltage[0][0]-43.0;
        PPM=convert_to_ppm(ppm_voltage,waterTemp);
        
        //float sensorReading = read_tds_sensor(TDS_NUM_SAMPLES, sampleDelay);
        //PPM = convert_to_ppm(sensorReading, waterTemp);


        upload_app=1;
        //break;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    //vTaskDelete(NULL);
}

void app_post(void *pvParameter)
{

    while(1)
    {
        if (get==0){
        post=1;
        post_test();
        upload_app=0;
        post=0;
        //esp_http_client_cleanup();
        vTaskDelay(pdMS_TO_TICKS(1000*15*1));
        }
        else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void app_get(void *pvParameter)
{
    while(1)
    {
        if (post==0){
        get=1;
        rest_get();
        get_decode();
        get=0;
        //esp_http_client_cleanup();
        vTaskDelay(pdMS_TO_TICKS(2000));
        }
        else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void counter_get(void *pvParameter)
{
    while(1)
    {
        //if (post==0){
        //get=1;
        rest_get_counter();
        get_decode_counter();
        //get_decode();
        //get=0;
        break;
        //vTaskDelay(pdMS_TO_TICKS(1000));
        //}
        //else {
            //vTaskDelay(pdMS_TO_TICKS(1000));
        //}
    }
    vTaskDelete(NULL);
}

void critical_levels(void *pvParameter)
{
    while(1)
    {
         if (PH<=(ph_min-2.0))
        {
                // Stop Solution Circulation due PH too LOW
                gpio_set_level(bomba1_pin, 0);
                ESP_LOGI("ph_control", "Stop Solution Circulation due PH too High: %.2f. Limit is %.2f", PH, ph_min);
                stop_circulation_critc=1;
        }
        else if (PH>=(ph_max+2.0))
        {
                // Stop Solution Circulation due PH too HIGH
                gpio_set_level(bomba1_pin, 0);
                ESP_LOGI("ph_control", "Stop Solution Circulation due PH too High: %.2f. Limit is %.2f", PH, ph_max);
                stop_circulation_critc=1;
        }
        else if (PPM<=(ppm_min-200.0))
        {
                // Stop Solution Circulation due PPM too LOW
                gpio_set_level(bomba1_pin, 0);
                ESP_LOGI("ppm_control", "Stop Solution Circulation due PPM too LOW: %.2f. Limit is %.2f", PPM, ppm_min);
                stop_circulation_critc=1;
        }
        else if (PPM>=(ppm_max+200.0))
        {
                // Stop Solution Circulation due PPM too HIGH
                gpio_set_level(bomba1_pin, 0);
                ESP_LOGI("ppm_control", "Stop Solution Circulation due PPM too High: %.2f. Limit is %.2f", PPM, ppm_max);
                stop_circulation_critc=1;
        }
        else {stop_circulation_critc=0; }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
       

void solution_level_control(void *pvParameter)
{
    bool low_level_main=0;
    bool high_level_main=0;
    bool low_level_water=0;
    bool low_level_nutrients=0;
    bool low_level_ph_low=0;
    bool low_level_ph_high=0;
    bool teste=0;
    uint8_t read_port=0;
    uint8_t read_port2=0;
    bool water_bomb=0;
    i2c_dev_t pcf8574;
    while(1)
    {
        
        read_port=pcf1_read();
        read_port2=pcf2_read();
        water_bomb = (read_port2 >> 4) & 1;
        low_level_main = (read_port >> 6) & 1;          // bomba 1
        high_level_main = (read_port >> 7) & 1;         // bomba 1
        low_level_water = (read_port >> 4) & 1;         // bomba 2
        low_level_nutrients = (read_port >> 3) & 1;     // bomba 3
        low_level_ph_low = (read_port >> 5) & 1;        // bomba 4
        low_level_ph_high = (read_port >> 2) & 1;       // bomba 5
        teste = (read_port >> 7) & 1;
        //ESP_LOGI("teste_read_pin", "PIN port: %d\n", read_port);
        
        //low_level_main=pcf1_read_pin(0);
        //high_level_main=pcf1_read_pin(1);
        //low_level_water=pcf1_read_pin(2);
        //low_level_nutrients=pcf1_read_pin(3);
        //low_level_ph_low=pcf1_read_pin(4);  
        //low_level_ph_high=pcf1_read_pin(5);
        
        /*ESP_LOGI("teste_read_pin", "Nivel alto principal: %d\n", high_level_main);
        ESP_LOGI("teste_read_pin", "Nivel baixo principal: %d\n", low_level_main);
        ESP_LOGI("teste_read_pin", "Nivel baixo bomba2: %d\n", low_level_water);
        ESP_LOGI("teste_read_pin", "Nivel baixo bomba3: %d\n", low_level_nutrients);
        ESP_LOGI("teste_read_pin", "Nivel baixo bomba4: %d\n", low_level_ph_low);
        ESP_LOGI("teste_read_pin", "Nivel baixo bomba5: %d\n", low_level_ph_high);
        ESP_LOGI("teste_read_pin", "water bomba: %d\n", water_bomb);
        */
        //ESP_LOGI("teste_read_pin", "water bomba: %d\n", water_bomb);
        
        if (low_level_main){ //if low level of main solution is not achieved
            // Stop Solution Circulation
            gpio_set_level(bomba1_pin, 0);
            solution_control=1;
            ESP_LOGI("level_control", "Stop Solution Circulation due solution level too LOW. Please fill the main reservatory with water until the minimum level");
            //pcf2_write_set_pin(4);  //bomb2 
            //vTaskDelay(pdMS_TO_TICKS(10000));
            //pcf2_write_clear_pin(4);  //bomb2 
            
        } 
        else {
            if (stop_circulation==0 && stop_circulation_critc==0){
                gpio_set_level(bomba1_pin, 1); 
            }
            if (water_bomb==1){
            //    pcf2_write_set_pin(4);
            }
            else {
                if ((!(PPM>=ppm_max)) && (solution_control)){
                    //pcf2_write_clear_pin(4);
                    //ESP_LOGI("level_control", "testeee");
                }    
                }
        } 
        
        if (!high_level_main){ //if high level of main solution is achieved
            // Stop Solution Control
            solution_control=0;
            ESP_LOGI("level_control", "Stop Solution PH and PPM Control due solution level too High. Please remove solution excess manually.");
        }
        else if (low_level_water || low_level_nutrients || low_level_ph_low || low_level_ph_high ) { //if low level of any secondary solution is achieved
            // Stop Solution Control
            solution_control=0;
            ESP_LOGI("level_control", "Stop Solution PH and PPM Control due lack of secondary solution. Please check the reservatories manually.");
        }
        else {
                solution_control=1;
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}


void pcf1_write_set_pin(char pin_number)
{
    port1_val |= 1 << pin_number;
    pcf1_write(port1_val);
}

void pcf2_write_set_pin(char pin_number)
{
    port2_val |= 1 << pin_number;
    pcf2_write(port2_val);
}

void pcf1_write_clear_pin(char pin_number)
{
    port1_val &= ~(1 << pin_number);
    pcf1_write(port1_val);
}

void pcf2_write_clear_pin(char pin_number)
{
    port2_val &= ~(1 << pin_number);
    pcf2_write(port2_val);
}

bool pcf1_read_pin(char pin_number)
{
    bool result=0;
    uint8_t port1_val_read=0;
    port1_val_read=pcf1_read();
    result = (port1_val_read >> pin_number) & 1;
    return result;
}

bool pcf2_read_pin(char pin_number)
{
    bool result=0;
    uint8_t port2_val_read=0;
    port2_val_read=pcf2_read();
    result = (port2_val_read >> pin_number) & 1;
    return result;
}


void pcf1_write(uint8_t port_val)
{
    i2c_dev_t pcf8574;

    // Zero device descriptor
    memset(&pcf8574, 0, sizeof(i2c_dev_t));

    // Init i2c device descriptor
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, PCF8574_1_BASE_ADDR, 0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    pcf8574_port_write(&pcf8574, port_val);
}

uint8_t pcf1_read(void)
{
    i2c_dev_t pcf8574;
    uint8_t port_val;
    // Zero device descriptor
    memset(&pcf8574, 0, sizeof(i2c_dev_t));

    // Init i2c device descriptor
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, PCF8574_1_BASE_ADDR, 0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    pcf8574_port_read(&pcf8574, &port_val);
    return port_val;
}

void pcf2_write(uint8_t port_val)
{
    i2c_dev_t pcf8574;

    // Zero device descriptor
    memset(&pcf8574, 0, sizeof(i2c_dev_t));

    // Init i2c device descriptor
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, PCF8574_2_BASE_ADDR, 0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    pcf8574_port_write(&pcf8574, port_val);

}

uint8_t pcf2_read(void)
{
    i2c_dev_t pcf8574;
    uint8_t port_val;
    // Zero device descriptor
    memset(&pcf8574, 0, sizeof(i2c_dev_t));

    // Init i2c device descriptor
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, PCF8574_2_BASE_ADDR, 0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    pcf8574_port_read(&pcf8574, &port_val);
    return port_val;
}

void pcf_test2(void *pvParameters)
{
    i2c_dev_t pcf8574;

    // Zero device descriptor
    memset(&pcf8574, 0, sizeof(i2c_dev_t));

    // Init i2c device descriptor
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, PCF8574_2_BASE_ADDR, 0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));

    // Do some blinking
    uint8_t port_val = 0xa;
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

void pcf_test1(void *pvParameters)
{
    i2c_dev_t pcf8574;

    // Zero device descriptor
    memset(&pcf8574, 0, sizeof(i2c_dev_t));

    // Init i2c device descriptor
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, PCF8574_1_BASE_ADDR, 0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));

    // Do some blinking
    uint8_t port_val = 0xa;
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

//float convert_to_ppm(float averageVoltage, float waterTemp){
//    ESP_LOGI("TDS", "Converting an analog value to a TDS PPM value.");
//    //https://www.dfrobot.com/wiki/index.php/Gravity:_Analog_TDS_Sensor_/_Meter_For_Arduino_SKU:_SEN0244#More_Documents
//    //float adcCompensation = 1 + (1/3.9); // 1/3.9 (11dB) attenuation.
//    //float vPerDiv = (TDS_VREF / 4096) * adcCompensation; // Calculate the volts per division using the VREF taking account of the chosen attenuation value.
//    //float averageVoltage = analogReading * vPerDiv; // Convert the ADC reading into volts
//    float offset=47.789410;
//    float compensationCoefficient=1.0+0.02*(waterTemp-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
//    //averageVoltage=averageVoltage/(0.3472*0.9210);
//    float compensationVolatge = (averageVoltage / compensationCoefficient)*((0.9210))/1000;  //temperature compensation
//    float tdsValue = ((((133.42 * compensationVolatge * compensationVolatge * compensationVolatge) - (255.86 * compensationVolatge * compensationVolatge) + (857.39 * compensationVolatge)) * 0.5)-offset)*3.8; //convert voltage value to tds value
//
//    //ESP_LOGI("TDS", "Volts per division = %f", vPerDiv);
//    //ESP_LOGI("TDS", "Average Voltage = %f", averageVoltage);
//    //ESP_LOGI("TDS", "Temperature (currently fixed, we should measure this) = %f", waterTemp);
//    //ESP_LOGI("TDS", "Compensation Coefficient = %f", compensationCoefficient);
//    //ESP_LOGI("TDS", "Compensation Voltge = %f", compensationVolatge);
//    ESP_LOGI("TDS", "tdsValue = %f ppm", tdsValue);
//    return tdsValue;
//}

float convert_to_ph(float averageVoltage){
    ESP_LOGI("PH", "Converting an analog value to a PH value.");
    //https://www.dfrobot.com/wiki/index.php/Gravity:_Analog_TDS_Sensor_/_Meter_For_Arduino_SKU:_SEN0244#More_Documents
    //float adcCompensation = 1 + (1/3.9); // 1/3.9 (11dB) attenuation.
    //float vPerDiv = (TDS_VREF / 4096) * adcCompensation; // Calculate the volts per division using the VREF taking account of the chosen attenuation value.
    //float averageVoltage = analogReading * vPerDiv; // Convert the ADC reading into volts
    float compensationCoefficient=0.8986;    // compensation 
    float phValue = (((compensationCoefficient * 2.8 * averageVoltage)/(0.4722*0.9210))/1000);

    ESP_LOGI("PH", "phValue = %f", phValue);
    return phValue;
}

double clamp(double value, double min, double max){
	if(value > max){ return max;}
	if(value < min){ return min;}
	return value;
}

uint64_t lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp=200, ki=0, kd=0;
double omax=256;
double omin=-256;
double iterm;
void Compute()
{
    esp_err_t res;
    /*How long since we last calculated*/
    res = dht_read_float_data_2(SENSOR_TYPE, CONFIG_EXAMPLE_DATA_GPIO, &airHumi, &airTemp);
    if (res != ESP_OK)
        ESP_LOGI("DHT_test:","Could not read data from sensor");
    else
    ESP_LOGI("DHT_test:","Air Humidity: %.2f Air Temp: %.2fC", airHumi, airTemp);
    ESP_LOGI("DHT_test:"," Air Temp Target: %.2fC", target_airTemp);
    Input= airTemp;
    Setpoint = target_airTemp;
    
    uint64_t now = esp_timer_get_time();
    now=now/1000;
    double timeChange = (double)(now - lastTime);
    
    /*Compute all the working error variables*/
    double error = Setpoint - Input;
    errSum += (error * timeChange);
    double dErr = (error - lastErr) / timeChange;
    /*Compute PID Output*/
    iterm=ki * errSum;
    if (iterm > omax){
		iterm = omax; 
    }
	else if (iterm < omin){
		iterm = omin; 
    }
    Output = (kp * error) + iterm + (kd * dErr);
    // Apply limit to output value
	
    if (Output > omax){
		Output = omax; 
    }
	else if (Output < omin){
		Output= omin;  
    }
    /*Remember some variables for next time*/
    if (error>0)
    {Output=0;}
    lastErr = error;
    lastTime = now;
    Output=-Output;
    Output= omax-Output;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, Output));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, Output));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, Output));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2));
    ESP_LOGI("PID", "Duty = %lf", Output);
    //ESP_LOGI("PID", "errorsum = %lf", errSum);
    ESP_LOGI("PID", "error = %lf", error);

}

void SetTunings(double Kp, double Ki, double Kd)
{
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

void pid_update(void *pvParameter)
{

    while(1)
    {
        Compute();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void wifi_start(void *pvParameter)
{
    ESP_LOGI("wifi station", "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    while(1)
    {
        
        break;
    }
    vTaskDelete(NULL);
}


void app_main(void)
{
    //Initialize NVS
    gpio_reset_pin(bomba1_pin);
    gpio_set_direction(bomba1_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(bomba1_pin, 0);
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
    gpio_pullup_dis(26);
    //ESP_ERROR_CHECK(i2c_master_init());
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    
    //-------------ADC1 Init---------------//
    //adc_oneshot_unit_handle_t adc1_handle;
    //adc_oneshot_unit_init_cfg_t init_config1 = {
    //    .unit_id = ADC_UNIT_1,
    //};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    //adc_oneshot_chan_cfg_t config = {
    //    .bitwidth = ADC_BITWIDTH_DEFAULT,
    //    .atten = ADC_ATTEN_DB_0,
    //};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config1));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config1));

    //-------------ADC1 Calibration Init---------------//
    //adc_cali_handle_t adc1_cali_handle = NULL;
    do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &adc1_cali_handle);

    config_pins();
    //-------------ADC2 Init---------------//
    //adc_oneshot_unit_handle_t adc2_handle;
    //adc_oneshot_unit_init_cfg_t init_config2 = {
    //    .unit_id = ADC_UNIT_2,
    //    .ulp_mode = ADC_ULP_MODE_DISABLE,
    //};
    //ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    //-------------ADC2 Calibration Init---------------//
    //adc_cali_handle_t adc2_cali_handle = NULL;
    //do_calibration2 = example_adc_calibration_init(ADC_UNIT_2, ADC_ATTEN_DB_11, &adc2_cali_handle);

    //-------------ADC2 Config---------------//
    //ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_7, &config2));

    
    xTaskCreatePinnedToCore(wifi_start, "wifi", configMINIMAL_STACK_SIZE * 5, NULL, 0, NULL,1);
    
    //ESP_LOGI("TESTE", "Primeiro GET");
    //rest_get();
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    //post_rest_function();
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    //ESP_LOGI("TESTE", "Primeiro POST");
    //post_test();
    //ESP_LOGI("TESTE", "Segundo GET");
    //rest_get();
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    //ESP_LOGI("TESTE", "Primeiro PATCH");
    
    //ESP_LOGI("TESTE", "Terceiro GET");
    //rest_get();

    
    
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
    
    //xTaskCreatePinnedToCore(dht_test, "dht_test", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL,1);
    //xTaskCreatePinnedToCore(ds18b20_test, "Water_temp_test", configMINIMAL_STACK_SIZE * 4, NULL, 0, NULL,1);
    //rest_get_counter();

    uint8_t test = 0xaa;
    uint8_t data[2];
    float ppm_voltage;
    float ph_voltage;
    float temperature, humidity;
    
    pcf1_write(port1_val);
    pcf2_write(port2_val);
    
    dht_read_float_data_2(SENSOR_TYPE, CONFIG_EXAMPLE_DATA_GPIO, &humidity, &temperature);
    vTaskDelay(100/portTICK_PERIOD_MS);
    
    vTaskDelay(5000/portTICK_PERIOD_MS);
    // Start System Operation
    gpio_set_level(bomba1_pin, 1);
    // Set the LEDC peripheral configuration
    example_ledc_init();
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, LEDC_DUTY));
    // Update duty to apply the new value
    //ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2));
    rest_get_counter();
    get_decode_counter();
    //xTaskCreatePinnedToCore(counter_get, "counter_get", configMINIMAL_STACK_SIZE * 10, NULL, 1, NULL,1);
    
    xTaskCreatePinnedToCore(get_sensors, "get_sensors", configMINIMAL_STACK_SIZE * 5, NULL, 3, NULL,0);
    
    xTaskCreatePinnedToCore(app_post, "app_update", configMINIMAL_STACK_SIZE * 5, NULL, 2, NULL,0);

    xTaskCreatePinnedToCore(app_get, "app_get", configMINIMAL_STACK_SIZE * 5, NULL, 0, NULL,0);
    //post=0;
    
    //vTaskDelay(1000/portTICK_PERIOD_MS);
   // xTaskCreatePinnedToCore(http_test_task, "http_task", configMINIMAL_STACK_SIZE * 5, NULL, 0, NULL,0);
    


    

    
    
    
    xTaskCreatePinnedToCore(pid_update, "pid_update", configMINIMAL_STACK_SIZE * 5, NULL, 0, NULL,1);
    xTaskCreatePinnedToCore(humi_control, "humi_control", configMINIMAL_STACK_SIZE * 5, NULL, 2, NULL,1);
    xTaskCreatePinnedToCore(ph_control, "ph_control", configMINIMAL_STACK_SIZE * 5, NULL, 1, NULL,1);
    xTaskCreatePinnedToCore(ppm_control, "ppm_control", configMINIMAL_STACK_SIZE * 5, NULL, 1, NULL,1);
    xTaskCreatePinnedToCore(light_control, "light_control", configMINIMAL_STACK_SIZE * 5, NULL, 3, NULL,1);
    xTaskCreatePinnedToCore(solution_level_control, "solution_level_control", configMINIMAL_STACK_SIZE * 5, NULL, 1, NULL,0);
    xTaskCreatePinnedToCore(critical_levels, "solution_level_control_critical_levels", configMINIMAL_STACK_SIZE * 5, NULL, 1, NULL,0);

    
    
    
    //xTaskCreate(pcf_test1, "pcf_test1", configMINIMAL_STACK_SIZE * 5, NULL, 1, NULL);
    //xTaskCreate(pcf_test2, "pcf_test2", configMINIMAL_STACK_SIZE * 5, NULL, 1, NULL);
    

    while (1)
    {
            //vTaskDelay(500/portTICK_PERIOD_MS);
            //gpio_set_level(bomba1_pin, 1);
            //gpio_set_level(fan3_4_pin, 1);
            //gpio_set_level(fan1_2_pin, 1);
            //gpio_set_level(motor5_pin, 1);
                    //test = ~test;

            // write value to port
            //pcf8574_port_write(&pcf8574, port_val);
            
            //dht_read_float_data_2(SENSOR_TYPE, CONFIG_EXAMPLE_DATA_GPIO, &humidity, &temperature);
            //ESP_LOGI("DHT_test:","Air Humidity: %.2f%% Air Temp: %.2fC", humidity, temperature);
           //patch_test();
            
            
            //ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw[0][0]));
            //ESP_LOGI("ADC_test", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_0, adc_raw[0][0]);
            //if (do_calibration1) {
            //ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[0][0], &voltage[0][0]));
            //ESP_LOGI("ADC_test", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC_CHANNEL_0, voltage[0][0]);
            //}
            //ph_voltage = voltage[0][0];
            //convert_to_ph(ph_voltage);
            //
            //ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC_CHANNEL_7, &adc_raw[0][0]));
            //ESP_LOGI("ADC_test", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_2 + 1, ADC_CHANNEL_7, adc_raw[0][0]);
            //if (do_calibration2) {
            //ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, adc_raw[0][0], &voltage[0][0]));
            //ESP_LOGI("ADC_test", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_2 + 1, ADC_CHANNEL_7, voltage[0][0]);
            //}
            //ppm_voltage = voltage[0][0];
            //convert_to_ppm(ppm_voltage,waterTemp);

            
            //pcf8574_register_write_byte(0x00, test);
            //pcf8574_register_read(0x00,data,1);
            //ESP_LOGI("i2c_test", "WHO_AM_I = %X", data[0]);
            //patch_test();
            //post_test();
            //target_airTemp=24.00;

            vTaskDelay(2000/portTICK_PERIOD_MS);
            //ESP_LOGI("led", "blink1");
            //test = ~test;
            //target_airTemp=23.00;
            //vTaskDelay(12000/portTICK_PERIOD_MS);
            //target_airTemp=26.8;
            //gpio_set_level(bomba1_pin, 0);
            //gpio_set_level(fan3_4_pin, 0);
            //gpio_set_level(fan1_2_pin, 0);
            //gpio_set_level(motor5_pin, 0);
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
    //ESP_ERROR_CHECK(adc_oneshot_del_unit(adc2_handle));
    //if (do_calibration2) {
    //    example_adc_calibration_deinit(adc2_cali_handle);
    //}
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
        ESP_LOGI("ADC Calibration", "calibration scheme version is %s", "Curve Fitting");
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
        ESP_LOGI("ADC Calibration", "calibration scheme version is %s", "Line Fitting");
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
        ESP_LOGI("ADC Calibration", "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW("ADC Calibration", "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE("ADC Calibration", "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI("ADC Calibration", "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI("ADC Calibration", "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
