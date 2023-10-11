/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/adc.h"

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define SDA_PIN 16
#define SCL_PIN 17

#define THERMISTOR_A 0.001125308852122 // Steinhart-Hart coefficient A
#define THERMISTOR_B 0.000234711863267 // Steinhart-Hart coefficient B
#define THERMISTOR_C 0.000000085663516 // Steinhart-Hart coefficient C
#define THERMISTOR_R0 10000.0          // Thermistor resistance at nominal temperature (10K Ohms)
#define ADC_RESOLUTION 4095            // 12-bit ADC resolution

static const char *TAG = "COLD-PLUNGE";

#define TICK_MILISECONDS 250

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 18

static uint8_t s_led_state = 0;

float steinTemp = -1.0f;

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void raise_high_gpio(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, 1);
}

static void raise_low_gpio(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, 0);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

float GetVoltageFromAdc(uint16_t adc)
{
    return (((float)adc / 4095.0f) * 3.3f);
}

float GetResistance(uint16_t adcValue)
{
    float measuredResistance;
    if (adcValue == 0)
    {
        return -1;
    }
    measuredResistance = 10000 * ((3.3 / GetVoltageFromAdc(adcValue)) - 1);
    return measuredResistance;
}

float SteinhartCalc(float resistance)
{
    // float R = (float)ADC_RESOLUTION / resistance;
    // float lnR = log(R);
    float lnR = log(resistance);
    // lnR = 0.0f;
    float temperature = 1.0 / (THERMISTOR_A + THERMISTOR_B * lnR + THERMISTOR_C * lnR * lnR * lnR);
    temperature -= 273.15; // Convert to Celsius
    // temperature = (temperature * 9.0) / 5.0 + 32.0; // convert to F
    return temperature;
}

float GetSteinhartValue(uint16_t adcValue)
{
    return SteinhartCalc(GetResistance(adcValue));
}

void floatToByteArray(float value, uint8_t *byteArray)
{
    // Assuming a little-endian architecture
    uint32_t intValue = *((uint32_t *)&value);

    byteArray[0] = (uint8_t)(intValue & 0xFF);
    byteArray[1] = (uint8_t)((intValue >> 8) & 0xFF);
    byteArray[2] = (uint8_t)((intValue >> 16) & 0xFF);
    byteArray[3] = (uint8_t)((intValue >> 24) & 0xFF);
}

void i2cTask(void *ignore)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.scl_io_num = SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, &conf);

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    uint8_t byteArray[4];

    while (1)
    {
        floatToByteArray(steinTemp, byteArray);

        esp_err_t res;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (0x12 << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
        i2c_master_write(cmd, byteArray, 4, 1);
        i2c_master_stop(cmd);

        printf("Byte array: ");
        for (int i = 0; i < 4; i++)
        {
            printf("%02X ", byteArray[i]);
        }
        printf("\n");

        res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
        printf("Sending Data!\n");
        i2c_cmd_link_delete(cmd);
        vTaskDelay(pdMS_TO_TICKS(TICK_MILISECONDS));
    }
}

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();
    adc1_config_width(ADC_WIDTH_12Bit);

    xTaskCreatePinnedToCore(i2cTask, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    while (1)
    {
        uint16_t results = adc1_get_raw(ADC1_CHANNEL_4);
        steinTemp = GetSteinhartValue(results);
        printf("\tRAW DATA: %d\n", results);
        printf("\tADC Voltage: %f\n", GetVoltageFromAdc(results));
        printf("\tResistance: %f\n", GetResistance(results));
        printf("\tSteinhart TEMP: %f\n", steinTemp);

        if (GetSteinhartValue(results) <= 5.0f)
        {
            raise_low_gpio();
        }
        else
        {
            raise_high_gpio();
        }
        // ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        // blink_led();
        /* Toggle the LED state */
        // s_led_state = !s_led_state;
        vTaskDelay(TICK_MILISECONDS / portTICK_PERIOD_MS);
    }
}
