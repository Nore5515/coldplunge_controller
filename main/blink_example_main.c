/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/adc.h"

#define THERMISTOR_A 0.001125308852122 // Steinhart-Hart coefficient A
#define THERMISTOR_B 0.000234711863267 // Steinhart-Hart coefficient B
#define THERMISTOR_C 0.000000085663516 // Steinhart-Hart coefficient C
#define THERMISTOR_R0 10000.0          // Thermistor resistance at nominal temperature (10K Ohms)
#define ADC_RESOLUTION 4095            // 12-bit ADC resolution

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

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

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();
    adc1_config_width(ADC_WIDTH_12Bit);

    while (1)
    {
        uint16_t results = adc1_get_raw(ADC1_CHANNEL_4);
        printf("\tRAW TEMP: %d\n", results);
        printf("\tADC Voltage: %f\n", GetVoltageFromAdc(results));
        printf("\tResistance: %f\n", GetResistance(results));
        printf("\tSteinhart TEMP: %f\n", GetSteinhartValue(results));

        if (GetSteinhartValue(results) <= 5.0f)
        {
            raise_low_gpio();
        }
        else
        {
            raise_high_gpio();
        }
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        // blink_led();
        /* Toggle the LED state */
        // s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
