#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t * adc_chars;
static const adc_channel_t ultra_channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t ir_channel = ADC_CHANNEL_0;
static const adc_channel_t therm_channel = ADC_CHANNEL_3;
static const adc_atten_t atten = ADC_ATTEN_DB_11; // attenuation - db 11 (for full range)
static const adc_unit_t unit = ADC_UNIT_1;

static void check_efuse(){
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void app_main() {
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();
    
    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12); //12 bit-witdth
        adc1_config_channel_atten(ultra_channel, atten);
        adc1_config_channel_atten(ir_channel, atten);
        adc1_config_channel_atten(therm_channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)ultra_channel, atten);
    }
    
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
    
    char output[100];
    //Continuously sample ADC1
    while (1) {
        double volt;
        uint32_t adc_reading = 0;
        uint32_t adc_reading_US = 0;
        uint32_t adc_reading_IR = 0;
        uint32_t adc_reading_TH = 0;
        
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading_US += adc1_get_raw((adc1_channel_t)ultra_channel);
                adc_reading_IR += adc1_get_raw((adc1_channel_t)ir_channel);
                adc_reading_TH += adc1_get_raw((adc1_channel_t)therm_channel);
            }
            else {
                int raw;
                adc2_get_raw((adc2_channel_t)ultra_channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading_US /= NO_OF_SAMPLES;
        adc_reading_IR /= NO_OF_SAMPLES;
        adc_reading_TH /= NO_OF_SAMPLES;

        //Convert adc_reading to voltage in mV
        uint32_t voltage_US = esp_adc_cal_raw_to_voltage(adc_reading_US, adc_chars);
        uint32_t voltage_IR = esp_adc_cal_raw_to_voltage(adc_reading_IR, adc_chars);
        uint32_t voltage_TH = esp_adc_cal_raw_to_voltage(adc_reading_TH, adc_chars);
        
        double volt_IR = (double)voltage_IR / 1000;
        double volt_TH = (double)voltage_TH / 1000;
        double volt_US = (double)voltage_US / 1000;
        
        // THERMISTOR
        double Rx;
        Rx = (double)(50000 - (10000 * volt_TH)) / volt_TH;
        double y;
        y = (double)298.15 * log(10000 / Rx);
        double temp;
        temp = (double)(85875 + (y * 273.15)) / (3435 - y);
        
        // Rangefinder
        double distance;
        if (volt_IR > 1) {
            distance = (double)(volt_IR - 3.316) / -0.0431;
        } else if (1 > volt_IR || volt_IR < 0.5) {
            distance = (double)(volt_IR - 1.5211) / -0.0099;
        } else {
            distance = (double)(volt_IR - 1.3335) / -0.0078;
        }
        distance = (double)distance / 100;
        
        // Ultrasonic
        double vol_meter = ((double)voltage_US / 6.4) * 25.4;
        
        //outputting as in JSON format
        sprintf(output, "{\"temp\":\"%f\",\"ir\":\"%f\",\"ultrasonic\":\"%f\"}\n", temp, distance, vol_meter/1000);
        printf("%s", output);
        
        //delay to output every 2 seconds (2000ms)
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
