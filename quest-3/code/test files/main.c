#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include "driver/i2c.h"

#define DEFAULT_VREF    1100
#define NO_OF_SAMPLES   64

static esp_adc_cal_characteristics_t * adc_chars;
static const adc_channel_t battery_channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t therm_channel = ADC_CHANNEL_3;
static const adc_atten_t atten = ADC_ATTEN_DB_11; // attenuation - db 11 (for full range)
static const adc_unit_t unit = ADC_UNIT_1;

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    }
    else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    }
    else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    }
    else {
        printf("Characterized using Default Vref\n");
    }
}

static void thermistor()
{
    check_efuse();
    
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(therm_channel, atten);
    }
    else {
        adc2_config_channel_atten((adc2_channel_t)therm_channel, atten);
    }
    
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
    
     char output[100];
        while (1) {
            //double volt;
            uint32_t adc_reading_TH = 0;
            //uint32_t adc_reading = 0;
            for (int i = 0; i < NO_OF_SAMPLES; i++) {
                if (unit == ADC_UNIT_1) {
                    adc_reading_TH += adc1_get_raw((adc1_channel_t)therm_channel);
                }
                else {
                    int raw;
                    adc2_get_raw((adc2_channel_t)therm_channel, ADC_WIDTH_BIT_12, &raw);
                    adc_reading_TH += raw;
                }
            }
            adc_reading_TH /= NO_OF_SAMPLES;
            uint32_t voltage_TH = esp_adc_cal_raw_to_voltage(adc_reading_TH, adc_chars);
            double volt_TH = (double)voltage_TH / 1000;
            // THERMISTOR
            double Rx;
            Rx = (double)(50000 - (10000 * volt_TH)) / volt_TH;
            double y;
            y = (double)298.15 * log(10000 / Rx);
            double temp;
            temp = (double)(85875 + (y * 273.15)) / (3435 - y);
            
            sprintf(output, "temp:%f\n", temp);
            printf("%s", output);
            
            
            vTaskDelay(pdMS_TO_TICKS(2000));
            
        }
}

    static void batterymonitor()
    {
        check_efuse();
        
        if (unit == ADC_UNIT_1) {
            adc1_config_width(ADC_WIDTH_BIT_12);
            adc1_config_channel_atten(battery_channel, atten);
        }
        else {
            adc2_config_channel_atten((adc2_channel_t)battery_channel, atten);
        }
        
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
        print_char_val_type(val_type);
        
         char output[100];
            while (1) {
                //double volt;
                uint32_t adc_reading_B = 0;
                //uint32_t adc_reading = 0;
                for (int i = 0; i < NO_OF_SAMPLES; i++) {
                    if (unit == ADC_UNIT_1) {
                        adc_reading_B += adc1_get_raw((adc1_channel_t)battery_channel);
                    }
                    else {
                        int raw;
                        adc2_get_raw((adc2_channel_t)battery_channel, ADC_WIDTH_BIT_12, &raw);
                        adc_reading_B += raw;
                    }
                }
                adc_reading_B /= NO_OF_SAMPLES;
                uint32_t voltage_B = esp_adc_cal_raw_to_voltage(adc_reading_B, adc_chars);
                float voltdivide;
                       //printf("Raw: %d\tVoltage: %dmV\n", adc_reading_B, voltage_B);
                voltdivide = (((float)voltage_B/1000.00)*2000.00)/(1000.00);
                      // printf("Vout: %fV\n", voltdivide)
                
                sprintf(output, "Battery:%f\n", voltdivide);
                printf("%s", output);
                
                
                vTaskDelay(pdMS_TO_TICKS(2000));
                
            }
    
}
void app_main()
{
    //init();
    xTaskCreate(thermistor, "thermistor",1024*2, NULL, configMAX_PRIORITIES, NULL);
    
    xTaskCreate(batterymonitor, "battery_monitor",1024*2, NULL, configMAX_PRIORITIES, NULL);

}



