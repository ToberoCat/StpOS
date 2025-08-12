//
// Created by tobias on 4/3/25.
//

// Improved wait_for_light function with averaging, low-pass filtering,
// hysteresis thresholding, and more frequent sensor readings.
// Designed to be more robust under difficult light conditions
// while still logging with ascii light bulbs.
// Improved by: Tobias Madlberger (tobias.madlberger@gmail.com)
// Original author: Zachary Sasser - zsasser@kipr.org

#include <cstdio>
#include <kipr/analog/analog.h>
#include <kipr/console/console.h>
#include <libstp/_config.h>

#include "kipr/button/button.h"
#include "kipr/digital/digital.h"
#include "libstp/sensor/sensor.h"
#include "libstp/utility/timing.h"

#define BAD_DIFFERENCE 100
#define NUM_SAMPLES 10
#define SENSOR_INTERVAL 50
#define DEBOUNCE_DELAY 50
#define ALPHA 0.3
#define ADAPTIVE_ALPHA 0.01 // For adaptive thresholding
#define HYSTERESIS_FACTOR 0.2 // Increased hysteresis
#define CONFIRMATION_TIME 500 // Increased confirmation time (milliseconds)

bool is_button_clicked()
{
    return any_button() || digital(9);
}


int get_average_sensor_value(const int port)
{
    int sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        sum += analog(port);
        libstp::utility::msleep(SENSOR_INTERVAL / NUM_SAMPLES);
    }
    return sum / NUM_SAMPLES;
}

void debounce_button_press()
{
    while (is_button_clicked())
    {
        libstp::utility::msleep(DEBOUNCE_DELAY);
    }
}

void display_calibration_on_prompt(const char* message, int value)
{
    printf("%s\n", message);
    printf("Value: %d  <---- \n", value);
    printf("     |=| \n");
    printf("     / \\ \n");
    printf("--- (   ) ---  \n");
    printf("   , `-'.   \n");
    printf("  /   |  \\   \n");
    printf(" '    |   `   \n");
    libstp::utility::msleep(SENSOR_INTERVAL);
    console_clear();
}

void display_calibration_off_prompt(const int value)
{
    printf("Light OFF Value (avg): %d  <---- \n", value);
    printf("     |=| \n");
    printf("     / \\ \n");
    printf("    (   ) \n");
    printf("     `-' \n");
    libstp::utility::msleep(SENSOR_INTERVAL);
    console_clear();
}


int calibrate_light_on_value(const int port)
{
    debounce_button_press();
    while (!is_button_clicked())
    {
        const int avg_on = get_average_sensor_value(port);
        display_calibration_on_prompt("Turn on light and press button...", avg_on);
    }
    debounce_button_press();
    return get_average_sensor_value(port);
}

int calibrate_light_off_value(const int port, const int light_on_value)
{
    while (!is_button_clicked())
    {
        const int avg_off = get_average_sensor_value(port);
        printf("Turn off light and press button... \n \n");
        printf("Light ON Value: %d \n", light_on_value);
        display_calibration_off_prompt(avg_off);
        libstp::utility::msleep(SENSOR_INTERVAL);
        console_clear();
    }
    debounce_button_press();
    return get_average_sensor_value(port);
}

bool is_calibration_valid(const int light_on_value, const int light_off_value)
{
    return light_off_value - light_on_value > BAD_DIFFERENCE;
}

void libstp::sensor::LightSensor::wait_for_light() const
{
    float filteredReading = 0;
    while (true)
    {
        const int light_on_value = calibrate_light_on_value(port);
        const int light_off_value = calibrate_light_off_value(port, light_on_value);

        SPDLOG_INFO("Calibrated Light ON Value: {}", light_on_value);
        SPDLOG_INFO("Calibrated Light OFF Value: {}", light_off_value);

        if (!is_calibration_valid(light_on_value, light_off_value))
        {
            console_clear();
            printf("BAD CALIBRATION!!!");
            SPDLOG_ERROR("BAD CALIBRATION!!!");
            if (light_off_value < 1000)
            {
                printf(" -> Shield Light Sensor. \n");
            }
            else
            {
                printf(" -> Values are too close.\n");
            }
            printf(" \n \n Press any button to restart calibration.");
            while (!is_button_clicked())
            {
                utility::msleep(DEBOUNCE_DELAY);
            }
            continue;
        }

        const int low_threshold = light_on_value + static_cast<int>((light_off_value - light_on_value) * 0.1);
        const int high_threshold = low_threshold + 5;
        unsigned long startTime = 0;

        SPDLOG_INFO("Waiting for starting light...");
        while (true)
        {
            int raw = analog(port);
            filteredReading = ALPHA * raw + (1 - ALPHA) * filteredReading;

            printf("Waiting for starting light...\n \n");
            printf("Light ON Value: %d \n", light_on_value);
            printf("Light OFF Value: %d \n", light_off_value);
            printf("---------------------- \n");
            printf("Low Threshold: %d \n", low_threshold);
            printf("High Threshold: %d \n", high_threshold);
            printf("Current Value: %d  <----  \n", static_cast<int>(filteredReading));
            utility::msleep(SENSOR_INTERVAL);
            console_clear();

            if (filteredReading < low_threshold)
            {
                SPDLOG_INFO("Possible light detected, starting timer...");
                auto start = std::chrono::high_resolution_clock::now();
                while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count() < 1)
                {
                    raw = analog(port);
                    filteredReading = ALPHA * raw + (1 - ALPHA) * filteredReading;
                    if (filteredReading > high_threshold)
                    {
                        SPDLOG_INFO("Light detection interrupted.");
                        break;
                    }
                    utility::msleep(10);
                }
                if (filteredReading < low_threshold)
                {
                    SPDLOG_INFO("Starting light detected.");
                    return; // Starting light detected.
                }
            }
        }
    }
}