/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_rom_sys.h" // lib used for us delay
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h" // PWM lib
#include "ble_server.h"

/*void app_main(void)
{
    printf("Hello world!\n");

    // Print chip information
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}*/

// === Shift Register (74HC595) GPIO mapping ===
#define SR_DATA   GPIO_NUM_21  // DS
#define SR_CLOCK  GPIO_NUM_22  // SHCP
#define SR_LATCH  GPIO_NUM_23  // STCP

// === Photo Resistor GPIO mapping ===
#define PHRES_ADC_CHANNEL ADC1_CHANNEL_6  // GPIO34 (ADC1_6)

// === LED mask mapping (shift register output bits) ===
#define LED_HEADLIGHT_LEFT     (1 << 0)  // Q0
#define LED_HEADLIGHT_RIGHT    (1 << 1)  // Q1
#define LED_TAIL_LEFT          (1 << 2)  // Q2
#define LED_TAIL_RIGHT         (1 << 3)  // Q3
#define LED_UNLOCKED_GREEN     (1 << 4)  // Q4
#define LED_LOCKED_RED         (1 << 5)  // Q5

// === Buzzer GPIO mapping ===
#define BUZZER_GPIO GPIO_NUM_19

// === Buzzer LEDC driver mapping
#define BUZZER_CHANNEL LEDC_CHANNEL_4
#define BUZZER_TIMER   LEDC_TIMER_3

// === DHT11 GPIO mapping ===
#define DHT11_PIN GPIO_NUM_18

// === State flags ===
bool is_locked = true;
bool is_dark = false;
bool follow_me_home = false;

// === DC Motor control ===
#define DC_MOTOR_GPIO GPIO_NUM_5
#define DC_MOTOR_CHANNEL LEDC_CHANNEL_5
#define DC_MOTOR_TIMER   LEDC_TIMER_1

// === SERVOMOTOR GPIO mapping ===
#define SERVO_GPIO GPIO_NUM_13
#define SERVO_CHANNEL LEDC_CHANNEL_6
#define SERVO_TIMER   LEDC_TIMER_2

// === ULTRASONIC SENSOR GPIO mapping ===
#define HC_TRIG_PIN GPIO_NUM_12
#define HC_ECHO_PIN GPIO_NUM_14
#define ECHO_TIMEOUT_US 30000 // 30000 us ~ 5 meter

// === TCRT5000 GPIO MAPPING ===
#define IR_LEFT_ADC_CHANNEL   ADC1_CHANNEL_4   // GPIO32
#define IR_RIGHT_ADC_CHANNEL  ADC1_CHANNEL_5   // GPIO33

// === motor A GPIO MAPPING ===
#define IN1 GPIO_NUM_25
#define IN2 GPIO_NUM_26

// === motor B GPIO MAPPING ===
#define IN3 GPIO_NUM_27
#define IN4 GPIO_NUM_4

// === Shift register init ===
void shift_register_init(void)
{
    gpio_set_direction(SR_DATA, GPIO_MODE_OUTPUT);
    gpio_set_direction(SR_CLOCK, GPIO_MODE_OUTPUT);
    gpio_set_direction(SR_LATCH, GPIO_MODE_OUTPUT);
}

// === Send 8-bit value to 74HC595 ===
void shift_register_write(uint8_t value)
{
    gpio_set_level(SR_LATCH, 0);

    for (int i = 7; i >= 0; i--) { // MSB to LSB
        gpio_set_level(SR_CLOCK, 0);
        gpio_set_level(SR_DATA, (value >> i) & 0x01);
        gpio_set_level(SR_CLOCK, 1);
    }

    gpio_set_level(SR_LATCH, 1);
}

// === LED logic ===
void update_leds(void)
{
    uint8_t pattern = 0;

    if (is_locked) {
        pattern |= LED_LOCKED_RED;

        if (follow_me_home) {
            pattern |= LED_HEADLIGHT_LEFT | LED_HEADLIGHT_RIGHT;
            pattern |= LED_TAIL_LEFT | LED_TAIL_RIGHT;
        }
    } else {
        pattern |= LED_UNLOCKED_GREEN;

        if (is_dark) {
            pattern |= LED_HEADLIGHT_LEFT | LED_HEADLIGHT_RIGHT;
            pattern |= LED_TAIL_LEFT | LED_TAIL_RIGHT;
        }
    }

    shift_register_write(pattern);
}

// === Photo resistor init
void photoresistor_init(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12); // Rersolution 0–4095 (2^12 bits)
    adc1_config_channel_atten(PHRES_ADC_CHANNEL, ADC_ATTEN_DB_12); // 0–3.3V
}

// === Photo resistor read 16-bits function ===
uint16_t photoresistor_read(void)
{
    return adc1_get_raw(PHRES_ADC_CHANNEL);
}

// === Buzzer init function
void buzzer_init(void)
{
    ledc_timer_config_t timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 2000,  // freq buzzer (20 Hz - 20kHz audible frequency -> Resonant Frequency of passive buzzer: ~2300 Hz => 2kHz is fine)
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = BUZZER_TIMER,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .channel    = BUZZER_CHANNEL,
        .duty       = 0,  // signal starts with 0% duty cycle
        .gpio_num   = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = BUZZER_TIMER
    };
    ledc_channel_config(&channel);
}

void buzzer_on(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL, 512); // 50% duty
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL);
}

void buzzer_off(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_CHANNEL);
}

void buzzer_beep_pattern(int times, int delay_ms)
{
    for (int i = 0; i < times; i++) {
        buzzer_on();
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
        buzzer_off();
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }
}

// ===  DHT11 struct ===
typedef struct
{
    uint8_t u8IntegralHum;
    uint8_t u8DecimalHum;
    uint8_t u8IntegralTemp;
    uint8_t u8DecimalTemp;
    uint8_t u8CheckSum;
} DHT11_struct;


// ===  DHT11 functions ===
void DHT11_vRequest(void)
{
    // Set the direction to output for a new communication
    gpio_set_direction(DHT11_PIN, GPIO_MODE_OUTPUT);

    // H-L-H seq of at least 18ms
    gpio_set_level(DHT11_PIN, 1);
    gpio_set_level(DHT11_PIN, 0);
    esp_rom_delay_us(20000); // 20ms LOW
    gpio_set_level(DHT11_PIN, 1);
    // Set as input so 10k pull-up from circuit drives pin High
    gpio_set_direction(DHT11_PIN, GPIO_MODE_INPUT);
}

int8_t DHT11_i8Response(void)
{
    uint8_t counter = 0;
    while (gpio_get_level(DHT11_PIN)) {
        if (++counter >= 45) return -1;
        esp_rom_delay_us(1);
    }

    counter = 0;
    while (!gpio_get_level(DHT11_PIN)) {
        if (++counter >= 85) return -1;
        esp_rom_delay_us(1);
    }

    counter = 0;
    while (gpio_get_level(DHT11_PIN)) {
        if (++counter >= 85) return -1;
        esp_rom_delay_us(1);
    }

    return 0;
}

int8_t DHT11_i8Receive(void)
{
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        uint8_t counter = 0;
        while (!gpio_get_level(DHT11_PIN)) {
            if (++counter >= 55) return -1;
            esp_rom_delay_us(1);
        }

        counter = 0;
        while (gpio_get_level(DHT11_PIN)) {
            if (++counter >= 75) return -1;
            esp_rom_delay_us(1);
        }

        data <<= 1;
        if (counter > 28) data |= 1;
    }
    return data;
}

DHT11_struct DHT11_dht11Read(void)
{
    DHT11_struct result = {-1, -1, -1, -1, -1};

    DHT11_vRequest();
    if (DHT11_i8Response() != 0) return result;

    result.u8IntegralHum = DHT11_i8Receive();
    result.u8DecimalHum = DHT11_i8Receive();
    result.u8IntegralTemp = DHT11_i8Receive();
    result.u8DecimalTemp = DHT11_i8Receive();
    result.u8CheckSum = DHT11_i8Receive();

    return result;
}

void DHT11_vPrintValues(void)
{
    DHT11_struct data = DHT11_dht11Read();

    if (data.u8IntegralTemp == (uint8_t)-1 || data.u8IntegralHum == (uint8_t)-1) {
        printf("DHT11 error: unable to read data.\n");
        return;
    }

    uint8_t sum = data.u8IntegralHum + data.u8DecimalHum + data.u8IntegralTemp + data.u8DecimalTemp;
    if (sum != data.u8CheckSum) {
        printf("DHT11 checksum failed.\n");
        return;
    }

    printf("Temperature: %d°C, Humidity: %d%%\n", data.u8IntegralTemp, data.u8IntegralHum);
}

void dc_motor_init(void)
{
    ledc_timer_config_t motor_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,  // 0–1023
        .freq_hz = 5000,                       // 5kHz for motor
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = DC_MOTOR_TIMER,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&motor_timer);
    
    ledc_channel_config_t motor_channel = {
        .channel    = DC_MOTOR_CHANNEL,
        .duty       = 0,
        .gpio_num   = 
        DC_MOTOR_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = DC_MOTOR_TIMER
    };
    ledc_channel_config(&motor_channel);
}

void dc_motor_set_speed(uint8_t percent)
{
    if (percent > 100) percent = 100;
    uint32_t duty = (percent * 1023) / 100;  // 10-bit resolution
    ledc_set_duty(LEDC_LOW_SPEED_MODE, DC_MOTOR_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, DC_MOTOR_CHANNEL);
}

void servo_init(void)
{
    ledc_timer_config_t servo_timer = {
        .duty_resolution = LEDC_TIMER_16_BIT, // high resolution for precision (0-65535)
        .freq_hz = 50,                        // standard freq for servo
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = SERVO_TIMER,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&servo_timer);

    ledc_channel_config_t servo_channel = {
        .channel    = SERVO_CHANNEL,
        .duty       = 0,
        .gpio_num   = SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = SERVO_TIMER
    };
    ledc_channel_config(&servo_channel);
}

void servo_set_angle(uint8_t angle)
{
    if (angle > 180) angle = 180;

    // 0° duty cycle is ~1638 (2.5) and for 180° is ~8192 (12.5%) at 16-bit resolution
    uint32_t min_duty = 1638;
    uint32_t max_duty = 8192;
    uint32_t duty = min_duty + ((max_duty - min_duty) * angle) / 180;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);
}

void ultrasonic_init(void)
{
    gpio_reset_pin(HC_TRIG_PIN);
    gpio_set_direction(HC_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(HC_TRIG_PIN, 0);

    gpio_reset_pin(HC_ECHO_PIN);
    gpio_set_direction(HC_ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(HC_ECHO_PIN, GPIO_PULLDOWN_ONLY);
}

float ultrasonic_get_distance_cm(void)
{
    /* --- Trigger the ultrasonic burst --- */
    gpio_set_level(HC_TRIG_PIN, 0);
    esp_rom_delay_us(2);

    gpio_set_level(HC_TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(HC_TRIG_PIN, 0);

    int64_t t0 = esp_timer_get_time();
    while (!gpio_get_level(HC_ECHO_PIN)) {
        if ((esp_timer_get_time() - t0) > ECHO_TIMEOUT_US) {
            // no rising edge -> sensor not responding
            return -1.0f;
        }
    }

    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(HC_ECHO_PIN)) {
        if ((esp_timer_get_time() - echo_start) > ECHO_TIMEOUT_US) {
            // pulse too long -> object out of range
            return -1.0f;
        }
    }
    int64_t echo_end = esp_timer_get_time();

    // duration (us) * speed of sound (0.0343 cm/us) / 2
    float duration_us = (float)(echo_end - echo_start);
    printf("Distance %.2f\n", (duration_us * 0.0343f) * 0.5f);
    return (duration_us * 0.0343f) * 0.5f;
}

void ir_sensors_init(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);  // 12-bit = 0 - 4095

    adc1_config_channel_atten(IR_LEFT_ADC_CHANNEL, ADC_ATTEN_DB_11);   // max ~3.3V
    adc1_config_channel_atten(IR_RIGHT_ADC_CHANNEL, ADC_ATTEN_DB_11);
}

uint16_t ir_sensor_read_left(void)
{
    return adc1_get_raw(IR_LEFT_ADC_CHANNEL);
}

uint16_t ir_sensor_read_right(void)
{
    return adc1_get_raw(IR_RIGHT_ADC_CHANNEL);
}

void motor_pwm_init()
{
    // common timer for both motors
    ledc_timer_config_t timer_config = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 1000, // PWM freq
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_config);

    // config PWM on each pin
    ledc_channel_config_t channels[] = {
        {.channel = LEDC_CHANNEL_0, .gpio_num = IN1, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_0, .duty = 0},
        {.channel = LEDC_CHANNEL_1, .gpio_num = IN2, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_0, .duty = 0},
        {.channel = LEDC_CHANNEL_2, .gpio_num = IN3, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_0, .duty = 0},
        {.channel = LEDC_CHANNEL_3, .gpio_num = IN4, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_0, .duty = 0},
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&channels[i]);
    }
}

void motorA_forward(uint32_t speed) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed); // IN1
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);     // IN2
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void motorA_backward(uint32_t speed) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);     // IN1
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed); // IN2
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void motorB_forward(uint32_t speed) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, speed); // IN3
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);     // IN4
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

void motorB_backward(uint32_t speed) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);     // IN3
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, speed); // IN4
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

void app_main(void)
{
    ble_server_init();
    shift_register_init();
    photoresistor_init();
    buzzer_init();
    dc_motor_init();
    servo_init();
    ultrasonic_init();
    ir_sensors_init();
    motor_pwm_init();

    // sim states
    is_locked = true;
    follow_me_home = false;
    is_dark = (photoresistor_read() < 1800);
    uint16_t light_value;
    float dist = ultrasonic_get_distance_cm();
    uint16_t left_val = ir_sensor_read_left();
    uint16_t right_val = ir_sensor_read_right();

    while (1) {
        // printf("Distance: %.2f cm\n", dist);
        // buzzer_beep_pattern(3, 200); // 3 beeps of 200ms
        // update_leds(); // turn on only Lock LED (Red)
        // light_value = photoresistor_read();
        // printf("Photoresistor value: %u\n", light_value);
        // DHT11_vPrintValues();
        // dc_motor_set_speed(20);
        // printf("Motor 20\n");
        // servo_set_angle(70);
        // printf("Servo set at %d degrees\n", 70);
        // dist = ultrasonic_get_distance_cm();
        // printf("Distance: %.2f cm\n", dist);
        // printf("IR Left: %u | IR Right: %u\n", left_val, right_val);
        // vTaskDelay(2500 / portTICK_PERIOD_MS);

        // // sim transitions
        // dist = ultrasonic_get_distance_cm();
        // printf("Distance: %.2f cm\n", dist);
        // is_locked = false;
        // is_dark = (photoresistor_read() < 1800);
        // update_leds(); // turn on only Unlock LED (Green)
        // light_value = photoresistor_read();
        // printf("Photoresistor value: %u\n", light_value);
        // DHT11_vPrintValues();
        // dc_motor_set_speed(40);
        // printf("Motor 40\n");
        // servo_set_angle(80);
        // printf("Servo set at %d degrees\n", 80);
        // dist = ultrasonic_get_distance_cm();
        // printf("Distance: %.2f cm\n", dist);
        // left_val = ir_sensor_read_left();
        // right_val = ir_sensor_read_right();
        // printf("IR Left: %u | IR Right: %u\n", left_val, right_val);
        // vTaskDelay(2500 / portTICK_PERIOD_MS);

        // dist = ultrasonic_get_distance_cm();
        // printf("Distance: %.2f cm\n", dist);
        // is_dark = (photoresistor_read() < 1800);
        // update_leds(); // Unlock LED (Green) remains turned on & turn on head lights + tail lights
        // light_value = photoresistor_read();
        // printf("Photoresistor value: %u\n", light_value);
        // DHT11_vPrintValues();
        // dc_motor_set_speed(60);
        // printf("Motor 60\n");
        // servo_set_angle(90);
        // printf("Servo set at %d degrees\n", 90);
        // dist = ultrasonic_get_distance_cm();
        // printf("Distance: %.2f cm\n", dist);
        // left_val = ir_sensor_read_left();
        // right_val = ir_sensor_read_right();
        // printf("IR Left: %u | IR Right: %u\n", left_val, right_val);
        // vTaskDelay(2500 / portTICK_PERIOD_MS);

        // follow_me_home = false;
        // is_dark = (photoresistor_read() < 1800);
        // update_leds(); // keep previous state because is_dark = true
        // light_value = photoresistor_read();
        // printf("Photoresistor value: %u\n", light_value);
        // DHT11_vPrintValues();
        // dc_motor_set_speed(80);
        // printf("Motor 80\n");

        // vTaskDelay(2500 / portTICK_PERIOD_MS); // ESP is resetting on 100 DUTY CYCLE on DC motor
       
        // is_dark = (photoresistor_read() < 1800);
        // update_leds(); // keep previous state because is_dark = true
        // light_value = photoresistor_read();
        // printf("Photoresistor value: %u\n", light_value);
        // DHT11_vPrintValues();
        // dc_motor_set_speed(100);
        // printf("Motor 100\n");
        // vTaskDelay(2500 / portTICK_PERIOD_MS);

        // dist = ultrasonic_get_distance_cm();
        // printf("Distance: %.2f cm\n", dist);
        // is_dark = (photoresistor_read() < 1800);
        // update_leds(); // keep previous state because is_dark = true
        // light_value = photoresistor_read();
        // printf("Photoresistor value: %u\n", light_value);
        // DHT11_vPrintValues();
        // dc_motor_set_speed(80);
        // printf("Motor 80\n");
        // servo_set_angle(95);
        // printf("Servo set at %d degrees\n", 95);
        // dist = ultrasonic_get_distance_cm();
        // printf("Distance: %.2f cm\n", dist);
        // left_val = ir_sensor_read_left();
        // right_val = ir_sensor_read_right();
        // printf("IR Left: %u | IR Right: %u\n", left_val, right_val);
        // vTaskDelay(2500 / portTICK_PERIOD_MS);

        // dist = ultrasonic_get_distance_cm();
        // printf("Distance: %.2f cm\n", dist);
        // is_dark = false;
        // update_leds(); // keep only Unlock LED (Green)
        // light_value = photoresistor_read();
        // printf("Photoresistor value: %u\n", light_value);
        // DHT11_vPrintValues();
        // dc_motor_set_speed(50);
        // printf("Motor 50\n");
        // servo_set_angle(100);
        // printf("Servo set at %d degrees\n", 100);
        // dist = ultrasonic_get_distance_cm();
        // printf("Distance: %.2f cm\n", dist);
        // left_val = ir_sensor_read_left();
        // right_val = ir_sensor_read_right();
        // printf("IR Left: %u | IR Right: %u\n", left_val, right_val);
        // vTaskDelay(2500 / portTICK_PERIOD_MS);

        // is_locked = true;

        // while (1) {
        //     printf("Forward\n");
        //     motorA_forward(800); // max 1023
        //     motorB_forward(800);
        //     vTaskDelay(pdMS_TO_TICKS(2000));

        //     printf("Backward\n");
        //     motorA_backward(800);
        //     motorB_backward(800);
        //     vTaskDelay(pdMS_TO_TICKS(2000));

        //     printf("Stop\n");
        //     motorA_forward(0);
        //     motorB_forward(0);
        //     vTaskDelay(pdMS_TO_TICKS(2000));
        //     break;
        // }
        // servo_set_angle(90);
        // printf("Servo set at %d degrees\n", 90);
        // vTaskDelay(1250 / portTICK_PERIOD_MS);

        for(int i = 0; i < 10; ++i)
        {
            dist = ultrasonic_get_distance_cm();
            vTaskDelay(1250 / portTICK_PERIOD_MS);
        }
    }
}