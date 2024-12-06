/*
 * Copyright (c) 2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"
#include "uLCD_4DGL.hpp"
#include "rtos.h"
#include "Motor.h"
#include "Servo.h"
#include "Timer.h"
#include <cstdint>
#include <math.h>

// Threads for multitasking
Thread thread2; // Handles LCD updates
Thread thread3; // Manages motor control and navigation
Thread thread4; // Controls servo motor and trash lid mechanism

// Mutex for motor control to avoid conflicts
Mutex stopMotorMtx;

// Trash detection sonar configuration
DigitalOut trigger(p7); // Sonar trigger pin
DigitalIn echo(p8);     // Sonar echo pin
volatile int dist;      // Measured distance (in cm)
volatile bool motionDetected = false; // Tracks if motion is detected for the lid
uint8_t dist_min = 5; // Minimum distance to consider the trash can full (in cm)
Timer sonarTrash; // Timer for measuring sonar echo duration

// LCD configuration
uLCD_4DGL uLCD(p9, p10, p11); // LCD interface pins

// Motor and line sensor configuration
Motor l_motor(p23, p14, p13); // Left motor: pwm, forward, reverse
Motor r_motor(p22, p17, p16); // Right motor: pwm, forward, reverse
Motor b_motor(p25, p5, p6);   // Back motor: pwm, forward, reverse
AnalogIn left_line(p15);      // Analog input for left line sensor
AnalogIn middle_line(p19);    // Analog input for middle line sensor
AnalogIn right_line(p20);     // Analog input for right line sensor

// Trash lid motion detection configuration
DigitalOut lidTrigger(p30);   // Trigger pin for lid sonar
DigitalOut lidEcho(p29);      // Echo pin for lid sonar
PwmOut lidServo(p24);         // Servo motor PWM output
uint8_t minHandDist = 50;     // Minimum distance to detect a hand (in cm)
Timer sonarLid;               // Timer for lid sonar

volatile bool motorsOff = false; // Flag to stop motors when needed

// Function to update the LCD display with trash status
void lcd_thread() {
    dist = 20;
    uint16_t distTrashSum = 0;
    uint8_t arrIdx = 0;
    int value = 64;
    bool readFlag = false;

    while (1) {
        // Trigger sonar pulse
        trigger = 1;
        sonarTrash.reset();
        wait_us(10.0); // Short trigger pulse
        trigger = 0;

        // Measure echo time
        while (echo == 0) {};
        sonarTrash.start();
        while (echo == 1) {};
        sonarTrash.stop();

        // Convert echo time to distance (in cm)
        distTrashSum += (sonarTrash.read_us()) / 58; // Convert microseconds to cm
        arrIdx++;

        // Average distance over 8 measurements for stability
        if (arrIdx > 7) {
            dist = distTrashSum / 8;
            distTrashSum = 0;
            arrIdx = 0;
            printf("Trash Sensor: %d cm \n\r", dist);
        }

        // Update LCD display based on distance
        uLCD.filled_rectangle(0, 0, 128, value, BLACK);
        if (!motionDetected) {
            value = dist;

            // Clamp value to fit display range
            if (value > 13) {
                value = 13;
            } else if (value < 5) {
                value = 5;
            }
            value = (int)(1.28 * (float(value - 5) / 8) * 100);

            // Color based on trash level
            if (value > 30) {
                uLCD.filled_rectangle(0, value, 128, 128, 0x00FF00); // Green for low level
            } else {
                uLCD.filled_rectangle(0, value, 128, 128, RED); // Red for high level
            }
        }

        ThisThread::sleep_for(80ms); // Sleep to reduce update frequency
    }
}

// Function to control motors for line following
void motor_thread() {
    while (1) {
        // Read line sensor values
        int left_value = 65536 - left_line.read_u16();
        int middle_value = 65536 - middle_line.read_u16();
        int right_value = 65536 - right_line.read_u16();

        if (!motorsOff) {
            // Adjust motor speeds based on sensor input
            if (left_value < 10000) {
                l_motor.speed(0.2);
                r_motor.speed(1.0);
                b_motor.speed(0.1);
            } else if (left_value < 13000 && middle_value < 16000) {
                l_motor.speed(0.2);
                r_motor.speed(0.8);
                b_motor.speed(0.1);
            } else if (left_value < 16000 && middle_value < 13000) {
                l_motor.speed(0.2);
                r_motor.speed(0.75);
                b_motor.speed(0.1);
            } else if (middle_value < 10000 && left_value > 16000 && right_value > 16000) {
                l_motor.speed(0.55);
                r_motor.speed(0.55);
                b_motor.speed(0.3);
            } else if (middle_value < 13000 && right_value < 16000) {
                l_motor.speed(0.2);
                r_motor.speed(0.75);
                b_motor.speed(0.0);
            } else if (middle_value < 16000 && right_value < 13000) {
                l_motor.speed(0.8);
                r_motor.speed(0.2);
                b_motor.speed(0.1);
            } else if (right_value < 10000) {
                l_motor.speed(1.0);
                r_motor.speed(0.2);
                b_motor.speed(0.0);
            } else {
                l_motor.speed(0.4);
                r_motor.speed(0.4);
                b_motor.speed(0.2);
            }
        } else {
            // Stop all motors if motorsOff flag is set
            l_motor.speed(0.0);
            r_motor.speed(0.0);
            b_motor.speed(0.0);
        }

        ThisThread::sleep_for(10ms); // Sleep to reduce loop frequency
    }
}

// Function to control the trash lid based on motion detection
void lid_control() {
    uint8_t count = 0;
    uint8_t handDist = 0;
    bool flag = false;

    lidServo.period(0.02); // Set PWM period for servo

    while (1) {
        if (!motionDetected) {
            // Trigger sonar pulse
            lidTrigger = 1;
            sonarLid.reset();
            wait_us(10.0); // Short trigger pulse
            lidTrigger = 0;

            // Measure echo time
            while (lidEcho == 0) {};
            sonarLid.start();
            while (lidEcho == 1) {};
            sonarLid.stop();
            handDist = (sonarLid.read_us()) / 58; // Convert microseconds to cm
            printf("Lid Sensor: %d cm \n\r", handDist);
        }

        if (handDist <= minHandDist && handDist > 30) {
            motionDetected = true;
        }

        if (motionDetected && count < 5) {
            // Open lid and stop motors during motion detection
            lidServo.pulsewidth(0.002);
            stopMotorMtx.lock();
            motorsOff = true;
            stopMotorMtx.unlock();
            count++;
            if (count == 5) {
                flag = true;
            }
        } else {
            // Close lid and resume motor operation
            lidServo.pulsewidth(0.001);
            stopMotorMtx.lock();
            motorsOff = false;
            stopMotorMtx.unlock();
            count = 0;
            if (flag) {
                wait_us(500000); // Delay to prevent rapid switching
                flag = false;
            }
            motionDetected = false;
        }

        ThisThread::sleep_for(100ms); // Sleep to reduce loop frequency
    }
}

// Main function to start the system
int main() {
    thread2.start(lcd_thread);  // Start the LCD thread
    thread3.start(motor_thread); // Start the motor thread
    thread4.start(lid_control); // Start the lid control thread

    // Main loop keeps the program running
    while (1) {
        ThisThread::sleep_for(10ms); // Prevent main thread from terminating
    }
}
