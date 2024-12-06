/*
 * Copyright (c) 2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"
#include "uLCD_4DGL.hpp"
#include "rtos.h"
#include "Motor.h"

// Trash Can Dims



Thread thread1; // Trash Sonar
Thread thread2; // LCD 
Thread thread3; // Motor driving and control
Thread thread4; // Servo motor + sonar 

Mutex mutex;

// Trash Sonar
DigitalOut trigger(p7);
DigitalIn echo(p8);
PwmOut buzzer(p21);
volatile int dist;
int correction = 0;
bool trashFull = false;
bool oldDetect = false;
int trashCanHeight = 20; // cm
int dist_min = 5; // cm

Timer sonarTrash;
//

// Motor and Control
Motor m1(p23, p20, p19); // pwm, fwd, rev
Motor m2(p22, p17, p16);
AnalogIn motorPot(p18);
AnalogIn lineIn(p15);
float s = 0.0;

//

//LCD
    uLCD_4DGL uLCD(p9,p10,p11);
    int x = 17;
    int y = 64;
    int xVel = 3;
    int yVel = 3;
    int radius = 10;
    int color = RED;


// Thread 1 Trash Sonar
void sonar_thread()
{
    buzzer.period(0.001);
    while(1) {

        trigger = 1;
 
        sonarTrash.reset();
        wait_us(10.0);
        trigger = 0;
   
        while (echo==0) {};
        sonarTrash.start();
        while (echo==1) {};
        sonarTrash.stop();
        // dist = (sonar.read_us()) / 148; // Inches
        mutex.lock();
        dist = (sonarTrash.read_us()) / 58; // centimeters
        mutex.unlock();
        if (dist <= dist_min) {
            trashFull = true;
            oldDetect = false;
            buzzer.write(0.5);
            // ThisThread::sleep_for(50ms);
            // buzzer.write(0.0);
        }
        else
        {
            trashFull = false;
            oldDetect = true;
             buzzer.write(0.0);
        }
        // printf(" %d cm \n\r",dist);
        int lineRead = 65536 - lineIn.read_u16();
        printf("Line Sensor: %d\n", lineRead);

        ThisThread::sleep_for(50ms);
    }
}

// Thread 2 LCD-bouncing ball
void lcd_thread() {
    // uLCD.baudrate(9600); 
    
    int value = 64;

    while(1){
        
        uLCD.filled_rectangle(0, 0, 128, 128, BLACK);

        mutex.lock();
        value = dist;
        mutex.unlock();
        printf("value from ulcd: %d  \n\r",value);


        if (value > 30) {
            value = 30;
        } else if (value < 5) {
            value = 5;
        }
        value = (int)(1.28 * (float(value-5)/25) * 100);
                

        if (value > 40) {
        uLCD.filled_rectangle(0, value, 128, 128, 0x00FF00);
        } else {
        uLCD.filled_rectangle(0, value, 128, 128, RED);
        }

        
        ThisThread::sleep_for(200ms);
    
    }
}


void motor_thread()
{
        while(1)
    {
        m1.speed(s);
        m2.speed(s);
        s = motorPot.read();
        if (s <= 0.25)
        {
            s = 0.0;
        }
        else if (s >= 0.5)
        {
            s = 1;
        }
    }
}

void lid_control()
{
    
}

int main()
{
    thread1.start(sonar_thread);
    thread2.start(lcd_thread);
    // thread3.start(motor_thread);
    // thread4.start(lid_control);

    while(1)
    {
        ThisThread::sleep_for(10ms);
    }
}
