# RC Car Collision Avoidance (STM32)

## Overview

This project implements a manually controlled RC car with built-in collision avoidance using an STM32 microcontroller.
The system reads PWM signals from a FlySky receiver and converts them into motor commands using a differential drive model. An ultrasonic sensor is used to detect obstacles and prevent forward motion when objects are too close.

## Features

* PWM input capture for throttle and steering
* Differential drive motor control
* Ultrasonic distance measurement
* Automatic stop when obstacle is too close
* Warning indication using onboard LED
* Real-time control loop on STM32

## Hardware

* STM32 NUCLEO-F446RE
* FlySky FS-i6X transmitter
* FlySky FS-iA6B receiver
* TB6612FNG motor driver
* Ultrasonic sensor (HC-SR04 or similar)
* 2WD RC car chassis

## System Behavior

* Reads PWM signals (1000–2000 µs) from RC receiver
* Maps inputs to throttle and steering commands
* Uses differential drive:

  * Left motor = throttle + steering
  * Right motor = throttle - steering
* Continuously measures front distance using ultrasonic sensor
* If obstacle is detected:

  * Turns on warning LED when within 25 cm
  * Prevents forward motion when within 12 cm

## Hardware Configuration

### PWM Input (Receiver)

* PA0 → TIM2_CH1 (Steering)
* PA1 → TIM2_CH2 (Throttle)

### Motor Control (PWM Output)

* PA6 → TIM3_CH1 (Left Motor PWM)
* PA7 → TIM3_CH2 (Right Motor PWM)

### Motor Driver Direction Pins

* PB0 → AIN1
* PB1 → AIN2
* PB2 → BIN1
* PB10 → BIN2
* STBY pin enabled via GPIO

### Ultrasonic Sensor

* Trigger → GPIO output
* Echo → GPIO input
* Uses microsecond timing for pulse measurement

### UART (Debug)

* USART2 used for serial output (printf)

## Timer Configuration

### TIM2

* Prescaler: 83 (1 MHz timer)
* Used for:

  * PWM input capture
  * Microsecond timing (delay + ultrasonic)

### TIM3

* Prescaler: 83
* Period: 100
* Used for PWM motor control

## How It Works

1. Capture PWM input from RC receiver using TIM2
2. Convert pulse width to control commands (-100 to 100)
3. Compute left/right motor speeds using differential drive
4. Measure distance using ultrasonic sensor
5. If obstacle is too close:

   * Override forward throttle to zero
6. Output motor commands using PWM via TIM3

## How to Build and Run

1. Open project in STM32CubeIDE
2. Build the project
3. Flash to STM32 NUCLEO-F446RE
4. Connect:

   * RC receiver to PWM input pins
   * Motor driver to output pins
   * Ultrasonic sensor to GPIO
5. Currently powering system through laptop, but a 9 V battery to VIN will work too
