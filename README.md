# PID_STM_Control: Aerodynamic Levitation System

![Status: Active](https://img.shields.io/badge/Status-Active-success)
![Language: C](https://img.shields.io/badge/Language-C-blue)
![Platform: STM32](https://img.shields.io/badge/Platform-STM32-orange)

<img width="260" height="462" alt="PID_Stm32" src="https://github.com/user-attachments/assets/bed42bfd-7802-4ef6-9be8-a41be049812e" />


## Project Description

PID_STM_Control is an embedded systems project demonstrating non-linear PID control on an STM32 microcontroller. The system aims to achieve precise and stable levitation of a lightweight ball (approx. 3g) inside a 35mm aerodynamic tube using an industrial PWM-controlled fan.

The project successfully resolves several real-world engineering challenges, including hardware optical sensor noise, gravitational asymmetry, and non-linear fluid dynamics.

## Engineering Solutions & Features

The control loop implementation was written entirely from scratch and incorporates advanced mechanisms typically found in industrial automation:

* **Custom Parallel PID Controller:** An in-house implementation of the control algorithm featuring a configurable loop sampling time (`dt`).
* **Dynamic Feedforward (Gain Scheduling):** An adaptive PWM baseline that compensates for aerodynamic non-linearities. The system automatically shifts the operating point depending on the target Setpoint (varying baseline power required at the bottom vs. the top of the tube).
* **Zero-Crossing Anti-Windup:** A mechanism that instantly resets the accumulated integral term the moment the error sign changes (crosses the setpoint), effectively eliminating massive, delayed overshoots.
* **Asymmetric Saturation Limits:** Separate, dynamically tuned boundaries for braking (ball descending) and accelerating (ball ascending), preventing the object from violently crashing into the sensor at the top.
* **EMA Filtering (Exponential Moving Average):** A lightweight, single-pole low-pass filter that effectively dampens the hardware noise inherent to the ToF laser sensor, providing a clean signal for the derivative term ($K_d$).

## Architecture & Hardware

* **Microcontroller:** STM32 (configured via HAL)
* **Actuator:** Industrial fan - ARCTIC S8038-10K with a built-in driver (Timer PWM control mapped directly to registers)
* **Sensor:** VL53L0X Time-of-Flight laser distance sensor (I2C communication, operating in Continuous Fast read mode)
* **Telemetry:** Bi-directional UART serial communication with a host PC.

## Real-Time Telemetry

The project includes a custom Python script (`plot_pid.py`) utilizing the `matplotlib` library. Operating like a digital oscilloscope, this tool captures incoming data from the microcontroller over the serial port every 50ms and visualizes:
* The current position of the ball tracked against the active Setpoint.
* The raw PWM output signal alongside the real-time PID correction factor.
* Live numerical values embedded directly into the graphical interface headlines.

## Technologies
* C / STM32 HAL API
* I2C / PWM / UART
* Python 3 (PySerial, Matplotlib, Regex)
