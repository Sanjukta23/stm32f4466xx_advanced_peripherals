# STM32F4 Advanced Peripherals

A progression of STM32CubeIDE / HAL-based projects exploring the more advanced peripherals of the STM32F4 microcontroller: timers, CAN, low-power modes, RTC, and sensor fusion — culminating in a real-time, RTOS-based sensor-to-actuator control application.

Where the companion `STM32nucleo_basics` and `Driver_development` repositories work at the bare-metal register level, this repository builds on STM32CubeMX-generated initialization code and the STM32 HAL/CMSIS-RTOS layer, closer to how peripherals are configured in production firmware.

## Topics covered

| Area | Folders | What it demonstrates |
|---|---|---|
| UART | `001UART_Hello_World`, `002UART_Tx`, `003UART_Tx_IT` | Basic UART transmission, polling vs. interrupt-driven transmit |
| Clock configuration | `004HSE_Sysclk`, `005PLL_Sysclk_HSI`, `006PLL_Sysclk_HSE` | Switching the system clock source between HSE and PLL, with PLL fed from both HSI and HSE |
| Timers | `007Timer_base_100ms`, `008Timer_baseIT_100ms`, `009Timer_base_10micro`, `009Timer_baseIT_10micro`, `010Timer_IC`, `011Timer_OC`, `012Timer_PWM`, `013Timer_PWM_led` | Timer base generation (polled and interrupt-driven) at millisecond and microsecond resolution, input capture, output compare, and PWM generation driving an LED |
| CAN bus | `014CAN_Loopback`, `015CAN_LoopbackIT`, `016CAN_Normal_IT` | CAN peripheral configuration in loopback mode (polled and interrupt-driven) and normal-mode interrupt-driven communication |
| Low-power modes | `017Sleep_on_exit`, `018WFI`, `019WFE` | Cortex-M Sleep-on-exit, Wait-For-Interrupt, and Wait-For-Event low-power entry mechanisms |
| Standby / backup domain | `020BackupSRAM_Standby` | Retaining data in Backup SRAM across Standby mode |
| Real-time clock | `021RTC_DateandTime`, `022RTC_Alarm`, `023RTC_Alarm_12h`, `024RTC_Alarm_Standbywakeup` | RTC date/time keeping, alarm configuration in 24h and 12h formats, and waking the MCU from Standby via an RTC alarm |
| LED / interrupt basics | `025LED_Blinker`, `026LED_Blinker_Interrupt` | Polled vs. EXTI-interrupt-driven LED blinking |
| Sensor fusion project | `027MPU6050_PWM_LED` | Capstone project (below) |
| Reference | `UART_Hello_World` | Additional UART reference example |

### Highlight: `027MPU6050_PWM_LED` — tilt-driven PWM control

The capstone project of this repository is a small real-time application built on **FreeRTOS (CMSIS-RTOS v2)**:

- Reads accelerometer data from an **MPU6050** IMU over I2C, triggered by a data-ready interrupt on `PA8`
- Computes roll angle from the accelerometer's Y/Z axes (`atan2f`) and smooths it with a **Kalman filter**
- Runs the workload across three RTOS tasks:
  - `sensor_read_task` — waits on a semaphore released by the MPU6050 interrupt, reads and filters the sensor data, and pushes the result to a message queue
  - `pwm_control_task` — consumes the filtered angle from the queue and maps it to a PWM duty cycle
  - `defaultTask` — idle housekeeping task
- Drives an LED's brightness via **TIM2 PWM** in proportion to the measured tilt angle
- Uses inter-task synchronization primitives throughout: an `osSemaphoreId_t` for the interrupt handoff and an `osMessageQueueId_t` for passing filtered angle data between tasks
- Includes a DWT cycle counter (`init_cycle_counter`) for timing instrumentation

This project ties together I2C sensor drivers, interrupt-driven data acquisition, digital filtering, RTOS task/queue/semaphore management, and PWM output into a single closed-loop control application.

## Hardware

- STM32F4-series development board (Discovery/Nucleo)
- MPU6050 6-axis accelerometer/gyroscope module (I2C) for the capstone project
- CAN transceiver for the CAN bus exercises (loopback mode requires no external transceiver)
- LED(s) for PWM/blinker exercises

## Toolchain

- STM32CubeIDE with STM32CubeMX-generated peripheral initialization (`MX_*_Init` functions)
- STM32 HAL drivers
- CMSIS-RTOS v2 / FreeRTOS (used in `027MPU6050_PWM_LED`)
- ST-Link for flashing and debugging

## Building and running

1. Open STM32CubeIDE and import the desired numbered folder as an existing project.
2. Build the project (`Project > Build`).
3. Flash to the target board via the onboard ST-Link.
4. For UART exercises, open a serial terminal at the configured baud rate (115200 by default).
5. For `027MPU6050_PWM_LED`, wire an MPU6050 to the I2C1 pins and its interrupt line to `PA8` before flashing.

## Status

Personal learning repository, progressing from single-peripheral exercises to a complete RTOS-based sensor control application.
