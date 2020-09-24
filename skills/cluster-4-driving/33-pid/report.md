#  PID Control Using the Micro

Author: Vindhya Kuchibhotla, Jennifer Norell, Vanessa Schuweh, 2019-11-08

## Summary
In this skill, we used PID to monitor the distance from a setpoint distance of 40 cm using the LIDARLite range sensor. If the output threshold is less than zero, the red LED is on. If the output threshold is equal to zero, the green LED is on. If the output threshold is greater than zero, the blue LED is on. The video demonstrates the feedback when we move above or below the setpoint by printing the output from the calculation and by setting the LED. We used a timing loop with Timer Group 0 and initialized this for interrupts.  

The K defines were established based on trial and error.
- Kp = 0.75
- Ki = 0.5
- Kd = 0.0

## Sketches and Photos
Video of PID Demonstration:

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/TmoAgk6Eqdo/0.jpg)](https://www.youtube.com/watch?v=TmoAgk6Eqdo&feature=youtu.be)

## Modules, Tools, Source Used in Solution
- I2C and LIDARLite skill
- Timers
- FreeRTOS

## Supporting Artifacts
- [PID Control Design Pattern](http://whizzer.bu.edu/briefs/design-patterns/dp-pid)
- [Espress-if Documentation for Timers](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/timer.html)
- [Code Example on Timers](https://github.com/BU-EC444/code-examples/blob/master/timer-example/main/timer-example.c)

-----

## Reminders
- Repo is private
