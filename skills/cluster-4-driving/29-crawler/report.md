#  Skill Name

Author: Vindhya Kuchibhotla, Jennifer Norell, Vanessa Schuweh, 2019-11-04

## Summary
In this skill, we calibrated the crawler by first wiring the ESC to the ESP using pin A5 for PWM and using the ESC calibration code provided. The PWM is first set to the maximum value and the crawler moves forward, then the PWM is set to the minimum value and the crawler moves backwards, and finally the PWM is set to the middle or neutral value and the crawler stops.

Once the calibration was working, we wired the ESP to the steering servo using the H-bridge to enable high current draw from the battery. We used the H-bridge pin reference and initialized the GPIO pin 21 for steering servo PWM. Using the same code used the calibration, the crawler instead moved to the right, then the left, and finally straight.

In our implementation, steering and calibration happen simultaneously. So the crawler first moves forwards and to the right, then backwards and to the left, and then finally straightens out and stops.

## Sketches and Photos
[![Crawler Video](https://img.youtube.com/vi/O8Ak2gdg2VA/0.jpg)](https://www.youtube.com/watch?v=O8Ak2gdg2VA&feature=youtu.be)

## Modules, Tools, Source Used in Solution
- [Motor Control with PWM Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/mcpwm.html)
- ESC Design Pattern and calibration code
- [ESP-IDF Example Code](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/mcpwm/mcpwm_servo_control)

## Supporting Artifacts


-----

## Reminders
- Repo is private
