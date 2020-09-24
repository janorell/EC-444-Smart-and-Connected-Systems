# Sensors Central
Authors: Vindhya Kuchibhotla, Jennifer Norell, Vanessa Schuweh

2019-10-08

## Summary
In this quest, we used skills obtained over the past two weeks to wire a Thermistor, Ultrasonic sensor, and an IR Rangefinder, extract the sensor readings via the serial port module in Node.JS, and display these values on three plots vs. time on a web browser using Canvas.JS. Each plot was dedicated to a sensor for better readability. 

## Evaluation Criteria
We successfully demonstrated:
- Real time reporting of ultrasonic range in meters, of IR sensor in meters, and of temperature in Celsius.
- Printing these readings in a JSON format and saving this data into a CSV file.
- Reading and extracting data from this CSV file to graph the readings at every update.
- Graphing of these three readings continuously (every 5 seconds) on a dynamic scatter plot.
- Connecting the ESP to the web server and displaying the data coming from it.


## Solution Design

sensor.c performs the ADC operations to convert the analog readings of all three sensors into digital readings in corresponding engineering units. The printf statement in this file exports the readings in a JSON format in the following: "{\"temp\":\"%f\",\"ir\":\"%f\",\"ultrasonic\":\"%f\"}\n"

In dynamic-serial-plot.js, we set up the server and require the serial port module to read the serial input from the ESP32. We then write this data to a file, log.txt in the CSV format. The data is then extracted from the CSV and is stored in a variable data that is passed to the html via the Socket IO module. 

In dynamic-index.html, the data passed using socket IO from the server side is parsed once again (row by row) and pushed to an array of data points for graphing purposes. This file is responsible for parsing, graphing, and updating the graph based on the socket IO event using required skills from Canvas.JS and Node.JS. We struggled with using the dynamic line chart from CanvasJS and instead refresh the page every 5 seconds when another data point is read in from the serial port.


## Investigative Question

Speed
The thermal time constant is the time required for a thermistor to respond to a change in its ambient temperature. The sensor will sample at this speed. This time is less than 7s.  Regarding the IR Range Sensor it can be measured as quickly as 38 ms (plus or minus 10ms). The speed of this the ultrasonic sensor can be measured as quickly as 100ms per cycle.

Resolution
The best resolution at the ultrasonic measures is 1mm. Resolution can be found by using the 3.3 V value, multiplying it by the number of steps used by the board. The number of steps is 4096 because its a 12 bit board. After we have these values, we get the sensor range and divide it by the number of steps. This is the resolution.
The thermistor does not give us the voltage range so we are unable to calculate this. The voltage range of the IR sensor is 4.5V to 5.5V and the voltage range of the ultrasonic sensor is 2.5V to 5.5V. Therefore, the resolution of the IR sensor is .0011 to .0013 and the resolution of the ultrasonic sensor is .0006 to .0013.


## Sketches and Photos
Video Demonstration:

[![Alt text](https://img.youtube.com/vi/Rot8pgcuBcU/0.jpg)](https://www.youtube.com/watch?v=Rot8pgcuBcU&feature=youtu.be)

Video Presentation:

[![Alt text](https://img.youtube.com/vi/gNg-w3J8EsA/0.jpg)](https://www.youtube.com/watch?v=gNg-w3J8EsA&feature=youtu.be)

## Supporting Artifacts
- [Link to code repos](https://github.com/BU-EC444/Team7-Schuweh-Kuchibhotla-Norell/tree/master/quest-2/code)


## References
- [Analog to Digital Converter](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html)
- [Node.JS File System Module](https://www.w3schools.com/nodejs/nodejs_filesystem.asp)
- [Intro to IoT with Arduino, Node.js, and Chart.js](https://blog.priyanshrastogi.com/introduction-to-internet-of-things-with-arduino-node-js-and-chart-js-cb4885d176c4)
- [Resolution]( https://www.eaa.net.au/PDF/Hitech/MF52type.pdf)
-----

## Reminders

- Video recording in landscape not to exceed 90s
- Each team member appears in video
- Make sure video permission is set accessible to the instructors
- Repo is private
