# Code Readme
The code folder consists of the three files used to run our crawler:
- quest6.c
- demo.js
- demo.html

'quest6.c' utilizes many libraries and drivers, particularly i2c (LIDARLite, Alphanumeric display), gpio (LEDs), timer (PID control), mcpwm (steering and ESC control), uart (rx), ADC (ir range finder sensors), and pcnt (optical detector). 'quest6.c' uses UDP sockets to communicate with the node server over port 8082. 'demo.js' utilizes socket.io to communicate with the client side html.

How to run locally:
1. Start node server (after ssh into Raspberry Pi):
node demo.js

2. Build ESP .c code using espressif build and make files, and then flash the code to the ESP32.

3. Open localhost:4000 for the demo.html view

Finally, testing code we used during our development process is included in folder "other_test_code".